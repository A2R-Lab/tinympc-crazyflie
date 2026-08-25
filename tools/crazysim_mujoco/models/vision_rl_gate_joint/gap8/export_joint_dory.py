#!/usr/bin/env python3
"""Export a single-output DORY candidate and a sealed 200-clip parity corpus."""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
import onnx
import torch

from .deploy_model import JointDoryPackedNet, OUTPUT_LAYOUT, write_contract
from ..model import JointTemporalPolicy


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1 << 20), b""):
            digest.update(block)
    return digest.hexdigest()


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--checkpoint", type=Path, required=True)
    parser.add_argument("--calibration-source", type=Path, required=True,
                        help="Training-excluded test.npz used for affine/PTQ calibration; must have uint8 frames.")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--clips", type=int, default=200)
    parser.add_argument("--seed", type=int, default=5913)
    parser.add_argument("--terminal-scale", type=float, default=1.0,
                        help="Shared positive power-of-two terminal encoding scale.")
    parser.add_argument("--terminal-margin", type=float, default=0.25,
                        help="Positive terminal offset margin, inverted by the deployment decoder.")
    args = parser.parse_args()
    if args.clips != 200:
        raise ValueError("the deployment parity corpus is intentionally fixed at 200 clips")
    args.output.mkdir(parents=True, exist_ok=True)
    state = torch.load(args.checkpoint, map_location="cpu", weights_only=False)
    model = JointDoryPackedNet(int(state["width"])).eval()
    model.load_joint_checkpoint(args.checkpoint)
    np.savez_compressed(args.output / "nemo_bridge_state.npz",
                        width=np.asarray(int(state["width"]), dtype=np.int64),
                        **{key: value.detach().cpu().numpy() for key, value in state["model"].items()})
    with np.load(args.calibration_source, allow_pickle=False) as archive:
        frames = np.asarray(archive["frames"])
        if frames.dtype != np.uint8 or frames.ndim != 4 or frames.shape[1:] != (2, 160, 160):
            raise RuntimeError("calibration frames must be uint8[N,2,160,160]")
        if len(frames) < args.clips:
            raise RuntimeError("calibration source has fewer than 200 clips")
        indices = np.sort(np.random.default_rng(args.seed).choice(len(frames), args.clips, replace=False))
        selected = frames[indices].copy()
        labels = {key: np.asarray(archive[key])[indices].copy()
                  for key in ("action", "gate_corners", "gate_visible") if key in archive}
    model.set_terminal_affine_from_logits(torch.from_numpy(selected), scale=args.terminal_scale,
                                          margin=args.terminal_margin)
    model.eval()
    with torch.no_grad():
        encoded = model(torch.from_numpy(selected).float() / 255.0)
        logits = model.decode(encoded)
        source_model = JointTemporalPolicy(int(state["width"])).eval()
        source_model.load_state_dict(state["model"])
        source_features = source_model.features(torch.from_numpy(selected).float() / 255.0)
        source_logits = torch.cat((source_model.actor(source_features),
                                   source_model.gate_head(source_features)), 1)
        reexpression_max_abs_error = float((logits - source_logits).abs().max())
    if reexpression_max_abs_error > 2.0e-5:
        raise RuntimeError("Conv-only re-expression drift %.9g exceeds 2e-5" %
                           reexpression_max_abs_error)
    onnx_path = args.output / "joint_dory_packed_float.onnx"
    torch.onnx.export(model, torch.zeros(1, 2, 160, 160), onnx_path,
                      input_names=["frames"], output_names=["packed_encoded"],
                      opset_version=13, dynamo=False)
    graph = onnx.load(onnx_path)
    onnx.checker.check_model(graph)
    operators = sorted({node.op_type for node in graph.graph.node})
    unsupported = sorted(set(operators) - {"Conv", "Relu", "AveragePool", "Identity", "Constant"})
    if unsupported:
        raise RuntimeError("DORY export has unsupported ops: %s" % unsupported)
    # These clips fit the terminal affine and NeMO activation statistics, so
    # they are a calibration/parity corpus—not an independent held-out set.
    np.savez_compressed(args.output / "calibration_200_clips.npz", frames=selected,
                        source_indices=indices.astype(np.int64), encoded_float=encoded.cpu().numpy(),
                        reference_logits=logits.cpu().numpy(), **labels)
    provenance = {"source_checkpoint": str(args.checkpoint.resolve()),
                  "source_checkpoint_sha256": sha256(args.checkpoint),
                  "calibration_source": str(args.calibration_source.resolve()),
                  "calibration_source_sha256": sha256(args.calibration_source),
                  "selection": "numpy.default_rng(seed).choice(N, 200, replace=False), sorted",
                  "seed": args.seed, "clips": args.clips,
                  "terminal_scale": args.terminal_scale, "terminal_margin": args.terminal_margin,
                  "float_reexpression_max_abs_logit_error": reexpression_max_abs_error,
                  "onnx_sha256": sha256(onnx_path), "operators": operators,
                  "nemo_bridge_state_sha256": sha256(args.output / "nemo_bridge_state.npz"),
                  "output_layout": OUTPUT_LAYOUT}
    write_contract(args.output / "deployment_contract.json", model, provenance)
    (args.output / "export_report.json").write_text(json.dumps(provenance, indent=2, sort_keys=True) + "\n")
    print(json.dumps(provenance, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
