#!/usr/bin/env python3
"""Export only the image encoder and actor from a training checkpoint."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

import torch

from .model import TemporalPolicy


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def export(checkpoint_path: Path, output_path: Path, bundle_path: Path,
           training_config: dict | None = None) -> None:
    checkpoint = torch.load(checkpoint_path, map_location="cpu", weights_only=False)
    policy = TemporalPolicy(latent_dim=int(checkpoint["latent_dim"]))
    policy.load_state_dict(checkpoint["policy"])
    policy.eval()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    torch.onnx.export(
        policy, torch.zeros(1, 2, 160, 160), output_path,
        input_names=["frames"], output_names=["action_logits"],
        opset_version=17, do_constant_folding=True, dynamo=False,
    )
    manifest = {
        "format": "tinympc-vision-rl-mpc-poc-v1",
        "deployment": {
            "input": {"name": "frames", "shape": [1, 2, 160, 160],
                      "dtype": "float32", "range": [0.0, 1.0],
                      "temporal_order": ["previous", "current"]},
            "output": {"name": "action_logits", "shape": [1, 3],
                       "actions": ["TRACK", "LEFT", "RIGHT"]},
            "packet_mapping": {
                "TRACK": {"steering": 0.0, "collision": 0.0},
                "LEFT": {"steering": 1.0, "collision": 1.0},
                "RIGHT": {"steering": -1.0, "collision": 1.0},
            },
        },
        "camera_provenance": {
            "repository": "tinympc-perception",
            "commit": "890bbd6923a9459d4d7ab7bee92542e4cb61c64d",
            "source_files": [
                "gap8_perception/configs/hm01b0_calibration.json",
                "gap8_perception/configs/isaac_render_calibration.json",
            ],
            "copied_assets": [],
            "hm01b0": {
                "resolution": [160, 160],
                "camera_matrix_px": [[89.1558392549, 0.0, 81.1038105230],
                                     [0.0, 89.4608171623, 73.3473030288],
                                     [0.0, 0.0, 1.0]],
                "distortion": [-0.0176448766, 0.0994132451, 0.0054432154,
                               -0.0060400120, -0.1900189875],
            },
            "mujoco_render": {
                "resolution": [160, 160],
                "vertical_fov_deg": 83.6091095,
                "distortion_applied": False,
            },
            "note": "Only numeric calibration was transcribed; no texture or asset was copied. MuJoCo applies the matched vertical FOV but not the recorded lens distortion.",
        },
        "algorithm": {
            "name": "privileged-latent short-horizon Dyna/MBPO actor-critic",
            "imagined_horizon_max": 5,
            "deployment_contains": ["image_encoder", "categorical_actor"],
            "deployment_excludes": ["privileged_latent", "dynamics_ensemble", "critic"],
        },
        "runtime_adapter": "hybrid_rl",
        "training": training_config or checkpoint.get("training_config", {}),
        "artifacts": {
            "checkpoint": {"path": checkpoint_path.name,
                           "sha256": sha256(checkpoint_path)},
            "policy_onnx": {"path": output_path.name, "sha256": sha256(output_path)},
        },
    }
    bundle_path.write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--checkpoint", required=True, type=Path)
    parser.add_argument("--onnx", required=True, type=Path)
    parser.add_argument("--bundle", required=True, type=Path)
    args = parser.parse_args()
    export(args.checkpoint, args.onnx, args.bundle)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
