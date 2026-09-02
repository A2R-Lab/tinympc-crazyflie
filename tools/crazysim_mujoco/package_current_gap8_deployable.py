#!/usr/bin/env python3
"""Package the promoted task-branched GAP8 model for CrazySim replay."""

from __future__ import annotations

import argparse
import hashlib
import json
import shutil
import sys
from pathlib import Path

import numpy as np
import torch


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def load_partition(module: torch.nn.Module, archive: Path) -> None:
    values = np.load(archive)
    module.load_state_dict({key: torch.from_numpy(values[key]) for key in values.files})
    module.eval()


def package_close_rail_v9(candidate: Path, output: Path, promotion: dict) -> int:
    """Package the official exact-INT8 v8-collision/v9-rail selection."""
    base = candidate / "base_v8"
    recovery = candidate / "gate_recovery_v9"
    sources = {
        "encoder": base / "integer_symw8_gw7/encoder/encoder_int.onnx",
        "global_head": base / "integer_symw8_gw7/global_head/global_head_int.onnx",
        "collision_nemo_report": base / "integer_symw8_gw7/nemo_report.json",
        "gate_encoder": recovery / "head_only_integer/gate_encoder/gate_encoder_int.onnx",
        "gate_head": recovery / "head_only_integer/gate_head/gate_head_int.onnx",
        "gate_nemo_report": recovery / "head_only_integer/nemo_report.json",
        "gate_bridge_manifest": recovery / "head_only_bridge/manifest.json",
        "promotion_manifest": candidate / "promotion_manifest.json",
    }
    destination_names = {
        "encoder": "encoder_int.onnx",
        "global_head": "global_head_int.onnx",
        "collision_nemo_report": "collision_nemo_report.json",
        "gate_encoder": "gate_encoder_int.onnx",
        "gate_head": "gate_head_int.onnx",
        "gate_nemo_report": "gate_nemo_report.json",
        "gate_bridge_manifest": "gate_manifest.json",
        "promotion_manifest": "promotion_manifest.json",
    }
    for name, source in sources.items():
        expected = promotion.get("artifacts", {}).get(
            str(source.relative_to(candidate)), {}).get("sha256")
        if expected is not None and sha256(source) != expected:
            raise ValueError(f"promotion checksum mismatch: {source}")
        shutil.copy2(source, output / destination_names[name])
    artifacts = {
        name: {
            "path": destination_names[name],
            "sha256": sha256(output / destination_names[name]),
        }
        for name in sources
    }
    bundle = {
        "format": "tinympc-espnetv2-gap8-close-rail-v9",
        "runtime_adapter": "espnet_v7_reactive_gap8",
        "source_candidate": str(candidate),
        "student_architecture": "v8_collision_v9_close_rail_recovery",
        "input": {
            "dtype": "uint8", "layout": "CHW", "shape": [3, 160, 160],
            "channels": ["previous_gray", "current_gray", "signed_frame_difference"],
            "difference_formula": "(current-previous+255)//2",
        },
        "partition_execution": {
            "encoder": "exact_int8_onnx", "global_head": "exact_int8_onnx",
            "gate_encoder": "exact_int8_onnx", "gate_head": "exact_int8_onnx",
        },
        "rail_recovery": {
            "sector_order": ["left", "center", "right"],
            "output_order": [
                "rail_present_left", "rail_present_center", "rail_present_right",
                "pass_right_left", "pass_right_center", "pass_right_right",
            ],
            "rail_present_threshold": 0.45,
            "pass_right_threshold": 0.5,
            "selection": "highest_collision_risk_sector",
            "activation": "after_brake_near_hover_two_frame_confirmation",
        },
        "navigation_mapping": {
            "image_right": "negative_yaw_rate",
            "image_left": "positive_yaw_rate",
            "maneuver_owner": "firmware_route_free_reactive",
        },
        "artifacts": artifacts,
        "limitations": {
            "physical_gap8_evaluated": False,
            "closed_loop_crazysim_qualified": False,
        },
    }
    (output / "bundle.json").write_text(json.dumps(bundle, indent=2) + "\n")
    print(json.dumps(bundle, indent=2))
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--candidate", type=Path, required=True)
    parser.add_argument("--drone-rl-root", type=Path, default=Path("/home/cchen/drone_rl"))
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    candidate = args.candidate.resolve()
    output = args.output.resolve()
    promotion = json.loads((candidate / "promotion_manifest.json").read_text())
    architecture = promotion.get("architecture")
    if output.exists() and any(output.iterdir()):
        raise FileExistsError(f"refusing to overwrite non-empty bundle: {output}")
    output.mkdir(parents=True, exist_ok=True)
    if architecture == "v8_collision_v9_close_rail_recovery":
        return package_close_rail_v9(candidate, output, promotion)
    bridge = json.loads((candidate / "bridge/manifest.json").read_text())
    if promotion.get("architecture") != "v8_task_branched":
        raise ValueError("candidate is not the promoted v8 task-branched model")
    if bridge.get("partitions") != [
        "encoder", "global_head", "gate_encoder", "gate_head", "corner_head"
    ]:
        raise ValueError("unexpected task-branched bridge partition contract")

    sys.path.insert(0, str(args.drone_rl_root.resolve()))
    from deployment.espnetv2_gap8_student import (  # pylint: disable=import-outside-toplevel
        ESPNetV2Gap8StudentV8TaskBranched,
    )

    model = ESPNetV2Gap8StudentV8TaskBranched()
    gate_encoder = model.gate_encoder()
    gate_head = model.gate_head
    corner_head = model.corner_head
    load_partition(gate_encoder, candidate / "bridge/gate_encoder_state.npz")
    load_partition(gate_head, candidate / "bridge/gate_head_state.npz")
    load_partition(corner_head, candidate / "bridge/corner_head_state.npz")
    exports = {
        "gate_encoder": (gate_encoder, torch.zeros(1, 3, 160, 160)),
        "gate_head": (gate_head, torch.zeros(1, 32, 40, 40)),
        "corner_head": (corner_head, torch.zeros(1, 32, 40, 40)),
    }
    for name, (module, example) in exports.items():
        torch.onnx.export(
            module, example, output / f"{name}_float.onnx",
            input_names=["input"], output_names=["output"], opset_version=17,
        )

    sources = {
        "encoder": candidate / "integer_symw8_gw7/encoder/encoder_int.onnx",
        "global_head": candidate / "integer_symw8_gw7/global_head/global_head_int.onnx",
        "nemo_report": candidate / "integer_symw8_gw7/nemo_report.json",
        "output_decode": candidate / "bridge/output_decode.npz",
        "bridge_manifest": candidate / "bridge/manifest.json",
        "promotion_manifest": candidate / "promotion_manifest.json",
    }
    for source in sources.values():
        expected = promotion.get("artifacts", {}).get(str(source.relative_to(candidate)), {}).get("sha256")
        if expected is not None and sha256(source) != expected:
            raise ValueError(f"promotion checksum mismatch: {source}")
        shutil.copy2(source, output / source.name)

    artifact_paths = {
        "encoder": output / "encoder_int.onnx",
        "global_head": output / "global_head_int.onnx",
        "gate_encoder": output / "gate_encoder_float.onnx",
        "gate_head": output / "gate_head_float.onnx",
        "corner_head": output / "corner_head_float.onnx",
        "nemo_report": output / "nemo_report.json",
        "output_decode": output / "output_decode.npz",
        "bridge_manifest": output / "manifest.json",
        "promotion_manifest": output / "promotion_manifest.json",
    }
    artifacts = {
        name: {"path": path.name, "sha256": sha256(path)}
        for name, path in artifact_paths.items()
    }
    bundle = {
        "format": "tinympc-espnetv2-gap8-task-branched-v1",
        "runtime_adapter": "espnet_v7_reactive_gap8",
        "source_pointer": str(args.candidate.absolute()),
        "source_candidate": str(candidate),
        "student_architecture": "v8_task_branched",
        "input": {
            "dtype": "uint8", "layout": "CHW", "shape": [3, 160, 160],
            "channels": ["previous_gray", "current_gray", "signed_frame_difference"],
            "difference_formula": "(current-previous+255)//2",
        },
        "partition_execution": {
            "encoder": "exact_int8_onnx", "global_head": "exact_int8_onnx",
            "gate_encoder": "float32_onnx", "gate_head": "float32_onnx",
            "corner_head": "float32_onnx",
        },
        "gate_geometry": {
            "corner_order": ["left_top", "right_top", "left_bottom", "right_bottom"],
            "visibility_order": ["left_rail", "right_rail"],
            "visibility_thresholds": [0.5, 0.625], "heatmap_shape": [20, 20],
        },
        "navigation_mapping": {
            "model_collision_head_order": "left_center_right",
            "firmware_collision_head_order": "left_center_right",
            "probability_filter": "causal_median3",
            "maneuver_owner": "firmware_route_free_reactive",
        },
        "artifacts": artifacts,
        "limitations": {
            "gate_partitions_quantized": False,
            "gate_partitions_gap8_qualified": False,
            "physical_gap8_evaluated": False,
        },
    }
    (output / "bundle.json").write_text(json.dumps(bundle, indent=2) + "\n")
    print(json.dumps(bundle, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
