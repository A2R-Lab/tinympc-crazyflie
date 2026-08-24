#!/usr/bin/env python3
"""Offline evaluation of a procedural two-frame vision policy."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
from typing import Any, Iterable

import numpy as np

from .model import TemporalPolicy
from .validate_procedural_dataset import (
    DatasetValidationError, _jsonable, file_sha256, load_shards,
)


def _safe_ratio(numerator: int, denominator: int) -> float:
    # Treat an unpredicted or unsupported class as zero, as sklearn's
    # zero_division=0 does.  Silently omitting it would reward class collapse.
    return numerator / denominator if denominator else 0.0


def classification_metrics(expected: np.ndarray, predicted: np.ndarray) -> dict[str, Any]:
    matrix = np.zeros((3, 3), dtype=np.int64)
    np.add.at(matrix, (expected, predicted), 1)
    precision: list[float] = []
    recall: list[float] = []
    f1: list[float] = []
    for action in range(3):
        p = _safe_ratio(int(matrix[action, action]), int(matrix[:, action].sum()))
        r = _safe_ratio(int(matrix[action, action]), int(matrix[action].sum()))
        precision.append(p)
        recall.append(r)
        f1.append(2.0 * p * r / (p + r) if p + r > 0.0 else 0.0)
    return {
        "samples": int(len(expected)), "confusion_expected_rows_predicted_columns": matrix,
        "accuracy": float(np.mean(expected == predicted)) if len(expected) else None,
        "precision_track_left_right": precision,
        "recall_track_left_right": recall,
        "f1_track_left_right": f1,
        "balanced_accuracy": float(np.mean(recall)) if len(expected) else None,
        "macro_f1": float(np.mean(f1)) if len(expected) else None,
        "expected_action_counts": np.bincount(expected, minlength=3),
        "predicted_action_counts": np.bincount(predicted, minlength=3),
    }


def _run_onnx(session, input_name: str, frames: np.ndarray,
              batch_size: int = 128) -> np.ndarray:
    outputs = []
    shape = session.get_inputs()[0].shape
    fixed_one = shape[0] == 1
    step = 1 if fixed_one else batch_size
    for start in range(0, len(frames), step):
        batch = frames[start:start + step].astype(np.float32) / 255.0
        output = np.asarray(session.run(None, {input_name: batch})[0])
        if output.ndim != 2 or output.shape != (len(batch), 3):
            raise DatasetValidationError(
                f"ONNX output must be [N,3], received {output.shape}")
        outputs.append(output)
    return np.concatenate(outputs, axis=0)


def _checkpoint_policy(path: Path):
    import torch
    try:
        checkpoint = torch.load(path, map_location="cpu", weights_only=True)
    except TypeError:
        checkpoint = torch.load(path, map_location="cpu")
    latent_dim = int(checkpoint.get("latent_dim", 7))
    policy = TemporalPolicy(latent_dim)
    policy.load_state_dict(checkpoint["policy"])
    return policy.eval()


def evaluate_policy(
    manifest_paths: Iterable[Path], model_path: Path, *,
    splits: set[str] | None = None, checkpoint_path: Path | None = None,
    max_samples: int | None = None, parity_samples: int = 64,
) -> dict[str, Any]:
    """Evaluate ONNX actions, symmetry, and optional PyTorch parity."""
    try:
        import onnxruntime as ort
    except ImportError as error:
        raise DatasetValidationError("onnxruntime is required") from error
    selected_splits = splits or {"validation", "test"}
    shards, _ = load_shards(manifest_paths)
    selected = [item for item in shards if item.split in selected_splits]
    if not selected:
        raise DatasetValidationError(
            f"no shards for requested splits {sorted(selected_splits)}")
    required = {"frames", "expert_action", "behavior_action", "decision_mask",
                "hard_track_mask", "clearance"}
    for shard in selected:
        missing = required.difference(shard.arrays)
        if missing:
            raise DatasetValidationError(f"{shard.path}: missing arrays {sorted(missing)}")
    frames = np.concatenate([item.arrays["frames"] for item in selected])
    expert = np.concatenate([item.arrays["expert_action"] for item in selected]).astype(int)
    behavior = np.concatenate([item.arrays["behavior_action"] for item in selected]).astype(int)
    decision = np.concatenate([item.arrays["decision_mask"] for item in selected]).astype(bool)
    hard_track = np.concatenate([item.arrays["hard_track_mask"] for item in selected]).astype(bool)
    clearance = np.concatenate([item.arrays["clearance"] for item in selected]).astype(float)
    if max_samples is not None and len(frames) > max_samples:
        # Deterministic, full-range subsampling avoids a prefix/course-position bias.
        index = np.linspace(0, len(frames) - 1, max_samples, dtype=int)
        frames, expert, behavior, decision, hard_track, clearance = (
            value[index] for value in
            (frames, expert, behavior, decision, hard_track, clearance))
    model_path = model_path.resolve()
    session = ort.InferenceSession(str(model_path), providers=["CPUExecutionProvider"])
    inputs = session.get_inputs()
    outputs = session.get_outputs()
    if len(inputs) != 1 or len(outputs) != 1:
        raise DatasetValidationError("ONNX must have exactly one input and one output")
    shape = inputs[0].shape
    contract_ok = (len(shape) == 4 and shape[1:] == [2, 160, 160] and
                   outputs[0].shape[-1] == 3 and inputs[0].type == "tensor(float)")
    logits = _run_onnx(session, inputs[0].name, frames)
    if not np.all(np.isfinite(logits)):
        raise DatasetValidationError("ONNX produced non-finite logits")
    predicted = np.argmax(logits, axis=1)
    all_metrics = classification_metrics(expert, predicted)
    decision_metrics = classification_metrics(expert[decision], predicted[decision])
    hard_track_metrics = classification_metrics(expert[hard_track], predicted[hard_track])
    danger = clearance < 0.4
    danger_metrics = classification_metrics(expert[danger], predicted[danger])

    mirrored_frames = frames[..., ::-1].copy()
    mirrored_logits = _run_onnx(session, inputs[0].name, mirrored_frames)
    swap = np.asarray([0, 2, 1])
    expected_mirrored_logits = logits[:, swap]
    mirrored_prediction = np.argmax(mirrored_logits, axis=1)
    mirrored_expert = swap[expert]
    mirrored_equivariance = float(np.mean(mirrored_prediction == swap[predicted]))
    mirrored_expert_accuracy = float(np.mean(mirrored_prediction == mirrored_expert))
    mirrored_logit_mae = float(np.mean(np.abs(
        mirrored_logits - expected_mirrored_logits)))

    parity: dict[str, Any] | None = None
    if checkpoint_path is not None:
        import torch
        count = min(parity_samples, len(frames))
        index = np.linspace(0, len(frames) - 1, count, dtype=int)
        tensor = torch.from_numpy(frames[index].astype(np.float32) / 255.0)
        with torch.no_grad():
            torch_logits = _checkpoint_policy(checkpoint_path)(tensor).numpy()
        onnx_logits = logits[index]
        difference = np.abs(torch_logits - onnx_logits)
        parity = {
            "samples": count, "maximum_absolute_logit_error": float(difference.max()),
            "mean_absolute_logit_error": float(difference.mean()),
            "action_agreement": float(np.mean(
                np.argmax(torch_logits, axis=1) == np.argmax(onnx_logits, axis=1))),
            "checkpoint_sha256": file_sha256(checkpoint_path),
        }

    predicted_counts = np.bincount(predicted, minlength=3)
    predicted_fractions = predicted_counts / max(1, len(predicted))
    decision_direction_recall = decision_metrics["recall_track_left_right"][1:]
    direction_recall_ok = all(value is not None and value >= 0.60
                              for value in decision_direction_recall)
    checks = [
        {"name": "onnx_contract", "passed": contract_ok,
         "actual": {"input_shape": shape, "input_type": inputs[0].type,
                    "output_shape": outputs[0].shape},
         "required": "float [N,2,160,160] -> [N,3]"},
        {"name": "finite_logits", "passed": True, "actual": True, "required": True},
        {"name": "decision_balanced_accuracy",
         "passed": decision_metrics["balanced_accuracy"] is not None and
                   decision_metrics["balanced_accuracy"] >= 0.60,
         "actual": decision_metrics["balanced_accuracy"], "required": ">=0.60"},
        {"name": "decision_left_right_recall", "passed": direction_recall_ok,
         "actual": decision_direction_recall, "required": "each >=0.60"},
        {"name": "safe_side_accuracy", "passed": decision_metrics["accuracy"] is not None
                   and decision_metrics["accuracy"] >= 0.70,
         "actual": decision_metrics["accuracy"], "required": ">=0.70"},
        {"name": "hard_track_accuracy", "passed": hard_track_metrics["accuracy"] is not None
                   and hard_track_metrics["accuracy"] >= 0.90,
         "actual": hard_track_metrics["accuracy"], "required": ">=0.90"},
        {"name": "no_action_collapse", "passed": bool(np.all(predicted_counts > 0)),
         "actual": predicted_counts, "required": "all actions exercised"},
        {"name": "mirrored_side_equivariance", "passed": mirrored_equivariance >= 0.70,
         "actual": mirrored_equivariance, "required": ">=0.70"},
        {"name": "mirrored_expert_accuracy", "passed": mirrored_expert_accuracy >= 0.70,
         "actual": mirrored_expert_accuracy, "required": ">=0.70"},
    ]
    if parity is not None:
        checks.append({"name": "pytorch_onnx_parity",
                       "passed": parity["maximum_absolute_logit_error"] <= 1.0e-5,
                       "actual": parity["maximum_absolute_logit_error"],
                       "required": "<=1e-5"})
    failed = [item["name"] for item in checks if not item["passed"]]
    return {
        "format": "tinympc-procedural-policy-offline-evaluation-v1",
        "accepted": not failed, "model": str(model_path),
        "model_sha256": file_sha256(model_path),
        "manifest_paths": [str(Path(path).resolve()) for path in manifest_paths],
        "splits": sorted(selected_splits), "samples": len(frames),
        "action_contract": ["TRACK", "LEFT", "RIGHT"],
        "all": all_metrics, "decision": decision_metrics,
        "hard_track": hard_track_metrics, "danger_clearance_lt_0p4": danger_metrics,
        "behavior_against_expert": classification_metrics(expert, behavior),
        "predicted_action_fractions": predicted_fractions,
        "mirrored": {
            "side_equivariance": mirrored_equivariance,
            "expert_accuracy": mirrored_expert_accuracy,
            "logit_mean_absolute_error_after_left_right_swap": mirrored_logit_mae,
        },
        "pytorch_onnx_parity": parity, "checks": checks, "failed_checks": failed,
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--manifest", type=Path, action="append", required=True)
    parser.add_argument("--model", type=Path, required=True)
    parser.add_argument("--checkpoint", type=Path)
    parser.add_argument("--split", action="append", choices=("train", "validation", "test"))
    parser.add_argument("--max-samples", type=int)
    parser.add_argument("--parity-samples", type=int, default=64)
    parser.add_argument("--out", type=Path)
    args = parser.parse_args()
    try:
        report = evaluate_policy(
            args.manifest, args.model, splits=set(args.split) if args.split else None,
            checkpoint_path=args.checkpoint, max_samples=args.max_samples,
            parity_samples=args.parity_samples)
    except (DatasetValidationError, OSError, json.JSONDecodeError) as error:
        report = {"format": "tinympc-procedural-policy-offline-evaluation-v1",
                  "accepted": False, "fatal_error": str(error)}
    payload = json.dumps(report, indent=2, allow_nan=False, default=_jsonable) + "\n"
    if args.out:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        args.out.write_text(payload)
    print(payload, end="")
    return 0 if report.get("accepted") else 1


if __name__ == "__main__":
    raise SystemExit(main())
