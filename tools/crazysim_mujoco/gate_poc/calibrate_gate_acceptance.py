#!/usr/bin/env python3
"""Fit a small, deployment-compatible gate-acceptance calibrator.

This deliberately does *not* retrain the ESPNet perception backbone.  It fits
only a logistic confidence fusion on raw gate-head features already exported
from the selected IsaacSim evaluation corpus.  The three features mirror the
runtime quantities in ``EspnetDronetGateAdapter._decode_gate``: presence,
mask strength, and mean corner confidence.  The held-out test split is never
used for fitting or threshold selection.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import platform
import subprocess
import sys
from datetime import datetime, timezone
from pathlib import Path

import numpy as np
from sklearn.linear_model import LogisticRegression
from sklearn.metrics import (average_precision_score, precision_recall_curve,
                             roc_auc_score)


FEATURE_NAMES = ("presence_probability", "mask_top1pct",
                 "corner_peak_mean")
SOURCE_FEATURE_NAMES = ("presence_logit", "mask_top1pct",
                        "corner_peak_mean")
CURRENT = {
    "mean": [0.7985369851697791, 0.26617044903287485, 0.6138960815024689],
    "scale": [0.28441201539745553, 0.38346236824033086, 0.14942132396195934],
    "weight": [1.2141424246350632, 0.23188614391540915, 1.3052519850668753],
    "bias": 1.1140172563144006,
    "threshold": 0.6174671283211735,
}


def sigmoid(value: np.ndarray) -> np.ndarray:
    return 1.0 / (1.0 + np.exp(-np.clip(value, -60.0, 60.0)))


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def episode_id(source: str) -> str:
    """Use the retained shard or real-flight directory, never an image frame."""
    path = Path(source)
    parent = path.parent
    return str(parent)


def load_split(path: Path) -> dict[str, np.ndarray]:
    raw = np.load(path, allow_pickle=False)
    required = {"features", "labels", "domains", "sources"}
    if set(raw.files) != required:
        raise ValueError(f"{path} has keys {raw.files}; expected {sorted(required)}")
    features = np.asarray(raw["features"], dtype=np.float64)
    if features.ndim != 2 or features.shape[1] < 8:
        raise ValueError(f"{path} has unsupported feature shape {features.shape}")
    # Match the production adapter: it fuses probability(presence logit), a
    # high-mask score, and the mean of the four corner peaks.
    compatible = np.column_stack((sigmoid(features[:, 0]), features[:, 3],
                                  features[:, 7]))
    if not np.isfinite(compatible).all():
        raise ValueError(f"{path} contains non-finite compatible features")
    labels = np.asarray(raw["labels"], dtype=np.int8)
    if len(labels) != len(compatible) or not np.isin(labels, (0, 1)).all():
        raise ValueError(f"{path} labels are not binary/aligned")
    return {
        "x": compatible,
        "y": labels,
        "domains": np.asarray(raw["domains"]).astype(str),
        "sources": np.asarray(raw["sources"]).astype(str),
    }


def threshold_at_precision(y: np.ndarray, scores: np.ndarray,
                           minimum_precision: float) -> float:
    precision, recall, thresholds = precision_recall_curve(y, scores)
    # precision/recall contain a final point without a threshold.
    eligible = np.flatnonzero(precision[:-1] >= minimum_precision)
    if not len(eligible):
        return 1.0
    best = eligible[np.argmax(recall[eligible])]
    return float(thresholds[best])


def metrics(y: np.ndarray, scores: np.ndarray, threshold: float) -> dict[str, float | int]:
    predicted = scores >= threshold
    positive = y == 1
    negative = ~positive
    tp = int(np.sum(predicted & positive))
    fp = int(np.sum(predicted & negative))
    tn = int(np.sum(~predicted & negative))
    fn = int(np.sum(~predicted & positive))
    precision = tp / max(tp + fp, 1)
    recall = tp / max(tp + fn, 1)
    specificity = tn / max(tn + fp, 1)
    return {
        "samples": int(len(y)), "positive_count": int(positive.sum()),
        "auroc": float(roc_auc_score(y, scores)),
        "average_precision": float(average_precision_score(y, scores)),
        "threshold": float(threshold), "precision": precision,
        "recall": recall, "specificity": specificity,
        "balanced_accuracy": 0.5 * (recall + specificity),
        "f1": 2 * precision * recall / max(precision + recall, 1e-12),
        "tp": tp, "fp": fp, "tn": tn, "fn": fn,
    }


def run_git(root: Path, *args: str) -> str:
    try:
        return subprocess.check_output(["git", *args], cwd=root, text=True).strip()
    except (OSError, subprocess.CalledProcessError):
        return "unavailable"


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--features-dir", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--repo-root", type=Path, required=True)
    parser.add_argument("--minimum-validation-precision", type=float, default=0.96)
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()
    if not 0.5 <= args.minimum_validation_precision <= 1.0:
        raise ValueError("minimum validation precision must be in [0.5, 1.0]")

    paths = {name: args.features_dir / f"{name}_features.npz"
             for name in ("train", "validation", "test")}
    if missing := [str(path) for path in paths.values() if not path.is_file()]:
        raise FileNotFoundError("missing feature files: " + ", ".join(missing))
    split = {name: load_split(path) for name, path in paths.items()}
    episodes = {name: set(map(episode_id, payload["sources"]))
                for name, payload in split.items()}
    overlaps = {f"{left}_{right}": sorted(episodes[left] & episodes[right])[:10]
                for left, right in (("train", "validation"), ("train", "test"),
                                    ("validation", "test"))}
    if any(overlap for overlap in overlaps.values()):
        raise RuntimeError("episode leakage across supplied splits: " + repr(overlaps))

    center = split["train"]["x"].mean(axis=0)
    scale = split["train"]["x"].std(axis=0)
    if np.any(scale < 1e-8):
        raise RuntimeError("degenerate training feature scale")
    train_x = (split["train"]["x"] - center) / scale
    model = LogisticRegression(C=1.0, max_iter=1000, solver="lbfgs",
                               class_weight="balanced", random_state=20260824)
    model.fit(train_x, split["train"]["y"])
    scores = {name: model.predict_proba((payload["x"] - center) / scale)[:, 1]
              for name, payload in split.items()}
    current_mean = np.asarray(CURRENT["mean"])
    current_scale = np.asarray(CURRENT["scale"])
    current_weight = np.asarray(CURRENT["weight"])
    current_scores = {
        name: sigmoid(((payload["x"] - current_mean) / current_scale)
                      @ current_weight + CURRENT["bias"])
        for name, payload in split.items()
    }
    threshold = threshold_at_precision(split["validation"]["y"], scores["validation"],
                                       args.minimum_validation_precision)
    result = {
        "purpose": "POC-only gate acceptance confidence calibration; no backbone weights changed",
        "created_at": datetime.now(timezone.utc).isoformat(),
        "feature_names": list(FEATURE_NAMES),
        "source_feature_names": list(SOURCE_FEATURE_NAMES),
        "model": {"mean": center.tolist(), "scale": scale.tolist(),
                  "weight": model.coef_[0].tolist(), "bias": float(model.intercept_[0]),
                  "threshold": threshold,
                  "threshold_selection": f"max recall at validation precision >= {args.minimum_validation_precision}"},
        "fallback_current_runtime_coefficients": CURRENT,
        "metrics": {name: metrics(split[name]["y"], scores[name], threshold)
                    for name in split},
        "approximate_current_runtime_metrics": {
            "note": "The historical runtime used the mean of its 16 strongest mask pixels; this retained feature corpus stores top-1-percent mask strength, so this is a conservative compatibility comparison, not a replacement acceptance claim.",
            **{name: metrics(split[name]["y"], current_scores[name],
                            CURRENT["threshold"]) for name in split},
        },
        "provenance": {
            "feature_files": {name: {"path": str(path), "sha256": sha256(path)}
                              for name, path in paths.items()},
            "episodes": {name: {"count": len(episodes[name]),
                                 "sha256": hashlib.sha256("\n".join(sorted(episodes[name])).encode()).hexdigest()}
                         for name in episodes},
            "episode_overlap_examples": overlaps,
            "domains": {name: {key: int(value) for key, value in zip(*np.unique(
                split[name]["domains"], return_counts=True))} for name in split},
            "repo_commit": run_git(args.repo_root, "rev-parse", "HEAD"),
            "repo_dirty": run_git(args.repo_root, "status", "--short"),
            "python": sys.version, "platform": platform.platform(),
        },
    }
    if args.dry_run:
        print(json.dumps(result, indent=2, sort_keys=True))
        return
    args.output_dir.mkdir(parents=True, exist_ok=False)
    (args.output_dir / "calibration.json").write_text(json.dumps(result, indent=2, sort_keys=True) + "\n")
    (args.output_dir / "status.txt").write_text("success\n")
    print(json.dumps({"output_dir": str(args.output_dir), "validation": result["metrics"]["validation"],
                      "test": result["metrics"]["test"]}, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
