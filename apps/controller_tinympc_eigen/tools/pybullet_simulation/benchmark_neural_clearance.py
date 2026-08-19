#!/usr/bin/env python3
"""Benchmark the deployed four-direction ONNX clearance output in PyBullet."""

from __future__ import annotations

import argparse
import csv
import json
import math
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import pybullet as p

from firmware_pybullet_env import GateGeometry, GymPybulletGateEnv, GymPybulletGateEnvConfig


ANGLES_DEG = np.asarray([-40.0, -13.333333, 13.333333, 40.0])
MAX_CLEARANCE_M = 6.0


@dataclass(frozen=True)
class Box:
    name: str
    kind: str
    center: np.ndarray
    half: np.ndarray


@dataclass(frozen=True)
class Case:
    name: str
    trajectory: str
    boxes: tuple[Box, ...]


class Model:
    def __init__(self, path: Path) -> None:
        import onnxruntime as ort
        self.session = ort.InferenceSession(str(path.resolve()), providers=["CPUExecutionProvider"])
        self.input_name = self.session.get_inputs()[0].name
        manifest = json.loads(path.with_name("quantization_manifest.json").read_text())
        self.scale = float(manifest["scale"])

    def infer(self, frame: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        raw = np.asarray(self.session.run(None, {self.input_name: frame[20:140].astype(np.float32)[None, None]})[0])
        if raw.shape != (1, 12, 15, 20):
            raise RuntimeError(f"unexpected ONNX output shape {raw.shape}")
        logical = raw.astype(np.float32)[0] * self.scale - 6.0
        clearance = (np.clip(logical[4:8].mean(axis=(1, 2)), -6.0, 6.0) + 6.0) * 0.5
        confidence = logical[8:12].mean(axis=(1, 2))
        return clearance.astype(float), confidence.astype(float)


def box(name: str, kind: str, center, size) -> Box:
    return Box(name, kind, np.asarray(center, float), 0.5 * np.asarray(size, float))


def cases() -> list[Case]:
    gate = (
        box("gate_left", "gate_frame", (1.55, -0.42, 1.10), (0.10, 0.12, 0.90)),
        box("gate_right", "gate_frame", (1.55, 0.42, 1.10), (0.10, 0.12, 0.90)),
        box("gate_top", "gate_frame", (1.55, 0.0, 1.50), (0.10, 0.96, 0.10)),
        box("gate_bottom", "gate_frame", (1.55, 0.0, 0.70), (0.10, 0.96, 0.10)),
    )
    return [
        Case("straight_wall", "straight", (box("wall", "wall", (1.65, 0.0, 1.10), (0.12, 1.8, 1.2)),)),
        Case("straight_thin_post", "straight", (box("post", "thin_post", (1.55, 0.0, 1.10), (0.12, 0.12, 1.2)),)),
        Case("lateral_offset_block", "lateral_sweep", (box("block", "block", (1.55, 0.32, 1.10), (0.32, 0.38, 0.65)),)),
        Case("diagonal_tall_post", "diagonal", (box("post", "tall_post", (1.55, -0.20, 1.10), (0.16, 0.16, 1.5)),)),
        Case("slalom_two_blocks", "slalom", (
            box("left_block", "block", (1.25, -0.30, 1.10), (0.30, 0.42, 0.70)),
            box("right_block", "block", (1.85, 0.30, 1.10), (0.30, 0.42, 0.70)),
        )),
        Case("altitude_low_bar", "altitude_sweep", (box("bar", "low_bar", (1.55, 0.0, 0.88), (0.18, 1.10, 0.16)),)),
        Case("straight_gate_frame", "straight", gate),
        Case("lateral_gate_frame", "lateral_sweep", gate),
    ]


def pose(trajectory: str, u: float) -> tuple[np.ndarray, float]:
    x = -0.15 + 1.45 * u
    y, z, yaw = 0.0, 1.10, 0.0
    if trajectory == "lateral_sweep":
        y = -0.48 + 0.96 * u
    elif trajectory == "diagonal":
        y = -0.42 + 0.50 * u
        yaw = math.atan2(0.50, 1.45)
    elif trajectory == "slalom":
        y = 0.28 * math.sin(2.0 * math.pi * u)
    elif trajectory == "altitude_sweep":
        z = 0.72 + 0.70 * u
    return np.asarray([x, y, z]), yaw


def ray_aabb(origin: np.ndarray, direction: np.ndarray, obstacle: Box) -> float:
    low, high = obstacle.center - obstacle.half, obstacle.center + obstacle.half
    safe = np.where(np.abs(direction) < 1e-9, np.copysign(1e-9, direction + 1e-12), direction)
    t0, t1 = (low - origin) / safe, (high - origin) / safe
    entry, exit_ = max(0.0, float(np.minimum(t0, t1).max())), float(np.maximum(t0, t1).min())
    return entry if exit_ >= entry else math.inf


def ground_truth(position: np.ndarray, yaw: float, obstacles: tuple[Box, ...]) -> np.ndarray:
    distances = []
    for angle in ANGLES_DEG:
        heading = yaw + math.radians(float(angle))
        direction = np.asarray([math.cos(heading), math.sin(heading), 0.0])
        distances.append(min([ray_aabb(position, direction, item) for item in obstacles] + [MAX_CLEARANCE_M]))
    return np.minimum(distances, MAX_CLEARANCE_M)


def add_boxes(env: GymPybulletGateEnv, obstacles: tuple[Box, ...]) -> None:
    colors = {"wall": [0.75, 0.18, 0.12, 1], "thin_post": [0.95, 0.65, 0.05, 1],
              "tall_post": [0.95, 0.65, 0.05, 1], "block": [0.08, 0.42, 0.90, 1],
              "low_bar": [0.15, 0.70, 0.25, 1], "gate_frame": [0.55, 0.18, 0.75, 1]}
    for item in obstacles:
        collision = p.createCollisionShape(p.GEOM_BOX, halfExtents=item.half.tolist(), physicsClientId=env._client)
        visual = p.createVisualShape(p.GEOM_BOX, halfExtents=item.half.tolist(),
                                     rgbaColor=colors[item.kind], physicsClientId=env._client)
        p.createMultiBody(baseMass=0, baseCollisionShapeIndex=collision, baseVisualShapeIndex=visual,
                          basePosition=item.center.tolist(), physicsClientId=env._client)


def metrics(rows: list[dict], threshold: float) -> dict:
    truth = np.asarray([r["truth"] for r in rows], float)
    pred = np.asarray([r["prediction"] for r in rows], float)
    error = pred - truth
    truth_danger, pred_danger = truth < threshold, pred < threshold
    tp = int(np.sum(truth_danger & pred_danger)); fp = int(np.sum(~truth_danger & pred_danger))
    fn = int(np.sum(truth_danger & ~pred_danger)); tn = int(np.sum(~truth_danger & ~pred_danger))
    return {
        "samples": int(len(rows)), "danger_threshold_m": threshold,
        "mae_m": float(np.mean(np.abs(error))), "rmse_m": float(np.sqrt(np.mean(error * error))),
        "bias_m": float(np.mean(error)), "median_abs_error_m": float(np.median(np.abs(error))),
        "p90_abs_error_m": float(np.quantile(np.abs(error), 0.90)),
        "within_0_10m_fraction": float(np.mean(np.abs(error) <= 0.10)),
        "tp": tp, "fp": fp, "fn": fn, "tn": tn,
        "precision": tp / max(1, tp + fp), "recall": tp / max(1, tp + fn),
        "false_negative_rate": fn / max(1, tp + fn), "specificity": tn / max(1, tn + fp),
    }


def threshold_sweep(rows: list[dict], truth_threshold: float) -> dict:
    truth = np.asarray([r["truth"] for r in rows], float) < truth_threshold
    pred = np.asarray([r["prediction"] for r in rows], float)
    sweep = []
    for threshold in np.linspace(float(pred.min()), float(pred.max()), 1000):
        detected = pred < threshold
        tp = int(np.sum(truth & detected)); fp = int(np.sum(~truth & detected))
        fn = int(np.sum(truth & ~detected)); tn = int(np.sum(~truth & ~detected))
        recall = tp / max(1, tp + fn); precision = tp / max(1, tp + fp)
        f1 = 2 * precision * recall / max(1e-12, precision + recall)
        sweep.append({"prediction_threshold_m": float(threshold), "recall": recall,
                      "precision": precision, "false_positive_rate": fp / max(1, fp + tn), "f1": f1})
    best = max(sweep, key=lambda row: row["f1"])
    recall_targets = {}
    for target in (0.80, 0.90, 0.95, 0.99):
        eligible = [row for row in sweep if row["recall"] >= target]
        recall_targets[str(target)] = min(eligible, key=lambda row: row["false_positive_rate"]) if eligible else None
    return {"truth_danger_threshold_m": truth_threshold, "best_f1": best,
            "minimum_fpr_at_recall": recall_targets}


def _rank_auc(score: np.ndarray, positive: np.ndarray) -> float | None:
    """Probability that a random positive has a higher score than a negative."""
    n_pos, n_neg = int(positive.sum()), int((~positive).sum())
    if n_pos == 0 or n_neg == 0:
        return None
    order = np.argsort(score, kind="stable")
    ranks = np.empty(len(score), float)
    start = 0
    while start < len(order):
        end = start + 1
        while end < len(order) and score[order[end]] == score[order[start]]:
            end += 1
        ranks[order[start:end]] = 0.5 * (start + 1 + end)
        start = end
    return float((ranks[positive].sum() - n_pos * (n_pos + 1) / 2) / (n_pos * n_neg))


def confidence_reliability(rows: list[dict], danger_threshold: float,
                           prediction_threshold: float) -> dict:
    truth = np.asarray([r["truth"] for r in rows], float)
    pred = np.asarray([r["prediction"] for r in rows], float)
    confidence = np.asarray([r["confidence"] for r in rows], float)
    abs_error = np.abs(pred - truth)
    correct_danger = (truth < danger_threshold) == (pred < prediction_threshold)
    accurate_10cm = abs_error <= 0.10
    quantiles = np.quantile(confidence, np.linspace(0, 1, 11))
    bins = []
    for index in range(10):
        selected = (confidence >= quantiles[index]) & (
            confidence <= quantiles[index + 1] if index == 9 else confidence < quantiles[index + 1]
        )
        bins.append({
            "bin": index, "samples": int(selected.sum()),
            "confidence_min": float(quantiles[index]), "confidence_max": float(quantiles[index + 1]),
            "mean_confidence": float(confidence[selected].mean()),
            "mae_m": float(abs_error[selected].mean()),
            "within_0_10m_fraction": float(accurate_10cm[selected].mean()),
            "danger_decision_accuracy": float(correct_danger[selected].mean()),
        })
    selective = []
    for coverage in (1.0, .9, .75, .5, .25, .1):
        cutoff = float(np.quantile(confidence, 1.0 - coverage))
        selected = confidence >= cutoff
        selective.append({"requested_coverage": coverage, "actual_coverage": float(selected.mean()),
                          "confidence_min": cutoff, "mae_m": float(abs_error[selected].mean()),
                          "within_0_10m_fraction": float(accurate_10cm[selected].mean()),
                          "danger_decision_accuracy": float(correct_danger[selected].mean())})
    return {
        "score_note": "raw logical score; not a probability",
        "prediction_threshold_m": prediction_threshold,
        "confidence_range": [float(confidence.min()), float(confidence.max())],
        "pearson_confidence_vs_abs_error": float(np.corrcoef(confidence, abs_error)[0, 1]),
        "auc_for_error_within_0_10m": _rank_auc(confidence, accurate_10cm),
        "auc_for_correct_danger_decision": _rank_auc(confidence, correct_danger),
        "deciles": bins, "selective_coverage": selective,
    }


def plot_report(rows: list[dict], out: Path, threshold: float) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    truth = np.asarray([r["truth"] for r in rows]); pred = np.asarray([r["prediction"] for r in rows])
    fig, axes = plt.subplots(1, 2, figsize=(10, 4.2))
    axes[0].scatter(truth, pred, s=7, alpha=.25)
    axes[0].plot([0, 6], [0, 6], "k--", lw=1); axes[0].axvline(threshold, color="r", alpha=.5); axes[0].axhline(threshold, color="r", alpha=.5)
    axes[0].set(xlabel="geometric clearance [m]", ylabel="neural clearance [m]", xlim=(0, 6), ylim=(0, 6), title="All directional samples")
    labels = sorted({r["case"] for r in rows}); data = [[abs(r["prediction"]-r["truth"]) for r in rows if r["case"] == label] for label in labels]
    axes[1].boxplot(data, tick_labels=labels, showfliers=False); axes[1].tick_params(axis="x", rotation=55, labelsize=8)
    axes[1].set(ylabel="absolute error [m]", title="Error by scene")
    fig.tight_layout(); fig.savefig(out, dpi=180); plt.close(fig)


def plot_confidence(rows: list[dict], out: Path) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    confidence = np.asarray([r["confidence"] for r in rows], float)
    error = np.abs(np.asarray([r["prediction"] for r in rows], float) - np.asarray([r["truth"] for r in rows], float))
    edges = np.quantile(confidence, np.linspace(0, 1, 11)); centers, mae, accurate = [], [], []
    for i in range(10):
        selected = (confidence >= edges[i]) & (confidence <= edges[i+1] if i == 9 else confidence < edges[i+1])
        centers.append(confidence[selected].mean()); mae.append(error[selected].mean()); accurate.append(np.mean(error[selected] <= .10))
    fig, axes = plt.subplots(1, 2, figsize=(9, 4))
    axes[0].plot(centers, mae, "o-"); axes[0].set(xlabel="mean raw confidence score", ylabel="clearance MAE [m]", title="Confidence vs error")
    axes[1].plot(centers, accurate, "o-"); axes[1].set(xlabel="mean raw confidence score", ylabel="fraction within 0.10 m", ylim=(0, 1), title="Confidence vs accuracy")
    for ax in axes: ax.grid(True, alpha=.25)
    fig.tight_layout(); fig.savefig(out, dpi=180); plt.close(fig)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--onnx-model", type=Path, required=True)
    ap.add_argument("--out", type=Path, default=Path("sim_runs/neural_clearance_benchmark"))
    ap.add_argument("--frames-per-case", type=int, default=80)
    ap.add_argument("--danger-threshold", type=float, default=0.25)
    ap.add_argument("--overwrite", action="store_true")
    args = ap.parse_args()
    if args.out.exists() and not args.overwrite:
        raise SystemExit(f"{args.out} exists; pass --overwrite")
    args.out.mkdir(parents=True, exist_ok=True)
    model, all_rows = Model(args.onnx_model), []
    geometry = GateGeometry(3, 0, 1.1, .55, .5, .03)
    for case_index, case in enumerate(cases()):
        print(f"[{case_index+1}/{len(cases())}] {case.name}", flush=True)
        env = GymPybulletGateEnv(geometry, GymPybulletGateEnvConfig())
        try:
            env.reset(); add_boxes(env, case.boxes)
            body = int(env._aviary.DRONE_IDS[0])
            for frame_index, u in enumerate(np.linspace(0, 1, args.frames_per_case)):
                position, yaw = pose(case.trajectory, float(u)); quat = p.getQuaternionFromEuler([0, 0, yaw])
                p.resetBasePositionAndOrientation(body, position.tolist(), quat, physicsClientId=env._client)
                p.resetBaseVelocity(body, [0, 0, 0], [0, 0, 0], physicsClientId=env._client)
                prediction, confidence = model.infer(env.render_camera())
                truth = ground_truth(position, yaw, case.boxes)
                for direction in range(4):
                    all_rows.append({"case": case.name, "trajectory": case.trajectory, "frame": frame_index,
                                     "u": float(u), "x": position[0], "y": position[1], "z": position[2],
                                     "yaw": yaw, "direction": direction, "angle_deg": ANGLES_DEG[direction],
                                     "truth": truth[direction], "prediction": prediction[direction],
                                     "confidence": confidence[direction]})
        finally:
            env.close()
    with (args.out / "samples.csv").open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(all_rows[0])); writer.writeheader(); writer.writerows(all_rows)
    near_rows = [r for r in all_rows if r["truth"] < 1.0]
    sweep = threshold_sweep(all_rows, args.danger_threshold)
    best_threshold = float(sweep["best_f1"]["prediction_threshold_m"])
    report = {"model": str(args.onnx_model.resolve()), "definition": "first AABB surface along each fixed horizontal normal",
              "angles_deg": ANGLES_DEG.tolist(), "overall": metrics(all_rows, args.danger_threshold),
              "near_obstacle_truth_below_1m": metrics(near_rows, args.danger_threshold),
              "prediction_threshold_sweep": sweep,
              "confidence_reliability": confidence_reliability(all_rows, args.danger_threshold, best_threshold),
              "by_case": {case.name: metrics([r for r in all_rows if r["case"] == case.name], args.danger_threshold) for case in cases()},
              "by_direction": {str(i): metrics([r for r in all_rows if r["direction"] == i], args.danger_threshold) for i in range(4)}}
    (args.out / "report.json").write_text(json.dumps(report, indent=2) + "\n")
    plot_report(all_rows, args.out / "clearance_accuracy.png", args.danger_threshold)
    plot_confidence(all_rows, args.out / "confidence_reliability.png")
    print(json.dumps(report["overall"], indent=2)); print(f"wrote {args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
