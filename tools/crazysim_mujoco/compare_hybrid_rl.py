#!/usr/bin/env python3
"""Compare paired held-out DroNet and hybrid-RL CrazySim runs."""

from __future__ import annotations

import argparse
import copy
import csv
import json
import math
from pathlib import Path
from typing import Any


EXPECTED_SEEDS = (101, 202, 303)
ALLOWED_CONFIG_DIFFERENCES = ("random_seed", "vision_model", "vision_adapter")


def load_run(path: Path, controller: str) -> dict[str, Any]:
    config_path = path / "run_config.json"
    summary_path = path / "summary.json"
    if not config_path.is_file() or not summary_path.is_file():
        raise ValueError(f"{path}: missing run_config.json or summary.json")
    config = json.loads(config_path.read_text())
    summary = json.loads(summary_path.read_text())
    seed = config.get("random_seed")
    if seed not in EXPECTED_SEEDS:
        raise ValueError(f"{path}: unexpected held-out seed {seed!r}")
    required = (
        "course_success", "course_completion_time_s",
        "course_mean_horizontal_speed_mps", "course_obstacle_clearance_min_m",
        "course_wall_clearance_min_m", "course_cross_track_error_rmse_m",
        "course_cross_track_error_p95_m",
        "course_reverse_tangential_motion_fraction",
        "vision_inference_latency_ms",
    )
    missing = [name for name in required if name not in summary]
    if missing:
        raise ValueError(f"{path}: summary missing {', '.join(missing)}")
    return {
        "controller": controller, "seed": seed, "path": str(path.resolve()),
        "config": config, "summary": summary,
    }


def indexed_runs(paths: list[Path], controller: str) -> dict[int, dict[str, Any]]:
    runs: dict[int, dict[str, Any]] = {}
    for path in paths:
        run = load_run(path, controller)
        seed = int(run["seed"])
        if seed in runs:
            raise ValueError(f"duplicate {controller} seed {seed}")
        runs[seed] = run
    if tuple(sorted(runs)) != EXPECTED_SEEDS:
        raise ValueError(
            f"{controller} must contain exactly seeds {EXPECTED_SEEDS}, "
            f"received {tuple(sorted(runs))}")
    return runs


def comparison_config(config: dict[str, Any], *, remove_seed: bool) -> dict[str, Any]:
    normalized = copy.deepcopy(config)
    for name in ALLOWED_CONFIG_DIFFERENCES:
        if name == "random_seed" and not remove_seed:
            continue
        normalized.pop(name, None)
    return normalized


def require_config_fidelity(
    baseline: dict[int, dict[str, Any]], hybrid: dict[int, dict[str, Any]],
) -> None:
    reference = comparison_config(baseline[EXPECTED_SEEDS[0]]["config"], remove_seed=True)
    for controller_runs in (baseline, hybrid):
        first = controller_runs[EXPECTED_SEEDS[0]]["config"]
        expected_identity = (first.get("vision_model"), first.get("vision_adapter"))
        for seed, run in controller_runs.items():
            current = comparison_config(run["config"], remove_seed=True)
            if current != reference:
                raise ValueError(
                    f"configuration drift in {run['controller']} seed {seed}; "
                    "only random_seed, vision_model, and vision_adapter may differ")
            identity = (run["config"].get("vision_model"),
                        run["config"].get("vision_adapter"))
            if identity != expected_identity:
                raise ValueError(
                    f"model/adapter identity drift in {run['controller']} seed {seed}")
    if baseline[EXPECTED_SEEDS[0]]["config"].get("vision_adapter") != "dronet":
        raise ValueError("baseline vision_adapter must be dronet")
    if hybrid[EXPECTED_SEEDS[0]]["config"].get("vision_adapter") != "hybrid_rl":
        raise ValueError("hybrid vision_adapter must be hybrid_rl")
    for seed in EXPECTED_SEEDS:
        if baseline[seed]["config"].get("random_seed") != hybrid[seed]["config"].get(
                "random_seed"):
            raise ValueError(f"unpaired random seed {seed}")


def load_training_seeds(path: Path) -> set[int]:
    manifest = json.loads(path.read_text())
    training = manifest.get("training", manifest)
    if "training_seeds" not in training or not isinstance(
            training["training_seeds"], list):
        raise ValueError("training manifest must contain a training_seeds list")
    seeds = {int(value) for value in training["training_seeds"]}
    overlap = seeds.intersection(EXPECTED_SEEDS)
    if overlap:
        raise ValueError(f"held-out seeds appear in training manifest: {sorted(overlap)}")
    return seeds


def finite_mean(values: list[Any]) -> float | None:
    finite = [float(value) for value in values
              if value is not None and math.isfinite(float(value))]
    return sum(finite) / len(finite) if finite else None


def run_row(run: dict[str, Any]) -> dict[str, Any]:
    summary = run["summary"]
    latency = summary["vision_inference_latency_ms"]
    return {
        "controller": run["controller"],
        "random_seed": run["seed"],
        "run_directory": run["path"],
        "contact_free_course_completion": bool(summary["course_success"]),
        "course_completion_time_s": summary["course_completion_time_s"],
        "course_mean_horizontal_speed_mps": summary["course_mean_horizontal_speed_mps"],
        "minimum_obstacle_clearance_m": summary["course_obstacle_clearance_min_m"],
        "minimum_wall_clearance_m": summary["course_wall_clearance_min_m"],
        "cross_track_rmse_m": summary["course_cross_track_error_rmse_m"],
        "cross_track_p95_m": summary["course_cross_track_error_p95_m"],
        "reverse_tangential_motion_fraction": summary[
            "course_reverse_tangential_motion_fraction"],
        "inference_latency_p50_ms": latency["p50"],
        "inference_latency_p95_ms": latency["p95"],
        "inference_latency_max_ms": latency["maximum"],
        "emulated_delivery_latency_ms": latency.get("emulated_delivery_mean"),
    }


def aggregate(rows: list[dict[str, Any]]) -> dict[str, Any]:
    return {
        "trials": len(rows),
        "contact_free_completions": sum(
            bool(row["contact_free_course_completion"]) for row in rows),
        "contact_free_completion_rate": sum(
            bool(row["contact_free_course_completion"]) for row in rows) / len(rows),
        "mean_completion_time_s": finite_mean([
            row["course_completion_time_s"] for row in rows]),
        "mean_horizontal_speed_mps": finite_mean([
            row["course_mean_horizontal_speed_mps"] for row in rows]),
        "minimum_obstacle_clearance_m": min(
            float(row["minimum_obstacle_clearance_m"]) for row in rows),
        "minimum_wall_clearance_m": min(
            float(row["minimum_wall_clearance_m"]) for row in rows),
        "mean_cross_track_rmse_m": finite_mean([
            row["cross_track_rmse_m"] for row in rows]),
        "mean_cross_track_p95_m": finite_mean([
            row["cross_track_p95_m"] for row in rows]),
        "mean_reverse_tangential_motion_fraction": finite_mean([
            row["reverse_tangential_motion_fraction"] for row in rows]),
        "mean_inference_latency_p95_ms": finite_mean([
            row["inference_latency_p95_ms"] for row in rows]),
        "maximum_inference_latency_ms": max(
            float(row["inference_latency_max_ms"]) for row in rows),
    }


def compare(
    baseline_paths: list[Path], hybrid_paths: list[Path], training_manifest: Path,
) -> tuple[dict[str, Any], list[dict[str, Any]]]:
    training_seeds = load_training_seeds(training_manifest)
    baseline = indexed_runs(baseline_paths, "dronet_v3")
    hybrid = indexed_runs(hybrid_paths, "hybrid_rl")
    require_config_fidelity(baseline, hybrid)
    rows = [run_row(runs[seed]) for runs in (baseline, hybrid)
            for seed in EXPECTED_SEEDS]
    report = {
        "format": "tinympc-hybrid-rl-comparison-v1",
        "held_out_seeds": list(EXPECTED_SEEDS),
        "training_manifest": str(training_manifest.resolve()),
        "training_seed_count": len(training_seeds),
        "allowed_config_differences": list(ALLOWED_CONFIG_DIFFERENCES),
        "runs": rows,
        "controllers": {
            controller: aggregate([row for row in rows
                                   if row["controller"] == controller])
            for controller in ("dronet_v3", "hybrid_rl")
        },
    }
    return report, rows


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--baseline-run", action="append", required=True, type=Path)
    parser.add_argument("--hybrid-run", action="append", required=True, type=Path)
    parser.add_argument("--training-manifest", required=True, type=Path)
    parser.add_argument("--out", required=True, type=Path)
    args = parser.parse_args()
    report, rows = compare(
        args.baseline_run, args.hybrid_run, args.training_manifest)
    args.out.mkdir(parents=True, exist_ok=True)
    (args.out / "comparison.json").write_text(
        json.dumps(report, indent=2, allow_nan=False) + "\n")
    with (args.out / "comparison.csv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    print(json.dumps(report, indent=2, allow_nan=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
