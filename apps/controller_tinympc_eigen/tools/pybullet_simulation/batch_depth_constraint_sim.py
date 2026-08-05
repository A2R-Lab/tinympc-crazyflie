#!/usr/bin/env python3
"""Batch stress tests for the PyBullet depth-constraint controller sim."""

from __future__ import annotations

import argparse
import csv
import json
import subprocess
import sys
from pathlib import Path
from typing import Any

import numpy as np


APP_ROOT = Path(__file__).resolve().parents[2]
RUNNER = Path(__file__).with_name("run_depth_constraint_sim.py")


def parse_args() -> argparse.Namespace:
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", type=Path, default=Path("flow_sim_dataset/depth_constraint_batch"))
    ap.add_argument("--cases", type=int, default=24)
    ap.add_argument("--seed", type=int, default=7)
    ap.add_argument("--duration", type=float, default=8.0)
    ap.add_argument("--trajectory-file", type=Path, default=None,
                    help="CSV reference trajectory passed to each run")
    ap.add_argument("--control-mode", choices=["admm", "projection"], default="admm")
    ap.add_argument("--include-baseline", action="store_true",
                    help="also run every scenario with --no-avoidance")
    ap.add_argument("--multi-obstacle-prob", type=float, default=0.30)
    ap.add_argument("--depth-noise-std", type=float, default=0.04)
    ap.add_argument("--depth-bias-range", type=float, default=0.08,
                    help="sample constant depth bias uniformly from +/- this value [m]")
    ap.add_argument("--sector-dropout-prob", type=float, default=0.08)
    ap.add_argument("--false-hit-prob", type=float, default=0.03)
    ap.add_argument("--constraint-max-age-s", type=float, default=0.20,
                    help="passed through to pybullet_simulation/run_depth_constraint_sim.py")
    ap.add_argument("--admm-reference-sidestep", type=float, default=None,
                    help="override pybullet_simulation/run_depth_constraint_sim.py sidestep reference [m]")
    ap.add_argument("--admm-forward-slack-scale", type=float, default=None,
                    help="override pybullet_simulation/run_depth_constraint_sim.py forward slack cap")
    ap.add_argument("--camera-rate-choices", default="10,15,20,30",
                    help="comma-separated camera update rates to sample [Hz]")
    ap.add_argument("--target-speed-choices", default="0.14,0.18,0.22",
                    help="comma-separated forward reference speeds to sample [m/s]")
    ap.add_argument("--max-forward-speed", type=float, default=None)
    ap.add_argument("--max-lateral-speed", type=float, default=None)
    ap.add_argument("--max-accel", type=float, default=None)
    ap.add_argument("--start-k", type=int, default=None)
    ap.add_argument("--tinympc-max-iter", type=int, default=None)
    ap.add_argument("--max-active-halfspaces", type=int, default=None)
    ap.add_argument("--margin-min-choices", default="0.80,0.85,0.90",
                    help="comma-separated minimum margins to sample [m]")
    ap.add_argument("--overwrite", action="store_true")
    return ap.parse_args()


def main() -> int:
    args = parse_args()
    if args.out.exists() and not args.overwrite:
        raise SystemExit(f"{args.out} exists; use --overwrite or choose --out")
    args.out.mkdir(parents=True, exist_ok=True)

    rng = np.random.default_rng(int(args.seed))
    camera_rates = _parse_float_choices(args.camera_rate_choices)
    target_speeds = _parse_float_choices(args.target_speed_choices)
    margin_mins = _parse_float_choices(args.margin_min_choices)

    scenarios: list[dict[str, Any]] = []
    results: list[dict[str, Any]] = []
    for case_idx in range(int(args.cases)):
        scenario = _sample_scenario(case_idx, rng, camera_rates, target_speeds, margin_mins, args)
        scenarios.append(scenario)
        variants = [("avoid", False)]
        if args.include_baseline:
            variants.append(("baseline", True))
        for variant, no_avoidance in variants:
            result = _run_case(args.out, scenario, variant, bool(no_avoidance), args)
            results.append(result)
            status = "PASS" if result.get("passed") else "FAIL"
            print(
                f"{status} case={case_idx:03d} variant={variant} "
                f"collision={result.get('collision')} reached={result.get('reached_goal')} "
                f"clearance={result.get('min_obstacle_clearance_m')}"
            )

    _write_results(args.out / "batch_results.csv", results)
    (args.out / "scenarios.json").write_text(json.dumps(scenarios, indent=2) + "\n")
    summary = _summarize(results, args)
    (args.out / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    print(json.dumps(summary, indent=2))
    return 0


def _run_case(
    out_root: Path,
    scenario: dict[str, Any],
    variant: str,
    no_avoidance: bool,
    args: argparse.Namespace,
) -> dict[str, Any]:
    case_out = out_root / f"case_{int(scenario['case_idx']):03d}_{variant}"
    command = [
        sys.executable,
        str(RUNNER),
        "--out",
        str(case_out),
        "--duration",
        str(float(args.duration)),
        "--control-mode",
        str(args.control_mode),
        "--target-speed",
        str(float(scenario["target_speed"])),
        "--margin-min",
        str(float(scenario["margin_min"])),
        "--camera-rate-hz",
        str(float(scenario["camera_rate_hz"])),
        "--depth-noise-std",
        str(float(args.depth_noise_std)),
        "--depth-bias",
        str(float(scenario["depth_bias"])),
        "--sector-dropout-prob",
        str(float(args.sector_dropout_prob)),
        "--false-hit-prob",
        str(float(args.false_hit_prob)),
        "--constraint-max-age-s",
        str(float(args.constraint_max_age_s)),
        "--seed",
        str(int(scenario["seed"])),
        "--overwrite",
    ]
    for obstacle in scenario["obstacles"]:
        command.extend(["--obstacle", _format_obstacle(obstacle)])
    if args.trajectory_file is not None:
        command.extend(["--trajectory-file", str(args.trajectory_file)])
    if no_avoidance:
        command.append("--no-avoidance")
    if args.admm_reference_sidestep is not None:
        command.extend(["--admm-reference-sidestep", str(float(args.admm_reference_sidestep))])
    if args.admm_forward_slack_scale is not None:
        command.extend(["--admm-forward-slack-scale", str(float(args.admm_forward_slack_scale))])
    if args.max_forward_speed is not None:
        command.extend(["--max-forward-speed", str(float(args.max_forward_speed))])
    if args.max_lateral_speed is not None:
        command.extend(["--max-lateral-speed", str(float(args.max_lateral_speed))])
    if args.max_accel is not None:
        command.extend(["--max-accel", str(float(args.max_accel))])
    if args.start_k is not None:
        command.extend(["--start-k", str(int(args.start_k))])
    if args.tinympc_max_iter is not None:
        command.extend(["--tinympc-max-iter", str(int(args.tinympc_max_iter))])
    if args.max_active_halfspaces is not None:
        command.extend(["--max-active-halfspaces", str(int(args.max_active_halfspaces))])

    result: dict[str, Any] = {
        "case_idx": scenario["case_idx"],
        "variant": variant,
        "run_dir": str(case_out),
        "returncode": 0,
        "error": "",
        **{k: scenario[k] for k in ("target_speed", "margin_min", "camera_rate_hz", "depth_bias")},
        "n_obstacles": len(scenario["obstacles"]),
    }
    try:
        completed = subprocess.run(command, cwd=APP_ROOT, text=True, capture_output=True, check=False)
        result["returncode"] = int(completed.returncode)
        if completed.returncode != 0:
            result["error"] = (completed.stderr or completed.stdout)[-1000:]
            result["passed"] = False
            return result
        summary = json.loads((case_out / "summary.json").read_text())
        result.update(summary)
        reached_goal = bool(summary.get("reached_goal", summary.get("reached_goal_x", False)))
        result["reached_goal"] = reached_goal
        result["passed"] = bool((not summary.get("collision", True)) and reached_goal)
        return result
    except Exception as exc:
        result["returncode"] = -1
        result["error"] = str(exc)
        result["passed"] = False
        return result


def _sample_scenario(
    case_idx: int,
    rng: np.random.Generator,
    camera_rates: list[float],
    target_speeds: list[float],
    margin_mins: list[float],
    args: argparse.Namespace,
) -> dict[str, Any]:
    n_obstacles = 2 if rng.random() < float(args.multi_obstacle_prob) else 1
    obstacles = []
    for obs_idx in range(n_obstacles):
        x = float(rng.uniform(1.20, 2.20))
        y = float(rng.uniform(-0.38, 0.38))
        z = float(rng.uniform(0.82, 1.28))
        hx = float(rng.uniform(0.10, 0.24))
        hy = float(rng.uniform(0.12, 0.28))
        hz = float(rng.uniform(0.18, 0.36))
        if obs_idx > 0:
            x += float(rng.uniform(0.20, 0.60))
            y += float(rng.choice([-1.0, 1.0]) * rng.uniform(0.20, 0.45))
        obstacles.append({"name": f"box{obs_idx}", "x": x, "y": y, "z": z, "hx": hx, "hy": hy, "hz": hz})
    return {
        "case_idx": int(case_idx),
        "seed": int(rng.integers(1, 2**31 - 1)),
        "target_speed": float(rng.choice(target_speeds)),
        "margin_min": float(rng.choice(margin_mins)),
        "camera_rate_hz": float(rng.choice(camera_rates)),
        "depth_bias": float(rng.uniform(-float(args.depth_bias_range), float(args.depth_bias_range))),
        "obstacles": obstacles,
    }


def _summarize(results: list[dict[str, Any]], args: argparse.Namespace) -> dict[str, Any]:
    by_variant: dict[str, list[dict[str, Any]]] = {}
    for row in results:
        by_variant.setdefault(str(row["variant"]), []).append(row)
    variants = {}
    for variant, rows in by_variant.items():
        clearances = [float(r["min_obstacle_clearance_m"]) for r in rows if "min_obstacle_clearance_m" in r]
        variants[variant] = {
            "runs": len(rows),
            "passed": sum(1 for r in rows if r.get("passed")),
            "collisions": sum(1 for r in rows if r.get("collision")),
            "obstacle_collisions": sum(1 for r in rows if r.get("obstacle_collision")),
            "env_collisions": sum(1 for r in rows if r.get("env_collision")),
            "reached_goal": sum(1 for r in rows if r.get("reached_goal", r.get("reached_goal_x"))),
            "failed_processes": sum(1 for r in rows if int(r.get("returncode", 0)) != 0),
            "min_clearance_m": min(clearances) if clearances else None,
            "median_clearance_m": float(np.median(clearances)) if clearances else None,
            "p10_clearance_m": float(np.percentile(clearances, 10)) if clearances else None,
        }
    return {
        "cases": int(args.cases),
        "control_mode": str(args.control_mode),
        "include_baseline": bool(args.include_baseline),
        "variants": variants,
    }


def _write_results(path: Path, rows: list[dict[str, Any]]) -> None:
    fields = [
        "case_idx",
        "variant",
        "passed",
        "returncode",
        "collision",
        "obstacle_collision",
        "env_collision",
        "reached_goal",
        "reached_goal_x",
        "min_obstacle_clearance_m",
        "min_z_m",
        "sim_time_s",
        "constraint_active_fraction",
        "max_command_jump_mps",
        "max_terminal_jump_m",
        "dropout_count",
        "false_hit_count",
        "stale_disabled_count",
        "switch_suppressed_count",
        "target_speed",
        "margin_min",
        "camera_rate_hz",
        "depth_bias",
        "n_obstacles",
        "max_active_halfspaces",
        "run_dir",
        "error",
    ]
    with path.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fields, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)


def _parse_float_choices(raw: str) -> list[float]:
    values = [float(part.strip()) for part in str(raw).split(",") if part.strip()]
    if not values:
        raise SystemExit(f"empty choice list: {raw!r}")
    return values


def _format_obstacle(obstacle: dict[str, Any]) -> str:
    return ",".join(
        [
            str(obstacle["name"]),
            f"{float(obstacle['x']):.6f}",
            f"{float(obstacle['y']):.6f}",
            f"{float(obstacle['z']):.6f}",
            f"{float(obstacle['hx']):.6f}",
            f"{float(obstacle['hy']):.6f}",
            f"{float(obstacle['hz']):.6f}",
        ]
    )


if __name__ == "__main__":
    raise SystemExit(main())
