#!/usr/bin/env python3
"""Run racing-relevant TinyMPC maneuvers sequentially and compare model schedules."""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
import subprocess
import sys
from typing import Any

import numpy as np

from firmware_pybullet_env import GymPybulletGateEnv
from generate_racing_trajectories import MANEUVERS, generate


APP_ROOT = Path(__file__).resolve().parents[2]
RUNNER = Path(__file__).with_name("run_maneuver_sim.py")

REALISTIC_V1 = (
    "--motor-time-constant-ms", "10",
    "--motor-command-delay-ms", "2",
    "--controller-compute-delay-ms", "10",
    "--plant-mass-scale", "1.05",
    "--plant-inertia-scale", "1.10",
    "--plant-thrust-scale", "0.95",
    "--plant-motor-thrust-scales", "0.98,1.01,0.97,1.00",
    "--rotor-drag-scale", "1",
    "--state-estimate-delay-ms", "4",
    "--position-noise-std-m", "0.003",
    "--velocity-noise-std-mps", "0.02",
    "--attitude-noise-std-deg", "0.15",
    "--gyro-noise-std-deg-s", "0.5",
)


def _rows(path: Path) -> list[dict[str, str]]:
    with path.open(newline="") as stream:
        return list(csv.DictReader(stream))


def _quaternion_rpy(quaternion_wxyz: np.ndarray) -> np.ndarray:
    q = np.asarray(quaternion_wxyz, dtype=np.float64)
    qw, qx, qy, qz = q.T
    return np.column_stack((
        np.arctan2(2.0 * (qw * qx + qy * qz), 1.0 - 2.0 * (qx * qx + qy * qy)),
        np.arcsin(np.clip(2.0 * (qw * qy - qz * qx), -1.0, 1.0)),
        np.arctan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz)),
    ))


def _angle_error(actual: np.ndarray, reference: np.ndarray) -> np.ndarray:
    return np.arctan2(np.sin(actual - reference), np.cos(actual - reference))


def _analyze(
    run_dir: Path, trajectory_path: Path, tracking_tolerance_m: float
) -> tuple[dict[str, Any], dict[str, np.ndarray]]:
    summary = json.loads((run_dir / "summary.json").read_text())
    actual_rows = _rows(run_dir / "closed_loop.csv")
    reference_rows = _rows(trajectory_path)
    t = np.asarray([float(row["t"]) for row in actual_rows])
    actual_position = np.asarray([[float(row[key]) for key in ("x", "y", "z")]
                                  for row in actual_rows])
    actual_rpy = np.asarray([[float(row[key]) for key in ("roll_rad", "pitch_rad", "yaw_rad")]
                             for row in actual_rows])
    motor = np.asarray([[float(row[f"motor_{index}_thrust_n"]) for index in range(4)]
                        for row in actual_rows])
    reference_t = np.asarray([float(row["t"]) for row in reference_rows])
    trajectory_t = np.maximum(0.0, t - float(summary.get("trajectory_handoff_hold_s", 0.0)))
    reference_position_source = np.asarray(
        [[float(row[key]) for key in ("x", "y", "z")] for row in reference_rows]
    )
    reference_position = np.column_stack([
        np.interp(trajectory_t, reference_t, reference_position_source[:, axis]) for axis in range(3)
    ])
    reference_quaternion = np.asarray(
        [[float(row[key]) for key in ("qw", "qx", "qy", "qz")] for row in reference_rows]
    )
    reference_rpy_source = _quaternion_rpy(reference_quaternion)
    reference_rpy = np.column_stack([
        np.interp(trajectory_t, reference_t, np.unwrap(reference_rpy_source[:, axis])) for axis in range(3)
    ])
    position_error = np.linalg.norm(actual_position - reference_position, axis=1)
    altitude_error = actual_position[:, 2] - reference_position[:, 2]
    roll_error = _angle_error(actual_rpy[:, 0], reference_rpy[:, 0])
    actual_tilt = np.arccos(np.clip(
        np.cos(actual_rpy[:, 0]) * np.cos(actual_rpy[:, 1]), -1.0, 1.0
    ))
    reference_tilt = np.arccos(np.clip(
        np.cos(reference_rpy[:, 0]) * np.cos(reference_rpy[:, 1]), -1.0, 1.0
    ))
    motor_limit = GymPybulletGateEnv.FIRMWARE_MAX_MOTOR_THRUST_N
    final_reference = reference_position_source[-1]
    result = {
        "collision": bool(summary["collision"]),
        "sim_time_s": float(summary["sim_time_s"]),
        "mpc_solve_count": int(summary["mpc_solve_count"]),
        "model_switch_count": int(summary["model_switch_count"]),
        "reached_goal": bool(summary["reached_goal"]),
        "position_rmse_m": float(np.sqrt(np.mean(position_error**2))),
        "peak_position_error_m": float(np.max(position_error)),
        "within_tracking_tolerance_fraction": float(np.mean(position_error <= tracking_tolerance_m)),
        "final_position_error_m": float(np.linalg.norm(actual_position[-1] - final_reference)),
        "altitude_rmse_m": float(np.sqrt(np.mean(altitude_error**2))),
        "minimum_altitude_m": float(np.min(actual_position[:, 2])),
        "maximum_altitude_error_m": float(np.max(np.abs(altitude_error))),
        "roll_rmse_deg": math.degrees(float(np.sqrt(np.mean(roll_error**2)))),
        "reference_peak_tilt_deg": math.degrees(float(np.max(reference_tilt))),
        "peak_tilt_deg": math.degrees(float(np.max(actual_tilt))),
        "minimum_motor_thrust_n": float(np.min(motor)),
        "maximum_motor_thrust_n": float(np.max(motor)),
        "peak_motor_utilization_fraction": float(np.max(motor) / motor_limit),
        "motor_saturation_fraction": float(np.mean(np.any(motor >= 0.99 * motor_limit, axis=1))),
    }
    result["bounded_flight"] = bool(
        not result["collision"]
        and result["minimum_altitude_m"] >= 0.25
    )
    result["tracking_pass"] = bool(
        result["bounded_flight"]
        and result["reached_goal"]
        and result["within_tracking_tolerance_fraction"] >= 0.95
    )
    plot_data = {
        "t": t,
        "actual_position": actual_position,
        "reference_position": reference_position,
        "actual_tilt_deg": np.degrees(actual_tilt),
    }
    return result, plot_data


def _plot_comparison(
    out: Path, title: str, series: list[tuple[str, dict[str, np.ndarray]]]
) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    figure, axes = plt.subplots(1, 3, figsize=(14, 4.2))
    reference = series[0][1]["reference_position"]
    axes[0].plot(reference[:, 0], reference[:, 1], "k--", linewidth=2, label="reference")
    for label, data in series:
        position = data["actual_position"]
        axes[0].plot(position[:, 0], position[:, 1], linewidth=1.8, label=label)
        axes[1].plot(data["t"], position[:, 2], linewidth=1.8, label=label)
        axes[2].plot(data["t"], data["actual_tilt_deg"], linewidth=1.8, label=label)
    axes[1].plot(series[0][1]["t"], reference[:, 2], "k--", linewidth=2, label="reference")
    axes[0].set(xlabel="x [m]", ylabel="y [m]", title="Top-down tracking")
    axes[0].axis("equal")
    axes[1].set(xlabel="time [s]", ylabel="z [m]", title="Altitude")
    axes[2].set(xlabel="time [s]", ylabel="tilt [deg]", title="Actual attitude tilt")
    for axis in axes:
        axis.grid(True, alpha=0.25)
    axes[0].legend(fontsize=8)
    figure.suptitle(title)
    figure.tight_layout()
    out.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(out, dpi=160)
    plt.close(figure)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--out", type=Path, default=Path("sim_runs/racing_maneuver_suite"))
    parser.add_argument("--trajectory-dir", type=Path, default=Path("sim/trajectories/racing"))
    parser.add_argument("--maneuver", action="append", choices=[item.name for item in MANEUVERS])
    parser.add_argument("--profile", choices=("realistic-v1", "ideal"), default="realistic-v1")
    parser.add_argument("--tracking-tolerance-m", type=float, default=0.30,
                        help="centerline error used for racing tracking pass/fail")
    parser.add_argument("--scheduled-only", action="store_true")
    parser.add_argument(
        "--native-retries", type=int, default=3,
        help="retry a run when the isolated PyBullet process exits from a native signal",
    )
    parser.add_argument("--overwrite", action="store_true")
    args = parser.parse_args()
    if args.native_retries < 0:
        parser.error("--native-retries must be nonnegative")
    if args.out.exists() and not args.overwrite:
        raise SystemExit(f"{args.out} exists; use --overwrite or choose --out")
    args.out.mkdir(parents=True, exist_ok=True)
    selected = set(args.maneuver or [item.name for item in MANEUVERS])
    results: list[dict[str, Any]] = []
    for maneuver in MANEUVERS:
        if maneuver.name not in selected:
            continue
        trajectory_path = args.trajectory_dir / f"{maneuver.name}.csv"
        trajectory_info = generate(maneuver, trajectory_path)
        variants = [("scheduled", maneuver.model_schedule)]
        if not args.scheduled_only:
            variants.append(("level", "level"))
        plot_series: list[tuple[str, dict[str, np.ndarray]]] = []
        for variant, schedule in variants:
            run_dir = args.out / maneuver.name / variant
            command = [
                sys.executable, str(RUNNER),
                "--out", str(run_dir),
                "--duration", str(float(trajectory_info["duration_s"]) + 1.5),
                "--trajectory-file", str(trajectory_path),
                "--control-mode", "admm",
                "--model-schedule", schedule,
                "--progress-interval-s", "0",
                "--plot", "--overwrite",
            ]
            if args.profile == "realistic-v1":
                command.extend(REALISTIC_V1)
            completed = subprocess.run(command, cwd=APP_ROOT, text=True, capture_output=True)
            retry = 0
            while completed.returncode < 0 and retry < args.native_retries:
                retry += 1
                print(
                    f"RETRY {maneuver.name}/{variant}: PyBullet exited from signal "
                    f"{-completed.returncode} ({retry}/{args.native_retries})",
                    file=sys.stderr,
                )
                completed = subprocess.run(command, cwd=APP_ROOT, text=True, capture_output=True)
            if completed.returncode != 0:
                raise RuntimeError(
                    f"{maneuver.name}/{variant} failed ({completed.returncode}):\n"
                    f"{(completed.stderr or completed.stdout)[-3000:]}"
                )
            metrics, data = _analyze(run_dir, trajectory_path, float(args.tracking_tolerance_m))
            row = {
                "maneuver": maneuver.name,
                "description": maneuver.description,
                "variant": variant,
                "model_schedule": schedule,
                "profile": args.profile,
                "speed_mps": maneuver.speed_mps,
                "reference_bank_deg": maneuver.bank_deg,
                **metrics,
                "run_dir": str(run_dir),
            }
            results.append(row)
            plot_series.append((variant, data))
            print(
                f"{'PASS' if metrics['tracking_pass'] else 'FAIL'} "
                f"{maneuver.name:26s} {variant:9s} "
                f"rmse={metrics['position_rmse_m']:.3f}m "
                f"peak={metrics['peak_position_error_m']:.3f}m "
                f"in_tol={100.0 * metrics['within_tracking_tolerance_fraction']:.1f}% "
                f"minz={metrics['minimum_altitude_m']:.3f}m "
                f"sat={100.0 * metrics['motor_saturation_fraction']:.1f}% "
                f"collision={metrics['collision']}"
            )
        _plot_comparison(
            args.out / maneuver.name / "comparison.png",
            f"{maneuver.name.replace('_', ' ').title()} — {args.profile}",
            plot_series,
        )

    fieldnames = list(results[0]) if results else []
    with (args.out / "results.csv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(results)
    summary = {
        "profile": args.profile,
        "controller": {"mpc_rate_hz": 50, "max_iterations": 5, "horizon_knots": 20, "model_dt_s": 0.02},
        "tracking_acceptance": {
            "centerline_tolerance_m": float(args.tracking_tolerance_m),
            "minimum_fraction_within_tolerance": 0.95,
            "must_reach_goal": True,
            "minimum_altitude_m": 0.25,
            "collision": False,
        },
        "runs": len(results),
        "passes": sum(bool(row["tracking_pass"]) for row in results),
        "collisions": sum(bool(row["collision"]) for row in results),
        "results": results,
    }
    (args.out / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    print(json.dumps({key: summary[key] for key in ("profile", "runs", "passes", "collisions")}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
