#!/usr/bin/env python3
"""Run and score Tier-3 TinyMPC roll and pitch flips sequentially."""

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

from generate_acrobatic_trajectories import MANEUVERS, generate
from stored_ltv_controller import generate_artifact


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
    "--flow-z-mode", "inertial",
    "--flow-z-max-tilt-deg", "35",
    "--flow-z-reacquire-tau-ms", "120",
)


def _rows(path: Path) -> list[dict[str, str]]:
    with path.open(newline="") as stream:
        return list(csv.DictReader(stream))


def _quaternion_error_deg(actual: np.ndarray, reference: np.ndarray) -> np.ndarray:
    dots = np.abs(np.sum(actual * reference, axis=1))
    return np.degrees(2.0 * np.arccos(np.clip(dots, 0.0, 1.0)))


def _reference_at(
    trajectory_rows: list[dict[str, str]], query_time: np.ndarray,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    reference_t = np.asarray([float(row["t"]) for row in trajectory_rows])
    position_source = np.asarray([
        [float(row[key]) for key in ("x", "y", "z")] for row in trajectory_rows
    ])
    quaternion_source = np.asarray([
        [float(row[key]) for key in ("qw", "qx", "qy", "qz")] for row in trajectory_rows
    ])
    position = np.column_stack([
        np.interp(query_time, reference_t, position_source[:, axis]) for axis in range(3)
    ])
    quaternion = np.column_stack([
        np.interp(query_time, reference_t, quaternion_source[:, axis]) for axis in range(4)
    ])
    quaternion /= np.linalg.norm(quaternion, axis=1)[:, None]
    omega = np.column_stack([
        np.interp(query_time, reference_t, [float(row[key]) for row in trajectory_rows])
        for key in ("wx", "wy", "wz")
    ])
    return position, quaternion, omega


def _analyze(run_dir: Path, trajectory_path: Path, axis_index: int) -> tuple[dict[str, Any], dict[str, np.ndarray]]:
    summary = json.loads((run_dir / "summary.json").read_text())
    rows = _rows(run_dir / "closed_loop.csv")
    trajectory_rows = _rows(trajectory_path)
    time = np.asarray([float(row["t"]) for row in rows])
    query_time = np.maximum(0.0, time - float(summary["trajectory_handoff_hold_s"]))
    position = np.asarray([[float(row[key]) for key in ("x", "y", "z")] for row in rows])
    quaternion = np.asarray([
        [float(row[key]) for key in ("quat_w", "quat_x", "quat_y", "quat_z")] for row in rows
    ])
    omega = np.asarray([
        [float(row[key]) for key in ("body_wx_rad_s", "body_wy_rad_s", "body_wz_rad_s")]
        for row in rows
    ])
    reference_position, reference_quaternion, reference_omega = _reference_at(
        trajectory_rows, query_time
    )
    position_error = np.linalg.norm(position - reference_position, axis=1)
    attitude_error = _quaternion_error_deg(quaternion, reference_quaternion)
    achieved_rotation_deg = math.degrees(float(np.trapezoid(omega[:, axis_index], time)))
    requested_rotation_deg = math.degrees(
        float(np.trapezoid(reference_omega[:, axis_index], time))
    )
    final_speed = float(np.linalg.norm([float(rows[-1][key]) for key in ("vx", "vy", "vz")]))
    motor_thrust = np.asarray([
        [float(row[f"motor_{motor}_thrust_n"]) for motor in range(4)] for row in rows
    ])
    maximum_motor_thrust_n = 3.72e-8 * 2900.0**2
    motor_saturation_fraction = float(np.mean(np.any(
        (motor_thrust <= 1.0e-6) | (motor_thrust >= maximum_motor_thrust_n - 1.0e-6), axis=1
    )))
    correction = np.asarray([
        [float(row[f"tinympc_correction_motor_{motor}_thrust_n"] or "nan") for motor in range(4)]
        for row in rows
    ])
    finite_correction = correction[np.isfinite(correction)]
    metrics = {
        "collision": bool(summary["collision"]),
        "reached_goal": bool(summary["reached_goal"]),
        "achieved_rotation_deg": achieved_rotation_deg,
        "requested_rotation_deg": requested_rotation_deg,
        "rotation_error_deg": achieved_rotation_deg - requested_rotation_deg,
        "peak_body_rate_deg_s": math.degrees(float(np.max(np.abs(omega[:, axis_index])))),
        "position_rmse_m": float(np.sqrt(np.mean(position_error**2))),
        "peak_position_error_m": float(np.max(position_error)),
        "peak_attitude_error_deg": float(np.max(attitude_error)),
        "final_attitude_error_deg": float(attitude_error[-1]),
        "minimum_altitude_m": float(np.min(position[:, 2])),
        "maximum_altitude_m": float(np.max(position[:, 2])),
        "final_speed_mps": final_speed,
        "motor_saturation_fraction": motor_saturation_fraction,
        "tinympc_correction_rms_n": (
            float(np.sqrt(np.mean(finite_correction**2))) if finite_correction.size else None
        ),
        "tinympc_correction_peak_n": (
            float(np.max(np.abs(finite_correction))) if finite_correction.size else None
        ),
    }
    metrics["acrobatics_pass"] = bool(
        not metrics["collision"]
        and metrics["minimum_altitude_m"] >= 0.25
        and abs(metrics["rotation_error_deg"]) <= 20.0
        and metrics["final_attitude_error_deg"] <= 15.0
        and metrics["final_speed_mps"] <= 0.35
    )
    return metrics, {
        "time": time,
        "position": position,
        "reference_position": reference_position,
        "rotation_deg": np.degrees(np.r_[0.0, np.cumsum(
            0.5 * np.diff(time) * (omega[:-1, axis_index] + omega[1:, axis_index])
        )]),
        "reference_rotation_deg": np.degrees(np.r_[0.0, np.cumsum(
            0.5 * np.diff(time) * (
                reference_omega[:-1, axis_index] + reference_omega[1:, axis_index]
            )
        )]),
        "attitude_error_deg": attitude_error,
    }


def _plot(out: Path, title: str, data: dict[str, np.ndarray]) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    figure, axes = plt.subplots(2, 2, figsize=(12, 7.5))
    time = data["time"]
    position = data["position"]
    reference = data["reference_position"]
    axes[0, 0].plot(position[:, 0], position[:, 1], label="actual")
    axes[0, 0].plot(reference[:, 0], reference[:, 1], "k--", label="reference")
    axes[0, 0].set(xlabel="x [m]", ylabel="y [m]", title="Top-down")
    axes[0, 0].axis("equal")
    axes[0, 1].plot(time, position[:, 2], label="actual")
    axes[0, 1].plot(time, reference[:, 2], "k--", label="reference")
    axes[0, 1].set(xlabel="time [s]", ylabel="z [m]", title="Ballistic altitude")
    axes[1, 0].plot(time, data["rotation_deg"], label="actual")
    axes[1, 0].plot(time, data["reference_rotation_deg"], "k--", label="reference")
    axes[1, 0].axhline(180.0, color="#d62728", alpha=0.35, label="old chart singularity")
    axes[1, 0].set(xlabel="time [s]", ylabel="rotation [deg]", title="Integrated body rotation")
    axes[1, 1].plot(time, data["attitude_error_deg"], label="quaternion error")
    axes[1, 1].set(xlabel="time [s]", ylabel="quaternion error [deg]", title="Chart-safe attitude error")
    for axis in axes.ravel():
        axis.grid(True, alpha=0.25)
        axis.legend(fontsize=8)
    figure.suptitle(title)
    figure.tight_layout()
    out.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(out, dpi=170)
    plt.close(figure)


def _plot_flow_altitude(run_dir: Path, title: str) -> None:
    rows = _rows(run_dir / "flow_altitude.csv")
    if not rows:
        return
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    time = np.asarray([float(row["t"]) for row in rows])
    true_z = np.asarray([float(row["delayed_true_z_m"]) for row in rows])
    estimate_z = np.asarray([float(row["estimated_z_m"]) for row in rows])
    raw_range = np.asarray([float(row["raw_slant_range_m"]) for row in rows])
    cosine = np.asarray([float(row["body_z_world_z_cosine"]) for row in rows])
    valid = np.asarray([bool(int(row["range_valid"])) for row in rows])
    figure, axes = plt.subplots(2, 1, figsize=(10, 6.5), sharex=True)
    axes[0].plot(time, true_z, "k--", label="true delayed z")
    axes[0].plot(time, estimate_z, label="controller z estimate")
    axes[0].plot(time, raw_range, color="#d62728", alpha=0.45, label="raw slant range")
    axes[0].set(ylabel="altitude / range [m]", title="Flow-deck altitude handling")
    axes[1].plot(time, cosine, label="body-z · world-z")
    axes[1].axhline(math.cos(math.radians(35.0)), color="k", linestyle="--", label="35° validity threshold")
    axes[1].fill_between(time, -1.05, 1.05, where=~valid, color="#d62728", alpha=0.12, label="range rejected")
    axes[1].set(xlabel="time [s]", ylabel="orientation cosine", ylim=(-1.05, 1.05))
    for axis in axes:
        axis.grid(True, alpha=0.25)
        axis.legend(fontsize=8)
    figure.suptitle(title)
    figure.tight_layout()
    figure.savefig(run_dir / "flow_altitude.png", dpi=170)
    plt.close(figure)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--out", type=Path, default=Path("sim_runs/tier3_acrobatics"))
    parser.add_argument("--trajectory-dir", type=Path, default=Path("sim/trajectories/acrobatics"))
    parser.add_argument("--profile", choices=("ideal", "realistic-v1"), default="realistic-v1")
    parser.add_argument(
        "--controller", choices=("tinympc-acro", "tinympc-ltv", "admm", "geometric"), default="tinympc-acro",
        help=("tinympc-acro is the new firmware solver path; admm is the unmodified "
              "TinyMPC negative control; tinympc-ltv consumes fully offline stored "
              "linearizations/caches; geometric is a comparison controller"),
    )
    parser.add_argument("--maneuver", action="append", choices=[item.name for item in MANEUVERS])
    parser.add_argument("--native-retries", type=int, default=3)
    parser.add_argument(
        "--flow-z-mode", choices=("ideal", "raw-range", "freeze", "inertial"),
        help="override the profile's Flow-deck altitude policy",
    )
    parser.add_argument("--overwrite", action="store_true")
    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)
    selected = set(args.maneuver or [item.name for item in MANEUVERS])
    results: list[dict[str, Any]] = []
    for maneuver in MANEUVERS:
        if maneuver.name not in selected:
            continue
        trajectory_path = args.trajectory_dir / f"{maneuver.name}.csv"
        trajectory_info = generate(maneuver, trajectory_path)
        run_dir = args.out / args.controller / args.profile / maneuver.name
        command = [
            sys.executable, str(RUNNER), "--out", str(run_dir),
            "--duration", str(float(trajectory_info["duration_s"]) + 1.5),
            "--control-mode", args.controller, "--trajectory-file", str(trajectory_path),
            "--progress-interval-s", "0", "--plot", "--overwrite",
        ]
        if args.controller == "tinympc-ltv":
            artifact = args.out / "stored_ltv_models" / f"{maneuver.name}.npz"
            generate_artifact(trajectory_path, artifact)
            command.extend(("--stored-ltv-model", str(artifact)))
        if args.profile == "realistic-v1":
            command.extend(REALISTIC_V1)
        if args.flow_z_mode is not None:
            command.extend(("--flow-z-mode", args.flow_z_mode))
        completed = subprocess.run(command, cwd=APP_ROOT, text=True, capture_output=True)
        retry = 0
        while completed.returncode < 0 and retry < args.native_retries:
            retry += 1
            completed = subprocess.run(command, cwd=APP_ROOT, text=True, capture_output=True)
        if completed.returncode != 0:
            raise RuntimeError(
                f"{maneuver.name} failed ({completed.returncode}):\n"
                f"{(completed.stderr or completed.stdout)[-3000:]}"
            )
        axis_index = int(np.argmax(np.abs(maneuver.axis)))
        metrics, plot_data = _analyze(run_dir, trajectory_path, axis_index)
        row = {
            "maneuver": maneuver.name,
            "description": maneuver.description,
            "profile": args.profile,
            "controller": args.controller,
            **metrics,
            "run_dir": str(run_dir),
        }
        results.append(row)
        _plot(run_dir / "acrobatics.png", f"{maneuver.name} — {args.profile}", plot_data)
        _plot_flow_altitude(
            run_dir,
            f"{maneuver.name} — {args.profile} — {args.flow_z_mode or ('inertial' if args.profile == 'realistic-v1' else 'ideal')}",
        )
        print(
            f"{'PASS' if metrics['acrobatics_pass'] else 'FAIL'} {maneuver.name:20s} "
            f"rotation={metrics['achieved_rotation_deg']:.1f}deg "
            f"att_err={metrics['peak_attitude_error_deg']:.1f}deg "
            f"pos_rmse={metrics['position_rmse_m']:.3f}m "
            f"minz={metrics['minimum_altitude_m']:.3f}m"
        )
    summary = {
        "profile": args.profile,
        "controller": args.controller,
        "runs": len(results),
        "passes": sum(bool(row["acrobatics_pass"]) for row in results),
        "results": results,
    }
    (args.out / f"summary_{args.controller}_{args.profile}.json").write_text(
        json.dumps(summary, indent=2) + "\n"
    )
    print(json.dumps({key: summary[key] for key in ("profile", "runs", "passes")}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
