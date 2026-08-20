#!/usr/bin/env python3
"""Summarize and plot a CrazySim TinyMPC ground-truth run."""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


def load_numeric_csv(path: Path) -> dict[str, np.ndarray]:
    with path.open(newline="") as stream:
        reader = csv.DictReader(stream)
        rows = list(reader)
    if not rows:
        raise ValueError(f"No samples in {path}")
    return {
        name: np.asarray([float(row[name]) for row in rows], dtype=float)
        for name in reader.fieldnames or []
    }


def quaternion_to_euler_deg(qw, qx, qy, qz):
    sinr_cosp = 2.0 * (qw * qx + qy * qz)
    cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = np.arctan2(sinr_cosp, cosr_cosp)
    sinp = np.clip(2.0 * (qw * qy - qz * qx), -1.0, 1.0)
    pitch = np.arcsin(sinp)
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = np.arctan2(siny_cosp, cosy_cosp)
    return np.rad2deg(roll), np.rad2deg(pitch), np.rad2deg(yaw)


def build_summary(
    data: dict[str, np.ndarray], launch_time: float,
    reference: dict[str, np.ndarray] | None = None,
) -> dict[str, object]:
    t = data["time_s"]
    x, y, z = data["x_m"], data["y_m"], data["z_m"]
    q_abs_w = np.clip(np.abs(data["qw"]), 0.0, 1.0)
    attitude_geodesic = np.rad2deg(2.0 * np.arccos(q_abs_w))
    rpm = np.column_stack([data[f"rpm_{i}"] for i in range(1, 5)])
    after_launch = t >= launch_time + 0.05
    crash_mask = after_launch & (data["contacts"] > 0.0) & (z < 0.15)
    crash_indices = np.flatnonzero(crash_mask)
    first_crash = float(t[crash_indices[0]]) if crash_indices.size else None
    max_rpm = float(np.max(rpm))
    saturation_threshold = 0.995 * max_rpm if max_rpm > 0.0 else math.inf
    summary: dict[str, object] = {
        "samples": int(t.size),
        "duration_s": float(t[-1] - t[0]),
        "launch_time_s": launch_time,
        "crashed": bool(crash_indices.size),
        "first_crash_time_s": first_crash,
        "first_crash_after_launch_s": (
            first_crash - launch_time if first_crash is not None else None
        ),
        "altitude_min_m": float(np.min(z)),
        "altitude_max_m": float(np.max(z)),
        "altitude_final_m": float(z[-1]),
        "horizontal_displacement_max_m": float(np.max(np.hypot(x - x[0], y - y[0]))),
        "horizontal_displacement_final_m": float(np.hypot(x[-1] - x[0], y[-1] - y[0])),
        "horizontal_speed_final_mps": float(np.hypot(
            data["vx_mps"][-1], data["vy_mps"][-1]
        )),
        "attitude_geodesic_max_deg": float(np.max(attitude_geodesic)),
        "angular_rate_max_rad_s": float(np.max(np.sqrt(
            data["wx_radps"] ** 2 + data["wy_radps"] ** 2 + data["wz_radps"] ** 2
        ))),
        "rpm_max": max_rpm,
        "motor_saturation_fraction": float(np.mean(rpm >= saturation_threshold)),
        "contact_count_max": int(np.max(data["contacts"])),
    }
    if reference is not None:
        axes = ("x", "y", "z")
        reference_rates = np.column_stack(
            [reference[f"w{axis}"] for axis in axes]
        )
        axis_index = int(np.argmax(np.max(np.abs(reference_rates), axis=0)))
        actual_rate = data[f"w{axes[axis_index]}_radps"]
        launched = t >= launch_time
        summary.update({
            "maneuver_axis": axes[axis_index],
            "integrated_rotation_deg": float(np.rad2deg(np.trapezoid(
                actual_rate[launched], t[launched]
            ))),
            "reference_integrated_rotation_deg": float(np.rad2deg(np.trapezoid(
                reference_rates[:, axis_index], reference["t"]
            ))),
        })
    return summary


def plot_run(data, reference, launch_time: float, summary, output: Path):
    t = data["time_s"]
    roll, pitch, yaw = quaternion_to_euler_deg(
        data["qw"], data["qx"], data["qy"], data["qz"]
    )
    rpm = np.column_stack([data[f"rpm_{i}"] for i in range(1, 5)])
    crash_time = summary["first_crash_time_s"]
    launch_index = min(int(np.searchsorted(t, launch_time)), len(t) - 1)
    anchored_reference = None
    if reference is not None:
        # Firmware translates the stored position reference to the measured
        # handoff pose. Mirror that transformation in the plot so a non-default
        # spawn altitude is not shown as a tracking offset.
        anchored_reference = dict(reference)
        for axis, data_name in (("x", "x_m"), ("y", "y_m"), ("z", "z_m")):
            anchored_reference[axis] = (
                reference[axis] - reference[axis][0] + data[data_name][launch_index]
            )

    fig, axes = plt.subplots(2, 2, figsize=(13, 9), constrained_layout=True)
    ax = axes[0, 0]
    if anchored_reference is not None:
        ax.plot(anchored_reference["x"], anchored_reference["y"], "--",
                color="0.55", label="anchored reference")
    ax.plot(data["x_m"], data["y_m"], color="#1565c0", label="MuJoCo ground truth")
    ax.scatter(data["x_m"][0], data["y_m"][0], marker="o", color="#2e7d32", label="start")
    if crash_time is not None:
        idx = int(np.searchsorted(t, crash_time))
        ax.scatter(data["x_m"][idx], data["y_m"][idx], marker="X", s=100,
                   color="#c62828", label="ground contact")
    ax.set(title="Top-down path", xlabel="x [m]", ylabel="y [m]")
    ax.axis("equal")
    ax.grid(alpha=0.25)
    ax.legend(loc="best")

    ax = axes[0, 1]
    if anchored_reference is not None:
        ax.plot(anchored_reference["t"] + launch_time, anchored_reference["z"],
                "--", color="0.55", label="anchored reference z")
    ax.plot(t, data["z_m"], color="#1565c0", label="actual z")
    ax.axvline(launch_time, color="#2e7d32", linestyle=":", label="maneuver handoff")
    if crash_time is not None:
        ax.axvline(crash_time, color="#c62828", linestyle="--", label="crash")
    ax.set(title="Altitude", xlabel="simulation time [s]", ylabel="z [m]")
    ax.grid(alpha=0.25)
    ax.legend(loc="best")

    ax = axes[1, 0]
    ax.plot(t, roll, label="roll")
    ax.plot(t, pitch, label="pitch")
    ax.plot(t, yaw, label="yaw")
    ax.axvline(launch_time, color="#2e7d32", linestyle=":")
    if crash_time is not None:
        ax.axvline(crash_time, color="#c62828", linestyle="--")
    ax.set(title="Ground-truth attitude (Euler view)", xlabel="simulation time [s]",
           ylabel="angle [deg]", ylim=(-190, 190))
    ax.grid(alpha=0.25)
    ax.legend(loc="best")

    ax = axes[1, 1]
    for motor in range(4):
        ax.plot(t, rpm[:, motor], label=f"motor {motor + 1}")
    ax.axvline(launch_time, color="#2e7d32", linestyle=":")
    if crash_time is not None:
        ax.axvline(crash_time, color="#c62828", linestyle="--")
    ax.set(title="Motor speed", xlabel="simulation time [s]", ylabel="RPM")
    ax.grid(alpha=0.25)
    ax.legend(loc="best", ncol=2)

    status = "CRASH" if summary["crashed"] else "NO GROUND CONTACT"
    fig.suptitle(f"CrazySim/MuJoCo exact-firmware validation — {status}", fontsize=15)
    fig.savefig(output, dpi=160)
    plt.close(fig)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--csv", required=True, type=Path)
    parser.add_argument("--out", required=True, type=Path)
    parser.add_argument("--reference", type=Path)
    parser.add_argument("--launch-time", type=float, default=4.0)
    args = parser.parse_args()

    args.out.mkdir(parents=True, exist_ok=True)
    data = load_numeric_csv(args.csv)
    reference = load_numeric_csv(args.reference) if args.reference else None
    launch_time = args.launch_time
    if "airborne" in data:
        airborne_indices = np.flatnonzero(data["airborne"] > 0.5)
        if airborne_indices.size:
            launch_time = float(data["time_s"][airborne_indices[0]])
    summary = build_summary(data, launch_time, reference)
    (args.out / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    plot_run(data, reference, launch_time, summary, args.out / "validation.png")
    print(json.dumps(summary, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
