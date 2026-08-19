#!/usr/bin/env python3
"""Plot sustained-turn divergence for world and firmware-local MPC frames."""

from __future__ import annotations

import argparse
import csv
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


def read_rows(path: Path) -> list[dict[str, str]]:
    with path.open(newline="") as stream:
        return list(csv.DictReader(stream))


def reference(path: Path) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    rows = read_rows(path)
    time = np.asarray([float(row["t"]) for row in rows])
    position = np.asarray([[float(row[key]) for key in ("x", "y", "z")] for row in rows])
    quat = np.asarray([[float(row[key]) for key in ("qw", "qx", "qy", "qz")] for row in rows])
    qw, qx, qy, qz = quat.T
    yaw = np.unwrap(np.arctan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz)))
    return time, position, yaw


def run(path: Path, ref_time: np.ndarray, ref_position: np.ndarray, ref_yaw: np.ndarray) -> dict[str, np.ndarray]:
    rows = read_rows(path / "closed_loop.csv")
    time = np.asarray([float(row["t"]) for row in rows])
    position = np.asarray([[float(row[key]) for key in ("next_x", "next_y", "next_z")] for row in rows])
    yaw = np.unwrap(np.asarray([float(row["yaw_rad"]) for row in rows]))
    reference_position = np.column_stack([
        np.interp(time, ref_time, ref_position[:, axis]) for axis in range(3)
    ])
    return {
        "time": time,
        "position": position,
        "reference_yaw_deg": np.degrees(np.interp(time, ref_time, ref_yaw)),
        "actual_yaw_deg": np.degrees(yaw),
        "xy_error": np.linalg.norm(position[:, :2] - reference_position[:, :2], axis=1),
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--trajectory", type=Path, required=True)
    parser.add_argument("--world-run", type=Path, required=True)
    parser.add_argument("--local-run", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()

    ref_time, ref_position, ref_yaw = reference(args.trajectory)
    world = run(args.world_run, ref_time, ref_position, ref_yaw)
    local = run(args.local_run, ref_time, ref_position, ref_yaw)

    figure, axes = plt.subplots(1, 3, figsize=(16, 4.8))
    axes[0].plot(ref_position[:, 0], ref_position[:, 1], "--", color="#2ca66f", label="reference")
    axes[0].plot(world["position"][:, 0], world["position"][:, 1], color="#d1495b", label="old world frame")
    axes[0].plot(local["position"][:, 0], local["position"][:, 1], color="#2878b5", label="firmware-local")
    axes[0].set_aspect("equal", adjustable="box")
    axes[0].set_xlabel("x [m]"); axes[0].set_ylabel("y [m]"); axes[0].set_title("Top-down path")
    axes[0].legend()

    for values, color, label in ((world, "#d1495b", "old world frame"), (local, "#2878b5", "firmware-local")):
        axes[1].plot(values["reference_yaw_deg"], values["xy_error"], color=color, label=label)
        axes[2].plot(values["reference_yaw_deg"], values["actual_yaw_deg"], color=color, label=label)
    for axis in axes[1:]:
        axis.axvline(90.0, color="black", linestyle=":", linewidth=1.2, label="90° reference")
        axis.grid(alpha=0.25)
    axes[1].set_xlabel("reference yaw [deg]"); axes[1].set_ylabel("horizontal error [m]"); axes[1].set_title("Error growth")
    axes[2].plot([0, 360], [0, 360], "--", color="#2ca66f", label="ideal tracking")
    axes[2].set_xlabel("reference yaw [deg]"); axes[2].set_ylabel("actual unwrapped yaw [deg]"); axes[2].set_title("Yaw divergence")
    axes[1].legend(); axes[2].legend()
    figure.suptitle("Original hover matrices: sustained 15° bank, firmware reference generation")
    figure.tight_layout()
    args.out.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(args.out, dpi=180)
    print(f"wrote {args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
