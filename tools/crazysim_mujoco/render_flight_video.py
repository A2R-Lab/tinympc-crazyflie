#!/usr/bin/env python3
"""Render a compact 3-D replay from a CrazySim state.csv telemetry file."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib.animation import FFMpegWriter
import numpy as np


def read_state(path: Path) -> dict[str, np.ndarray]:
    with path.open(newline="") as stream:
        rows = list(csv.DictReader(stream))
    if not rows:
        raise ValueError(f"no telemetry rows in {path}")
    required = ("time_s", "x_m", "y_m", "z_m", "qw", "qx", "qy", "qz")
    missing = [name for name in required if name not in rows[0]]
    if missing:
        raise ValueError(f"missing columns in {path}: {', '.join(missing)}")
    return {
        name: np.asarray([float(row[name]) for row in rows], dtype=float)
        for name in required
    }


def rotate_vectors(quaternion: np.ndarray, vectors: np.ndarray) -> np.ndarray:
    """Rotate body vectors by a scalar-first unit quaternion."""
    quaternion = quaternion / max(np.linalg.norm(quaternion), 1.0e-12)
    w = quaternion[0]
    q = quaternion[1:]
    return vectors + 2.0 * np.cross(q, np.cross(q, vectors) + w * vectors)


def course_geometry(path: Path | None) -> tuple[list[dict], list[dict]]:
    if path is None:
        return [], []
    manifest = json.loads(path.read_text())
    return manifest.get("obstacles", []), manifest.get("gates", [])


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--csv", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--fps", type=float, default=24.0)
    parser.add_argument("--playback-speed", type=float, default=2.0)
    parser.add_argument("--launch-time", type=float, default=0.0)
    parser.add_argument("--course", type=Path)
    args = parser.parse_args()
    if args.fps <= 0.0 or args.playback_speed <= 0.0:
        parser.error("fps and playback speed must be positive")

    state = read_state(args.csv)
    t, x, y, z = (state[name] for name in ("time_s", "x_m", "y_m", "z_m"))
    frame_times = np.arange(t[0], t[-1] + 0.5 / args.fps,
                            args.playback_speed / args.fps)
    frame_indices = np.clip(np.searchsorted(t, frame_times), 0, len(t) - 1)
    obstacles, gates = course_geometry(args.course)

    margin = 0.25
    x_limits = (float(np.min(x) - margin), float(np.max(x) + margin))
    y_limits = (float(np.min(y) - margin), float(np.max(y) + margin))
    z_limits = (min(0.0, float(np.min(z) - margin)),
                float(np.max(z) + margin))
    if x_limits[1] - x_limits[0] < 0.5:
        center = 0.5 * sum(x_limits); x_limits = (center - 0.25, center + 0.25)
    if y_limits[1] - y_limits[0] < 0.5:
        center = 0.5 * sum(y_limits); y_limits = (center - 0.25, center + 0.25)

    fig = plt.figure(figsize=(9, 7))
    axis = fig.add_subplot(111, projection="3d")
    axis.set(xlim=x_limits, ylim=y_limits, zlim=z_limits,
             xlabel="x [m]", ylabel="y [m]", zlabel="z [m]")
    axis.set_box_aspect((x_limits[1] - x_limits[0],
                         y_limits[1] - y_limits[0],
                         z_limits[1] - z_limits[0]))
    axis.plot(x, y, z, color="0.75", linewidth=1.0, label="full flight path")
    for obstacle in obstacles:
        center = obstacle.get("center", [0.0, 0.0])
        height = float(obstacle.get("height", 0.0))
        axis.scatter([center[0]], [center[1]], [0.5 * height], marker="s",
                     s=100, color="#c62828", alpha=0.6)
    for gate in gates:
        center = gate.get("center", [0.0, 0.0, 0.0])
        axis.scatter([center[0]], [center[1]], [center[2]], marker="o",
                     s=80, facecolors="none", edgecolors="#ef6c00")
    flown, = axis.plot([], [], [], color="#1565c0", linewidth=2.2,
                       label="flown")
    arm_a, = axis.plot([], [], [], color="#d32f2f", linewidth=4)
    arm_b, = axis.plot([], [], [], color="#212121", linewidth=4)
    time_text = axis.text2D(0.03, 0.95, "", transform=axis.transAxes)
    axis.legend(loc="upper right")

    metadata = {"title": "CrazySim TinyMPC flight replay", "artist": "TinyMPC"}
    writer = FFMpegWriter(fps=args.fps, metadata=metadata, bitrate=3000)
    args.out.parent.mkdir(parents=True, exist_ok=True)
    arm_length = 0.09
    body_arms = np.asarray([
        [-arm_length, -arm_length, 0.0], [arm_length, arm_length, 0.0],
        [-arm_length, arm_length, 0.0], [arm_length, -arm_length, 0.0],
    ])
    with writer.saving(fig, str(args.out), dpi=140):
        for index in frame_indices:
            position = np.asarray([x[index], y[index], z[index]])
            quaternion = np.asarray([
                state["qw"][index], state["qx"][index],
                state["qy"][index], state["qz"][index],
            ])
            arms = rotate_vectors(quaternion, body_arms) + position
            flown.set_data_3d(x[:index + 1], y[:index + 1], z[:index + 1])
            arm_a.set_data_3d(arms[:2, 0], arms[:2, 1], arms[:2, 2])
            arm_b.set_data_3d(arms[2:, 0], arms[2:, 1], arms[2:, 2])
            phase = "staged" if t[index] < args.launch_time else "airborne"
            time_text.set_text(f"t = {t[index]:.2f} s   {phase}")
            writer.grab_frame()
    plt.close(fig)
    print(f"Rendered {len(frame_indices)} frames to {args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
