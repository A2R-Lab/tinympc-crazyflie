#!/usr/bin/env python3
"""Plot the demo-2 figure-eight reference and obstacle-avoidance geometry."""

from __future__ import annotations

import argparse
import re
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Circle


FIGURE8_WAYPOINTS = 892
OBSTACLES = np.array([[0.0, 0.92], [0.0, -0.92]])
PHYSICAL_RADIUS = 0.18
SAFETY_MARGIN = 0.10
CONSTRAINT_RADIUS = PHYSICAL_RADIUS + SAFETY_MARGIN
ACTIVATION_MARGIN = 0.12

# Sparse positions printed while each obstacle constraint was active. The
# firmware did not print positions on the unobserved portions between them.
OBSERVED_TOP = np.array(
    [
        [-0.31, 0.47],
        [-0.28, 0.53],
        [-0.31, 0.64],
        [-0.33, 0.79],
        [-0.27, 0.98],
        [0.16, 1.23],
        [0.46, 1.18],
    ]
)
OBSERVED_BOTTOM = np.array(
    [
        [-0.32, -0.48],
        [-0.28, -0.54],
        [-0.31, -0.67],
        [-0.33, -0.82],
        [-0.25, -0.99],
        [0.16, -1.22],
        [0.48, -1.16],
    ]
)


def load_reference(header: Path) -> np.ndarray:
    text = header.read_text(encoding="utf-8")
    match = re.search(r"X_ref_data\[\]\s*=\s*\{(.*?)\};", text, re.DOTALL)
    if match is None:
        raise RuntimeError(f"Could not locate X_ref_data in {header}")

    body = re.sub(r"//.*", "", match.group(1))
    values = np.fromstring(body.replace("\n", " "), sep=",")
    if values.size % 3 != 0:
        raise RuntimeError(f"Trajectory table has {values.size} values, not XYZ triplets")

    points = values.reshape((-1, 3))
    if len(points) < FIGURE8_WAYPOINTS:
        raise RuntimeError(f"Trajectory has only {len(points)} samples")
    return points[:FIGURE8_WAYPOINTS, :2]


def load_trace(log_file: Path) -> np.ndarray:
    pattern = re.compile(
        r"TRACE:\s+idx=\d+\s+pos=\("
        r"([-+\d.eE]+),([-+\d.eE]+),([-+\d.eE]+)\)"
    )
    points = []
    for line in log_file.read_text(encoding="utf-8").splitlines():
        match = pattern.search(line)
        if match is not None:
            points.append(tuple(float(value) for value in match.groups()))
    if not points:
        raise RuntimeError(f"No TRACE positions found in {log_file}")
    return np.asarray(points)


def add_direction_arrow(ax: plt.Axes, path: np.ndarray, index: int) -> None:
    start = path[index]
    end = path[min(index + 8, len(path) - 1)]
    ax.annotate(
        "",
        xy=end,
        xytext=start,
        arrowprops={"arrowstyle": "-|>", "color": "#2e7d32", "lw": 1.8},
        zorder=6,
    )


def main() -> None:
    app_dir = Path(__file__).resolve().parent
    default_header = app_dir.parent / "controller_tinympc_eigen" / "src" / "traj_fig8_single.h"
    default_output = app_dir / "demo2_reference_vs_actual.png"

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--header", type=Path, default=default_header)
    parser.add_argument("--log", type=Path, help="Console log containing full-flight TRACE lines")
    parser.add_argument("--output", type=Path, default=default_output)
    args = parser.parse_args()

    reference = load_reference(args.header)
    if args.log is not None:
        observed_segments = (load_trace(args.log)[:, :2],)
        actual_label = "Actual path from 10 Hz TRACE log"
        trace_description = f"Orange: {len(observed_segments[0])} measured samples from {args.log.name}"
    else:
        observed_segments = (OBSERVED_TOP, OBSERVED_BOTTOM)
        actual_label = "Previous actual path samples (partial)"
        trace_description = "Orange: previous sparse samples; pass --log for the complete new flight"

    fig, ax = plt.subplots(figsize=(8.5, 10.5), constrained_layout=True)
    ax.plot(
        reference[:, 0],
        reference[:, 1],
        color="#2e7d32",
        lw=2.6,
        label="Reference trajectory (perfect figure eight)",
        zorder=4,
    )

    for index, center in enumerate(OBSTACLES, start=1):
        ax.add_patch(
            Circle(
                center,
                CONSTRAINT_RADIUS + ACTIVATION_MARGIN,
                fill=False,
                ls=":",
                lw=1.8,
                ec="#c62828",
                alpha=0.65,
                zorder=1,
            )
        )
        ax.add_patch(
            Circle(
                center,
                CONSTRAINT_RADIUS,
                fill=False,
                ls="--",
                ec="#b71c1c",
                alpha=0.85,
                lw=1.8,
                zorder=2,
            )
        )
        ax.add_patch(
            Circle(
                center,
                PHYSICAL_RADIUS,
                fc="#ef5350",
                ec="#b71c1c",
                alpha=0.30,
                lw=2.0,
                zorder=3,
            )
        )
        ax.plot(center[0], center[1], "+", color="#b71c1c", ms=10, mew=2)
        ax.text(
            center[0] + 0.04,
            center[1],
            f"Obstacle {index}\ncenter=({center[0]:.2f}, {center[1]:.2f})\n"
            f"physical r={PHYSICAL_RADIUS:.2f} m\nconstraint r={CONSTRAINT_RADIUS:.2f} m",
            fontsize=9,
            va="center",
            color="#7f0000",
        )

    for segment_index, observed in enumerate(observed_segments):
        ax.plot(
            observed[:, 0],
            observed[:, 1],
            "x-",
            color="#ef6c00",
            lw=2.2,
            ms=8,
            mew=2,
            label=actual_label if segment_index == 0 else None,
            zorder=7,
        )
    all_observed = np.vstack(observed_segments)
    closest_distances = []
    for center in OBSTACLES:
        distances = np.linalg.norm(all_observed - center, axis=1)
        closest_index = int(np.argmin(distances))
        closest = all_observed[closest_index]
        closest_distances.append(float(distances[closest_index]))
        ax.plot(
            closest[0],
            closest[1],
            marker="o",
            ms=13,
            mfc="none",
            mec="#bf360c",
            mew=2.2,
            zorder=8,
        )
    ax.plot(0.0, 0.0, marker="*", color="#00897b", ms=15, label="Start, return, and landing")

    for arrow_index in (55, 260, 500, 705):
        add_direction_arrow(ax, reference, arrow_index)

    ax.set_title("demo-2: Reference versus measured flight path", fontsize=14)
    ax.set_xlabel("x position [m]")
    ax.set_ylabel("y position [m]")
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlim(-0.82, 0.82)
    ax.set_ylim(-1.48, 1.48)
    ax.grid(True, alpha=0.25)
    ax.legend(loc="lower left", fontsize=9, framealpha=0.95)
    closest_description = (
        f"Closest measured: obs1={closest_distances[0]:.3f} m, "
        f"obs2={closest_distances[1]:.3f} m | physical={PHYSICAL_RADIUS:.2f} m, "
        f"constraint={CONSTRAINT_RADIUS:.2f} m"
    )
    ax.text(
        0.02,
        0.985,
        "Altitude: 0.50 m  |  Reference duration: 17.84 s  |  Direction shown by green arrows\n"
        f"Filled red: physical obstacle ({PHYSICAL_RADIUS:.2f} m) | dashed: safety constraint "
        f"({CONSTRAINT_RADIUS:.2f} m) | dotted: activation ({CONSTRAINT_RADIUS + ACTIVATION_MARGIN:.2f} m)\n"
        f"{trace_description}\n{closest_description}",
        transform=ax.transAxes,
        va="top",
        fontsize=9,
        bbox={"boxstyle": "round,pad=0.35", "fc": "white", "ec": "#bdbdbd", "alpha": 0.92},
    )

    args.output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.output, dpi=190)
    plt.close(fig)
    print(args.output)


if __name__ == "__main__":
    main()
