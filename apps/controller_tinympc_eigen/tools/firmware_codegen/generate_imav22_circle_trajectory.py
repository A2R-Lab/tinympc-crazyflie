#!/usr/bin/env python3
"""Generate the exact circular nominal route used by the IMAV22 ESPNet task."""

from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path

from generate_crazyflie_trajectory import generate_trajectory, render_header


RADIUS_M = 3.6
SPEED_MPS = 0.8
HEIGHT_M = 1.0
SAMPLE_RATE_HZ = 50
DURATION_S = 2.0 * math.pi * RADIUS_M / SPEED_MPS


def imav22_circle_path(time_s: float, duration_s: float):
    # This is the training circle after its -pi/2 rotation: local start is the
    # arena entrance, local center is (+R, 0), and travel is clockwise.
    angular_rate = 2.0 * math.pi / duration_s
    phase = angular_rate * time_s
    return (
        (RADIUS_M * (1.0 - math.cos(phase)), -RADIUS_M * math.sin(phase), 0.0),
        (RADIUS_M * angular_rate * math.sin(phase),
         -RADIUS_M * angular_rate * math.cos(phase), 0.0),
        (RADIUS_M * angular_rate**2 * math.cos(phase),
         RADIUS_M * angular_rate**2 * math.sin(phase), 0.0),
    )


def main() -> int:
    app = Path(__file__).resolve().parents[2]
    output = app / "src/trajectories/50hz/traj_imav22_circle_50hz.h"
    trajectory = generate_trajectory(
        "imav22_circle", SAMPLE_RATE_HZ, DURATION_S, HEIGHT_M, 0.0,
        True, True, imav22_circle_path,
    )
    rendered = render_header(trajectory).replace(
        "#define TRAJECTORY_TANGENT_HEADING 1\n",
        "#define TRAJECTORY_TANGENT_HEADING 1\n"
        "#define TRAJECTORY_TURN_DIRECTION (-1)\n",
    )
    output.write_text(rendered)
    digest = hashlib.sha256(output.read_bytes()).hexdigest()
    output.with_suffix(".provenance.json").write_text(json.dumps({
        "format": "tinympc-imav22-circle-trajectory-v1",
        "source_task": "crazyflie_espnetv2_imav22",
        "trajectory_family": "circle",
        "radius_m": RADIUS_M,
        "nominal_speed_mps": SPEED_MPS,
        "rotation_rad": -math.pi / 2.0,
        "height_m": HEIGHT_M,
        "sample_rate_hz": SAMPLE_RATE_HZ,
        "duration_s": trajectory.duration_s,
        "world_spawn_xy_m": [-RADIUS_M, 0.0],
        "world_circle_center_xy_m": [0.0, 0.0],
        "header_sha256": digest,
    }, indent=2) + "\n")
    print(output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
