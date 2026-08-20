#!/usr/bin/env python3
"""Generate a firmware-format hover trajectory with a smooth yaw sweep."""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path

import numpy as np


def smoothstep5(value: np.ndarray) -> np.ndarray:
    value = np.clip(value, 0.0, 1.0)
    return 10.0 * value**3 - 15.0 * value**4 + 6.0 * value**5


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--yaw-deg", type=float, required=True)
    parser.add_argument("--slew-duration", type=float, default=2.0)
    parser.add_argument("--hold-duration", type=float, default=1.0)
    parser.add_argument("--z", type=float, default=1.1)
    args = parser.parse_args()

    dt = 0.02
    total = float(args.slew_duration) + float(args.hold_duration)
    time = np.arange(0.0, total + 0.5 * dt, dt)
    phase = time / float(args.slew_duration)
    yaw = math.radians(float(args.yaw_deg)) * smoothstep5(phase)
    yaw_rate = np.gradient(yaw, dt, edge_order=2)

    args.out.parent.mkdir(parents=True, exist_ok=True)
    fields = ["t", "x", "y", "z", "vx", "vy", "vz", "qw", "qx", "qy", "qz", "wx", "wy", "wz"]
    with args.out.open("w", newline="") as stream:
        writer = csv.writer(stream, lineterminator="\n")
        writer.writerow(fields)
        for index, stamp in enumerate(time):
            half = 0.5 * yaw[index]
            writer.writerow([
                f"{stamp:.9f}", 0.0, 0.0, float(args.z), 0.0, 0.0, 0.0,
                math.cos(half), 0.0, 0.0, math.sin(half),
                0.0, 0.0, yaw_rate[index],
            ])
    print(f"wrote {args.out}: yaw={args.yaw_deg:g} deg over {args.slew_duration:g} s")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
