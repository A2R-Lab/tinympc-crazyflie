#!/usr/bin/env python3
"""Verify discrete-waypoint figure-eight Crazysim evidence."""

from __future__ import annotations

import argparse
import csv
import json
import math
import re
from pathlib import Path


WAYPOINT_RE = re.compile(
    r"Waypoint reached index=(\d+)/(\d+) next_knot=(\d+) complete=(\d+)"
)
SOLVER_FAILURE_RE = re.compile(
    r"\b(?:nan|inf|assert|hardfault|solver (?:fail|error)|segmentation fault)\b",
    re.IGNORECASE,
)


def load_csv(path: Path) -> list[dict[str, str]]:
    with path.open(newline="") as stream:
        return list(csv.DictReader(stream))


def first_index(values: list[bool], start: int = 0) -> int | None:
    return next((index for index in range(start, len(values)) if values[index]), None)


def percentile(values: list[float], probability: float) -> float:
    ordered = sorted(values)
    position = probability * (len(ordered) - 1)
    lower = int(math.floor(position))
    upper = int(math.ceil(position))
    alpha = position - lower
    return (1.0 - alpha) * ordered[lower] + alpha * ordered[upper]


def yaw_from_quaternion(row: dict[str, str]) -> float:
    qw, qx, qy, qz = (float(row[name]) for name in ("qw", "qx", "qy", "qz"))
    return math.atan2(
        2.0 * (qw * qz + qx * qy),
        1.0 - 2.0 * (qy * qy + qz * qz),
    )


def verify_run(
    run: Path, radius_m: float, minimum_yaw_speed_mps: float,
    maximum_median_yaw_error_deg: float, maximum_p90_yaw_error_deg: float,
) -> dict[str, object]:
    rows = load_csv(run / "state.csv")
    firmware = (run / "firmware.log").read_text(errors="replace")
    summary = json.loads((run / "summary.json").read_text())
    events = [tuple(map(int, match.groups())) for match in WAYPOINT_RE.finditer(firmware)]
    route_count = events[0][1] if events else 0
    reached_indices = [event[0] for event in events]
    waypoint_sequence_ok = bool(
        route_count >= 5
        and reached_indices == list(range(route_count))
        and events[-1][3] == 1
    )

    airborne = [int(row["airborne"]) != 0 for row in rows]
    launch = first_index(airborne)
    if launch is None:
        launch = 0
    origin_x = float(rows[launch]["x_m"])
    origin_y = float(rows[launch]["y_m"])
    x = [float(row["x_m"]) - origin_x for row in rows]
    y = [float(row["y_m"]) - origin_y for row in rows]
    distance = [math.hypot(px, py) for px, py in zip(x, y)]

    # The commanded lobes peak at +/-0.50 m. Require the measured vehicle to
    # enter the outer 0.10 m of each lobe before accepting either crossing.
    right = first_index([px >= 0.40 for px in x], launch)
    crossing = None if right is None else first_index(
        [d <= radius_m for d in distance], right + 1
    )
    left = None if crossing is None else first_index(
        [px <= -0.40 for px in x], crossing + 1
    )
    finish = None if left is None else first_index(
        [d <= radius_m for d in distance], left + 1
    )
    ordered_lobes_ok = all(index is not None for index in (right, crossing, left, finish))
    center_crossing_error = None if crossing is None else distance[crossing]
    terminal_center_error = None if finish is None else distance[finish]
    # Yaw acceptance covers the flight itself, from launch through the ordered
    # terminal-center entry. Do not let post-completion hover corrections or
    # estimator noise change a completed flight's course-alignment result.
    yaw_window_end = finish if finish is not None else len(rows) - 1
    yaw_errors_deg = []
    for row in rows[launch:yaw_window_end + 1]:
        vx = float(row["vx_mps"])
        vy = float(row["vy_mps"])
        if math.hypot(vx, vy) <= minimum_yaw_speed_mps:
            continue
        yaw = yaw_from_quaternion(row)
        course = math.atan2(vy, vx)
        yaw_errors_deg.append(abs(math.degrees(math.remainder(yaw - course, 2.0 * math.pi))))
    median_yaw_error_deg = percentile(yaw_errors_deg, 0.50) if yaw_errors_deg else None
    p90_yaw_error_deg = percentile(yaw_errors_deg, 0.90) if yaw_errors_deg else None
    yaw_tracking_ok = bool(
        median_yaw_error_deg is not None
        and median_yaw_error_deg <= maximum_median_yaw_error_deg
        and p90_yaw_error_deg is not None
        and p90_yaw_error_deg <= maximum_p90_yaw_error_deg
    )
    solver_failure_matches = sorted(set(
        match.group(0) for match in SOLVER_FAILURE_RE.finditer(firmware)
    ))
    solver_healthy = not solver_failure_matches and "MPC: iterations=" in firmware
    crashed = bool(summary.get("crashed", True))
    contact_count_max = int(summary.get("contact_count_max", 1))
    contact_free = contact_count_max == 0
    success = bool(
        waypoint_sequence_ok and ordered_lobes_ok and
        center_crossing_error is not None and center_crossing_error <= radius_m and
        terminal_center_error is not None and terminal_center_error <= radius_m and
        yaw_tracking_ok and not crashed and contact_free and solver_healthy
    )
    return {
        "run": str(run.resolve()),
        "success": success,
        "route_waypoint_count": route_count,
        "waypoints_reached": len(events),
        "all_waypoints_reached_in_order": waypoint_sequence_ok,
        "ordered_right_center_left_center": ordered_lobes_ok,
        "center_crossing_error_m": center_crossing_error,
        "terminal_center_error_m": terminal_center_error,
        "maximum_positive_x_m": max(x[launch:]),
        "minimum_negative_x_m": min(x[launch:]),
        "yaw_samples_above_minimum_speed": len(yaw_errors_deg),
        "yaw_evaluation_start_time_s": float(rows[launch]["time_s"]),
        "yaw_evaluation_end_time_s": float(rows[yaw_window_end]["time_s"]),
        "minimum_yaw_speed_mps": minimum_yaw_speed_mps,
        "median_yaw_to_motion_error_deg": median_yaw_error_deg,
        "p90_yaw_to_motion_error_deg": p90_yaw_error_deg,
        "yaw_tracking_ok": yaw_tracking_ok,
        "crashed": crashed,
        "contact_count_max": contact_count_max,
        "contact_free": contact_free,
        "solver_healthy": solver_healthy,
        "solver_failure_matches": solver_failure_matches,
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("runs", nargs="+", type=Path)
    parser.add_argument("--radius-m", type=float, default=0.25)
    parser.add_argument("--minimum-yaw-speed-mps", type=float, default=0.15)
    parser.add_argument("--maximum-median-yaw-error-deg", type=float, default=35.0)
    parser.add_argument("--maximum-p90-yaw-error-deg", type=float, default=75.0)
    parser.add_argument("--out", type=Path)
    args = parser.parse_args()
    results = [verify_run(
        run, args.radius_m, args.minimum_yaw_speed_mps,
        args.maximum_median_yaw_error_deg, args.maximum_p90_yaw_error_deg,
    ) for run in args.runs]
    report = {
        "format": "tinympc-waypoint-figure8-tangent-yaw-acceptance-v2",
        "required_consecutive_runs": 3,
        "radius_m": args.radius_m,
        "minimum_yaw_speed_mps": args.minimum_yaw_speed_mps,
        "maximum_median_yaw_error_deg": args.maximum_median_yaw_error_deg,
        "maximum_p90_yaw_error_deg": args.maximum_p90_yaw_error_deg,
        "runs": results,
        "success": len(results) >= 3 and all(run["success"] for run in results),
    }
    rendered = json.dumps(report, indent=2) + "\n"
    if args.out:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        args.out.write_text(rendered)
    print(rendered, end="")
    if not report["success"]:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
