#!/usr/bin/env python3
"""Compare waypoint and dense trajectory reference modes on closed routes."""

from __future__ import annotations

import argparse
import csv
import json
import math
import re
from collections import defaultdict
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial import cKDTree


ROW_RE = re.compile(r"^\s*\{([^{}]+)\},\s*$", re.MULTILINE)
DEFINE_RE = re.compile(r"^#define\s+(\w+)\s+\(?([^\s)]+)", re.MULTILINE)
WAYPOINT_RE = re.compile(
    r"Waypoint reached index=(\d+)/(\d+) next_knot=(\d+) complete=(\d+)"
)
PROGRESS_COMPLETE_RE = re.compile(r"Progress path complete sample=([0-9.]+)/([0-9]+)")
SOLVER_FAILURE_RE = re.compile(
    r"\b(?:nan|inf|assert|hardfault|solver (?:fail|error)|segmentation fault)\b",
    re.IGNORECASE,
)
MAX_RPM_RE = re.compile(r"max_rpm\s*:\s*([0-9.]+)")


def load_csv(path: Path) -> dict[str, np.ndarray]:
    with path.open(newline="") as stream:
        rows = list(csv.DictReader(stream))
    if not rows:
        raise ValueError(f"no rows in {path}")
    return {
        name: np.asarray([float(row[name]) for row in rows], dtype=float)
        for name in rows[0]
    }


def load_header(path: Path) -> tuple[dict[str, str], np.ndarray]:
    text = path.read_text()
    defines = dict(DEFINE_RE.findall(text))
    rows = []
    for body in ROW_RE.findall(text):
        values = [float(item.strip().removesuffix("f")) for item in body.split(",")]
        if len(values) == 13:
            rows.append(values)
    if not rows:
        raise ValueError(f"no 13-state trajectory rows in {path}")
    return defines, np.asarray(rows, dtype=float)


def quaternion_yaw(q: np.ndarray) -> np.ndarray:
    w, x, y, z = q.T
    return np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def quaternion_multiply(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    aw, ax, ay, az = np.moveaxis(a, -1, 0)
    bw, bx, by, bz = np.moveaxis(b, -1, 0)
    return np.stack((
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    ), axis=-1)


def angle_error_deg(actual: np.ndarray, desired: np.ndarray) -> np.ndarray:
    return np.abs(np.rad2deg(np.angle(np.exp(1j * (actual - desired)))))


def statistics(values: np.ndarray, median: bool = False) -> dict[str, float | None]:
    finite = values[np.isfinite(values)]
    if not finite.size:
        return {"median": None, "p90": None, "max": None} if median else {
            "rmse": None, "p95": None, "max": None
        }
    magnitude = np.abs(finite)
    if median:
        return {
            "median": float(np.median(magnitude)),
            "p90": float(np.percentile(magnitude, 90)),
            "max": float(np.max(magnitude)),
        }
    return {
        "rmse": float(np.sqrt(np.mean(finite**2))),
        "p95": float(np.percentile(magnitude, 95)),
        "max": float(np.max(magnitude)),
    }


def waypoint_route_indices(route: np.ndarray, spacing_m: float = 0.30) -> list[int]:
    indices = [0]
    while indices[-1] + 1 < len(route):
        anchor = route[indices[-1], :2]
        distance = np.linalg.norm(route[indices[-1] + 1:, :2] - anchor, axis=1)
        candidates = np.flatnonzero(distance >= spacing_m)
        if not candidates.size:
            break
        indices.append(indices[-1] + 1 + int(candidates[0]))
    if indices[-1] != len(route) - 1:
        indices.append(len(route) - 1)
    return indices


def ordered_waypoint_completion(points: np.ndarray, route: np.ndarray,
                                radius_m: float = 0.25) -> int | None:
    search = 0
    for route_index in waypoint_route_indices(route):
        distance = np.linalg.norm(points[search:] - route[route_index, :3], axis=1)
        reached = np.flatnonzero(distance <= radius_m)
        if not reached.size:
            return None
        search += int(reached[0]) + 1
        if search >= len(points) and route_index != len(route) - 1:
            return None
    return min(search - 1, len(points) - 1)


def anchored_reference(reference: np.ndarray, launch_position: np.ndarray,
                       launch_yaw: float) -> np.ndarray:
    result = reference.copy()
    c, s = math.cos(launch_yaw), math.sin(launch_yaw)
    rotation = np.asarray([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
    result[:, :3] = launch_position + (reference[:, :3] - reference[0, :3]) @ rotation.T
    result[:, 7:10] = reference[:, 7:10] @ rotation.T
    yaw_q = np.asarray([math.cos(launch_yaw / 2.0), 0.0, 0.0,
                        math.sin(launch_yaw / 2.0)])
    result[:, 3:7] = quaternion_multiply(
        np.broadcast_to(yaw_q, result[:, 3:7].shape), result[:, 3:7]
    )
    return result


def nearest_route(points: np.ndarray, route: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Return exact local-segment cross-track and a nearby route-knot index."""
    tree = cKDTree(route[:, :2])
    _, knots = tree.query(points[:, :2])
    best = np.full(len(points), np.inf)
    best_knot = knots.copy()
    for offset in (-2, -1, 0, 1):
        first = np.clip(knots + offset, 0, len(route) - 2)
        a = route[first, :2]
        segment = route[first + 1, :2] - a
        denominator = np.einsum("ij,ij->i", segment, segment)
        alpha = np.divide(
            np.einsum("ij,ij->i", points[:, :2] - a, segment), denominator,
            out=np.zeros(len(points)), where=denominator > 0.0,
        )
        alpha = np.clip(alpha, 0.0, 1.0)
        distance = np.linalg.norm(points[:, :2] - (a + alpha[:, None] * segment), axis=1)
        improve = distance < best
        best[improve] = distance[improve]
        best_knot[improve] = first[improve] + (alpha[improve] >= 0.5)
    return best, best_knot


def interp_reference(reference: np.ndarray, sample: np.ndarray) -> np.ndarray:
    sample = np.clip(sample, 0.0, len(reference) - 1.0)
    lower = np.floor(sample).astype(int)
    upper = np.minimum(lower + 1, len(reference) - 1)
    alpha = sample - lower
    result = (1.0 - alpha[:, None]) * reference[lower] + alpha[:, None] * reference[upper]
    q = result[:, 3:7]
    result[:, 3:7] = q / np.linalg.norm(q, axis=1)[:, None]
    return result


def progress_speed_profile(reference: np.ndarray) -> np.ndarray:
    difference = np.diff(reference[:, :2], axis=0)
    length = np.linalg.norm(difference, axis=1)
    curvature = np.zeros(len(reference), dtype=float)
    if len(reference) >= 3:
        turn = np.arctan2(
            difference[:-1, 0] * difference[1:, 1]
                - difference[:-1, 1] * difference[1:, 0],
            np.einsum("ij,ij->i", difference[:-1], difference[1:]),
        )
        denominator = 0.5 * (length[:-1] + length[1:])
        local = np.divide(turn, denominator, out=np.zeros_like(turn), where=denominator > 1e-9)
        curvature[:-2] = local
        curvature[-2:] = local[-1]
    return np.clip(0.15 / (1.0 + 0.75 * np.abs(curvature)), 0.05, 0.15)


def analyze_run(run: Path, headers: Path, minimum_yaw_speed: float) -> dict[str, object]:
    config = json.loads((run / "run_config.json").read_text())
    shape = str(config["trajectory"])
    mode = str(config.get("reference_mode", "waypoint"))
    defines, raw_reference = load_header(headers / f"traj_{shape}_50hz.h")
    sample_dt = float(defines["TRAJECTORY_SAMPLE_DT_S"].removesuffix("f"))
    data = load_csv(run / "state.csv")
    firmware = (run / "firmware.log").read_text(errors="replace")
    simulator = (run / "simulator.log").read_text(errors="replace")
    t = data["time_s"]
    airborne = np.flatnonzero(data["airborne"] > 0.5)
    launch_index = int(airborne[0]) if airborne.size else int(np.searchsorted(t, config["launch_time_s"]))
    launch_position = np.asarray([data[key][launch_index] for key in ("x_m", "y_m", "z_m")])
    actual_q_all = np.column_stack([data[key] for key in ("qw", "qx", "qy", "qz")])
    launch_yaw = float(quaternion_yaw(actual_q_all[launch_index:launch_index + 1])[0])
    reference = anchored_reference(raw_reference, launch_position, launch_yaw)
    waypoint_events = [tuple(map(int, match.groups())) for match in WAYPOINT_RE.finditer(firmware)]
    all_points = np.column_stack([data[key] for key in ("x_m", "y_m", "z_m")])
    crash_indices = np.flatnonzero((data["contacts"] > 0.0) & (np.arange(len(t)) >= launch_index))
    first_crash_index = int(crash_indices[0]) if crash_indices.size else None
    route_completion_index: int | None = None
    if mode in ("waypoint", "progress"):
        route_completion_index = ordered_waypoint_completion(
            all_points[launch_index:], reference,
            0.15 if mode == "progress" else 0.25,
        )
        if mode == "waypoint":
            complete = bool(
                route_completion_index is not None and waypoint_events
                and waypoint_events[-1][3] == 1
            )
        else:
            complete = bool(
                route_completion_index is not None
                and PROGRESS_COMPLETE_RE.search(firmware)
            )
        evaluation_end_s = float(
            t[launch_index + route_completion_index]
            if route_completion_index is not None else t[-1]
        )
    elif mode == "trajectory":
        evaluation_end_s = float(t[launch_index] + 0.4 + (len(reference) - 1) * sample_dt)
        complete = bool(t[-1] >= evaluation_end_s)
        route_completion_index = ordered_waypoint_completion(
            all_points[launch_index:int(np.searchsorted(t, evaluation_end_s, side="right"))],
            reference, 0.25,
        )
    else:
        evaluation_end_s = float(t[-1])
        complete = False
    evaluation_end_s = min(evaluation_end_s, float(t[-1]))
    if first_crash_index is not None:
        evaluation_end_s = min(evaluation_end_s, float(t[first_crash_index]))
    window = (t >= t[launch_index]) & (t <= evaluation_end_s)
    indices = np.flatnonzero(window)
    points = np.column_stack([data[key][indices] for key in ("x_m", "y_m", "z_m")])
    velocity = np.column_stack([data[key][indices] for key in ("vx_mps", "vy_mps", "vz_mps")])
    omega = np.column_stack([data[key][indices] for key in ("wx_radps", "wy_radps", "wz_radps")])
    actual_q = actual_q_all[indices]
    cross_track, route_index = nearest_route(points, reference)
    route_local = reference[route_index]
    altitude_error = points[:, 2] - route_local[:, 2]
    geometric_position_error = np.hypot(cross_track, altitude_error)
    flight_envelope_violated = bool(
        np.any(np.abs(altitude_error) > 1.0) or np.any(cross_track > 2.0)
    )
    horizontal_speed = np.linalg.norm(velocity[:, :2], axis=1)
    if mode == "progress":
        nominal_horizontal_speed = progress_speed_profile(reference)[route_index]
    else:
        nominal_horizontal_speed = np.linalg.norm(route_local[:, 7:9], axis=1)
    speed_error = horizontal_speed - nominal_horizontal_speed
    actual_yaw = quaternion_yaw(actual_q)
    tangent_yaw = np.arctan2(route_local[:, 8], route_local[:, 7])
    tangent_valid = (
        (nominal_horizontal_speed >= minimum_yaw_speed)
        & (horizontal_speed >= minimum_yaw_speed)
    )
    course_yaw = np.arctan2(velocity[:, 1], velocity[:, 0])
    course_valid = horizontal_speed >= minimum_yaw_speed
    attitude_dot = np.abs(np.sum(actual_q * route_local[:, 3:7], axis=1))
    nearest_attitude_error = np.rad2deg(2.0 * np.arccos(np.clip(attitude_dot, 0.0, 1.0)))
    rate_error = np.linalg.norm(omega - route_local[:, 10:13], axis=1)
    rpm = np.column_stack([data[f"rpm_{motor}"][indices] for motor in range(1, 5)])
    max_rpm_match = MAX_RPM_RE.search(simulator)
    max_rpm = float(max_rpm_match.group(1)) if max_rpm_match else float(np.max(rpm))
    normalized_rpm = rpm / max_rpm
    dt = np.diff(t[indices])
    rpm_slew = np.diff(normalized_rpm, axis=0) / dt[:, None] if len(indices) > 1 else np.zeros((0, 4))
    contacts = int(np.max(data["contacts"][indices]))
    solver_failure_matches = sorted(set(match.group(0).lower() for match in SOLVER_FAILURE_RE.finditer(firmware)))
    result: dict[str, object] = {
        "run": str(run.resolve()),
        "shape": shape,
        "method": mode,
        "seed": int(config["random_seed"]),
        "evaluation_start_s": float(t[launch_index]),
        "evaluation_end_s": evaluation_end_s,
        "evaluation_duration_s": evaluation_end_s - float(t[launch_index]),
        "completion_time_s": (
            float(t[launch_index + route_completion_index] - t[launch_index])
            if complete and route_completion_index is not None
            and first_crash_index is None and not flight_envelope_violated else None
        ),
        "reference_command_completed": complete,
        "route_geometry_completed": route_completion_index is not None,
        "route_attempt_completed": bool(
            complete and route_completion_index is not None
            and first_crash_index is None and not flight_envelope_violated
        ),
        "waypoints_reached": len(waypoint_events),
        "waypoint_count": waypoint_events[0][1] if waypoint_events else None,
        "contact_count_max": contacts,
        "crashed": contacts > 0,
        "flight_envelope_healthy": not flight_envelope_violated,
        "solver_healthy": not solver_failure_matches and "MPC: iterations=" in firmware,
        "solver_failure_matches": solver_failure_matches,
        "cross_track_error_m": statistics(cross_track),
        "geometric_position_error_m": statistics(geometric_position_error),
        "altitude_error_m": statistics(altitude_error),
        "nominal_speed_error_mps": statistics(speed_error),
        "yaw_to_route_tangent_error_deg": statistics(
            angle_error_deg(actual_yaw[tangent_valid], tangent_yaw[tangent_valid]), median=True
        ),
        "yaw_to_velocity_course_error_deg": statistics(
            angle_error_deg(actual_yaw[course_valid], course_yaw[course_valid]), median=True
        ),
        "nearest_reference_attitude_error_deg": statistics(nearest_attitude_error),
        "nearest_reference_body_rate_error_radps": statistics(rate_error),
        "motor_saturation_fraction": float(np.mean(normalized_rpm >= 0.995)),
        "motor_rms_normalized_rpm": float(np.sqrt(np.mean(normalized_rpm**2))),
        "motor_rms_imbalance_normalized_rpm": float(np.sqrt(np.mean(
            (normalized_rpm - np.mean(normalized_rpm, axis=1)[:, None])**2
        ))),
        "motor_rms_slew_normalized_per_s": float(np.sqrt(np.mean(rpm_slew**2))) if rpm_slew.size else 0.0,
    }
    if mode == "trajectory":
        sample = np.maximum(0.0, (t[indices] - t[launch_index] - 0.4) / sample_dt)
        scheduled = interp_reference(reference, sample)
        scheduled_position = np.linalg.norm(points - scheduled[:, :3], axis=1)
        scheduled_velocity = np.linalg.norm(velocity - scheduled[:, 7:10], axis=1)
        scheduled_yaw = quaternion_yaw(scheduled[:, 3:7])
        scheduled_dot = np.abs(np.sum(actual_q * scheduled[:, 3:7], axis=1))
        result.update({
            "scheduled_position_error_m": statistics(scheduled_position),
            "scheduled_velocity_error_mps": statistics(scheduled_velocity),
            "scheduled_yaw_error_deg": statistics(
                angle_error_deg(actual_yaw, scheduled_yaw), median=True
            ),
            "scheduled_attitude_error_deg": statistics(
                np.rad2deg(2.0 * np.arccos(np.clip(scheduled_dot, 0.0, 1.0)))
            ),
            "scheduled_body_rate_error_radps": statistics(
                np.linalg.norm(omega - scheduled[:, 10:13], axis=1)
            ),
        })
    return result


def aggregate(results: list[dict[str, object]]) -> list[dict[str, object]]:
    groups: dict[tuple[str, str], list[dict[str, object]]] = defaultdict(list)
    for result in results:
        groups[(str(result["shape"]), str(result["method"]))].append(result)
    output = []
    fields = (
        ("cross_track_rmse_m", "cross_track_error_m", "rmse"),
        ("cross_track_p95_m", "cross_track_error_m", "p95"),
        ("cross_track_max_m", "cross_track_error_m", "max"),
        ("geometric_position_rmse_m", "geometric_position_error_m", "rmse"),
        ("geometric_position_p95_m", "geometric_position_error_m", "p95"),
        ("geometric_position_max_m", "geometric_position_error_m", "max"),
        ("altitude_rmse_m", "altitude_error_m", "rmse"),
        ("altitude_p95_m", "altitude_error_m", "p95"),
        ("altitude_max_m", "altitude_error_m", "max"),
        ("yaw_tangent_median_deg", "yaw_to_route_tangent_error_deg", "median"),
        ("yaw_tangent_p90_deg", "yaw_to_route_tangent_error_deg", "p90"),
        ("yaw_tangent_max_deg", "yaw_to_route_tangent_error_deg", "max"),
        ("yaw_course_median_deg", "yaw_to_velocity_course_error_deg", "median"),
        ("yaw_course_p90_deg", "yaw_to_velocity_course_error_deg", "p90"),
        ("yaw_course_max_deg", "yaw_to_velocity_course_error_deg", "max"),
        ("nominal_speed_rmse_mps", "nominal_speed_error_mps", "rmse"),
        ("nominal_speed_p95_mps", "nominal_speed_error_mps", "p95"),
        ("nominal_speed_max_mps", "nominal_speed_error_mps", "max"),
        ("attitude_rmse_deg", "nearest_reference_attitude_error_deg", "rmse"),
        ("attitude_p95_deg", "nearest_reference_attitude_error_deg", "p95"),
        ("attitude_max_deg", "nearest_reference_attitude_error_deg", "max"),
        ("body_rate_rmse_radps", "nearest_reference_body_rate_error_radps", "rmse"),
        ("body_rate_p95_radps", "nearest_reference_body_rate_error_radps", "p95"),
        ("body_rate_max_radps", "nearest_reference_body_rate_error_radps", "max"),
        ("motor_rms_normalized_rpm", "motor_rms_normalized_rpm", None),
        ("motor_rms_imbalance_normalized_rpm", "motor_rms_imbalance_normalized_rpm", None),
        ("motor_rms_slew_normalized_per_s", "motor_rms_slew_normalized_per_s", None),
        ("motor_saturation_fraction", "motor_saturation_fraction", None),
    )
    scheduled_fields = (
        ("scheduled_position_rmse_m", "scheduled_position_error_m", "rmse"),
        ("scheduled_position_p95_m", "scheduled_position_error_m", "p95"),
        ("scheduled_position_max_m", "scheduled_position_error_m", "max"),
        ("scheduled_velocity_rmse_mps", "scheduled_velocity_error_mps", "rmse"),
        ("scheduled_velocity_p95_mps", "scheduled_velocity_error_mps", "p95"),
        ("scheduled_velocity_max_mps", "scheduled_velocity_error_mps", "max"),
        ("scheduled_yaw_median_deg", "scheduled_yaw_error_deg", "median"),
        ("scheduled_yaw_p90_deg", "scheduled_yaw_error_deg", "p90"),
        ("scheduled_yaw_max_deg", "scheduled_yaw_error_deg", "max"),
        ("scheduled_attitude_rmse_deg", "scheduled_attitude_error_deg", "rmse"),
        ("scheduled_body_rate_rmse_radps", "scheduled_body_rate_error_radps", "rmse"),
    )
    for (shape, method), runs in sorted(groups.items()):
        row: dict[str, object] = {
            "shape": shape, "method": method, "runs": len(runs),
            "completed": sum(bool(run["route_attempt_completed"]) for run in runs),
            "contact_free": sum(not bool(run["crashed"]) for run in runs),
            "envelope_healthy": sum(bool(run["flight_envelope_healthy"]) for run in runs),
            "solver_healthy": sum(bool(run["solver_healthy"]) for run in runs),
        }
        completion_times = [float(run["completion_time_s"]) for run in runs
                            if run["completion_time_s"] is not None]
        attempt_durations = [float(run["evaluation_duration_s"]) for run in runs]
        row["completion_time_s_mean"] = (
            float(np.mean(completion_times)) if completion_times else None
        )
        row["completion_time_s_std"] = (
            float(np.std(completion_times, ddof=1)) if len(completion_times) > 1 else
            (0.0 if completion_times else None)
        )
        row["evaluation_duration_s_mean"] = float(np.mean(attempt_durations))
        row["evaluation_duration_s_std"] = (
            float(np.std(attempt_durations, ddof=1)) if len(attempt_durations) > 1 else 0.0
        )
        for output_name, field, subfield in fields:
            raw_values = [run[field][subfield] if subfield else run[field] for run in runs]
            values = [float(value) for value in raw_values if value is not None]
            row[output_name + "_mean"] = float(np.mean(values)) if values else None
            row[output_name + "_std"] = (
                float(np.std(values, ddof=1)) if len(values) > 1 else
                (0.0 if values else None)
            )
        for output_name, field, subfield in scheduled_fields:
            values = [float(run[field][subfield]) for run in runs if field in run]
            row[output_name + "_mean"] = float(np.mean(values)) if values else None
            row[output_name + "_std"] = (
                float(np.std(values, ddof=1)) if len(values) > 1 else
                (0.0 if values else None)
            )
        output.append(row)
    return output


def write_csv(rows: list[dict[str, object]], path: Path) -> None:
    with path.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def plot_summary(rows: list[dict[str, object]], output: Path) -> None:
    shapes = ("figure8", "oval", "circle")
    methods = ("waypoint", "progress", "trajectory")
    metrics = (
        ("cross_track_rmse_m_mean", "Cross-track RMSE", "m"),
        ("altitude_rmse_m_mean", "Altitude RMSE", "m"),
        ("yaw_tangent_median_deg_mean", "Yaw to tangent, median", "deg"),
        ("yaw_course_median_deg_mean", "Yaw to motion, median", "deg"),
        ("nominal_speed_rmse_mps_mean", "Nominal speed RMSE", "m/s"),
        ("motor_rms_imbalance_normalized_rpm_mean", "Motor imbalance RMS", "fraction max RPM"),
    )
    lookup = {(row["shape"], row["method"]): row for row in rows}
    fig, axes = plt.subplots(2, 3, figsize=(14, 8), constrained_layout=True)
    colors = {"waypoint": "#1565c0", "progress": "#2e7d32", "trajectory": "#ef6c00"}
    x = np.arange(len(shapes))
    for ax, (field, title, unit) in zip(axes.flat, metrics):
        for index, method in enumerate(methods):
            values = [lookup[(shape, method)][field]
                      if (shape, method) in lookup else np.nan for shape in shapes]
            errors = [lookup[(shape, method)][field.replace("_mean", "_std")]
                      if (shape, method) in lookup else np.nan for shape in shapes]
            values = [np.nan if value is None else value for value in values]
            errors = [np.nan if value is None else value for value in errors]
            ax.bar(x + (index - 1.0) * 0.25, values, 0.25, yerr=errors,
                   label=method, color=colors[method], capsize=3)
        ax.set_title(title)
        ax.set_ylabel(unit)
        ax.set_xticks(x, ["figure-8", "oval", "circle"])
        ax.grid(axis="y", alpha=0.25)
    axes[0, 0].legend()
    fig.suptitle("TinyMPC reference-method comparison (mean ± seed SD)")
    fig.savefig(output, dpi=180)
    plt.close(fig)


def plot_flight_paths(results: list[dict[str, object]], headers: Path,
                      output: Path) -> None:
    """Plot every evaluated flight in its launch-aligned frame."""
    shapes = ("figure8", "oval", "circle")
    methods = ("waypoint", "progress", "trajectory")
    colors = {"waypoint": "#1565c0", "progress": "#2e7d32", "trajectory": "#ef6c00"}
    fig, axes = plt.subplots(3, 3, figsize=(12, 11), constrained_layout=True)
    for row, shape in enumerate(shapes):
        _, raw_reference = load_header(headers / f"traj_{shape}_50hz.h")
        route = raw_reference[:, :2] - raw_reference[0, :2]
        for column, method in enumerate(methods):
            ax = axes[row, column]
            ax.plot(route[:, 0], route[:, 1], color="#263238", linewidth=1.5,
                    linestyle="--", label="route")
            matching = sorted(
                (result for result in results
                 if result["shape"] == shape and result["method"] == method),
                key=lambda result: int(result["seed"]),
            )
            for result in matching:
                data = load_csv(Path(str(result["run"])) / "state.csv")
                t = data["time_s"]
                selected = ((t >= float(result["evaluation_start_s"]))
                            & (t <= float(result["evaluation_end_s"])))
                indices = np.flatnonzero(selected)
                if not indices.size:
                    continue
                xy = np.column_stack((data["x_m"][indices], data["y_m"][indices]))
                origin = xy[0].copy()
                q = np.column_stack(tuple(data[key][indices[:1]]
                                          for key in ("qw", "qx", "qy", "qz")))
                yaw = float(quaternion_yaw(q)[0])
                c, s = math.cos(yaw), math.sin(yaw)
                world_to_launch = np.asarray([[c, s], [-s, c]])
                xy = (xy - origin) @ world_to_launch.T
                crashed = bool(result["crashed"])
                ax.plot(xy[:, 0], xy[:, 1], color=colors[method], alpha=0.55,
                        linewidth=1.0, label=f"seed {result['seed']}")
                ax.scatter(xy[-1, 0], xy[-1, 1], s=28,
                           marker="x" if crashed else "o", color=colors[method])
            if row == 0:
                ax.set_title(method)
            if column == 0:
                ax.set_ylabel(f"{shape}\ny (m)")
            if row == len(shapes) - 1:
                ax.set_xlabel("x (m)")
            ax.set_aspect("equal", adjustable="box")
            ax.grid(alpha=0.2)
            ax.legend(fontsize=7, loc="best")
    fig.suptitle("CrazySim flights in launch-aligned coordinates (× = contact)")
    fig.savefig(output, dpi=180)
    plt.close(fig)


def write_markdown_report(rows: list[dict[str, object]], output: Path) -> None:
    def metric(row: dict[str, object], field: str, digits: int) -> str:
        value = row[field]
        return "—" if value is None else f"{float(value):.{digits}f}"

    lines = [
        "# TinyMPC CrazySim reference-method comparison",
        "",
        "Each cell is the mean across three deterministic seeds. Errors are evaluated from "
        "launch to spatial completion (waypoint/progress) or schedule end (trajectory), "
        "capped by run end or first contact.",
        "",
        "| Shape | Method | Complete | Contact-free | In envelope | Solver healthy | Completion (s) | Cross-track RMSE (m) | Altitude RMSE (m) | Yaw–tangent median (deg) | Yaw–course median (deg) | Speed RMSE (m/s) | Attitude RMSE (deg) | Body-rate RMSE (rad/s) |",
        "|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for row in rows:
        completion = row["completion_time_s_mean"]
        completion_text = f"{completion:.1f}" if completion is not None else "—"
        lines.append(
            f"| {row['shape']} | {row['method']} | {row['completed']}/{row['runs']} "
            f"| {row['contact_free']}/{row['runs']} "
            f"| {row['envelope_healthy']}/{row['runs']} "
            f"| {row['solver_healthy']}/{row['runs']} "
            f"| {completion_text} "
            f"| {metric(row, 'cross_track_rmse_m_mean', 3)} "
            f"| {metric(row, 'altitude_rmse_m_mean', 3)} "
            f"| {metric(row, 'yaw_tangent_median_deg_mean', 1)} "
            f"| {metric(row, 'yaw_course_median_deg_mean', 1)} "
            f"| {metric(row, 'nominal_speed_rmse_mps_mean', 3)} "
            f"| {metric(row, 'attitude_rmse_deg_mean', 1)} "
            f"| {metric(row, 'body_rate_rmse_radps_mean', 3)} |"
        )
    lines += ["", "## Per-shape result", ""]
    for shape in ("figure8", "oval", "circle"):
        candidates = [row for row in rows if row["shape"] == shape]
        if not candidates:
            continue
        overall = sorted(
            candidates,
            key=lambda row: (-int(row["completed"]), -int(row["envelope_healthy"]),
                             -int(row["contact_free"]), float(
                                 row["cross_track_rmse_m_mean"]
                                 if row["cross_track_rmse_m_mean"] is not None else np.inf)),
        )[0]
        completed = [row for row in candidates if int(row["completed"]) > 0]
        yaw_pool = completed if completed else candidates
        yaw = min(yaw_pool, key=lambda row: float(
            row["yaw_tangent_median_deg_mean"]
            if row["yaw_tangent_median_deg_mean"] is not None else np.inf))
        qualifier = "among methods that completed at least one seed" if completed else (
            "although no method completed a seed"
        )
        lines.append(
            f"- **{shape}:** `{overall['method']}` ranks first by completion, then "
            f"flight-envelope health, contact-free count, and cross-track error "
            f"({overall['completed']}/{overall['runs']} complete, "
            f"{overall['envelope_healthy']}/{overall['runs']} in envelope, "
            f"{overall['contact_free']}/{overall['runs']} contact-free). "
            f"`{yaw['method']}` has the lowest yaw-to-tangent median {qualifier} "
            f"({metric(yaw, 'yaw_tangent_median_deg_mean', 1)}°)."
        )
    lines += [
        "",
        "The common geometric metrics compare all methods against the same route. The dense "
        "trajectory method also has wall-clock scheduled position, velocity, yaw, attitude, "
        "and body-rate errors in `comparison.json`; those are not directly defined for the "
        "spatial waypoint and progress methods.",
        "",
        "A contact-free run is not automatically a completed route. A failed run is retained "
        "in the aggregate rather than discarded, and its metric window ends at first contact.",
        "The flight envelope is route-relative: vertical error must stay within 1.0 m and "
        "horizontal cross-track error within 2.0 m. This catches non-contact flyaways.",
        "Consequently, a failed method can show a deceptively small yaw or geometric error if "
        "it tracked only an easy prefix; completion and contact counts take precedence.",
        "",
        "See `comparison.png` for metric bars and `flight_paths.png` for every top-down flight.",
    ]
    output.write_text("\n".join(lines) + "\n")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("runs", nargs="+", type=Path)
    parser.add_argument("--headers", type=Path, default=Path(
        "apps/controller_tinympc_eigen/src/trajectories/50hz"
    ))
    parser.add_argument("--minimum-yaw-speed-mps", type=float, default=0.05)
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)
    results = [analyze_run(run, args.headers, args.minimum_yaw_speed_mps) for run in args.runs]
    rows = aggregate(results)
    report = {
        "format": "tinympc-reference-method-benchmark-v1",
        "minimum_yaw_speed_mps": args.minimum_yaw_speed_mps,
        "runs": results,
        "aggregate": rows,
        "matrix_complete": len(results) == 27 and all(
            len([run for run in results if run["shape"] == shape and run["method"] == method]) == 3
            for shape in ("figure8", "oval", "circle")
            for method in ("waypoint", "progress", "trajectory")
        ),
    }
    (args.out / "comparison.json").write_text(json.dumps(report, indent=2) + "\n")
    write_csv(rows, args.out / "comparison.csv")
    plot_summary(rows, args.out / "comparison.png")
    plot_flight_paths(results, args.headers, args.out / "flight_paths.png")
    write_markdown_report(rows, args.out / "REPORT.md")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
