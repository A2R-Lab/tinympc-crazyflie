#!/usr/bin/env python3
"""Analyze sequential constant-speed TinyMPC progress runs on a circle.

References are reconstructed from the compiled circle header and the measured
handoff pose.  Controller progress is not present in state.csv, so dense
time-resolved progress is inferred with the controller's monotonic local-window
polyline projection.  Sparse firmware progress remains the authoritative
completion signal. Default-run metrics retain the historical inferred endpoint;
literal-uncapped runs end only at firmware-timed completion, contact, or run end.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import re
from pathlib import Path
from typing import Any

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


ROW_RE = re.compile(r"^\s*\{([^{}]+)\},\s*$", re.MULTILINE)
PROGRESS_RE = re.compile(r"Progress path (?:sample|complete sample)=([0-9.]+)/([0-9]+)")
COMPLETE_RE = re.compile(r"Progress path complete sample=([0-9.]+)/([0-9]+)")
READY_RE = re.compile(r"Progress path ready samples=(\d+) speed=([0-9.]+)\.\.([0-9.]+)m/s")
REFERENCE_LIMITS_RE = re.compile(r"Progress reference limits=(default|uncapped)\b")
DIAGNOSTIC_PREFIX_RE = re.compile(
    r"(?:Progress (?:reference|uncapped)(?: diagnostic| diag)?|"
    r"UNCAPPED (projection|refmax))\s*[: ]\s*(.*)", re.I)
KEY_VALUE_RE = re.compile(
    r"([A-Za-z_][A-Za-z0-9_]*)\s*=\s*([-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?)")
FAILURE_RE = re.compile(
    r"\b(?:nan|inf|assert|hardfault|solver (?:fail|error)|infeasible|segmentation fault)\b",
    re.IGNORECASE,
)
MAX_RPM_RE = re.compile(r"max_rpm\s*:\s*([0-9.]+)")
SPEED_KEYS = (
    "progress_speed_mps", "requested_speed_mps", "constant_speed_mps",
    "progress_max_speed_mps", "speed_mps",
)
THRESHOLDS = {
    "velocity_heading_error_over_45_deg": ("velocity_heading_error_deg", 45.0, 0.5),
    "cross_track_over_0p25_m": ("cross_track_abs_m", 0.25, 0.5),
    "altitude_error_over_0p20_m": ("altitude_error_abs_m", 0.20, 0.5),
    "vertical_speed_over_0p50_mps": ("vertical_speed_abs_mps", 0.50, 0.5),
    "yaw_error_over_25_deg": ("yaw_error_abs_deg", 25.0, 0.5),
    "yaw_rate_error_over_0p50_radps": ("yaw_rate_error_abs_radps", 0.50, 0.5),
    "tilt_over_30_deg": ("tilt_deg", 30.0, 0.25),
    "tilt_over_35_deg": ("tilt_deg", 35.0, 0.25),
    "yaw_frame_rodrigues_norm_over_1": ("yaw_frame_rodrigues_norm", 1.0, 0.25),
    "body_rate_norm_over_5_radps": ("body_rate_norm_radps", 5.0, 0.25),
    "rpm_saturation_over_98_percent": ("rpm_max_fraction", 0.98, 0.10),
}

UNCAPPED_FAILURE_CROSSINGS = {
    "cross_track_over_0p25_m",
    "altitude_error_over_0p20_m",
    "tilt_over_35_deg",
    "body_rate_norm_over_5_radps",
    "rpm_saturation_over_98_percent",
}


def load_json(path: Path) -> dict[str, Any]:
    value = json.loads(path.read_text())
    if not isinstance(value, dict):
        raise ValueError(f"expected JSON object in {path}")
    return value


def load_state(path: Path) -> dict[str, np.ndarray]:
    with path.open(newline="") as stream:
        rows = list(csv.DictReader(stream))
    if not rows:
        raise ValueError(f"no rows in {path}")
    data = {key: np.asarray([float(row[key]) for row in rows]) for key in rows[0]}
    required = {
        "time_s", "x_m", "y_m", "z_m", "qw", "qx", "qy", "qz",
        "vx_mps", "vy_mps", "vz_mps", "wx_radps", "wy_radps", "wz_radps",
        "rpm_1", "rpm_2", "rpm_3", "rpm_4", "contacts", "airborne",
    }
    missing = sorted(required - data.keys())
    if missing:
        raise ValueError(f"{path} lacks columns: {', '.join(missing)}")
    return data


def load_header(path: Path) -> np.ndarray:
    rows = []
    for body in ROW_RE.findall(path.read_text()):
        values = [float(item.strip().removesuffix("f")) for item in body.split(",")]
        if len(values) == 13:
            rows.append(values)
    if len(rows) < 3:
        raise ValueError(f"no usable 13-state route in {path}")
    return np.asarray(rows)


def quaternion_yaw(q: np.ndarray) -> np.ndarray:
    w, x, y, z = q.T
    return np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def quaternion_rpy(q: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    w, x, y, z = q.T
    roll = np.arctan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch = np.arcsin(np.clip(2.0 * (w * y - z * x), -1.0, 1.0))
    return roll, pitch, quaternion_yaw(q)


def wrap(angle: np.ndarray) -> np.ndarray:
    return (angle + np.pi) % (2.0 * np.pi) - np.pi


def anchor_route(raw: np.ndarray, state: dict[str, np.ndarray], launch: int) -> np.ndarray:
    route = raw.copy()
    q = np.column_stack([state[key] for key in ("qw", "qx", "qy", "qz")])
    yaw = float(quaternion_yaw(q[launch:launch + 1])[0])
    c, s = math.cos(yaw), math.sin(yaw)
    rotation = np.asarray([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
    launch_position = np.asarray([state[key][launch] for key in ("x_m", "y_m", "z_m")])
    route[:, :3] = launch_position + (raw[:, :3] - raw[0, :3]) @ rotation.T
    route[:, 7:10] = raw[:, 7:10] @ rotation.T
    return route


def route_geometry(route: np.ndarray) -> dict[str, np.ndarray]:
    segment = np.diff(route[:, :3], axis=0)
    length = np.linalg.norm(segment, axis=1)
    if np.any(length <= 1e-9):
        # Keep the original knot count/progress semantics; replace a zero
        # tangent only for metric reconstruction.
        positive = np.flatnonzero(length > 1e-9)
        if not positive.size:
            raise ValueError("circle route has zero length")
        for index in np.flatnonzero(length <= 1e-9):
            nearest = positive[np.argmin(np.abs(positive - index))]
            segment[index] = segment[nearest]
            length[index] = length[nearest]
    tangent = segment / length[:, None]
    curvature = np.zeros(len(segment))
    if len(segment) > 1:
        turn = np.arctan2(
            tangent[:-1, 0] * tangent[1:, 1] - tangent[:-1, 1] * tangent[1:, 0],
            np.einsum("ij,ij->i", tangent[:-1, :2], tangent[1:, :2]),
        )
        curvature[:-1] = turn / (0.5 * (length[:-1] + length[1:]))
        curvature[-1] = curvature[-2]
    return {"segment": segment, "length": length, "tangent": tangent, "curvature": curvature}


def reference_severity(route: np.ndarray, geometry: dict[str, np.ndarray],
                       speed: float) -> dict[str, Any]:
    """Reconstruct literal geometric commands at every header segment."""
    curvature = geometry["curvature"]
    yaw_rate = curvature * speed
    lateral_acceleration = curvature * speed * speed
    bank = np.arctan2(np.abs(lateral_acceleration), 9.81)
    thrust_scale = np.sqrt(1.0 + (lateral_acceleration / 9.81) ** 2) - 1.0
    bank_rodrigues = np.tan(0.5 * bank)
    absolute_curvature = np.abs(curvature)
    p95 = float(np.percentile(absolute_curvature, 95))
    median = float(np.median(absolute_curvature))
    robust_baseline = max(p95, median, 1.0e-9)
    artifact = absolute_curvature > 3.0 * robust_baseline
    first_artifact = int(np.flatnonzero(artifact)[0]) if np.any(artifact) else None
    maximum = int(np.argmax(absolute_curvature))
    return {
        "series": {
            "sample": np.arange(len(curvature), dtype=float),
            "curvature_per_m": curvature,
            "kappa_v_radps": yaw_rate,
            "kappa_v2_mps2": lateral_acceleration,
            "raw_bank_tilt_deg": np.degrees(bank),
            "thrust_scale": thrust_scale,
            "bank_only_local_reference_rodrigues_norm": bank_rodrigues,
        },
        "summary": {
            "source": "exact circle-header segment geometry",
            "curvature_abs_per_m": stats(absolute_curvature),
            "kappa_v_radps": stats(yaw_rate),
            "kappa_v2_mps2": stats(lateral_acceleration),
            "raw_bank_tilt_deg": stats(np.degrees(bank)),
            "thrust_scale": stats(thrust_scale),
            "bank_only_local_reference_rodrigues_norm": stats(bank_rodrigues),
            "maximum": {
                "segment_sample": maximum,
                "position_route_m": list(map(float, route[maximum, :3])),
                "curvature_per_m": float(curvature[maximum]),
                "kappa_v_radps": float(yaw_rate[maximum]),
                "kappa_v2_mps2": float(lateral_acceleration[maximum]),
                "raw_bank_tilt_deg": float(np.degrees(bank[maximum])),
                "thrust_scale": float(thrust_scale[maximum]),
                "bank_only_local_reference_rodrigues_norm": float(bank_rodrigues[maximum]),
            },
            "startup_curvature_artifact": {
                "exposed": bool(np.any(artifact)),
                "rule": "abs(curvature) > 3 * max(p95_abs, median_abs, 1e-9)",
                "median_abs_per_m": median,
                "p95_abs_per_m": p95,
                "first_segment_sample": first_artifact,
                "first_position_route_m": (
                    list(map(float, route[first_artifact, :3]))
                    if first_artifact is not None else None),
            },
        },
    }


def parse_uncapped_diagnostics(firmware: str) -> list[dict[str, Any]]:
    """Parse optional key=value progress-reference diagnostic lines."""
    records = []
    for line in firmware.splitlines():
        match = DIAGNOSTIC_PREFIX_RE.search(line)
        if not match:
            continue
        kind = (match.group(1) or "reference").lower()
        values = {key.lower(): float(value) for key, value in KEY_VALUE_RE.findall(match.group(2))}
        if not values or not any(key in values for key in (
                "sample", "progress", "kappa", "curvature", "kappa_v", "yaw_rate",
                "kappa_v2", "accel", "bank_deg", "tilt_deg", "thrust_scale",
                "ref_rod", "reference_rodrigues", "jump", "progress_jump",
                "previous", "current", "delta", "roll", "pitch", "tilt")):
            continue
        aliases = {
            "time_after_launch_s": ("time_after_launch_s", "t_after_launch", "t"),
            "progress_sample": ("progress_sample", "progress", "sample", "current"),
            "previous_progress_sample": ("previous_progress_sample", "previous"),
            "curvature_per_m": ("curvature_per_m", "curvature", "kappa"),
            "kappa_v_radps": ("kappa_v_radps", "kappa_v", "yaw_rate"),
            "kappa_v2_mps2": ("kappa_v2_mps2", "kappa_v2", "accel"),
            "raw_bank_tilt_deg": ("raw_bank_tilt_deg", "bank_deg", "tilt_deg"),
            "reference_roll_rad": ("reference_roll_rad", "roll"),
            "reference_pitch_rad": ("reference_pitch_rad", "pitch"),
            "reference_tilt_rad": ("reference_tilt_rad", "tilt"),
            "speed_mps": ("speed_mps", "speed"),
            "thrust_scale": ("thrust_scale",),
            "reference_rodrigues_norm": ("reference_rodrigues_norm", "reference_rodrigues", "ref_rod"),
            "progress_jump_samples": ("progress_jump_samples", "progress_jump", "jump", "delta"),
            "material_projection_jump": ("material_projection_jump", "material"),
            "uref_clamped": ("uref_clamped", "uref_clamp"),
        }
        record = {}
        for canonical, choices in aliases.items():
            for choice in choices:
                if choice in values:
                    record[canonical] = values[choice]
                    break
        record["diagnostic_kind"] = kind
        if "curvature_per_m" in record and "speed_mps" in record:
            record.setdefault("kappa_v_radps", record["curvature_per_m"] * record["speed_mps"])
            record["kappa_v2_mps2"] = record["curvature_per_m"] * record["speed_mps"] ** 2
        if "reference_tilt_rad" in record:
            record["raw_bank_tilt_deg"] = math.degrees(record["reference_tilt_rad"])
        if "reference_roll_rad" in record and "reference_pitch_rad" in record:
            w = (math.cos(0.5 * record["reference_roll_rad"])
                 * math.cos(0.5 * record["reference_pitch_rad"]))
            record["bank_only_reference_rodrigues_norm"] = (
                math.sqrt(max(1.0 - w * w, 0.0)) / max(abs(w), 1.0e-6))
        records.append(record)
    return records


def progress_jump_evidence(records: list[dict[str, Any]]) -> dict[str, Any]:
    explicit = [item["progress_jump_samples"] for item in records
                if "progress_jump_samples" in item]
    samples = [item["progress_sample"] for item in records if "progress_sample" in item]
    jumps = explicit if explicit else list(np.diff(samples))
    if not jumps:
        return {
            "validation": "unproven_sparse_firmware",
            "invalid": False,
            "reason": "per-solve progress diagnostics absent or insufficient",
            "observed_jumps_samples": [],
        }
    nonmonotonic = any(value < -1.0e-4 for value in jumps)
    too_large = any(value > 60.0001 for value in jumps)
    return {
        "validation": "invalid" if nonmonotonic or too_large else "proven_valid",
        "invalid": bool(nonmonotonic or too_large),
        "nonmonotonic": nonmonotonic,
        "over_60_segment_window": too_large,
        "maximum_jump_samples": float(max(jumps)),
        "observed_jumps_samples": list(map(float, jumps)),
        "source": "explicit firmware jump field" if explicit else "difference of diagnostic progress samples",
    }


def summarize_firmware_diagnostics(records: list[dict[str, Any]],
                                   route: np.ndarray) -> dict[str, Any]:
    fields = sorted({field for record in records for field, value in record.items()
                     if isinstance(value, (int, float))})
    summary: dict[str, Any] = {"record_count": len(records), "fields": {}}
    for field in fields:
        values = np.asarray([record[field] for record in records if field in record])
        summary["fields"][field] = stats(values)
        maximum_index = int(np.argmax(np.abs(values)))
        candidates = [record for record in records if field in record]
        record = candidates[maximum_index]
        progress = record.get("progress_sample")
        route_index = (min(max(int(math.floor(progress)), 0), len(route) - 1)
                       if progress is not None else None)
        summary["fields"][field]["maximum_record"] = {
            "value": float(record[field]),
            "time_after_launch_s": record.get("time_after_launch_s"),
            "progress_sample": progress,
            "position_route_m": (
                list(map(float, route[route_index, :3])) if route_index is not None else None),
        }
    return summary


def project_progress(points: np.ndarray, route: np.ndarray,
                     geometry: dict[str, np.ndarray]) -> dict[str, np.ndarray]:
    """Mirror the firmware's 4-back/60-forward monotonic projection window."""
    segment = geometry["segment"]
    length = geometry["length"]
    progress = np.zeros(len(points))
    projected = np.empty_like(points)
    current = 0.0
    for sample, point in enumerate(points):
        current_segment = min(int(math.floor(current)), len(segment) - 1)
        current_alpha = current - current_segment
        current_point = route[current_segment, :3] + current_alpha * segment[current_segment]
        best_distance = float(np.sum((point - current_point) ** 2))
        best = current
        first = max(0, current_segment - 4)
        last = min(len(segment) - 1, current_segment + 60)
        for index in range(first, last + 1):
            alpha = float(np.clip(
                np.dot(point - route[index, :3], segment[index]) / (length[index] ** 2),
                0.0, 1.0,
            ))
            candidate = index + alpha
            distance = float(np.sum((point - (route[index, :3] + alpha * segment[index])) ** 2))
            if distance < best_distance and candidate + 1e-4 >= current:
                best_distance, best = distance, candidate
        current = max(current, min(best, len(route) - 1.0))
        terminal_window_start = max(0.0, len(route) - 1.0 - 60.0)
        if (current >= terminal_window_start
                and float(np.sum((point - route[-1, :3]) ** 2)) <= 0.15**2):
            current = len(route) - 1.0
        progress[sample] = current
        index = min(int(math.floor(current)), len(segment) - 1)
        projected[sample] = route[index, :3] + (current - index) * segment[index]
    index = np.minimum(np.floor(progress).astype(int), len(segment) - 1)
    return {"sample": progress, "index": index, "projected": projected}


def stats(values: np.ndarray) -> dict[str, float | None]:
    finite = values[np.isfinite(values)]
    if not finite.size:
        return {"mean": None, "rmse": None, "p95_abs": None, "max_abs": None}
    return {
        "mean": float(np.mean(finite)),
        "rmse": float(np.sqrt(np.mean(finite * finite))),
        "p95_abs": float(np.percentile(np.abs(finite), 95)),
        "max_abs": float(np.max(np.abs(finite))),
    }


def first_sustained(time: np.ndarray, values: np.ndarray, threshold: float,
                    duration: float) -> float | None:
    active_start: float | None = None
    for stamp, value in zip(time, values):
        if np.isfinite(value) and value > threshold:
            if active_start is None:
                active_start = float(stamp)
            if stamp - active_start >= duration:
                return active_start
        else:
            active_start = None
    return None


def median_filter(values: np.ndarray, window: int = 5) -> np.ndarray:
    """Small dependency-free robust filter, preserving array length."""
    if window < 1 or window % 2 == 0:
        raise ValueError("median-filter window must be a positive odd number")
    if len(values) < window:
        return values.copy()
    radius = window // 2
    padded = np.pad(values, (radius, radius), mode="edge")
    return np.median(np.lib.stride_tricks.sliding_window_view(padded, window), axis=1)


def corrected_rate_grid(
    time: np.ndarray,
    euler_yaw: np.ndarray,
    measured_body_wz: np.ndarray,
    measured_tangent_speed: np.ndarray,
    curvature: np.ndarray,
    commanded_speed: float,
    tangent_yaw_error: np.ndarray,
    tilt: np.ndarray,
) -> dict[str, np.ndarray]:
    """Resample and robustly differentiate Euler yaw on the MPC's 50 Hz grid."""
    if len(time) < 2 or time[-1] <= time[0]:
        raise ValueError("rate analysis requires at least two increasing timestamps")
    grid = np.arange(float(time[0]), float(time[-1]) + 1e-9, 0.02)
    if len(grid) < 2:
        grid = np.asarray([float(time[0]), float(time[-1])])
    unwrapped_yaw = np.unwrap(euler_yaw)
    yaw_grid_raw = np.interp(grid, time, unwrapped_yaw)
    yaw_grid_filtered = median_filter(yaw_grid_raw, 5)
    euler_yaw_rate = np.gradient(yaw_grid_filtered, grid, edge_order=1)
    tangent_speed = np.interp(grid, time, measured_tangent_speed)
    curvature_grid = np.interp(grid, time, curvature)
    body_wz = np.interp(grid, time, measured_body_wz)
    required_rate = curvature_grid * tangent_speed
    commanded_rate = curvature_grid * commanded_speed
    return {
        "time_s": grid,
        "measured_signed_tangent_speed_mps": tangent_speed,
        "inferred_local_path_curvature_per_m": curvature_grid,
        "commanded_speed_mps": np.full(len(grid), commanded_speed),
        "commanded_curvature_speed_rate_radps": commanded_rate,
        "inferred_measured_speed_geometric_rate_radps": required_rate,
        "quaternion_euler_yaw_unwrapped_rad": yaw_grid_filtered,
        "quaternion_euler_yaw_rate_radps": euler_yaw_rate,
        "measured_body_wz_radps": body_wz,
        "euler_yaw_rate_minus_geometric_radps": euler_yaw_rate - required_rate,
        "body_wz_minus_geometric_radps": body_wz - required_rate,
        "euler_yaw_rate_minus_commanded_radps": euler_yaw_rate - commanded_rate,
        "body_wz_minus_commanded_radps": body_wz - commanded_rate,
        "body_wz_minus_euler_yaw_rate_radps": body_wz - euler_yaw_rate,
        "tangent_yaw_error_deg": np.degrees(np.interp(grid, time, tangent_yaw_error)),
        "tilt_deg": np.degrees(np.interp(grid, time, tilt)),
    }


def infer_speed(config: dict[str, Any], firmware: str, run: Path) -> float | None:
    for key in SPEED_KEYS:
        if key in config:
            return float(config[key])
    ready = READY_RE.search(firmware)
    if ready and abs(float(ready.group(2)) - float(ready.group(3))) < 1e-6:
        return float(ready.group(2))
    match = re.search(r"(?:speed|v)[_-]?([0-9]+(?:p[0-9]+|\.[0-9]+)?)", run.name, re.I)
    if match:
        return float(match.group(1).replace("p", "."))
    return None


def motor_metrics(rpm: np.ndarray, maximum_rpm: float) -> tuple[dict[str, Any], np.ndarray]:
    thrust_proxy = (rpm / maximum_rpm) ** 2
    basis = np.asarray([
        [1, 1, 1, 1], [-1, -1, 1, 1], [-1, 1, 1, -1], [-1, 1, -1, 1],
    ], dtype=float) / 2.0
    modes = thrust_proxy @ basis.T
    energy = np.sum(modes * modes, axis=0)
    fractions = energy / max(float(np.sum(energy)), 1e-12)
    return {
        "maximum_rpm_model": maximum_rpm,
        "rpm": stats(rpm.reshape(-1)),
        "saturation_fraction_samples_any_motor": float(np.mean(np.max(rpm, axis=1) >= 0.98 * maximum_rpm)),
        "modal_energy_fraction": dict(zip(("collective", "roll", "pitch", "yaw"), map(float, fractions))),
    }, modes


def analyze_run(run: Path, speed: float | None, raw_route: np.ndarray) -> tuple[dict[str, Any], dict[str, np.ndarray]]:
    required = ("run_config.json", "summary.json", "state.csv", "firmware.log")
    missing = [name for name in required if not (run / name).is_file()]
    if missing:
        raise ValueError(f"{run} lacks: {', '.join(missing)}")
    config = load_json(run / "run_config.json")
    summary = load_json(run / "summary.json")
    state = load_state(run / "state.csv")
    firmware = (run / "firmware.log").read_text(errors="replace")
    simulator = (run / "simulator.log").read_text(errors="replace") if (run / "simulator.log").exists() else ""
    requested_speed = speed if speed is not None else infer_speed(config, firmware, run)
    if requested_speed is None:
        raise ValueError(f"cannot infer requested constant speed for {run}; use --run SPEED=DIR")
    banner_match = REFERENCE_LIMITS_RE.search(firmware)
    configured_limits = str(config.get("progress_reference_limits", "default"))
    firmware_limits = banner_match.group(1) if banner_match else None
    reference_limits = configured_limits if configured_limits in ("default", "uncapped") else "unknown"
    uncapped = reference_limits == "uncapped"
    diagnostics = parse_uncapped_diagnostics(firmware)
    jump_evidence = progress_jump_evidence(diagnostics)
    time = state["time_s"]
    airborne = np.flatnonzero(state["airborne"] > 0.5)
    requested_launch = float(config.get("launch_time_s", 0.0))
    launch = int(airborne[0]) if airborne.size else int(np.searchsorted(time, requested_launch))
    launch = min(launch, len(time) - 1)
    actual_launch = float(summary.get("launch_time_s", time[launch]))
    calibrated = bool(airborne.size and abs(actual_launch - requested_launch) <= 0.5)
    route = anchor_route(raw_route, state, launch)
    geometry = route_geometry(route)
    severity = reference_severity(route, geometry, requested_speed)
    diagnostic_summary = summarize_firmware_diagnostics(diagnostics, route)
    raw_after = np.arange(launch, len(time))
    telemetry = np.column_stack([values[raw_after] for values in state.values()])
    nonfinite_rows = np.flatnonzero(~np.all(np.isfinite(telemetry), axis=1))
    first_nonfinite_at = int(nonfinite_rows[0]) if nonfinite_rows.size else None
    if first_nonfinite_at == 0:
        raise ValueError(f"first post-launch telemetry sample is nonfinite in {run}")
    after = raw_after[:first_nonfinite_at] if first_nonfinite_at is not None else raw_after
    positions = np.column_stack([state[key][after] for key in ("x_m", "y_m", "z_m")])
    projection = project_progress(positions, route, geometry)
    contact_relative = np.flatnonzero(state["contacts"][after] > 0.0)
    inferred_complete_relative = np.flatnonzero(projection["sample"] >= len(route) - 1.001)
    contact_at = int(contact_relative[0]) if contact_relative.size else None
    inferred_complete_at = int(inferred_complete_relative[0]) if inferred_complete_relative.size else None
    progress_events = [(float(value), int(total)) for value, total in PROGRESS_RE.findall(firmware)]
    firmware_max = max((value for value, _ in progress_events), default=0.0)
    firmware_total = max((total for _, total in progress_events), default=len(route) - 1)
    complete_events = [(float(value), int(total)) for value, total in COMPLETE_RE.findall(firmware)]
    firmware_complete = any(total == 750 and value >= 749.999 for value, total in complete_events)
    diagnostic_completion_times = [
        item["time_after_launch_s"] for item in diagnostics
        if item.get("progress_sample", -math.inf) >= len(route) - 1.001
        and "time_after_launch_s" in item
    ]
    firmware_complete_time = min(diagnostic_completion_times) if diagnostic_completion_times else None
    authoritative_complete_at = (
        int(np.searchsorted(time[after] - time[launch], firmware_complete_time))
        if firmware_complete and firmware_complete_time is not None else None
    )
    end = len(after) - 1
    if contact_at is not None:
        end = min(end, contact_at)
    if uncapped:
        if authoritative_complete_at is not None:
            end = min(end, authoritative_complete_at)
    elif inferred_complete_at is not None:
        # Compatibility window for retained default-mode evidence only.
        end = min(end, inferred_complete_at)
    rel = np.arange(end + 1)
    indices = after[rel]
    segment_index = projection["index"][rel]
    projected = projection["projected"][rel]
    tangent3 = geometry["tangent"][segment_index]
    tangent_xy = tangent3[:, :2]
    tangent_xy /= np.linalg.norm(tangent_xy, axis=1)[:, None]
    normal_xy = np.column_stack([-tangent_xy[:, 1], tangent_xy[:, 0]])
    velocity = np.column_stack([state[key][indices] for key in ("vx_mps", "vy_mps", "vz_mps")])
    tangent_velocity = np.einsum("ij,ij->i", velocity[:, :2], tangent_xy)
    normal_velocity = np.einsum("ij,ij->i", velocity[:, :2], normal_xy)
    desired_world_velocity = np.column_stack([
        requested_speed * tangent_xy, np.zeros(len(indices)),
    ])
    world_velocity_error = velocity - desired_world_velocity
    horizontal_speed = np.linalg.norm(velocity[:, :2], axis=1)
    velocity_heading_error = np.full(len(indices), np.nan)
    moving = horizontal_speed >= 0.02
    velocity_heading_error[moving] = np.degrees(np.arccos(np.clip(
        tangent_velocity[moving] / horizontal_speed[moving], -1.0, 1.0)))
    displacement = positions[rel, :2] - projected[:, :2]
    cross_track = np.einsum("ij,ij->i", displacement, normal_xy)
    altitude_error = positions[rel, 2] - projected[:, 2]
    q = np.column_stack([state[key][indices] for key in ("qw", "qx", "qy", "qz")])
    roll, pitch, yaw = quaternion_rpy(q)
    desired_yaw = np.arctan2(tangent_xy[:, 1], tangent_xy[:, 0])
    yaw_error = wrap(yaw - desired_yaw)
    desired_yaw_rate = geometry["curvature"][segment_index] * requested_speed
    yaw_rate_error = state["wz_radps"][indices] - desired_yaw_rate
    inferred_curvature = geometry["curvature"][segment_index]
    inferred_lateral_acceleration = inferred_curvature * requested_speed * requested_speed
    inferred_reference_bank = -np.arctan2(inferred_lateral_acceleration, 9.81)
    inferred_reference_tilt = np.abs(inferred_reference_bank)
    inferred_local_yaw = np.clip(-yaw_error, -math.radians(15.0), math.radians(15.0))
    inferred_reference_w = (
        np.cos(0.5 * inferred_reference_bank) * np.cos(0.5 * inferred_local_yaw))
    inferred_reference_rodrigues = np.sqrt(np.maximum(
        1.0 - inferred_reference_w * inferred_reference_w, 0.0)) / np.maximum(
            np.abs(inferred_reference_w), 1.0e-6)
    inferred_thrust_scale = np.sqrt(
        1.0 + (inferred_lateral_acceleration / 9.81) ** 2) - 1.0
    tilt = np.arccos(np.clip(np.cos(roll) * np.cos(pitch), -1.0, 1.0))
    # q_local = q_yaw^-1 * q; this is the current firmware's yaw-only chart.
    cy, sy = np.cos(yaw / 2.0), np.sin(yaw / 2.0)
    local_x = cy * q[:, 1] + sy * q[:, 2]
    local_y = cy * q[:, 2] - sy * q[:, 1]
    local_z = cy * q[:, 3] - sy * q[:, 0]
    local_w = cy * q[:, 0] + sy * q[:, 3]
    rodrigues_norm = np.sqrt(local_x**2 + local_y**2 + local_z**2) / np.maximum(np.abs(local_w), 1e-6)
    rates = np.column_stack([state[key][indices] for key in ("wx_radps", "wy_radps", "wz_radps")])
    rate_norm = np.linalg.norm(rates, axis=1)
    rpm = np.column_stack([state[f"rpm_{motor}"][indices] for motor in range(1, 5)])
    rpm_match = MAX_RPM_RE.search(simulator)
    maximum_rpm = float(rpm_match.group(1)) if rpm_match else max(float(np.max(rpm)), 1.0)
    motors, modes = motor_metrics(rpm, maximum_rpm)
    completion_authority_at = authoritative_complete_at if uncapped else inferred_complete_at
    active_contact = bool(contact_at is not None and (
        uncapped or completion_authority_at is None or contact_at <= completion_authority_at))
    post_completion_contact = bool(not uncapped and contact_at is not None and (
        completion_authority_at is not None and contact_at > completion_authority_at))
    # A summary-only crash cannot be timed. Uncapped evidence never lets an
    # offline endpoint demote it to post-completion.
    if summary.get("crashed", False) and contact_at is None:
        active_contact = uncapped or completion_authority_at is None
        post_completion_contact = not uncapped and completion_authority_at is not None
    nonfinite = bool(first_nonfinite_at is not None and (
        uncapped or completion_authority_at is None))
    post_completion_nonfinite = bool(not uncapped and
        first_nonfinite_at is not None and completion_authority_at is not None)
    failures = sorted(set(match.group(0).lower() for match in FAILURE_RE.finditer(firmware)))
    gross_envelope = bool(
        np.any(np.abs(cross_track) > 2.0) or np.any(np.abs(altitude_error) > 1.0))
    threshold_series = {
        "velocity_heading_error_deg": velocity_heading_error,
        "cross_track_abs_m": np.abs(cross_track),
        "altitude_error_abs_m": np.abs(altitude_error),
        "vertical_speed_abs_mps": np.abs(velocity[:, 2]),
        "yaw_error_abs_deg": np.abs(np.degrees(yaw_error)),
        "yaw_rate_error_abs_radps": np.abs(yaw_rate_error),
        "tilt_deg": np.degrees(tilt),
        "yaw_frame_rodrigues_norm": rodrigues_norm,
        "body_rate_norm_radps": rate_norm,
        "rpm_max_fraction": np.max(rpm, axis=1) / maximum_rpm,
    }
    relative_time = time[indices] - time[launch]
    rate_grid = corrected_rate_grid(
        relative_time, yaw, rates[:, 2], tangent_velocity,
        geometry["curvature"][segment_index], requested_speed, yaw_error, tilt,
    )
    crossings = {
        name: {
            "threshold": threshold, "required_duration_s": duration,
            "first_start_time_after_launch_s": first_sustained(relative_time, threshold_series[key], threshold, duration),
        }
        for name, (key, threshold, duration) in THRESHOLDS.items()
    }
    tight_envelope = any(
        crossings[name]["first_start_time_after_launch_s"] is not None
        for name in ("cross_track_over_0p25_m", "altitude_error_over_0p20_m"))
    envelope = gross_envelope or (uncapped and tight_envelope)
    uncapped_crossing_failures = sorted(
        name for name in UNCAPPED_FAILURE_CROSSINGS
        if crossings[name]["first_start_time_after_launch_s"] is not None)
    for name, key, threshold in (
        ("corrected_euler_yaw_rate_error_over_0p50_radps",
         "euler_yaw_rate_minus_geometric_radps", 0.50),
        ("corrected_body_wz_error_over_0p50_radps",
         "body_wz_minus_geometric_radps", 0.50),
        ("body_wz_minus_euler_yaw_rate_over_0p25_radps",
         "body_wz_minus_euler_yaw_rate_radps", 0.25),
    ):
        crossings[name] = {
            "threshold": threshold,
            "required_duration_s": 0.5,
            "first_start_time_after_launch_s": first_sustained(
                rate_grid["time_s"], np.abs(rate_grid[key]), threshold, 0.5),
        }
    inferred_completion_time = (
        float(time[after[inferred_complete_at]] - time[launch])
        if inferred_complete_at is not None else None)
    uncapped_banner_valid = not uncapped or firmware_limits == "uncapped"
    uncapped_failure_reasons = list(uncapped_crossing_failures)
    if uncapped and jump_evidence["invalid"]:
        uncapped_failure_reasons.append("invalid_progress_jump")
    if uncapped and not uncapped_banner_valid:
        uncapped_failure_reasons.append("uncapped_firmware_banner_missing_or_mismatched")
    passed = bool(
        calibrated and firmware_complete and not active_contact and not envelope
        and not nonfinite and not failures and not uncapped_failure_reasons
    )
    result = {
        "status": "passed" if passed else "failed",
        "passed": passed,
        "requested_speed_mps": requested_speed,
        "run_directory": str(run.resolve()),
        "progress_reference_limits": reference_limits,
        "evidence_label": (
            "artifact_exposed" if uncapped and
            severity["summary"]["startup_curvature_artifact"]["exposed"] else "standard"),
        "launch": {"calibrated": calibrated, "requested_s": requested_launch, "actual_s": actual_launch},
        "progress": {
            "firmware_max_sample": firmware_max, "firmware_total_samples": firmware_total,
            "firmware_complete_750_of_750": firmware_complete,
            "firmware_completion_time_after_launch_s": firmware_complete_time,
            "inferred_max_sample": float(projection["sample"][end]),
            "inferred_fraction": float(projection["sample"][end] / (len(route) - 1)),
            "inferred_completion": inferred_complete_at is not None,
            "inferred_completion_time_after_launch_s": inferred_completion_time,
            "offline_completion_used_as_authority": False if uncapped else True,
            "jump_evidence": jump_evidence,
        },
        "safety": {
            "contact_or_crash_active_route": active_contact,
            "contact_or_crash_post_completion": post_completion_contact,
            "contact_or_crash": active_contact,
            "envelope_violation": envelope,
            "gross_envelope_violation": gross_envelope,
            "tight_sustained_envelope_violation": tight_envelope,
            "nonfinite_state": nonfinite,
            "nonfinite_state_post_completion": post_completion_nonfinite,
            "failure_signatures": failures,
            "uncapped_failure_reasons": uncapped_failure_reasons,
        },
        "active_route_window": {
            "start_time_s": float(time[indices[0]]), "end_time_s": float(time[indices[-1]]),
            "samples": len(indices), "ended_by": "contact" if contact_at is not None and contact_at <= end else (
                "firmware_timed_completion" if authoritative_complete_at is not None and authoritative_complete_at <= end else (
                    "inferred_completion_compatibility" if not uncapped and inferred_complete_at is not None and inferred_complete_at <= end
                    else "run_end")),
            "post_completion_terminal_drift_excluded": bool(
                authoritative_complete_at is not None or
                (not uncapped and inferred_complete_at is not None)),
            "completion_authority": "firmware" if uncapped else "legacy_inferred_window",
        },
        "reference_evidence": {
            "configured_limits": configured_limits,
            "firmware_banner_limits": firmware_limits,
            "uncapped_banner_confirmed": uncapped_banner_valid,
            "raw_commanded_severity": severity["summary"],
            "firmware_diagnostics": {
                "records": diagnostics,
                "summary": diagnostic_summary,
                "record_count": len(diagnostics),
                "limitations": (
                    [] if diagnostics else
                    ["per-solve uncapped reference/progress diagnostics absent; exact jump and reference timing unproven"]),
            },
            "telemetry_aligned_inferred_severity": {
                "provenance": "offline monotonic projection; not firmware progress",
                "curvature_per_m": stats(inferred_curvature),
                "kappa_v_radps": stats(inferred_curvature * requested_speed),
                "kappa_v2_mps2": stats(inferred_lateral_acceleration),
                "raw_bank_tilt_deg": stats(np.degrees(inferred_reference_tilt)),
                "thrust_scale": stats(inferred_thrust_scale),
                "expected_local_reference_rodrigues_norm": stats(inferred_reference_rodrigues),
                "maximum_time_location": {
                    "time_after_launch_s": float(relative_time[np.argmax(inferred_reference_tilt)]),
                    "position_world_m": list(map(float, positions[rel][np.argmax(inferred_reference_tilt)])),
                    "inferred_progress_sample": float(projection["sample"][rel][np.argmax(inferred_reference_tilt)]),
                },
            },
        },
        "tracking": {
            "tangent_velocity_mps": stats(tangent_velocity),
            "normal_velocity_mps": stats(normal_velocity),
            "world_vx_error_mps": stats(world_velocity_error[:, 0]),
            "world_vy_error_mps": stats(world_velocity_error[:, 1]),
            "world_vz_error_mps": stats(world_velocity_error[:, 2]),
            "velocity_heading_error_deg": stats(velocity_heading_error),
            "cross_track_m": stats(cross_track), "altitude_error_m": stats(altitude_error),
            "vertical_speed_mps": stats(velocity[:, 2]),
            "tangent_yaw_error_deg": stats(np.degrees(yaw_error)),
            "yaw_rate_error_radps": stats(yaw_rate_error),
            "corrected_yaw_rate_50hz": {
                "sample_rate_hz": 50.0,
                "measured_signed_tangent_speed_mps": stats(
                    rate_grid["measured_signed_tangent_speed_mps"]),
                "inferred_local_path_curvature_per_m": stats(
                    rate_grid["inferred_local_path_curvature_per_m"]),
                "commanded_curvature_speed_rate_radps": stats(
                    rate_grid["commanded_curvature_speed_rate_radps"]),
                "inferred_measured_speed_geometric_rate_radps": stats(
                    rate_grid["inferred_measured_speed_geometric_rate_radps"]),
                "quaternion_euler_yaw_rate_radps": stats(
                    rate_grid["quaternion_euler_yaw_rate_radps"]),
                "measured_body_wz_radps": stats(rate_grid["measured_body_wz_radps"]),
                "euler_yaw_rate_minus_geometric_radps": stats(
                    rate_grid["euler_yaw_rate_minus_geometric_radps"]),
                "body_wz_minus_geometric_radps": stats(
                    rate_grid["body_wz_minus_geometric_radps"]),
                "euler_yaw_rate_minus_commanded_radps": stats(
                    rate_grid["euler_yaw_rate_minus_commanded_radps"]),
                "body_wz_minus_commanded_radps": stats(
                    rate_grid["body_wz_minus_commanded_radps"]),
                "body_wz_minus_euler_yaw_rate_radps": stats(
                    rate_grid["body_wz_minus_euler_yaw_rate_radps"]),
            },
        },
        "attitude_rates": {
            "roll_deg": stats(np.degrees(roll)), "pitch_deg": stats(np.degrees(pitch)),
            "tilt_deg": stats(np.degrees(tilt)),
            "yaw_only_local_rodrigues_norm": stats(rodrigues_norm),
            "wx_radps": stats(rates[:, 0]), "wy_radps": stats(rates[:, 1]),
            "wz_radps": stats(rates[:, 2]), "rate_norm_radps": stats(rate_norm),
        },
        "motors": motors,
        "first_sustained_threshold_crossings": crossings,
        "inference_notes": [
            "time-resolved progress/reference are reconstructed from measured handoff pose and circle header",
            "projection mirrors the 4-back/60-forward monotonic search but omits firmware's per-50Hz 0.02 m advance cap because telemetry is 1 kHz",
            "offline inferred completion is never authoritative for literal-uncapped classification",
            "raw commanded severity uses exact header curvature; telemetry timing uses inferred progress unless firmware diagnostics provide it",
            "yaw-rate reference is reconstructed as circle curvature times requested constant speed",
            "corrected geometric heading rate is inferred as local path curvature times measured signed tangent speed",
            "Euler yaw is quaternion-derived, unwrapped, resampled to 50 Hz, median-filtered over 5 samples, then differentiated",
            "body wz is reported separately from Euler yaw derivative because they differ when roll/pitch are nonzero",
            "RPM modal fractions use squared normalized RPM as a thrust proxy",
        ],
    }
    plot = {
        "time": relative_time, "position": positions[rel], "projected": projected,
        "route": route, "progress": projection["sample"][rel],
        "cross_track": cross_track, "altitude_error": altitude_error,
        "tangent_velocity": tangent_velocity, "normal_velocity": normal_velocity,
        "yaw_error_deg": np.degrees(yaw_error), "yaw_rate_error": yaw_rate_error,
        "roll_deg": np.degrees(roll), "pitch_deg": np.degrees(pitch), "tilt_deg": np.degrees(tilt),
        "rates": rates, "rpm": rpm, "modes": modes,
        "raw_reference_tilt_deg": np.degrees(inferred_reference_tilt),
        "expected_reference_rodrigues": inferred_reference_rodrigues,
        "raw_reference_thrust_scale": inferred_thrust_scale,
        "header_severity": severity["series"],
        "rate_grid": rate_grid,
    }
    return result, plot


def diagnostic_plot(result: dict[str, Any], data: dict[str, np.ndarray], path: Path) -> None:
    fig, axes = plt.subplots(3, 3, figsize=(15, 12))
    t = data["time"]
    grid = data["rate_grid"]
    axes[0, 0].plot(data["route"][:, 0], data["route"][:, 1], "--", color="0.6", label="inferred reference")
    axes[0, 0].plot(data["position"][:, 0], data["position"][:, 1], label="measured")
    axes[0, 0].axis("equal"); axes[0, 0].set_title("Top-down active route")
    axes[0, 1].plot(grid["time_s"], grid["commanded_speed_mps"], "--", label="commanded")
    axes[0, 1].plot(grid["time_s"], grid["measured_signed_tangent_speed_mps"], label="measured signed tangent")
    axes[0, 1].legend(); axes[0, 1].set_title("Command vs measured speed (m/s)")
    axes[0, 2].plot(grid["time_s"], grid["commanded_curvature_speed_rate_radps"], "--", label="curvature × command")
    axes[0, 2].plot(grid["time_s"], grid["inferred_measured_speed_geometric_rate_radps"], label="curvature × measured tangent")
    axes[0, 2].plot(grid["time_s"], grid["quaternion_euler_yaw_rate_radps"], label="Euler yaw derivative")
    axes[0, 2].plot(grid["time_s"], grid["measured_body_wz_radps"], alpha=0.7, label="body wz")
    axes[0, 2].legend(fontsize=8); axes[0, 2].set_title("Commanded, required, and actual rates")
    axes[1, 0].plot(grid["time_s"], grid["euler_yaw_rate_minus_geometric_radps"], label="yaw-dot − geometric")
    axes[1, 0].plot(grid["time_s"], grid["body_wz_minus_geometric_radps"], label="wz − geometric")
    axes[1, 0].plot(grid["time_s"], grid["body_wz_minus_euler_yaw_rate_radps"], label="wz − yaw-dot")
    axes[1, 0].legend(fontsize=8); axes[1, 0].set_title("Corrected rate errors (rad/s)")
    axes[1, 1].plot(grid["time_s"], grid["tangent_yaw_error_deg"])
    axes[1, 1].set_title("Tangent-yaw error (deg)")
    axes[1, 2].plot(grid["time_s"], grid["tilt_deg"], label="tilt deg")
    axes[1, 2].plot(t, data["raw_reference_tilt_deg"], "--", label="raw reference tilt deg")
    axes[1, 2].plot(t, 100.0 * np.abs(data["cross_track"]), label="|cross-track| cm")
    axes[1, 2].plot(t, 100.0 * np.abs(data["altitude_error"]), label="|altitude error| cm")
    axes[1, 2].legend(fontsize=8); axes[1, 2].set_title("Safety / tilt")
    axes[2, 0].plot(t, data["rates"]); axes[2, 0].legend(("wx", "wy", "wz")); axes[2, 0].set_title("Body rates (rad/s)")
    axes[2, 1].plot(t, data["rpm"]); axes[2, 1].set_title("Motor RPM")
    header = data["header_severity"]
    axes[2, 2].plot(header["sample"], header["raw_bank_tilt_deg"], label="raw bank deg")
    axes[2, 2].plot(header["sample"], header["bank_only_local_reference_rodrigues_norm"] * 100.0,
                    label="bank Rodrigues ×100")
    axes[2, 2].legend(fontsize=8); axes[2, 2].set_title("Exact header reference severity")
    for axis in axes.flat:
        axis.grid(True, alpha=0.25); axis.set_xlabel("time after launch (s)")
    fig.suptitle(f"{result['requested_speed_mps']:.3f} m/s — {result['status']}")
    fig.tight_layout()
    fig.savefig(path, dpi=160)
    plt.close(fig)


def comparison_plot(results: list[dict[str, Any]], plots: dict[float, dict[str, np.ndarray]], path: Path) -> None:
    fig, axes = plt.subplots(3, 2, figsize=(14, 13))
    for result in results:
        speed = float(result["requested_speed_mps"])
        data = plots[speed]
        label = f"{speed:.3f} ({result['status']})"
        axes[0, 0].plot(data["position"][:, 0], data["position"][:, 1], label=label)
        grid = data["rate_grid"]
        axes[0, 1].plot(grid["time_s"], grid["measured_signed_tangent_speed_mps"], label=label)
        axes[0, 1].plot(grid["time_s"], grid["commanded_speed_mps"], "--", alpha=0.6)
        axes[1, 0].plot(grid["time_s"], grid["inferred_measured_speed_geometric_rate_radps"], label=f"{label} geometric")
        axes[1, 0].plot(grid["time_s"], grid["quaternion_euler_yaw_rate_radps"], alpha=0.75, label=f"{label} yaw-dot")
        axes[1, 0].plot(grid["time_s"], grid["measured_body_wz_radps"], ":", alpha=0.75, label=f"{label} body wz")
        axes[1, 0].plot(grid["time_s"], grid["commanded_curvature_speed_rate_radps"], "--", alpha=0.5)
        axes[1, 1].plot(grid["time_s"], grid["euler_yaw_rate_minus_geometric_radps"], label=label)
        axes[1, 1].plot(grid["time_s"], grid["body_wz_minus_geometric_radps"], ":", alpha=0.75, label=f"{label} body wz")
        axes[2, 0].plot(grid["time_s"], np.abs(grid["tangent_yaw_error_deg"]), label=label)
        axes[2, 1].plot(grid["time_s"], grid["tilt_deg"], label=f"{label} tilt deg")
        axes[2, 1].plot(data["time"], data["raw_reference_tilt_deg"], "--", alpha=0.7,
                        label=f"{label} raw ref tilt")
        axes[2, 1].plot(data["time"], 100.0 * np.abs(data["cross_track"]), "--", alpha=0.65, label=f"{label} cross-track cm")
    axes[0, 0].plot(next(iter(plots.values()))["route"][:, 0], next(iter(plots.values()))["route"][:, 1], "--", color="0.6", label="inferred reference")
    axes[0, 0].set_title("Top-down paths"); axes[0, 0].axis("equal")
    axes[0, 1].set_title("Commanded (dashed) vs measured signed tangent speed")
    axes[1, 0].set_title("Commanded (dashed), geometric-required, Euler yaw, and body-wz rates")
    axes[1, 1].set_title("Corrected Euler/body rate errors vs measured-speed geometry")
    axes[2, 0].set_title("Absolute tangent-yaw error (deg)")
    axes[2, 1].set_title("Safety / tilt")
    for axis in axes.flat:
        axis.grid(True, alpha=0.25); axis.legend(fontsize=8)
    fig.tight_layout(); fig.savefig(path, dpi=170); plt.close(fig)


def write_rate_grid(path: Path, data: dict[str, np.ndarray]) -> None:
    fields = list(data)
    with path.open("w", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(fields)
        writer.writerows(zip(*(data[field] for field in fields)))


def discover_runs(root: Path) -> list[tuple[float | None, Path]]:
    return [(None, path.parent) for path in sorted(root.rglob("run_config.json"))
            if (path.parent / "summary.json").exists() and (path.parent / "state.csv").exists()]


def planned_speeds(args: argparse.Namespace, root: Path | None) -> list[float]:
    values = list(args.planned_speed)
    if args.planned_speeds:
        values.extend(float(item) for item in args.planned_speeds.split(",") if item.strip())
    if root:
        for name in ("experiment_plan.json", "plan.json"):
            path = root / name
            if path.exists():
                plan = load_json(path)
                for key in ("planned_speeds_mps", "speeds_mps", "requested_speeds_mps"):
                    if key in plan:
                        values.extend(map(float, plan[key]))
                        break
    return sorted(set(values))


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--experiment-root", type=Path)
    group.add_argument("--run", action="append", default=[], metavar="SPEED=DIR")
    parser.add_argument("--planned-speed", action="append", type=float, default=[])
    parser.add_argument("--planned-speeds", help="comma-separated planned ladder")
    parser.add_argument("--circle-header", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    entries: list[tuple[float | None, Path]] = []
    if args.experiment_root:
        entries = discover_runs(args.experiment_root)
    else:
        for value in args.run:
            speed, separator, directory = value.partition("=")
            if not separator:
                raise ValueError(f"--run must be SPEED=DIR, got {value!r}")
            entries.append((float(speed), Path(directory)))
    if not entries:
        raise ValueError("no completed run directories found")
    raw = load_header(args.circle_header)
    analyzed = [analyze_run(run, speed, raw) for speed, run in entries]
    analyzed.sort(key=lambda pair: float(pair[0]["requested_speed_mps"]))
    results = [pair[0] for pair in analyzed]
    plots = {float(result["requested_speed_mps"]): plot for result, plot in analyzed}
    args.out.mkdir(parents=True, exist_ok=True)
    diagnostics = args.out / "diagnostics"
    diagnostics.mkdir(exist_ok=True)
    for result, plot in analyzed:
        stem = f"speed_{result['requested_speed_mps']:.3f}"
        diagnostic_plot(result, plot, diagnostics / f"{stem}.png")
        write_rate_grid(diagnostics / f"{stem}_corrected_rate_50hz.csv", plot["rate_grid"])
        severity_name = f"{stem}_raw_reference_by_sample.csv"
        write_rate_grid(diagnostics / severity_name, plot["header_severity"])
        result["reference_evidence"]["raw_reference_by_sample_artifact"] = (
            f"diagnostics/{severity_name}")
    executed = {float(item["requested_speed_mps"]) for item in results}
    failed = [float(item["requested_speed_mps"]) for item in results if not item["passed"]]
    first_failure = min(failed) if failed else None
    skipped = [
        {"requested_speed_mps": speed, "status": "skipped_after_first_failure",
         "reason": f"sequential ladder stopped after {first_failure:.3f} m/s failed"}
        for speed in planned_speeds(args, args.experiment_root)
        if speed not in executed and first_failure is not None and speed > first_failure
    ]
    report = {
        "format": "tinympc-progress-speed-ladder-v2",
        "circle_header": str(args.circle_header.resolve()),
        "reference_provenance": "reconstructed/inferred from compiled header and measured handoff pose",
        "pass_rule": (
            "calibrated launch; firmware complete exactly 750/750; no contact/crash, nonfinite, failure signature, "
            "or route-envelope failure; uncapped additionally fails sustained cross-track >0.25m/0.5s, "
            "altitude error >0.20m/0.5s, tilt >35deg/0.25s, body rate >5rad/s/0.25s, "
            "any motor >=98%/0.1s, invalid progress jumps, or missing/mismatched uncapped banner"),
        "first_failed_speed_mps": first_failure,
        "runs": results,
        "skipped": skipped,
    }
    (args.out / "comparison.json").write_text(json.dumps(report, indent=2, allow_nan=False) + "\n")
    fields = ["requested_speed_mps", "progress_reference_limits", "evidence_label",
              "status", "passed", "calibrated_launch",
              "firmware_max_progress", "firmware_complete_750_of_750", "inferred_progress_fraction",
              "completion_time_s", "completion_time_source", "contact_or_crash", "envelope_violation", "nonfinite_state",
              "yaw_error_rmse_deg", "yaw_rate_error_rmse_radps", "corrected_euler_yaw_rate_rmse_radps",
              "corrected_body_wz_rmse_radps", "wz_minus_euler_yaw_rate_rmse_radps", "cross_track_rmse_m",
              "altitude_rmse_m", "tilt_max_deg", "rate_norm_max_radps", "rpm_saturation_fraction",
              "startup_curvature_artifact", "raw_reference_tilt_max_deg",
              "raw_reference_rodrigues_max", "progress_jump_validation", "uncapped_failure_reasons"]
    with (args.out / "comparison.csv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields); writer.writeheader()
        for item in results:
            writer.writerow({
                "requested_speed_mps": item["requested_speed_mps"],
                "progress_reference_limits": item["progress_reference_limits"],
                "evidence_label": item["evidence_label"],
                "status": item["status"], "passed": item["passed"],
                "calibrated_launch": item["launch"]["calibrated"],
                "firmware_max_progress": item["progress"]["firmware_max_sample"],
                "firmware_complete_750_of_750": item["progress"]["firmware_complete_750_of_750"],
                "inferred_progress_fraction": item["progress"]["inferred_fraction"],
                "completion_time_s": (
                    item["progress"]["firmware_completion_time_after_launch_s"]
                    if item["progress_reference_limits"] == "uncapped"
                    else item["progress"]["inferred_completion_time_after_launch_s"]),
                "completion_time_source": (
                    "firmware_diagnostic" if item["progress_reference_limits"] == "uncapped" and
                    item["progress"]["firmware_completion_time_after_launch_s"] is not None
                    else ("unavailable" if item["progress_reference_limits"] == "uncapped"
                          else "legacy_offline_inference")),
                "contact_or_crash": item["safety"]["contact_or_crash"],
                "envelope_violation": item["safety"]["envelope_violation"],
                "nonfinite_state": item["safety"]["nonfinite_state"],
                "yaw_error_rmse_deg": item["tracking"]["tangent_yaw_error_deg"]["rmse"],
                "yaw_rate_error_rmse_radps": item["tracking"]["yaw_rate_error_radps"]["rmse"],
                "corrected_euler_yaw_rate_rmse_radps": item["tracking"]["corrected_yaw_rate_50hz"]["euler_yaw_rate_minus_geometric_radps"]["rmse"],
                "corrected_body_wz_rmse_radps": item["tracking"]["corrected_yaw_rate_50hz"]["body_wz_minus_geometric_radps"]["rmse"],
                "wz_minus_euler_yaw_rate_rmse_radps": item["tracking"]["corrected_yaw_rate_50hz"]["body_wz_minus_euler_yaw_rate_radps"]["rmse"],
                "cross_track_rmse_m": item["tracking"]["cross_track_m"]["rmse"],
                "altitude_rmse_m": item["tracking"]["altitude_error_m"]["rmse"],
                "tilt_max_deg": item["attitude_rates"]["tilt_deg"]["max_abs"],
                "rate_norm_max_radps": item["attitude_rates"]["rate_norm_radps"]["max_abs"],
                "rpm_saturation_fraction": item["motors"]["saturation_fraction_samples_any_motor"],
                "startup_curvature_artifact": item["reference_evidence"]["raw_commanded_severity"]["startup_curvature_artifact"]["exposed"],
                "raw_reference_tilt_max_deg": item["reference_evidence"]["raw_commanded_severity"]["raw_bank_tilt_deg"]["max_abs"],
                "raw_reference_rodrigues_max": item["reference_evidence"]["raw_commanded_severity"]["bank_only_local_reference_rodrigues_norm"]["max_abs"],
                "progress_jump_validation": item["progress"]["jump_evidence"]["validation"],
                "uncapped_failure_reasons": ";".join(item["safety"]["uncapped_failure_reasons"]),
            })
    comparison_plot(results, plots, args.out / "comparison.png")
    print(json.dumps({str(item["requested_speed_mps"]): item["status"] for item in results}, indent=2))


if __name__ == "__main__":
    main()
