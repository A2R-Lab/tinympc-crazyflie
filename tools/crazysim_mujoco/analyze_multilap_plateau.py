#!/usr/bin/env python3
"""Analyze direct-motor TinyMPC continuous-circle speed plateau trials.

The active flight window is reconstructed from airborne telemetry and measured,
wrap-aware projection onto the closed compiled route.  Lap statistics therefore
do not include terminal braking.  Optional ``MULTILAP``/``PROGRESS`` firmware
key-value diagnostics add commanded-speed and invariant evidence, but never
replace measured motion as the source of completed laps.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
import re
from typing import Any

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

import analyze_progress_speed_ladder as ladder


MAX_LAPS = 8
MIN_LAPS = 3
MAX_MEDIAN_CHANGE_MPS = 0.05
MAX_ABS_SLOPE_MPS2 = 0.05
KEY_VALUE_RE = re.compile(
    r"([A-Za-z_][A-Za-z0-9_]*)\s*=\s*"
    r"([-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?)")
MAX_RPM_RE = re.compile(r"max_rpm\s*:\s*([0-9.]+)")
FAILURE_RE = re.compile(
    r"\b(?:nan|inf|assert|hardfault|solver (?:fail|error)|infeasible|"
    r"segmentation fault)\b", re.IGNORECASE)
SHA256_RE = re.compile(r"^[0-9a-f]{64}$")


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def finite_stats(values: np.ndarray) -> dict[str, float | None]:
    finite = values[np.isfinite(values)]
    if not finite.size:
        return {key: None for key in ("mean", "median", "p10", "p90", "max")}
    return {
        "mean": float(np.mean(finite)),
        "median": float(np.median(finite)),
        "p10": float(np.percentile(finite, 10)),
        "p90": float(np.percentile(finite, 90)),
        "max": float(np.max(finite)),
    }


def least_squares_slope(time_s: np.ndarray, values: np.ndarray) -> float | None:
    valid = np.isfinite(time_s) & np.isfinite(values)
    if np.count_nonzero(valid) < 2 or np.ptp(time_s[valid]) <= 1e-9:
        return None
    centered = time_s[valid] - float(np.mean(time_s[valid]))
    denominator = float(np.dot(centered, centered))
    return float(np.dot(centered, values[valid] - np.mean(values[valid])) / denominator)


def parse_firmware_diagnostics(text: str) -> list[dict[str, float]]:
    """Parse relevant timestamped key-value records without fixing one spelling."""
    records: list[dict[str, float]] = []
    for line in text.splitlines():
        upper = line.upper()
        if "MULTILAP" not in upper and "MULTI-LAP" not in upper and "PROGRESS" not in upper:
            continue
        values = {key.lower(): float(value) for key, value in KEY_VALUE_RE.findall(line)}
        if values:
            records.append(values)
    return records


def diagnostic_series(records: list[dict[str, float]], aliases: tuple[str, ...]
                      ) -> tuple[np.ndarray, np.ndarray]:
    times: list[float] = []
    values: list[float] = []
    time_aliases = ("time_after_launch_s", "elapsed_s", "t_after_launch", "t")
    for record in records:
        time_value = next((record[key] for key in time_aliases if key in record), None)
        value = next((record[key] for key in aliases if key in record), None)
        if time_value is not None and value is not None:
            times.append(time_value)
            values.append(value)
    return np.asarray(times), np.asarray(values)


def closed_route_projection(points: np.ndarray, route: np.ndarray,
                            search_half_width: int = 80) -> dict[str, np.ndarray]:
    """Project onto a periodic route while retaining signed measured motion.

    The search is local and periodic.  The wrapped coordinate is unwrapped via
    the shortest modular displacement; it is not forced to advance and thus
    cannot invent forward progress when the vehicle stops or moves backward.
    """
    segment = np.diff(route[:, :3], axis=0)
    length2 = np.einsum("ij,ij->i", segment, segment)
    valid = length2 > 1e-12
    if len(segment) < 3 or not np.any(valid):
        raise ValueError("closed route has insufficient nonzero segments")
    period = len(segment)
    wrapped = np.zeros(len(points))
    cumulative = np.zeros(len(points))
    projected = np.empty_like(points)
    indices = np.zeros(len(points), dtype=int)
    previous: float | None = None
    previous_index = 0
    for sample, point in enumerate(points):
        if previous is None:
            candidates = np.arange(period)
        else:
            candidates = np.unique(np.mod(
                np.arange(previous_index - search_half_width,
                          previous_index + search_half_width + 1), period))
        candidates = candidates[valid[candidates]]
        starts = route[candidates, :3]
        vectors = segment[candidates]
        alpha = np.clip(np.einsum("ij,ij->i", point - starts, vectors)
                        / length2[candidates], 0.0, 1.0)
        candidates_projected = starts + alpha[:, None] * vectors
        nearest = int(np.argmin(np.sum((point - candidates_projected) ** 2, axis=1)))
        index = int(candidates[nearest])
        coordinate = float(index + alpha[nearest])
        if coordinate >= period:
            coordinate -= period
        if previous is None:
            delta = coordinate
        else:
            delta = (coordinate - previous + 0.5 * period) % period - 0.5 * period
        wrapped[sample] = coordinate
        cumulative[sample] = (cumulative[sample - 1] + delta) if sample else delta
        projected[sample] = candidates_projected[nearest]
        indices[sample] = index
        previous = coordinate
        previous_index = index
    return {
        "wrapped_sample": wrapped,
        "cumulative_sample": cumulative,
        "projected": projected,
        "segment_index": indices,
    }


def completed_lap_windows(cumulative_sample: np.ndarray, period: int,
                          maximum_laps: int = MAX_LAPS) -> list[tuple[int, int]]:
    """Return measured sample windows for complete forward laps."""
    windows: list[tuple[int, int]] = []
    nonnegative = np.flatnonzero(cumulative_sample >= 0.0)
    start = int(nonnegative[0]) if nonnegative.size else 0
    previous_end = start
    for lap in range(maximum_laps):
        threshold = float((lap + 1) * period)
        crossings = np.flatnonzero(cumulative_sample[previous_end:] >= threshold - 1e-6)
        if not crossings.size:
            break
        end = previous_end + int(crossings[0])
        if end > previous_end:
            windows.append((previous_end, end))
        previous_end = end
    return windows


def detect_plateau(laps: list[dict[str, Any]]) -> dict[str, Any]:
    evaluation: list[dict[str, Any]] = []
    first_plateau_lap: int | None = None
    for index in range(2, len(laps)):
        medians = [float(laps[item]["tangential_speed_mps"]["median"])
                   for item in (index - 2, index - 1, index)]
        slopes = [laps[item]["within_lap_slope_mps2"] for item in (index - 1, index)]
        changes = [abs(medians[1] - medians[0]), abs(medians[2] - medians[1])]
        passed = bool(
            all(change <= MAX_MEDIAN_CHANGE_MPS for change in changes)
            and all(slope is not None and abs(float(slope)) <= MAX_ABS_SLOPE_MPS2
                    for slope in slopes))
        item = {
            "through_lap": index + 1,
            "latest_two_median_changes_mps": changes,
            "latest_two_abs_slopes_mps2": [None if value is None else abs(float(value))
                                             for value in slopes],
            "passed": passed,
        }
        evaluation.append(item)
        if passed and first_plateau_lap is None:
            first_plateau_lap = index + 1
    plateau_laps = (laps[:first_plateau_lap] if first_plateau_lap is not None else laps)
    plateau_values = np.asarray([
        item["tangential_speed_mps"]["median"] for item in plateau_laps[-2:]
    ], dtype=float)
    return {
        "established": first_plateau_lap is not None,
        "first_established_through_lap": first_plateau_lap,
        "plateau_median_mps": (float(np.mean(plateau_values))
                                if first_plateau_lap is not None else None),
        "latest_two_lap_median_range_mps": (
            float(np.ptp(plateau_values)) if first_plateau_lap is not None else None),
        "criteria": {
            "minimum_complete_laps": MIN_LAPS,
            "maximum_laps": MAX_LAPS,
            "maximum_each_of_two_median_changes_mps": MAX_MEDIAN_CHANGE_MPS,
            "maximum_abs_slope_on_each_latest_two_laps_mps2": MAX_ABS_SLOPE_MPS2,
        },
        "evaluations": evaluation,
    }


def _config_value(config: dict[str, Any], names: tuple[str, ...], default: float) -> float:
    for name in names:
        if name in config and config[name] not in (None, ""):
            return float(config[name])
    return default


def validate_run_contract(config: dict[str, Any], target_speed: float,
                          acceleration: float) -> list[str]:
    """Verify the experiment is the requested direct-motor configuration."""
    errors: list[str] = []
    expected = {
        "trajectory": "circle", "reference_mode": "progress",
        "actuator_lti": True, "rate_cascade": False,
        "stop_on_contact": True, "progress_reference_limits": "uncapped",
    }
    for key, value in expected.items():
        if config.get(key) != value:
            errors.append(f"{key} must be {value!r}, found {config.get(key)!r}")
    if not math.isclose(target_speed, 1.0, rel_tol=0.0, abs_tol=1e-9):
        errors.append(f"progress_speed_mps must be 1.0, found {target_speed!r}")
    if "progress_speed_mps" not in config:
        errors.append("progress_speed_mps is absent")
    if "progress_entry_acceleration_mps2" not in config:
        errors.append("progress_entry_acceleration_mps2 is absent")
    if not math.isfinite(acceleration) or acceleration <= 0.0 or acceleration > 0.25 + 1e-12:
        errors.append(
            f"progress_entry_acceleration_mps2 must be in (0, 0.25], found {acceleration!r}")
    laps = config.get("progress_laps")
    if isinstance(laps, bool) or not isinstance(laps, int) or not MIN_LAPS <= laps <= MAX_LAPS:
        errors.append(f"progress_laps must be an integer in [3, 8], found {laps!r}")
    contract = config.get("acceptance_contract")
    if not isinstance(contract, dict):
        errors.append("acceptance_contract is absent")
        return errors
    for field in ("controller_sha256", "firmware_binary_sha256"):
        value = contract.get(field)
        if not isinstance(value, str) or SHA256_RE.fullmatch(value) is None:
            errors.append(f"acceptance_contract.{field} is absent or malformed")
    source_hashes = config.get("controller_source_sha256")
    source_key = "apps/controller_tinympc_eigen/src/controller_tinympc.cpp"
    if not isinstance(source_hashes, dict) or source_key not in source_hashes:
        errors.append(f"controller_source_sha256 lacks {source_key}")
    elif source_hashes[source_key] != contract.get("controller_sha256"):
        errors.append("controller source hash does not match acceptance contract")
    return errors


def analyze_run(run: Path, raw_route: np.ndarray) -> tuple[dict[str, Any], dict[str, np.ndarray]]:
    run = run.resolve()
    required = [run / name for name in ("state.csv", "run_config.json", "summary.json", "firmware.log")]
    missing = [str(path) for path in required if not path.exists()]
    if missing:
        raise FileNotFoundError("missing run artifacts: " + ", ".join(missing))
    state = ladder.load_state(run / "state.csv")
    config = ladder.load_json(run / "run_config.json")
    summary = ladder.load_json(run / "summary.json")
    firmware = (run / "firmware.log").read_text(errors="replace")
    simulator_path = run / "simulator.log"
    simulator = simulator_path.read_text(errors="replace") if simulator_path.exists() else ""
    time = state["time_s"]
    airborne = np.flatnonzero(state["airborne"] > 0.5)
    requested_launch = float(config.get("launch_time_s", 0.0))
    launch = int(airborne[0]) if airborne.size else int(np.searchsorted(time, requested_launch))
    launch = min(launch, len(time) - 1)
    actual_launch = float(summary.get("launch_time_s", time[launch]))
    timing_error = actual_launch - requested_launch
    calibrated = bool(airborne.size and abs(timing_error) <= 0.5)
    route = ladder.anchor_route(raw_route, state, launch)
    period = len(route) - 1
    indices = np.arange(launch, len(time))
    finite_state_keys = (
        "time_s", "x_m", "y_m", "z_m", "qw", "qx", "qy", "qz",
        "vx_mps", "vy_mps", "vz_mps", "wx_radps", "wy_radps", "wz_radps",
        "rpm_1", "rpm_2", "rpm_3", "rpm_4", "contacts", "airborne",
    )
    # Optional packet-age columns legitimately contain NaN before the first
    # command; only physical/control telemetry determines a nonfinite failure.
    numeric = np.column_stack([state[key][indices] for key in finite_state_keys])
    finite_rows = np.all(np.isfinite(numeric), axis=1)
    first_nonfinite = np.flatnonzero(~finite_rows)
    if first_nonfinite.size:
        indices = indices[:int(first_nonfinite[0])]
    positions = np.column_stack([state[key][indices] for key in ("x_m", "y_m", "z_m")])
    projection = closed_route_projection(positions, route)
    windows = completed_lap_windows(projection["cumulative_sample"], period)
    if not len(indices):
        raise ValueError(f"no finite post-launch samples in {run}")
    contact_relative = np.flatnonzero(state["contacts"][indices] > 0.0)
    contact_at = int(contact_relative[0]) if contact_relative.size else None
    if contact_at is not None:
        windows = [window for window in windows if window[1] <= contact_at]
    geometry = ladder.route_geometry(route)
    segment_index = projection["segment_index"]
    tangent = geometry["tangent"][segment_index, :2]
    tangent /= np.maximum(np.linalg.norm(tangent, axis=1)[:, None], 1e-12)
    normal = np.column_stack([-tangent[:, 1], tangent[:, 0]])
    velocity = np.column_stack([state[key][indices] for key in ("vx_mps", "vy_mps", "vz_mps")])
    tangential_speed = np.einsum("ij,ij->i", velocity[:, :2], tangent)
    displacement = positions[:, :2] - projection["projected"][:, :2]
    cross_track = np.einsum("ij,ij->i", displacement, normal)
    altitude_error = positions[:, 2] - projection["projected"][:, 2]
    quaternion = np.column_stack([state[key][indices] for key in ("qw", "qx", "qy", "qz")])
    roll, pitch, yaw = ladder.quaternion_rpy(quaternion)
    rates = np.column_stack([state[key][indices] for key in
                             ("wx_radps", "wy_radps", "wz_radps")])
    rpm = np.column_stack([state[f"rpm_{motor}"][indices] for motor in range(1, 5)])
    rpm_ref = np.column_stack([
        state.get(f"rpm_ref_{motor}", state[f"rpm_{motor}"])[indices]
        for motor in range(1, 5)])
    max_match = MAX_RPM_RE.search(simulator)
    maximum_rpm = (float(max_match.group(1)) if max_match else
                   max(float(np.nanmax(rpm_ref)), float(np.nanmax(rpm)), 1.0))
    rpm_fraction = rpm / maximum_rpm
    rpm_ref_fraction = rpm_ref / maximum_rpm
    target_speed = _config_value(config, (
        "progress_speed_mps", "requested_speed_mps", "constant_speed_mps"), 1.0)
    acceleration = _config_value(config, (
        "progress_entry_acceleration_mps2", "progress_acceleration_limit_mps2",
        "progress_ramp_acceleration_mps2",
        "progress_accel_limit_mps2"), 0.25)
    contract_errors = validate_run_contract(config, target_speed, acceleration)
    relative_time = time[indices] - time[launch]
    command = np.minimum(target_speed, np.maximum(0.0, acceleration * relative_time))
    records = parse_firmware_diagnostics(firmware)
    diag_time, diag_command = diagnostic_series(records, (
        "command_speed_mps", "commanded_speed_mps", "reference_speed_mps", "speed_mps"))
    if len(diag_time) >= 2:
        order = np.argsort(diag_time)
        command = np.interp(relative_time, diag_time[order], diag_command[order],
                            left=diag_command[order][0], right=diag_command[order][-1])
        command_source = "firmware_timestamped_diagnostic"
    else:
        command_source = "run_config_linear_ramp_reconstruction"
    lap_results: list[dict[str, Any]] = []
    for lap_number, (start, end) in enumerate(windows, 1):
        selection = slice(start, end + 1)
        lap_time = relative_time[selection]
        lap_speed = tangential_speed[selection]
        lap_results.append({
            "lap": lap_number,
            "start_time_after_launch_s": float(lap_time[0]),
            "end_time_after_launch_s": float(lap_time[-1]),
            "duration_s": float(lap_time[-1] - lap_time[0]),
            "samples": int(len(lap_time)),
            "tangential_speed_mps": finite_stats(lap_speed),
            "within_lap_slope_mps2": least_squares_slope(lap_time, lap_speed),
            "commanded_speed_mps": finite_stats(command[selection]),
            "cross_track_error_m": {
                "rmse": float(np.sqrt(np.mean(cross_track[selection] ** 2))),
                "p95_abs": float(np.percentile(np.abs(cross_track[selection]), 95)),
            },
            "altitude_error_m": {
                "rmse": float(np.sqrt(np.mean(altitude_error[selection] ** 2))),
                "p95_abs": float(np.percentile(np.abs(altitude_error[selection]), 95)),
            },
            "motor_saturation_fraction": float(np.mean(
                np.max(rpm_ref_fraction[selection], axis=1) >= 0.98)),
        })
    plateau = detect_plateau(lap_results)
    configured_laps = int(config["progress_laps"])
    if contact_at is not None:
        analysis_end = contact_at
        active_end_reason = "contact"
    elif len(windows) >= configured_laps:
        analysis_end = windows[configured_laps - 1][1]
        active_end_reason = "configured_final_lap"
    else:
        # Preserve an incomplete active lap as failure evidence. It is not used
        # in per-lap plateau statistics, but it must remain visible in plots.
        analysis_end = len(indices) - 1
        active_end_reason = "run_end_incomplete_lap"
    active = slice(0, analysis_end + 1)
    flyaway = bool(np.any(np.abs(cross_track[active]) > 2.0)
                   or np.any(np.abs(altitude_error[active]) > 1.0))
    failures = sorted(set(match.group(0).lower() for match in FAILURE_RE.finditer(firmware)))
    diagnostic_counters: dict[str, float] = {}
    expected_counter_keys = (
        "projection_violation_count", "command_violation_count",
        "lead_violation_count")
    for key in expected_counter_keys + ("progress_violation_count",):
        values = [record[key] for record in records if key in record]
        if values:
            diagnostic_counters[key] = max(values)
    firmware_lap_events = [int(record["lap"]) for record in records if "lap" in record]
    firmware_completed_laps = max(firmware_lap_events, default=0)
    lap_completion_consistent = firmware_completed_laps == len(lap_results)
    direct_motor_fields = {
        key: config.get(key) for key in (
            "actuator_lti", "rate_cascade", "controller_architecture",
            "level_controller_architecture", "reference_mode")
        if key in config
    }
    identity = {
        "run_config_sha256": sha256(run / "run_config.json"),
        "state_csv_sha256": sha256(run / "state.csv"),
        "firmware_log_sha256": sha256(run / "firmware.log"),
        "firmware_binary_sha256": config.get("acceptance_contract", {}).get(
            "firmware_binary_sha256") if isinstance(config.get("acceptance_contract"), dict) else None,
        "controller_sha256": config.get("acceptance_contract", {}).get(
            "controller_sha256") if isinstance(config.get("acceptance_contract"), dict) else None,
        "mode_fields": direct_motor_fields,
    }
    result = {
        "run_directory": str(run),
        "random_seed": config.get("random_seed"),
        "requested_speed_mps": target_speed,
        "ramp_acceleration_limit_mps2": acceleration,
        "launch": {"requested_s": requested_launch, "actual_s": actual_launch,
                   "timing_error_s": timing_error, "calibrated": calibrated},
        "identity": identity,
        "contract_validation": {"valid": not contract_errors, "errors": contract_errors},
        "command_speed_source": command_source,
        "completed_measured_laps": len(lap_results),
        "laps": lap_results,
        "plateau": plateau,
        "safety": {
            "contact": contact_at is not None or bool(summary.get("crashed", False)),
            "flyaway": flyaway,
            "nonfinite_post_launch": bool(first_nonfinite.size),
            "failure_signatures": failures,
        },
        "progress_invariants": {
            "firmware_counter_maxima": diagnostic_counters,
            "all_expected_present_and_zero": bool(
                all(key in diagnostic_counters for key in expected_counter_keys)
                and all(value == 0.0 for value in diagnostic_counters.values())),
            "diagnostics_present": bool(records),
            "firmware_completed_laps": firmware_completed_laps,
            "firmware_measured_laps_match_offline_projection": lap_completion_consistent,
            "measured_progress_source": "wrap-aware closed-route projection; signed and not forced monotonic",
            "maximum_backward_step_samples": float(max(
                0.0, -np.min(np.diff(projection["cumulative_sample"]))))
                if len(indices) > 1 else 0.0,
        },
        "experiment_valid": bool(
            not contract_errors and calibrated and len(lap_results) >= MIN_LAPS
            and all(key in diagnostic_counters for key in expected_counter_keys)
            and all(value == 0.0 for value in diagnostic_counters.values())
            and lap_completion_consistent
            and contact_at is None
            and not summary.get("crashed", False) and not flyaway
            and not first_nonfinite.size and not failures),
        "active_window": {
            "start_time_s": float(time[indices[0]]),
            "end_time_s": float(time[indices[analysis_end]]),
            "samples": int(analysis_end + 1),
            "takeoff_excluded": True,
            "post_completion_braking_excluded": active_end_reason == "configured_final_lap",
            "ended_at": active_end_reason,
        },
    }
    plot = {
        "time": relative_time[active], "position": positions[active],
        "projected": projection["projected"][active], "route": route,
        "cumulative_laps": projection["cumulative_sample"][active] / period,
        "wrapped_progress": projection["wrapped_sample"][active] / period,
        "tangent_speed": tangential_speed[active], "command_speed": command[active],
        "cross_track": cross_track[active], "altitude_error": altitude_error[active],
        "rpy_deg": np.degrees(np.column_stack([roll[active], pitch[active], yaw[active]])),
        "rates": rates[active], "rpm_fraction": rpm_fraction[active],
        "rpm_ref_fraction": rpm_ref_fraction[active],
        "contacts": state["contacts"][indices][active],
        "lap_windows": windows,
    }
    return result, plot


def write_per_lap_csv(results: list[dict[str, Any]], path: Path) -> None:
    fields = [
        "run_directory", "random_seed", "lap", "start_time_after_launch_s",
        "end_time_after_launch_s", "duration_s", "samples", "speed_mean_mps",
        "speed_median_mps", "speed_p10_mps", "speed_p90_mps", "speed_max_mps",
        "within_lap_slope_mps2", "command_median_mps", "cross_track_rmse_m",
        "cross_track_p95_m", "altitude_rmse_m", "altitude_p95_m",
        "motor_saturation_fraction",
    ]
    with path.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for result in results:
            for lap in result["laps"]:
                speed = lap["tangential_speed_mps"]
                writer.writerow({
                    "run_directory": result["run_directory"],
                    "random_seed": result["random_seed"], "lap": lap["lap"],
                    "start_time_after_launch_s": lap["start_time_after_launch_s"],
                    "end_time_after_launch_s": lap["end_time_after_launch_s"],
                    "duration_s": lap["duration_s"], "samples": lap["samples"],
                    "speed_mean_mps": speed["mean"], "speed_median_mps": speed["median"],
                    "speed_p10_mps": speed["p10"], "speed_p90_mps": speed["p90"],
                    "speed_max_mps": speed["max"],
                    "within_lap_slope_mps2": lap["within_lap_slope_mps2"],
                    "command_median_mps": lap["commanded_speed_mps"]["median"],
                    "cross_track_rmse_m": lap["cross_track_error_m"]["rmse"],
                    "cross_track_p95_m": lap["cross_track_error_m"]["p95_abs"],
                    "altitude_rmse_m": lap["altitude_error_m"]["rmse"],
                    "altitude_p95_m": lap["altitude_error_m"]["p95_abs"],
                    "motor_saturation_fraction": lap["motor_saturation_fraction"],
                })


def diagnostic_plot(result: dict[str, Any], data: dict[str, np.ndarray], path: Path) -> None:
    fig, axes = plt.subplots(4, 3, figsize=(17, 16))
    t = data["time"]
    axes[0, 0].plot(t, data["command_speed"], "k--", label="command")
    axes[0, 0].plot(t, data["tangent_speed"], label="measured tangent")
    axes[0, 0].set_title("Tangential speed vs time"); axes[0, 0].set_ylabel("m/s")
    axes[0, 0].legend()
    axes[0, 1].plot(data["cumulative_laps"], data["command_speed"], "k--", label="command")
    axes[0, 1].plot(data["cumulative_laps"], data["tangent_speed"], label="measured")
    axes[0, 1].set_title("Tangential speed vs measured progress")
    axes[0, 1].set_xlabel("laps"); axes[0, 1].set_ylabel("m/s")
    laps = result["laps"]
    if laps:
        number = np.asarray([lap["lap"] for lap in laps])
        median = np.asarray([lap["tangential_speed_mps"]["median"] for lap in laps])
        p10 = np.asarray([lap["tangential_speed_mps"]["p10"] for lap in laps])
        p90 = np.asarray([lap["tangential_speed_mps"]["p90"] for lap in laps])
        mean = np.asarray([lap["tangential_speed_mps"]["mean"] for lap in laps])
        axes[0, 2].errorbar(number, median, yerr=[median - p10, p90 - median],
                            marker="o", capsize=3, label="median, p10–p90")
        axes[0, 2].plot(number, mean, "s--", label="mean")
        slope_axis = axes[0, 2].twinx()
        slope_axis.plot(number, [lap["within_lap_slope_mps2"] for lap in laps],
                        "d:", color="tab:red", label="slope")
        slope_axis.axhline(MAX_ABS_SLOPE_MPS2, color="tab:red", alpha=.25)
        slope_axis.axhline(-MAX_ABS_SLOPE_MPS2, color="tab:red", alpha=.25)
        slope_axis.set_ylabel("slope (m/s²)", color="tab:red")
        axes[0, 2].legend(loc="upper left")
    axes[0, 2].set_title("Per-lap speed statistics"); axes[0, 2].set_xlabel("lap")
    axes[0, 2].set_ylabel("m/s")
    axes[1, 0].plot(data["route"][:, 0], data["route"][:, 1], "k--", label="reference")
    axes[1, 0].plot(data["position"][:, 0], data["position"][:, 1], alpha=.8, label="measured")
    axes[1, 0].axis("equal"); axes[1, 0].set_title("XY trajectory")
    axes[1, 0].set_xlabel("x (m)"); axes[1, 0].set_ylabel("y (m)"); axes[1, 0].legend()
    axes[1, 1].plot(t, data["cross_track"])
    axes[1, 1].set_title("Signed cross-track error"); axes[1, 1].set_ylabel("m")
    axes[1, 2].plot(t, data["position"][:, 2], label="measured z")
    axes[1, 2].plot(t, data["position"][:, 2] - data["altitude_error"], "k--", label="reference z")
    axes[1, 2].set_title("Altitude"); axes[1, 2].set_ylabel("m"); axes[1, 2].legend()
    for index, label in enumerate(("roll", "pitch", "yaw")):
        axes[2, 0].plot(t, data["rpy_deg"][:, index], label=label)
    axes[2, 0].set_title("Attitude"); axes[2, 0].set_ylabel("deg"); axes[2, 0].legend(ncol=3)
    for index, label in enumerate(("p", "q", "r")):
        axes[2, 1].plot(t, data["rates"][:, index], label=label)
    axes[2, 1].set_title("Body rates"); axes[2, 1].set_ylabel("rad/s"); axes[2, 1].legend(ncol=3)
    for motor in range(4):
        axes[2, 2].plot(t, 100 * data["rpm_ref_fraction"][:, motor], label=f"M{motor + 1} cmd")
    axes[2, 2].axhline(98, color="r", ls="--", label="98% saturation")
    axes[2, 2].set_title("Motor commands"); axes[2, 2].set_ylabel("% max RPM")
    axes[2, 2].legend(ncol=2, fontsize=8)
    for motor in range(4):
        axes[3, 0].plot(t, 100 * data["rpm_fraction"][:, motor], label=f"M{motor + 1}")
    axes[3, 0].set_title("Realized motor speed"); axes[3, 0].set_ylabel("% max RPM")
    axes[3, 1].step(t, data["contacts"], where="post", label="contacts")
    axes[3, 1].plot(t, np.max(data["rpm_ref_fraction"], axis=1) >= .98,
                    label="any motor ≥98%", alpha=.8)
    axes[3, 1].set_title("Contacts and saturation"); axes[3, 1].legend()
    axes[3, 2].plot(t, data["cumulative_laps"], label="signed measured progress")
    axes[3, 2].plot(t, np.maximum.accumulate(data["cumulative_laps"]), "k--",
                    label="monotonic envelope", alpha=.7)
    counters = result["progress_invariants"]["firmware_counter_maxima"]
    axes[3, 2].set_title("Measured progress / invariants: " +
                         (", ".join(f"{key}={value:g}" for key, value in counters.items())
                          if counters else "firmware counters absent"))
    axes[3, 2].set_ylabel("laps"); axes[3, 2].legend(fontsize=8)
    for axis in axes.flat:
        axis.grid(True, alpha=.25)
        if axis not in axes[0, 1:3] and axis not in (axes[1, 0],):
            axis.set_xlabel("time after launch (s)")
    status = (f"seed {result['random_seed']} | {result['completed_measured_laps']} complete laps | "
              f"plateau={result['plateau']['established']} | valid={result['experiment_valid']}")
    fig.suptitle("Direct-motor TinyMPC multi-lap plateau analysis — " + status)
    fig.tight_layout(rect=(0, 0, 1, .97)); fig.savefig(path, dpi=150); plt.close(fig)


def summary_plot(results: list[dict[str, Any]], path: Path) -> None:
    fig, axes = plt.subplots(1, 2, figsize=(13, 5))
    for result in results:
        laps = result["laps"]
        if not laps:
            continue
        label = f"seed {result['random_seed']}"
        number = [lap["lap"] for lap in laps]
        median = [lap["tangential_speed_mps"]["median"] for lap in laps]
        axes[0].plot(number, median, "o-", label=label)
        axes[1].plot(number, [lap["within_lap_slope_mps2"] for lap in laps], "o-", label=label)
    axes[0].axhline(1.0, color="k", ls="--", label="command")
    axes[0].set_title("Per-lap median tangential speed"); axes[0].set_ylabel("m/s")
    axes[1].axhspan(-MAX_ABS_SLOPE_MPS2, MAX_ABS_SLOPE_MPS2, color="green", alpha=.15)
    axes[1].set_title("Least-squares within-lap speed slope"); axes[1].set_ylabel("m/s²")
    for axis in axes:
        axis.set_xlabel("completed measured lap"); axis.grid(True, alpha=.25); axis.legend()
    fig.tight_layout(); fig.savefig(path, dpi=150); plt.close(fig)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run", action="append", type=Path, required=True,
                        help="Run directory; repeat once per seed")
    parser.add_argument("--circle-header", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    args.out.mkdir(parents=True, exist_ok=True)
    diagnostics = args.out / "diagnostics"
    diagnostics.mkdir(exist_ok=True)
    route = ladder.load_header(args.circle_header)
    results: list[dict[str, Any]] = []
    for run in args.run:
        result, plot = analyze_run(run, route)
        results.append(result)
        seed = result["random_seed"]
        diagnostic_plot(result, plot, diagnostics / f"seed_{seed}.png")
    valid_plateaus = [item["plateau"]["plateau_median_mps"] for item in results
                      if item["experiment_valid"] and item["plateau"]["established"]]
    controller_hashes = {item["identity"]["controller_sha256"] for item in results}
    firmware_hashes = {item["identity"]["firmware_binary_sha256"] for item in results}
    configured_header_hashes = {
        ladder.load_json(Path(item["run_directory"]) / "run_config.json").get(
            "trajectory_header_sha256") for item in results
    }
    exact_header_hash = sha256(args.circle_header)
    matrix_identity_valid = bool(
        len(controller_hashes) == 1 and None not in controller_hashes
        and len(firmware_hashes) == 1 and None not in firmware_hashes
        and configured_header_hashes == {exact_header_hash})
    report = {
        "format": "tinympc-direct-motor-multilap-plateau-v1",
        "circle_header": str(args.circle_header.resolve()),
        "circle_header_sha256": sha256(args.circle_header),
        "runs": results,
        "aggregate": {
            "run_count": len(results),
            "valid_run_count": sum(item["experiment_valid"] for item in results),
            "plateau_run_count": len(valid_plateaus),
            "all_three_valid_plateaus": (
                len(results) == 3 and len(valid_plateaus) == 3 and matrix_identity_valid),
            "matrix_identity_consistent": matrix_identity_valid,
            "controller_sha256": next(iter(controller_hashes)) if len(controller_hashes) == 1 else None,
            "firmware_binary_sha256": next(iter(firmware_hashes)) if len(firmware_hashes) == 1 else None,
            "plateau_speed_mean_across_seeds_mps": (
                float(np.mean(valid_plateaus)) if valid_plateaus else None),
            "plateau_speed_range_across_seeds_mps": (
                float(np.ptp(valid_plateaus)) if valid_plateaus else None),
            "conclusion": (
                "plateau established in all three valid seeds" if len(results) == 3
                and len(valid_plateaus) == 3 else
                "plateau not established in all three valid seeds; inspect per-run failure/evaluation evidence"),
        },
    }
    (args.out / "plateau_report.json").write_text(
        json.dumps(report, indent=2, sort_keys=True, allow_nan=False) + "\n")
    write_per_lap_csv(results, args.out / "per_lap.csv")
    summary_plot(results, args.out / "plateau_summary.png")
    print(json.dumps(report["aggregate"], indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
