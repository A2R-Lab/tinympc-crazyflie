#!/usr/bin/env python3
"""Compare isolated TinyMPC yaw-cost changes on the first circle semicircle.

The progress controller is spatially indexed, so this analyzer does not use
wall-clock trajectory samples.  It takes the first half of the circle header's
cumulative horizontal arc length, anchors that geometry using the firmware's
reported trajectory origin/yaw, and monotonically projects measured positions
onto it.  The resulting projection supplies tangent yaw, curvature, scheduled
progress speed, cross-track error, and geometric completion.
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


ORIGIN_RE = re.compile(
    r"Trajectory origin=\(([-+0-9.eE]+),([-+0-9.eE]+),([-+0-9.eE]+)\)"
    r" yaw=([-+0-9.eE]+)deg"
)
PROGRESS_RE = re.compile(r"Progress path (?:sample|complete sample)=([-+0-9.eE]+)/([0-9]+)")
ROW_RE = re.compile(r"\{([^{}]+)\}")
FLOAT_RE = re.compile(r"[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?[fF]?")
MODES = ("baseline", "yaw_angle", "yaw_rate", "motor_differential")


def load_json(path: Path) -> dict[str, Any]:
    value = json.loads(path.read_text())
    if not isinstance(value, dict):
        raise ValueError(f"expected JSON object in {path}")
    return value


def load_state(path: Path) -> dict[str, np.ndarray]:
    with path.open(newline="") as stream:
        reader = csv.DictReader(stream)
        rows = list(reader)
        names = reader.fieldnames or []
    required = {
        "time_s", "x_m", "y_m", "z_m", "qw", "qx", "qy", "qz",
        "wz_radps", "contacts", "airborne",
    }
    missing = sorted(required - set(names))
    if missing:
        raise ValueError(f"{path} lacks columns: {', '.join(missing)}")
    if not rows:
        raise ValueError(f"no samples in {path}")
    return {
        name: np.asarray([float(row[name]) for row in rows], dtype=float)
        for name in names
    }


def load_trajectory_header(path: Path) -> np.ndarray:
    text = path.read_text()
    marker = "trajectory_reference_data"
    start = text.find(marker)
    if start < 0:
        raise ValueError(f"trajectory_reference_data not found in {path}")
    body_start = text.find("{", start)
    body_end = text.find("};", body_start)
    if body_start < 0 or body_end < 0:
        raise ValueError(f"trajectory array is incomplete in {path}")
    rows: list[list[float]] = []
    for match in ROW_RE.finditer(text[body_start + 1:body_end]):
        values = [float(token.rstrip("fF")) for token in FLOAT_RE.findall(match.group(1))]
        if len(values) == 13:
            rows.append(values)
    if len(rows) < 3:
        raise ValueError(f"fewer than three 13-state samples parsed from {path}")
    return np.asarray(rows, dtype=float)


def quaternion_yaw(qw: np.ndarray, qx: np.ndarray, qy: np.ndarray,
                   qz: np.ndarray) -> np.ndarray:
    return np.arctan2(
        2.0 * (qw * qz + qx * qy),
        1.0 - 2.0 * (qy * qy + qz * qz),
    )


def wrap_angle(angle: np.ndarray) -> np.ndarray:
    return (angle + np.pi) % (2.0 * np.pi) - np.pi


def semicircle_reference(raw: np.ndarray) -> np.ndarray:
    """Return a horizontal-arc-length-exact first half of a closed circle."""
    points = raw[:, :3]
    keep = np.r_[True, np.linalg.norm(np.diff(points[:, :2], axis=0), axis=1) > 1e-9]
    points = points[keep]
    if len(points) < 3:
        raise ValueError("trajectory has insufficient distinct horizontal points")
    lengths = np.linalg.norm(np.diff(points[:, :2], axis=0), axis=1)
    arc = np.r_[0.0, np.cumsum(lengths)]
    target = 0.5 * arc[-1]
    upper = int(np.searchsorted(arc, target, side="left"))
    if upper == 0:
        raise ValueError("trajectory has zero horizontal arc length")
    result = points[:upper + 1].copy()
    if arc[upper] > target:
        alpha = (target - arc[upper - 1]) / (arc[upper] - arc[upper - 1])
        result[-1] = points[upper - 1] + alpha * (points[upper] - points[upper - 1])
    return result


def anchor_reference(local: np.ndarray, firmware: str, state: dict[str, np.ndarray],
                     launch_index: int) -> tuple[np.ndarray, dict[str, Any]]:
    match = ORIGIN_RE.search(firmware)
    if match:
        ox, oy, oz, yaw_deg = map(float, match.groups())
        source = "firmware Trajectory origin log"
        yaw = math.radians(yaw_deg)
        origin = np.asarray([ox, oy, oz], dtype=float)
    else:
        yaw = float(quaternion_yaw(
            state["qw"][launch_index:launch_index + 1],
            state["qx"][launch_index:launch_index + 1],
            state["qy"][launch_index:launch_index + 1],
            state["qz"][launch_index:launch_index + 1],
        )[0])
        c, s = math.cos(yaw), math.sin(yaw)
        rotated_first = np.asarray([
            c * local[0, 0] - s * local[0, 1],
            s * local[0, 0] + c * local[0, 1], local[0, 2],
        ])
        measured = np.asarray([state[key][launch_index] for key in ("x_m", "y_m", "z_m")])
        origin = measured - rotated_first
        source = "inferred from measured launch pose and launch yaw"
    c, s = math.cos(yaw), math.sin(yaw)
    world = np.empty_like(local)
    world[:, 0] = origin[0] + c * local[:, 0] - s * local[:, 1]
    world[:, 1] = origin[1] + s * local[:, 0] + c * local[:, 1]
    world[:, 2] = origin[2] + local[:, 2]
    return world, {
        "source": source,
        "origin_world_m": origin.tolist(),
        "yaw_world_deg": math.degrees(yaw),
    }


def reference_geometry(reference: np.ndarray) -> dict[str, np.ndarray | float]:
    delta = np.diff(reference, axis=0)
    horizontal_length = np.linalg.norm(delta[:, :2], axis=1)
    valid = horizontal_length > 1e-9
    if not np.all(valid):
        reference = reference[np.r_[True, valid]]
        delta = np.diff(reference, axis=0)
        horizontal_length = np.linalg.norm(delta[:, :2], axis=1)
    tangent = delta[:, :2] / horizontal_length[:, None]
    curvature = np.zeros(len(delta), dtype=float)
    if len(delta) > 1:
        turn = np.arctan2(
            tangent[:-1, 0] * tangent[1:, 1] - tangent[:-1, 1] * tangent[1:, 0],
            np.einsum("ij,ij->i", tangent[:-1], tangent[1:]),
        )
        local = turn / (0.5 * (horizontal_length[:-1] + horizontal_length[1:]))
        curvature[:-1] = local
        curvature[-1] = local[-1]
    speed = np.clip(0.15 / (1.0 + 0.75 * np.abs(curvature)), 0.05, 0.15)
    return {
        "reference": reference,
        "segment": delta,
        "segment_length": horizontal_length,
        "arc": np.r_[0.0, np.cumsum(horizontal_length)],
        "tangent": tangent,
        "curvature": curvature,
        "speed": speed,
        "total_length": float(np.sum(horizontal_length)),
    }


def project_monotonic(points: np.ndarray, geometry: dict[str, Any],
                      chunk_size: int = 2048) -> dict[str, np.ndarray]:
    reference = geometry["reference"]
    segment = geometry["segment"]
    length = geometry["segment_length"]
    arc = geometry["arc"]
    raw_s_parts: list[np.ndarray] = []
    for first in range(0, len(points), chunk_size):
        xy = points[first:first + chunk_size, :2]
        relative = xy[:, None, :] - reference[None, :-1, :2]
        alpha = np.clip(
            np.einsum("csi,si->cs", relative, segment[:, :2]) / (length * length)[None, :],
            0.0, 1.0,
        )
        projected = reference[None, :-1, :2] + alpha[:, :, None] * segment[None, :, :2]
        distance_sq = np.sum((xy[:, None, :] - projected) ** 2, axis=2)
        best = np.argmin(distance_sq, axis=1)
        raw_s_parts.append(arc[best] + alpha[np.arange(len(xy)), best] * length[best])
    raw_s = np.concatenate(raw_s_parts)
    monotonic_s = np.maximum.accumulate(raw_s)
    monotonic_s = np.clip(monotonic_s, 0.0, float(geometry["total_length"]))
    index = np.searchsorted(arc, monotonic_s, side="right") - 1
    index = np.clip(index, 0, len(segment) - 1)
    alpha = (monotonic_s - arc[index]) / length[index]
    projected = reference[index] + alpha[:, None] * segment[index]
    return {
        "arc_m": monotonic_s,
        "segment_index": index,
        "projected": projected,
        "cross_track_m": np.linalg.norm(points[:, :2] - projected[:, :2], axis=1),
    }


def rms(values: np.ndarray) -> float:
    return float(np.sqrt(np.mean(np.square(values))))


def cost_metadata(mode: str, config: dict[str, Any]) -> dict[str, Any]:
    selected = {
        key: value for key, value in config.items()
        if any(token in key.lower() for token in ("cost", "penalty", "yaw", "differential"))
    }
    return {"mode": mode, "run_config_fields": selected}


def analyze_one(mode: str, run: Path, raw_reference: np.ndarray) -> tuple[dict[str, Any], dict[str, np.ndarray]]:
    required = ("run_config.json", "summary.json", "state.csv", "firmware.log")
    missing = [name for name in required if not (run / name).is_file()]
    if missing:
        raise ValueError(f"{run} lacks required files: {', '.join(missing)}")
    config = load_json(run / "run_config.json")
    summary = load_json(run / "summary.json")
    state = load_state(run / "state.csv")
    firmware = (run / "firmware.log").read_text(errors="replace")
    time = state["time_s"]
    airborne = np.flatnonzero(state["airborne"] > 0.5)
    requested_launch = float(config.get("launch_time_s", 0.0))
    launch_index = int(airborne[0]) if airborne.size else int(np.searchsorted(time, requested_launch))
    launch_index = min(launch_index, len(time) - 1)
    actual_launch = float(summary.get("launch_time_s", time[launch_index]))
    valid_launch = bool(airborne.size and abs(actual_launch - requested_launch) <= 0.5)

    local_half = semicircle_reference(raw_reference)
    reference, anchor = anchor_reference(local_half, firmware, state, launch_index)
    geometry = reference_geometry(reference)
    after_launch = np.arange(launch_index, len(time))
    positions = np.column_stack([state[key][after_launch] for key in ("x_m", "y_m", "z_m")])
    projection = project_monotonic(positions, geometry)
    progress_fraction_all = projection["arc_m"] / float(geometry["total_length"])
    completion_relative = np.flatnonzero(progress_fraction_all >= 0.99)
    contact_relative = np.flatnonzero(state["contacts"][after_launch] > 0.0)
    completion_index = int(completion_relative[0]) if completion_relative.size else None
    contact_index = int(contact_relative[0]) if contact_relative.size else None
    end_relative = len(after_launch) - 1
    if completion_index is not None:
        end_relative = min(end_relative, completion_index)
    if contact_index is not None:
        end_relative = min(end_relative, contact_index)
    window = slice(0, end_relative + 1)
    indices = after_launch[window]
    segment_index = projection["segment_index"][window]
    desired_yaw = np.arctan2(
        geometry["tangent"][segment_index, 1], geometry["tangent"][segment_index, 0]
    )
    measured_yaw = quaternion_yaw(
        state["qw"][indices], state["qx"][indices], state["qy"][indices], state["qz"][indices]
    )
    yaw_error = wrap_angle(measured_yaw - desired_yaw)
    desired_yaw_rate = (
        geometry["curvature"][segment_index] * geometry["speed"][segment_index]
    )
    yaw_rate_error = state["wz_radps"][indices] - desired_yaw_rate
    altitude_error = positions[window, 2] - projection["projected"][window, 2]
    contact = bool(contact_index is not None or summary.get("crashed", False))
    progress_matches = [(float(a), int(b)) for a, b in PROGRESS_RE.findall(firmware)]
    firmware_progress = max((value / total for value, total in progress_matches if total > 0), default=None)
    completion_time = (
        float(time[after_launch[completion_index]] - time[launch_index])
        if completion_index is not None else None
    )
    crash_position = None
    crash_time = None
    if contact_index is not None:
        crash_global = after_launch[contact_index]
        crash_position = [float(state[key][crash_global]) for key in ("x_m", "y_m", "z_m")]
        crash_time = float(time[crash_global])
    result: dict[str, Any] = {
        "mode": mode,
        "run_directory": str(run.resolve()),
        "cost_metadata": cost_metadata(mode, config),
        "valid_launch": valid_launch,
        "requested_launch_time_s": requested_launch,
        "actual_launch_time_s": actual_launch,
        "contact": contact,
        "first_contact_time_s": crash_time,
        "first_contact_position_m": crash_position,
        "semicircle_complete": completion_index is not None,
        "semicircle_completion_time_after_launch_s": completion_time,
        "geometric_progress_fraction": float(progress_fraction_all[end_relative]),
        "firmware_full_circle_progress_fraction": firmware_progress,
        "evaluation_samples": int(len(indices)),
        "evaluation_end_time_s": float(time[indices[-1]]),
        "tangent_yaw_mae_deg": float(np.degrees(np.mean(np.abs(yaw_error)))),
        "tangent_yaw_rmse_deg": float(np.degrees(rms(yaw_error))),
        "yaw_rate_rmse_rad_s": rms(yaw_rate_error),
        "cross_track_rmse_m": rms(projection["cross_track_m"][window]),
        "altitude_error_rmse_m": rms(altitude_error),
        "anchor": anchor,
    }
    plot_data = {
        "time_s": time[indices] - time[launch_index],
        "position": positions[window],
        "reference": reference,
        "measured_yaw_deg": np.degrees(np.unwrap(measured_yaw)),
        "desired_yaw_deg": np.degrees(np.unwrap(desired_yaw)),
        "yaw_error_deg": np.degrees(yaw_error),
    }
    return result, plot_data


def relative_improvement(baseline: float, candidate: float) -> float | None:
    if baseline <= 1e-12:
        return None
    return (baseline - candidate) / baseline


def relative_degradation(baseline: float, candidate: float) -> float | None:
    if baseline <= 1e-12:
        return 0.0 if candidate <= 1e-12 else None
    return (candidate - baseline) / baseline


def evaluate_promising(results: list[dict[str, Any]]) -> None:
    baseline = next(item for item in results if item["mode"] == "baseline")
    baseline["comparison_to_baseline"] = None
    baseline["promising"] = None
    for item in results:
        if item is baseline:
            continue
        yaw_gain = relative_improvement(
            float(baseline["tangent_yaw_mae_deg"]), float(item["tangent_yaw_mae_deg"])
        )
        rate_gain = relative_improvement(
            float(baseline["yaw_rate_rmse_rad_s"]), float(item["yaw_rate_rmse_rad_s"])
        )
        cross_degrade = relative_degradation(
            float(baseline["cross_track_rmse_m"]), float(item["cross_track_rmse_m"])
        )
        altitude_degrade = relative_degradation(
            float(baseline["altitude_error_rmse_m"]), float(item["altitude_error_rmse_m"])
        )
        improvement_pass = bool(
            (yaw_gain is not None and yaw_gain >= 0.15)
            or (rate_gain is not None and rate_gain >= 0.15)
        )
        comparison = {
            "yaw_mae_improvement_fraction": yaw_gain,
            "yaw_rate_rmse_improvement_fraction": rate_gain,
            "cross_track_degradation_fraction": cross_degrade,
            "altitude_degradation_fraction": altitude_degrade,
            "improvement_at_least_15_percent": improvement_pass,
            "no_contact": not bool(item["contact"]),
            "cross_track_degradation_at_most_20_percent": (
                cross_degrade is not None and cross_degrade <= 0.20
            ),
            "altitude_degradation_at_most_20_percent": (
                altitude_degrade is not None and altitude_degrade <= 0.20
            ),
        }
        item["comparison_to_baseline"] = comparison
        item["promising"] = bool(all(comparison[key] for key in (
            "improvement_at_least_15_percent", "no_contact",
            "cross_track_degradation_at_most_20_percent",
            "altitude_degradation_at_most_20_percent",
        )))


def make_plot(results: list[dict[str, Any]], plot_data: dict[str, dict[str, np.ndarray]],
              output: Path) -> None:
    colors = {
        "baseline": "#222222", "yaw_angle": "#0072b2",
        "yaw_rate": "#009e73", "motor_differential": "#d55e00",
    }
    figure, axes = plt.subplots(1, 3, figsize=(17, 5.2))
    reference = plot_data["baseline"]["reference"]
    axes[0].plot(reference[:, 0], reference[:, 1], "--", color="#888888", label="semicircle reference")
    for result in results:
        mode = str(result["mode"])
        data = plot_data[mode]
        axes[0].plot(data["position"][:, 0], data["position"][:, 1], color=colors[mode], label=mode)
        crash = result["first_contact_position_m"]
        if crash is not None:
            axes[0].scatter(crash[0], crash[1], marker="X", s=90, color=colors[mode], edgecolor="black")
        axes[1].plot(data["time_s"], data["measured_yaw_deg"], color=colors[mode], label=f"{mode} measured")
        axes[2].plot(data["time_s"], data["yaw_error_deg"], color=colors[mode], label=mode)
    axes[1].plot(
        plot_data["baseline"]["time_s"], plot_data["baseline"]["desired_yaw_deg"],
        "--", color="#888888", label="tangent reference",
    )
    axes[0].set(title="Top-down semicircle", xlabel="world x (m)", ylabel="world y (m)")
    axes[0].axis("equal")
    axes[1].set(title="Yaw tracking", xlabel="time after handoff (s)", ylabel="unwrapped yaw (deg)")
    axes[2].set(title="Wrapped tangent-yaw error", xlabel="time after handoff (s)", ylabel="error (deg)")
    for axis in axes:
        axis.grid(True, alpha=0.25)
        axis.legend(fontsize=8)
    figure.tight_layout()
    figure.savefig(output, dpi=180)
    plt.close(figure)


def write_csv(results: list[dict[str, Any]], path: Path) -> None:
    fields = [
        "mode", "valid_launch", "contact", "semicircle_complete",
        "semicircle_completion_time_after_launch_s", "geometric_progress_fraction",
        "firmware_full_circle_progress_fraction", "tangent_yaw_mae_deg",
        "tangent_yaw_rmse_deg", "yaw_rate_rmse_rad_s", "cross_track_rmse_m",
        "altitude_error_rmse_m", "promising",
    ]
    with path.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows({key: item.get(key) for key in fields} for item in results)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--baseline", type=Path, required=True)
    parser.add_argument("--yaw-angle", type=Path, required=True)
    parser.add_argument("--yaw-rate", type=Path, required=True)
    parser.add_argument("--motor-differential", type=Path, required=True)
    parser.add_argument("--trajectory-header", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    raw_reference = load_trajectory_header(args.trajectory_header)
    run_paths = {
        "baseline": args.baseline, "yaw_angle": args.yaw_angle,
        "yaw_rate": args.yaw_rate, "motor_differential": args.motor_differential,
    }
    results: list[dict[str, Any]] = []
    plots: dict[str, dict[str, np.ndarray]] = {}
    for mode in MODES:
        result, plot = analyze_one(mode, run_paths[mode], raw_reference)
        results.append(result)
        plots[mode] = plot
    evaluate_promising(results)
    args.out.mkdir(parents=True, exist_ok=True)
    report = {
        "format": "tinympc-yaw-cost-semicircle-comparison-v1",
        "trajectory_header": str(args.trajectory_header.resolve()),
        "methodology": {
            "window": "from measured airborne handoff through first 99% geometric semicircle completion, first contact, or final sample",
            "semicircle": "first 50% of circle header cumulative horizontal arc length",
            "progress": "global nearest polyline projection followed by cumulative maximum; inferred because state.csv does not log controller progress",
            "yaw_reference": "horizontal tangent at inferred monotonic progress",
            "yaw_rate_reference": "polyline curvature * clip(0.15/(1+0.75*abs(curvature)), 0.05, 0.15)",
            "promising_rule": "yaw MAE or yaw-rate RMSE improves >=15%; no contact; cross-track and altitude RMSE degradation each <=20%",
            "launch_validity": "airborne sample exists and summary launch is within 0.5 s of run_config request",
        },
        "results": results,
    }
    (args.out / "comparison.json").write_text(json.dumps(report, indent=2, allow_nan=False) + "\n")
    write_csv(results, args.out / "comparison.csv")
    make_plot(results, plots, args.out / "comparison.png")
    print(json.dumps({item["mode"]: item["promising"] for item in results}, indent=2))


if __name__ == "__main__":
    main()
