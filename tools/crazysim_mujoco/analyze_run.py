#!/usr/bin/env python3
"""Summarize and plot a CrazySim TinyMPC ground-truth run."""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


def load_numeric_csv(path: Path) -> dict[str, np.ndarray]:
    with path.open(newline="") as stream:
        reader = csv.DictReader(stream)
        rows = list(reader)
    if not rows:
        raise ValueError(f"No samples in {path}")
    return {
        name: np.asarray([float(row[name]) for row in rows], dtype=float)
        for name in reader.fieldnames or []
    }


def quaternion_to_euler_deg(qw, qx, qy, qz):
    sinr_cosp = 2.0 * (qw * qx + qy * qz)
    cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = np.arctan2(sinr_cosp, cosr_cosp)
    sinp = np.clip(2.0 * (qw * qy - qz * qx), -1.0, 1.0)
    pitch = np.arcsin(sinp)
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = np.arctan2(siny_cosp, cosy_cosp)
    return np.rad2deg(roll), np.rad2deg(pitch), np.rad2deg(yaw)


def point_to_polyline_distance(points: np.ndarray, line: np.ndarray) -> np.ndarray:
    result = np.full(len(points), np.inf, dtype=float)
    for first, second in zip(line[:-1], line[1:]):
        segment = second - first
        denominator = float(np.dot(segment, segment))
        if denominator <= 0.0:
            continue
        alpha = np.clip(((points - first) @ segment) / denominator, 0.0, 1.0)
        projected = first + alpha[:, None] * segment
        result = np.minimum(result, np.linalg.norm(points - projected, axis=1))
    return result


def clearance_to_obstacle(points: np.ndarray, obstacle: dict, radius: float) -> np.ndarray:
    center = np.asarray(obstacle["center"], dtype=float)
    relative = points - center
    if obstacle["shape"] == "circle":
        return np.linalg.norm(relative, axis=1) - float(obstacle["radius"]) - radius
    if obstacle["shape"] != "box":
        raise ValueError(f"unsupported obstacle shape {obstacle['shape']}")
    half = np.asarray(obstacle["half_size"], dtype=float)
    outside_vector = np.maximum(np.abs(relative) - half, 0.0)
    outside = np.linalg.norm(outside_vector, axis=1)
    inside = np.all(np.abs(relative) <= half, axis=1)
    signed = outside
    signed[inside] = -np.min(half - np.abs(relative[inside]), axis=1)
    return signed - radius


def ordered_gate_crossings(
    data: dict[str, np.ndarray], gates: list[dict], launch_time: float,
    vehicle_radius: float,
) -> tuple[list[dict], bool]:
    """Find physically valid forward crossings, preserving declared gate order."""
    points = np.column_stack((data["x_m"], data["y_m"]))
    z = data["z_m"]
    t = data["time_s"]
    search_from = max(0, int(np.searchsorted(t, launch_time)))
    results = []
    for gate in gates:
        center = np.asarray(gate["center"][:2], dtype=float)
        normal = np.asarray(gate["normal"], dtype=float)
        normal /= np.linalg.norm(normal)
        lateral_axis = np.asarray([-normal[1], normal[0]])
        signed = (points - center) @ normal
        crossings = np.flatnonzero(
            (signed[:-1] <= 0.0) & (signed[1:] > 0.0)
            & (np.arange(len(signed) - 1) >= search_from)
        )
        best = None
        for index in crossings:
            denominator = signed[index + 1] - signed[index]
            alpha = -signed[index] / denominator if denominator else 0.0
            crossing_xy = points[index] + alpha * (points[index + 1] - points[index])
            crossing_z = z[index] + alpha * (z[index + 1] - z[index])
            lateral_error = abs(float((crossing_xy - center) @ lateral_axis))
            vertical_error = abs(float(crossing_z - float(gate["center"][2])))
            half_width = 0.5 * float(gate["opening"][0]) - vehicle_radius
            half_height = 0.5 * float(gate["opening"][1]) - vehicle_radius
            valid = lateral_error <= half_width and vertical_error <= half_height
            candidate = {
                "name": gate["name"],
                "crossed": bool(valid),
                "time_s": float(t[index] + alpha * (t[index + 1] - t[index])),
                "lateral_error_m": lateral_error,
                "vertical_error_m": vertical_error,
                "clearance_margin_m": float(min(
                    half_width - lateral_error, half_height - vertical_error
                )),
            }
            if best is None or candidate["clearance_margin_m"] > best["clearance_margin_m"]:
                best = candidate
            if valid:
                search_from = int(index + 1)
                best = candidate
                break
        if best is None:
            best = {
                "name": gate["name"], "crossed": False, "time_s": None,
                "lateral_error_m": None, "vertical_error_m": None,
                "clearance_margin_m": None,
            }
        results.append(best)
        if not best["crossed"]:
            # A later gate cannot count if an earlier one was missed.
            search_from = len(t)
    return results, bool(all(item["crossed"] for item in results))


def build_summary(
    data: dict[str, np.ndarray], launch_time: float,
    reference: dict[str, np.ndarray] | None = None,
    scene_kind: str = "none",
    course: dict | None = None,
) -> dict[str, object]:
    t = data["time_s"]
    x, y, z = data["x_m"], data["y_m"], data["z_m"]
    q_abs_w = np.clip(np.abs(data["qw"]), 0.0, 1.0)
    attitude_geodesic = np.rad2deg(2.0 * np.arccos(q_abs_w))
    rpm = np.column_stack([data[f"rpm_{i}"] for i in range(1, 5)])
    after_launch = t >= launch_time + 0.05
    crash_mask = after_launch & (data["contacts"] > 0.0)
    crash_indices = np.flatnonzero(crash_mask)
    first_crash = float(t[crash_indices[0]]) if crash_indices.size else None
    max_rpm = float(np.max(rpm))
    saturation_threshold = 0.995 * max_rpm if max_rpm > 0.0 else math.inf
    summary: dict[str, object] = {
        "samples": int(t.size),
        "duration_s": float(t[-1] - t[0]),
        "launch_time_s": launch_time,
        "crashed": bool(crash_indices.size),
        "first_crash_time_s": first_crash,
        "first_crash_after_launch_s": (
            first_crash - launch_time if first_crash is not None else None
        ),
        "altitude_min_m": float(np.min(z)),
        "altitude_max_m": float(np.max(z)),
        "altitude_final_m": float(z[-1]),
        "horizontal_displacement_max_m": float(np.max(np.hypot(x - x[0], y - y[0]))),
        "horizontal_displacement_final_m": float(np.hypot(x[-1] - x[0], y[-1] - y[0])),
        "horizontal_speed_final_mps": float(np.hypot(
            data["vx_mps"][-1], data["vy_mps"][-1]
        )),
        "attitude_geodesic_max_deg": float(np.max(attitude_geodesic)),
        "angular_rate_max_rad_s": float(np.max(np.sqrt(
            data["wx_radps"] ** 2 + data["wy_radps"] ** 2 + data["wz_radps"] ** 2
        ))),
        "rpm_max": max_rpm,
        "motor_saturation_fraction": float(np.mean(rpm >= saturation_threshold)),
        "contact_count_max": int(np.max(data["contacts"])),
    }
    if scene_kind == "obstacle":
        # Exact obstacle footprint from vision_obstacle.xml. Report clearance
        # for a conservative 0.10 m Crazyflie footprint, not merely whether
        # MuJoCo happened to register contact at a sampled instant.
        xmin, xmax, ymin, ymax = 1.41, 1.69, -0.34, 0.34
        radius = 0.10
        dx = np.maximum(np.maximum(xmin - x, 0.0), x - xmax)
        dy = np.maximum(np.maximum(ymin - y, 0.0), y - ymax)
        outside = np.hypot(dx, dy)
        inside = (x >= xmin) & (x <= xmax) & (y >= ymin) & (y <= ymax)
        signed_center_distance = outside
        signed_center_distance[inside] = -np.minimum.reduce([
            x[inside] - xmin, xmax - x[inside],
            y[inside] - ymin, ymax - y[inside],
        ])
        obstacle_clearance = signed_center_distance - radius
        wall_clearance = 1.30 - np.abs(y) - radius
        passed_far_face = bool(np.max(x) >= xmax + radius)
        summary.update({
            "obstacle_clearance_min_m": float(np.min(obstacle_clearance)),
            "corridor_wall_clearance_min_m": float(np.min(wall_clearance)),
            "obstacle_far_face_passed": passed_far_face,
            "track_lateral_final_m": float(y[-1]),
            "avoidance_success": bool(
                passed_far_face
                and np.min(obstacle_clearance) >= 0.0
                and np.min(wall_clearance) >= 0.0
                and not crash_indices.size
            ),
        })
    if course is not None:
        points = np.column_stack((x, y))
        vehicle_radius = 0.10
        per_obstacle = {
            obstacle["name"]: float(np.min(clearance_to_obstacle(
                points, obstacle, vehicle_radius
            )))
            for obstacle in course.get("obstacles", [])
        }
        minimum_obstacle_clearance = min(per_obstacle.values(), default=math.inf)
        per_wall = {
            wall["name"]: float(np.min(clearance_to_obstacle(
                points, wall, vehicle_radius
            )))
            for wall in course.get("walls", [])
        }
        minimum_wall_clearance = min(per_wall.values(), default=math.inf)
        centerline = np.asarray(course["centerline"], dtype=float)
        cross_track = point_to_polyline_distance(points, centerline)
        pass_point = np.asarray(course["pass_point"], dtype=float)
        pass_distance = np.linalg.norm(points - pass_point, axis=1)
        pass_reached = bool(np.min(pass_distance) <= float(course["pass_radius_m"]))
        launched_index = int(np.searchsorted(t, launch_time))
        finish_candidates = np.flatnonzero(
            (np.arange(t.size) >= launched_index)
            & (pass_distance <= float(course["pass_radius_m"]))
        )
        finish_index = int(finish_candidates[0]) if finish_candidates.size else None
        completion_time = (
            float(t[finish_index] - launch_time) if finish_index is not None else None
        )
        mean_speed = None
        if finish_index is not None and finish_index > launched_index:
            speed = np.hypot(data["vx_mps"], data["vy_mps"])
            mean_speed = float(np.trapezoid(
                speed[launched_index:finish_index + 1],
                t[launched_index:finish_index + 1],
            ) / max(1.0e-9, t[finish_index] - t[launched_index]))
        segment_results = []
        segment_search_index = launched_index
        for segment in course.get("segments", []):
            segment_point = np.asarray(segment["pass_point"], dtype=float)
            segment_distance = np.linalg.norm(points - segment_point, axis=1)
            candidates = np.flatnonzero(
                (np.arange(t.size) >= segment_search_index)
                & (segment_distance <= float(segment["pass_radius_m"]))
            )
            reached = bool(candidates.size)
            reached_index = int(candidates[0]) if reached else None
            segment_results.append({
                "name": segment["name"],
                "reached": reached,
                "time_after_launch_s": (
                    float(t[reached_index] - launch_time)
                    if reached_index is not None else None
                ),
                "minimum_distance_m": float(np.min(
                    segment_distance[segment_search_index:]
                )),
            })
            if reached_index is not None:
                segment_search_index = reached_index
        _, _, yaw_deg = quaternion_to_euler_deg(
            data["qw"], data["qx"], data["qy"], data["qz"]
        )
        yaw_unwrapped = np.rad2deg(np.unwrap(np.deg2rad(yaw_deg)))
        launched_indices = np.flatnonzero(t >= launch_time)
        heading_change = 0.0
        if launched_indices.size:
            heading_change = float(np.max(np.abs(
                yaw_unwrapped[launched_indices] - yaw_unwrapped[launched_indices[0]]
            )))
        required_heading = float(course.get("required_heading_change_deg", 0.0))
        heading_requirement_met = heading_change >= 0.75 * required_heading
        maximum_final_cross_track = float(
            course.get("maximum_final_cross_track_m", np.inf)
        )
        final_cross_track = float(cross_track[-1])
        final_cross_track_met = final_cross_track <= maximum_final_cross_track
        wall_clearance = (minimum_wall_clearance
                          if course.get("walls", []) else None)
        wall_clearance_safe = minimum_wall_clearance >= 0.0
        if "corridor_y" in course:
            lower, upper = map(float, course["corridor_y"])
            wall_clearance = float(np.min(np.minimum(
                y - lower - vehicle_radius, upper - y - vehicle_radius
            )))
            wall_clearance_safe = wall_clearance >= 0.0
        gate_results, gates_passed = ordered_gate_crossings(
            data, course.get("gates", []), launch_time, vehicle_radius
        )
        course_success = bool(
            pass_reached
            and minimum_obstacle_clearance >= 0.0
            and wall_clearance_safe
            and gates_passed
            and heading_requirement_met
            and final_cross_track_met
            and not crash_indices.size
        )
        summary.update({
            "course": course["name"],
            "course_description": course["description"],
            "course_pass_point_reached": pass_reached,
            "course_pass_point_min_distance_m": float(np.min(pass_distance)),
            "course_obstacle_clearance_min_m": minimum_obstacle_clearance,
            "course_obstacle_clearance_by_name_m": per_obstacle,
            "course_wall_clearance_min_m": wall_clearance,
            "course_wall_clearance_by_name_m": per_wall,
            "course_segments": segment_results,
            "course_completion_time_s": completion_time,
            "course_mean_horizontal_speed_mps": mean_speed,
            "course_cross_track_error_max_m": float(np.max(cross_track)),
            "course_cross_track_error_final_m": final_cross_track,
            "course_cross_track_error_final_limit_m": maximum_final_cross_track,
            "course_final_cross_track_requirement_met": final_cross_track_met,
            "course_heading_change_max_deg": heading_change,
            "course_required_heading_change_deg": required_heading,
            "course_heading_requirement_met": heading_requirement_met,
            "course_gate_results": gate_results,
            "course_gates_passed_in_order": gates_passed,
            "course_success": course_success,
        })
    return summary


def plot_run(data, reference, launch_time: float, summary, output: Path,
             vision=None, scene_kind: str = "none", course: dict | None = None):
    t = data["time_s"]
    roll, pitch, yaw = quaternion_to_euler_deg(
        data["qw"], data["qx"], data["qy"], data["qz"]
    )
    rpm = np.column_stack([data[f"rpm_{i}"] for i in range(1, 5)])
    crash_time = summary["first_crash_time_s"]
    launch_index = min(int(np.searchsorted(t, launch_time)), len(t) - 1)
    anchored_reference = None
    if reference is not None:
        # Firmware translates the stored position reference to the measured
        # handoff pose. Mirror that transformation in the plot so a non-default
        # spawn altitude is not shown as a tracking offset.
        anchored_reference = dict(reference)
        for axis, data_name in (("x", "x_m"), ("y", "y_m"), ("z", "z_m")):
            anchored_reference[axis] = (
                reference[axis] - reference[axis][0] + data[data_name][launch_index]
            )

    fig, axes = plt.subplots(2, 2, figsize=(13, 9), constrained_layout=True)
    ax = axes[0, 0]
    if anchored_reference is not None:
        ax.plot(anchored_reference["x"], anchored_reference["y"], "--",
                color="0.55", label="anchored reference")
    ax.plot(data["x_m"], data["y_m"], color="#1565c0", label="MuJoCo ground truth")
    if scene_kind == "obstacle":
        from matplotlib.patches import Rectangle
        ax.add_patch(Rectangle((1.41, -0.34), 0.28, 0.68,
                               color="#d84315", alpha=0.40, label="obstacle"))
        ax.add_patch(Rectangle((1.31, -0.44), 0.48, 0.88, fill=False,
                               edgecolor="#d84315", linestyle=":",
                               label="0.10 m vehicle envelope"))
        ax.axhline(1.20, color="0.65", linestyle=":", label="wall-safe centerline")
        ax.axhline(-1.20, color="0.65", linestyle=":")
    elif scene_kind == "gate":
        ax.plot([1.55, 1.55], [-0.33, 0.33], color="#ef6c00", linewidth=5,
                alpha=0.7, label="gate plane")
    if course is not None:
        from matplotlib.patches import Circle, Rectangle
        centerline = np.asarray(course["centerline"], dtype=float)
        ax.plot(centerline[:, 0], centerline[:, 1], "--", color="0.45",
                label="course centerline")
        for index, obstacle in enumerate(course.get("obstacles", [])):
            label = "physical obstacles" if index == 0 else None
            center = obstacle["center"]
            if obstacle["shape"] == "circle":
                patch = Circle(center, obstacle["radius"], color="#d84315",
                               alpha=0.40, label=label)
            else:
                half = obstacle["half_size"]
                patch = Rectangle((center[0] - half[0], center[1] - half[1]),
                                  2 * half[0], 2 * half[1], color="#d84315",
                                  alpha=0.40, label=label)
            ax.add_patch(patch)
        for index, wall in enumerate(course.get("walls", [])):
            center = wall["center"]
            half = wall["half_size"]
            ax.add_patch(Rectangle(
                (center[0] - half[0], center[1] - half[1]),
                2 * half[0], 2 * half[1], color="0.40", alpha=0.35,
                label="course walls" if index == 0 else None,
            ))
        for index, gate in enumerate(course.get("gates", [])):
            center = np.asarray(gate["center"][:2], dtype=float)
            normal = np.asarray(gate["normal"], dtype=float)
            normal /= np.linalg.norm(normal)
            tangent = np.asarray([-normal[1], normal[0]])
            half_width = 0.5 * float(gate["opening"][0])
            endpoints = np.vstack((center - half_width * tangent,
                                   center + half_width * tangent))
            ax.plot(endpoints[:, 0], endpoints[:, 1], color="#ef6c00",
                    linewidth=4, alpha=0.8,
                    label="ordered gates" if index == 0 else None)
            ax.annotate(str(index + 1), center, color="#e65100", weight="bold")
        ax.add_patch(Circle(course["pass_point"], course["pass_radius_m"],
                            fill=False, edgecolor="#2e7d32", linestyle=":",
                            label="course pass region"))
    if vision is not None:
        # Purple is reserved for the DroNet-style collision head so it has one
        # unambiguous meaning across adapters. Raw clearances and sector scores
        # remain available in vision.csv and the dedicated vision panels.
        triggered = vision["collision"] > 0.77
        trigger_label = "DroNet risk > 0.77"
        if np.any(triggered):
            event_t = vision["time_s"][triggered]
            event_x = np.interp(event_t, t, data["x_m"])
            event_y = np.interp(event_t, t, data["y_m"])
            ax.scatter(event_x, event_y, s=14, color="#7b1fa2", alpha=0.65,
                       label=trigger_label)
    ax.scatter(data["x_m"][0], data["y_m"][0], marker="o", color="#2e7d32", label="start")
    if crash_time is not None:
        idx = int(np.searchsorted(t, crash_time))
        ax.scatter(data["x_m"][idx], data["y_m"][idx], marker="X", s=100,
                   color="#c62828", label="collision/contact")
    ax.set(title="Top-down path", xlabel="x [m]", ylabel="y [m]")
    ax.axis("equal")
    ax.grid(alpha=0.25)
    ax.legend(loc="best")

    ax = axes[0, 1]
    if anchored_reference is not None:
        ax.plot(anchored_reference["t"] + launch_time, anchored_reference["z"],
                "--", color="0.55", label="anchored reference z")
    ax.plot(t, data["z_m"], color="#1565c0", label="actual z")
    ax.axvline(launch_time, color="#2e7d32", linestyle=":", label="maneuver handoff")
    if crash_time is not None:
        ax.axvline(crash_time, color="#c62828", linestyle="--", label="crash")
    ax.set(title="Altitude", xlabel="simulation time [s]", ylabel="z [m]")
    ax.grid(alpha=0.25)
    ax.legend(loc="best")

    ax = axes[1, 0]
    ax.plot(t, roll, label="roll")
    ax.plot(t, pitch, label="pitch")
    ax.plot(t, yaw, label="yaw")
    ax.axvline(launch_time, color="#2e7d32", linestyle=":")
    if crash_time is not None:
        ax.axvline(crash_time, color="#c62828", linestyle="--")
    ax.set(title="Ground-truth attitude (Euler view)", xlabel="simulation time [s]",
           ylabel="angle [deg]", ylim=(-190, 190))
    ax.grid(alpha=0.25)
    ax.legend(loc="best")

    ax = axes[1, 1]
    for motor in range(4):
        ax.plot(t, rpm[:, motor], label=f"motor {motor + 1}")
    ax.axvline(launch_time, color="#2e7d32", linestyle=":")
    if crash_time is not None:
        ax.axvline(crash_time, color="#c62828", linestyle="--")
    ax.set(title="Motor speed", xlabel="simulation time [s]", ylabel="RPM")
    ax.grid(alpha=0.25)
    ax.legend(loc="best", ncol=2)

    if "course_success" in summary:
        status = "COURSE PASS" if summary["course_success"] else "COURSE FAIL"
    else:
        status = "CRASH/CONTACT" if summary["crashed"] else "NO CONTACT"
    fig.suptitle(f"CrazySim/MuJoCo exact-firmware validation — {status}", fontsize=15)
    fig.savefig(output, dpi=160)
    plt.close(fig)


def load_vision_csv(path: Path) -> dict[str, np.ndarray]:
    with path.open(newline="") as stream:
        rows = list(csv.DictReader(stream))
    numeric = ["time_s", "inference_ms", "steering", "collision", "metric",
               "spatial_danger", "navigation", "gate_valid", "gate_confidence"] + [
               "danger_threshold"] + [
               f"clearance_{i}_m" for i in range(4)] + [
               f"confidence_{i}" for i in range(4)] + [
               f"danger_{i}" for i in range(4)] + [
               f"raw_danger_{i}" for i in range(4)]
    return {name: np.asarray([float(row.get(name, 0.0)) for row in rows], dtype=float)
            for name in numeric}


def plot_vision(vision: dict[str, np.ndarray], output: Path) -> None:
    t = vision["time_s"]
    fig, axes = plt.subplots(3, 1, figsize=(11, 9), sharex=True,
                             constrained_layout=True)
    metric = vision["metric"] > 0.5
    if np.any(vision["spatial_danger"] > 0.5):
        raw = np.column_stack([
            vision[f"raw_danger_{sector}"] for sector in range(4)
        ])
        sent = np.column_stack([
            vision[f"danger_{sector}"] for sector in range(4)
        ])
        if not np.any(raw) and np.any(sent):
            # Backward compatibility for runs recorded before raw model
            # scores were logged separately from controller-domain sectors.
            raw = sent
        for sector in range(4):
            axes[0].plot(t, raw[:, sector], label=f"raw sector {sector}")
        thresholds = vision["danger_threshold"]
        thresholds = thresholds[np.isfinite(thresholds) & (thresholds > 0.0)]
        model_threshold = float(np.median(thresholds)) if thresholds.size else 0.25
        axes[0].axhline(model_threshold, color="#c62828", linestyle="--",
                        label=f"model threshold {model_threshold:.2f}")
        if not np.allclose(raw, sent, equal_nan=True):
            activated = np.sum(sent >= 0.25, axis=1)
            axes[0].step(t, activated / 4.0, where="post", color="black",
                         alpha=0.55, label="active sectors / 4")
        axes[0].set(title="Neural spatial danger (before control thresholding)")
        axes[0].legend(ncol=3, fontsize=8)
    elif np.any(metric):
        for sector in range(4):
            axes[0].plot(t, vision[f"confidence_{sector}"],
                         label=f"sector {sector}")
        axes[0].axhline(-1.0986123, color="#c62828", linestyle="--",
                        label="firmware confidence threshold")
        axes[0].set(title="Sequential clearance confidence logits")
        axes[0].legend(ncol=5, fontsize=8)
    else:
        axes[0].plot(t, vision["collision"], color="#d32f2f",
                     label="non-spatial collision probability")
        axes[0].set(title="Collision head (no sector geometry)")
        axes[0].legend()
    axes[0].set(ylabel="probability")
    axes[0].grid(alpha=0.25)
    if np.any(metric):
        for sector in range(4):
            values = np.where(metric, vision[f"clearance_{sector}_m"], np.nan)
            axes[1].plot(t, values, label=f"sector {sector}")
        axes[1].axhline(0.30, color="#c62828", linestyle="--")
        axes[1].set(ylabel="clearance [m]", title="Metric clearance head")
    else:
        axes[1].text(0.5, 0.5, "This model does not claim metric clearance",
                     transform=axes[1].transAxes, ha="center", va="center")
        axes[1].set(title="Metric clearance head")
    axes[1].grid(alpha=0.25)
    has_navigation = np.any(vision["navigation"] > 0.5)
    steering_label = ("steering (left +)" if has_navigation else
                      "derived steering diagnostic (not sent)")
    collision_label = ("collision risk" if has_navigation else
                       "derived activation score")
    axes[2].plot(t, vision["steering"], label=steering_label)
    axes[2].plot(t, vision["collision"], label=collision_label)
    axes[2].plot(t, vision["gate_valid"], linestyle=":", label="gate valid")
    axes[2].set(xlabel="simulation time [s]", ylabel="normalized",
                title=("Navigation and gate lock" if has_navigation else
                       "Perception diagnostics and gate lock"),
                ylim=(-1.05, 1.05))
    axes[2].grid(alpha=0.25)
    axes[2].legend()
    fig.savefig(output, dpi=160)
    plt.close(fig)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--csv", required=True, type=Path)
    parser.add_argument("--out", required=True, type=Path)
    parser.add_argument("--reference", type=Path)
    parser.add_argument("--launch-time", type=float, default=4.0)
    parser.add_argument("--vision-csv", type=Path)
    parser.add_argument("--scene-kind", choices=("none", "obstacle", "gate"), default="none")
    parser.add_argument("--course", type=Path,
                        help="Course geometry and acceptance manifest")
    args = parser.parse_args()

    args.out.mkdir(parents=True, exist_ok=True)
    data = load_numeric_csv(args.csv)
    reference = load_numeric_csv(args.reference) if args.reference else None
    vision = load_vision_csv(args.vision_csv) if args.vision_csv else None
    course = json.loads(args.course.read_text()) if args.course else None
    if course is not None and course.get("format") not in (
        "tinympc-crazysim-course-v1", "tinympc-crazysim-course-v2"
    ):
        raise ValueError("unsupported course manifest")
    launch_time = args.launch_time
    if "airborne" in data:
        airborne_indices = np.flatnonzero(data["airborne"] > 0.5)
        if airborne_indices.size:
            launch_time = float(data["time_s"][airborne_indices[0]])
    summary = build_summary(data, launch_time, reference, args.scene_kind, course)
    (args.out / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    plot_run(data, reference, launch_time, summary, args.out / "validation.png",
             vision, args.scene_kind, course)
    if vision is not None:
        plot_vision(vision, args.out / "vision.png")
    print(json.dumps(summary, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
