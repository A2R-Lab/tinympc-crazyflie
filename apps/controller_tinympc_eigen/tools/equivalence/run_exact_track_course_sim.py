#!/usr/bin/env python3
"""Run continuous 4 m x 4 m multi-obstacle courses through exact firmware."""

from __future__ import annotations

import argparse
import json
import math
import re
import struct
import tempfile
from dataclasses import dataclass
from pathlib import Path

from run_exact_track_image_sim import build, run_csv, stm32_wire

import sys

HERE = Path(__file__).resolve().parent
TOOLS = HERE.parent
sys.path.insert(0, str(TOOLS))

from sim_gap8_deployment_suite import render_gap8  # noqa: E402
from sim_monocular_flow_suite import Box, Scene  # noqa: E402

ARENA_SIZE_M = 4.0
DT_US = 65_000
SPEED_M_S = 1.0
DRONE_RADIUS_M = 0.10
OBSTACLE_MATCH_MARGIN_M = 0.45
MIN_DETECTION_CLEARANCE_M = 0.35


@dataclass(frozen=True)
class CourseCase:
    name: str
    reverse: bool = False
    firmware_circuit: bool = False
    read_noise: float = 0.0
    exposure_variation: float = 0.0
    blur_fraction: float = 0.0


COURSES = (
    CourseCase("firmware_circuit_1mps", firmware_circuit=True),
    CourseCase("clockwise_clean"),
    CourseCase("counterclockwise_clean", reverse=True),
    CourseCase("clockwise_degraded", read_noise=2.0,
               exposure_variation=0.08, blur_fraction=0.35),
)

OBSTACLES = (
    Box("obstacle_south", 1.85, 2.15, 0.40, 0.90, 1),
    Box("obstacle_east", 3.10, 3.60, 1.85, 2.15, 2),
    Box("obstacle_north", 1.85, 2.15, 3.10, 3.60, 3),
    Box("obstacle_west", 0.40, 0.90, 1.85, 2.15, 4),
)

WALLS = (
    Box("wall_south", -0.10, 4.10, -0.10, 0.0, 5),
    Box("wall_north", -0.10, 4.10, 4.0, 4.10, 5),
    Box("wall_west", -0.10, 0.0, 0.0, 4.0, 5),
    Box("wall_east", 4.0, 4.10, 0.0, 4.0, 5),
)

# The reference is a continuous slalom around four solid obstacles. Each
# obstacle blocks the corresponding nominal rectangular straight; the route
# supplies an explicit, inspectable avoidance path rather than teleporting the
# vehicle or resetting estimator state between encounters.
CONTROL_POINTS = (
    (0.60, 0.60), (1.40, 0.60), (1.55, 1.25), (2.45, 1.25),
    (2.60, 0.60), (3.40, 0.60), (3.40, 1.40), (2.75, 1.55),
    (2.75, 2.45), (3.40, 2.60), (3.40, 3.40), (2.60, 3.40),
    (2.45, 2.75), (1.55, 2.75), (1.40, 3.40), (0.60, 3.40),
    (0.60, 2.60), (1.25, 2.45), (1.25, 1.55), (0.60, 1.40),
)


def chaikin_closed(points: list[tuple[float, float]],
                   iterations: int = 3) -> list[tuple[float, float]]:
    current = points
    for _ in range(iterations):
        refined: list[tuple[float, float]] = []
        for index, point in enumerate(current):
            nxt = current[(index + 1) % len(current)]
            refined.append((0.75 * point[0] + 0.25 * nxt[0],
                            0.75 * point[1] + 0.25 * nxt[1]))
            refined.append((0.25 * point[0] + 0.75 * nxt[0],
                            0.25 * point[1] + 0.75 * nxt[1]))
        current = refined
    return current


def resample_closed(points: list[tuple[float, float]],
                    spacing: float) -> list[tuple[float, float]]:
    segments = []
    total = 0.0
    for index, point in enumerate(points):
        nxt = points[(index + 1) % len(points)]
        length = math.hypot(nxt[0] - point[0], nxt[1] - point[1])
        segments.append((point, nxt, length, total))
        total += length
    count = max(2, round(total / spacing))
    samples = []
    segment_index = 0
    for sample_index in range(count):
        distance = sample_index * total / count
        while (segment_index + 1 < len(segments) and
               segments[segment_index + 1][3] <= distance):
            segment_index += 1
        start, end, length, offset = segments[segment_index]
        fraction = 0.0 if length == 0.0 else (distance - offset) / length
        samples.append((start[0] + fraction * (end[0] - start[0]),
                        start[1] + fraction * (end[1] - start[1])))
    return samples


def course_path(case: CourseCase) -> list[tuple[float, float]]:
    if case.firmware_circuit:
        # controller_tinympc.cpp::circuitPoint with equal semi-axes, translated
        # into the 4 m arena and resampled at the requested 1 m/s. Setting
        # circSpeed=1.0 is the only difference from the firmware's 0.3 default.
        radius = 1.25
        circumference = 2.0 * math.pi * radius
        count = round(circumference / (SPEED_M_S * DT_US * 1.0e-6))
        direction = -1.0 if case.reverse else 1.0
        return [
            (2.0 + radius * math.cos(direction * 2.0 * math.pi * i / count),
             2.0 + radius * math.sin(direction * 2.0 * math.pi * i / count))
            for i in range(count)
        ]
    points = resample_closed(
        chaikin_closed(list(CONTROL_POINTS)), SPEED_M_S * DT_US * 1.0e-6)
    if case.reverse:
        points = [points[0], *reversed(points[1:])]
    return points


def wrap_angle(value: float) -> float:
    return math.atan2(math.sin(value), math.cos(value))


def box_clearance(x: float, y: float, box: Box) -> float:
    dx = max(box.xmin - x, 0.0, x - box.xmax)
    dy = max(box.ymin - y, 0.0, y - box.ymax)
    outside = math.hypot(dx, dy)
    if dx == 0.0 and dy == 0.0:
        return -min(x - box.xmin, box.xmax - x,
                    y - box.ymin, box.ymax - y)
    return outside


def render_inputs() -> tuple[bytes, list[dict[str, float | int]],
                             dict[int, list[tuple[float, float]]]]:
    wire = bytearray()
    states: list[dict[str, float | int]] = []
    paths: dict[int, list[tuple[float, float]]] = {}
    dt_s = DT_US * 1.0e-6
    scene = Scene("four_meter_course", WALLS + OBSTACLES, truth=None)
    for case_id, case in enumerate(COURSES):
        path = course_path(case)
        paths[case_id] = path
        previous_yaw = 0.0
        for frame, (x, y) in enumerate(path):
            next_x, next_y = path[(frame + 1) % len(path)]
            world_vx = (next_x - x) / dt_s
            world_vy = (next_y - y) / dt_s
            yaw = math.atan2(world_vy, world_vx)
            yaw_rate = (0.0 if frame == 0 else
                        wrap_angle(yaw - previous_yaw) / dt_s)
            previous_yaw = yaw
            speed = math.hypot(world_vx, world_vy)
            body_vx = speed
            body_vy = 0.0
            timestamp = (frame + 1) * DT_US
            exposure = 1.0 + case.exposure_variation * math.sin(frame * 1.7)
            image = render_gap8(
                scene, x, y, yaw, exposure_scale=exposure,
                read_noise=case.read_noise, seed=case_id * 1000 + frame,
                motion_blur_x=speed * dt_s * case.blur_fraction)
            wire.extend(struct.pack("<III", case_id, frame, timestamp))
            wire.extend(bytes(value for row in image for value in row))
            states.append({
                "case": case_id, "frame": frame, "tick": timestamp // 1000,
                "vx": body_vx, "vy": body_vy, "yaw_rate": yaw_rate,
                "x": x, "y": y, "yaw": yaw, "clock_bias_ms": 0,
            })
    return bytes(wire), states, paths


def matched_obstacle(x: float, y: float) -> int | None:
    best_index = None
    best_distance = math.inf
    for index, obstacle in enumerate(OBSTACLES):
        distance = box_clearance(x, y, obstacle)
        if distance < best_distance:
            best_distance = distance
            best_index = index
    return (best_index if best_distance <= OBSTACLE_MATCH_MARGIN_M else None)


def summarize(rows: list[dict[str, str]],
              states: list[dict[str, float | int]],
              paths: dict[int, list[tuple[float, float]]]) -> dict[str, object]:
    results = []
    for case_id, case in enumerate(COURSES):
        paired = [
            (row, state) for row, state in zip(rows, states, strict=True)
            if int(state["case"]) == case_id
        ]
        detected = [False] * len(OBSTACLES)
        first_clearance: list[float | None] = [None] * len(OBSTACLES)
        false_cylinders = 0
        emergency_frames = 0
        collisions = 0
        first_collision_frame = None
        boundary_violations = 0
        minimum_clearance = math.inf
        for row, state in paired:
            x, y = float(state["x"]), float(state["y"])
            if (x < DRONE_RADIUS_M or y < DRONE_RADIUS_M or
                    x > ARENA_SIZE_M - DRONE_RADIUS_M or
                    y > ARENA_SIZE_M - DRONE_RADIUS_M):
                boundary_violations += 1
            for obstacle in OBSTACLES:
                clearance = box_clearance(x, y, obstacle) - DRONE_RADIUS_M
                minimum_clearance = min(minimum_clearance, clearance)
                if clearance < 0.0:
                    collisions += 1
                    if first_collision_frame is None:
                        first_collision_frame = int(state["frame"])
            emergency_frames += int(row["emergency"])
            if not int(float(row["cyl_valid"])):
                continue
            cylinder_x = float(row["cyl_world_x"])
            cylinder_y = float(row["cyl_world_y"])
            obstacle_index = matched_obstacle(cylinder_x, cylinder_y)
            if obstacle_index is None:
                false_cylinders += 1
                continue
            if not detected[obstacle_index]:
                detected[obstacle_index] = True
                first_clearance[obstacle_index] = box_clearance(
                    x, y, OBSTACLES[obstacle_index])
        passed = (
            all(detected) and
            all(clearance is not None and
                clearance >= MIN_DETECTION_CLEARANCE_M
                for clearance in first_clearance) and
            false_cylinders == 0 and collisions == 0 and
            boundary_violations == 0)
        results.append({
            "course": case.name,
            "arena_m": [ARENA_SIZE_M, ARENA_SIZE_M],
            "frames": len(paths[case_id]),
            "duration_s": len(paths[case_id]) * DT_US * 1.0e-6,
            "path_length_m": len(paths[case_id]) * SPEED_M_S * DT_US * 1.0e-6,
            "obstacles": len(OBSTACLES),
            "obstacles_detected": sum(detected),
            "detected_by_name": {
                obstacle.name: detected[index]
                for index, obstacle in enumerate(OBSTACLES)
            },
            "first_detection_clearance_m": {
                obstacle.name: first_clearance[index]
                for index, obstacle in enumerate(OBSTACLES)
            },
            "false_cylinder_frames": false_cylinders,
            "emergency_frames": emergency_frames,
            "max_tracks_per_frame": max(
                int(row["accepted"]) + int(row["reject_motion"]) +
                int(row["reject_geometry"]) + int(row["reject_uncertainty"])
                for row, _ in paired),
            "max_accepted_depth_tracks": max(
                int(row["accepted"]) for row, _ in paired),
            "max_cluster_support": max(
                int(row["support"]) for row, _ in paired),
            "frames_with_depth_tracks": sum(
                int(row["accepted"]) > 0 for row, _ in paired),
            "frames_with_clusters": sum(
                int(row["support"]) >= 2 for row, _ in paired),
            "frames_above_yaw_gate": sum(
                abs(float(state["yaw_rate"])) >= 0.20
                for _, state in paired),
            "minimum_path_clearance_m": minimum_clearance,
            "collision_contacts": collisions,
            "first_collision_time_s": (
                None if first_collision_frame is None else
                first_collision_frame * DT_US * 1.0e-6),
            "boundary_violation_frames": boundary_violations,
            "reference_replay_completed": True,
            "physical_lap_completed": (
                collisions == 0 and boundary_violations == 0),
            "production_controller_configuration": {
                "circEn": 1 if case.firmware_circuit else 0,
                "circSpeed": SPEED_M_S if case.firmware_circuit else None,
                "obsEnable": 0,
                "obsUseFlow": 0,
                "obsLogOnly": 1,
                "unconditional_response": "looming current-position hold",
            },
            "pass": passed,
        })
    return {
        "implementation": {
            "gap8": "verbatim functions extracted from pulp-frontnet/main.c",
            "stm32": "flowdeck_obstacle_link.c directly included by host harness",
            "controller": (
                "perfect reference tracking until collision; production "
                "obstacle defaults are audited and cause no cylinder response"),
        },
        "passed": sum(bool(result["pass"]) for result in results),
        "total": len(results),
        "results": results,
    }


def audit_controller_defaults(repo: Path) -> dict[str, int]:
    source = (
        repo / "apps/controller_tinympc_eigen/src/controller_tinympc.cpp"
    ).read_text()
    expected = {
        "obsEnable": 0,
        "obsUseFlow": 0,
        "obsLogOnly": 1,
    }
    for name, value in expected.items():
        pattern = rf"uint8_t\s+{name}\s*=\s*{value}\s*;"
        if re.search(pattern, source) is None:
            raise RuntimeError(
                f"production controller default changed: expected {name}={value}")
    return expected


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--nanocockpit", type=Path, required=True)
    parser.add_argument("--out", type=Path)
    args = parser.parse_args()
    repo = HERE.parents[3]
    controller_defaults = audit_controller_defaults(repo)
    with tempfile.TemporaryDirectory(prefix="exact-course-sim-") as tmp_name:
        gap, stm = build(repo, args.nanocockpit, Path(tmp_name))
        image_wire, states, paths = render_inputs()
        gap_rows = run_csv(gap, image_wire)
        stm_rows: list[dict[str, str]] = []
        for case_id in range(len(COURSES)):
            indexes = [
                index for index, state in enumerate(states)
                if int(state["case"]) == case_id
            ]
            stm_rows.extend(run_csv(
                stm, stm32_wire(
                    [gap_rows[index] for index in indexes],
                    [states[index] for index in indexes])))
        summary = summarize(stm_rows, states, paths)
        summary["controller_defaults_audited_from_source"] = controller_defaults
    output = json.dumps(summary, indent=2)
    print(output)
    if args.out:
        args.out.write_text(output + "\n")
    return 0 if summary["passed"] == summary["total"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
