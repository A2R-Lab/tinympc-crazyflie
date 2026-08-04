#!/usr/bin/env python3
"""Exercise the exact GAP8/STM32 perception path in 1 m/s race approaches."""

from __future__ import annotations

import argparse
import json
import math
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

DT_US = 65_000
FRAMES = 28
SPEED_M_S = 1.0
MIN_ACTION_RANGE_M = 1.0
MAX_RANGE_OVERESTIMATE_M = 0.35
MAX_LATERAL_ERROR_M = 0.35
MIN_ESTIMATED_FORWARD_CLEARANCE_M = 0.30


@dataclass(frozen=True)
class RaceCase:
    name: str
    scene: Scene
    lane_y: float = 0.0
    yaw_amplitude_rad: float = 0.0
    read_noise: float = 0.0
    exposure_variation: float = 0.0
    blur_fraction: float = 0.0
    expect_detection: bool = True
    speed_m_s: float = SPEED_M_S
    timestamp_jitter_us: int = 0
    doubled_interval_frame: int = -1


CASES = (
    RaceCase("center_clean", Scene(
        "center_clean", (Box("target", 3.0, 3.25, -0.45, 0.45, 1),),
        truth=(3.0, 0.0))),
    RaceCase("left_offset", Scene(
        "left_offset", (Box("target", 3.0, 3.25, -0.05, 0.65, 2),),
        truth=(3.0, 0.30))),
    RaceCase("right_offset", Scene(
        "right_offset", (Box("target", 3.0, 3.25, -0.65, 0.05, 3),),
        truth=(3.0, -0.30))),
    RaceCase("narrow_center", Scene(
        "narrow_center", (Box("target", 3.0, 3.25, -0.18, 0.18, 4),),
        truth=(3.0, 0.0))),
    RaceCase("cluttered", Scene(
        "cluttered",
        (Box("target", 2.8, 3.05, -0.40, 0.40, 1),
         Box("left_far", 3.7, 4.0, 0.55, 1.05, 3),
         Box("right_far", 4.1, 4.4, -1.05, -0.55, 4)),
        truth=(2.8, 0.0))),
    RaceCase("moderate_sensor_degradation", Scene(
        "degraded", (Box("target", 3.0, 3.25, -0.45, 0.45, 1),),
        truth=(3.0, 0.0)), read_noise=2.0, exposure_variation=0.08,
        blur_fraction=0.35),
    RaceCase("moderate_sensor_degradation_texture2", Scene(
        "degraded2", (Box("target", 3.0, 3.25, -0.45, 0.45, 2),),
        truth=(3.0, 0.0)), read_noise=2.0, exposure_variation=0.08,
        blur_fraction=0.35),
    RaceCase("moderate_sensor_degradation_texture3", Scene(
        "degraded3", (Box("target", 3.0, 3.25, -0.45, 0.45, 3),),
        truth=(3.0, 0.0)), read_noise=2.0, exposure_variation=0.08,
        blur_fraction=0.35),
    RaceCase("gentle_yaw", Scene(
        "gentle_yaw", (Box("target", 3.0, 3.25, -0.45, 0.45, 2),),
        truth=(3.0, 0.0)), yaw_amplitude_rad=math.radians(4.0)),
    RaceCase("gentle_yaw_opposite", Scene(
        "gentle_yaw_opposite",
        (Box("target", 3.0, 3.25, -0.45, 0.45, 3),),
        truth=(3.0, 0.0)), yaw_amplitude_rad=-math.radians(4.0)),
    RaceCase("timing_jitter_and_dropped_capture", Scene(
        "timing_jitter",
        (Box("target", 3.0, 3.25, -0.45, 0.45, 4),),
        truth=(3.0, 0.0)), timestamp_jitter_us=5_000,
        doubled_interval_frame=12),
    RaceCase("background_negative", Scene(
        "background_negative", (), truth=None, detectable=False),
        expect_detection=False),
    RaceCase("background_degraded_negative", Scene(
        "background_degraded_negative", (), truth=None, detectable=False),
        read_noise=2.0, exposure_variation=0.08, blur_fraction=0.35,
        expect_detection=False),
    RaceCase("pure_yaw_negative", Scene(
        "pure_yaw_negative",
        (Box("target", 3.0, 3.25, -0.45, 0.45, 1),),
        truth=(3.0, 0.0), detectable=False),
        yaw_amplitude_rad=math.radians(6.0), expect_detection=False,
        speed_m_s=0.0),
)


def render_inputs() -> tuple[bytes, list[dict[str, float | int]]]:
    import struct

    wire = bytearray()
    states: list[dict[str, float | int]] = []
    for case_id, case in enumerate(CASES):
        previous_yaw = 0.0
        capture_us = DT_US
        motion_time_s = 0.0
        for frame in range(FRAMES):
            if frame == 0:
                frame_dt_us = DT_US
            else:
                jitter_sign = 1 if frame % 2 else -1
                frame_dt_us = DT_US + jitter_sign * case.timestamp_jitter_us
                if frame == case.doubled_interval_frame:
                    frame_dt_us += DT_US
                capture_us += frame_dt_us
                motion_time_s += frame_dt_us * 1.0e-6
            dt_s = frame_dt_us * 1.0e-6
            t = motion_time_s
            x = case.speed_m_s * t
            y = case.lane_y
            yaw = case.yaw_amplitude_rad * math.sin(2.0 * math.pi * t / 2.4)
            yaw_rate = 0.0 if frame == 0 else (yaw - previous_yaw) / dt_s
            previous_yaw = yaw
            body_vx = case.speed_m_s * math.cos(yaw)
            body_vy = -case.speed_m_s * math.sin(yaw)
            exposure = 1.0 + case.exposure_variation * math.sin(frame * 1.7)
            image = render_gap8(
                case.scene, x, y, yaw, exposure_scale=exposure,
                read_noise=case.read_noise, seed=case_id * 100 + frame,
                motion_blur_x=case.speed_m_s * dt_s * case.blur_fraction)
            timestamp = capture_us
            wire.extend(struct.pack("<III", case_id, frame, timestamp))
            wire.extend(bytes(value for row in image for value in row))
            states.append({
                "case": case_id, "frame": frame, "tick": timestamp // 1000,
                "vx": body_vx, "vy": body_vy, "yaw_rate": yaw_rate,
                "x": x, "y": y, "yaw": yaw, "clock_bias_ms": 0,
            })
    return bytes(wire), states


def summarize(rows: list[dict[str, str]],
              states: list[dict[str, float | int]]) -> dict[str, object]:
    results = []
    for case_id, case in enumerate(CASES):
        paired = [
            (row, state) for row, state in zip(rows, states, strict=True)
            if int(state["case"]) == case_id
        ]
        detections = [
            (row, state) for row, state in paired
            if int(float(row["cyl_valid"]))
        ]
        first = detections[0] if detections else None
        first_range = None
        position_error = None
        range_overestimate = None
        lateral_error = None
        estimated_forward_clearance = None
        if first and case.scene.truth:
            row, state = first
            first_range = case.scene.truth[0] - float(state["x"])
            estimated_forward_clearance = (
                float(row["cyl_world_x"]) - float(state["x"]))
            range_overestimate = (
                float(row["cyl_world_x"]) - case.scene.truth[0])
            lateral_error = abs(
                float(row["cyl_world_y"]) - case.scene.truth[1])
            position_error = math.hypot(
                range_overestimate, lateral_error)
        emergency = [
            (row, state) for row, state in paired if int(row["emergency"])
        ]
        first_emergency_range = None
        if emergency and case.scene.truth:
            first_emergency_range = (
                case.scene.truth[0] - float(emergency[0][1]["x"]))
        if case.expect_detection:
            passed = (
                first is not None and first_range is not None and
                first_range >= MIN_ACTION_RANGE_M and
                range_overestimate is not None and
                range_overestimate <= MAX_RANGE_OVERESTIMATE_M and
                lateral_error is not None and
                lateral_error <= MAX_LATERAL_ERROR_M and
                estimated_forward_clearance is not None and
                estimated_forward_clearance >=
                    MIN_ESTIMATED_FORWARD_CLEARANCE_M)
        else:
            passed = first is None and not emergency
        results.append({
            "case": case.name,
            "speed_m_s": case.speed_m_s,
            "expected_detection": case.expect_detection,
            "detected": first is not None,
            "first_detection_range_m": first_range,
            "first_emergency_range_m": first_emergency_range,
            "position_error_m": position_error,
            "range_overestimate_m": range_overestimate,
            "lateral_error_m": lateral_error,
            "estimated_forward_clearance_m": estimated_forward_clearance,
            "first_estimated_world_xy": None if first is None else [
                float(first[0]["cyl_world_x"]),
                float(first[0]["cyl_world_y"]),
            ],
            "truth_world_xy": case.scene.truth,
            "max_accepted_tracks": max(
                int(row["accepted"]) for row, _ in paired),
            "max_reject_motion": max(
                int(row["reject_motion"]) for row, _ in paired),
            "max_reject_geometry": max(
                int(row["reject_geometry"]) for row, _ in paired),
            "max_reject_uncertainty": max(
                int(row["reject_uncertainty"]) for row, _ in paired),
            "max_cluster_support": max(
                int(row["support"]) for row, _ in paired),
            "max_baseline_m": max(
                float(row["baseline"]) for row, _ in paired),
            "max_observation_hits": max(
                int(row["obs_hits"]) for row, _ in paired),
            "max_cylinder_accepts": max(
                int(row["cyl_accepts"]) for row, _ in paired),
            "max_map_peak": max(
                float(row["map_peak"]) for row, _ in paired),
            "emergency_frames": len(emergency),
            "max_looming_rate_s_inv": max(
                float(row["loom_rate"]) for row, _ in paired),
            "failure_trace": None if passed else [
                {
                    "frame": int(state["frame"]),
                    "accepted": int(row["accepted"]),
                    "support": int(row["support"]),
                    "sigma": float(row["cluster_sigma"]),
                    "baseline": float(row["baseline"]),
                    "hits": int(row["obs_hits"]),
                    "map_peak": float(row["map_peak"]),
                    "cylinder_accepts": int(row["cyl_accepts"]),
                    "cylinder_valid": bool(int(float(row["cyl_valid"]))),
                    "cylinder_world_x": float(row["cyl_world_x"]),
                    "cylinder_world_y": float(row["cyl_world_y"]),
                }
                for row, state in paired
                if int(row["support"]) > 0 or float(row["map_peak"]) > 0.0
            ],
            "pass": passed,
        })
    return {
        "implementation": {
            "gap8": "verbatim functions extracted from pulp-frontnet/main.c",
            "stm32": "flowdeck_obstacle_link.c directly included by host harness",
        },
        "requirements": {
            "speed_m_s": SPEED_M_S,
            "minimum_action_range_m": MIN_ACTION_RANGE_M,
            "maximum_range_overestimate_m": MAX_RANGE_OVERESTIMATE_M,
            "maximum_lateral_error_m": MAX_LATERAL_ERROR_M,
            "minimum_estimated_forward_clearance_m":
                MIN_ESTIMATED_FORWARD_CLEARANCE_M,
            "routine_peering_m": 0.0,
        },
        "passed": sum(bool(result["pass"]) for result in results),
        "total": len(results),
        "results": results,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--nanocockpit", type=Path, required=True)
    parser.add_argument("--out", type=Path)
    args = parser.parse_args()
    with tempfile.TemporaryDirectory(prefix="exact-race-sim-") as tmp_name:
        gap, stm = build(HERE.parents[3], args.nanocockpit, Path(tmp_name))
        image_wire, states = render_inputs()
        gap_rows = run_csv(gap, image_wire)
        stm_rows: list[dict[str, str]] = []
        for case_id in range(len(CASES)):
            indexes = [
                i for i, state in enumerate(states)
                if int(state["case"]) == case_id
            ]
            stm_rows.extend(run_csv(
                stm, stm32_wire(
                    [gap_rows[i] for i in indexes],
                    [states[i] for i in indexes])))
        summary = summarize(stm_rows, states)
    output = json.dumps(summary, indent=2)
    print(output)
    if args.out:
        args.out.write_text(output + "\n")
    return 0 if summary["passed"] == summary["total"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
