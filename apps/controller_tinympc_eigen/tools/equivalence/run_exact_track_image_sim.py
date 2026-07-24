#!/usr/bin/env python3
"""Run rendered images through the exact GAP8 and STM32 firmware sources."""

from __future__ import annotations

import argparse
import csv
import io
import json
import math
import struct
import subprocess
import sys
import tempfile
from dataclasses import dataclass
from pathlib import Path

HERE = Path(__file__).resolve().parent
TOOLS = HERE.parent
sys.path.insert(0, str(TOOLS))

from sim_gap8_deployment_suite import render_gap8  # noqa: E402
from sim_monocular_flow_suite import Box, Scene  # noqa: E402

DT_US = 65_000
FRAMES = 14


@dataclass(frozen=True)
class SimCase:
    name: str
    scene: Scene
    peer_span: float
    detectable: bool
    read_noise: float = 0.0
    exposure_variation: float = 0.0
    blur_fraction: float = 0.0
    clock_bias_ms: int = 0
    approach_span: float = 0.0
    expect_emergency: bool = False


TARGET = Box("target", 1.0, 1.2, -0.45, 0.45, 1)
CASES = (
    SimCase("nominal_peer", Scene("nominal", (TARGET,), truth=(1.0, 0.0)),
            0.20, True),
    SimCase("near_center", Scene(
        "near", (Box("target", 0.8, 1.0, -0.4, 0.4, 1),),
        truth=(0.8, 0.0)), 0.30, True),
    SimCase("far_center", Scene(
        "far", (Box("target", 1.4, 1.6, -0.5, 0.5, 1),),
        truth=(1.4, 0.0)), 0.40, True),
    SimCase("left_offset", Scene(
        "left", (Box("target", 1.0, 1.2, -0.2, 0.6, 1),),
        truth=(1.0, 0.2)), 0.30, True),
    SimCase("right_offset", Scene(
        "right", (Box("target", 1.0, 1.2, -0.6, 0.2, 1),),
        truth=(1.0, -0.2)), -0.30, True),
    SimCase("no_motion", Scene("still", (TARGET,), truth=(1.0, 0.0)),
            0.0, False),
    SimCase("insufficient_peer", Scene("small", (TARGET,), truth=(1.0, 0.0)),
            0.025, False),
    SimCase("sensor_degraded", Scene(
            "degraded", (TARGET,), truth=(1.0, 0.0)),
            0.20, True, read_noise=5.0, exposure_variation=0.12,
            blur_fraction=0.7),
    SimCase("timestamp_bias", Scene("bias", (TARGET,), truth=(1.0, 0.0)),
            0.20, True, clock_bias_ms=25),
    SimCase("mixed_depth", Scene(
        "mixed",
        (Box("near", 1.0, 1.2, -0.45, 0.0, 2),
         Box("far", 1.8, 2.0, 0.0, 0.55, 3)),
        truth=(1.0, -0.22)), 0.20, True),
    SimCase("low_texture_negative", Scene(
        "low", (Box("target", 1.0, 1.2, -0.45, 0.45, 0),),
        truth=(1.0, 0.0), detectable=False), 0.20, False),
    SimCase("looming_emergency", Scene(
        "looming", (TARGET,), truth=(1.0, 0.0)),
        0.0, True, approach_span=0.55, expect_emergency=True),
)


def run_csv(executable: Path, data: bytes) -> list[dict[str, str]]:
    result = subprocess.run(
        [str(executable)], input=data, capture_output=True, check=True)
    return list(csv.DictReader(io.StringIO(result.stdout.decode())))


def build(repo: Path, nanocockpit: Path, directory: Path) -> tuple[Path, Path]:
    gap = directory / "gap8"
    stm = directory / "stm32"
    subprocess.run([
        sys.executable, str(HERE / "build_gap8_host_replay.py"),
        "--nanocockpit", str(nanocockpit), "--output", str(gap),
    ], check=True)
    subprocess.run([
        "cc", "-std=c11", "-O2", "-Wall", "-Wextra",
        "-Wno-unused-variable", "-I", str(HERE / "mocks"),
        "-I", str(HERE / "stubs"),
        str(HERE / "stm32_track_replay.c"), "-lm", "-o", str(stm),
    ], check=True)
    return gap, stm


def render_inputs() -> tuple[bytes, list[dict[str, float | int]]]:
    wire = bytearray()
    states: list[dict[str, float | int]] = []
    for case_id, case in enumerate(CASES):
        previous_x = 0.0
        previous_y = -0.5*case.peer_span
        for frame in range(FRAMES):
            fraction = frame/(FRAMES-1)
            x = case.approach_span*fraction
            y = case.peer_span*(fraction-0.5)
            vx = 0.0 if frame == 0 else (x-previous_x)/(DT_US*1.0e-6)
            vy = 0.0 if frame == 0 else (y-previous_y)/(DT_US*1.0e-6)
            previous_x = x
            previous_y = y
            timestamp = (frame+1)*DT_US
            exposure = 1.0 + case.exposure_variation*math.sin(frame*1.7)
            image = render_gap8(
                case.scene, x, y, 0.0, exposure_scale=exposure,
                read_noise=case.read_noise, seed=case_id*100+frame,
                motion_blur_y=vy*DT_US*1.0e-6*case.blur_fraction)
            wire.extend(struct.pack("<III", case_id, frame, timestamp))
            wire.extend(bytes(value for row in image for value in row))
            states.append({
                "case": case_id, "frame": frame, "tick": timestamp//1000,
                "vx": vx, "vy": vy, "yaw_rate": 0.0,
                "x": x, "y": y, "yaw": 0.0,
                "clock_bias_ms": case.clock_bias_ms,
            })
    return bytes(wire), states


def stm32_wire(rows: list[dict[str, str]],
               states: list[dict[str, float | int]]) -> bytes:
    lines = []
    sequence_by_case: dict[int, int] = {}
    for row, state in zip(rows, states, strict=True):
        case_id = int(state["case"])
        sequence = sequence_by_case.get(case_id, 1)
        sequence_by_case[case_id] = sequence+1
        values: list[float | int] = [
            case_id, int(state["frame"]), int(state["tick"]),
            float(state["vx"]), float(state["vy"]), float(state["yaw_rate"]),
            float(state["x"]), float(state["y"]), float(state["yaw"]),
            int(row["track_ts"]),
            int(row["track_echo"])-int(state["clock_bias_ms"]),
            sequence, int(row["track_dt_us"]), int(row["track_count"]),
        ]
        for index in range(32):
            values.extend(int(row[f"{field}{index}"])
                          for field in ("tu", "tv", "tdu", "tdv", "tlk", "tfb"))
        lines.append(",".join(str(value) for value in values))
    return ("\n".join(lines)+"\n").encode()


def summarize(rows: list[dict[str, str]]) -> dict[str, object]:
    results = []
    for case_id, case in enumerate(CASES):
        case_rows = [row for row in rows if int(row["case"]) == case_id]
        valid = [row for row in case_rows if int(float(row["cyl_valid"]))]
        final = valid[-1] if valid else None
        error = None
        if final and case.scene.truth:
            error = math.hypot(
                float(final["cyl_world_x"])-case.scene.truth[0],
                float(final["cyl_world_y"])-case.scene.truth[1])
        emergency_frames = sum(int(row["emergency"]) for row in case_rows)
        passed = (
            final is not None and error is not None and error <= 0.35
            if case.detectable else final is None)
        if case.expect_emergency:
            passed = passed and emergency_frames > 0
        results.append({
            "case": case.name,
            "expected_detection": case.detectable,
            "detected": final is not None,
            "error_m": error,
            "max_accepted_tracks": max(int(row["accepted"]) for row in case_rows),
            "max_reject_motion": max(int(row["reject_motion"]) for row in case_rows),
            "max_reject_geometry": max(int(row["reject_geometry"]) for row in case_rows),
            "max_reject_uncertainty": max(
                int(row["reject_uncertainty"]) for row in case_rows),
            "max_cluster_support": max(int(row["support"]) for row in case_rows),
            "min_positive_cluster_sigma": min(
                (float(row["cluster_sigma"]) for row in case_rows
                 if float(row["cluster_sigma"]) > 0.0), default=None),
            "max_baseline_m": max(float(row["baseline"]) for row in case_rows),
            "max_obs_hits": max(int(row["obs_hits"]) for row in case_rows),
            "max_cylinder_accepts": max(
                int(row["cyl_accepts"]) for row in case_rows),
            "max_map_peak": max(float(row["map_peak"]) for row in case_rows),
            "max_looming_rate": max(float(row["loom_rate"]) for row in case_rows),
            "emergency_frames": emergency_frames,
            "failure_trace": None if passed else [{
                key: row[key] for key in (
                    "frame", "support", "validated_sigma", "safety_age",
                    "baseline", "obs_hits", "obs_valid", "cyl_accepts",
                    "map_peak", "cyl_valid")
            } for row in case_rows],
            "pass": passed,
        })
    return {
        "implementation": {
            "gap8": "verbatim functions extracted from pulp-frontnet/main.c",
            "stm32": "flowdeck_obstacle_link.c directly included by host harness",
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
    repo = HERE.parents[3]
    with tempfile.TemporaryDirectory(prefix="exact-track-sim-") as tmp_name:
        gap, stm = build(repo, args.nanocockpit, Path(tmp_name))
        image_wire, states = render_inputs()
        gap_rows = run_csv(gap, image_wire)
        stm_rows: list[dict[str, str]] = []
        for case_id in range(len(CASES)):
            indexes = [i for i, state in enumerate(states)
                       if int(state["case"]) == case_id]
            selected_gap = [gap_rows[i] for i in indexes]
            selected_states = [states[i] for i in indexes]
            stm_rows.extend(run_csv(
                stm, stm32_wire(selected_gap, selected_states)))
        summary = summarize(stm_rows)
    output = json.dumps(summary, indent=2)
    print(output)
    if args.out:
        args.out.write_text(output+"\n")
    return 0 if summary["passed"] == summary["total"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
