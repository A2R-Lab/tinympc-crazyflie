#!/usr/bin/env python3
"""Compare the host-compiled STM32 estimator source with its Python mirror.

The deterministic corpus contains 18 twelve-frame cases (216 controller
updates) at 15 Hz and exercises all estimator paths added for obstacle-quality
work. The C executable includes the production flowdeck_obstacle_link.c file.
"""

from __future__ import annotations

import argparse
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
from sim_flow_obstacle_sectors import FirmwareMirror, Sector  # noqa: E402

DT = 1.0 / 15.0
AZIMUTHS = [math.radians(-32 + 8 * i) for i in range(9)]


def f32(value: float) -> float:
    return struct.unpack("<f", struct.pack("<f", value))[0]


@dataclass(frozen=True)
class Case:
    name: str
    distance: float = 1.0
    vx: float = 0.0
    vy: float = 0.5
    yaw_rate: float = 0.0
    confidence: float = 0.6
    mode: str = "obstacle"
    active: tuple[int, ...] = tuple(range(9))


CASES = (
    Case("range_0p5", 0.5),
    Case("range_0p75", 0.75),
    Case("range_1p0", 1.0),
    Case("range_1p5", 1.5),
    Case("range_2p0", 2.0),
    Case("range_2p5", 2.5),
    Case("forward_looming", 1.2, vx=0.6, vy=0.0),
    Case("combined_yaw", 1.1, vx=0.2, vy=0.5, yaw_rate=0.12),
    Case("narrow_two_sector", 1.0, active=(3, 4)),
    Case("noncontiguous_foreground", mode="foreground"),
    Case("single_edge_foreground_far_wall", mode="edge_background"),
    Case("intermittent_two_of_three", mode="intermittent"),
    Case("spatial_gate_jump", mode="jump"),
    Case("duplicate_controller_reads", mode="duplicates"),
    Case("parallax_looming_disagreement", 1.0, vx=0.4, vy=0.5,
         mode="disagreement"),
    Case("stationary", mode="stationary", vx=0.0, vy=0.0),
    Case("pure_yaw", mode="pure_yaw", vx=0.0, vy=0.0, yaw_rate=0.5),
    Case("low_texture", mode="low_texture", confidence=0.005),
)


def sectors_for(case: Case, frame: int) -> list[Sector]:
    if case.mode in {"stationary", "low_texture"}:
        return [Sector(f32(math.tan(az)), 0.0, f32(case.confidence), flow_y=0.0)
                for az in AZIMUTHS]
    if case.mode == "pure_yaw":
        return [Sector(f32(math.tan(az)),
                       f32(case.yaw_rate * (1.0 + math.tan(az) ** 2)),
                       f32(case.confidence), flow_y=0.0) for az in AZIMUTHS]
    if case.mode == "intermittent" and frame % 3 == 1:
        return [Sector(f32(math.tan(az)), 0.0, 0.0, flow_y=0.0)
                for az in AZIMUTHS]

    result = []
    for i, az in enumerate(AZIMUTHS):
        q = math.tan(az)
        confidence = case.confidence if i in case.active else 0.0
        distance = case.distance
        if case.mode == "jump":
            distance = 0.7 if frame % 2 == 0 else 2.0
        elif case.mode == "foreground":
            distance = 1.0 if i in (1, 4, 7) else 2.2
            confidence = 0.9 if i in (1, 4, 7) else 0.3
        elif case.mode == "edge_background":
            distance = 0.5 if i == 0 else 2.7
            confidence = 0.6 if i in (0, 4, 5) else 0.0
        vel_eff = case.vx * math.sin(az) - case.vy * math.cos(az)
        angular = case.yaw_rate + vel_eff / distance
        looming = case.vx / distance if abs(case.vx) >= 0.15 else 0.0
        if case.mode == "disagreement":
            looming = 4.0
        result.append(Sector(f32(q), f32(angular * (1.0 + q*q)),
                             f32(confidence), flow_y=f32(looming)))
    return result


def is_new_sample(case: Case, frame: int) -> bool:
    return not (case.mode == "duplicates" and frame not in (0, 6))


def build_harness(output: Path) -> None:
    command = [
        "cc", "-std=c11", "-O2", "-Wall", "-Wextra", "-Werror",
        "-Wno-unused-variable", "-I", str(HERE / "stubs"),
        str(HERE / "stm32_estimator_harness.c"), "-lm", "-o", str(output),
    ]
    subprocess.run(command, check=True)


def input_and_python(case: Case) -> tuple[str, list[dict[str, object]]]:
    mirror = FirmwareMirror()
    lines = []
    expected = []
    world_x = world_y = yaw = 0.0
    latest_sectors: list[Sector] | None = None
    for frame in range(12):
        new_sample = is_new_sample(case, frame)
        if new_sample or latest_sectors is None:
            latest_sectors = sectors_for(case, frame)
        sectors = latest_sectors
        values = [float(new_sample), f32(DT), f32(case.vx), f32(case.vy),
                  f32(case.yaw_rate), f32(world_x), f32(world_y), f32(yaw)]
        for sector in sectors:
            values.extend((sector.azimuth, sector.flow_x, sector.flow_y,
                           sector.confidence))
        line = " ".join(f"{value:.9g}" for value in values)
        parsed = [float(token) for token in line.split()]
        parsed_sectors = [
            Sector(*parsed[8 + 4*i:8 + 4*i + 2],
                   confidence=parsed[8 + 4*i + 3],
                   flow_y=parsed[8 + 4*i + 2]) for i in range(9)
        ]
        result = mirror.update(parsed_sectors, *parsed[2:5], *parsed[5:8],
                               new_sample=new_sample, dt=parsed[1])
        expected.append(result)
        lines.append(line)
        world_x += case.vx * DT
        world_y += case.vy * DT
        yaw += case.yaw_rate * DT
    return "\n".join(lines) + "\n", expected


def parse_c_row(line: str) -> dict[str, object]:
    values = [float(value) for value in line.split()]
    sectors = []
    at = 2
    for _ in range(9):
        sectors.append(dict(zip(("valid", "inv_depth", "range", "body_x", "body_y"),
                                values[at:at+5])))
        at += 5
    return {
        "sectors": sectors,
        "cluster": {
            "start": int(values[at]), "count": int(values[at+1]),
            "valid": bool(values[at+2]), "range": values[at+3],
            "body_x": values[at+4], "body_y": values[at+5],
            "score": values[at+6], "aggregate_displacement": values[at+7],
            "yaw_ratio": values[at+8], "depth_disagreement": values[at+9],
            "reject": int(values[at+10]),
        },
        "cylinder": {
            "valid": bool(values[at+11]), "confidence": values[at+12],
            "world_x": values[at+13], "world_y": values[at+14],
            "age": values[at+15],
        },
        "obs_hits": int(values[at+16]),
    }


def compare(case: Case, c_rows: list[dict[str, object]],
            py_rows: list[dict[str, object]]) -> dict[str, object]:
    max_error = 0.0
    mismatches = []
    comparisons = 0
    for frame, (actual, expected) in enumerate(zip(c_rows, py_rows)):
        duplicate = not is_new_sample(case, frame)
        if not duplicate:
            for index, (c_sector, py_sector) in enumerate(
                    zip(actual["sectors"], expected["sectors"])):
                py_valid = bool(py_sector.get("valid", False))
                if bool(c_sector["valid"]) != py_valid:
                    mismatches.append(f"frame {frame} sector {index} validity")
                comparisons += 1
                if py_valid:
                    for field in ("inv_depth", "range", "body_x", "body_y"):
                        error = abs(float(c_sector[field]) - float(py_sector[field]))
                        max_error = max(max_error, error)
            c_cluster, py_cluster = actual["cluster"], expected["cluster"]
            if c_cluster["count"] != py_cluster["count"]:
                mismatches.append(f"frame {frame} group count")
            if bool(c_cluster["valid"]) != bool(py_cluster["valid"]):
                mismatches.append(f"frame {frame} persistence validity")
            for c_name, py_name in (
                ("aggregate_displacement", "aggregate_displacement"),
                ("yaw_ratio", "yaw_ratio"),
            ):
                error = abs(float(c_cluster[c_name]) - float(py_cluster[py_name]))
                max_error = max(max_error, error)
        if bool(actual["cylinder"]["valid"]) != bool(expected["cylinder"]["valid"]):
            mismatches.append(f"frame {frame} cylinder validity")
        if actual["cylinder"]["valid"] and expected["cylinder"]["valid"]:
            for field in ("confidence", "world_x", "world_y"):
                error = abs(float(actual["cylinder"][field]) -
                            float(expected["cylinder"][field]))
                max_error = max(max_error, error)
    return {
        "name": case.name, "frames": len(c_rows), "comparisons": comparisons,
        "max_abs_error": max_error, "mismatches": mismatches,
        "passed": not mismatches and max_error <= 2.0e-5,
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--out", type=Path,
                        default=TOOLS / "results" / "flow_15hz_equivalence.json")
    args = parser.parse_args()
    args.out.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix="tinympc-equivalence-") as tmp:
        executable = Path(tmp) / "stm32_estimator_harness"
        build_harness(executable)
        results = []
        for case in CASES:
            stdin, expected = input_and_python(case)
            process = subprocess.run([str(executable)], input=stdin, text=True,
                                     capture_output=True, check=True)
            rows = [parse_c_row(line) for line in process.stdout.splitlines()]
            results.append(compare(case, rows, expected))
    summary = {
        "rate_hz": 15, "cases": len(CASES),
        "frames": sum(item["frames"] for item in results),
        "compiled_source": "apps/controller_tinympc_eigen/src/flowdeck_obstacle_link.c",
        "python_mirror": "apps/controller_tinympc_eigen/tools/sim_flow_obstacle_sectors.py",
        "max_abs_error": max(item["max_abs_error"] for item in results),
        "all_passed": all(item["passed"] for item in results),
        "results": results,
    }
    args.out.write_text(json.dumps(summary, indent=2) + "\n")
    print(json.dumps(summary, indent=2))
    return 0 if summary["all_passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
