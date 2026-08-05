#!/usr/bin/env python3
"""Replay one 15 Hz corpus through compiled firmware and Python mirrors."""

from __future__ import annotations

import argparse
import csv
import hashlib
import io
import json
import math
import struct
import subprocess
import sys
import tempfile
from dataclasses import asdict, dataclass
from pathlib import Path

HERE = Path(__file__).resolve().parent
TOOLS = HERE.parent
sys.path.insert(0, str(TOOLS))

from sim_flow_obstacle_sectors import FirmwareMirror, Sector  # noqa: E402
from sim_gap8_deployment_suite import render_gap8  # noqa: E402
from sim_gap8_flow_frontend import Gap8FlowFrontend  # noqa: E402
from sim_monocular_flow_suite import Box, Scene  # noqa: E402


FRAME_DT_US = 66_667
FRAME_DT_S = FRAME_DT_US * 1.0e-6
FRAMES_PER_CASE = 12
TARGET = Box("target", 3.0, 3.25, -0.45, 0.45, 1)
SCENE = Scene("equivalence_box", (TARGET,), 0.0, 0.0, (3.0, 0.0))


@dataclass(frozen=True)
class Case:
    case_id: int
    distance_m: float
    obstacle_orientation_deg: float
    lateral_offset_m: float


def cases() -> list[Case]:
    result = []
    case_id = 0
    for distance in (0.75, 1.50, 2.50):
        for orientation in (-45.0, -20.0, 0.0, 20.0, 45.0):
            result.append(Case(case_id, distance, orientation, 0.0))
            case_id += 1
    # Off-axis cases exercise sector boundaries independently of face angle.
    for lateral in (-0.30, 0.30):
        result.append(Case(case_id, 1.50, 0.0, lateral))
        case_id += 1
    return result


def render_corpus() -> tuple[list[dict[str, object]], bytes]:
    records: list[dict[str, object]] = []
    wire = bytearray()
    for case in cases():
        view = math.radians(case.obstacle_orientation_deg)
        heading = (math.cos(view), math.sin(view))
        side = (-heading[1], heading[0])
        start = (
            3.0-case.distance_m*heading[0]+case.lateral_offset_m*side[0],
            -case.distance_m*heading[1]+case.lateral_offset_m*side[1],
        )
        for frame_id in range(FRAMES_PER_CASE):
            timestamp_us = (frame_id + 1) * FRAME_DT_US
            travel = 0.75*frame_id*FRAME_DT_S
            pos = (
                start[0]+heading[0]*travel,
                start[1]+heading[1]*travel,
            )
            image = render_gap8(SCENE, pos[0], pos[1], view)
            if len(image) != 160 or any(len(row) != 160 for row in image):
                raise RuntimeError("unexpected rendered frame dimensions")
            wire.extend(struct.pack("<III", case.case_id, frame_id, timestamp_us))
            wire.extend(bytes(value for row in image for value in row))
            records.append({
                "case": case,
                "frame": frame_id,
                "timestamp_us": timestamp_us,
                "position": pos,
                "yaw": view,
                "image": image,
            })
    return records, bytes(wire)


def run_csv(executable: Path, input_bytes: bytes) -> list[dict[str, str]]:
    completed = subprocess.run(
        [str(executable)], input=input_bytes, capture_output=True, check=True,
    )
    return list(csv.DictReader(io.StringIO(completed.stdout.decode())))


def build_gap8(nanocockpit: Path, output: Path) -> None:
    subprocess.run([
        sys.executable, str(HERE / "build_gap8_host_replay.py"),
        "--nanocockpit", str(nanocockpit), "--output", str(output),
    ], check=True)


def build_stm32(repo: Path, output: Path) -> None:
    subprocess.run([
        "cc", "-std=c11", "-O2", "-Wall", "-Wextra",
        "-Wno-unused-variable",
        "-I", str(HERE / "mocks"),
        "-I", str(HERE / "stubs"),
        "-I", str(repo / "apps/controller_tinympc_eigen/src"),
        str(HERE / "stm32_flow_replay.c"), "-lm", "-o", str(output),
    ], check=True)


def python_gap8(records: list[dict[str, object]]) -> tuple[list[list[Sector]],
                                                           list[dict[str, int]]]:
    outputs: list[list[Sector]] = []
    diagnostics: list[dict[str, int]] = []
    frontend: Gap8FlowFrontend | None = None
    active_case = -1
    for record in records:
        case = record["case"]
        assert isinstance(case, Case)
        if case.case_id != active_case:
            frontend = Gap8FlowFrontend(balanced_features=True,
                                        near_aggregation=True)
            active_case = case.case_id
        assert frontend is not None
        sectors, diag = frontend.process(
            record["image"], FRAME_DT_S,
        )
        if sectors is None:
            sectors = [
                Sector(((i+0.5)*160/9-81.10381)/89.15584, 0.0, 0.0,
                       flow_y=0.0)
                for i in range(9)
            ]
        outputs.append(sectors)
        diagnostics.append(diag)
    return outputs, diagnostics


def compare_gap8(compiled: list[dict[str, str]],
                 modeled: list[list[Sector]],
                 diagnostics: list[dict[str, int]]) -> dict[str, object]:
    maxima = {"azimuth": 0.0, "flow_x": 0.0, "flow_y": 0.0,
              "confidence": 0.0}
    mismatches = 0
    examples = []
    diagnostic_mismatches = 0
    for row, sectors, diag in zip(compiled, modeled, diagnostics, strict=True):
        if (int(row["selected"]) != diag["features"] or
                int(row["accepted"]) != diag["tracks"] or
                int(row["feature_x_sum"]) != diag["feature_x_sum"] or
                int(row["feature_y_sum"]) != diag["feature_y_sum"] or
                int(row["feature_score_sum"]) != diag["feature_score_sum"] or
                abs(float(row["track_dx_sum"])-diag["track_dx_sum"]) > 2e-5 or
                abs(float(row["track_dy_sum"])-diag["track_dy_sum"]) > 2e-5 or
                any(int(row[f"count{i}"]) != diag["sector_counts"][i]
                    for i in range(9))):
            diagnostic_mismatches += 1
        for index, sector in enumerate(sectors):
            values = {
                "azimuth": (float(row[f"q{index}"]), sector.azimuth),
                "flow_x": (float(row[f"fx{index}"]), sector.flow_x),
                "flow_y": (float(row[f"fy{index}"]), sector.flow_y),
                "confidence": (float(row[f"c{index}"]), sector.confidence),
            }
            for name, (actual, expected) in values.items():
                error = abs(actual-expected)
                maxima[name] = max(maxima[name], error)
                if error > 2.0e-5:
                    mismatches += 1
                    if len(examples) < 12:
                        examples.append({
                            "case": int(row["case"]),
                            "frame": int(row["frame"]),
                            "sector": index,
                            "field": name,
                            "compiled": actual,
                            "python": expected,
                        })
    trace = [{
        "frame": int(compiled[i]["frame"]),
        "count4": int(compiled[i]["count4"]),
        "compiled_c4": float(compiled[i]["c4"]),
        "python_c4": modeled[i][4].confidence,
    } for i in range(min(8, len(compiled)))]
    return {"rows": len(compiled), "sector_values": len(compiled)*9*4,
            "mismatches_over_2e-5": mismatches, "max_abs_error": maxima,
            "diagnostic_count_mismatches": diagnostic_mismatches,
            "examples": examples, "trace_case0_sector4": trace}


def compiled_sectors(rows: list[dict[str, str]]) -> list[list[Sector]]:
    return [[
        Sector(
            float(row[f"q{index}"]),
            float(row[f"fx{index}"]),
            float(row[f"c{index}"]),
            flow_y=float(row[f"fy{index}"]),
        )
        for index in range(9)
    ] for row in rows]


def stm32_input(records: list[dict[str, object]],
                sectors_by_frame: list[list[Sector]]) -> bytes:
    rows = []
    for record, sectors in zip(records, sectors_by_frame, strict=True):
        case = record["case"]
        assert isinstance(case, Case)
        pos = record["position"]
        values: list[float | int] = [
            case.case_id, int(record["frame"]),
            int(record["timestamp_us"]) // 1000,
            0.75, 0.0, 0.0, pos[0], pos[1], record["yaw"], FRAME_DT_S,
        ]
        for sector in sectors:
            values.extend((sector.azimuth, sector.flow_x,
                           sector.flow_y, sector.confidence))
        rows.append(",".join(f"{value:.12g}" if isinstance(value, float)
                             else str(value) for value in values))
    return ("\n".join(rows) + "\n").encode()


def run_stm32_by_case(executable: Path,
                      records: list[dict[str, object]],
                      sectors_by_frame: list[list[Sector]]) -> list[dict[str, str]]:
    """Use a fresh process per independent scene, resetting firmware statics."""
    output = []
    for case in cases():
        indexes = [
            index for index, record in enumerate(records)
            if isinstance(record["case"], Case) and
            record["case"].case_id == case.case_id
        ]
        case_records = [records[index] for index in indexes]
        case_sectors = [sectors_by_frame[index] for index in indexes]
        output.extend(run_csv(
            executable, stm32_input(case_records, case_sectors),
        ))
    return output


def python_stm32(records: list[dict[str, object]],
                 sectors_by_frame: list[list[Sector]]) -> list[dict[str, object]]:
    rows = []
    mirror: FirmwareMirror | None = None
    active_case = -1
    for record, sectors in zip(records, sectors_by_frame, strict=True):
        case = record["case"]
        assert isinstance(case, Case)
        if case.case_id != active_case:
            mirror = FirmwareMirror()
            active_case = case.case_id
        assert mirror is not None
        x, y = record["position"]
        result = mirror.update(sectors, 0.75, 0.0, 0.0,
                               x, y, float(record["yaw"]), new_sample=True)
        rows.append(result)
    return rows


def compare_stm32(compiled: list[dict[str, str]],
                  modeled: list[dict[str, object]]) -> dict[str, object]:
    maxima = {"obs_world_x": 0.0, "obs_world_y": 0.0,
              "cyl_world_x": 0.0, "cyl_world_y": 0.0,
              "cyl_conf": 0.0}
    flag_mismatches = 0
    value_mismatches = 0
    for actual, expected in zip(compiled, modeled, strict=True):
        cluster = expected["cluster"]
        cylinder = expected["cylinder"]
        assert isinstance(cluster, dict) and isinstance(cylinder, dict)
        if bool(float(actual["obs_valid"])) != bool(cluster["valid"]):
            flag_mismatches += 1
        if bool(float(actual["cyl_valid"])) != bool(cylinder["valid"]):
            flag_mismatches += 1
        pairs = {
            "cyl_world_x": (float(actual["cyl_world_x"]),
                            float(cylinder["world_x"])),
            "cyl_world_y": (float(actual["cyl_world_y"]),
                            float(cylinder["world_y"])),
            "cyl_conf": (float(actual["cyl_conf"]),
                         float(cylinder["confidence"])),
        }
        # Firmware intentionally retains the last cluster coordinates while
        # obs_valid is false; the Python result omits those stale values.
        if bool(cluster["valid"]) and "world_x" in cluster:
            pairs["obs_world_x"] = (
                float(actual["obs_world_x"]), float(cluster["world_x"]))
            pairs["obs_world_y"] = (
                float(actual["obs_world_y"]), float(cluster["world_y"]))
        for name, (left, right) in pairs.items():
            error = abs(left-right)
            maxima[name] = max(maxima[name], error)
            if error > 2.0e-5:
                value_mismatches += 1
    return {"rows": len(compiled), "flag_mismatches": flag_mismatches,
            "value_mismatches_over_2e-5": value_mismatches,
            "max_abs_error": maxima}


def detection_coverage(rows: list[dict[str, str]]) -> dict[str, object]:
    per_case = []
    for case in cases():
        subset = [row for row in rows if int(row["case"]) == case.case_id]
        valid = [row for row in subset if float(row["cyl_valid"]) > 0.5]
        per_case.append({
            "case_id": case.case_id,
            "distance_m": case.distance_m,
            "obstacle_orientation_deg": case.obstacle_orientation_deg,
            "lateral_offset_m": case.lateral_offset_m,
            "detected": bool(valid),
            "first_valid_frame": int(valid[0]["frame"]) if valid else None,
            "peak_confidence": max(float(row["cyl_conf"]) for row in subset),
            "max_cluster_sectors": max(
                int(row["cluster_count"]) for row in subset),
            "max_cluster_hits": max(int(row["obs_hits"]) for row in subset),
        })
    return {
        "detected_cases": sum(item["detected"] for item in per_case),
        "total_cases": len(per_case),
        "missed_case_ids": [
            item["case_id"] for item in per_case if not item["detected"]
        ],
        "per_case": per_case,
    }


def git_head(repo: Path) -> str:
    return subprocess.run(
        ["git", "-C", str(repo), "rev-parse", "HEAD"],
        check=True, capture_output=True, text=True,
    ).stdout.strip()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--nanocockpit", type=Path,
                        default=Path("/home/cchen/tinympc-nanocockpit"))
    parser.add_argument("--out", type=Path)
    args = parser.parse_args()
    repo = HERE.parents[4]

    records, corpus_wire = render_corpus()
    state_wire = stm32_input(records, python_gap8(records)[0])
    with tempfile.TemporaryDirectory(prefix="flow-equiv-") as directory:
        temp = Path(directory)
        gap8_exe = temp / "gap8_replay"
        stm32_exe = temp / "stm32_replay"
        build_gap8(args.nanocockpit, gap8_exe)
        build_stm32(repo, stm32_exe)

        compiled_gap8 = run_csv(gap8_exe, corpus_wire)
        modeled_gap8, modeled_gap8_diag = python_gap8(records)
        gap8_result = compare_gap8(compiled_gap8, modeled_gap8,
                                   modeled_gap8_diag)
        firmware_gap8 = compiled_sectors(compiled_gap8)

        # Exercise each end-to-end path independently: compiled GAP8 output
        # enters compiled STM32, while Python GAP8 output enters its mirror.
        compiled_stm32 = run_stm32_by_case(
            stm32_exe, records, firmware_gap8,
        )
        modeled_stm32 = python_stm32(records, modeled_gap8)
        stm32_result = compare_stm32(compiled_stm32, modeled_stm32)

    manifest = {
        "rate_hz": 15,
        "frame_dt_us": FRAME_DT_US,
        "frames_per_case": FRAMES_PER_CASE,
        "frame_corpus_sha256": hashlib.sha256(corpus_wire).hexdigest(),
        "state_corpus_sha256": hashlib.sha256(state_wire).hexdigest(),
        "cases": [asdict(case) for case in cases()],
        "repositories": {
            "tinympc_crazyflie": git_head(repo),
            "tinympc_nanocockpit": git_head(args.nanocockpit),
        },
        "gap8": gap8_result,
        "stm32": stm32_result,
        "detection_coverage": detection_coverage(compiled_stm32),
    }
    text = json.dumps(manifest, indent=2)
    print(text)
    if args.out:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        args.out.write_text(text + "\n")
    passed = (
        gap8_result["mismatches_over_2e-5"] == 0 and
        stm32_result["flag_mismatches"] == 0 and
        stm32_result["value_mismatches_over_2e-5"] == 0
    )
    return 0 if passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
