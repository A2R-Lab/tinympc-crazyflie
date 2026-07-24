#!/usr/bin/env python3
"""Replay recorded 160x160 Himax PNGs through the exact firmware sources."""

from __future__ import annotations

import argparse
import csv
import json
import math
import struct
import tempfile
import zlib
from collections import Counter
from pathlib import Path

from run_exact_track_image_sim import build, run_csv, stm32_wire

VISION_PERIOD_US = 66_667
CAMERA_PERIOD_US = 33_333


def decode_gray_png(path: Path) -> bytes:
    """Decode the capture format without changing its grayscale samples."""
    raw = path.read_bytes()
    if raw[:8] != b"\x89PNG\r\n\x1a\n":
        raise ValueError(f"{path}: not a PNG")
    pos = 8
    compressed = bytearray()
    width = height = bit_depth = color_type = None
    while pos < len(raw):
        length = struct.unpack(">I", raw[pos:pos + 4])[0]
        kind = raw[pos + 4:pos + 8]
        data = raw[pos + 8:pos + 8 + length]
        pos += 12 + length
        if kind == b"IHDR":
            width, height, bit_depth, color_type = struct.unpack(">IIBB", data[:10])
        elif kind == b"IDAT":
            compressed.extend(data)
        elif kind == b"IEND":
            break
    if (width, height, bit_depth, color_type) != (160, 160, 8, 0):
        raise ValueError(
            f"{path}: expected 160x160 8-bit grayscale, got "
            f"{width}x{height}, depth={bit_depth}, type={color_type}")
    filtered = zlib.decompress(compressed)
    stride = width
    pixels = bytearray(width * height)
    previous = bytearray(stride)
    offset = 0
    for y in range(height):
        filter_type = filtered[offset]
        scanline = filtered[offset + 1:offset + 1 + stride]
        offset += stride + 1
        current = bytearray(stride)
        for x, sample in enumerate(scanline):
            left = current[x - 1] if x else 0
            up = previous[x]
            upper_left = previous[x - 1] if x else 0
            if filter_type == 0:
                value = sample
            elif filter_type == 1:
                value = sample + left
            elif filter_type == 2:
                value = sample + up
            elif filter_type == 3:
                value = sample + ((left + up) // 2)
            elif filter_type == 4:
                estimate = left + up - upper_left
                dl = abs(estimate - left)
                du = abs(estimate - up)
                dul = abs(estimate - upper_left)
                predictor = left if dl <= du and dl <= dul else up if du <= dul else upper_left
                value = sample + predictor
            else:
                raise ValueError(f"{path}: unsupported PNG filter {filter_type}")
            current[x] = value & 0xFF
        pixels[y * stride:(y + 1) * stride] = current
        previous = current
    return bytes(pixels)


def decode_gray_pgm(path: Path) -> bytes:
    """Decode the raw P5 format emitted by the low-overhead capture writer."""
    raw = path.read_bytes()
    tokens = []
    pos = 0
    while len(tokens) < 4:
        while pos < len(raw) and raw[pos] in b" \t\r\n":
            pos += 1
        if pos < len(raw) and raw[pos] == ord("#"):
            while pos < len(raw) and raw[pos] not in b"\r\n":
                pos += 1
            continue
        start = pos
        while pos < len(raw) and raw[pos] not in b" \t\r\n":
            pos += 1
        tokens.append(raw[start:pos])
    if tokens != [b"P5", b"160", b"160", b"255"]:
        raise ValueError(f"{path}: expected raw 160x160 8-bit PGM")
    if pos >= len(raw) or raw[pos] not in b" \t\r\n":
        raise ValueError(f"{path}: missing PGM raster separator")
    if raw[pos:pos + 2] == b"\r\n":
        pos += 2
    else:
        pos += 1
    pixels = raw[pos:]
    if len(pixels) != 160 * 160:
        raise ValueError(f"{path}: expected 25,600 pixels, got {len(pixels)}")
    return pixels


def decode_gray_image(path: Path) -> bytes:
    if path.suffix.lower() == ".png":
        return decode_gray_png(path)
    if path.suffix.lower() == ".pgm":
        return decode_gray_pgm(path)
    raise ValueError(f"{path}: unsupported capture image format")


def number(row: dict[str, str], key: str, default: float = 0.0) -> float:
    try:
        return float(row[key])
    except (KeyError, TypeError, ValueError):
        return default


def deadline_select(rows: list[dict[str, str]]) -> list[dict[str, str]]:
    selected = []
    deadline = 0
    for row in rows:
        timestamp = int(row["frame_gap8_timestamp"])
        if deadline == 0 or ((timestamp - deadline) & 0xFFFFFFFF) < 0x80000000:
            selected.append(row)
            if deadline == 0:
                deadline = timestamp
            while ((timestamp - deadline) & 0xFFFFFFFF) < 0x80000000:
                deadline = (deadline + VISION_PERIOD_US) & 0xFFFFFFFF
    return selected


def make_inputs(capture: Path) -> tuple[bytes, list[dict[str, float | int]],
                                        list[dict[str, str]], int]:
    with (capture / "frames.csv").open(newline="") as stream:
        all_rows = list(csv.DictReader(stream))
    rows = deadline_select(all_rows)
    wire = bytearray()
    states: list[dict[str, float | int]] = []
    previous_yaw = None
    previous_ts = None
    for frame, row in enumerate(rows):
        timestamp = int(row["frame_gap8_timestamp"])
        image = decode_gray_image(capture / row["image"])
        wire.extend(struct.pack("<III", 0, frame, timestamp))
        wire.extend(image)
        yaw = math.radians(number(row, "log_yaw_deg"))
        world_vx = number(row, "log_vx")
        world_vy = number(row, "log_vy")
        body_vx = math.cos(yaw) * world_vx + math.sin(yaw) * world_vy
        body_vy = -math.sin(yaw) * world_vx + math.cos(yaw) * world_vy
        yaw_rate = 0.0
        if previous_yaw is not None and previous_ts is not None:
            dyaw = math.atan2(math.sin(yaw - previous_yaw), math.cos(yaw - previous_yaw))
            dt = ((timestamp - previous_ts) & 0xFFFFFFFF) * 1e-6
            if dt > 0.0:
                yaw_rate = dyaw / dt
        states.append({
            "case": 0, "frame": frame, "tick": timestamp // 1000,
            "vx": body_vx, "vy": body_vy, "yaw_rate": yaw_rate,
            "x": number(row, "log_x"), "y": number(row, "log_y"),
            "yaw": yaw, "clock_bias_ms": 0,
        })
        previous_yaw, previous_ts = yaw, timestamp
    return bytes(wire), states, rows, len(all_rows)


def summarize(capture_rows: list[dict[str, str]],
              gap_rows: list[dict[str, str]],
              stm_rows: list[dict[str, str]],
              saved_frames: int) -> dict[str, object]:
    gaps = []
    for first, second in zip(capture_rows, capture_rows[1:]):
        gaps.append(((int(second["frame_gap8_timestamp"]) -
                      int(first["frame_gap8_timestamp"])) & 0xFFFFFFFF))
    phase_counts = Counter(row["phase"] for row in capture_rows)
    valid = [row for row in stm_rows if int(float(row["cyl_valid"]))]
    observations = [row for row in stm_rows if int(float(row["obs_valid"]))]
    accepted = [row for row in stm_rows if int(row["accepted"]) > 0]
    valid_steps = [
        math.hypot(float(second["cyl_world_x"]) - float(first["cyl_world_x"]),
                   float(second["cyl_world_y"]) - float(first["cyl_world_y"]))
        for first, second in zip(valid, valid[1:])
    ]
    def frame_info(output: dict[str, str]) -> dict[str, object]:
        index = int(output["frame"])
        source = capture_rows[index]
        return {
            "replay_frame": index,
            "image": source["image"],
            "phase": source["phase"],
            "gap8_timestamp": int(source["frame_gap8_timestamp"]),
            "world_x_m": number(source, "log_x"),
            "world_y_m": number(source, "log_y"),
            "yaw_deg": number(source, "log_yaw_deg"),
        }
    return {
        "implementation": {
            "gap8": "verbatim production functions extracted from pulp-frontnet/main.c",
            "stm32": "production flowdeck_obstacle_link.c directly included by host harness",
            "pixels": "native 160x160 8-bit grayscale samples; no resize or filtering",
            "selection": "production accumulated 66,667 us vision deadline applied to saved frames",
        },
        "coverage": {
            "saved_frames": saved_frames,
            "selected_frames": len(capture_rows),
            "selected_by_phase": dict(phase_counts),
            "selected_pair_gap_camera_frames": dict(sorted(Counter(
                str(round(gap / CAMERA_PERIOD_US)) for gap in gaps).items(),
                key=lambda item: int(item[0]))),
            "nominal_pairs": sum(abs(gap - VISION_PERIOD_US) <= 100 for gap in gaps),
            "non_nominal_pairs": sum(abs(gap - VISION_PERIOD_US) > 100 for gap in gaps),
        },
        "frontend": {
            "frames_with_tracks": sum(int(row["track_count"]) > 0 for row in gap_rows),
            "max_track_count": max(int(row["track_count"]) for row in gap_rows),
        },
        "estimator": {
            "frames_with_accepted_depth": len(accepted),
            "frames_with_observation": len(observations),
            "frames_with_valid_cylinder": len(valid),
            "max_accepted_tracks": max(int(row["accepted"]) for row in stm_rows),
            "max_cluster_support": max(int(row["support"]) for row in stm_rows),
            "max_baseline_m": max(float(row["baseline"]) for row in stm_rows),
            "max_map_peak": max(float(row["map_peak"]) for row in stm_rows),
            "first_accepted": frame_info(accepted[0]) if accepted else None,
            "first_observation": frame_info(observations[0]) if observations else None,
            "first_valid_cylinder": ({
                **frame_info(valid[0]),
                "estimated_world_x_m": float(valid[0]["cyl_world_x"]),
                "estimated_world_y_m": float(valid[0]["cyl_world_y"]),
                "confidence": float(valid[0]["cyl_conf"]),
            } if valid else None),
            "max_valid_cylinder_step_m": max(valid_steps, default=0.0),
            "last_valid_cylinder": ({
                **frame_info(valid[-1]),
                "estimated_world_x_m": float(valid[-1]["cyl_world_x"]),
                "estimated_world_y_m": float(valid[-1]["cyl_world_y"]),
                "confidence": float(valid[-1]["cyl_conf"]),
            } if valid else None),
        },
        "limitations": [
            "The Wi-Fi recording omitted on-board camera frames; non-nominal selected gaps "
            "cannot reproduce the flow pairs processed in flight.",
            "No measured obstacle world coordinate or dimensions are present, so absolute "
            "range/bearing error cannot be scored from these files alone.",
            "Gyroscope samples were not logged; yaw rate is derived from logged yaw.",
        ],
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("capture", type=Path)
    parser.add_argument("--nanocockpit", type=Path, required=True)
    parser.add_argument("--out", type=Path)
    args = parser.parse_args()
    image_wire, states, capture_rows, saved_frames = make_inputs(args.capture)
    repo = Path(__file__).resolve().parents[3]
    with tempfile.TemporaryDirectory(prefix="himax-capture-replay-") as tmp:
        gap, stm = build(repo, args.nanocockpit, Path(tmp))
        gap_rows = run_csv(gap, image_wire)
        stm_rows = run_csv(stm, stm32_wire(gap_rows, states))
    result = summarize(capture_rows, gap_rows, stm_rows, saved_frames)
    output = json.dumps(result, indent=2)
    print(output)
    if args.out:
        args.out.write_text(output + "\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
