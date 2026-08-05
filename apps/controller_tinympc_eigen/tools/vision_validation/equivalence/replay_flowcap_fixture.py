#!/usr/bin/env python3
"""Replay a parsed FLOWCAP PGM pair through exact extracted GAP8 C code."""

from __future__ import annotations

import argparse
import struct
import subprocess
import sys
import tempfile
from pathlib import Path

HERE = Path(__file__).resolve().parent


def read_pgm(path: Path) -> bytes:
    data = path.read_bytes()
    fields = data.split(b"\n", 3)
    if len(fields) != 4 or fields[0] != b"P5":
        raise ValueError(f"{path}: expected binary P5 PGM")
    width, height = map(int, fields[1].split())
    maximum = int(fields[2])
    if (width, height, maximum) != (160, 160, 255):
        raise ValueError(f"{path}: expected 160x160 uint8 image")
    pixels = fields[3]
    if len(pixels) != 160*160:
        raise ValueError(f"{path}: truncated pixel payload")
    return pixels


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("fixture_prefix", type=Path)
    parser.add_argument("--nanocockpit", type=Path, required=True)
    args = parser.parse_args()
    metadata = __import__("json").loads(
        args.fixture_prefix.with_suffix(".json").read_text())
    prev = read_pgm(args.fixture_prefix.with_name(
        f"{args.fixture_prefix.name}_prev.pgm"))
    cur = read_pgm(args.fixture_prefix.with_name(
        f"{args.fixture_prefix.name}_cur.pgm"))
    frames = (
        struct.pack("<III", 0, 0, int(metadata["prev_ts_us"])) + prev +
        struct.pack("<III", 0, 1, int(metadata["cur_ts_us"])) + cur
    )
    with tempfile.TemporaryDirectory(prefix="flowcap-replay-") as directory:
        executable = Path(directory) / "gap8_replay"
        subprocess.run([
            sys.executable, str(HERE / "build_gap8_host_replay.py"),
            "--nanocockpit", str(args.nanocockpit),
            "--output", str(executable),
        ], check=True)
        completed = subprocess.run([str(executable)], input=frames, check=True)
    raise SystemExit(completed.returncode)


if __name__ == "__main__":
    main()
