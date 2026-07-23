#!/usr/bin/env python3
"""Launch one physical-corpus case with metadata copied from its manifest."""

import argparse
import json
import shlex
import subprocess
import sys
from pathlib import Path


DEFAULT_MANIFEST = Path(__file__).with_name("obstacle_corpus.json")
APP_DIR = Path(__file__).resolve().parents[2]
LOGGER = APP_DIR / "bench_flow_obstacle_log.py"


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("case_id")
    parser.add_argument("--manifest", type=Path, default=DEFAULT_MANIFEST)
    parser.add_argument("--logs-dir", type=Path,
                        default=Path(__file__).with_name("logs"))
    parser.add_argument("--uri", default="radio://0/80/2M/E7E7E7E7E8")
    parser.add_argument("--duration", type=float, default=30.0)
    parser.add_argument("--start-delay", type=float, default=5.0)
    parser.add_argument("--period-ms", type=int, default=30)
    parser.add_argument("--sample-hz", type=float, default=30.0)
    parser.add_argument("--truth-world-x", type=float)
    parser.add_argument("--truth-world-y", type=float)
    parser.add_argument("--truth-start-range-m", type=float)
    parser.add_argument("--truth-start-bearing-deg", type=float)
    parser.add_argument("--truth-obstacle-width-m", type=float)
    parser.add_argument("--truth-orientation-deg", type=float)
    parser.add_argument("--notes", default="")
    parser.add_argument("--dry-run", action="store_true")
    return parser.parse_args()


def find_case(manifest, case_id):
    matches = [case for case in manifest["cases"] if case["id"] == case_id]
    if len(matches) != 1:
        raise SystemExit(f"expected exactly one manifest case {case_id!r}, found {len(matches)}")
    return matches[0]


def command_for(case, args):
    world_complete = (
        args.truth_world_x is not None and args.truth_world_y is not None
    )
    relative_complete = (
        args.truth_start_range_m is not None
        and args.truth_start_bearing_deg is not None
    )
    world_partial = (
        (args.truth_world_x is None) != (args.truth_world_y is None)
    )
    relative_partial = (
        (args.truth_start_range_m is None)
        != (args.truth_start_bearing_deg is None)
    )
    if world_partial or relative_partial or (world_complete and relative_complete):
        raise SystemExit(
            "provide exactly one complete ground-truth pair: world x/y or "
            "initial range/bearing"
        )
    if case["label"] == "positive" and not (
        world_complete or relative_complete
    ):
        raise SystemExit(
            "positive cases require measured --truth-world-x/--truth-world-y "
            "or --truth-start-range-m/--truth-start-bearing-deg"
        )
    if case["label"] == "positive" and (
        args.truth_obstacle_width_m is None
        or args.truth_orientation_deg is None
    ):
        raise SystemExit(
            "positive cases require measured --truth-obstacle-width-m and "
            "--truth-orientation-deg"
        )
    if (args.truth_obstacle_width_m is not None
            and args.truth_obstacle_width_m <= 0.0):
        raise SystemExit("--truth-obstacle-width-m must be positive")
    output = args.logs_dir / f"{case['id']}.csv"
    command = [
        sys.executable, str(LOGGER),
        "--uri", args.uri,
        "--out", str(output),
        "--duration", str(args.duration),
        "--start-delay", str(args.start_delay),
        "--period-ms", str(args.period_ms),
        "--sample-hz", str(args.sample_hz),
        "--case-id", case["id"],
        "--configuration", str(case["configuration_features"]),
        "--label", case["label"],
        "--obstacle-shape", case["obstacle_shape"],
        "--obstacle-width", case["obstacle_width"],
        "--depth-configuration", case["depth_configuration"],
        "--texture", case["texture"],
        "--lighting", case["lighting"],
        "--motion", case["motion"],
        "--no-plot",
    ]
    for option, key in (
        ("--distance-m", "distance_m"),
        ("--orientation-deg", "orientation_deg"),
        ("--lateral-offset-m", "lateral_offset_m"),
    ):
        if case[key] is not None:
            command.extend((option, str(case[key])))
    if args.truth_world_x is not None:
        command.extend(("--truth-world-x", str(args.truth_world_x)))
    if args.truth_world_y is not None:
        command.extend(("--truth-world-y", str(args.truth_world_y)))
    if args.truth_start_range_m is not None:
        command.extend((
            "--truth-start-range-m", str(args.truth_start_range_m)
        ))
    if args.truth_start_bearing_deg is not None:
        command.extend((
            "--truth-start-bearing-deg", str(args.truth_start_bearing_deg)
        ))
    if args.truth_obstacle_width_m is not None:
        command.extend((
            "--truth-obstacle-width-m", str(args.truth_obstacle_width_m)
        ))
    if args.truth_orientation_deg is not None:
        command.extend((
            "--truth-orientation-deg", str(args.truth_orientation_deg)
        ))
    if args.notes:
        command.extend(("--notes", args.notes))
    return output, command


def main():
    args = parse_args()
    manifest = json.loads(args.manifest.read_text())
    case = find_case(manifest, args.case_id)
    output, command = command_for(case, args)
    print(json.dumps(case, indent=2))
    print("command:", shlex.join(command))
    if args.dry_run:
        return
    if output.exists() or output.with_suffix(output.suffix + ".json").exists():
        raise SystemExit(f"refusing to overwrite existing case output: {output}")
    args.logs_dir.mkdir(parents=True, exist_ok=True)
    subprocess.run(command, cwd=APP_DIR, check=True)


if __name__ == "__main__":
    main()
