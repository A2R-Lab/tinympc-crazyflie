#!/usr/bin/env python3
"""Extract raw generated-C GVSOC outputs/cycles; never infer them from wall time."""
from __future__ import annotations

import argparse
import json
import re
from pathlib import Path

import numpy as np


def deployment_flash_accounting(app_dir: Path) -> dict[str, int]:
    """Return a conservative deployable-image bound from the built GVSOC image.

    The instrumented image contains 200 raw evaluation clips that are not part
    of a deployed policy.  Subtract only those byte-for-byte payloads from the
    complete generated flash image.  All code, filesystem metadata, DORY
    weights, the generated default input, alignment, and boot payload remain,
    so the result is an upper bound rather than a weights-only estimate.
    """
    images = list(app_dir.glob(
        "BUILD/**/target.board.devices.flash.img"))
    if len(images) != 1:
        raise RuntimeError(
            "generated GAP8 build must contain exactly one complete flash image")
    clips = sorted((app_dir / "hex").glob("joint_gvsoc_clip_*.hex"))
    if len(clips) != 200 or [path.name for path in clips] != [
            f"joint_gvsoc_clip_{index:03d}.hex" for index in range(200)]:
        raise RuntimeError("flash accounting requires exactly clips 000..199")
    full_image_bytes = images[0].stat().st_size
    evaluation_corpus_bytes = sum(path.stat().st_size for path in clips)
    deployment_upper_bound = full_image_bytes - evaluation_corpus_bytes
    if evaluation_corpus_bytes <= 0 or deployment_upper_bound <= 0:
        raise RuntimeError("invalid complete-image/evaluation-corpus accounting")
    return {
        "complete_instrumented_flash_image_bytes": full_image_bytes,
        "gvsoc_evaluation_corpus_bytes": evaluation_corpus_bytes,
        "deployment_flash_upper_bound_bytes": deployment_upper_bound,
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--log", type=Path, required=True)
    parser.add_argument("--terminal-epsilon", type=float, required=True)
    parser.add_argument("--app-dir", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--memory-output", type=Path, required=True)
    args = parser.parse_args()
    rows = []
    for line in args.log.read_text(errors="replace").splitlines():
        match = re.fullmatch(r"JOINT_GVSOC (\d+) (\d+)((?: \d+){12})", line.strip())
        if match:
            rows.append((int(match.group(1)), int(match.group(2)),
                         [int(value) for value in match.group(3).split()]))
    if len(rows) != 200 or [row[0] for row in rows] != list(range(200)):
        raise RuntimeError("GVSOC log must contain exactly ordered JOINT_GVSOC rows 0..199")
    layer_rows = []
    for line in args.log.read_text(errors="replace").splitlines():
        match = re.fullmatch(r"JOINT_LAYER_CHECK (\d+) (\d+) (\d+)",
                             line.strip())
        if match:
            layer_rows.append(tuple(map(int, match.groups())))
    if ([row[0] for row in layer_rows] != list(range(6)) or
            any(expected != actual for _, expected, actual in layer_rows)):
        raise RuntimeError(
            "clip-zero generated-C layer checks must match all six DORY goldens: %r" %
            (layer_rows,))
    encoded = np.asarray([row[2] for row in rows], dtype=np.uint8)
    cycles = np.asarray([row[1] for row in rows], dtype=np.int64)
    np.savez_compressed(args.output, encoded=encoded, cycles=cycles,
                        terminal_epsilon=np.asarray(args.terminal_epsilon, dtype=np.float64))
    # GNU ld's --print-memory-usage lines are retained verbatim; a parser must
    # not pretend a source-level estimate is target memory evidence.
    log_lines = args.log.read_text(errors="replace").splitlines()
    memory_lines = [line for line in log_lines
                    if "Memory region" in line or re.match(r"^\s*(FLASH|RAM|L1|L2|L3)\b", line)]
    linker_used = {}
    for line in log_lines:
        match = re.match(r"^\s*(L1|L2):\s*(\d+)\s*([KMG]?B)", line)
        if match:
            multiplier = {"B": 1, "KB": 1024, "MB": 1024 * 1024, "GB": 1024 * 1024 * 1024}[match.group(3)]
            linker_used[match.group(1).lower() + "_bytes"] = int(match.group(2)) * multiplier
    if set(linker_used) != {"l1_bytes", "l2_bytes"}:
        raise RuntimeError("generated GAP8 build did not emit numeric L1/L2 memory lines")
    main_source = (args.app_dir / "src/gap8_main.c").read_text()
    network_source = (args.app_dir / "src/gap8_network.c").read_text()
    l2_workspace = re.search(r"#define\s+JOINT_L2_BYTES\s+(\d+)", main_source)
    output_bytes = re.search(r"#define\s+JOINT_OUTPUT_BYTES\s+(\d+)", main_source)
    l1_allocations = [int(value) for value in
                      re.findall(r"pmsis_l1_malloc\((\d+)\)", network_source)]
    if l2_workspace is None or output_bytes is None or not l1_allocations:
        raise RuntimeError("generated C lacks explicit dynamic L1/L2 allocation evidence")
    l1_dynamic = max(l1_allocations)
    l2_dynamic = int(l2_workspace.group(1)) + int(output_bytes.group(1))
    used = {
        "l1_static_bytes": linker_used["l1_bytes"],
        "l1_dynamic_bytes": l1_dynamic,
        "l1_bytes": linker_used["l1_bytes"] + l1_dynamic,
        "l2_static_bytes": linker_used["l2_bytes"],
        "l2_dynamic_bytes": l2_dynamic,
        "l2_bytes": linker_used["l2_bytes"] + l2_dynamic,
    }
    weight_files = sorted((args.app_dir / "hex").glob("gap8_*_weights.hex"))
    if not weight_files:
        raise RuntimeError("generated DORY app has no flash weight payload")
    used["model_weight_payload_bytes"] = int(
        sum(path.stat().st_size for path in weight_files))
    used.update(deployment_flash_accounting(args.app_dir))
    if used["deployment_flash_upper_bound_bytes"] < used["model_weight_payload_bytes"]:
        raise RuntimeError("deployment flash bound is smaller than its weight payload")
    args.memory_output.write_text(json.dumps(dict(source=str(args.log), memory_lines=memory_lines,
                                                   flash_definition="complete built flash-image bytes minus only the exact 200-clip raw GVSOC evaluation corpus; retains code, weights, filesystem overhead, alignment, boot payload, and generated default input",
                                                   **used), indent=2) + "\n")
    print("parsed 200 generated-C GVSOC outputs")


if __name__ == "__main__":
    main()
