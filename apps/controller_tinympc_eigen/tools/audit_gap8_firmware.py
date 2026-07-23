#!/usr/bin/env python3
"""Static GAP8 memory, UART, and alternating-schedule deployment audit."""

from __future__ import annotations

import argparse
import json
import re
from pathlib import Path


GAP8_L2_BYTES = 512 * 1024
UART_BAUD = 115200
GATE_PACKET_BYTES = 44
FLOW_PACKET_BYTES = 168


def define(text: str, name: str) -> int:
    match = re.search(rf"^\s*#define\s+{re.escape(name)}\s+\(?(\d+)", text, re.M)
    if not match:
        raise ValueError(f"missing integer define {name}")
    return int(match.group(1))


def int_array(text: str, name: str) -> list[int]:
    match = re.search(rf"{re.escape(name)}\s*\[[^\]]*\]\s*=\s*\{{([^}}]+)\}}", text)
    if not match:
        raise ValueError(f"missing array {name}")
    return [int(v) for v in re.findall(r"\d+", match.group(1))]


def directional_peak(activations: list[int], outputs: list[int],
                     weights: list[int]) -> int:
    """Replay DORY's two-ended allocator for this branch-free network."""
    begin = end = peak = 0
    direction = True
    for i, (activation, output, weight) in enumerate(
            zip(activations, outputs, weights)):
        if direction:
            end += output
        else:
            begin += output
        if i == 0:
            if direction:
                begin += activation
            else:
                end += activation
        if direction:
            begin += weight
        else:
            end += weight
        peak = max(peak, begin + end)
        if direction:
            begin -= weight + activation
        else:
            end -= weight + activation
        direction = not direction
    return peak


def audit(root: Path, cnn_ms: float, flow_ms: float,
          linked_l2_bytes: int = 143368) -> dict[str, object]:
    app = root / "src/gap/examples/pulp-frontnet"
    main = (app / "main.c").read_text()
    config = (app / "config.h").read_text()
    network = (app / "app/networks/gate8-async/inc/network.h").read_text()
    camera_source = (root / "src/gap/lib/camera.c").read_text()
    himax_source = (root / "src/gap/lib/camera/himax.c").read_text()

    arena = define(main, "L2_BUF_SIZE")
    features = define(main, "FLOW_MAX_FEATURES")
    sectors = 9
    image_w = define(main, "IMG_W")
    image_h = define(main, "IMG_H_CAM")
    camera_buffers = define(config, "CAMERA_BUFFERS")
    # HIMAX half mode is 162x162; CPI captures all but the configured one-row
    # bottom crop. Top/left/right cropping happens in-place afterward.
    camera_capture_bytes = camera_buffers * 162 * 161
    flow_bytes = (2*image_w*image_h + 2*(image_w//2)*(image_h//2) +
                  features*12 + 2*168)
    peak = directional_peak(int_array(network, "activations_size"),
                            int_array(network, "activations_out_size"),
                            int_array(network, "weights_size"))
    # The linker-reported L2 already contains the static flow image/feature
    # buffers. Only the DORY arena and dynamically allocated camera buffers
    # are additive at runtime.
    reserved = linked_l2_bytes + arena + camera_capture_bytes
    headroom = GAP8_L2_BYTES - reserved

    gate_hz = flow_hz = 15
    uart_fraction = ((GATE_PACKET_BYTES*gate_hz + FLOW_PACKET_BYTES*flow_hz) *
                     10 / UART_BAUD)
    slot_ms = 1000/15
    checks = {
        "combined_mode_enabled": "#define FLOW_OBSTACLE_ENABLE" in config,
        "camera_test_disabled": not re.search(
            r"^\s*#define\s+FLOW_OBSTACLE_CAMERA_TEST", config, re.M),
        "uart_test_disabled": not re.search(
            r"^\s*#define\s+FLOW_OBSTACLE_TEST_UART", config, re.M),
        "uart_only_test_disabled": not re.search(
            r"^\s*#define\s+FLOW_OBSTACLE_TEST_ONLY", config, re.M),
        "binary_uart_debug_disabled":
            bool(re.search(r"^\s*#define\s+GATE8_DEBUG_PRINT\s+0", main, re.M)),
        "network_arena_fits": arena >= peak,
        "l2_headroom_at_least_128k": headroom >= 128*1024,
        "uart_below_50_percent": uart_fraction < 0.50,
        "cnn_fits_alternating_slot": cnn_ms < slot_ms,
        "flow_fits_alternating_slot": flow_ms < slot_ms,
        "single_uart_owner": "static void vision_uart_service" in main,
        "uart_dma_uses_stable_gate_copy":
            "uart_write_async(&uart, &gate8_uart_msg" in main,
        "flow_packets_have_sequence":
            "flow_tx_payload.reserved = flow_wire_seq++" in main,
        "camera_watchdog_enabled": "camera_watchdog_poll(&camera)" in main,
        "camera_consumers_serialized":
            "(consume_idx - 1) % CAMERA_BUFFERS" in camera_source,
        "camera_recovery_is_capture_stage_only":
            "camera->stage != CAMERA_STAGE_WAIT_CAPTURE" in camera_source,
        "himax_model_id_checked": "model_id != 0x01B0u" in himax_source,
        "himax_i2c_failures_counted": "s_i2c_error_count++" in himax_source,
        "software_frame_alternation":
            "camera_frame->sequence_id & 1u" in main,
        "overrun_drops_cnn_not_flow": "cnn_frame_dropped++" in main,
    }
    return {
        "network": {"directional_peak_bytes": peak, "arena_bytes": arena,
                    "arena_headroom_bytes": arena-peak},
        "l2": {"capacity_bytes": GAP8_L2_BYTES,
               "linked_static_bytes": linked_l2_bytes,
               "network_arena_bytes": arena,
               "camera_buffers_bytes": camera_capture_bytes,
               "flow_buffers_in_linked_static_bytes": flow_bytes,
               "estimated_runtime_peak_bytes": reserved,
               "unallocated_headroom_bytes": headroom},
        "schedule": {"cnn_hz": gate_hz, "flow_hz": flow_hz,
                     "slot_ms": slot_ms, "assumed_cnn_ms": cnn_ms,
                     "assumed_flow_ms": flow_ms},
        "uart": {"baud": UART_BAUD, "utilization": uart_fraction,
                 "serialization_ms_per_second": uart_fraction*1000},
        "checks": checks,
        "pass": all(checks.values()),
    }


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("nanocockpit", type=Path)
    ap.add_argument("--cnn-ms", type=float, default=50.0,
                    help="conservative measured/assumed CNN duration")
    ap.add_argument("--flow-ms", type=float, default=50.0,
                    help="conservative measured/assumed FC flow duration")
    ap.add_argument("--linked-l2", type=int, default=143368,
                    help="L2 bytes reported by the successful GAP8 link")
    ap.add_argument("--out", type=Path)
    args = ap.parse_args()
    result = audit(args.nanocockpit, args.cnn_ms, args.flow_ms,
                   args.linked_l2)
    text = json.dumps(result, indent=2)
    print(text)
    if args.out:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        args.out.write_text(text + "\n")
    return 0 if result["pass"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
