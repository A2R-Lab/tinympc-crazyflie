#!/usr/bin/env python3
"""Record native AI-deck camera crops while the disarmed drone is handheld.

This script only receives the NanoCockpit CPX image stream. It never connects
to CrazyRadio, arms the Crazyflie, or sends a flight setpoint.
"""

from __future__ import annotations

import argparse
import csv
import json
import sys
import time
from pathlib import Path

import numpy as np


APP_ROOT = Path(__file__).resolve().parents[2]
NANOCOCKPIT_ROOT = APP_ROOT.parents[2] / "tinympc-nanocockpit"
STREAMER_CLIENT = NANOCOCKPIT_ROOT / "src/client/aideck_cpx_streamer"
if str(STREAMER_CLIENT) not in sys.path:
    sys.path.insert(0, str(STREAMER_CLIENT))

try:
    from aideck_cpx_streamer.cpx import StreamerClient
except ImportError as exc:
    raise SystemExit(
        f"Could not import the NanoCockpit streamer from {STREAMER_CLIENT}. "
        "Keep tinympc-nanocockpit beside tinympc-crazyflie."
    ) from exc


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", type=Path, required=True,
                        help="new or empty capture directory")
    parser.add_argument("--host", default="192.168.4.1",
                        help="AI-deck address in AP mode")
    parser.add_argument("--port", type=int, default=5000)
    parser.add_argument("--duration-s", type=float, default=0.0,
                        help="stop after this many seconds; 0 waits for Ctrl-C")
    parser.add_argument("--frames", type=int, default=0,
                        help="stop after this many frames; 0 is unlimited")
    parser.add_argument("--no-udp-send", action="store_false", dest="udp_send",
                        help="do not send streamer acknowledgements")
    return parser.parse_args()


def field(obj, name, default=""):
    return getattr(obj, name, default) if obj is not None else default


def as_u8_gray(frame: np.ndarray) -> np.ndarray:
    array = np.asarray(frame)
    if array.ndim == 3:
        array = array[..., 0]
    if array.dtype == np.uint8:
        return np.ascontiguousarray(array)
    minimum = float(np.min(array))
    maximum = float(np.max(array))
    if maximum <= minimum:
        return np.zeros(array.shape, dtype=np.uint8)
    return np.ascontiguousarray(
        np.clip((array - minimum) * 255.0 / (maximum - minimum), 0, 255)
        .astype(np.uint8)
    )


def write_pgm(path: Path, image: np.ndarray) -> None:
    height, width = image.shape
    with path.open("wb") as stream:
        stream.write(f"P5\n{width} {height}\n255\n".encode("ascii"))
        stream.write(image.tobytes())


def main() -> None:
    args = parse_args()
    if args.out.exists() and any(args.out.iterdir()):
        raise SystemExit(f"Refusing to mix captures: {args.out} is not empty")
    frames_dir = args.out / "frames"
    frames_dir.mkdir(parents=True, exist_ok=True)

    fields = [
        "image", "capture_index", "host_monotonic_s", "host_unix_s",
        "frame_id", "frame_width", "frame_height", "frame_bpp",
        "frame_format", "frame_gap8_timestamp", "state_gap8_timestamp",
        "state_stm32_timestamp", "state_x_m", "state_y_m", "state_z_m",
        "state_vx_mps", "state_vy_mps", "state_vz_mps",
    ]
    client = StreamerClient(
        host=args.host, port=args.port, udp_send=args.udp_send
    )
    started = time.monotonic()
    count = 0
    print(
        f"Recording the disarmed handheld stream from "
        f"{args.host}:{args.port} into {args.out}"
    )
    print("Move toward, past, and away from the obstacle; press Ctrl-C to stop.")

    try:
        with (args.out / "frames.csv").open("w", newline="") as csv_stream:
            writer = csv.DictWriter(csv_stream, fieldnames=fields)
            writer.writeheader()
            for frame, _tof_frame, metadata in client.receive():
                client.send_reply(metadata, None)
                image = as_u8_gray(frame)
                count += 1
                name = f"frame_{count:06d}.pgm"
                write_pgm(frames_dir / name, image)
                state = field(metadata, "state", None)
                writer.writerow({
                    "image": f"frames/{name}",
                    "capture_index": count,
                    "host_monotonic_s": f"{time.monotonic() - started:.6f}",
                    "host_unix_s": f"{time.time():.6f}",
                    "frame_id": field(metadata, "frame_id"),
                    "frame_width": image.shape[1],
                    "frame_height": image.shape[0],
                    "frame_bpp": field(metadata, "frame_bpp"),
                    "frame_format": field(metadata, "frame_format"),
                    "frame_gap8_timestamp": field(metadata, "frame_timestamp"),
                    "state_gap8_timestamp": field(metadata, "state_timestamp"),
                    "state_stm32_timestamp": field(state, "timestamp"),
                    "state_x_m": field(state, "x", 0) / 1000.0,
                    "state_y_m": field(state, "y", 0) / 1000.0,
                    "state_z_m": field(state, "z", 0) / 1000.0,
                    "state_vx_mps": field(state, "vx", 0) / 1000.0,
                    "state_vy_mps": field(state, "vy", 0) / 1000.0,
                    "state_vz_mps": field(state, "vz", 0) / 1000.0,
                })
                csv_stream.flush()
                if count == 1 or count % 30 == 0:
                    elapsed = max(time.monotonic() - started, 1.0e-6)
                    print(
                        f"  saved {count} frames "
                        f"({image.shape[1]}x{image.shape[0]}, "
                        f"{count / elapsed:.1f} fps)",
                        flush=True,
                    )
                if args.frames and count >= args.frames:
                    break
                if args.duration_s > 0 and time.monotonic() - started >= args.duration_s:
                    break
    except KeyboardInterrupt:
        print("\nCapture stopped.")
    finally:
        client.shutdown()

    elapsed = time.monotonic() - started
    manifest = {
        "format": "tinympc-handheld-danger-capture-v1",
        "frames": count,
        "duration_s": elapsed,
        "host": args.host,
        "port": args.port,
        "flight_commands_sent": False,
        "image_note": "Native grayscale crop received from NanoCockpit CPX streamer.",
    }
    (args.out / "capture.json").write_text(
        json.dumps(manifest, indent=2) + "\n"
    )
    print(f"Saved {count} frames in {elapsed:.1f} s to {args.out}")


if __name__ == "__main__":
    main()
