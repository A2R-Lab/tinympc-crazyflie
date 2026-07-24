#!/usr/bin/env python3
"""Capture AI-deck streamed images during a PID peering/approach motion.

This is for perception debugging, not avoidance. It connects to the AI-deck
NanoCockpit CPX image streamer over WiFi, saves grayscale frames, and commands a
bounded Crazyflie PID motion profile over CrazyRadio:

  takeoff -> settle -> lateral peering -> slow forward approach + peering -> land

The output directory contains:
  frames/frame_000001.png ...
  frames.csv              per-image host/GAP/state/command metadata
  state_log.csv           CrazyRadio state/voltage log snapshots
  commands.csv            commanded setpoints at 50 Hz
"""

from __future__ import annotations

import argparse
import csv
import os
from pathlib import Path
import sys
import threading
import time

import cv2
import numpy as np

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.utils import uri_helper


APP_ROOT = Path(__file__).resolve().parent
NANOCOCKPIT_ROOT = APP_ROOT.parents[2] / "tinympc-nanocockpit"
STREAMER_CLIENT = NANOCOCKPIT_ROOT / "src/client/aideck_cpx_streamer"
if str(STREAMER_CLIENT) not in sys.path:
    sys.path.insert(0, str(STREAMER_CLIENT))

from aideck_cpx_streamer.cpx import StreamerClient  # noqa: E402


DEFAULT_URI = "radio://0/80/2M/E7E7E7E7E8"

LOG_BLOCKS = [
    ("state_pos", 50, [
        ("stateEstimate.x", "float"),
        ("stateEstimate.y", "float"),
        ("stateEstimate.z", "float"),
        ("stateEstimate.yaw", "float"),
    ]),
    ("state_vel", 50, [
        ("stateEstimate.vx", "float"),
        ("stateEstimate.vy", "float"),
        ("stateEstimate.vz", "float"),
    ]),
    ("pm", 100, [("pm.vbat", "float")]),
]


def parse_args():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--uri", default=uri_helper.uri_from_env(default=DEFAULT_URI))
    p.add_argument("--out", type=Path, default=Path("aideck_peer_approach_capture"))
    p.add_argument("--host", default="192.168.4.1", help="AI-deck streamer IP")
    p.add_argument("--port", type=int, default=5000, help="AI-deck streamer port")
    p.add_argument("--no-udp-send", action="store_false", dest="udp_send",
                   help="do not send CPX streamer replies")
    p.add_argument("--stream-timeout-s", type=float, default=8.0,
                   help="wait this long for the first image before arming")
    p.add_argument("--height", type=float, default=0.5)
    p.add_argument("--takeoff-s", type=float, default=2.0)
    p.add_argument("--settle-s", type=float, default=1.0)
    p.add_argument("--peer-s", type=float, default=10.0)
    p.add_argument("--peer-amp", type=float, default=0.25)
    p.add_argument("--peer-period-s", type=float, default=3.0)
    p.add_argument("--approach-s", type=float, default=6.0)
    p.add_argument("--approach-x", type=float, default=0.4)
    p.add_argument("--approach-hold-s", type=float, default=4.0)
    p.add_argument("--approach-peer-amp", type=float, default=0.08)
    p.add_argument("--start-x", type=float, default=0.0)
    p.add_argument("--start-y", type=float, default=0.0)
    p.add_argument("--yaw-deg", type=float, default=0.0)
    p.add_argument("--land-s", type=float, default=2.0)
    p.add_argument("--min-vbat", type=float, default=3.55)
    p.add_argument("--no-reset-estimator", action="store_true")
    p.add_argument("--cache-dir", default=".cf_cache")
    return p.parse_args()


class CaptureState:
    def __init__(self):
        self.lock = threading.Lock()
        self.phase = "init"
        self.cmd_x = 0.0
        self.cmd_y = 0.0
        self.cmd_z = 0.0
        self.yaw_deg = 0.0
        self.latest = {}

    def set_command(self, phase, x, y, z, yaw_deg):
        with self.lock:
            self.phase = phase
            self.cmd_x = float(x)
            self.cmd_y = float(y)
            self.cmd_z = float(z)
            self.yaw_deg = float(yaw_deg)

    def update_log(self, data):
        with self.lock:
            self.latest.update(data)

    def snapshot(self):
        with self.lock:
            return {
                "phase": self.phase,
                "cmd_x": self.cmd_x,
                "cmd_y": self.cmd_y,
                "cmd_z": self.cmd_z,
                "cmd_yaw_deg": self.yaw_deg,
                **self.latest,
            }


def set_param(cf, name, value, delay=0.03):
    cf.param.set_value(name, str(value))
    time.sleep(delay)


def as_u8_image(frame):
    if frame.dtype == np.uint8:
        return frame
    return cv2.normalize(frame, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)


def streamer_thread(args, cap, stop_event, first_frame_event, streamer_errors, client):
    frames_dir = args.out / "frames"
    frames_dir.mkdir(parents=True, exist_ok=True)
    csv_path = args.out / "frames.csv"
    count = 0
    try:
        with csv_path.open("w", newline="") as fp:
            fieldnames = [
                "image", "host_time", "phase", "cmd_x", "cmd_y", "cmd_z",
                "cmd_yaw_deg", "frame_id", "frame_width", "frame_height",
                "frame_bpp", "frame_format", "frame_gap8_timestamp",
                "state_gap8_timestamp", "state_stm32_timestamp",
                "meta_x_m", "meta_y_m", "meta_z_m",
                "meta_vx_mps", "meta_vy_mps", "meta_vz_mps",
                "meta_quat", "meta_rate_roll_rps", "meta_rate_pitch_rps",
                "meta_rate_yaw_rps", "log_x", "log_y", "log_z", "log_yaw_deg",
                "log_vx", "log_vy", "log_vz", "vbat",
            ]
            writer = csv.DictWriter(fp, fieldnames=fieldnames)
            writer.writeheader()
            for frame, _tof_frame, metadata in client.receive():
                if stop_event.is_set():
                    break
                client.send_reply(metadata, None)
                count += 1
                image_name = f"frame_{count:06d}.png"
                cv2.imwrite(str(frames_dir / image_name), as_u8_image(frame))
                snap = cap.snapshot()
                writer.writerow({
                    "image": f"frames/{image_name}",
                    "host_time": f"{time.time():.6f}",
                    "phase": snap.get("phase"),
                    "cmd_x": snap.get("cmd_x"),
                    "cmd_y": snap.get("cmd_y"),
                    "cmd_z": snap.get("cmd_z"),
                    "cmd_yaw_deg": snap.get("cmd_yaw_deg"),
                    "frame_id": metadata.frame_id,
                    "frame_width": metadata.frame_width,
                    "frame_height": metadata.frame_height,
                    "frame_bpp": metadata.frame_bpp,
                    "frame_format": metadata.frame_format,
                    "frame_gap8_timestamp": metadata.frame_timestamp,
                    "state_gap8_timestamp": metadata.state_timestamp,
                    "state_stm32_timestamp": metadata.state.timestamp,
                    "meta_x_m": metadata.state.x / 1000.0,
                    "meta_y_m": metadata.state.y / 1000.0,
                    "meta_z_m": metadata.state.z / 1000.0,
                    "meta_vx_mps": metadata.state.vx / 1000.0,
                    "meta_vy_mps": metadata.state.vy / 1000.0,
                    "meta_vz_mps": metadata.state.vz / 1000.0,
                    "meta_quat": metadata.state.quat,
                    "meta_rate_roll_rps": metadata.state.rateRoll / 1000.0,
                    "meta_rate_pitch_rps": metadata.state.ratePitch / 1000.0,
                    "meta_rate_yaw_rps": metadata.state.rateYaw / 1000.0,
                    "log_x": snap.get("stateEstimate.x"),
                    "log_y": snap.get("stateEstimate.y"),
                    "log_z": snap.get("stateEstimate.z"),
                    "log_yaw_deg": snap.get("stateEstimate.yaw"),
                    "log_vx": snap.get("stateEstimate.vx"),
                    "log_vy": snap.get("stateEstimate.vy"),
                    "log_vz": snap.get("stateEstimate.vz"),
                    "vbat": snap.get("pm.vbat"),
                })
                fp.flush()
                first_frame_event.set()
    except Exception as exc:  # noqa: BLE001
        if not stop_event.is_set():
            streamer_errors.append(str(exc))
            first_frame_event.set()
            print(f"streamer stopped: {exc}")
    finally:
        if count == 0 and not stop_event.is_set() and not streamer_errors:
            streamer_errors.append("stream ended before any frame was received")
            first_frame_event.set()
        print(f"streamer saved {count} frames")


def command_position(cf, cap, commands_writer, phase, x, y, z, yaw_deg):
    cap.set_command(phase, x, y, z, yaw_deg)
    cf.commander.send_position_setpoint(x, y, z, yaw_deg)
    commands_writer.writerow({
        "host_time": f"{time.time():.6f}",
        "phase": phase,
        "cmd_x": x,
        "cmd_y": y,
        "cmd_z": z,
        "cmd_yaw_deg": yaw_deg,
    })


def stream_position(cf, cap, writer, seconds, x, y, z, yaw_deg, phase):
    deadline = time.time() + seconds
    while time.time() < deadline:
        command_position(cf, cap, writer, phase, x, y, z, yaw_deg)
        time.sleep(0.02)


def stream_line(cf, cap, writer, seconds, x0, y0, z0, x1, y1, z1, yaw_deg, phase):
    start = time.time()
    while True:
        t = time.time() - start
        if t >= seconds:
            break
        a = min(1.0, t / max(1e-6, seconds))
        x = x0 + a * (x1 - x0)
        y = y0 + a * (y1 - y0)
        z = z0 + a * (z1 - z0)
        command_position(cf, cap, writer, phase, x, y, z, yaw_deg)
        time.sleep(0.02)


def stream_peer(cf, cap, writer, seconds, x, y0, z, yaw_deg, amp, period_s, phase):
    start = time.time()
    while True:
        t = time.time() - start
        if t >= seconds:
            break
        y = y0 + amp * np.sin(2.0 * np.pi * t / max(1e-6, period_s))
        command_position(cf, cap, writer, phase, x, y, z, yaw_deg)
        time.sleep(0.02)


def log_writer(path):
    fp = path.open("w", newline="")
    writer = csv.DictWriter(fp, fieldnames=[
        "host_time", "stateEstimate.x", "stateEstimate.y", "stateEstimate.z",
        "stateEstimate.yaw", "stateEstimate.vx", "stateEstimate.vy",
        "stateEstimate.vz", "pm.vbat",
    ])
    writer.writeheader()
    return fp, writer


def main():
    args = parse_args()
    args.out.mkdir(parents=True, exist_ok=True)
    Path(args.cache_dir).mkdir(parents=True, exist_ok=True)
    cap = CaptureState()
    stop_event = threading.Event()
    first_frame_event = threading.Event()
    streamer_errors = []
    client = StreamerClient(host=args.host, port=args.port,
                            udp_send=args.udp_send, log_fn=lambda *a, **k: None)

    thread = threading.Thread(
        target=streamer_thread,
        args=(args, cap, stop_event, first_frame_event, streamer_errors, client),
        daemon=True,
    )
    thread.start()
    print(f"waiting for AI-deck image stream at {args.host}:{args.port}")
    got_stream_event = first_frame_event.wait(args.stream_timeout_s)
    if (not got_stream_event) or streamer_errors:
        stop_event.set()
        client.shutdown()
        if streamer_errors:
            raise SystemExit(f"ABORT: image streamer failed before arming: {streamer_errors[-1]}")
        raise SystemExit("ABORT: no image stream before arming")

    state_fp, state_writer = log_writer(args.out / "state_log.csv")
    commands_fp = (args.out / "commands.csv").open("w", newline="")
    commands_writer = csv.DictWriter(commands_fp, fieldnames=[
        "host_time", "phase", "cmd_x", "cmd_y", "cmd_z", "cmd_yaw_deg",
    ])
    commands_writer.writeheader()
    configs = []

    cflib.crtp.init_drivers()
    try:
        with SyncCrazyflie(args.uri, cf=Crazyflie(rw_cache=args.cache_dir)) as scf:
            cf = scf.cf

            def on_log(_timestamp, data, _logconf):
                cap.update_log(data)
                row = {"host_time": f"{time.time():.6f}"}
                row.update(data)
                state_writer.writerow(row)
                state_fp.flush()

            for name, period_ms, variables in LOG_BLOCKS:
                lc = LogConfig(name=name, period_in_ms=period_ms)
                for var, typ in variables:
                    lc.add_variable(var, typ)
                cf.log.add_config(lc)
                lc.data_received_cb.add_callback(on_log)
                lc.start()
                configs.append(lc)

            time.sleep(1.0)
            vbat = cap.snapshot().get("pm.vbat")
            if vbat is not None and vbat < args.min_vbat:
                raise SystemExit(f"ABORT: battery {vbat:.2f} V < {args.min_vbat:.2f} V")

            print(f"connected {args.uri}; capturing to {args.out}")
            if not args.no_reset_estimator:
                set_param(cf, "kalman.resetEstimation", 1)
                time.sleep(0.1)
                set_param(cf, "kalman.resetEstimation", 0)
                time.sleep(5.0)

            set_param(cf, "commander.enHighLevel", 0)
            set_param(cf, "stabilizer.controller", 1)
            cf.platform.send_arming_request(True)
            time.sleep(0.5)

            print("takeoff")
            stream_line(cf, cap, commands_writer, args.takeoff_s,
                        args.start_x, args.start_y, 0.05,
                        args.start_x, args.start_y, args.height,
                        args.yaw_deg, "takeoff")
            stream_position(cf, cap, commands_writer, args.settle_s,
                            args.start_x, args.start_y, args.height,
                            args.yaw_deg, "settle")
            print("peer")
            stream_peer(cf, cap, commands_writer, args.peer_s,
                        args.start_x, args.start_y, args.height,
                        args.yaw_deg, args.peer_amp, args.peer_period_s, "peer")
            print("approach")
            stream_line(cf, cap, commands_writer, args.approach_s,
                        args.start_x, args.start_y, args.height,
                        args.approach_x, args.start_y, args.height,
                        args.yaw_deg, "approach")
            if args.approach_hold_s > 0.0:
                stream_peer(cf, cap, commands_writer, args.approach_hold_s,
                            args.approach_x, args.start_y, args.height,
                            args.yaw_deg, args.approach_peer_amp,
                            args.peer_period_s, "approach_hold")
            print("land")
            snap = cap.snapshot()
            x = snap.get("stateEstimate.x", args.approach_x) or args.approach_x
            y = snap.get("stateEstimate.y", args.start_y) or args.start_y
            z = snap.get("stateEstimate.z", args.height) or args.height
            yaw = snap.get("stateEstimate.yaw", args.yaw_deg) or args.yaw_deg
            stream_line(cf, cap, commands_writer, args.land_s,
                        x, y, z, x, y, 0.05, yaw, "land")
            cf.commander.send_stop_setpoint()
            cf.platform.send_arming_request(False)
    except KeyboardInterrupt:
        print("abort requested")
    finally:
        stop_event.set()
        client.shutdown()
        for lc in configs:
            try:
                lc.stop()
            except Exception:
                pass
        state_fp.close()
        commands_fp.close()
        thread.join(timeout=3.0)


if __name__ == "__main__":
    main()
