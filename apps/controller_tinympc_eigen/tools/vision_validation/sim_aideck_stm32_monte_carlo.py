#!/usr/bin/env python3
"""Fast AI-deck/GAP8/UART/STM32 deployment Monte Carlo.

The expensive rendered-image suite calibrates the frontend behavior. This model
then sweeps geometry, tracking noise, calibration, state estimation, timing and
packet failures at scale while retaining the deployed 5 Hz/50 Hz rate mismatch.
"""

from __future__ import annotations

import argparse
import json
import math
import random
import struct
from pathlib import Path

from sim_flow_obstacle_sectors import FirmwareMirror, Sector, ray_box_range
from sim_gap8_flow_frontend import CX, FX, H, HOLD_CONF_DECAY, HOLD_UPDATES, SECTORS, SMOOTH_ALPHA, W


WIRE_TIME_S = 168 * 10 / 115200
Q_CENTERS = [(((i + 0.5) * W / SECTORS) - CX) / FX for i in range(SECTORS)]


def f32(x: float) -> float:
    return struct.unpack("<f", struct.pack("<f", x))[0]


def percentile(values: list[float], p: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    return ordered[min(len(ordered)-1, round((len(ordered)-1)*p))]


def trial(rng: random.Random, obstacle: bool, profile: str) -> dict[str, float | bool]:
    nominal = profile == "nominal"
    xmin = rng.uniform(0.7, 2.8)
    width = rng.uniform(0.15, 1.0)
    center_y = rng.uniform(-0.35, 0.35)
    box = (xmin, xmin+rng.uniform(0.12, 0.35), center_y-width/2, center_y+width/2)
    vy = rng.choice((-1.0, 1.0))*rng.uniform(0.07, 0.25)
    start_y = center_y - vy*1.5
    velocity_scale = 1.0 + rng.gauss(0.0, 0.01 if nominal else 0.03)
    velocity_bias = rng.gauss(0.0, 0.004 if nominal else 0.012)
    focal_scale = 1.0 + rng.gauss(0.0, 0.007 if nominal else 0.02)
    pixel_noise = rng.uniform(0.04, 0.12 if nominal else 0.22)
    packet_loss = rng.uniform(0.0, 0.02 if nominal else 0.08)
    crc_reject = rng.uniform(0.0, 0.001 if nominal else 0.01)
    latency = WIRE_TIME_S + rng.uniform(0.0, 0.012 if nominal else 0.035)
    yaw_rate = rng.uniform(-0.10 if nominal else -0.25, 0.10 if nominal else 0.25)
    gyro_bias = rng.gauss(0.0, 0.002 if nominal else 0.012)

    smooth_flow, smooth_conf, hold = [0.0]*SECTORS, [0.0]*SECTORS, [0]*SECTORS
    estimator = FirmwareMirror()
    latest: list[Sector] | None = None
    valid_ticks = 0
    final: dict[str, float | int | bool] | None = None
    def pose(at: float) -> tuple[float, float, float]:
        yaw = yaw_rate*at
        if abs(yaw_rate) < 1e-8:
            return 0.0, start_y+vy*at, yaw
        # Integrate body-frame velocity (0, vy) under constant yaw rate.
        return (vy*(math.cos(yaw)-1.0)/yaw_rate,
                start_y+vy*math.sin(yaw)/yaw_rate, yaw)

    for frame in range(1, 16):
        t = frame*0.2
        capture_x, capture_y, capture_yaw = pose(t)
        raw: list[tuple[float, float]] = []
        for i, q_reported in enumerate(Q_CENTERS):
            bearing = math.atan(q_reported/focal_scale)
            distance = ray_box_range(capture_x, capture_y, capture_yaw, bearing, box) if obstacle else None
            if distance is None:
                distance = (4.0-capture_x)/max(0.15, math.cos(capture_yaw+bearing))
                # Background still supplies tracks under balanced allocation.
                count = rng.randint(2, 5)
            else:
                count = rng.randint(2, 5)
            vel_eff = -vy*math.cos(bearing)
            angular_flow = yaw_rate + vel_eff/distance
            qdot = angular_flow*(1.0+(q_reported/focal_scale)**2)*focal_scale
            qdot += rng.gauss(0.0, pixel_noise/(FX*0.2*math.sqrt(count)))
            if rng.random() < 0.05:
                count = 0
            raw.append((qdot, min(1.0, count/32.0) if count >= 2 else 0.0))
        packet: list[Sector] = []
        for i, (flow, conf) in enumerate(raw):
            if conf > 0:
                if smooth_conf[i] <= 0:
                    smooth_flow[i], smooth_conf[i] = flow, conf
                else:
                    smooth_flow[i] += SMOOTH_ALPHA*(flow-smooth_flow[i])
                    smooth_conf[i] += SMOOTH_ALPHA*(conf-smooth_conf[i])
                hold[i] = HOLD_UPDATES
            elif hold[i] > 0:
                hold[i] -= 1; smooth_conf[i] *= HOLD_CONF_DECAY
            else:
                smooth_flow[i] = smooth_conf[i] = 0.0
            packet.append(Sector(f32(Q_CENTERS[i]), f32(smooth_flow[i]), f32(smooth_conf[i])))
        delivered = rng.random() >= packet_loss and rng.random() >= crc_reject
        if delivered:
            latest = packet
        if latest is None:
            continue
        for repeat in range(10):
            now = t + latency + repeat*0.02
            state_x, state_y, state_yaw = pose(now)
            state_x += rng.gauss(0.0, 0.003 if nominal else 0.006)
            state_y += rng.gauss(0.0, 0.003 if nominal else 0.006)
            measured_vy = vy*velocity_scale + velocity_bias + rng.gauss(0.0, 0.003 if nominal else 0.006)
            measured_yaw_rate = yaw_rate + gyro_bias + rng.gauss(0.0, 0.002 if nominal else 0.006)
            result = estimator.update(latest, 0.0, measured_vy, measured_yaw_rate,
                                      state_x, state_y, state_yaw,
                                      new_sample=(delivered and repeat == 0))
            if result["cylinder"]["valid"]:
                valid_ticks += 1
                final = result["cylinder"]
    detected = final is not None
    error = math.nan
    if obstacle and final:
        error = math.hypot(float(final["world_x"])-xmin,
                           float(final["world_y"])-center_y)
    return {"obstacle": obstacle, "detected": detected, "error": error,
            "range": xmin, "width": width, "speed": abs(vy),
            "valid_ticks": float(valid_ticks)}


def parse_args() -> argparse.Namespace:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--trials", type=int, default=10000)
    ap.add_argument("--seed", type=int, default=19)
    ap.add_argument("--profile", choices=("nominal", "stress"), default="nominal")
    ap.add_argument("--out", type=Path)
    return ap.parse_args()


def main() -> int:
    args = parse_args(); rng = random.Random(args.seed)
    rows = [trial(rng, obstacle=(i % 5 != 0), profile=args.profile) for i in range(args.trials)]
    positive = [r for r in rows if r["obstacle"]]
    negative = [r for r in rows if not r["obstacle"]]
    detected = [r for r in positive if r["detected"]]
    errors = [float(r["error"]) for r in detected]
    accurate = [r for r in detected if float(r["error"]) <= 0.35]
    summary = {
        "profile": args.profile, "trials": len(rows), "obstacle_trials": len(positive),
        "detection_rate": len(detected)/len(positive),
        "accurate_detection_rate_0p35m": len(accurate)/len(positive),
        "gross_error_rate_over_0p35m": (len(detected)-len(accurate))/len(positive),
        "false_positive_rate": sum(bool(r["detected"]) for r in negative)/len(negative),
        "error_m": {"median": percentile(errors, .5), "p90": percentile(errors, .9),
                    "p95": percentile(errors, .95), "p99": percentile(errors, .99)},
        "detection_by_range": {}, "detection_by_width": {},
        "assumptions": {"camera": "160x160 crop, 5 Hz flow snapshots",
                        "uart_baud": 115200, "message_bytes": 168,
                        "wire_time_ms": WIRE_TIME_S*1000,
                        "controller_hz": 50, "gap8_features": "5 per sector proposed"}
    }
    for label, lo, hi in (("0.7-1.2", .7, 1.2), ("1.2-2.0", 1.2, 2.0), ("2.0-2.8", 2.0, 2.81)):
        bucket = [r for r in positive if lo <= float(r["range"]) < hi]
        summary["detection_by_range"][label] = sum(bool(r["detected"]) for r in bucket)/len(bucket)
    for label, lo, hi in (("0.15-0.3", .15, .3), ("0.3-0.6", .3, .6), ("0.6-1.0", .6, 1.01)):
        bucket = [r for r in positive if lo <= float(r["width"]) < hi]
        summary["detection_by_width"][label] = sum(bool(r["detected"]) for r in bucket)/len(bucket)
    print(json.dumps(summary, indent=2))
    if args.out:
        args.out.mkdir(parents=True, exist_ok=True)
        (args.out/"summary.json").write_text(json.dumps(summary, indent=2)+"\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
