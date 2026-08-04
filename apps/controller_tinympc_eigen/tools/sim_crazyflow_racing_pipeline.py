#!/usr/bin/env python3
"""Closed-loop Crazyflow + HM01B0/GAP8/UART/STM32 obstacle simulation.

Crazyflow supplies 500 Hz first-principles vehicle dynamics.  Camera images
are generated at the AI-deck's 160x160 crop and passed through the
source-matched sector frontend.  Only delayed sector packets and simulated
state estimates reach the firmware mirror and avoidance policy.
"""

from __future__ import annotations

import argparse
import contextlib
import io
import json
import math
import time
from collections import deque
from pathlib import Path

import numpy as np
with contextlib.redirect_stdout(io.StringIO()):
    from crazyflow.sim import Sim

from sim_flow_obstacle_sectors import FirmwareMirror, Sector
from sim_gap8_deployment_suite import render_gap8
from sim_gap8_flow_frontend import Gap8FlowFrontend
from sim_monocular_flow_suite import Box, Scene


HERE = Path(__file__).resolve().parent
TARGET = Box("target", 3.0, 3.25, -0.45, 0.45, 1)
CAMERA_SCENE = Scene("crazyflow_box", (TARGET,), 0.0, 0.0, (3.0, 0.0))
UART_SECONDS = 168 * 10 / 115200
CAMERA_HZ = 30
CONTROL_HZ = 100


def yaw_from_xyzw(q: np.ndarray) -> float:
    x, y, z, w = q
    return math.atan2(2 * (w*z + x*y), 1 - 2 * (y*y + z*z))


def smoothstep5(u: float) -> tuple[float, float, float]:
    """Quintic position, first derivative, and second derivative on [0, 1]."""
    u = max(0.0, min(1.0, u))
    return (10*u**3 - 15*u**4 + 6*u**5,
            30*u**2 - 60*u**3 + 30*u**4,
            60*u - 180*u**2 + 120*u**3)


def speed_ramp(t: float, speed: float, ramp_s: float = 0.75) -> tuple[float, float, float]:
    """Position, velocity and acceleration for a bounded launch from rest."""
    if t < ramp_s:
        acc = speed/ramp_s
        return 0.5*acc*t*t, acc*t, acc
    return speed*(t-0.5*ramp_s), speed, 0.0


def run(speed: float, improved: bool, seed: int, packet_loss: float,
        image_noise: float, flow_hz: int | None = None) -> dict[str, object]:
    rng = np.random.default_rng(seed)
    sim = Sim(control="state", dynamics="first_principles", freq=500,
              state_freq=100, attitude_freq=500, force_torque_freq=500,
              xml_path=HERE / "crazyflow_racing_scene.xml", rng_key=seed)
    sim.reset()
    sim.data = sim.data.replace(states=sim.data.states.replace(
        pos=sim.data.states.pos.at[0, 0].set(np.array([0.0, 0.0, 0.5])),
        rotor_vel=sim.data.states.rotor_vel.at[0, 0].set(np.ones(4) * 20000)))

    gap8 = Gap8FlowFrontend(balanced_features=improved,
                            near_aggregation=improved)
    stm32 = FirmwareMirror()
    queue: deque[tuple[float, list[Sector]]] = deque()
    latest: list[Sector] | None = None
    first_detection = None
    first_cluster = None
    avoidance_t = None
    avoiding = False
    avoidance_complete = False
    avoid_side = 0.0
    avoid_start_pos = np.zeros(3)
    avoid_start_vel = np.zeros(3)
    resume_t = None
    resume_x = 0.0
    packets_sent = packets_delivered = 0
    min_clearance = 99.0
    collision = False
    ground_contact = False
    min_altitude = 99.0
    camera_stride = (flow_hz or CAMERA_HZ) if improved else 5
    camera_period = 1.0 / camera_stride
    next_camera = 0.0
    wall_start = time.perf_counter()
    max_time = 6.0
    steps = int(max_time * CONTROL_HZ)

    for tick in range(steps):
        t = tick / CONTROL_HZ
        pos = np.asarray(sim.data.states.pos[0, 0], dtype=float)
        vel = np.asarray(sim.data.states.vel[0, 0], dtype=float)
        quat = np.asarray(sim.data.states.quat[0, 0], dtype=float)
        yaw = yaw_from_xyzw(quat)

        if t + 1e-9 >= next_camera:
            frame = render_gap8(CAMERA_SCENE, pos[0], pos[1], yaw)
            if image_noise:
                a = np.asarray(frame, dtype=np.float32)
                a += rng.normal(0.0, image_noise, a.shape)
                frame = np.clip(np.rint(a), 0, 255).astype(np.uint8).tolist()
            sectors, _ = gap8.process(frame, camera_period)
            next_camera += camera_period
            if sectors is not None and rng.random() >= packet_loss:
                if not improved:
                    sectors = [Sector(s.azimuth, s.flow_x, s.confidence, flow_y=0.0)
                               for s in sectors]
                queue.append((t + UART_SECONDS, sectors))
                packets_sent += 1

        new_sample = False
        while queue and queue[0][0] <= t:
            _, latest = queue.popleft()
            packets_delivered += 1
            new_sample = True
        if latest is not None:
            # Match controller_tinympc.cpp: rotate EKF world velocity with the
            # full measured R(q)^T, including racing-flight roll and pitch.
            qx, qy, qz, qw = quat
            body_vx = ((1-2*(qy*qy+qz*qz))*vel[0] +
                       2*(qx*qy+qw*qz)*vel[1] +
                       2*(qx*qz-qw*qy)*vel[2])
            body_vy = (2*(qx*qy-qw*qz)*vel[0] +
                       (1-2*(qx*qx+qz*qz))*vel[1] +
                       2*(qy*qz+qw*qx)*vel[2])
            result = stm32.update(latest, body_vx, body_vy,
                                  float(sim.data.states.ang_vel[0, 0, 2]),
                                  pos[0], pos[1], yaw, new_sample=new_sample)
            cyl = result["cylinder"]
            cluster = result["cluster"]
            if cluster["fresh"] and first_cluster is None:
                first_cluster = {
                    "time_s": t, "remaining_m": 3.0-pos[0],
                    "range_m": float(cluster["range"]),
                    "sector_count": int(cluster["count"]),
                }
            if cyl["valid"] and first_detection is None:
                first_detection = {
                    "time_s": t,
                    "remaining_m": 3.0-pos[0],
                    "estimate_xy_m": [float(cyl["world_x"]), float(cyl["world_y"])],
                    "error_m": math.hypot(float(cyl["world_x"])-3.0,
                                          float(cyl["world_y"])),
                }

        # Perception-only avoidance: command a one-metre lateral offset when
        # the persistent cylinder lies in the forward racing corridor.
        lateral_goal = 0.0
        if latest is not None and stm32.cylinder["valid"]:
            bx = float(stm32.cylinder["body_x"])
            by = float(stm32.cylinder["body_y"])
            if 0.15 < bx < 2.4 and abs(by) < 0.75 and not avoidance_complete:
                if not avoiding:
                    avoiding = True
                    avoid_side = -1.0 if by >= 0 else 1.0
                    avoid_start_pos = pos.copy()
                    avoid_start_vel = vel.copy()
                if avoidance_t is None:
                    avoidance_t = t

        cmd = np.zeros((1, 1, 13))
        if avoiding:
            tau = t - float(avoidance_t)
            lateral_time = 1.00
            lateral_distance = 0.95
            entry_vx = max(0.0, avoid_start_vel[0])
            decel = 8.0
            stop_time = entry_vx/decel
            # Brake first, then sidestep. Combining peak braking and lateral
            # acceleration exceeded the Crazyflie's thrust/tilt envelope.
            lateral_tau = max(0.0, tau-stop_time-0.12)
            p, dp, ddp = smoothstep5(lateral_tau/lateral_time)
            lateral_goal = avoid_start_pos[1] + avoid_side*lateral_distance*p
            lateral_vel = avoid_side*lateral_distance*dp/lateral_time
            lateral_acc = avoid_side*lateral_distance*ddp/(lateral_time*lateral_time)

            # A consistent braking trajectory avoids the contradictory
            # position/velocity command that previously saturated attitude
            # and collective thrust.
            brake_tau = min(tau, stop_time)
            x_goal = avoid_start_pos[0] + entry_vx*brake_tau - 0.5*decel*brake_tau**2
            x_vel = max(0.0, entry_vx-decel*tau)
            x_acc = -decel if tau < stop_time else 0.0
            cmd[0, 0, :3] = [x_goal, lateral_goal, 0.5]
            cmd[0, 0, 3:6] = [x_vel, lateral_vel, 0.0]
            cmd[0, 0, 6:9] = [x_acc, lateral_acc, 0.0]
            if lateral_tau >= lateral_time and abs(pos[1]) > 0.62 and abs(vel[1]) < 0.45:
                avoiding = False
                avoidance_complete = True
                resume_t = t
                resume_x = pos[0]
        else:
            cruise_y = avoid_side*0.95 if avoidance_complete else 0.0
            if resume_t is None:
                x_goal, x_vel, x_acc = speed_ramp(t, speed)
            else:
                dx, x_vel, x_acc = speed_ramp(t-resume_t, speed)
                x_goal = resume_x + dx
            cmd[0, 0, :3] = [min(x_goal, 4.2), cruise_y, 0.5]
            cmd[0, 0, 3:6] = [x_vel, 0.0, 0.0]
            cmd[0, 0, 6:9] = [x_acc, 0.0, 0.0]
        sim.state_control(cmd)
        sim.step(sim.freq // CONTROL_HZ)

        pos2 = np.asarray(sim.data.states.pos[0, 0], dtype=float)
        min_altitude = min(min_altitude, float(pos2[2]))
        ground_contact = bool(ground_contact or pos2[2] < 0.12)
        if 2.80 <= pos2[0] <= 3.45:
            clearance = abs(pos2[1]) - 0.45 - 0.08
            min_clearance = min(min_clearance, clearance)
            if clearance < 0 and pos2[0] >= 2.92:
                collision = True
        if pos2[0] > 3.55:
            break

    sim.close()
    elapsed = time.perf_counter() - wall_start
    final = np.asarray(sim.data.states.pos[0, 0], dtype=float)
    return {
        "speed_m_s": speed,
        "pipeline": (f"{camera_stride}hz_hybrid" if improved else "deployed_5hz"),
        "seed": seed, "packet_loss": packet_loss, "image_noise_sigma": image_noise,
        "first_cluster": first_cluster, "first_detection": first_detection,
        "avoidance_started_s": avoidance_t,
        "collision": collision, "ground_contact": ground_contact,
        "safe_pass": bool(not collision and not ground_contact and final[0] > 3.45),
        "minimum_altitude_m": min_altitude,
        "minimum_clearance_m": None if min_clearance == 99 else min_clearance,
        "final_position_m": final.tolist(), "packets_sent": packets_sent,
        "packets_delivered": packets_delivered, "simulated_seconds": tick/CONTROL_HZ,
        "wall_seconds": elapsed, "realtime_factor": (tick/CONTROL_HZ)/elapsed,
    }


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--trials", type=int, default=2)
    ap.add_argument("--packet-loss", type=float, default=0.02)
    ap.add_argument("--image-noise", type=float, default=2.0,
                    help="Gaussian grayscale noise sigma in pixel values")
    ap.add_argument("--improved-only", action="store_true")
    ap.add_argument("--flow-hz", type=int, default=CAMERA_HZ,
                    choices=(10, 15, 30),
                    help="flow processing rate; 15 Hz models alternating CNN/flow frames")
    ap.add_argument("--quiet", action="store_true",
                    help="print only aggregate results")
    ap.add_argument("--out", type=Path)
    args = ap.parse_args()
    rows = []
    for speed in (1.0, 2.0, 3.0):
        for improved in ((True,) if args.improved_only else (False, True)):
            for trial in range(args.trials):
                rows.append(run(speed, improved, 1000 + trial,
                                args.packet_loss, args.image_noise, args.flow_hz))
    summary = {
        "backend": "Crazyflow first_principles / MuJoCo MJX, CPU",
        "camera": "HM01B0 160x160, fx=89.15584, 10ms exposure approximation",
        "transport": {"baud": 115200, "packet_bytes": 168,
                      "serialization_delay_ms": UART_SECONDS*1000},
        "trials": rows,
        "aggregate": {},
    }
    for pipeline in sorted({str(r["pipeline"]) for r in rows}):
        subset = [r for r in rows if r["pipeline"] == pipeline]
        detected = [r for r in subset if r["first_detection"]]
        summary["aggregate"][pipeline] = {
            "runs": len(subset),
            "detection_rate": len(detected)/len(subset),
            "collision_rate": sum(bool(r["collision"]) for r in subset)/len(subset),
            "ground_contact_rate": sum(bool(r["ground_contact"]) for r in subset)/len(subset),
            "safe_pass_rate": sum(bool(r["safe_pass"]) for r in subset)/len(subset),
            "mean_realtime_factor": sum(float(r["realtime_factor"]) for r in subset)/len(subset),
            "mean_detection_error_m": (sum(float(r["first_detection"]["error_m"])
                                            for r in detected)/len(detected)
                                       if detected else None),
        }
    text = json.dumps(summary, indent=2)
    print(json.dumps(summary["aggregate"], indent=2) if args.quiet else text)
    if args.out:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        args.out.write_text(text + "\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
