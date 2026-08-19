#!/usr/bin/env python3
"""Generate a compact set of flat-output trajectories for drone racing tests."""

from __future__ import annotations

import argparse
import csv
from dataclasses import dataclass
import math
from pathlib import Path

import numpy as np

from firmware_pybullet_env import GymPybulletGateEnv
from generate_banked_turn_trajectory import (
    ARM,
    DT,
    G,
    INERTIA,
    MASS,
    YAW_RATIO,
    desired_rotation,
    rotation_to_quaternion,
    smooth,
)


@dataclass(frozen=True)
class Turn:
    yaw_deg: float
    straight_after_s: float = 0.0


@dataclass(frozen=True)
class RacingManeuver:
    name: str
    description: str
    speed_mps: float
    bank_deg: float
    turns: tuple[Turn, ...]
    model_schedule: str
    altitude_bump_m: float = 0.0


MANEUVERS = (
    RacingManeuver(
        "sweeper_90", "90 degree constant-radius racing corner", 3.0, 30.0,
        (Turn(90.0),), "bank30",
    ),
    RacingManeuver(
        "hairpin_180", "tight 180 degree direction reversal", 2.0, 30.0,
        (Turn(180.0),), "bank30",
    ),
    RacingManeuver(
        "chicane", "left-right 45 degree gate chicane", 2.5, 30.0,
        (Turn(45.0, 0.45), Turn(-45.0)), "bank30",
    ),
    RacingManeuver(
        "slalom", "four alternating offsets with zero net heading", 2.0, 25.0,
        (Turn(30.0, 0.30), Turn(-60.0, 0.30), Turn(60.0, 0.30), Turn(-30.0)),
        "bank30",
    ),
    RacingManeuver(
        "elevation_chicane", "left-right chicane with a 0.5 m gate-height change", 2.5, 25.0,
        (Turn(40.0, 0.70), Turn(-40.0)), "bank30", altitude_bump_m=0.5,
    ),
    RacingManeuver(
        "high_speed_sweeper_180", "5 m/s, 60 degree banked half-circle", 5.0, 60.0,
        (Turn(180.0),), "bank60",
    ),
)


def _turn_profile(delta_yaw_deg: float, speed_mps: float, bank_deg: float) -> list[float]:
    direction = 1.0 if delta_yaw_deg >= 0.0 else -1.0
    target_bank = -direction * math.radians(bank_deg)
    ramp_s = 0.60
    ramp = [target_bank * smooth(index / max(1, int(round(ramp_s / DT))))
            for index in range(1, int(round(ramp_s / DT)) + 1)]
    ramp_yaw = 2.0 * sum((-G * math.tan(phi) / speed_mps) * DT for phi in ramp)
    requested = math.radians(delta_yaw_deg)
    remaining = requested - ramp_yaw
    hold_rate = -G * math.tan(target_bank) / speed_mps
    hold_count = max(0, int(round(remaining / (hold_rate * DT))))
    profile = ramp + [target_bank] * hold_count
    for index in range(1, len(ramp) + 1):
        profile.append(target_bank * (1.0 - smooth(index / len(ramp))))
    return profile


def _altitude_bump(times: np.ndarray, amplitude_m: float) -> tuple[np.ndarray, np.ndarray]:
    if amplitude_m == 0.0:
        return np.full_like(times, 1.1), np.zeros_like(times)
    duration = float(times[-1])
    u = times / max(DT, duration)
    height = np.zeros_like(times)
    for index, phase in enumerate(u):
        if 0.18 <= phase < 0.40:
            height[index] = amplitude_m * smooth((phase - 0.18) / 0.22)
        elif 0.40 <= phase < 0.62:
            height[index] = amplitude_m
        elif 0.62 <= phase < 0.84:
            height[index] = amplitude_m * (1.0 - smooth((phase - 0.62) / 0.22))
    altitude = 1.1 + height
    return altitude, np.gradient(altitude, DT, edge_order=2)


def generate(maneuver: RacingManeuver, out: Path) -> dict[str, float | str]:
    speed_ramp_s = 1.0
    ramp_count = int(round(speed_ramp_s / DT))
    bank_values = [0.0] * (ramp_count + 1)
    for turn in maneuver.turns:
        bank_values.extend(_turn_profile(turn.yaw_deg, maneuver.speed_mps, maneuver.bank_deg))
        bank_values.extend([0.0] * int(round(turn.straight_after_s / DT)))
    bank_values.extend([0.0] * ramp_count)
    bank = np.asarray(bank_values, dtype=np.float64)
    times = np.arange(len(bank), dtype=np.float64) * DT

    speed = np.full(len(times), maneuver.speed_mps, dtype=np.float64)
    for index in range(ramp_count + 1):
        speed[index] = maneuver.speed_mps * smooth(index / ramp_count)
        speed[-1 - index] = maneuver.speed_mps * smooth(index / ramp_count)
    yaw_rate = np.where(speed > 0.1, -G * np.tan(bank) / np.maximum(speed, 0.1), 0.0)
    yaw = np.zeros_like(times)
    for index in range(1, len(times)):
        yaw[index] = yaw[index - 1] + 0.5 * DT * (yaw_rate[index - 1] + yaw_rate[index])

    altitude, vertical_speed = _altitude_bump(times, maneuver.altitude_bump_m)
    velocity = np.column_stack((speed * np.cos(yaw), speed * np.sin(yaw), vertical_speed))
    position = np.zeros((len(times), 3), dtype=np.float64)
    position[:, 2] = altitude
    for index in range(1, len(times)):
        position[index, :2] = position[index - 1, :2] + 0.5 * DT * (
            velocity[index - 1, :2] + velocity[index, :2]
        )
    acceleration = np.gradient(velocity, DT, axis=0, edge_order=2)
    rotations = np.asarray([desired_rotation(value, float(heading))
                            for value, heading in zip(acceleration, yaw)])
    quaternions = np.asarray([rotation_to_quaternion(rotation) for rotation in rotations])
    for index in range(1, len(quaternions)):
        if np.dot(quaternions[index - 1], quaternions[index]) < 0.0:
            quaternions[index] *= -1.0

    omega = np.zeros((len(times), 3), dtype=np.float64)
    for index, rotation in enumerate(rotations):
        before = rotations[max(0, index - 1)]
        after = rotations[min(len(times) - 1, index + 1)]
        denominator = DT if index in (0, len(times) - 1) else 2.0 * DT
        skew = rotation.T @ ((after - before) / denominator)
        omega[index] = [
            0.5 * (skew[2, 1] - skew[1, 2]),
            0.5 * (skew[0, 2] - skew[2, 0]),
            0.5 * (skew[1, 0] - skew[0, 1]),
        ]

    allocation = np.asarray([
        [1, 1, 1, 1],
        [-ARM, -ARM, ARM, ARM],
        [-ARM, ARM, ARM, -ARM],
        [-YAW_RATIO, YAW_RATIO, -YAW_RATIO, YAW_RATIO],
    ], dtype=np.float64)
    omega_dot = np.gradient(omega, DT, axis=0, edge_order=2)
    motors = np.zeros((len(times), 4), dtype=np.float64)
    for index in range(len(times)):
        total = MASS * np.linalg.norm(acceleration[index] + np.asarray([0.0, 0.0, G]))
        torque = INERTIA @ omega_dot[index] + np.cross(omega[index], INERTIA @ omega[index])
        motors[index] = np.linalg.solve(allocation, np.r_[total, torque])
    maximum = GymPybulletGateEnv.FIRMWARE_MAX_MOTOR_THRUST_N
    if motors.min() < -1.0e-6 or motors.max() > maximum + 1.0e-6:
        raise RuntimeError(
            f"{maneuver.name} feedforward is infeasible: {motors.min():.4f}..{motors.max():.4f} N, "
            f"motor limit is {maximum:.4f} N"
        )

    out.parent.mkdir(parents=True, exist_ok=True)
    fields = [
        "t", "x", "y", "z", "vx", "vy", "vz", "qw", "qx", "qy", "qz",
        "wx", "wy", "wz", *[f"motor_{index}_thrust_n" for index in range(4)],
    ]
    with out.open("w", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(fields)
        for index, timestamp in enumerate(times):
            writer.writerow([
                f"{timestamp:.9f}", *position[index], *velocity[index], *quaternions[index],
                *omega[index], *motors[index],
            ])
    return {
        "name": maneuver.name,
        "description": maneuver.description,
        "trajectory": str(out),
        "duration_s": float(times[-1]),
        "speed_mps": maneuver.speed_mps,
        "bank_deg": maneuver.bank_deg,
        "yaw_change_deg": math.degrees(float(yaw[-1] - yaw[0])),
        "min_feedforward_motor_thrust_n": float(motors.min()),
        "max_feedforward_motor_thrust_n": float(motors.max()),
        "model_schedule": maneuver.model_schedule,
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--out-dir", type=Path, default=Path("sim/trajectories/racing"))
    parser.add_argument("--maneuver", action="append", choices=[item.name for item in MANEUVERS])
    args = parser.parse_args()
    selected = set(args.maneuver or [item.name for item in MANEUVERS])
    for maneuver in MANEUVERS:
        if maneuver.name not in selected:
            continue
        result = generate(maneuver, args.out_dir / f"{maneuver.name}.csv")
        print(
            f"wrote {result['trajectory']}: duration={result['duration_s']:.2f}s, "
            f"yaw={result['yaw_change_deg']:.1f}deg, motor="
            f"{result['min_feedforward_motor_thrust_n']:.3f}.."
            f"{result['max_feedforward_motor_thrust_n']:.3f}N"
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
