#!/usr/bin/env python3
"""Compare Gym/PyBullet one-step dynamics with the generated firmware model."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import numpy as np
import pybullet as p

from firmware_pybullet_env import GateGeometry, GymPybulletGateEnv, GymPybulletGateEnvConfig


def expected_acceleration(env, quat_xyzw, velocity_body, thrust):
    rotation = np.asarray(p.getMatrixFromQuaternion(quat_xyzw), float).reshape(3, 3)
    linear = np.asarray([0.0, 0.0, -env.GRAVITY_MPS2]) + rotation @ np.asarray(
        [0.0, 0.0, np.sum(thrust) / env._aviary.M]
    )
    arm = env.FIRMWARE_ARM_OFFSET_M
    yaw_ratio = env.FIRMWARE_THRUST_TO_YAW_TORQUE_M
    torque = np.asarray([
        arm * (-thrust[0] - thrust[1] + thrust[2] + thrust[3]),
        arm * (-thrust[0] + thrust[1] + thrust[2] - thrust[3]),
        yaw_ratio * (-thrust[0] + thrust[1] - thrust[2] + thrust[3]),
    ])
    inertia = np.diag(env.FIRMWARE_INERTIA_KGM2)
    angular = np.linalg.solve(inertia, torque - np.cross(velocity_body, inertia @ velocity_body))
    return linear, angular


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", type=Path, default=Path("sim_runs/plant_fidelity/report.json"))
    ap.add_argument("--dt", type=float, default=0.0005,
                    help="small one-step interval used for finite-difference comparison")
    args = ap.parse_args()
    hover = GymPybulletGateEnv.FIRMWARE_HOVER_THRUST_N
    cases = [
        ("hover", np.zeros(3), np.zeros(3), hover.copy()),
        ("collective", np.zeros(3), np.zeros(3), hover + 0.020),
        ("roll", np.zeros(3), np.zeros(3), hover + np.asarray([-0.015, -0.015, 0.015, 0.015])),
        ("pitch", np.zeros(3), np.zeros(3), hover + np.asarray([-0.015, 0.015, 0.015, -0.015])),
        ("yaw", np.zeros(3), np.zeros(3), hover + np.asarray([-0.010, 0.010, -0.010, 0.010])),
        ("single_motor_saturation", np.zeros(3), np.zeros(3),
         np.asarray([GymPybulletGateEnv.FIRMWARE_MAX_MOTOR_THRUST_N, 0.0, 0.0, 0.0])),
        ("tilted_collective", np.asarray([0.40, -0.30, 0.20]), np.zeros(3), hover + 0.015),
        ("coupled_rates", np.asarray([0.35, -0.25, 0.30]), np.asarray([0.50, -0.30, 0.40]),
         hover + np.asarray([-0.010, 0.006, 0.014, -0.004])),
    ]
    geometry = GateGeometry(3.0, 0.0, 1.1, 0.5, 0.5, 0.03)
    results = []
    for name, rpy, omega_body, thrust in cases:
        env = GymPybulletGateEnv(geometry, GymPybulletGateEnvConfig(dt=args.dt, ctrl_freq=round(1 / args.dt)))
        try:
            env.reset()
            body = int(env._aviary.DRONE_IDS[0])
            quat = np.asarray(p.getQuaternionFromEuler(rpy.tolist()))
            rotation = np.asarray(p.getMatrixFromQuaternion(quat), float).reshape(3, 3)
            p.resetBasePositionAndOrientation(body, [0, 0, 1.1], quat.tolist(), physicsClientId=env._client)
            p.resetBaseVelocity(body, [0.17, -0.11, 0.08], (rotation @ omega_body).tolist(), physicsClientId=env._client)
            before_linear, before_angular_world = p.getBaseVelocity(body, physicsClientId=env._client)
            expected_linear, expected_angular = expected_acceleration(env, quat, omega_body, thrust)
            env.step_motor_thrust(thrust - hover)
            _, quat_after = p.getBasePositionAndOrientation(body, physicsClientId=env._client)
            after_linear, after_angular_world = p.getBaseVelocity(body, physicsClientId=env._client)
            rotation_after = np.asarray(p.getMatrixFromQuaternion(quat_after), float).reshape(3, 3)
            before_body = rotation.T @ np.asarray(before_angular_world)
            after_body = rotation_after.T @ np.asarray(after_angular_world)
            measured_linear = (np.asarray(after_linear) - np.asarray(before_linear)) / args.dt
            measured_angular = (after_body - before_body) / args.dt
            results.append({
                "case": name,
                "expected_linear_accel_mps2": expected_linear.tolist(),
                "measured_linear_accel_mps2": measured_linear.tolist(),
                "linear_error_norm_mps2": float(np.linalg.norm(measured_linear - expected_linear)),
                "expected_body_angular_accel_radps2": expected_angular.tolist(),
                "measured_body_angular_accel_radps2": measured_angular.tolist(),
                "angular_error_norm_radps2": float(np.linalg.norm(measured_angular - expected_angular)),
            })
        finally:
            env.close()
    report = {
        "dt_s": args.dt,
        "state_conventions": {"quaternion": "PyBullet xyzw", "attitude": "Rodrigues q_xyz/q_w",
                              "linear_velocity": "world", "angular_velocity": "body"},
        "max_linear_error_norm_mps2": max(r["linear_error_norm_mps2"] for r in results),
        "max_angular_error_norm_radps2": max(r["angular_error_norm_radps2"] for r in results),
        "cases": results,
    }
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
