#!/usr/bin/env python3
"""Generate verified actuator-aware TinyMPC coordinated-turn model bundles.

The level bundle retains the controller's hover-relative local state chart.
Each signed bank bundle uses reference-relative rotating Frenet rigid-body
errors, with rotor states and motor-thrust inputs relative to that bundle's
coordinated-turn operating point. No runtime differentiation or Riccati work
is required.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import multiprocessing
from pathlib import Path
import sys

import numpy as np


HERE = Path(__file__).resolve().parent
APP = HERE.parents[1]
PYBULLET_TOOLS = APP / "tools" / "pybullet_simulation"
sys.path.insert(0, str(PYBULLET_TOOLS))
sys.path.insert(0, str(HERE))

import generate_level_actuator_lti as level  # noqa: E402
import crazysim_runtime_profile as plant_profile  # noqa: E402
from quadrotor_dynamics import (  # noqa: E402
    ALLOCATION,
    ARM_OFFSET_M,
    BODY_LINEAR_DRAG_N_PER_MPS,
    INERTIA_KGM2,
    MASS_KG,
    MOTOR_STATE_DIM,
    RPM_TO_TORQUE,
    ROTOR_STATE_SCALE_RPM,
    _quat_conjugate,
    _quat_product,
    _quat_rotation,
    _normalize_quat,
    _rk4_step,
    _rpm_polynomial,
    _thrust_to_rpm,
)
from tinympc_to_crazyflie_adapter import (  # noqa: E402
    CompileTimeProblem,
    build_upstream_cache,
)


STATE_DIM = level.STATE_DIM
INPUT_DIM = level.INPUT_DIM
DT_S = level.DT_S
RADIUS_M = 0.75
SPEEDS_MPS = (1.0, 1.5, 2.0, 2.5, 3.0)
NONLINEAR_FIXED_POINT_TOLERANCE = 1.0e-7
BANK_ROLL_STATE_WEIGHT = 40.0
BANK_PITCH_STATE_WEIGHT = 4.0
BANK_ROLL_RATE_STATE_WEIGHT = 20.0
BANK_PITCH_RATE_STATE_WEIGHT = 20.0
BANK_TANGENTIAL_VELOCITY_STATE_WEIGHT = 5000.0
BANK_LATERAL_VELOCITY_STATE_WEIGHT = 5000.0
# Keep the contouring position cost modest. The path tunnel provides the hard
# cross-track safety envelope instead of asking this quadratic term to pin the
# vehicle to the centerline.
BANK_LATERAL_POSITION_STATE_WEIGHT = level.POSITION_STATE_WEIGHT
STATE_STEPS = np.asarray(
    [1.0e-4] * 3 + [2.0e-5] * 3 + [1.0e-4] * 3
    + [2.0e-4] * 3 + [2.0e-5] * MOTOR_STATE_DIM)
INPUT_STEPS = np.full(INPUT_DIM, 2.0e-5)


def _physical_force_moment(motor_thrust_n: np.ndarray) -> np.ndarray:
    motor_thrust_n = np.asarray(motor_thrust_n, dtype=float)
    force_moment = ALLOCATION @ motor_thrust_n
    motor_rpm = _thrust_to_rpm(motor_thrust_n)
    motor_torque_nm = _rpm_polynomial(motor_rpm, RPM_TO_TORQUE)
    force_moment[3] = np.asarray([-1.0, 1.0, -1.0, 1.0]) @ motor_torque_nm
    return force_moment


def _solve_motor_thrust(target_force_moment: np.ndarray) -> np.ndarray:
    """Solve the nonlinear RPM-polynomial yaw allocation deterministically."""
    target_force_moment = np.asarray(target_force_moment, dtype=float)
    yaw_ratio = 7.73e-11 / 3.72e-8
    linear_allocation = np.asarray([
        [1.0, 1.0, 1.0, 1.0],
        [-ARM_OFFSET_M, -ARM_OFFSET_M, ARM_OFFSET_M, ARM_OFFSET_M],
        [-ARM_OFFSET_M, ARM_OFFSET_M, ARM_OFFSET_M, -ARM_OFFSET_M],
        [-yaw_ratio, yaw_ratio, -yaw_ratio, yaw_ratio],
    ])
    thrust = np.linalg.solve(linear_allocation, target_force_moment)
    for _ in range(12):
        residual = _physical_force_moment(thrust) - target_force_moment
        if float(np.max(np.abs(residual))) < 1.0e-12:
            break
        jacobian = np.empty((4, 4))
        for column in range(4):
            step = 1.0e-6
            perturbation = np.zeros(4)
            perturbation[column] = step
            jacobian[:, column] = (
                _physical_force_moment(thrust + perturbation)
                - _physical_force_moment(thrust - perturbation)
            ) / (2.0 * step)
        thrust -= np.linalg.solve(jacobian, residual)
    residual = _physical_force_moment(thrust) - target_force_moment
    if float(np.max(np.abs(residual))) >= 1.0e-10:
        raise RuntimeError(
            f"motor allocation did not converge: {np.max(np.abs(residual))}")
    if np.any(thrust < 0.0) or np.any(thrust > level.MAX_MOTOR_THRUST_N):
        raise RuntimeError(f"motor allocation outside physical limits: {thrust}")
    return thrust


def _controller_state_to_absolute(
    state: np.ndarray, hover_rotor_state: float
) -> np.ndarray:
    quaternion = _normalize_quat(np.r_[1.0, state[3:6]])
    return np.r_[
        state[0:3], quaternion, state[6:9], state[9:12],
        state[12:16] + hover_rotor_state,
    ]


def _absolute_to_controller_state(
    absolute: np.ndarray, hover_rotor_state: float
) -> np.ndarray:
    quaternion = _normalize_quat(absolute[3:7])
    if quaternion[0] < 0.0:
        quaternion *= -1.0
    if abs(float(quaternion[0])) < 1.0e-9:
        raise RuntimeError("bank model reached the Rodrigues chart singularity")
    return np.r_[
        absolute[0:3], quaternion[1:4] / quaternion[0], absolute[7:10],
        absolute[10:13], absolute[13:17] - hover_rotor_state,
    ]


def _transition(
    state: np.ndarray,
    delta_motor_command_n: np.ndarray,
    hover_thrust_n: float,
    hover_rotor_state: float,
) -> np.ndarray:
    absolute = _controller_state_to_absolute(state, hover_rotor_state)
    next_absolute = _rk4_step(
        absolute,
        np.full(INPUT_DIM, hover_thrust_n) + delta_motor_command_n,
        DT_S,
    )
    return _absolute_to_controller_state(next_absolute, hover_rotor_state)


def _bank_reference_absolute(
    metadata: dict[str, object], physical_input: np.ndarray, time_s: float
) -> np.ndarray:
    """Canonical coordinated-turn reference in a yaw-rotating Frenet frame."""
    speed_mps = float(metadata["speed_mps"])
    yaw_rate_rad_s = float(metadata["yaw_rate_rad_s"])
    roll_rad = float(metadata["roll_rad"])
    pitch_rad = float(metadata.get("pitch_rad", 0.0))
    yaw_rad = yaw_rate_rad_s * time_s
    curvature_per_m = yaw_rate_rad_s / speed_mps
    position = np.asarray([
        math.sin(yaw_rad) / curvature_per_m,
        (1.0 - math.cos(yaw_rad)) / curvature_per_m,
        0.0,
    ])
    half_yaw = 0.5 * yaw_rad
    half_pitch = 0.5 * pitch_rad
    half_roll = 0.5 * roll_rad
    yaw_quaternion = np.asarray([
        math.cos(half_yaw), 0.0, 0.0, math.sin(half_yaw)])
    pitch_quaternion = np.asarray([
        math.cos(half_pitch), 0.0, math.sin(half_pitch), 0.0])
    roll_quaternion = np.asarray([
        math.cos(half_roll), math.sin(half_roll), 0.0, 0.0])
    quaternion = _normalize_quat(_quat_product(
        yaw_quaternion, _quat_product(pitch_quaternion, roll_quaternion)))
    velocity = np.asarray([
        speed_mps * math.cos(yaw_rad),
        speed_mps * math.sin(yaw_rad),
        0.0,
    ])
    body_rate = _quat_rotation(quaternion).T @ np.asarray(
        [0.0, 0.0, yaw_rate_rad_s])
    rotor_state = _thrust_to_rpm(physical_input) / ROTOR_STATE_SCALE_RPM
    return np.r_[position, quaternion, velocity, body_rate, rotor_state]


def _yaw_rotation(yaw_rad: float) -> np.ndarray:
    cosine = math.cos(yaw_rad)
    sine = math.sin(yaw_rad)
    return np.asarray([
        [cosine, -sine, 0.0],
        [sine, cosine, 0.0],
        [0.0, 0.0, 1.0],
    ])


def _bank_error_to_absolute(
    error: np.ndarray, reference: np.ndarray, yaw_rad: float
) -> np.ndarray:
    yaw_rotation = _yaw_rotation(yaw_rad)
    error_quaternion = _normalize_quat(np.r_[1.0, error[3:6]])
    quaternion = _normalize_quat(_quat_product(reference[3:7], error_quaternion))
    actual_rotation = _quat_rotation(quaternion)
    reference_rotation = _quat_rotation(reference[3:7])
    return np.r_[
        reference[0:3] + yaw_rotation @ error[0:3],
        quaternion,
        reference[7:10] + yaw_rotation @ error[6:9],
        error[9:12] + actual_rotation.T @ reference_rotation @ reference[10:13],
        reference[13:17] + error[12:16],
    ]


def _bank_error_from_absolute(
    absolute: np.ndarray, reference: np.ndarray, yaw_rad: float
) -> np.ndarray:
    actual_quaternion = _normalize_quat(absolute[3:7])
    error_quaternion = _normalize_quat(_quat_product(
        _quat_conjugate(reference[3:7]), actual_quaternion))
    if error_quaternion[0] < 0.0:
        error_quaternion *= -1.0
    if abs(float(error_quaternion[0])) < 1.0e-9:
        raise RuntimeError("bank error reached the Rodrigues chart singularity")
    yaw_rotation = _yaw_rotation(yaw_rad)
    actual_rotation = _quat_rotation(actual_quaternion)
    reference_rotation = _quat_rotation(reference[3:7])
    return np.r_[
        yaw_rotation.T @ (absolute[0:3] - reference[0:3]),
        error_quaternion[1:4] / error_quaternion[0],
        yaw_rotation.T @ (absolute[7:10] - reference[7:10]),
        absolute[10:13]
        - actual_rotation.T @ reference_rotation @ reference[10:13],
        absolute[13:17] - reference[13:17],
    ]


def _bank_error_transition(
    error: np.ndarray,
    delta_motor_command_n: np.ndarray,
    metadata: dict[str, object],
    physical_input: np.ndarray,
) -> np.ndarray:
    reference = _bank_reference_absolute(metadata, physical_input, 0.0)
    absolute = _bank_error_to_absolute(error, reference, 0.0)
    next_absolute = _rk4_step(
        absolute, physical_input + delta_motor_command_n, DT_S)
    next_yaw = float(metadata["yaw_rate_rad_s"]) * DT_S
    next_reference = _bank_reference_absolute(
        metadata, physical_input, DT_S)
    return _bank_error_from_absolute(next_absolute, next_reference, next_yaw)


def _linearize_transition(transition) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    nominal_state = np.zeros(STATE_DIM)
    nominal_input = np.zeros(INPUT_DIM)
    nominal_next = transition(nominal_state, nominal_input)
    a = np.empty((STATE_DIM, STATE_DIM))
    b = np.empty((STATE_DIM, INPUT_DIM))
    for column, step in enumerate(STATE_STEPS):
        perturbation = np.zeros(STATE_DIM)
        perturbation[column] = step
        a[:, column] = (
            transition(nominal_state + perturbation, nominal_input)
            - transition(nominal_state - perturbation, nominal_input)
        ) / (2.0 * step)
    for column, step in enumerate(INPUT_STEPS):
        perturbation = np.zeros(INPUT_DIM)
        perturbation[column] = step
        b[:, column] = (
            transition(nominal_state, nominal_input + perturbation)
            - transition(nominal_state, nominal_input - perturbation)
        ) / (2.0 * step)
    affine = nominal_next.copy()
    return a, b, affine, nominal_next


def _linearize(
    nominal_state: np.ndarray,
    nominal_input: np.ndarray,
    hover_thrust_n: float,
    hover_rotor_state: float,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    transition = lambda state, motor_input: _transition(
        state, motor_input, hover_thrust_n, hover_rotor_state)
    nominal_next = transition(nominal_state, nominal_input)
    a = np.empty((STATE_DIM, STATE_DIM))
    b = np.empty((STATE_DIM, INPUT_DIM))
    for column, step in enumerate(STATE_STEPS):
        perturbation = np.zeros(STATE_DIM)
        perturbation[column] = step
        a[:, column] = (
            transition(nominal_state + perturbation, nominal_input)
            - transition(nominal_state - perturbation, nominal_input)
        ) / (2.0 * step)
    for column, step in enumerate(INPUT_STEPS):
        perturbation = np.zeros(INPUT_DIM)
        perturbation[column] = step
        b[:, column] = (
            transition(nominal_state, nominal_input + perturbation)
            - transition(nominal_state, nominal_input - perturbation)
        ) / (2.0 * step)
    affine = nominal_next - a @ nominal_state - b @ nominal_input
    return a, b, affine, nominal_next


def _operating_point(
    side_sign: int,
    speed_mps: float,
    hover_thrust_n: float,
    hover_rotor_state: float,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, dict[str, object]]:
    if side_sign == 0:
        physical_input = np.full(INPUT_DIM, hover_thrust_n)
        target_force_moment = np.asarray(
            [INPUT_DIM * hover_thrust_n, 0.0, 0.0, 0.0])
        metadata = {
            "side_sign": 0,
            "speed_mps": 0.0,
            "radius_m": 0.0,
            "roll_rad": 0.0,
            "pitch_rad": 0.0,
            "yaw_rate_rad_s": 0.0,
            "nominal_total_thrust_n": float(INPUT_DIM * hover_thrust_n),
            "nominal_tangential_drag_accel_mps2": 0.0,
        }
        return (
            np.zeros(STATE_DIM), np.zeros(INPUT_DIM), physical_input,
            target_force_moment, metadata)

    yaw_rate_rad_s = float(side_sign) * speed_mps / RADIUS_M
    velocity_local = np.asarray([speed_mps, 0.0, 0.0])
    acceleration_local = np.asarray([
        0.0, float(side_sign) * speed_mps * speed_mps / RADIUS_M, 0.0])
    gravity_local = np.asarray([0.0, 0.0, -9.81])

    # Solve the constant yaw-relative attitude whose thrust balances gravity,
    # centripetal acceleration, and CrazySim's anisotropic body-frame drag.
    roll_rad = -float(side_sign) * math.atan(
        speed_mps * speed_mps / (9.81 * RADIUS_M))
    pitch_rad = 0.0
    for _ in range(100):
        attitude = _bank_reference_absolute({
            "speed_mps": speed_mps,
            "yaw_rate_rad_s": yaw_rate_rad_s,
            "roll_rad": roll_rad,
            "pitch_rad": pitch_rad,
        }, np.full(INPUT_DIM, hover_thrust_n), 0.0)
        rotation_body_to_local = _quat_rotation(attitude[3:7])
        drag_force_local = rotation_body_to_local @ (
            BODY_LINEAR_DRAG_N_PER_MPS
            @ (rotation_body_to_local.T @ velocity_local))
        required_thrust_local = (
            MASS_KG * (acceleration_local - gravity_local)
            - drag_force_local)
        thrust_direction = required_thrust_local / np.linalg.norm(
            required_thrust_local)
        next_roll = -math.asin(float(np.clip(
            thrust_direction[1], -1.0, 1.0)))
        next_pitch = math.atan2(
            float(thrust_direction[0]), float(thrust_direction[2]))
        if max(abs(next_roll - roll_rad), abs(next_pitch - pitch_rad)) < 1.0e-14:
            roll_rad, pitch_rad = next_roll, next_pitch
            break
        roll_rad, pitch_rad = next_roll, next_pitch
    else:
        raise RuntimeError("drag-aware coordinated-turn attitude did not converge")

    attitude = _bank_reference_absolute({
        "speed_mps": speed_mps,
        "yaw_rate_rad_s": yaw_rate_rad_s,
        "roll_rad": roll_rad,
        "pitch_rad": pitch_rad,
    }, np.full(INPUT_DIM, hover_thrust_n), 0.0)
    rotation_body_to_local = _quat_rotation(attitude[3:7])
    drag_force_local = rotation_body_to_local @ (
        BODY_LINEAR_DRAG_N_PER_MPS
        @ (rotation_body_to_local.T @ velocity_local))
    required_thrust_local = (
        MASS_KG * (acceleration_local - gravity_local) - drag_force_local)
    total_thrust_n = float(np.linalg.norm(required_thrust_local))
    translational_residual = (
        rotation_body_to_local @ np.asarray([0.0, 0.0, total_thrust_n])
        + drag_force_local
        + MASS_KG * gravity_local
        - MASS_KG * acceleration_local)
    if float(np.max(np.abs(translational_residual))) > 1.0e-12:
        raise RuntimeError(
            f"drag-aware operating point residual {translational_residual}")
    body_rate = rotation_body_to_local.T @ np.asarray(
        [0.0, 0.0, yaw_rate_rad_s])
    required_body_torque = np.cross(body_rate, INERTIA_KGM2 @ body_rate)
    target_force_moment = np.r_[total_thrust_n, required_body_torque]
    physical_input = _solve_motor_thrust(target_force_moment)
    rotor_state = (
        _thrust_to_rpm(physical_input) / ROTOR_STATE_SCALE_RPM
        - hover_rotor_state)
    # Banked bundles operate in reference-relative error coordinates. Their
    # equilibrium state and input correction are exactly zero; physical_input
    # records the motor-command origin used by firmware.
    nominal_state = np.zeros(STATE_DIM)
    nominal_input = np.zeros(INPUT_DIM)
    metadata = {
        "side_sign": int(side_sign),
        "speed_mps": float(speed_mps),
        "radius_m": RADIUS_M,
        "roll_rad": roll_rad,
        "pitch_rad": pitch_rad,
        "yaw_rate_rad_s": yaw_rate_rad_s,
        "nominal_total_thrust_n": total_thrust_n,
        "nominal_tangential_drag_accel_mps2": float(
            -BODY_LINEAR_DRAG_N_PER_MPS[0, 0] * speed_mps / MASS_KG),
        "nominal_drag_force_local_n": drag_force_local.tolist(),
        "translational_equilibrium_residual_max_abs": float(
            np.max(np.abs(translational_residual))),
    }
    return (
        nominal_state, nominal_input, physical_input,
        target_force_moment, metadata)


def _cache_bundle(
    a: np.ndarray, b: np.ndarray, affine: np.ndarray, *, banked: bool
) -> tuple[np.ndarray, np.ndarray, dict[str, np.ndarray], float]:
    q_diagonal, r = level._cost_modes()["baseline"]
    q_diagonal = q_diagonal.copy()
    if banked:
        q_diagonal[1] = BANK_LATERAL_POSITION_STATE_WEIGHT
        q_diagonal[3] = BANK_ROLL_STATE_WEIGHT
        q_diagonal[4] = BANK_PITCH_STATE_WEIGHT
        q_diagonal[6] = BANK_TANGENTIAL_VELOCITY_STATE_WEIGHT
        q_diagonal[7] = BANK_LATERAL_VELOCITY_STATE_WEIGHT
        q_diagonal[9] = BANK_ROLL_RATE_STATE_WEIGHT
        q_diagonal[10] = BANK_PITCH_RATE_STATE_WEIGHT
    problem = CompileTimeProblem(state_dim=STATE_DIM, input_dim=INPUT_DIM)
    cache = build_upstream_cache(
        problem, a, b, affine, np.diag(q_diagonal), r)
    augmented_r = r + level.RHO * np.eye(INPUT_DIM)
    cache["coeff_d2p"] = (
        cache["Kinf"].T @ augmented_r
        - cache["AmBKt"] @ cache["Pinf"] @ b)
    spectral_radius = float(np.max(np.abs(
        np.linalg.eigvals(a - b @ cache["Kinf"]))))
    return q_diagonal, r, cache, spectral_radius


def _verify_bundle(bundle: dict[str, object]) -> dict[str, float | bool]:
    a = bundle["A"]
    b = bundle["B"]
    affine = bundle["affine"]
    nominal_state = bundle["nominal_state"]
    nominal_input = bundle["nominal_input"]
    physical_input = bundle["physical_input"]
    nominal_next = bundle["nominal_next"]
    q_diagonal = bundle["Q_diagonal"]
    r = bundle["R"]
    cache = bundle["cache"]
    expected_shapes = {
        "A": (STATE_DIM, STATE_DIM),
        "B": (STATE_DIM, INPUT_DIM),
        "affine": (STATE_DIM,),
        "nominal_state": (STATE_DIM,),
        "nominal_input": (INPUT_DIM,),
        "physical_input": (INPUT_DIM,),
        "Q_diagonal": (STATE_DIM,),
        "R": (INPUT_DIM, INPUT_DIM),
        "Kinf": (INPUT_DIM, STATE_DIM),
        "Pinf": (STATE_DIM, STATE_DIM),
        "Quu_inv": (INPUT_DIM, INPUT_DIM),
        "AmBKt": (STATE_DIM, STATE_DIM),
        "coeff_d2p": (STATE_DIM, INPUT_DIM),
        "APf": (STATE_DIM,),
        "BPf": (INPUT_DIM,),
    }
    values = {
        "A": a, "B": b, "affine": affine,
        "nominal_state": nominal_state, "nominal_input": nominal_input,
        "physical_input": physical_input, "Q_diagonal": q_diagonal, "R": r,
        **{key: cache[key] for key in (
            "Kinf", "Pinf", "Quu_inv", "AmBKt", "coeff_d2p", "APf", "BPf")},
    }
    for name, shape in expected_shapes.items():
        if np.asarray(values[name]).shape != shape:
            raise RuntimeError(
                f"{bundle['name']} {name} shape {np.asarray(values[name]).shape} != {shape}")
    if not all(np.all(np.isfinite(value)) for value in values.values()):
        raise RuntimeError(f"{bundle['name']} contains a non-finite value")
    if np.any(physical_input < 0.0) or np.any(
        physical_input > level.MAX_MOTOR_THRUST_N):
        raise RuntimeError(f"{bundle['name']} input bounds violated")
    affine_identity_error = float(np.max(np.abs(
        nominal_next - (a @ nominal_state + b @ nominal_input + affine))))
    quu = r + level.RHO * np.eye(INPUT_DIM) + b.T @ cache["Pinf"] @ b
    quu_identity_error = float(np.max(np.abs(
        cache["Quu_inv"] @ quu - np.eye(INPUT_DIM))))
    ambkt_error = float(np.max(np.abs(
        cache["AmBKt"] - (a - b @ cache["Kinf"]).T)))
    apf_error = float(np.max(np.abs(
        cache["APf"] - cache["AmBKt"] @ cache["Pinf"] @ affine)))
    bpf_error = float(np.max(np.abs(
        cache["BPf"] - b.T @ cache["Pinf"] @ affine)))
    coeff_error = float(np.max(np.abs(
        cache["coeff_d2p"] - (
            cache["Kinf"].T @ (r + level.RHO * np.eye(INPUT_DIM))
            - cache["AmBKt"] @ cache["Pinf"] @ b))))
    maximum_cache_error = max(
        quu_identity_error, ambkt_error, apf_error, bpf_error, coeff_error)
    if affine_identity_error > 1.0e-10 or maximum_cache_error > 1.0e-8:
        raise RuntimeError(
            f"{bundle['name']} cache identity failed: "
            f"affine={affine_identity_error}, cache={maximum_cache_error}")
    spectral_radius = float(bundle["spectral_radius"])
    if not spectral_radius < 1.0:
        raise RuntimeError(
            f"{bundle['name']} closed-loop radius {spectral_radius} >= 1")
    if not all(np.all(np.isfinite(np.asarray(value, dtype=np.float32)))
               for value in values.values()):
        raise RuntimeError(f"{bundle['name']} float32 emission is non-finite")
    force_moment_error = float(np.max(np.abs(
        _physical_force_moment(physical_input)
        - bundle["target_force_moment"])))
    if force_moment_error > 1.0e-10:
        raise RuntimeError(
            f"{bundle['name']} operating-point allocation residual {force_moment_error}")
    return {
        "dimensions_valid": True,
        "finite_double_and_float32": True,
        "input_bounds_valid": True,
        "affine_identity_max_abs": affine_identity_error,
        "cache_identity_max_abs": maximum_cache_error,
        "allocation_residual_max_abs": force_moment_error,
        "closed_loop_spectral_radius": spectral_radius,
    }


def _make_bundle(
    name: str,
    model_id: int,
    side_sign: int,
    speed_mps: float,
    hover_thrust_n: float,
    hover_rotor_state: float,
) -> dict[str, object]:
    (nominal_state, nominal_input, physical_input,
     target_force_moment, metadata) = _operating_point(
         side_sign, speed_mps, hover_thrust_n, hover_rotor_state)
    if side_sign == 0:
        a, b, affine, nominal_next = _linearize(
            nominal_state, nominal_input, hover_thrust_n, hover_rotor_state)
    else:
        transition = lambda state, motor_input: _bank_error_transition(
            state, motor_input, metadata, physical_input)
        a, b, affine, nominal_next = _linearize_transition(transition)
    q_diagonal, r, cache, spectral_radius = _cache_bundle(
        a, b, affine, banked=side_sign != 0)
    bundle: dict[str, object] = {
        "name": name,
        "model_id": model_id,
        "A": a,
        "B": b,
        "affine": affine,
        "nominal_state": nominal_state,
        "nominal_input": nominal_input,
        "physical_input": physical_input,
        "nominal_next": nominal_next,
        "Q_diagonal": q_diagonal,
        "R": r,
        "cache": cache,
        "spectral_radius": spectral_radius,
        "target_force_moment": target_force_moment,
        "metadata": metadata,
    }
    bundle["verification"] = _verify_bundle(bundle)
    if side_sign != 0:
        absolute = _bank_reference_absolute(metadata, physical_input, 0.0)
        maximum_error = 0.0
        for knot in range(1, 20):
            absolute = _rk4_step(absolute, physical_input, DT_S)
            reference = _bank_reference_absolute(
                metadata, physical_input, knot * DT_S)
            error = _bank_error_from_absolute(
                absolute, reference,
                float(metadata["yaw_rate_rad_s"]) * knot * DT_S)
            maximum_error = max(maximum_error, float(np.max(np.abs(error))))
        bundle["verification"][
            "nonlinear_fixed_point_20_knot_max_abs"] = maximum_error
        # The high-rate coordinated-turn reference and plant are both RK4
        # integrated in float64. At the 2.5--3.0 m/s operating points their
        # 20-knot subtraction residual is a few 1e-8, well below firmware
        # float precision but above the old low-speed-only threshold.
        if maximum_error > NONLINEAR_FIXED_POINT_TOLERANCE:
            raise RuntimeError(
                f"{name} nonlinear Frenet fixed point failed: {maximum_error}")
    return bundle


def build_bundles() -> list[dict[str, object]]:
    hover_reference = level._hover_reference()
    hover_thrust_n = float(hover_reference.motor_thrust_n[0, 0])
    hover_rotor_state = float(hover_reference.motor_state[0, 0])
    definitions = [
        ("level", 0, 0, 0.0),
        ("left_low", 1, 1, SPEEDS_MPS[0]),
        ("right_low", 2, -1, SPEEDS_MPS[0]),
        ("left_medium", 3, 1, SPEEDS_MPS[1]),
        ("right_medium", 4, -1, SPEEDS_MPS[1]),
        ("left_high", 5, 1, SPEEDS_MPS[2]),
        ("right_high", 6, -1, SPEEDS_MPS[2]),
        ("left_very_high", 7, 1, SPEEDS_MPS[3]),
        ("right_very_high", 8, -1, SPEEDS_MPS[3]),
        ("left_maximum", 9, 1, SPEEDS_MPS[4]),
        ("right_maximum", 10, -1, SPEEDS_MPS[4]),
    ]
    # Upstream cache construction loads a temporary native library.  ctypes
    # cannot unload it reliably, so building every bundle in one process
    # can retain hundreds of MiB per bundle.  One deterministic forked worker
    # per bundle bounds peak memory without changing any generated numerics.
    context = multiprocessing.get_context("fork")
    bundles = []
    for name, identifier, side_sign, speed_mps in definitions:
        with context.Pool(processes=1, maxtasksperchild=1) as pool:
            bundles.append(pool.apply(
                _make_bundle,
                (name, identifier, side_sign, speed_mps,
                 hover_thrust_n, hover_rotor_state)))
    # The direct hover model must reproduce the already accepted level model.
    accepted_a, accepted_b, accepted_affine = level._linearize_interval(
        hover_reference, 0)
    level_bundle = bundles[0]
    parity_error = max(
        float(np.max(np.abs(level_bundle["A"] - accepted_a))),
        float(np.max(np.abs(level_bundle["B"] - accepted_b))),
        float(np.max(np.abs(level_bundle["affine"] - accepted_affine))),
    )
    if parity_error > 1.0e-8:
        raise RuntimeError(f"accepted hover-model parity failed: {parity_error}")
    level_bundle["verification"]["accepted_hover_parity_max_abs"] = parity_error
    return bundles


def _array(name: str, values: np.ndarray) -> str:
    return level._array(name, values)


def _bundle_arrays(bundle: dict[str, object]) -> str:
    prefix = f"tinympc_bank_{bundle['name']}"
    cache = bundle["cache"]
    arrays = [
        ("A", bundle["A"]),
        ("B", bundle["B"]),
        ("affine", bundle["affine"]),
        ("nominal_state", bundle["nominal_state"]),
        ("nominal_input", bundle["nominal_input"]),
        ("physical_input", bundle["physical_input"]),
        ("Q_diagonal", bundle["Q_diagonal"]),
        ("R", bundle["R"]),
        ("K", cache["Kinf"]),
        ("P", cache["Pinf"]),
        ("Quu_inv", cache["Quu_inv"]),
        ("AmBKt", cache["AmBKt"]),
        ("coeff_d2p", cache["coeff_d2p"]),
        ("APf", cache["APf"]),
        ("BPf", cache["BPf"]),
    ]
    return "\n\n".join(
        _array(f"{prefix}_{suffix}", value) for suffix, value in arrays)


def _render_header(
    bundles: list[dict[str, object]], generator_sha256: str,
    dynamics_sha256: str, profile_sha256: str
) -> str:
    blocks = [_bundle_arrays(bundle) for bundle in bundles]
    table_rows = []
    for bundle in bundles:
        prefix = f"tinympc_bank_{bundle['name']}"
        metadata = bundle["metadata"]
        table_rows.append(
            "    {"
            f"{bundle['model_id']}, {level._literal(metadata['roll_rad'])}, "
            f"{level._literal(metadata['speed_mps'])}, "
            f"{level._literal(metadata['radius_m'])}, "
            f"{level._literal(metadata['yaw_rate_rad_s'])}, "
            f"{level._literal(metadata['pitch_rad'])}, "
            f"{level._literal(metadata['nominal_total_thrust_n'])}, "
            f"{level._literal(metadata['nominal_tangential_drag_accel_mps2'])}, "
            f"{prefix}_A, {prefix}_B, {prefix}_affine, "
            f"{prefix}_nominal_state, {prefix}_nominal_input, "
            f"{prefix}_physical_input, {prefix}_Q_diagonal, {prefix}_R, "
            f"{prefix}_K, {prefix}_P, {prefix}_Quu_inv, {prefix}_AmBKt, "
            f"{prefix}_coeff_d2p, {prefix}_APf, {prefix}_BPf"
            "},")
    return f"""/* Autogenerated by generate_banked_model_bank.py.
 * generator_sha256={generator_sha256}
 * dynamics_sha256={dynamics_sha256}
 * profile_sha256={profile_sha256}
 * LEVEL uses the legacy absolute local chart. Banked bundles use rotating
 * Frenet/reference-relative error states and motor inputs relative to each
 * bundle's physical_input operating point. */
#ifndef TINYMPC_BANKED_MODEL_BANK_H
#define TINYMPC_BANKED_MODEL_BANK_H

#define TINYMPC_BANK_MODEL_STATE_DIM {STATE_DIM}
#define TINYMPC_BANK_MODEL_INPUT_DIM {INPUT_DIM}
#define TINYMPC_BANK_MODEL_COUNT {len(bundles)}
#define TINYMPC_BANK_MODEL_RADIUS_M ({level._literal(RADIUS_M)})
#define TINYMPC_BANK_MODEL_MASS_KG ({level._literal(plant_profile.MASS_KG)})
#define TINYMPC_BANK_MODEL_DRAG_X_N_PER_MPS ({level._literal(plant_profile.BODY_LINEAR_DRAG_DIAGONAL_N_PER_MPS[0])})
#define TINYMPC_BANK_MODEL_DRAG_Y_N_PER_MPS ({level._literal(plant_profile.BODY_LINEAR_DRAG_DIAGONAL_N_PER_MPS[1])})
#define TINYMPC_BANK_MODEL_DRAG_Z_N_PER_MPS ({level._literal(plant_profile.BODY_LINEAR_DRAG_DIAGONAL_N_PER_MPS[2])})
#define TINYMPC_BANK_BUNDLE_ID_LEVEL 0
#define TINYMPC_BANK_BUNDLE_ID_LEFT_LOW 1
#define TINYMPC_BANK_BUNDLE_ID_RIGHT_LOW 2
#define TINYMPC_BANK_BUNDLE_ID_LEFT_MEDIUM 3
#define TINYMPC_BANK_BUNDLE_ID_RIGHT_MEDIUM 4
#define TINYMPC_BANK_BUNDLE_ID_LEFT_HIGH 5
#define TINYMPC_BANK_BUNDLE_ID_RIGHT_HIGH 6
#define TINYMPC_BANK_BUNDLE_ID_LEFT_VERY_HIGH 7
#define TINYMPC_BANK_BUNDLE_ID_RIGHT_VERY_HIGH 8
#define TINYMPC_BANK_BUNDLE_ID_LEFT_MAXIMUM 9
#define TINYMPC_BANK_BUNDLE_ID_RIGHT_MAXIMUM 10
#define TINYMPC_BANK_MODEL_LOW_SPEED_MPS ({level._literal(SPEEDS_MPS[0])})
#define TINYMPC_BANK_MODEL_MEDIUM_SPEED_MPS ({level._literal(SPEEDS_MPS[1])})
#define TINYMPC_BANK_MODEL_HIGH_SPEED_MPS ({level._literal(SPEEDS_MPS[2])})
#define TINYMPC_BANK_MODEL_VERY_HIGH_SPEED_MPS ({level._literal(SPEEDS_MPS[3])})
#define TINYMPC_BANK_MODEL_MAXIMUM_SPEED_MPS ({level._literal(SPEEDS_MPS[4])})
#define TINYMPC_BANK_MODEL_LEFT_LOW_ROLL_RAD ({level._literal(bundles[1]['metadata']['roll_rad'])})
#define TINYMPC_BANK_MODEL_RIGHT_LOW_ROLL_RAD ({level._literal(bundles[2]['metadata']['roll_rad'])})
#define TINYMPC_BANK_MODEL_LEFT_MEDIUM_ROLL_RAD ({level._literal(bundles[3]['metadata']['roll_rad'])})
#define TINYMPC_BANK_MODEL_RIGHT_MEDIUM_ROLL_RAD ({level._literal(bundles[4]['metadata']['roll_rad'])})
#define TINYMPC_BANK_MODEL_LEFT_HIGH_ROLL_RAD ({level._literal(bundles[5]['metadata']['roll_rad'])})
#define TINYMPC_BANK_MODEL_RIGHT_HIGH_ROLL_RAD ({level._literal(bundles[6]['metadata']['roll_rad'])})
#define TINYMPC_BANK_MODEL_LEFT_VERY_HIGH_ROLL_RAD ({level._literal(bundles[7]['metadata']['roll_rad'])})
#define TINYMPC_BANK_MODEL_RIGHT_VERY_HIGH_ROLL_RAD ({level._literal(bundles[8]['metadata']['roll_rad'])})
#define TINYMPC_BANK_MODEL_LEFT_MAXIMUM_ROLL_RAD ({level._literal(bundles[9]['metadata']['roll_rad'])})
#define TINYMPC_BANK_MODEL_RIGHT_MAXIMUM_ROLL_RAD ({level._literal(bundles[10]['metadata']['roll_rad'])})

{chr(10).join(blocks)}

typedef struct {{
  int model_id;
  float nominal_roll_rad;
  float nominal_speed_mps;
  float nominal_radius_m;
  float nominal_yaw_rate_rad_s;
  float nominal_pitch_rad;
  float nominal_total_thrust_n;
  float nominal_tangential_drag_accel_mps2;
  const float *A;
  const float *B;
  const float *affine;
  const float *nominal_state;
  const float *nominal_input;
  const float *physical_input;
  const float *Q_diagonal;
  const float *R;
  const float *K;
  const float *P;
  const float *Quu_inv;
  const float *AmBKt;
  const float *coeff_d2p;
  const float *APf;
  const float *BPf;
}} TinyMpcBankedModelData;

static const TinyMpcBankedModelData tinympc_banked_models[TINYMPC_BANK_MODEL_COUNT] = {{
{chr(10).join(table_rows)}
}};

#endif
"""


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def generate(output: Path, provenance_output: Path) -> dict[str, object]:
    bundles = build_bundles()
    generator_sha256 = _sha256(Path(__file__))
    dynamics_path = HERE / "quadrotor_dynamics.py"
    dynamics_sha256 = _sha256(dynamics_path)
    profile_path = APP / "tools" / "crazysim_runtime_profile.py"
    profile_sha256 = _sha256(profile_path)
    adapter_path = HERE / "tinympc_to_crazyflie_adapter.py"
    adapter_sha256 = _sha256(adapter_path)
    header = _render_header(
        bundles, generator_sha256, dynamics_sha256, profile_sha256)
    report_models = []
    for bundle in bundles:
        metadata = bundle["metadata"]
        report_models.append({
            "model_id": bundle["model_id"],
            "name": bundle["name"],
            **metadata,
            "bank_magnitude_deg": abs(math.degrees(metadata["roll_rad"])),
            "nominal_state": bundle["nominal_state"].tolist(),
            "nominal_input_delta_n": bundle["nominal_input"].tolist(),
            "physical_input_n": bundle["physical_input"].tolist(),
            "verification": bundle["verification"],
        })
    report: dict[str, object] = {
        "format": "tinympc-actuator-banked-models-v3",
        "coordinate_system": {
            "level": "legacy absolute yaw-local state; rotor/input relative to hover",
            "banked": (
                "rotating Frenet reference error: yaw-frame position/velocity, "
                "reference-relative Rodrigues attitude, invariant body-rate error, "
                "rotor and motor-input correction relative to physical_input"
            ),
            "bundle_selection": "one complete offline bundle atomically selected per solve",
        },
        "generator_sha256": generator_sha256,
        "dynamics_sha256": dynamics_sha256,
        "adapter_sha256": adapter_sha256,
        "profile_sha256": profile_sha256,
        "plant_profile": plant_profile.provenance(),
        "state_dim": STATE_DIM,
        "input_dim": INPUT_DIM,
        "model_count": len(bundles),
        "radius_m": RADIUS_M,
        "speeds_mps": list(SPEEDS_MPS),
        "bank_cost": {
            "roll_state_index": 3,
            "roll_state_weight": BANK_ROLL_STATE_WEIGHT,
            "pitch_state_index": 4,
            "pitch_state_weight": BANK_PITCH_STATE_WEIGHT,
            "roll_rate_state_index": 9,
            "roll_rate_state_weight": BANK_ROLL_RATE_STATE_WEIGHT,
            "pitch_rate_state_index": 10,
            "pitch_rate_state_weight": BANK_PITCH_RATE_STATE_WEIGHT,
            "lateral_position_state_index": 1,
            "lateral_position_state_weight":
                BANK_LATERAL_POSITION_STATE_WEIGHT,
            "tangential_velocity_state_index": 6,
            "tangential_velocity_state_weight":
                BANK_TANGENTIAL_VELOCITY_STATE_WEIGHT,
            "lateral_velocity_state_index": 7,
            "lateral_velocity_state_weight":
                BANK_LATERAL_VELOCITY_STATE_WEIGHT,
        },
        "models": report_models,
        "header_sha256": hashlib.sha256(header.encode()).hexdigest(),
        "self_check": {
            "all_models_verified": True,
            "deterministic_render": header == _render_header(
                bundles, generator_sha256, dynamics_sha256, profile_sha256),
        },
    }
    if not report["self_check"]["deterministic_render"]:
        raise RuntimeError("header rendering is not deterministic")
    output.parent.mkdir(parents=True, exist_ok=True)
    provenance_output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(header)
    provenance_output.write_text(json.dumps(report, indent=2) + "\n")
    output.chmod(0o644)
    provenance_output.chmod(0o644)
    print(f"generated {output}")
    print(f"generated {provenance_output}")
    print(f"header sha256: {report['header_sha256']}")
    for model in report_models:
        verification = model["verification"]
        print(
            f"{model['model_id']} {model['name']}: "
            f"roll={math.degrees(model['roll_rad']):.6f} deg "
            f"speed={model['speed_mps']:.3f} m/s "
            f"rho_cl={verification['closed_loop_spectral_radius']:.9f} "
            f"cache_error={verification['cache_identity_max_abs']:.3e}")
    return report


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--output", type=Path,
        default=APP / "src" / "tinympc_banked_model_bank.h")
    parser.add_argument(
        "--provenance-output", type=Path,
        default=APP / "src" / "tinympc_banked_model_bank.provenance.json")
    args = parser.parse_args()
    generate(args.output, args.provenance_output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
