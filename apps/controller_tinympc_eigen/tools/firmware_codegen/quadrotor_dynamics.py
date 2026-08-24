#!/usr/bin/env python3
"""Shared nonlinear quadrotor dynamics for level-flight model generation."""

from __future__ import annotations
from dataclasses import dataclass
import csv
import json
import math
from pathlib import Path
import sys
from typing import Any
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
import crazysim_runtime_profile as plant_profile

RIGID_BODY_STATE_DIM = 12
MOTOR_STATE_DIM = 4
STATE_DIM = RIGID_BODY_STATE_DIM + MOTOR_STATE_DIM
INPUT_DIM = 4
MASS_KG = plant_profile.MASS_KG
GRAVITY_MPS2 = 9.81
INERTIA_KGM2 = np.diag(plant_profile.INERTIA_DIAGONAL_KGM2)
ARM_OFFSET_M = plant_profile.ARM_OFFSET_M
BODY_LINEAR_DRAG_N_PER_MPS = np.diag(
    plant_profile.BODY_LINEAR_DRAG_DIAGONAL_N_PER_MPS)
YAW_TORQUE_RATIO_M = (
    plant_profile.RPM_TO_TORQUE[2] / plant_profile.RPM_TO_THRUST[2])
MOTOR_TIME_CONSTANT_S = plant_profile.MOTOR_TIME_CONSTANT_S
ROTOR_STATE_SCALE_RPM = plant_profile.ROTOR_STATE_SCALE_RPM
RPM_TO_THRUST = np.asarray(plant_profile.RPM_TO_THRUST)
RPM_TO_TORQUE = np.asarray(plant_profile.RPM_TO_TORQUE)

def _allocation() -> np.ndarray:
    return np.asarray([[1.0, 1.0, 1.0, 1.0], [-ARM_OFFSET_M, -ARM_OFFSET_M, ARM_OFFSET_M, ARM_OFFSET_M], [-ARM_OFFSET_M, ARM_OFFSET_M, ARM_OFFSET_M, -ARM_OFFSET_M], [-YAW_TORQUE_RATIO_M, YAW_TORQUE_RATIO_M, -YAW_TORQUE_RATIO_M, YAW_TORQUE_RATIO_M]])
ALLOCATION = _allocation()
INERTIA_INV = np.linalg.inv(INERTIA_KGM2)

def _quat_product(left: np.ndarray, right: np.ndarray) -> np.ndarray:
    lw, lx, ly, lz = left
    rw, rx, ry, rz = right
    return np.asarray([lw * rw - lx * rx - ly * ry - lz * rz, lw * rx + lx * rw + ly * rz - lz * ry, lw * ry - lx * rz + ly * rw + lz * rx, lw * rz + lx * ry - ly * rx + lz * rw])

def _quat_conjugate(quaternion: np.ndarray) -> np.ndarray:
    result = np.asarray(quaternion, dtype=np.float64).copy()
    result[1:] *= -1.0
    return result

def _normalize_quat(quaternion: np.ndarray) -> np.ndarray:
    quaternion = np.asarray(quaternion, dtype=np.float64)
    norm = float(np.linalg.norm(quaternion))
    if norm < 1e-12:
        raise ValueError('zero quaternion')
    return quaternion / norm

def _quat_rotation(quaternion: np.ndarray) -> np.ndarray:
    w, x, y, z = _normalize_quat(quaternion)
    return np.asarray([[1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)], [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)], [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)]])

@dataclass(frozen=True)
class StoredReference:
    time_s: np.ndarray
    position_w: np.ndarray
    quaternion_wb: np.ndarray
    velocity_w: np.ndarray
    omega_b: np.ndarray
    motor_thrust_n: np.ndarray

    @classmethod
    def from_csv(cls, path: Path) -> 'StoredReference':
        with Path(path).open(newline='') as handle:
            rows = list(csv.DictReader(handle))
        if len(rows) < 2:
            raise ValueError(f'{path} must contain at least two trajectory samples')

        def matrix(names: tuple[str, ...]) -> np.ndarray:
            return np.asarray([[float(row[name]) for name in names] for row in rows])
        time_s = matrix(('t',)).reshape(-1)
        if not np.allclose(np.diff(time_s), np.diff(time_s)[0], atol=1e-09):
            raise ValueError('model generation requires a uniform trajectory sample period')
        return cls(time_s=time_s, position_w=matrix(('x', 'y', 'z')), quaternion_wb=np.asarray([_normalize_quat(item) for item in matrix(('qw', 'qx', 'qy', 'qz'))]), velocity_w=matrix(('vx', 'vy', 'vz')), omega_b=matrix(('wx', 'wy', 'wz')), motor_thrust_n=matrix(tuple((f'motor_{index}_thrust_n' for index in range(4)))))

    @property
    def dt_s(self) -> float:
        return float(self.time_s[1] - self.time_s[0])

    @property
    def motor_command_n(self) -> np.ndarray:
        """Nominal thrust command that makes rotor RPM hit the next reference."""
        decay = math.exp(-self.dt_s / MOTOR_TIME_CONSTANT_S)
        rotor_rpm = self.motor_state * ROTOR_STATE_SCALE_RPM
        command_rpm = np.empty_like(rotor_rpm)
        command_rpm[:-1] = (rotor_rpm[1:] - decay * rotor_rpm[:-1]) / (1.0 - decay)
        command_rpm[-1] = rotor_rpm[-1]
        return _rpm_polynomial(command_rpm, RPM_TO_THRUST)

    @property
    def motor_state(self) -> np.ndarray:
        return _thrust_to_rpm(self.motor_thrust_n) / ROTOR_STATE_SCALE_RPM

def _rpm_polynomial(rpm: np.ndarray, coefficients: np.ndarray) -> np.ndarray:
    return coefficients[0] + coefficients[1] * rpm + coefficients[2] * rpm * rpm

def _thrust_to_rpm(thrust_n: np.ndarray) -> np.ndarray:
    a, b, c = RPM_TO_THRUST
    discriminant = b * b - 4.0 * c * (a - np.asarray(thrust_n))
    return (-b + np.sqrt(np.maximum(discriminant, 0.0))) / (2.0 * c)

def _absolute_from_error(reference: StoredReference, index: int, error: np.ndarray) -> np.ndarray:
    ref_q = reference.quaternion_wb[index]
    ref_rotation = _quat_rotation(ref_q)
    rodrigues = np.asarray(error[3:6], dtype=np.float64)
    error_q = _normalize_quat(np.r_[1.0, rodrigues])
    actual_q = _normalize_quat(_quat_product(ref_q, error_q))
    actual_rotation = _quat_rotation(actual_q)
    position = reference.position_w[index] + ref_rotation @ error[0:3]
    velocity = reference.velocity_w[index] + ref_rotation @ error[6:9]
    omega = error[9:12] + actual_rotation.T @ ref_rotation @ reference.omega_b[index]
    motor_state = reference.motor_state[index] + error[12:16]
    return np.r_[position, actual_q, velocity, omega, motor_state]

def _error_from_absolute(reference: StoredReference, index: int, absolute: np.ndarray) -> np.ndarray:
    ref_q = reference.quaternion_wb[index]
    actual_q = _normalize_quat(absolute[3:7])
    ref_rotation = _quat_rotation(ref_q)
    actual_rotation = _quat_rotation(actual_q)
    error_q = _normalize_quat(_quat_product(_quat_conjugate(ref_q), actual_q))
    if error_q[0] < 0.0:
        error_q *= -1.0
    denominator = max(1e-09, float(error_q[0]))
    return np.r_[ref_rotation.T @ (absolute[0:3] - reference.position_w[index]), error_q[1:4] / denominator, ref_rotation.T @ (absolute[7:10] - reference.velocity_w[index]), absolute[10:13] - actual_rotation.T @ ref_rotation @ reference.omega_b[index], absolute[13:17] - reference.motor_state[index]]

def _absolute_derivative(absolute: np.ndarray, motor_command_n: np.ndarray) -> np.ndarray:
    quaternion = _normalize_quat(absolute[3:7])
    rotation = _quat_rotation(quaternion)
    velocity = absolute[7:10]
    omega = absolute[10:13]
    motor_rpm = absolute[13:17] * ROTOR_STATE_SCALE_RPM
    motor_thrust_n = _rpm_polynomial(motor_rpm, RPM_TO_THRUST)
    motor_torque_nm = _rpm_polynomial(motor_rpm, RPM_TO_TORQUE)
    force_moment = ALLOCATION @ motor_thrust_n
    force_moment[3] = np.asarray([-1.0, 1.0, -1.0, 1.0]) @ motor_torque_nm
    velocity_body = rotation.T @ velocity
    force_body = np.asarray([0.0, 0.0, force_moment[0]]) + (
        BODY_LINEAR_DRAG_N_PER_MPS @ velocity_body)
    acceleration = rotation @ (force_body / MASS_KG) - np.asarray([0.0, 0.0, GRAVITY_MPS2])
    quaternion_dot = 0.5 * _quat_product(quaternion, np.r_[0.0, omega])
    omega_dot = INERTIA_INV @ (force_moment[1:4] - np.cross(omega, INERTIA_KGM2 @ omega))
    command_rpm_state = _thrust_to_rpm(np.asarray(motor_command_n, dtype=np.float64)) / ROTOR_STATE_SCALE_RPM
    motor_state_dot = (command_rpm_state - absolute[13:17]) / MOTOR_TIME_CONSTANT_S
    return np.r_[velocity, quaternion_dot, acceleration, omega_dot, motor_state_dot]

def _rk4_step(absolute: np.ndarray, motor_command_n: np.ndarray, dt_s: float) -> np.ndarray:
    k1 = _absolute_derivative(absolute, motor_command_n)
    k2 = _absolute_derivative(absolute + 0.5 * dt_s * k1, motor_command_n)
    k3 = _absolute_derivative(absolute + 0.5 * dt_s * k2, motor_command_n)
    k4 = _absolute_derivative(absolute + dt_s * k3, motor_command_n)
    result = absolute + dt_s * (k1 + 2.0 * k2 + 2.0 * k3 + k4) / 6.0
    result[3:7] = _normalize_quat(result[3:7])
    return result

def _error_transition(reference: StoredReference, index: int, error: np.ndarray, correction_n: np.ndarray) -> np.ndarray:
    absolute = _absolute_from_error(reference, index, error)
    physical_command = reference.motor_command_n[index] + correction_n
    next_absolute = _rk4_step(absolute, physical_command, reference.dt_s)
    return _error_from_absolute(reference, index + 1, next_absolute)

def _linearize_interval(reference: StoredReference, index: int) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    zero_x = np.zeros(STATE_DIM)
    zero_u = np.zeros(INPUT_DIM)
    affine = _error_transition(reference, index, zero_x, zero_u)
    state_steps = np.asarray([0.0001] * 3 + [2e-05] * 3 + [0.0001] * 3 + [0.0002] * 3 + [2e-05] * MOTOR_STATE_DIM)
    input_steps = np.full(INPUT_DIM, 2e-05)
    a = np.empty((STATE_DIM, STATE_DIM))
    b = np.empty((STATE_DIM, INPUT_DIM))
    for column, epsilon in enumerate(state_steps):
        perturbation = np.zeros(STATE_DIM)
        perturbation[column] = epsilon
        a[:, column] = (_error_transition(reference, index, perturbation, zero_u) - _error_transition(reference, index, -perturbation, zero_u)) / (2.0 * epsilon)
    for column, epsilon in enumerate(input_steps):
        perturbation = np.zeros(INPUT_DIM)
        perturbation[column] = epsilon
        b[:, column] = (_error_transition(reference, index, zero_x, perturbation) - _error_transition(reference, index, zero_x, -perturbation)) / (2.0 * epsilon)
    return (a, b, affine)
