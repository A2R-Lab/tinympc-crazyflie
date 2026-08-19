#!/usr/bin/env python3
"""Offline-generated, runtime-indexed LTV controller for stored maneuvers.

The runtime class in this module never differentiates dynamics and never runs a
Riccati recursion.  ``generate_artifact`` performs both operations offline and
writes the resulting trajectory-indexed matrices and feedback caches to NPZ.
"""

from __future__ import annotations

from dataclasses import dataclass
import csv
import json
import math
from pathlib import Path
from typing import Any

import numpy as np


STATE_DIM = 12
INPUT_DIM = 4
MASS_KG = 0.045
GRAVITY_MPS2 = 9.81
INERTIA_KGM2 = np.diag([2.3951e-5, 2.3951e-5, 3.2347e-5])
ARM_OFFSET_M = 0.03535
YAW_TORQUE_RATIO_M = 7.73e-11 / 3.72e-8
MAX_MOTOR_THRUST_N = 3.72e-8 * 2900.0**2
Q = np.diag([100.0, 100.0, 10000.0, 4.0, 4.0, 400.0,
             4.0, 4.0, 4.0, 2.0408163, 2.0408163, 4.0])
R = np.eye(INPUT_DIM) * 724.068315


def _allocation() -> np.ndarray:
    return np.asarray([
        [1.0, 1.0, 1.0, 1.0],
        [-ARM_OFFSET_M, -ARM_OFFSET_M, ARM_OFFSET_M, ARM_OFFSET_M],
        [-ARM_OFFSET_M, ARM_OFFSET_M, ARM_OFFSET_M, -ARM_OFFSET_M],
        [-YAW_TORQUE_RATIO_M, YAW_TORQUE_RATIO_M,
         -YAW_TORQUE_RATIO_M, YAW_TORQUE_RATIO_M],
    ])


ALLOCATION = _allocation()
INERTIA_INV = np.linalg.inv(INERTIA_KGM2)


def _quat_product(left: np.ndarray, right: np.ndarray) -> np.ndarray:
    lw, lx, ly, lz = left
    rw, rx, ry, rz = right
    return np.asarray([
        lw * rw - lx * rx - ly * ry - lz * rz,
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
    ])


def _quat_conjugate(quaternion: np.ndarray) -> np.ndarray:
    result = np.asarray(quaternion, dtype=np.float64).copy()
    result[1:] *= -1.0
    return result


def _normalize_quat(quaternion: np.ndarray) -> np.ndarray:
    quaternion = np.asarray(quaternion, dtype=np.float64)
    norm = float(np.linalg.norm(quaternion))
    if norm < 1.0e-12:
        raise ValueError("zero quaternion")
    return quaternion / norm


def _quat_rotation(quaternion: np.ndarray) -> np.ndarray:
    w, x, y, z = _normalize_quat(quaternion)
    return np.asarray([
        [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
        [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
        [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
    ])


@dataclass(frozen=True)
class StoredReference:
    time_s: np.ndarray
    position_w: np.ndarray
    quaternion_wb: np.ndarray
    velocity_w: np.ndarray
    omega_b: np.ndarray
    motor_thrust_n: np.ndarray

    @classmethod
    def from_csv(cls, path: Path) -> "StoredReference":
        with Path(path).open(newline="") as handle:
            rows = list(csv.DictReader(handle))
        if len(rows) < 2:
            raise ValueError(f"{path} must contain at least two trajectory samples")
        def matrix(names: tuple[str, ...]) -> np.ndarray:
            return np.asarray([[float(row[name]) for name in names] for row in rows])
        time_s = matrix(("t",)).reshape(-1)
        if not np.allclose(np.diff(time_s), np.diff(time_s)[0], atol=1.0e-9):
            raise ValueError("stored LTV generation requires a uniform trajectory sample period")
        return cls(
            time_s=time_s,
            position_w=matrix(("x", "y", "z")),
            quaternion_wb=np.asarray([_normalize_quat(item) for item in matrix(("qw", "qx", "qy", "qz"))]),
            velocity_w=matrix(("vx", "vy", "vz")),
            omega_b=matrix(("wx", "wy", "wz")),
            motor_thrust_n=matrix(tuple(f"motor_{index}_thrust_n" for index in range(4))),
        )

    @property
    def dt_s(self) -> float:
        return float(self.time_s[1] - self.time_s[0])


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
    return np.r_[position, actual_q, velocity, omega]


def _error_from_absolute(reference: StoredReference, index: int, absolute: np.ndarray) -> np.ndarray:
    ref_q = reference.quaternion_wb[index]
    actual_q = _normalize_quat(absolute[3:7])
    ref_rotation = _quat_rotation(ref_q)
    actual_rotation = _quat_rotation(actual_q)
    error_q = _normalize_quat(_quat_product(_quat_conjugate(ref_q), actual_q))
    if error_q[0] < 0.0:
        error_q *= -1.0
    denominator = max(1.0e-9, float(error_q[0]))
    return np.r_[
        ref_rotation.T @ (absolute[0:3] - reference.position_w[index]),
        error_q[1:4] / denominator,
        ref_rotation.T @ (absolute[7:10] - reference.velocity_w[index]),
        absolute[10:13] - actual_rotation.T @ ref_rotation @ reference.omega_b[index],
    ]


def _absolute_derivative(absolute: np.ndarray, motor_thrust_n: np.ndarray) -> np.ndarray:
    quaternion = _normalize_quat(absolute[3:7])
    velocity = absolute[7:10]
    omega = absolute[10:13]
    force_moment = ALLOCATION @ motor_thrust_n
    acceleration = (
        _quat_rotation(quaternion) @ np.asarray([0.0, 0.0, force_moment[0] / MASS_KG])
        - np.asarray([0.0, 0.0, GRAVITY_MPS2])
    )
    quaternion_dot = 0.5 * _quat_product(quaternion, np.r_[0.0, omega])
    omega_dot = INERTIA_INV @ (
        force_moment[1:4] - np.cross(omega, INERTIA_KGM2 @ omega)
    )
    return np.r_[velocity, quaternion_dot, acceleration, omega_dot]


def _rk4_step(absolute: np.ndarray, motor_thrust_n: np.ndarray, dt_s: float) -> np.ndarray:
    k1 = _absolute_derivative(absolute, motor_thrust_n)
    k2 = _absolute_derivative(absolute + 0.5 * dt_s * k1, motor_thrust_n)
    k3 = _absolute_derivative(absolute + 0.5 * dt_s * k2, motor_thrust_n)
    k4 = _absolute_derivative(absolute + dt_s * k3, motor_thrust_n)
    result = absolute + dt_s * (k1 + 2.0 * k2 + 2.0 * k3 + k4) / 6.0
    result[3:7] = _normalize_quat(result[3:7])
    return result


def _error_transition(
    reference: StoredReference, index: int, error: np.ndarray, correction_n: np.ndarray
) -> np.ndarray:
    absolute = _absolute_from_error(reference, index, error)
    physical_input = reference.motor_thrust_n[index] + correction_n
    next_absolute = _rk4_step(absolute, physical_input, reference.dt_s)
    return _error_from_absolute(reference, index + 1, next_absolute)


def _linearize_interval(reference: StoredReference, index: int) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    zero_x = np.zeros(STATE_DIM)
    zero_u = np.zeros(INPUT_DIM)
    affine = _error_transition(reference, index, zero_x, zero_u)
    state_steps = np.asarray([1e-4] * 3 + [2e-5] * 3 + [1e-4] * 3 + [2e-4] * 3)
    input_steps = np.full(INPUT_DIM, 2.0e-5)
    a = np.empty((STATE_DIM, STATE_DIM))
    b = np.empty((STATE_DIM, INPUT_DIM))
    for column, epsilon in enumerate(state_steps):
        perturbation = np.zeros(STATE_DIM)
        perturbation[column] = epsilon
        a[:, column] = (
            _error_transition(reference, index, perturbation, zero_u)
            - _error_transition(reference, index, -perturbation, zero_u)
        ) / (2.0 * epsilon)
    for column, epsilon in enumerate(input_steps):
        perturbation = np.zeros(INPUT_DIM)
        perturbation[column] = epsilon
        b[:, column] = (
            _error_transition(reference, index, zero_x, perturbation)
            - _error_transition(reference, index, zero_x, -perturbation)
        ) / (2.0 * epsilon)
    return a, b, affine


def _terminal_dare(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    p = Q.copy()
    for _ in range(20000):
        hessian = R + b.T @ p @ b
        gain = np.linalg.solve(hessian, b.T @ p @ a)
        next_p = Q + a.T @ p @ (a - b @ gain)
        next_p = 0.5 * (next_p + next_p.T)
        if float(np.max(np.abs(next_p - p))) < 1.0e-10:
            return next_p
        p = next_p
    raise RuntimeError("terminal DARE did not converge")


def generate_artifact(trajectory_csv: Path, output: Path, rho: float = 250.0) -> dict[str, Any]:
    """Perform all linearization and Riccati work and write a runtime artifact."""
    reference = StoredReference.from_csv(Path(trajectory_csv))
    intervals = len(reference.time_s) - 1
    a = np.empty((intervals, STATE_DIM, STATE_DIM))
    b = np.empty((intervals, STATE_DIM, INPUT_DIM))
    affine = np.empty((intervals, STATE_DIM))
    for index in range(intervals):
        a[index], b[index], affine[index] = _linearize_interval(reference, index)

    # One offline time-varying Riccati pass over the stored maneuver. Runtime
    # receives P/K/Hinv arrays and performs no matrix factorization or recursion.
    p = np.empty((intervals + 1, STATE_DIM, STATE_DIM))
    gain = np.empty((intervals, INPUT_DIM, STATE_DIM))
    hessian_inverse = np.empty((intervals, INPUT_DIM, INPUT_DIM))
    p[-1] = _terminal_dare(a[-1], b[-1])
    augmented_r = R + float(rho) * np.eye(INPUT_DIM)
    for index in range(intervals - 1, -1, -1):
        hessian = augmented_r + b[index].T @ p[index + 1] @ b[index]
        hessian_inverse[index] = np.linalg.inv(hessian)
        gain[index] = hessian_inverse[index] @ b[index].T @ p[index + 1] @ a[index]
        p[index] = Q + a[index].T @ p[index + 1] @ (a[index] - b[index] @ gain[index])
        p[index] = 0.5 * (p[index] + p[index].T)

    metadata = {
        "format": 1,
        "trajectory": str(Path(trajectory_csv)),
        "samples": int(len(reference.time_s)),
        "intervals": int(intervals),
        "dt_s": reference.dt_s,
        "rho": float(rho),
        "max_affine_defect": float(np.max(np.abs(affine))),
        "stored_float32_bytes": int(sum(array.size for array in (a, b, affine, p, gain, hessian_inverse)) * 4),
    }
    output = Path(output)
    output.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(
        output,
        metadata=np.asarray(json.dumps(metadata)),
        time_s=reference.time_s.astype(np.float32),
        motor_thrust_n=reference.motor_thrust_n.astype(np.float32),
        A=a.astype(np.float32), B=b.astype(np.float32), affine=affine.astype(np.float32),
        P=p.astype(np.float32), K=gain.astype(np.float32),
        Hinv=hessian_inverse.astype(np.float32),
    )
    metadata["artifact_bytes"] = output.stat().st_size
    return metadata


def render_firmware_header(artifact: Path, output: Path) -> None:
    """Render one artifact as const float arrays for an STM32-selected build."""
    archive = np.load(Path(artifact), allow_pickle=False)
    metadata = json.loads(str(archive["metadata"].item()))

    def literal(value: float) -> str:
        rendered = f"{float(value):.9g}"
        if "." not in rendered and "e" not in rendered:
            rendered += ".0"
        return rendered + "f"

    def array(name: str, values: np.ndarray) -> str:
        flat = np.asarray(values, dtype=np.float32).reshape(-1)
        lines = []
        for start in range(0, len(flat), 8):
            lines.append("    " + ", ".join(literal(value) for value in flat[start:start + 8]) + ",")
        return f"static const float {name}[{len(flat)}] = {{\n" + "\n".join(lines) + "\n};"

    text = "\n\n".join([
        "/* Autogenerated by generate_stored_ltv.py; all linearization and Riccati work is offline. */\n"
        "#ifndef TINYMPC_STORED_LTV_DATA_H\n#define TINYMPC_STORED_LTV_DATA_H\n\n"
        f"#define TINYMPC_STORED_LTV_INTERVALS {int(metadata['intervals'])}\n"
        f"#define TINYMPC_STORED_LTV_DT_S ({literal(metadata['dt_s'])})\n"
        f"#define TINYMPC_STORED_LTV_RHO ({literal(metadata['rho'])})",
        array("tinympc_stored_ltv_A", archive["A"]),
        array("tinympc_stored_ltv_B", archive["B"]),
        array("tinympc_stored_ltv_affine", archive["affine"]),
        array("tinympc_stored_ltv_P", archive["P"]),
        array("tinympc_stored_ltv_K", archive["K"]),
        array("tinympc_stored_ltv_Hinv", archive["Hinv"]),
        "#endif",
    ]) + "\n"
    output = Path(output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(text)


class StoredLtvController:
    """Runtime-only consumer of an offline-generated LTV artifact."""

    def __init__(self, artifact: Path, horizon: int = 20, iterations: int = 5) -> None:
        archive = np.load(Path(artifact), allow_pickle=False)
        self.metadata = json.loads(str(archive["metadata"].item()))
        self.time_s = np.asarray(archive["time_s"], dtype=np.float64)
        self.motor_thrust_n = np.asarray(archive["motor_thrust_n"], dtype=np.float64)
        self.a = np.asarray(archive["A"], dtype=np.float64)
        self.b = np.asarray(archive["B"], dtype=np.float64)
        self.affine = np.asarray(archive["affine"], dtype=np.float64)
        self.p = np.asarray(archive["P"], dtype=np.float64)
        self.gain = np.asarray(archive["K"], dtype=np.float64)
        self.hessian_inverse = np.asarray(archive["Hinv"], dtype=np.float64)
        self.horizon = int(horizon)
        self.iterations = int(iterations)
        self.rho = float(self.metadata["rho"])
        if self.horizon < 2 or self.iterations < 1:
            raise ValueError("stored LTV horizon and iteration count must be positive")
        self._z = np.zeros((self.horizon - 1, INPUT_DIM))
        self._dual = np.zeros((self.horizon - 1, INPUT_DIM))
        self._last_start: int | None = None

    def _indices(self, query_t: float) -> np.ndarray:
        start = int(round((float(query_t) - float(self.time_s[0])) / float(self.metadata["dt_s"])))
        return np.clip(start + np.arange(self.horizon - 1), 0, len(self.a) - 1)

    def solve(self, x0: np.ndarray, query_t: float) -> dict[str, Any]:
        indices = self._indices(query_t)
        start = int(indices[0])
        if self._last_start is not None and start == self._last_start + 1:
            self._z[:-1] = self._z[1:]
            self._z[-1].fill(0.0)
            self._dual[:-1] = self._dual[1:]
            self._dual[-1].fill(0.0)
        elif self._last_start != start:
            self._z.fill(0.0)
            self._dual.fill(0.0)
        self._last_start = start

        lower = -self.motor_thrust_n[indices]
        upper = MAX_MOTOR_THRUST_N - self.motor_thrust_n[indices]
        states = np.zeros((self.horizon, STATE_DIM))
        controls = np.zeros((self.horizon - 1, INPUT_DIM))
        for _ in range(self.iterations):
            linear_p = np.zeros((self.horizon, STATE_DIM))
            feedforward = np.zeros_like(controls)
            for knot in range(self.horizon - 2, -1, -1):
                index = int(indices[knot])
                next_p = self.p[min(index + 1, len(self.p) - 1)]
                r_tilde = -self.rho * (self._z[knot] - self._dual[knot])
                value_gradient = next_p @ self.affine[index] + linear_p[knot + 1]
                feedforward[knot] = self.hessian_inverse[index] @ (
                    r_tilde + self.b[index].T @ value_gradient
                )
                closed_loop = self.a[index] - self.b[index] @ self.gain[index]
                linear_p[knot] = (
                    closed_loop.T @ value_gradient
                    - self.gain[index].T @ r_tilde
                )
            states[0] = np.asarray(x0, dtype=np.float64).reshape(STATE_DIM)
            for knot, index_value in enumerate(indices):
                index = int(index_value)
                controls[knot] = -self.gain[index] @ states[knot] - feedforward[knot]
                states[knot + 1] = (
                    self.a[index] @ states[knot]
                    + self.b[index] @ controls[knot]
                    + self.affine[index]
                )
            self._z = np.minimum(upper, np.maximum(lower, controls + self._dual))
            self._dual += controls - self._z

        return {
            "states": states,
            "controls": controls,
            "physical_baseline_n": self.motor_thrust_n[indices[0]].copy(),
            "iterations": self.iterations,
            "status": 1,
            "pri_res": float(np.max(np.abs(controls - self._z))),
            "dua_res": 0.0,
            "model_id": 100,
            "ltv_start_index": start,
        }
