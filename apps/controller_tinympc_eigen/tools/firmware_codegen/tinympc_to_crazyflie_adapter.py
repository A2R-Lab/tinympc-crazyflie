#!/usr/bin/env python3
"""Generates firmware compatible code from TinyMPC."""

from __future__ import annotations

import ctypes
import subprocess
import sys
import tempfile
from dataclasses import dataclass
from pathlib import Path


@dataclass(frozen=True)
class CompileTimeProblem:
    """Default dimensions, knots, dt, iteration count, half-spaces, and solve rates for the firmware."""

    state_dim: int = 12
    input_dim: int = 4
    horizon_knots: int = 20
    model_dt_s: float = 0.04
    solve_rate_hz: int = 5
    admm_max_iterations: int = 5
    scalar_c_type: str = "float"
    max_active_state_halfspaces: int = 1
    constrained_horizon_knots: int = 10
    crazyflie: str = "brushless"
    deck: str = "aideck"
    propeller_guards: bool = True
    admm_rho: float = 63.0


PROBLEM = CompileTimeProblem()


def vehicle_mass(problem: CompileTimeProblem) -> float:
    base_masses = {"brushless": 0.039, "brushed": 0.028}
    deck_masses = {"none": 0.0, "aideck": 0.0044, "flowdeck": 0.0016, "both": 0.006}
    try:
        return base_masses[problem.crazyflie] + deck_masses[problem.deck]
    except KeyError as error:
        raise ValueError(f"unsupported vehicle configuration: {error.args[0]}") from error


def build_continuous_model(problem: CompileTimeProblem):
    # Brushless: Busetto et al., "Nonlinear System Identification Nano-drone
    # Benchmark," arXiv:2512.14450 (2025), Section 4.1 and Equations (5), (21), and (22).
    # Brushed: Förster, "System Identification of the Crazyflie 2.0 Nano
    # Quadrocopter," ETH Zürich Bachelor Thesis, 2015, Appendix A.
    # Deck masses and dimensions: Bitcraze AI-deck 1.1 and Flow deck v2 datasheets.
    if problem.state_dim != 12 or problem.input_dim != 4:
        raise ValueError("the Crazyflie model requires 12 states and 4 inputs")

    import autograd.numpy as np

    gravity = 9.81
    deck_properties = {
        "aideck": (0.0044, np.array([0.030, 0.052, 0.008]), np.array([0.0, 0.0, 0.010])),
        "flowdeck": (0.0016, np.array([0.021, 0.028, 0.004]), np.array([0.0, 0.0, -0.007])),
    }

    def rectangular_inertia(mass, dimensions):
        width, depth, height = dimensions
        return mass / 12.0 * np.array(
            [
                [depth * depth + height * height, 0.0, 0.0],
                [0.0, width * width + height * height, 0.0],
                [0.0, 0.0, width * width + depth * depth],
            ]
        )

    def shifted_inertia(mass, inertia_at_center, offset):
        return inertia_at_center + mass * (
            np.dot(offset, offset) * np.eye(3) - np.outer(offset, offset)
        )

    selected_decks = {
        "none": (),
        "aideck": ("aideck",),
        "flowdeck": ("flowdeck",),
        "both": ("aideck", "flowdeck"),
    }.get(problem.deck)
    if selected_decks is None:
        raise ValueError(f"unsupported deck selection: {problem.deck}")

    match problem.crazyflie:
        case "brushless":
            both_deck_mass = 0.045
            both_deck_inertia = np.array(
                [
                    [2.3951e-5, 0.0, 0.0],
                    [0.0, 2.3951e-5, 0.0],
                    [0.0, 0.0, 3.2347e-5],
                ]
            )
            arm_offset = 0.03535
            thrust_coefficient = 3.72e-8
            yaw_torque_coefficient = 7.73e-11
            thrust_to_torque = yaw_torque_coefficient / thrust_coefficient
            both_decks = ("aideck", "flowdeck")
            base_mass = both_deck_mass - sum(deck_properties[name][0] for name in both_decks)
            both_center = sum(
                (deck_properties[name][0] * deck_properties[name][2] for name in both_decks),
                np.zeros(3),
            ) / both_deck_mass
            base_inertia = both_deck_inertia - shifted_inertia(
                base_mass, np.zeros((3, 3)), -both_center
            )
            for name in both_decks:
                deck_mass, dimensions, position = deck_properties[name]
                base_inertia -= shifted_inertia(
                    deck_mass,
                    rectangular_inertia(deck_mass, dimensions),
                    position - both_center,
                )
        case "brushed":
            base_mass = 0.028
            base_inertia = np.array(
                [
                    [16.571710e-6, 0.830806e-6, 0.718277e-6],
                    [0.830806e-6, 16.655602e-6, 1.800197e-6],
                    [0.718277e-6, 1.800197e-6, 29.261652e-6],
                ]
            )
            arm_offset = 0.046 / np.sqrt(2.0)
            thrust_to_torque = 0.0008
        case _:
            raise ValueError(f"unsupported Crazyflie: {problem.crazyflie}")

    mass = base_mass + sum(deck_properties[name][0] for name in selected_decks)
    center_of_mass = sum(
        (deck_properties[name][0] * deck_properties[name][2] for name in selected_decks),
        np.zeros(3),
    ) / mass
    inertia = shifted_inertia(base_mass, base_inertia, -center_of_mass)
    for name in selected_decks:
        deck_mass, dimensions, position = deck_properties[name]
        inertia += shifted_inertia(
            deck_mass,
            rectangular_inertia(deck_mass, dimensions),
            position - center_of_mass,
        )

    inertia_inverse = np.linalg.inv(inertia)

    def dynamics(x, u):
        rodrigues = x[3:6]
        velocity = x[6:9]
        angular_velocity = x[9:12]

        quaternion = np.concatenate((np.array([1.0]), rodrigues))
        quaternion = quaternion / np.sqrt(np.dot(quaternion, quaternion))
        qw, qx, qy, qz = quaternion
        rotation = np.array(
            [
                [1.0 - 2.0 * (qy * qy + qz * qz), 2.0 * (qx * qy - qw * qz), 2.0 * (qx * qz + qw * qy)],
                [2.0 * (qx * qy + qw * qz), 1.0 - 2.0 * (qx * qx + qz * qz), 2.0 * (qy * qz - qw * qx)],
                [2.0 * (qx * qz - qw * qy), 2.0 * (qy * qz + qw * qx), 1.0 - 2.0 * (qx * qx + qy * qy)],
            ]
        )

        motor_thrust = u
        yaw_torque = thrust_to_torque * (
            -motor_thrust[0]
            + motor_thrust[1]
            - motor_thrust[2]
            + motor_thrust[3]
        )
        total_thrust = np.sum(motor_thrust)
        acceleration = np.array([0.0, 0.0, -gravity]) + (
            rotation @ np.array([0.0, 0.0, total_thrust / mass])
        )

        roll_torque = arm_offset * (-motor_thrust[0] - motor_thrust[1] + motor_thrust[2] + motor_thrust[3])
        pitch_torque = arm_offset * (-motor_thrust[0] + motor_thrust[1] + motor_thrust[2] - motor_thrust[3])
        torque = np.array([roll_torque, pitch_torque, yaw_torque])
        angular_acceleration = inertia_inverse @ (
            torque - np.cross(angular_velocity, inertia @ angular_velocity)
        )

        rodrigues_dot = 0.5 * (
            angular_velocity
            + np.cross(rodrigues, angular_velocity)
            + rodrigues * np.dot(rodrigues, angular_velocity)
        )
        return np.concatenate(
            (velocity, rodrigues_dot, acceleration, angular_acceleration)
        )

    return dynamics


def rk4(dynamics, x, u, dt):
    f1 = dt * dynamics(x, u)
    f2 = dt * dynamics(x + f1 / 2.0, u)
    f3 = dt * dynamics(x + f2 / 2.0, u)
    f4 = dt * dynamics(x + f3, u)
    return x + (f1 + 2.0 * f2 + 2.0 * f3 + f4) / 6.0


def linearize_discrete_model(problem: CompileTimeProblem):
    import autograd as AG
    import autograd.numpy as np

    dynamics = build_continuous_model(problem)
    xgoal = np.zeros(problem.state_dim)
    hover_input = vehicle_mass(problem) * 9.81 / problem.input_dim
    ugoal = np.full(problem.input_dim, hover_input)
    A = AG.jacobian(lambda x_: rk4(dynamics, x_, ugoal, problem.model_dt_s))(xgoal)
    B = AG.jacobian(
        lambda delta_u: rk4(dynamics, xgoal, ugoal + delta_u, problem.model_dt_s)
    )(np.zeros(problem.input_dim))
    f = rk4(dynamics, xgoal, ugoal, problem.model_dt_s)
    return A, B, f


def build_cost_matrices(problem: CompileTimeProblem):
    if problem.state_dim != 12 or problem.input_dim != 4:
        raise ValueError("the Crazyflie cost requires 12 states and 4 inputs")

    import autograd.numpy as np

    Q = np.diag(
        np.array(
            [
                100.0,
                100.0,
                10000.0,
                4.0,
                4.0,
                400.0,
                4.0,
                4.0,
                4.0,
                2.0408163,
                2.0408163,
                4.0,
            ]
        )
    )
    _, _, physical_hover = build_input_bounds_and_reference(problem)
    match problem.crazyflie:
        case "brushless":
            hover_command = np.sqrt(physical_hover / (3.72e-8 * 2900.0**2))
            thrust_slope = 2.0 * 3.72e-8 * 2900.0**2 * hover_command
        case "brushed":
            a = 2.130295e-11
            b = 1.032633e-6
            c = 5.484560e-4 - physical_hover
            raw_hover_command = (-b + np.sqrt(b * b - 4.0 * a * c)) / (2.0 * a)
            thrust_slope = (2.0 * a * raw_hover_command + b) * 65535.0
        case _:
            raise ValueError(f"unsupported Crazyflie: {problem.crazyflie}")

    # Preserve the legacy normalized-command cost after changing solver inputs
    # to thrust deviations in Newtons.
    R = np.diag(100.0 / thrust_slope**2)
    return Q, R


def build_state_bounds(problem: CompileTimeProblem):
    if problem.state_dim != 12:
        raise ValueError("the Crazyflie state bounds require 12 states")

    import autograd.numpy as np

    unbounded = 1e6
    lower = np.array(
        [
            -5.0,
            -5.0,
            0.05,
            -0.5,
            -0.5,
            -unbounded,
            -2.5,
            -2.5,
            -1.5,
            -10.0,
            -10.0,
            -10.0,
        ]
    )
    upper = np.array(
        [
            5.0,
            5.0,
            3.0,
            0.5,
            0.5,
            unbounded,
            2.5,
            2.5,
            1.5,
            10.0,
            10.0,
            10.0,
        ]
    )
    return lower, upper


def build_input_bounds_and_reference(problem: CompileTimeProblem):
    import numpy as np

    match problem.crazyflie:
        case "brushless":
            maximum_motor_thrust = 3.72e-8 * 2900.0**2
        case "brushed":
            maximum_command = 65535.0
            maximum_motor_thrust = (
                2.130295e-11 * maximum_command**2
                + 1.032633e-6 * maximum_command
                + 5.484560e-4
            )
        case _:
            raise ValueError(f"unsupported Crazyflie: {problem.crazyflie}")

    lower = np.zeros(problem.input_dim)
    upper = np.full(problem.input_dim, maximum_motor_thrust)
    hover = np.full(
        problem.input_dim,
        vehicle_mass(problem) * 9.81 / problem.input_dim,
    )
    return lower, upper, hover


def build_constraint_time_mask(problem: CompileTimeProblem):
    import numpy as np

    mask = np.zeros(problem.horizon_knots, dtype=np.uint8)
    mask[: problem.constrained_horizon_knots] = 1
    return mask


def build_upstream_cache(problem: CompileTimeProblem, A, B, f, Q, R):
    import numpy as np

    repository_root = Path(__file__).resolve().parents[4]
    upstream_root = (repository_root / "TinyMPC").resolve()
    bridge_source = Path(__file__).with_name("tinympc_cpp_bridge.cpp")
    required = (
        bridge_source,
        upstream_root / "src/tinympc/tiny_api.cpp",
        upstream_root / "src/tinympc/admm.cpp",
        upstream_root / "src/tinympc/rho_benchmark.cpp",
        upstream_root / "src/tinympc/tiny_api.hpp",
    )
    missing = [str(path) for path in required if not path.exists()]
    if missing:
        raise FileNotFoundError("missing upstream TinyMPC files: " + ", ".join(missing))

    with tempfile.TemporaryDirectory(prefix="crazyflie-tinympc-") as temporary_directory:
        library_suffix = ".dylib" if sys.platform == "darwin" else ".so"
        bridge_library = Path(temporary_directory) / f"cache_bridge{library_suffix}"
        command = [
            "c++",
            "-std=c++17",
            "-O2",
            "-fPIC",
            "-shared",
            str(bridge_source),
            str(upstream_root / "src/tinympc/tiny_api.cpp"),
            str(upstream_root / "src/tinympc/admm.cpp"),
            str(upstream_root / "src/tinympc/rho_benchmark.cpp"),
            "-I",
            str(upstream_root / "src"),
            "-I",
            str(upstream_root / "include/Eigen"),
            "-o",
            str(bridge_library),
        ]
        completed = subprocess.run(command, capture_output=True, text=True)
        if completed.returncode != 0:
            raise RuntimeError(
                "failed to build the upstream TinyMPC cache bridge:\n"
                + completed.stdout
                + completed.stderr
            )

        library = ctypes.CDLL(str(bridge_library))
        precompute = library.crazyflie_tinympc_precompute
        pointer = ctypes.POINTER(ctypes.c_double)
        precompute.argtypes = [
            pointer,
            pointer,
            pointer,
            pointer,
            pointer,
            ctypes.c_double,
            ctypes.c_int,
            ctypes.c_int,
            pointer,
            pointer,
            pointer,
            pointer,
            pointer,
            pointer,
            pointer,
            pointer,
        ]
        precompute.restype = ctypes.c_int

        arrays = {
            "Kinf": np.empty((problem.input_dim, problem.state_dim), dtype=np.float64),
            "Pinf": np.empty((problem.state_dim, problem.state_dim), dtype=np.float64),
            "Quu_inv": np.empty((problem.input_dim, problem.input_dim), dtype=np.float64),
            "AmBKt": np.empty((problem.state_dim, problem.state_dim), dtype=np.float64),
            "APf": np.empty(problem.state_dim, dtype=np.float64),
            "BPf": np.empty(problem.input_dim, dtype=np.float64),
            "C1": np.empty((problem.input_dim, problem.input_dim), dtype=np.float64),
            "C2": np.empty((problem.state_dim, problem.state_dim), dtype=np.float64),
        }

        inputs = [
            np.ascontiguousarray(value, dtype=np.float64)
            for value in (A, B, f, np.diag(Q), np.diag(R))
        ]
        status = precompute(
            *(value.ctypes.data_as(pointer) for value in inputs),
            problem.admm_rho,
            problem.state_dim,
            problem.input_dim,
            *(value.ctypes.data_as(pointer) for value in arrays.values()),
        )
        if status != 0:
            raise RuntimeError(f"upstream TinyMPC cache generation failed with status {status}")
        return {name: value.copy() for name, value in arrays.items()}


def format_c_array(name: str, values, c_type: str = "float", constant: bool = True) -> str:
    import numpy as np

    flattened = np.asarray(values).reshape(-1)
    qualifiers = "static const" if constant else "static"
    lines = [f"{qualifiers} {c_type} {name}[{flattened.size}] = {{"]
    for start in range(0, flattened.size, 6):
        entries = []
        for value in flattened[start : start + 6]:
            if c_type == "float":
                literal = f"{float(value):.9g}"
                if "." not in literal and "e" not in literal.lower():
                    literal += ".0"
                entries.append(literal + "f")
            else:
                entries.append(str(int(value)))
        lines.append("    " + ", ".join(entries) + ",")
    lines.append("};")
    return "\n".join(lines)


def emit_firmware_header(problem: CompileTimeProblem, A, B, f, Q, R, cache) -> Path:
    import numpy as np

    state_lower, state_upper = build_state_bounds(problem)
    physical_lower, physical_upper, physical_hover = build_input_bounds_and_reference(problem)
    input_lower = physical_lower - physical_hover
    input_upper = physical_upper - physical_hover
    hover_reference = np.zeros(problem.input_dim)
    state_lower_horizon = np.repeat(state_lower[:, None], problem.horizon_knots, axis=1)
    state_upper_horizon = np.repeat(state_upper[:, None], problem.horizon_knots, axis=1)
    input_lower_horizon = np.repeat(input_lower[:, None], problem.horizon_knots - 1, axis=1)
    input_upper_horizon = np.repeat(input_upper[:, None], problem.horizon_knots - 1, axis=1)
    constraint_mask = build_constraint_time_mask(problem)
    tv_state_a = np.zeros(
        (
            problem.horizon_knots,
            problem.max_active_state_halfspaces,
            problem.state_dim,
        )
    )
    tv_state_a[:, :, 0] = 1.0
    tv_state_b = np.full(
        (problem.horizon_knots, problem.max_active_state_halfspaces),
        1e6,
    )
    tv_state_enabled = np.zeros(
        (problem.horizon_knots, problem.max_active_state_halfspaces),
        dtype=np.uint8,
    )

    # The embedded fixed-size fork retains this algebraically-zero Riccati
    # compatibility term in its backward pass.
    coeff_d2p = cache["Kinf"].T @ R - cache["AmBKt"] @ cache["Pinf"] @ B

    arrays = [
        format_c_array("tinympc_generated_A", A),
        format_c_array("tinympc_generated_B", B),
        format_c_array("tinympc_generated_f", f),
        format_c_array("tinympc_generated_Q_diagonal", np.diag(Q)),
        format_c_array("tinympc_generated_R_diagonal", np.diag(R)),
        format_c_array("tinympc_generated_state_lower", state_lower_horizon),
        format_c_array("tinympc_generated_state_upper", state_upper_horizon),
        format_c_array("tinympc_generated_input_lower", input_lower_horizon),
        format_c_array("tinympc_generated_input_upper", input_upper_horizon),
        format_c_array("tinympc_generated_hover_reference", hover_reference),
        format_c_array("tinympc_generated_physical_hover_thrust", physical_hover),
        format_c_array("tinympc_generated_constraint_knot_mask", constraint_mask, "uint8_t"),
        format_c_array("tinympc_generated_coeff_d2p", coeff_d2p),
        format_c_array("tinympc_generated_tv_state_a", tv_state_a, constant=False),
        format_c_array("tinympc_generated_tv_state_b", tv_state_b, constant=False),
        format_c_array(
            "tinympc_generated_tv_state_enabled",
            tv_state_enabled,
            "uint8_t",
            constant=False,
        ),
    ]
    arrays.extend(
        format_c_array(f"tinympc_generated_{name}", value)
        for name, value in cache.items()
    )

    if problem.crazyflie == "brushless":
        actuator_body = """\
  float command = sqrtf(thrust_newtons / 3.72e-8f) / 2900.0f;
  return command > 1.0f ? 1.0f : command;
"""
        inverse_actuator_body = """\
  const float rotor_speed = 2900.0f * command;
  return 3.72e-8f * rotor_speed * rotor_speed;
"""
    else:
        actuator_body = """\
  const float a = 2.130295e-11f;
  const float b = 1.032633e-6f;
  const float c = 5.484560e-4f - thrust_newtons;
  const float discriminant = b * b - 4.0f * a * c;
  if (discriminant <= 0.0f) return 0.0f;
  float command = (-b + sqrtf(discriminant)) / (2.0f * a * 65535.0f);
  if (command <= 0.0f) return 0.0f;
  return command > 1.0f ? 1.0f : command;
"""
        inverse_actuator_body = """\
  const float raw_command = 65535.0f * command;
  return 2.130295e-11f * raw_command * raw_command
      + 1.032633e-6f * raw_command + 5.484560e-4f;
"""

    content = f"""\
/* Autogenerated by tinympc_to_crazyflie_adapter.py. */
#ifndef TINYMPC_GENERATED_PARAMS_H
#define TINYMPC_GENERATED_PARAMS_H

#include <math.h>
#include <stdint.h>

#define TINYMPC_GENERATED_STATE_DIM {problem.state_dim}
#define TINYMPC_GENERATED_INPUT_DIM {problem.input_dim}
#define TINYMPC_GENERATED_HORIZON_KNOTS {problem.horizon_knots}
#define TINYMPC_GENERATED_MODEL_DT_S ((float){problem.model_dt_s:.9g})
#define TINYMPC_GENERATED_SOLVE_RATE_HZ {problem.solve_rate_hz}
#define TINYMPC_GENERATED_ADMM_MAX_ITERATIONS {problem.admm_max_iterations}
#define TINYMPC_GENERATED_ADMM_RHO ((float){problem.admm_rho:.9g})
#define TINYMPC_GENERATED_MAX_STATE_HALFSPACES {problem.max_active_state_halfspaces}
#define TINYMPC_GENERATED_CONSTRAINED_HORIZON_KNOTS {problem.constrained_horizon_knots}
#define TINYMPC_GENERATED_IS_BRUSHLESS {1 if problem.crazyflie == "brushless" else 0}
#define TINYMPC_GENERATED_HAS_AIDECK {1 if problem.deck in ("aideck", "both") else 0}
#define TINYMPC_GENERATED_HAS_FLOWDECK {1 if problem.deck in ("flowdeck", "both") else 0}
#define TINYMPC_GENERATED_HAS_PROPELLER_GUARDS {1 if problem.propeller_guards else 0}

{chr(10).join(arrays)}

static inline void tinympc_generated_clear_halfspaces(void) {{
  for (int knot = 0; knot < TINYMPC_GENERATED_HORIZON_KNOTS; ++knot) {{
    for (int slot = 0; slot < TINYMPC_GENERATED_MAX_STATE_HALFSPACES; ++slot) {{
      const int constraint = knot * TINYMPC_GENERATED_MAX_STATE_HALFSPACES + slot;
      tinympc_generated_tv_state_b[constraint] = 1e6f;
      tinympc_generated_tv_state_enabled[constraint] = 0;
      for (int state = 0; state < TINYMPC_GENERATED_STATE_DIM; ++state) {{
        tinympc_generated_tv_state_a[
            constraint * TINYMPC_GENERATED_STATE_DIM + state] = 0.0f;
      }}
      tinympc_generated_tv_state_a[
          constraint * TINYMPC_GENERATED_STATE_DIM] = 1.0f;
    }}
  }}
}}

static inline int tinympc_generated_set_position_halfspace(
    int knot, int slot, float ax, float ay, float az, float boundary) {{
  if (knot < 0 || knot >= TINYMPC_GENERATED_CONSTRAINED_HORIZON_KNOTS ||
      slot < 0 || slot >= TINYMPC_GENERATED_MAX_STATE_HALFSPACES) return 0;
  if (ax * ax + ay * ay + az * az <= 1e-12f) return 0;
  const int constraint = knot * TINYMPC_GENERATED_MAX_STATE_HALFSPACES + slot;
  float* normal = &tinympc_generated_tv_state_a[
      constraint * TINYMPC_GENERATED_STATE_DIM];
  for (int state = 0; state < TINYMPC_GENERATED_STATE_DIM; ++state) normal[state] = 0.0f;
  normal[0] = ax;
  normal[1] = ay;
  normal[2] = az;
  tinympc_generated_tv_state_b[constraint] = boundary;
  tinympc_generated_tv_state_enabled[constraint] = 1;
  return 1;
}}

static inline float tinympc_generated_thrust_to_normalized_command(float thrust_newtons) {{
  if (thrust_newtons <= 0.0f) return 0.0f;
{actuator_body}}}

static inline float tinympc_generated_normalized_command_to_thrust(float command) {{
  if (command <= 0.0f) return 0.0f;
  if (command > 1.0f) command = 1.0f;
{inverse_actuator_body}}}

#endif
"""

    output_path = Path(__file__).resolve().parents[2] / "src/tinympc_generated_params.h"
    output_path.write_text(content)
    return output_path


def validate_generated_problem(problem: CompileTimeProblem, A, B, f, Q, R, cache) -> None:
    import numpy as np

    physical_lower, physical_upper, physical_hover = build_input_bounds_and_reference(problem)
    input_lower = physical_lower - physical_hover
    input_upper = physical_upper - physical_hover
    hover_reference = np.zeros(problem.input_dim)
    hover_next = A @ np.zeros(problem.state_dim) + B @ hover_reference + f
    if np.linalg.norm(hover_next, ord=np.inf) > 1e-8:
        raise ValueError("the generated affine model does not preserve hover")
    if not np.all(input_lower <= hover_reference) or not np.all(hover_reference <= input_upper):
        raise ValueError("zero hover deviation lies outside the generated input bounds")
    if not np.all(physical_lower <= physical_hover) or not np.all(physical_hover <= physical_upper):
        raise ValueError("physical hover thrust lies outside the actuator bounds")
    state_lower, state_upper = build_state_bounds(problem)
    if not np.all(state_lower < state_upper):
        raise ValueError("every lower state bound must be below its upper bound")
    if np.min(np.linalg.eigvalsh(Q)) < 0.0 or np.min(np.linalg.eigvalsh(R)) <= 0.0:
        raise ValueError("Q must be positive semidefinite and R must be positive definite")
    expected_shapes = {
        "Kinf": (problem.input_dim, problem.state_dim),
        "Pinf": (problem.state_dim, problem.state_dim),
        "Quu_inv": (problem.input_dim, problem.input_dim),
        "AmBKt": (problem.state_dim, problem.state_dim),
        "APf": (problem.state_dim,),
        "BPf": (problem.input_dim,),
        "C1": (problem.input_dim, problem.input_dim),
        "C2": (problem.state_dim, problem.state_dim),
    }
    for name, value in cache.items():
        if value.shape != expected_shapes[name]:
            raise ValueError(
                f"upstream cache matrix {name} has shape {value.shape}, "
                f"expected {expected_shapes[name]}"
            )
        if not np.all(np.isfinite(value)):
            raise ValueError(f"upstream cache matrix {name} contains a non-finite value")
    A32 = np.asarray(A, dtype=np.float32)
    B32 = np.asarray(B, dtype=np.float32)
    f32 = np.asarray(f, dtype=np.float32)
    hover32 = np.asarray(hover_reference, dtype=np.float32)
    emitted_hover_next = A32 @ np.zeros(problem.state_dim, dtype=np.float32) + B32 @ hover32 + f32
    if np.linalg.norm(emitted_hover_next, ord=np.inf) > 1e-5:
        raise ValueError("float firmware matrices do not preserve hover")


def validate(problem: CompileTimeProblem) -> None:
    """Checks for consistently supplied arguments to TinyMPC"""
    if problem.state_dim <= 0 or problem.input_dim <= 0:
        raise ValueError("state_dim and input_dim must be positive")
    if problem.horizon_knots < 2:
        raise ValueError("horizon_knots must provide at least one input interval")
    if problem.horizon_knots > 25:
        raise ValueError("firmware half-space storage supports at most 25 horizon knots")
    if problem.model_dt_s <= 0.0 or problem.solve_rate_hz <= 0:
        raise ValueError("model_dt_s and solve_rate_hz must be positive")
    if not 0 <= problem.constrained_horizon_knots <= problem.horizon_knots:
        raise ValueError("constrained_horizon_knots must lie within the horizon")
    if problem.max_active_state_halfspaces < 1:
        raise ValueError("max_active_state_halfspaces must be at least one")
    if problem.admm_rho <= 0.0:
        raise ValueError("admm_rho must be positive")


def print_config(problem: CompileTimeProblem) -> None:
    """Print the proposed parameters"""
    horizon_s = (problem.horizon_knots - 1) * problem.model_dt_s
    print("Upstream TinyMPC -> Crazyflie fixed-size specialization")
    print(f"  state/input dimensions: {problem.state_dim} / {problem.input_dim}")
    print(f"  horizon: {problem.horizon_knots} knots, {problem.horizon_knots - 1} inputs")
    print(f"  model timestep: {problem.model_dt_s:.3f} s "
          f"({1.0 / problem.model_dt_s:.0f} Hz prediction)")
    print(f"  prediction span: {horizon_s:.3f} s")
    print(f"  MPC solve rate: {problem.solve_rate_hz} Hz")
    print(f"  ADMM iteration cap: {problem.admm_max_iterations}")
    print(f"  ADMM rho: {problem.admm_rho:g}")
    print(f"  STM32 scalar type: {problem.scalar_c_type}")
    print(f"  Crazyflie: {problem.crazyflie}")
    print(f"  deck: {problem.deck}")
    print(f"  propeller guards: {'yes' if problem.propeller_guards else 'no'}")
    print(
        "  obstacle half-spaces: "
        f"{problem.max_active_state_halfspaces} active, "
        f"first {problem.constrained_horizon_knots} knots only"
    )


def prompt_problem(defaults: CompileTimeProblem) -> CompileTimeProblem:
    print("Select Crazyflie: 1) Brushless  2) Brushed")
    crazyflie_default = "1" if defaults.crazyflie == "brushless" else "2"
    while True:
        try:
            crazyflie_choice = input(f"Select 1 or 2 [{crazyflie_default}]: ").strip()
        except EOFError:
            crazyflie_choice = crazyflie_default
        crazyflie_choice = crazyflie_choice or crazyflie_default
        if crazyflie_choice in ("1", "2"):
            break
        print("Please select 1 or 2.")

    print("Select decks: 1) None  2) AI deck  3) Flow deck  4) Both")
    deck_defaults = {"none": "1", "aideck": "2", "flowdeck": "3", "both": "4"}
    while True:
        try:
            deck_choice = input(f"Select 1, 2, 3, or 4 [{deck_defaults[defaults.deck]}]: ").strip()
        except EOFError:
            deck_choice = deck_defaults[defaults.deck]
        deck_choice = deck_choice or deck_defaults[defaults.deck]
        if deck_choice in ("1", "2", "3", "4"):
            break
        print("Please select 1, 2, 3, or 4.")

    print("Propeller guards: 1) Attached  2) Not attached")
    guards_default = "1" if defaults.propeller_guards else "2"
    while True:
        try:
            guards_choice = input(f"Select 1 or 2 [{guards_default}]: ").strip()
        except EOFError:
            guards_choice = guards_default
        guards_choice = guards_choice or guards_default
        if guards_choice in ("1", "2"):
            break
        print("Please select 1 or 2.")

    values = {}
    for name, default, value_type in (
        ("horizon_knots", defaults.horizon_knots, int),
        ("dt", defaults.model_dt_s, float),
        ("solve_rate", defaults.solve_rate_hz, int),
        ("admm_iterations", defaults.admm_max_iterations, int),
        ("admm_rho", defaults.admm_rho, float),
        ("maximum_halfspaces", defaults.max_active_state_halfspaces, int),
        ("constrained_horizon_knots", defaults.constrained_horizon_knots, int),
    ):
        while True:
            try:
                supplied = input(f"{name} [{default}]: ").strip()
            except EOFError:
                supplied = ""
            if not supplied:
                values[name] = default
                break
            try:
                values[name] = value_type(supplied)
                break
            except ValueError:
                print(f"Please enter a valid {value_type.__name__} or press Enter for {default}.")

    return CompileTimeProblem(
        state_dim=defaults.state_dim,
        input_dim=defaults.input_dim,
        crazyflie={"1": "brushless", "2": "brushed"}[crazyflie_choice],
        deck={"1": "none", "2": "aideck", "3": "flowdeck", "4": "both"}[deck_choice],
        propeller_guards=guards_choice == "1",
        horizon_knots=values["horizon_knots"],
        model_dt_s=values["dt"],
        solve_rate_hz=values["solve_rate"],
        admm_max_iterations=values["admm_iterations"],
        scalar_c_type=defaults.scalar_c_type,
        max_active_state_halfspaces=values["maximum_halfspaces"],
        constrained_horizon_knots=values["constrained_horizon_knots"],
        admm_rho=values["admm_rho"],
    )


def main() -> int:
    problem = prompt_problem(PROBLEM)
    validate(problem)
    print_config(problem)
    A, B, f = linearize_discrete_model(problem)
    Q, R = build_cost_matrices(problem)
    cache = build_upstream_cache(problem, A, B, f, Q, R)
    validate_generated_problem(problem, A, B, f, Q, R, cache)
    output_path = emit_firmware_header(problem, A, B, f, Q, R, cache)
    print(f"generated {output_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
