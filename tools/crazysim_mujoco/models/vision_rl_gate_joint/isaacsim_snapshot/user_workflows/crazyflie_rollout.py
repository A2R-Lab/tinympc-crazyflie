"""Execute reference paths with the generated TinyMPC controller and Crazyflie plant."""

from __future__ import annotations

import math
import os
from pathlib import Path

import numpy as np

from gap8_perception.firmware_tinympc_backend_v2 import FirmwareGeneratedTinyMpcBackend
from gap8_perception.tinympc_execution_v2 import (
    MODEL_DT_S,
    PHYSICS_DT_S,
    load_deck_aware_plant,
    physical_state_from_solver,
    rk4_step,
    rotation_matrix_from_solver_rodrigues,
    solver_rodrigues_from_rotation_matrix,
)


DEFAULT_TINY_MPC_REPOSITORY = Path(__file__).resolve().parents[2] / "tinympc-crazyflie"
TINY_MPC_REPOSITORY = Path(os.environ.get("TINY_MPC_REPO", DEFAULT_TINY_MPC_REPOSITORY))
TINY_MPC_ADAPTER = (
    TINY_MPC_REPOSITORY
    / "apps/controller_tinympc_eigen/tools/firmware_codegen/tinympc_to_crazyflie_adapter.py"
)
CONTROL_DT_S = 0.02  # Re-solve at 50 Hz; the prediction model itself uses 100 ms knots.
HORIZON_KNOTS = 20


def _vector(values):
    return [float(value) for value in values]


def _yaw_rotation(yaw):
    cosine, sine = math.cos(yaw), math.sin(yaw)
    return np.asarray(((cosine, -sine, 0.0), (sine, cosine, 0.0), (0.0, 0.0, 1.0)))


def _yaw_from_rotation(rotation):
    return math.atan2(float(rotation[1, 0]), float(rotation[0, 0]))


def _rpy_from_rotation(rotation):
    pitch = math.asin(float(np.clip(-rotation[2, 0], -1.0, 1.0)))
    roll = math.atan2(float(rotation[2, 1]), float(rotation[2, 2]))
    yaw = _yaw_from_rotation(rotation)
    return [roll, pitch, yaw]


def _reference_arrays(reference_states):
    times = np.asarray([record["time_s"] for record in reference_states], dtype=float)
    positions = np.asarray([record["position_m"] for record in reference_states], dtype=float)
    velocities = np.asarray([record["velocity_mps"] for record in reference_states], dtype=float)
    accelerations = np.asarray([record["acceleration_mps2"] for record in reference_states], dtype=float)
    yaws = np.unwrap(np.asarray([record["attitude_rpy_rad"][2] for record in reference_states]))
    yaw_rates = np.asarray([record["angular_velocity_rps"][2] for record in reference_states])
    return times, positions, velocities, accelerations, yaws, yaw_rates


def _interpolate(times, values, sample_times):
    return np.stack([
        np.interp(sample_times, times, values[:, axis]) for axis in range(values.shape[1])
    ], axis=1)


def _local_problem(state, simulation_time, reference):
    """Express current state and the next reference horizon in current-yaw frame."""
    times, positions, velocities, _accelerations, yaws, yaw_rates = reference
    rotation_w_b = rotation_matrix_from_solver_rodrigues(state[3:6])
    current_yaw = _yaw_from_rotation(rotation_w_b)
    rotation_l_w = _yaw_rotation(-current_yaw)
    origin_w = state[:3].copy()

    initial = state.copy()
    initial[:3] = 0.0
    initial[3:6] = solver_rodrigues_from_rotation_matrix(rotation_l_w @ rotation_w_b)
    initial[6:9] = rotation_l_w @ state[6:9]

    knot_times = simulation_time + MODEL_DT_S * np.arange(HORIZON_KNOTS)
    reference_positions = _interpolate(times, positions, knot_times)
    reference_velocities = _interpolate(times, velocities, knot_times)
    reference_yaws = np.interp(knot_times, times, yaws)
    reference_yaw_rates = np.interp(knot_times, times, yaw_rates)
    horizon = np.zeros((HORIZON_KNOTS, 12), dtype=np.float32)
    horizon[:, :3] = (rotation_l_w @ (reference_positions - origin_w).T).T
    horizon[:, 6:9] = (rotation_l_w @ reference_velocities.T).T
    for index, yaw in enumerate(reference_yaws):
        horizon[index, 3:6] = solver_rodrigues_from_rotation_matrix(
            rotation_l_w @ _yaw_rotation(float(yaw))
        )
    horizon[:, 11] = reference_yaw_rates
    return initial.astype(np.float32), horizon


def execute_trajectory(trajectory_manifest):
    """Track a geometric reference with exact generated TinyMPC in closed loop.

    TinyMPC is solved every 20 ms in the drone's current-yaw local frame, and
    each solve predicts on the firmware's 20 ms model knots. Only
    its first motor command is applied, through the deck-aware nonlinear plant
    at 500 Hz, before solving again. Camera records retain their independent
    nominal 30 Hz sampling and timestamp jitter.
    """
    backend = FirmwareGeneratedTinyMpcBackend(TINY_MPC_REPOSITORY)
    plant = load_deck_aware_plant(TINY_MPC_ADAPTER)
    reference_states = trajectory_manifest["states"]
    reference = _reference_arrays(reference_states)

    state = np.zeros(12, dtype=float)
    state[:3] = np.asarray(reference_states[0]["position_m"], dtype=float)
    state[6:9] = np.asarray(reference_states[0]["velocity_mps"], dtype=float)
    state[9:12] = np.asarray(reference_states[0]["angular_velocity_rps"], dtype=float)
    initial_yaw = float(reference_states[0]["attitude_rpy_rad"][2])
    state[5] = math.tan(0.5 * initial_yaw)
    motors = np.asarray(plant.hover_thrust_n, dtype=float)
    records = []
    # The camera manifest may begin at a later sample of a longer reference
    # path.  That first state is the physical initial condition, so execution
    # starts at its timestamp; beginning at zero would hold its position while
    # chasing its already-nonzero path velocity before the first rendered frame.
    simulation_time = float(reference_states[0]["time_s"])
    next_control_time = simulation_time
    physical_rollout_times = [simulation_time]
    physical_rollout_states = [physical_state_from_solver(state)]
    solve_count = 0
    maximum_primal_residual = 0.0
    maximum_dual_residual = 0.0
    motor_saturation_count = 0
    maximum_motor_envelope_violation_n = 0.0

    for requested in reference_states:
        target_time = float(requested["time_s"])
        while simulation_time + 1.0e-9 < target_time:
            if simulation_time + 1.0e-9 >= next_control_time:
                local_state, local_reference = _local_problem(state, simulation_time, reference)
                solution = backend.solve(local_state, local_reference)
                if not solution.success:
                    raise RuntimeError(
                        "TinyMPC failed during trajectory execution "
                        f"(status={solution.status}, primal={solution.primal_residual:.3g}, "
                        f"dual={solution.dual_residual:.3g})"
                    )
                requested_motors = np.asarray(plant.hover_thrust_n) + solution.control_deviation_n[0]
                violation = np.maximum(
                    plant.minimum_thrust_n - requested_motors,
                    requested_motors - plant.maximum_thrust_n,
                )
                maximum_violation = float(np.maximum(violation, 0.0).max())
                if maximum_violation > 0.0:
                    motor_saturation_count += 1
                    maximum_motor_envelope_violation_n = max(
                        maximum_motor_envelope_violation_n, maximum_violation
                    )
                # The physical motors cannot realize an out-of-envelope solver
                # request. Model the same actuator saturation that occurs on the
                # vehicle and retain its frequency/magnitude in episode metadata.
                motors = np.clip(
                    requested_motors, plant.minimum_thrust_n, plant.maximum_thrust_n
                )
                solve_count += 1
                maximum_primal_residual = max(maximum_primal_residual, solution.primal_residual)
                maximum_dual_residual = max(maximum_dual_residual, solution.dual_residual)
                next_control_time += CONTROL_DT_S
            step = min(PHYSICS_DT_S, target_time - simulation_time, next_control_time - simulation_time)
            if step <= 1.0e-10:
                simulation_time = next_control_time
                continue
            if math.isclose(step, PHYSICS_DT_S, rel_tol=0.0, abs_tol=1.0e-12):
                state = rk4_step(plant.derivative, state, motors)
            else:
                # Exact final partial step to land on a jittered camera timestamp.
                k1 = step * np.asarray(plant.derivative(state, motors))
                k2 = step * np.asarray(plant.derivative(state + k1 / 2.0, motors))
                k3 = step * np.asarray(plant.derivative(state + k2 / 2.0, motors))
                k4 = step * np.asarray(plant.derivative(state + k3, motors))
                state = state + (k1 + 2.0 * k2 + 2.0 * k3 + k4) / 6.0
            simulation_time += step
            physical_rollout_times.append(simulation_time)
            physical_rollout_states.append(physical_state_from_solver(state))

        physical = physical_state_from_solver(state)
        rotation = rotation_matrix_from_solver_rodrigues(state[3:6])
        acceleration = np.asarray(plant.derivative(state, motors), dtype=float)[6:9]
        records.append({
            "frame_index": int(requested["frame_index"]),
            "time_s": target_time,
            "delta_t_s": float(requested["delta_t_s"]),
            "position_m": _vector(physical[:3]),
            "velocity_mps": _vector(physical[7:10]),
            "acceleration_mps2": _vector(acceleration),
            "attitude_quaternion_xyzw": _vector(physical[3:7]),
            "attitude_rpy_rad": _rpy_from_rotation(rotation),
            "angular_velocity_rps": _vector(physical[10:13]),
            "motor_thrust_n": _vector(motors),
            "active_waypoint_m": requested.get("active_waypoint_m"),
            "active_waypoint_kind": requested.get("active_waypoint_kind"),
            "active_waypoint_source": requested.get("active_waypoint_source"),
        })

    return {
        "completion_profile": "firmware_sil_physical_execution",
        "backend": backend.backend_id,
        "controller_commit": backend.controller_commit,
        "firmware_equivalent_tinympc": True,
        "model_selection_eligible": True,
        "physics_rate_hz": int(round(1.0 / PHYSICS_DT_S)),
        "controller_rate_hz": int(round(1.0 / CONTROL_DT_S)),
        "capture_timing": trajectory_manifest["capture_timing"],
        "vehicle_configuration": {
            "crazyflie": "brushless", "deck": "both", "propeller_guards": True,
        },
        "solver_statistics": {
            "solve_count": solve_count,
            "maximum_primal_residual": float(maximum_primal_residual),
            "maximum_dual_residual": float(maximum_dual_residual),
            "motor_saturation_count": motor_saturation_count,
            "maximum_motor_envelope_violation_n": maximum_motor_envelope_violation_n,
        },
        # Private binary payload consumed by the renderer. Keeping this at the
        # physics integration rate lets target generation sample the candidate
        # horizon at TinyMPC's exact 100 ms model-knot interval, independently of the
        # jittered 30 Hz camera timestamps.
        "_physical_rollout_time_s": np.asarray(physical_rollout_times, dtype=np.float64),
        "_physical_rollout_state_f32": np.asarray(physical_rollout_states, dtype=np.float32),
        "states": records,
    }
