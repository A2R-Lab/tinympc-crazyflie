"""TinyMPC solver-to-nonlinear-rollout boundary for dataset v2.

The solver state is the firmware 12-vector ``[p, Rodrigues(q), v, omega_B]``.
Solver controls are motor-thrust deviations about physical hover. Raw dataset
states use ``[p, q_xyzw, v, omega_B]`` and never conflate solver and nonlinear
predicted trajectories.
"""
from __future__ import annotations

import importlib.util
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, Protocol

import numpy as np

from gap8_perception.dataset_contracts_v2 import (
    CONTROL_FEATURES,
    CONTROL_INTERVALS,
    PHYSICAL_STATE_FEATURES,
    STATE_KNOTS,
)


SOLVER_STATE_FEATURES = 12
MODEL_DT_S = 0.02
PHYSICS_DT_S = 0.002
HIGH_RATE_STATES = CONTROL_INTERVALS * int(round(MODEL_DT_S / PHYSICS_DT_S)) + 1


class TinyMpcBackend(Protocol):
    """Backend implemented by firmware SIL or a declared firmware-shadow solver."""

    backend_id: str
    controller_commit: str
    firmware_equivalent: bool

    def solve(self, initial_state: np.ndarray, reference: np.ndarray) -> TinyMpcSolution:
        """Solve one fixed 20-knot TinyMPC problem."""


class HighRatePhysicalPredictor(Protocol):
    predictor_id: str

    def rollout(
        self, initial_state: np.ndarray, physical_controls_n: np.ndarray
    ) -> tuple[np.ndarray, np.ndarray]:
        """Return physical high-rate states and exact 100 ms model knots."""


@dataclass(frozen=True)
class TinyMpcSolution:
    states: np.ndarray
    control_deviation_n: np.ndarray
    success: bool
    status: int
    iterations: int
    primal_residual: float
    dual_residual: float


@dataclass(frozen=True)
class PlantModel:
    """Deck-aware nonlinear plant and exact physical motor envelope."""

    derivative: Callable[[np.ndarray, np.ndarray], np.ndarray]
    hover_thrust_n: np.ndarray
    minimum_thrust_n: np.ndarray
    maximum_thrust_n: np.ndarray


@dataclass(frozen=True)
class PredictedCandidate:
    x_reference: np.ndarray
    x_solver: np.ndarray
    x_pre_high_rate: np.ndarray
    x_pre_knots: np.ndarray
    physical_controls_n: np.ndarray
    solver_success: bool
    solver_status: int
    solver_iterations: int
    primal_residual: float
    dual_residual: float
    backend_id: str
    controller_commit: str
    firmware_equivalent: bool


def load_deck_aware_plant(adapter_path: str | Path) -> PlantModel:
    """Load the pinned TinyMPC Crazyflie model with AI, flow, and guards."""
    path = Path(adapter_path).resolve()
    spec = importlib.util.spec_from_file_location("horizon_aware_tinympc_adapter", path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load TinyMPC adapter: {path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    problem = module.CompileTimeProblem(
        state_dim=SOLVER_STATE_FEATURES,
        input_dim=CONTROL_FEATURES,
        horizon_knots=STATE_KNOTS,
        model_dt_s=MODEL_DT_S,
        solve_rate_hz=50,
        crazyflie="brushless",
        deck="both",
        propeller_guards=True,
    )
    lower, upper, hover = module.build_input_bounds_and_reference(problem)
    return PlantModel(
        derivative=module.build_continuous_model(problem),
        hover_thrust_n=np.asarray(hover, dtype=np.float64),
        minimum_thrust_n=np.asarray(lower, dtype=np.float64),
        maximum_thrust_n=np.asarray(upper, dtype=np.float64),
    )


def _validate_solver_state(value: np.ndarray, shape: tuple[int, ...], name: str) -> np.ndarray:
    result = np.asarray(value, dtype=np.float64)
    if result.shape != shape or not np.isfinite(result).all():
        raise ValueError(f"{name} must have finite shape {shape}")
    return result


def quaternion_xyzw_from_rodrigues(rodrigues: np.ndarray) -> np.ndarray:
    """Convert firmware Gibbs/Rodrigues parameters ``q_xyz/q_w`` to quaternion."""
    vector = np.asarray(rodrigues, dtype=np.float64)
    quaternion_wxyz = np.concatenate(((1.0,), vector))
    quaternion_wxyz /= np.linalg.norm(quaternion_wxyz)
    return quaternion_wxyz[[1, 2, 3, 0]]


def solver_state_from_physical(state: np.ndarray) -> np.ndarray:
    """Convert physical ``[p,q_xyzw,v,omega_B]`` into the firmware 12-vector."""
    physical = _validate_solver_state(state, (PHYSICAL_STATE_FEATURES,), "physical state")
    quaternion = physical[3:7].copy()
    norm = float(np.linalg.norm(quaternion))
    if norm <= 1.0e-12:
        raise ValueError("physical-state quaternion has zero norm")
    quaternion /= norm
    if quaternion[3] < 0.0:
        quaternion = -quaternion
    if quaternion[3] <= 1.0e-8:
        raise ValueError("physical-state attitude is singular in firmware Rodrigues coordinates")
    return np.concatenate((physical[:3], quaternion[:3] / quaternion[3], physical[7:13]))


def rotation_matrix_from_solver_rodrigues(rodrigues: np.ndarray) -> np.ndarray:
    x, y, z, w = quaternion_xyzw_from_rodrigues(rodrigues)
    return np.asarray((
        (1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)),
        (2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)),
        (2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)),
    ), dtype=np.float64)


def solver_rodrigues_from_rotation_matrix(rotation: np.ndarray) -> np.ndarray:
    """Convert a proper rotation to firmware Gibbs/Rodrigues coordinates."""
    matrix = np.asarray(rotation, dtype=np.float64)
    if matrix.shape != (3, 3) or not np.isfinite(matrix).all():
        raise ValueError("rotation must have finite shape [3,3]")
    if not np.allclose(matrix.T @ matrix, np.eye(3), atol=1.0e-6) or not np.isclose(
        np.linalg.det(matrix), 1.0, atol=1.0e-6
    ):
        raise ValueError("rotation must be proper orthonormal")
    trace = float(np.trace(matrix))
    if trace > 0.0:
        scale = 2.0 * np.sqrt(trace + 1.0)
        quaternion = np.asarray((
            (matrix[2, 1] - matrix[1, 2]) / scale,
            (matrix[0, 2] - matrix[2, 0]) / scale,
            (matrix[1, 0] - matrix[0, 1]) / scale,
            0.25 * scale,
        ))
    else:
        index = int(np.argmax(np.diag(matrix)))
        j, k = (index + 1) % 3, (index + 2) % 3
        scale = 2.0 * np.sqrt(max(1.0 + matrix[index, index] - matrix[j, j] - matrix[k, k], 0.0))
        if scale <= 1.0e-12:
            raise ValueError("rotation is singular in firmware Rodrigues coordinates")
        xyz = np.zeros(3)
        xyz[index] = 0.25 * scale
        xyz[j] = (matrix[j, index] + matrix[index, j]) / scale
        xyz[k] = (matrix[k, index] + matrix[index, k]) / scale
        quaternion = np.concatenate((xyz, ((matrix[k, j] - matrix[j, k]) / scale,)))
    quaternion /= np.linalg.norm(quaternion)
    if quaternion[3] < 0.0:
        quaternion = -quaternion
    if quaternion[3] <= 1.0e-8:
        raise ValueError("rotation is singular in firmware Rodrigues coordinates")
    return quaternion[:3] / quaternion[3]


def physical_state_from_solver(state: np.ndarray) -> np.ndarray:
    state = _validate_solver_state(state, (SOLVER_STATE_FEATURES,), "solver state")
    physical = np.concatenate(
        (state[:3], quaternion_xyzw_from_rodrigues(state[3:6]), state[6:9], state[9:12])
    )
    if physical.shape != (PHYSICAL_STATE_FEATURES,):
        raise AssertionError("physical state feature contract changed")
    return physical.astype(np.float32)


def rk4_step(
    derivative: Callable[[np.ndarray, np.ndarray], np.ndarray],
    state: np.ndarray,
    control: np.ndarray,
) -> np.ndarray:
    first = PHYSICS_DT_S * np.asarray(derivative(state, control), dtype=np.float64)
    second = PHYSICS_DT_S * np.asarray(derivative(state + first / 2.0, control), dtype=np.float64)
    third = PHYSICS_DT_S * np.asarray(derivative(state + second / 2.0, control), dtype=np.float64)
    fourth = PHYSICS_DT_S * np.asarray(derivative(state + third, control), dtype=np.float64)
    return state + (first + 2.0 * second + 2.0 * third + fourth) / 6.0


def rollout_high_rate(
    initial_state: np.ndarray,
    physical_controls_n: np.ndarray,
    plant: PlantModel,
) -> tuple[np.ndarray, np.ndarray]:
    """Roll zero-order-held controls at 500 Hz and sample exact 100 ms model knots."""
    state = _validate_solver_state(initial_state, (SOLVER_STATE_FEATURES,), "initial state").copy()
    controls = _validate_solver_state(
        physical_controls_n,
        (CONTROL_INTERVALS, CONTROL_FEATURES),
        "physical controls",
    )
    if np.any(controls < plant.minimum_thrust_n) or np.any(controls > plant.maximum_thrust_n):
        raise ValueError("physical motor control violates the plant envelope")
    steps_per_interval = int(round(MODEL_DT_S / PHYSICS_DT_S))
    if not np.isclose(steps_per_interval * PHYSICS_DT_S, MODEL_DT_S, atol=1.0e-12):
        raise ValueError("physics timestep must divide the 100 ms model interval exactly")
    high_rate = np.empty((HIGH_RATE_STATES, PHYSICAL_STATE_FEATURES), dtype=np.float32)
    high_rate[0] = physical_state_from_solver(state)
    output_index = 1
    for control in controls:
        for _ in range(steps_per_interval):
            state = rk4_step(plant.derivative, state, control)
            if not np.isfinite(state).all():
                raise RuntimeError("nonlinear prediction diverged")
            high_rate[output_index] = physical_state_from_solver(state)
            output_index += 1
    knot_indices = np.arange(STATE_KNOTS) * steps_per_interval
    return high_rate, high_rate[knot_indices].copy()


def execute_candidate_prediction(
    backend: TinyMpcBackend,
    plant: PlantModel,
    initial_state: np.ndarray,
    reference: np.ndarray,
    physical_predictor: HighRatePhysicalPredictor | None = None,
) -> PredictedCandidate:
    """Solve, convert physical controls, and produce distinct solver/plant paths."""
    initial = _validate_solver_state(initial_state, (SOLVER_STATE_FEATURES,), "initial state")
    reference_states = _validate_solver_state(
        reference,
        (STATE_KNOTS, SOLVER_STATE_FEATURES),
        "reference",
    )
    solution = backend.solve(initial, reference_states)
    solver_states = _validate_solver_state(
        solution.states,
        (STATE_KNOTS, SOLVER_STATE_FEATURES),
        "X_solver",
    )
    deviations = _validate_solver_state(
        solution.control_deviation_n,
        (CONTROL_INTERVALS, CONTROL_FEATURES),
        "solver controls",
    )
    if not solution.success:
        raise RuntimeError(f"TinyMPC backend {backend.backend_id} failed with status {solution.status}")
    hover = np.asarray(plant.hover_thrust_n, dtype=np.float64)
    if hover.shape != (CONTROL_FEATURES,):
        raise ValueError("hover_thrust_n must have shape [4]")
    physical_controls = hover[None, :] + deviations
    if np.any(physical_controls < plant.minimum_thrust_n) or np.any(
        physical_controls > plant.maximum_thrust_n
    ):
        raise ValueError("solver motor control violates the physical plant envelope")
    if physical_predictor is None:
        high_rate, knots = rollout_high_rate(initial, physical_controls, plant)
    else:
        high_rate, knots = physical_predictor.rollout(initial, physical_controls)
        high_rate = _validate_solver_state(
            high_rate, (HIGH_RATE_STATES, PHYSICAL_STATE_FEATURES), "native X_pre high-rate"
        ).astype(np.float32)
        knots = _validate_solver_state(
            knots, (STATE_KNOTS, PHYSICAL_STATE_FEATURES), "native X_pre knots"
        ).astype(np.float32)
    return PredictedCandidate(
        x_reference=np.stack([physical_state_from_solver(state) for state in reference_states]),
        x_solver=np.stack([physical_state_from_solver(state) for state in solver_states]),
        x_pre_high_rate=high_rate,
        x_pre_knots=knots,
        physical_controls_n=physical_controls.astype(np.float32),
        solver_success=True,
        solver_status=int(solution.status),
        solver_iterations=int(solution.iterations),
        primal_residual=float(solution.primal_residual),
        dual_residual=float(solution.dual_residual),
        backend_id=backend.backend_id,
        controller_commit=backend.controller_commit,
        firmware_equivalent=bool(backend.firmware_equivalent),
    )
