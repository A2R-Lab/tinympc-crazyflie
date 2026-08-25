"""Strict structural contracts for horizon-aware dataset v2 artifacts.

The validator is intentionally independent of Isaac Sim and PyTorch so every
generation stage can reject incomplete artifacts before expensive rendering or
training begins.
"""
from __future__ import annotations

import re
from dataclasses import dataclass
from pathlib import Path
from typing import Mapping

import numpy as np


SCHEMA_VERSION = "horizon-aware-dataset-v2.2"
STATE_KNOTS = 20
CONTROL_INTERVALS = 19
PHYSICAL_STATE_FEATURES = 13  # position(3), quaternion xyzw(4), velocity(3), body rate(3)
NETWORK_KNOT_FEATURES = 13
CONTROL_FEATURES = 4

SEED_NAMES = (
    "geometry",
    "material_texture",
    "lighting_appearance",
    "reference_mission",
    "controller_execution",
    "counterfactual_candidate",
    "sensor_estimator_noise",
)
SHA256_PATTERN = re.compile(r"^[0-9a-f]{64}$")


class DatasetContractError(ValueError):
    """Raised when a v2 artifact violates the normative dataset contract."""


@dataclass(frozen=True)
class CandidateBankDimensions:
    observations: int
    candidates: int
    high_rate_states: int
    brake_branches: int
    brake_state_knots: int
    brake_high_rate_states: int

    def validate(self) -> None:
        if min(
            self.observations,
            self.candidates,
            self.high_rate_states,
            self.brake_branches,
            self.brake_state_knots,
            self.brake_high_rate_states,
        ) <= 0:
            raise DatasetContractError("all candidate-bank dimensions must be positive")


@dataclass(frozen=True)
class ExecutionLinkageDimensions:
    decisions: int
    high_rate_states: int

    def validate(self) -> None:
        if self.decisions <= 0 or self.high_rate_states < 2:
            raise DatasetContractError("execution linkage dimensions must be positive with H>=2")


def _require_keys(record: Mapping[str, object], required: set[str], artifact: str) -> None:
    missing = sorted(required.difference(record))
    if missing:
        raise DatasetContractError(f"{artifact} is missing required fields: {', '.join(missing)}")


def validate_dataset_manifest(manifest: Mapping[str, object]) -> None:
    """Validate immutable provenance needed to reproduce a dataset."""
    _require_keys(
        manifest,
        {
            "schema_version",
            "dataset_id",
            "completion_profile",
            "seeds",
            "versions",
            "contract_hashes",
            "layouts",
        },
        "dataset manifest",
    )
    if manifest["schema_version"] != SCHEMA_VERSION:
        raise DatasetContractError(
            f"unsupported schema_version {manifest['schema_version']!r}; expected {SCHEMA_VERSION!r}"
        )
    if manifest["completion_profile"] not in {"diagnostic_sim", "conforming_risk_only", "flight_candidate"}:
        raise DatasetContractError("invalid completion_profile")
    seeds = manifest["seeds"]
    if not isinstance(seeds, Mapping):
        raise DatasetContractError("seeds must be an object")
    _require_keys(seeds, set(SEED_NAMES), "dataset seed map")
    if any(not isinstance(seeds[name], int) or isinstance(seeds[name], bool) for name in SEED_NAMES):
        raise DatasetContractError("every deterministic seed must be an integer")
    versions = manifest["versions"]
    hashes = manifest["contract_hashes"]
    if not isinstance(versions, Mapping) or not isinstance(hashes, Mapping):
        raise DatasetContractError("versions and contract_hashes must be objects")
    _require_keys(
        versions,
        {"generator", "isaac_sim", "controller", "dynamics", "estimator", "camera", "label_oracle"},
        "version map",
    )
    _require_keys(
        hashes,
        {
            "scene_distribution",
            "camera_acquisition",
            "camera_detectability",
            "camera_calibration",
            "camera_extrinsic",
            "flight_envelope",
            "candidate_generator",
            "architecture_registry",
            "operating_design_domain",
            "candidate_coverage",
            "execution_linkage",
            "braking_characterization",
            "free_space_memory",
            "runtime_abi",
            "change_control",
        },
        "contract hash map",
    )
    if any(not isinstance(value, str) or not value for value in versions.values()):
        raise DatasetContractError("every version identifier must be a nonempty string")
    if any(not isinstance(value, str) or SHA256_PATTERN.fullmatch(value) is None for value in hashes.values()):
        raise DatasetContractError("every contract hash must be a lowercase 64-character SHA-256")


def validate_frame_record(frame: Mapping[str, object]) -> None:
    """Validate one raw exposure/state record without inspecting referenced files."""
    _require_keys(
        frame,
        {
            "frame_index",
            "exposure_start_s",
            "exposure_end_s",
            "exposure_midpoint_s",
            "estimator_state_time_s",
            "controller_tick",
            "physics_tick",
            "actuator_command",
            "estimated_state",
            "simulator_truth_state",
            "camera_state",
            "history_relative_camera_transform",
            "mission_state",
            "contact_state",
            "files",
        },
        "frame record",
    )
    start = float(frame["exposure_start_s"])
    end = float(frame["exposure_end_s"])
    midpoint = float(frame["exposure_midpoint_s"])
    if not start <= midpoint <= end:
        raise DatasetContractError("exposure midpoint must lie between exposure start and end")
    if not np.isclose(midpoint, 0.5 * (start + end), atol=1.0e-9):
        raise DatasetContractError("exposure midpoint does not match start/end")
    history = np.asarray(frame["history_relative_camera_transform"], dtype=np.float64)
    if history.size == 0:
        history = history.reshape(0, 6)
    if history.ndim != 2 or history.shape[1] != 6 or not np.isfinite(history).all():
        raise DatasetContractError("history_relative_camera_transform must have finite shape [T-1,6]")
    contact = frame["contact_state"]
    if not isinstance(contact, Mapping) or not isinstance(contact.get("contact"), bool):
        raise DatasetContractError("contact_state must contain a boolean contact field")
    files = frame["files"]
    if not isinstance(files, Mapping):
        raise DatasetContractError("files must be an object")
    _require_keys(files, {"rgb", "depth", "semantic"}, "frame file map")
    for name, artifact in files.items():
        if not isinstance(artifact, Mapping) or set(artifact) != {"path", "sha256"}:
            raise DatasetContractError(f"frame file {name} must contain exactly path and sha256")
        if not isinstance(artifact["path"], str) or not artifact["path"]:
            raise DatasetContractError(f"frame file {name} path must be nonempty")
        if not isinstance(artifact["sha256"], str) or SHA256_PATTERN.fullmatch(artifact["sha256"]) is None:
            raise DatasetContractError(f"frame file {name} must have a lowercase SHA-256")


def _validate_array(
    archive: Mapping[str, np.ndarray],
    name: str,
    shape: tuple[int, ...],
    kind: str,
    finite: bool = True,
) -> None:
    if name not in archive:
        raise DatasetContractError(f"candidate bank is missing {name}")
    value = np.asarray(archive[name])
    if value.shape != shape:
        raise DatasetContractError(f"{name} has shape {value.shape}; expected {shape}")
    if value.dtype.kind not in kind:
        raise DatasetContractError(f"{name} has incompatible dtype {value.dtype}")
    if finite and value.dtype.kind in "fc" and not np.isfinite(value).all():
        raise DatasetContractError(f"{name} contains non-finite values")


def validate_candidate_bank(
    archive: Mapping[str, np.ndarray], dimensions: CandidateBankDimensions
) -> None:
    """Validate separated ordinary and branch-to-brake raw artifacts."""
    dimensions.validate()
    n, c = dimensions.observations, dimensions.candidates
    h, nb, kb = dimensions.high_rate_states, dimensions.brake_branches, dimensions.brake_state_knots
    brake_h = dimensions.brake_high_rate_states
    arrays = {
        "candidate_generator_sha256_u8": ((n, 32), "u"),
        "capture_epoch_s_f64": ((n,), "f"),
        "solve_epoch_s_f64": ((n,), "f"),
        "execute_epoch_s_f64": ((n,), "f"),
        "observation_frame_index_i32": ((n,), "iu"),
        "bank_id_i64": ((n,), "iu"),
        "candidate_id_i64": ((n, c), "iu"),
        "solver_status_i32": ((n, c), "iu"),
        "solver_iterations_i32": ((n, c), "iu"),
        "solver_primal_residual_f32": ((n, c), "f"),
        "solver_dual_residual_f32": ((n, c), "f"),
        "capture_state_estimate_w_f32": ((n, PHYSICAL_STATE_FEATURES), "f"),
        "capture_camera_transform_w_f_c_f32": ((n, 4, 4), "f"),
        "predicted_execute_state_w_f32": ((n, PHYSICAL_STATE_FEATURES), "f"),
        "transform_w_l_e_f32": ((n, 4, 4), "f"),
        "transform_f_c_l_e_f32": ((n, 4, 4), "f"),
        "common_execute_state_l_e_f32": ((n, PHYSICAL_STATE_FEATURES), "f"),
        "incumbent_intervals_consumed_i32": ((n,), "iu"),
        "current_motion_f32": ((n, 6), "f"),
        "incumbent_control_prefix_f32": ((n, CONTROL_INTERVALS, CONTROL_FEATURES), "f"),
        "incumbent_control_valid_u8": ((n, CONTROL_INTERVALS), "bu"),
        "x_ref_f32": ((n, c, STATE_KNOTS, PHYSICAL_STATE_FEATURES), "f"),
        "x_solver_valid_u8": ((n, c), "bu"),
        "x_pre_high_rate_f32": ((n, c, h, PHYSICAL_STATE_FEATURES), "f"),
        "x_pre_knots_f32": ((n, c, STATE_KNOTS, PHYSICAL_STATE_FEATURES), "f"),
        "candidate_controls_f32": ((n, c, CONTROL_INTERVALS, CONTROL_FEATURES), "f"),
        "candidate_valid_u8": ((n, c), "bu"),
        "horizon_valid_u8": ((n, c, STATE_KNOTS), "bu"),
        "projection_domain_valid_u8": ((n, c), "bu"),
        "candidate_horizon_f32": ((n, c, STATE_KNOTS, NETWORK_KNOT_FEATURES), "f"),
        "brake_x_pre_high_rate_f32": ((n, c, nb, brake_h, PHYSICAL_STATE_FEATURES), "f"),
        "brake_x_pre_knots_f32": ((n, c, nb, kb, PHYSICAL_STATE_FEATURES), "f"),
        "brake_controls_f32": ((n, c, nb, kb - 1, CONTROL_FEATURES), "f"),
        "brake_contingency_f32": ((n, c, nb, kb, NETWORK_KNOT_FEATURES), "f"),
        "brake_contingency_valid_u8": ((n, c, nb), "bu"),
        "brake_reaches_stopped_u8": ((n, c, nb), "bu"),
        "brake_horizon_valid_u8": ((n, c, nb, kb), "bu"),
        "brake_projection_domain_valid_u8": ((n, c, nb), "bu"),
        "brake_reachable_tube_radius_f32": ((n, c, nb, kb - 1), "f"),
        "brake_reachable_tube_valid_u8": ((n, c, nb, kb - 1), "bu"),
        "brake_terminal_reachable_radius_f32": ((n, c, nb), "f"),
        "brake_terminal_hold_valid_u8": ((n, c, nb), "bu"),
    }
    for name, (shape, kind) in arrays.items():
        _validate_array(archive, name, shape, kind)
    if "x_solver_f32" in archive:
        _validate_array(
            archive,
            "x_solver_f32",
            (n, c, STATE_KNOTS, PHYSICAL_STATE_FEATURES),
            "f",
        )
    elif np.asarray(archive["x_solver_valid_u8"], dtype=bool).any():
        raise DatasetContractError("x_solver_valid_u8 cannot be true when x_solver_f32 is absent")
    capture = np.asarray(archive["capture_epoch_s_f64"])
    solve = np.asarray(archive["solve_epoch_s_f64"])
    execute = np.asarray(archive["execute_epoch_s_f64"])
    if not np.all(capture <= solve) or not np.all(solve < execute):
        raise DatasetContractError("candidate-bank epochs must satisfy capture <= solve < execute")
    consumed = np.asarray(archive["incumbent_intervals_consumed_i32"], dtype=np.int64)
    expected_consumed = np.ceil((execute - capture) / 0.02 - 1.0e-12).astype(np.int64)
    if np.any(consumed != expected_consumed) or np.any((consumed < 0) | (consumed > CONTROL_INTERVALS)):
        raise DatasetContractError("incumbent interval count differs from capture-to-execute duration")
    transform_w_l_e = np.asarray(archive["transform_w_l_e_f32"], dtype=np.float64)
    transform_w_f_c = np.asarray(archive["capture_camera_transform_w_f_c_f32"], dtype=np.float64)
    transform_f_c_l_e = np.asarray(archive["transform_f_c_l_e_f32"], dtype=np.float64)
    predicted_execute = np.asarray(archive["predicted_execute_state_w_f32"], dtype=np.float64)
    common_execute = np.asarray(archive["common_execute_state_l_e_f32"], dtype=np.float64)
    homogeneous = np.asarray((0.0, 0.0, 0.0, 1.0))
    if not np.allclose(transform_w_l_e[:, 3], homogeneous, atol=1.0e-6) or not np.allclose(
        transform_w_f_c[:, 3], homogeneous, atol=1.0e-6
    ) or not np.allclose(transform_f_c_l_e[:, 3], homogeneous, atol=1.0e-6):
        raise DatasetContractError("execution-frame transform has an invalid homogeneous row")
    if not np.allclose(transform_w_l_e[:, :3, 3], predicted_execute[:, :3], atol=1.0e-5):
        raise DatasetContractError("L_e origin differs from the predicted t_e position")
    if not np.allclose(common_execute[:, :3], 0.0, atol=1.0e-6):
        raise DatasetContractError("common execute state is not expressed at the L_e origin")
    reconstructed = np.linalg.inv(transform_w_f_c) @ transform_w_l_e
    if not np.allclose(transform_f_c_l_e, reconstructed, atol=2.0e-5):
        raise DatasetContractError("stored F_c<-L_e transform does not follow the frame chain")
    frame_indices = np.asarray(archive["observation_frame_index_i32"])
    if np.any(frame_indices < 0) or np.any(np.diff(frame_indices) <= 0):
        raise DatasetContractError("observation frame indices must be nonnegative and strictly increasing")
    candidate_valid = np.asarray(archive["candidate_valid_u8"], dtype=bool)
    horizon_valid = np.asarray(archive["horizon_valid_u8"], dtype=bool)
    if np.any(candidate_valid & ~horizon_valid.all(axis=-1)):
        raise DatasetContractError("valid candidates require all 20 physical horizon states")
    contingency = np.asarray(archive["brake_contingency_valid_u8"], dtype=bool)
    brake_horizon_valid = np.asarray(archive["brake_horizon_valid_u8"], dtype=bool)
    if np.any(contingency & ~brake_horizon_valid.all(axis=-1)):
        raise DatasetContractError("valid brake contingencies require every horizon state")
    reaches_stopped = np.asarray(archive["brake_reaches_stopped_u8"], dtype=bool)
    tube_valid = np.asarray(archive["brake_reachable_tube_valid_u8"], dtype=bool)
    tube_radius = np.asarray(archive["brake_reachable_tube_radius_f32"], dtype=np.float64)
    terminal_hold = np.asarray(archive["brake_terminal_hold_valid_u8"], dtype=bool)
    terminal_radius = np.asarray(archive["brake_terminal_reachable_radius_f32"], dtype=np.float64)
    if np.any(tube_radius < 0.0) or np.any(terminal_radius < 0.0):
        raise DatasetContractError("reachable-tube radii must be nonnegative")
    partial_tube = tube_valid.any(axis=-1) & ~tube_valid.all(axis=-1)
    if np.any(partial_tube):
        raise DatasetContractError("reachable-tube evidence must cover every segment or none")
    if np.any(tube_valid.all(axis=-1) & ~contingency):
        raise DatasetContractError("reachable-tube evidence requires a dynamically valid branch")
    if np.any(terminal_hold & ~(tube_valid.all(axis=-1) & reaches_stopped & contingency)):
        raise DatasetContractError("terminal-hold validity requires complete tube and stopped branch evidence")


def validate_candidate_bank_file(path: str | Path, dimensions: CandidateBankDimensions) -> None:
    with np.load(Path(path), allow_pickle=False) as archive:
        validate_candidate_bank(archive, dimensions)


def validate_execution_linkage(
    archive: Mapping[str, np.ndarray],
    dimensions: ExecutionLinkageDimensions,
    candidate_bank: Mapping[str, np.ndarray] | None = None,
) -> None:
    """Validate selected -> resolved -> executed lineage for every decision."""
    dimensions.validate()
    n, h = dimensions.decisions, dimensions.high_rate_states
    arrays = {
        "observation_frame_index_i32": ((n,), "iu"),
        "bank_id_i64": ((n,), "iu"),
        "selected_candidate_id_i64": ((n,), "iu"),
        "scored_plan_id_i64": ((n,), "iu"),
        "resolved_plan_id_i64": ((n,), "iu"),
        "capture_epoch_s_f64": ((n,), "f"),
        "execute_epoch_s_f64": ((n,), "f"),
        "resolve_epoch_s_f64": ((n,), "f"),
        "actuation_epoch_s_f64": ((n,), "f"),
        "decision_code_i8": ((n,), "iu"),
        "decision_reason_u64": ((n,), "U"),
        "selected_x_pre_high_rate_f32": ((n, h, PHYSICAL_STATE_FEATURES), "f"),
        "selected_x_pre_valid_u8": ((n,), "bu"),
        "resolved_x_high_rate_f32": ((n, h, PHYSICAL_STATE_FEATURES), "f"),
        "resolved_x_valid_u8": ((n,), "bu"),
        "executed_x_high_rate_f32": ((n, h, PHYSICAL_STATE_FEATURES), "f"),
        "executed_x_valid_u8": ((n, h), "bu"),
        "handoff_metric_valid_u8": ((n,), "bu"),
        "handoff_position_error_m_f32": ((n,), "f"),
        "handoff_velocity_error_mps_f32": ((n,), "f"),
        "handoff_attitude_error_rad_f32": ((n,), "f"),
        "handoff_angular_velocity_error_rps_f32": ((n,), "f"),
        "handoff_execution_epoch_error_s_f32": ((n,), "f"),
        "handoff_swept_body_deviation_m_f32": ((n,), "f"),
        "handoff_timing_deviation_m_f32": ((n,), "f"),
        "handoff_allocated_uncertainty_m_f32": ((n,), "f"),
        "handoff_clearance_degradation_bound_m_f32": ((n,), "f"),
    }
    for name, (shape, kind) in arrays.items():
        _validate_array(archive, name, shape, kind)
    codes = np.asarray(archive["decision_code_i8"])
    if np.any(~np.isin(codes, (0, 1, 2))):
        raise DatasetContractError("decision_code must be 0=no-selection, 1=accepted, or 2=rejected-fallback")
    capture = np.asarray(archive["capture_epoch_s_f64"])
    execute = np.asarray(archive["execute_epoch_s_f64"])
    resolve_epoch = np.asarray(archive["resolve_epoch_s_f64"])
    actuation = np.asarray(archive["actuation_epoch_s_f64"])
    if np.any(capture > execute) or np.any(resolve_epoch < capture) or np.any(resolve_epoch > actuation):
        raise DatasetContractError("execution linkage epochs are inconsistent")
    accepted = codes == 1
    selected_valid = np.asarray(archive["selected_x_pre_valid_u8"], dtype=bool)
    resolved_valid = np.asarray(archive["resolved_x_valid_u8"], dtype=bool)
    executed_valid = np.asarray(archive["executed_x_valid_u8"], dtype=bool)
    metric_valid = np.asarray(archive["handoff_metric_valid_u8"], dtype=bool)
    if np.any(accepted & ~(selected_valid & resolved_valid & metric_valid & executed_valid[:, 0])):
        raise DatasetContractError("accepted decisions require selected, resolved, handoff, and executed evidence")
    no_selection = codes == 0
    selected_ids = np.asarray(archive["selected_candidate_id_i64"])
    if np.any(no_selection & (selected_ids != -1)) or np.any(no_selection & selected_valid):
        raise DatasetContractError("no-selection decisions may not name or store a selected path")
    if candidate_bank is not None:
        bank_frames = np.asarray(candidate_bank["observation_frame_index_i32"])
        bank_ids = np.asarray(candidate_bank["bank_id_i64"])
        candidate_ids = np.asarray(candidate_bank["candidate_id_i64"])
        candidates = np.asarray(candidate_bank["x_pre_high_rate_f32"])
        if len(bank_frames) != n:
            raise DatasetContractError("execution decisions and candidate-bank observations differ")
        if not np.array_equal(bank_frames, np.asarray(archive["observation_frame_index_i32"])) or not np.array_equal(bank_ids, np.asarray(archive["bank_id_i64"])):
            raise DatasetContractError("execution linkage frame/bank IDs differ from candidate bank")
        for decision in np.flatnonzero(selected_valid):
            matches = np.flatnonzero(candidate_ids[decision] == selected_ids[decision])
            if len(matches) != 1:
                raise DatasetContractError("selected candidate ID is absent or ambiguous in its bank")
            if not np.array_equal(
                np.asarray(archive["selected_x_pre_high_rate_f32"])[decision],
                candidates[decision, matches[0]],
            ):
                raise DatasetContractError("stored selected X_pre does not exactly match its candidate bank")


def validate_target_cache(
    archive: Mapping[str, np.ndarray], dimensions: CandidateBankDimensions, dense_size: int = 40
) -> None:
    """Validate model-independent derived labels separately from immutable raw data."""
    dimensions.validate()
    n, c = dimensions.observations, dimensions.candidates
    nb, kb = dimensions.brake_branches, dimensions.brake_state_knots
    arrays = {
        "observation_frame_index_i32": ((n,), "iu"),
        "inverse_depth_f32": ((n, 1, dense_size, dense_size), "f"),
        "depth_valid_u8": ((n, 1, dense_size, dense_size), "bu"),
        "flow_f32": ((n, 2, dense_size, dense_size), "f"),
        "flow_valid_u8": ((n, 1, dense_size, dense_size), "bu"),
        "divergence_f32": ((n, 1, dense_size, dense_size), "f"),
        "segment_clearance_f32": ((n, c, CONTROL_INTERVALS), "f"),
        "oracle_label_valid_u8": ((n, c, CONTROL_INTERVALS), "bu"),
        "projection_label_defined_u8": ((n, c), "bu"),
        "brake_segment_clearance_f32": ((n, c, nb, kb - 1), "f"),
        "brake_oracle_label_valid_u8": ((n, c, nb, kb - 1), "bu"),
        "brake_projection_label_defined_u8": ((n, c, nb), "bu"),
        "brake_projection_domain_valid_u8": ((n, c, nb), "bu"),
        "brake_terminal_clearance_f32": ((n, c, nb), "f"),
        "brake_terminal_oracle_label_valid_u8": ((n, c, nb), "bu"),
        "raw_sensor_record_valid_u8": ((n,), "bu"),
    }
    for name, (shape, kind) in arrays.items():
        _validate_array(archive, name, shape, kind)
    if not np.array_equal(
        np.asarray(archive["observation_frame_index_i32"]),
        np.asarray(archive["observation_frame_index_i32"], dtype=np.int64),
    ):
        raise DatasetContractError("target observation frame indices must be integers")


def validate_target_cache_file(
    path: str | Path, dimensions: CandidateBankDimensions, dense_size: int = 40
) -> None:
    with np.load(Path(path), allow_pickle=False) as archive:
        validate_target_cache(archive, dimensions, dense_size)
