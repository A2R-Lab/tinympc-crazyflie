#!/usr/bin/env python3
"""Export paired HM01B0 joint gate/obstacle expert-rollout transitions.

This is deliberately a thin exporter over ``generate_course_shard``.  It
reuses that module's room sampler, textured collision bars, calibrated camera,
HM01B0 sensor model, and physical trajectory execution; only the episode
family and training-only transition contract are new here.
"""

from __future__ import annotations

import argparse
from collections import Counter
import hashlib
import json
import math
import os
import sys
import tempfile
from pathlib import Path



FAMILIES = ("gate_only", "obstacle_only", "gate_nearby_obstacle")
DRONE_RADIUS_M = 0.10
SHRUNK_OPENING_MARGIN_M = 0.10
SCHEMA_VERSION = "joint_gate_obstacle_rollout_v3_easy_transition"
ACTION_TRACK, ACTION_LEFT, ACTION_RIGHT = 0, 1, 2
ACTION_LATERAL_THRESHOLD_MPS = 0.20
REWARD_WEIGHTS = {
    "course_progress_per_m": 2.0,
    "approach_gate_center_error": -0.40,
    "ordinary_clearance_per_m": -1.25,
    "frame_clearance_per_m": -1.50,
    "action_switch": -0.05,
    "ordinary_collision": -5.0,
    "frame_collision": -5.0,
    "ordered_gate_pass": 5.0,
}
REWARD_CLEARANCE_MARGIN_M = 0.20
# This is deliberately an easier, fixed transition course, not a sampled
# near-gate obstacle.  The full-height obstacle's leading face is 2.05 m after
# the x=4.00 gate plane: 6.20 - 0.30 / 2 - 4.00 = 2.05 m.
EASY_TRANSITION_COURSE = {
    "version": "v3_easy_post_gate_box",
    "description": "NewBee gate followed by a centered full-height box with a 2.05 m recovery gap",
    "gate_center_m": [4.00, 0.30, 1.50],
    "gate_yaw_rad": 0.0,
    "clear_opening_m": [0.45, 0.45],
    "post_gate_box_center_m": [6.20, 0.00, 1.10],
    "post_gate_box_full_dimensions_m": [0.30, 1.00, 2.20],
    "gate_to_box_leading_face_gap_m": 2.05,
    "start_m": [0.80, 0.30, 1.50],
    "finish_m": [8.40, 0.30, 1.50],
}
# CrazySim's NewBeeDrone square gate has a 0.45 m clear aperture.  The rail
# centres are deliberately not used as the pass aperture: they are the visual
# landmark that the deployed gate head must predict.
NEWBEE_GATE = {
    "model": "NewBeeDrone Micro Race Gate - Square",
    "clear_opening_m": [0.450, 0.450],
    # The firmware visual-servo/PnP contract is a square 0.555 m rail-centre
    # landmark.  MuJoCo's rounded collider placement is 0.558 m, but using it
    # as a training target would knowingly disagree with the deployed model.
    "rail_center_span_m": [0.555, 0.555],
    "outer_size_m": [0.666, 0.666],
    # CrazySim collision rails use a 0.025 m half-depth.
    "depth_m": 0.050,
    "asset_root": "/home/cchen/tinympc-crazyflie/tools/crazysim_mujoco/scenes",
    "mesh_parts": [
        "meshes/newbeedrone_gate_top.obj",
        "meshes/newbeedrone_gate_bottom.obj",
        "meshes/newbeedrone_gate_left.obj",
        "meshes/newbeedrone_gate_right.obj",
    ],
    "mesh_scale": [1.0, 0.9, 0.9],
    "texture": "textures/newbeedrone_gate_front_rgba_v1.png",
    "source_asset_sha256": {
        "meshes/newbeedrone_gate_top.obj": "50bb32875b961ccc004e0ce0594d9c7c65fb524d44b3a575834abb50c3121b36",
        "meshes/newbeedrone_gate_bottom.obj": "f42ab8b226bb4a5027134392ba53c8286ac2cfa9800a28ce25ce55c69214db62",
        "meshes/newbeedrone_gate_left.obj": "49369904cbaf7758f1d5fdcffa0af1a85993f60214b0bfa3841ac41bd3cbbf41",
        "meshes/newbeedrone_gate_right.obj": "f1886b33260de3c2bb1feb2c52ffb7dce56b1911f3f3fe09320700dd0402152e",
        "textures/newbeedrone_gate_front_rgba_v1.png": "ae87f32d4ad17e3aea4f101eb511448e0dbe1a8a5bdea9ddd8e83a3dff2217d9",
    },
    "appearance_mapping": "the authoritative CrazySim NewBeeDrone four-piece OBJ mesh and texture",
}
# At an ideal 4.00 m optical range the HM01B0 fy projects the 0.555 m
# rail-centre span to 89.4608 * 0.555 / 4.00 = 12.41 px.  The fixed v3 course
# begins at x=0.80 facing the x=4.00 gate, however, so its actual farthest
# usable optical range is 3.20 m and the corresponding span is 15.52 px
# before the calibrated distortion/remap.  Use 16 px (a 0.48 px margin) to
# validate the real v3 far-frame contract rather than requiring an unobserved
# 4 m pose. This does not select or move layouts after rendering.
FAR_GATE_REFERENCE_RANGE_M = 4.00
V3_FARTHEST_CAMERA_RANGE_M = 3.20
CALIBRATED_RAIL_SPAN_AT_4M_PX = 89.4608171623 * 0.555 / FAR_GATE_REFERENCE_RANGE_M
CALIBRATED_V3_FARTHEST_RAIL_SPAN_PX = 89.4608171623 * 0.555 / V3_FARTHEST_CAMERA_RANGE_M
SMALL_GATE_MAX_SPAN_PX = 16.0


def newbee_gate_geometry(center: np.ndarray, yaw: float) -> dict:
    """Return CrazySim-matched rail collision geometry and rail-centre labels."""
    asset_root = Path(NEWBEE_GATE["asset_root"])
    for relative, expected in NEWBEE_GATE["source_asset_sha256"].items():
        source = asset_root / relative
        if not source.is_file() or hashlib.sha256(source.read_bytes()).hexdigest() != expected:
            raise RuntimeError(f"authoritative NewBee asset changed or is missing: {source}")
    center = np.asarray(center, dtype=float)
    lateral = np.asarray((-math.sin(yaw), math.cos(yaw), 0.0))
    width_span, height_span = NEWBEE_GATE["rail_center_span_m"]
    outer_width, outer_height = NEWBEE_GATE["outer_size_m"]
    rail_width = (outer_width - float(NEWBEE_GATE["clear_opening_m"][0])) / 2.0
    rail_height = (outer_height - float(NEWBEE_GATE["clear_opening_m"][1])) / 2.0
    depth = float(NEWBEE_GATE["depth_m"])
    parts = []
    for name, position, size in (
        # Match the MuJoCo environment colliders: side rails span only the
        # clear-opening height; the top/bottom pieces span the full width.
        ("left", center - lateral * 0.279, (depth, rail_width, float(NEWBEE_GATE["clear_opening_m"][1]))),
        ("right", center + lateral * 0.279, (depth, rail_width, float(NEWBEE_GATE["clear_opening_m"][1]))),
        ("top", center + np.asarray((0.0, 0.0, height_span / 2.0)), (depth, outer_width, rail_height)),
        ("bottom", center - np.asarray((0.0, 0.0, height_span / 2.0)), (depth, outer_width, rail_height)),
    ):
        parts.append({"object_id": f"newbee_gate_{name}", "center_m": position.tolist(),
                      "size_m": list(size), "yaw_rad": float(yaw)})
    rail_corners = [
        (center - lateral * width_span / 2.0 + np.asarray((0.0, 0.0, height_span / 2.0))).tolist(),
        (center + lateral * width_span / 2.0 + np.asarray((0.0, 0.0, height_span / 2.0))).tolist(),
        (center + lateral * width_span / 2.0 - np.asarray((0.0, 0.0, height_span / 2.0))).tolist(),
        (center - lateral * width_span / 2.0 - np.asarray((0.0, 0.0, height_span / 2.0))).tolist(),
    ]
    return {"center_m": center.tolist(), "yaw_rad": float(yaw),
            "opening_m": list(NEWBEE_GATE["clear_opening_m"]), "parts": parts,
            "rail_center_corners_world_m_tl_tr_br_bl": rail_corners,
            "geometry_contract": NEWBEE_GATE,
            "visual_asset": {
                "mesh_parts": [str(asset_root / path) for path in NEWBEE_GATE["mesh_parts"]],
                "mesh_scale": list(NEWBEE_GATE["mesh_scale"]),
                "texture": str(asset_root / NEWBEE_GATE["texture"]),
            }}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--episodes", type=int, default=90)
    # 29000+ is reserved for this v3 dataset and does not overlap the v2
    # 27000-series rollout export.
    parser.add_argument("--layout-start-index", type=int, default=29000)
    parser.add_argument("--seed", type=int, default=20260826)
    # 210 frames retain about 4.9 seconds before the event-aligned gate pass,
    # which includes the NewBee rail span at roughly 10 px with the HM01B0.
    parser.add_argument("--frames", type=int, default=210)
    parser.add_argument("--rt-subframes", type=int, default=1)
    parser.add_argument("--geometry-smoke", action="store_true")
    parser.add_argument("--render-smoke", action="store_true", help="render one gate episode and stop before compaction")
    parser.add_argument("--compact-only", action="store_true", help="compact an existing JSONL dataset without Isaac")
    return parser.parse_args()


def split_for_cluster(seed: int, cluster_id: int) -> str:
    """Split release clusters, never appearances or individual frames."""
    digest = hashlib.sha256(f"{seed}:cluster:{cluster_id}".encode()).digest()
    value = int.from_bytes(digest[:8], "little") / 2**64
    return "train" if value < 0.70 else "validation" if value < 0.85 else "sealed_test"


def source_hashes() -> dict[str, str]:
    """Hash code inputs consumed by this export, alongside asset hashes."""
    root = Path(__file__).resolve().parent
    names = ("export_joint_gate_obstacle_rollouts.py", "generate_course_shard.py",
             "generate_flight_trajectories.py", "crazyflie_rollout.py")
    return {name: hashlib.sha256((root / name).read_bytes()).hexdigest() for name in names}


def point_box_clearance(point: np.ndarray, spec: dict) -> float:
    """Return sphere clearance to a yawed box; course primitives use this ABI."""
    center = np.asarray(spec["center_m"], dtype=float)
    local = np.asarray(point, dtype=float) - center
    yaw = float(spec.get("yaw_rad", 0.0))
    cosine, sine = math.cos(yaw), math.sin(yaw)
    local[:2] = ((cosine, sine), (-sine, cosine)) @ local[:2]
    half = np.asarray(spec["size_m"], dtype=float) / 2.0
    return float(np.linalg.norm(np.maximum(np.abs(local) - half, 0.0)) - DRONE_RADIUS_M)


def gate_plane(gate: dict) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """Return center, forward normal, lateral, and up axes for native gate bars."""
    yaw = float(gate["yaw_rad"])
    return (
        np.asarray(gate["center_m"], dtype=float),
        np.asarray((math.cos(yaw), math.sin(yaw), 0.0)),
        np.asarray((-math.sin(yaw), math.cos(yaw), 0.0)),
        np.asarray((0.0, 0.0, 1.0)),
    )


def gate_label(gate: dict, previous: np.ndarray, current: np.ndarray, passed: bool) -> dict:
    """Compute ordered shrunken-opening passage and frame clearance."""
    center, normal, lateral, up = gate_plane(gate)
    previous_local, current_local = previous - center, current - center
    before, after = float(previous_local @ normal), float(current_local @ normal)
    opening_w, opening_h = (float(value) for value in gate["opening_m"])
    lateral_offset, vertical_offset = float(current_local @ lateral), float(current_local @ up)
    shrunken_w = opening_w - 2.0 * SHRUNK_OPENING_MARGIN_M
    shrunken_h = opening_h - 2.0 * SHRUNK_OPENING_MARGIN_M
    ordered_crossing = (
        not passed and before < 0.0 <= after and shrunken_w > 0.0 and shrunken_h > 0.0
        and abs(lateral_offset) <= shrunken_w / 2.0
        and abs(vertical_offset) <= shrunken_h / 2.0
    )
    frame_clearance = min(point_box_clearance(current, part) for part in gate["parts"])
    corners = []
    for lateral_sign, vertical_sign in ((-1, -1), (1, -1), (1, 1), (-1, 1)):
        corners.append((center + lateral * lateral_sign * opening_w / 2.0 + up * vertical_sign * opening_h / 2.0).tolist())
    return {
        "present": True,
        "center_m": center.tolist(),
        "plane_normal_world": normal.tolist(),
        "opening_m": [opening_w, opening_h],
        "opening_corners_world_m": corners,
        "rail_center_corners_world_m_tl_tr_br_bl": gate["rail_center_corners_world_m_tl_tr_br_bl"],
        "geometry_contract": gate["geometry_contract"],
        "signed_plane_progress_m": after,
        "signed_plane_delta_m": after - before,
        "opening_lateral_offset_m": lateral_offset,
        "opening_vertical_offset_m": vertical_offset,
        "ordered_shrunken_opening_pass": ordered_crossing,
        "frame_clearance_m": frame_clearance,
        "frame_contact": frame_clearance <= 0.0,
    }


def route_progress(point: np.ndarray, route: list[list[float]]) -> float:
    """Project onto the native course route and return monotonic path distance."""
    waypoints = [np.asarray(item, dtype=float) for item in route]
    best_distance, best_progress, accumulated = math.inf, 0.0, 0.0
    for start, end in zip(waypoints[:-1], waypoints[1:]):
        delta = end - start
        length = float(np.linalg.norm(delta))
        if length <= 1.0e-9:
            continue
        fraction = float(np.clip((point - start) @ delta / (length * length), 0.0, 1.0))
        distance = float(np.linalg.norm(point - (start + fraction * delta)))
        if distance < best_distance:
            best_distance, best_progress = distance, accumulated + fraction * length
        accumulated += length
    return best_progress


def normalized_gate_corners(gate: dict, camera_state: dict, calibration: dict) -> tuple[np.ndarray, float]:
    """Project NewBee rail-centre TL/TR/BR/BL labels, never opening corners."""
    if "rail_center_corners_world_m_tl_tr_br_bl" not in gate:
        raise ValueError("joint rollout gate lacks NewBee rail-centre label geometry")
    world = np.asarray(gate["rail_center_corners_world_m_tl_tr_br_bl"], dtype=np.float64)
    eye = np.asarray(camera_state["position_m"], dtype=np.float64)
    right = np.asarray(camera_state["right_world"], dtype=np.float64)
    up_world = np.asarray(camera_state["up_world"], dtype=np.float64)
    forward = np.asarray(camera_state["forward_world"], dtype=np.float64)
    camera_from_world = np.stack((right, -up_world, forward))
    camera_points = (camera_from_world @ (world - eye).T).T
    pixels = cv2.projectPoints(
        camera_points, np.zeros(3), np.zeros(3),
        np.asarray(calibration["camera_matrix"], dtype=np.float64),
        # Match the monotonic simulation coefficients used by the native
        # HM01B0 remap, falling back to measured calibration when unavailable.
        np.asarray(calibration.get("simulation_distortion_coefficients", calibration["distortion_coefficients"]), dtype=np.float64),
    )[0][:, 0]
    by_y = np.argsort(pixels[:, 1])
    top = by_y[:2][np.argsort(pixels[by_y[:2], 0])]
    bottom = by_y[2:][np.argsort(pixels[by_y[2:], 0])[::-1]]
    ordered = pixels[np.concatenate((top, bottom))]
    image_w, image_h = calibration["resolution"]
    visible = float(bool(np.all(camera_points[:, 2] > 0.0) and np.all(ordered[:, 0] >= 0.0)
                    and np.all(ordered[:, 0] < image_w) and np.all(ordered[:, 1] >= 0.0)
                    and np.all(ordered[:, 1] < image_h)))
    normalized = (ordered / np.asarray((image_w, image_h), dtype=np.float64)).astype(np.float32)
    # Training consumers use zero corners whenever the whole opening is not
    # usable; this prevents invalid out-of-frame normalized targets.
    return (normalized if visible else np.zeros((4, 2), dtype=np.float32)), visible


def gate_span_px(corners: np.ndarray, visible: float, calibration: dict) -> float:
    """Largest rail-centre span, used to audit far/small visible gates."""
    if visible < 0.5:
        return 0.0
    width, height = calibration["resolution"]
    return float(max(np.ptp(corners[:, 0]) * width, np.ptp(corners[:, 1]) * height))


def executed_action(velocity: np.ndarray, camera_state: dict) -> int:
    """Classify executed body-local lateral velocity: TRACK/LEFT/RIGHT at 0.20 m/s."""
    body_left = -np.asarray(camera_state["right_world"], dtype=float)
    lateral_velocity = float(np.asarray(velocity, dtype=float) @ body_left)
    if lateral_velocity > ACTION_LATERAL_THRESHOLD_MPS:
        return ACTION_LEFT
    if lateral_velocity < -ACTION_LATERAL_THRESHOLD_MPS:
        return ACTION_RIGHT
    return ACTION_TRACK


def reward_scalar(progress_delta: float, gate: dict, ordinary_clearance: float | None,
                  action: int, previous_action: int | None) -> float:
    """Return the fully privileged scalar reward defined by ``REWARD_WEIGHTS``."""
    reward = REWARD_WEIGHTS["course_progress_per_m"] * progress_delta
    if gate.get("present") and gate["signed_plane_progress_m"] < 0.0:
        width, height = gate["opening_m"]
        normalized_error = math.hypot(gate["opening_lateral_offset_m"] / (width / 2),
                                      gate["opening_vertical_offset_m"] / (height / 2))
        reward += REWARD_WEIGHTS["approach_gate_center_error"] * normalized_error
    for clearance, key in ((ordinary_clearance, "ordinary_clearance_per_m"),
                           (gate.get("frame_clearance_m"), "frame_clearance_per_m")):
        if clearance is not None:
            reward += REWARD_WEIGHTS[key] * max(0.0, REWARD_CLEARANCE_MARGIN_M - clearance)
    if previous_action is not None and action != previous_action:
        reward += REWARD_WEIGHTS["action_switch"]
    if ordinary_clearance is not None and ordinary_clearance <= 0.0:
        reward += REWARD_WEIGHTS["ordinary_collision"]
    if gate.get("frame_contact"):
        reward += REWARD_WEIGHTS["frame_collision"]
    if gate.get("ordered_shrunken_opening_pass"):
        reward += REWARD_WEIGHTS["ordered_gate_pass"]
    return float(reward)


def validate_physical_execution(execution: dict, track: dict) -> dict:
    """Reject any rollout with a high-rate physical collision, not just a camera-frame hit."""
    states = np.asarray(execution.get("_physical_rollout_state_f32"), dtype=float)
    if states.ndim != 2 or states.shape[1] < 3 or not len(states):
        raise RuntimeError("physical execution lacks high-rate state provenance")
    positions = states[:, :3]
    gate_clearance = math.inf
    if track["gates"]:
        gate_clearance = min(
            point_box_clearance(position, part)
            for position in positions for part in track["gates"][0]["parts"]
        )
    obstacle_clearance = math.inf
    if track["obstacles"]:
        obstacle_clearance = min(
            point_box_clearance(position, obstacle)
            for position in positions for obstacle in track["obstacles"]
        )
    if gate_clearance <= 0.0 or obstacle_clearance <= 0.0:
        raise RuntimeError(
            "rejecting physical rollout with high-rate contact: "
            f"gate_clearance={gate_clearance:.4f} obstacle_clearance={obstacle_clearance:.4f}"
        )
    return {"minimum_gate_frame_clearance_m": float(gate_clearance),
            "minimum_ordinary_obstacle_clearance_m": float(obstacle_clearance),
            "sample_count": int(len(positions))}


def validate_expert_records(records: list[dict]) -> None:
    """Reject a rollout set that did not actually realize its expert contract.

    These labels are deliberately checked from the executed records, rather
    than inferred from the requested route or reward.  A controller miss is a
    failed expert episode, not a reason to manufacture a LEFT/RIGHT sample.
    """
    split_alias = {"sealed_test": "test", "train": "train", "validation": "validation"}
    grouped = {name: [] for name in split_alias.values()}
    for record in records:
        if record.get("course_geometry") != EASY_TRANSITION_COURSE:
            raise RuntimeError("record lacks the fixed v3 easy-transition geometry contract")
        grouped[split_alias[record["split"]]].append(record)
    for split, rows in grouped.items():
        small_gate_rows = [row for row in rows if float(row.get("gate_visible", 0.0)) >= 0.5
                           and 1.0 <= float(row.get("gate_rail_center_span_px", math.inf)) <= SMALL_GATE_MAX_SPAN_PX]
        if not small_gate_rows:
            raise RuntimeError(f"{split} lacks a visible far/small NewBee gate (<= {SMALL_GATE_MAX_SPAN_PX:g} px)")
        episodes: dict[tuple[str, str], list[dict]] = {}
        for row in rows:
            gate = row.get("privileged", {}).get("gate", {})
            if gate.get("present"):
                contract = gate.get("geometry_contract", {})
                if (contract.get("clear_opening_m") != NEWBEE_GATE["clear_opening_m"] or
                        contract.get("rail_center_span_m") != NEWBEE_GATE["rail_center_span_m"] or
                        row.get("gate_label_semantics") != "rail_center_tl_tr_br_bl"):
                    raise RuntimeError(f"{split} has wrong NewBee geometry or label semantics")
            episodes.setdefault((row["family"], row["episode_id"]), []).append(row)
        for (family, episode_id), episode_rows in episodes.items():
            if family in ("gate_only", "gate_nearby_obstacle"):
                if not any(row["reward_inputs"]["gate_pass_reward_event"] for row in episode_rows):
                    raise RuntimeError(f"{split}/{episode_id} lacks an executed ordered gate pass")
                visible = sum(float(row["gate_visible"]) >= 0.5 for row in episode_rows)
                if visible < 5:
                    raise RuntimeError(
                        f"{split}/{episode_id} has only {visible} usable gate frames"
                    )
        for family in FAMILIES:
            family_rows = [row for row in rows if row["family"] == family]
            if not family_rows:
                raise RuntimeError(f"empty family in {split}: {family}")
            contacts = [
                row for row in family_rows
                if row["reward_inputs"]["gate_frame_penalty_event"]
                or row["reward_inputs"]["ordinary_obstacle_penalty_event"]
            ]
            if contacts:
                raise RuntimeError(
                    f"safe expert family {family} has {len(contacts)} contact records in {split}"
                )
        for family in ("gate_only", "gate_nearby_obstacle"):
            if not any(row["reward_inputs"]["gate_pass_reward_event"] for row in rows if row["family"] == family):
                raise RuntimeError(f"{split}/{family} lacks an executed ordered gate pass")
        for family in ("obstacle_only", "gate_nearby_obstacle"):
            family_rows = [row for row in rows if row["family"] == family]
            action_counts = Counter(int(row["executed_expert_action"]) for row in family_rows)
            actions = set(action_counts)
            required = {ACTION_LEFT, ACTION_RIGHT}
            if not required <= actions:
                raise RuntimeError(
                    f"{split}/{family} lacks executed avoidance labels: missing {sorted(required - actions)}"
                )
            # Timing jitter changes how many camera pairs fall within each
            # physical dodge, so balance is defined on independently rendered
            # episode directions rather than incidental frame multiplicity.
            # Each split must contain both directions and differ by <= 1
            # episode, with no ambiguous two-sided episode.
            episode_actions: dict[str, set[int]] = {}
            for row in family_rows:
                episode_actions.setdefault(row["episode_id"], set()).add(int(row["executed_expert_action"]))
            direction_counts = Counter()
            for episode_id, episode_action_set in episode_actions.items():
                directions = episode_action_set & required
                if len(directions) != 1:
                    raise RuntimeError(f"{split}/{family}/{episode_id} has ambiguous avoidance direction: {sorted(directions)}")
                direction_counts[next(iter(directions))] += 1
            if not required <= set(direction_counts) or abs(direction_counts[ACTION_LEFT] - direction_counts[ACTION_RIGHT]) > 1:
                raise RuntimeError(f"{split}/{family} has unbalanced LEFT/RIGHT episodes: {dict(direction_counts)}")


def smoke() -> None:
    """Test distorted gate projection and direct NPZ compaction without Isaac."""
    calibration = {"resolution": [160, 160], "camera_matrix": [[90, 0, 80], [0, 90, 80], [0, 0, 1]],
                   "distortion_coefficients": [0, 0, 0, 0, 0]}
    gate = newbee_gate_geometry(np.asarray([2.0, 0.0, 1.0]), 0.0)
    corners, visible = normalized_gate_corners(gate, {"position_m": [0, 0, 1],
        "right_world": [0, -1, 0], "up_world": [0, 0, 1], "forward_world": [1, 0, 0]}, calibration)
    if not visible or not np.all((corners >= 0.0) & (corners <= 1.0)):
        raise RuntimeError("distorted gate-corner geometry smoke failed")
    invisible_gate = newbee_gate_geometry(np.asarray([2.0, 3.0, 1.0]), 0.0)
    invisible_corners, invisible_visible = normalized_gate_corners(invisible_gate, {"position_m": [0, 0, 1],
        "right_world": [0, -1, 0], "up_world": [0, 0, 1], "forward_world": [1, 0, 0]}, calibration)
    if invisible_visible or np.any(invisible_corners):
        raise RuntimeError("invisible gate corners must be zero")
    # The canonical dodge geometry stays comfortably outside the obstacle plus
    # drone envelope on its outward leg, side traversal, and recovery leg.
    obstacle = {"center_m": [0, 0, 0.60], "size_m": [0.44, 0.48, 1.20], "yaw_rad": 0.0}
    safe_route = [(-0.68, 0.0, 0.78), (-0.48, 0.82, 0.78),
                  (0.66, 0.82, 0.78), (0.88, 0.0, 0.78)]
    clearance = min(
        point_box_clearance((1.0 - fraction) * np.asarray(start) + fraction * np.asarray(finish), obstacle)
        for start, finish in zip(safe_route[:-1], safe_route[1:])
        for fraction in np.linspace(0.0, 1.0, 401)
    )
    if clearance <= REWARD_CLEARANCE_MARGIN_M:
        raise RuntimeError(f"safe dodge geometry is too close: {clearance:.3f} m")
    with tempfile.TemporaryDirectory() as temporary:
        root = Path(temporary)
        (root / "frames").mkdir()
        rows = []
        for split in ("train", "validation", "sealed_test"):
            for family_index, family in enumerate(FAMILIES):
                action_cycle = (ACTION_TRACK,) if family == "gate_only" else (ACTION_LEFT, ACTION_RIGHT)
                for action_index, expert_action in enumerate(action_cycle):
                    for sample_index in range(5):
                        paths = []
                        for temporal_index in range(2):
                            path = Path("frames") / f"{split}_{family}_{action_index}_{sample_index}_{temporal_index}.png"
                            cv2.imwrite(str(root / path), np.full((160, 160), family_index * 30, np.uint8))
                            paths.append(str(path))
                        rows.append({"split": split, "release_cluster_id": f"{split}_{family}_{action_index}", "family": family,
                            "course_geometry": EASY_TRANSITION_COURSE,
                            "frame_t_minus_1": paths[0], "frame_t": paths[1], "gate_corners_normalized_tl_tr_br_bl": corners.tolist(),
                            "gate_visible": visible, "gate_label_semantics": "rail_center_tl_tr_br_bl",
                            "gate_rail_center_span_px": 10.0 if sample_index == 4 else gate_span_px(corners, visible, calibration),
                            "obstacle_risk": 0.0, "reward": 0.0, "executed_expert_action": expert_action,
                            "episode_id": f"joint_{family_index * 10 + action_index:05d}", "layout_id": f"joint_{family_index * 10 + action_index:05d}",
                            "layout_seed": family_index, "transition_index": sample_index,
                            "reward_inputs": {"gate_pass_reward_event": family != "obstacle_only" and sample_index == 0,
                                              "gate_frame_penalty_event": False,
                                              "ordinary_obstacle_penalty_event": False},
                            "privileged": {"gate": gate_label(gate, np.asarray([0., 0., 0.]), np.asarray([0., 0., 0.]), False)}})
        (root / "transitions.jsonl").write_text("\n".join(json.dumps(row) for row in rows) + "\n")
        validate_expert_records(rows)
        result = compact_dataset(root)
        if result["per_split"] != {"train": 25, "validation": 25, "test": 25}:
            raise RuntimeError(f"compaction smoke failed: {result}")
        write_canonical_manifest(root, {"schema_version": SCHEMA_VERSION, "splits": result["splits"]})
        canonical = json.loads((root / "manifest.json").read_text())
        for split, contract in canonical["splits"].items():
            if contract["samples"] != 25 or contract["sha256"] != hashlib.sha256((root / f"{split}.npz").read_bytes()).hexdigest():
                raise RuntimeError(f"manifest hash smoke failed for {split}")
    print(json.dumps({"schema_version": SCHEMA_VERSION, "geometry_and_compaction_smoke": "passed"}))


def compact_dataset(output_dir: Path) -> dict:
    """Write direct-training split arrays from provenance JSONL and validate isolation."""
    records = [json.loads(line) for line in (output_dir / "transitions.jsonl").read_text().splitlines() if line]
    if not records:
        raise RuntimeError("cannot compact an empty transition JSONL")
    split_alias = {"sealed_test": "test", "train": "train", "validation": "validation"}
    grouped = {name: [] for name in split_alias.values()}
    cluster_splits: dict[str, str] = {}
    for record in records:
        split = record["split"]
        if split not in split_alias:
            raise RuntimeError(f"unknown split: {split}")
        cluster = record["release_cluster_id"]
        previous = cluster_splits.setdefault(cluster, split)
        if previous != split:
            raise RuntimeError(f"release-cluster leakage: {cluster} spans {previous} and {split}")
        grouped[split_alias[split]].append(record)
    validate_expert_records(records)
    summary = {"per_split": {}, "per_family": {}, "splits": {}}
    family_codes = {name: index for index, name in enumerate(FAMILIES)}
    for split, rows in grouped.items():
        if not rows:
            raise RuntimeError(f"empty required split: {split}")
        family_count = {family: sum(row["family"] == family for row in rows) for family in FAMILIES}
        missing = [family for family, count in family_count.items() if count == 0]
        if missing:
            raise RuntimeError(f"empty family in {split}: {missing}")
        frames = np.empty((len(rows), 2, 160, 160), dtype=np.uint8)
        corners = np.empty((len(rows), 4, 2), dtype=np.float32)
        for index, row in enumerate(rows):
            for temporal_index, key in enumerate(("frame_t_minus_1", "frame_t")):
                image = cv2.imread(str(output_dir / row[key]), cv2.IMREAD_GRAYSCALE)
                if image is None or image.shape != (160, 160):
                    raise RuntimeError(f"bad training image: {row[key]}")
                frames[index, temporal_index] = image
            corners[index] = np.asarray(row["gate_corners_normalized_tl_tr_br_bl"], dtype=np.float32)
        split_path = output_dir / f"{split}.npz"
        np.savez_compressed(
            split_path,
            frames=frames,
            action=np.asarray([row["executed_expert_action"] for row in rows], dtype=np.int64),
            gate_corners=corners,
            gate_visible=np.asarray([row["gate_visible"] for row in rows], dtype=np.float32),
            obstacle_risk=np.asarray([row["obstacle_risk"] for row in rows], dtype=np.float32),
            reward=np.asarray([row["reward"] for row in rows], dtype=np.float32),
            episode_id=np.asarray([int(row["episode_id"].rsplit("_", 1)[1]) for row in rows], dtype=np.int64),
            scene_id=np.asarray([int(row["layout_id"].rsplit("_", 1)[1]) for row in rows], dtype=np.int64),
            layout_seed=np.asarray([row["layout_seed"] for row in rows], dtype=np.uint64),
            family=np.asarray([family_codes[row["family"]] for row in rows], dtype=np.int64),
            transition_index=np.asarray([row["transition_index"] for row in rows], dtype=np.int64),
        )
        summary["per_split"][split] = len(rows)
        summary["splits"][split] = {
            "samples": len(rows),
            "sha256": hashlib.sha256(split_path.read_bytes()).hexdigest(),
        }
        for family, count in family_count.items():
            summary["per_family"].setdefault(family, {})[split] = count
    return summary


def write_canonical_manifest(output_dir: Path, manifest: dict) -> None:
    """Write the canonical external-consumer manifest and compatibility copy."""
    encoded = json.dumps(manifest, indent=2) + "\n"
    (output_dir / "manifest.json").write_text(encoded)
    (output_dir / "dataset_manifest.json").write_text(encoded)


def import_course(output_dir: Path, args: argparse.Namespace):
    """Load the canonical renderer without allowing it to consume this CLI."""
    original = sys.argv
    sys.argv = ["generate_course_shard.py", "--output-dir", str(output_dir / "_course_unused"),
                "--layouts", "1", "--frames", str(args.frames), "--rt-subframes", str(args.rt_subframes)]
    try:
        import generate_course_shard as course
    finally:
        sys.argv = original
    course.args.width = course.args.height = 160
    return course


def family_track(course, layout_index: int, family: str, avoidance_side: float) -> dict:
    """Build the fixed, easier physical gate-to-box transition course.

    ``avoidance_side`` is only used to choose scene/route geometry.  The
    exported action remains the *executed* body-local lateral velocity.
    """
    scenario = "clear_gate" if family != "obstacle_only" else "collision"
    track = course.layout_document(layout_index, scenario, split="unassigned")
    track["joint_family"] = family
    track["obstacles"] = []
    start = np.asarray(EASY_TRANSITION_COURSE["start_m"], dtype=float)
    finish = np.asarray(EASY_TRANSITION_COURSE["finish_m"], dtype=float)
    rng = np.random.default_rng(layout_index + 17)
    # Preserve the native randomized Isaac room/light/material family, while
    # keeping the evaluated physical geometry fixed and documented.  The room
    # randomization is appearance-only; it cannot alter the gate/box gap.
    altitude = float(EASY_TRANSITION_COURSE["gate_center_m"][2])
    track["easy_transition_course"] = EASY_TRANSITION_COURSE
    if family != "obstacle_only":
        gate_center = np.asarray(EASY_TRANSITION_COURSE["gate_center_m"], dtype=float)
        track["gates"] = [newbee_gate_geometry(gate_center, float(EASY_TRANSITION_COURSE["gate_yaw_rad"]))]
        track["appearance"]["gate_texture"] = track["gates"][0]["visual_asset"]["texture"]
        track["appearance"]["joint_appearance_contract"] = {
            "base_domain": "randomized Isaac room/light/material profile from generate_course_shard",
            "gate_texture": NEWBEE_GATE["texture"], "bounded_variation": "native room/light/material appearance seed only",
        }
    if family == "gate_only":
        track["route_waypoints_m"] = [start.tolist(), gate_center.tolist(), finish.tolist()]
        track["expert_route"] = {"avoidance_segment_index": None, "side": None}
        return track

    if family == "obstacle_only":
        track["gates"] = []
        obstacle_x = float(EASY_TRANSITION_COURSE["post_gate_box_center_m"][0])
        prefix = [start]
    else:
        # Cross the aperture on centre, retain the documented 2.05 m clear
        # gap, then dodge the centered box.
        obstacle_x = float(EASY_TRANSITION_COURSE["post_gate_box_center_m"][0])
        prefix = [start, gate_center]
    obstacle_center = EASY_TRANSITION_COURSE["post_gate_box_center_m"]
    obstacle_size = EASY_TRANSITION_COURSE["post_gate_box_full_dimensions_m"]
    course.add_obstacle(
        track["obstacles"], obstacle_center, obstacle_size,
        primitive="box", arrangement="v3_easy_post_gate_full_height_box",
    )
    track["obstacles"][0]["texture"] = course.choose_texture(rng)
    # Move sideways while still well before the obstacle, traverse alongside
    # it, then recover.  This is a physical route, not a label-only edit.
    side_y = float(avoidance_side * 0.92)
    approach = np.asarray((obstacle_x - 0.72, 0.30, altitude))
    side_entry = np.asarray((obstacle_x - 0.50, side_y, altitude))
    side_exit = np.asarray((obstacle_x + 0.70, side_y, altitude))
    recovery = np.asarray((obstacle_x + 0.98, 0.30, altitude))
    route = [*prefix, approach, side_entry, side_exit, recovery, finish]
    track["route_waypoints_m"] = [point.tolist() for point in route]
    # ``side_entry`` is the sole intentional dodge leg.  Side traversal and
    # recovery are hard TRACK so a return-to-centre velocity is never taught as
    # the opposite avoidance command.
    track["expert_route"] = {"avoidance_segment_index": len(prefix), "side": int(avoidance_side)}
    return track


def expert_manifest(trajectory, track: dict, layout_index: int, seed: int, frame_count: int) -> dict:
    """Sample a native physically executed reference while retaining label phases."""
    rng = np.random.default_rng(trajectory.trajectory_seed(seed, layout_index, 0))
    waypoints = [np.asarray(point, dtype=float) for point in track["route_waypoints_m"]]
    nominal_speed = 1.0
    segment_durations = [trajectory.segment_duration(start, finish, nominal_speed) for start, finish in zip(waypoints[:-1], waypoints[1:])]
    segment_starts = np.concatenate(([0.0], np.cumsum(segment_durations)))
    avoidance_segment = track["expert_route"]["avoidance_segment_index"]
    if avoidance_segment is None:
        # The gate is the end of the first path segment.
        event_time = float(segment_starts[1])
        avoidance_window = None
    else:
        # Centre the finite camera window on the explicit dodge, so it retains
        # the preceding gate crossing (if any) and both visual sides of the
        # obstacle.  Segment index is the route edge ending at side_entry.
        event_time = float((segment_starts[avoidance_segment] + segment_starts[avoidance_segment + 1]) / 2.0)
        avoidance_window = [float(segment_starts[avoidance_segment]), float(segment_starts[avoidance_segment + 1])]
    times, dts, positions, velocities, accelerations = trajectory.sample_path(
        waypoints, rng, frame_count, nominal_speed, event_time_s=event_time,
    )
    # These are straight-course avoidance demonstrations.  Keep yaw/camera
    # aligned with the nominal course axis, rather than yawing into the dodge
    # tangent.  Otherwise the camera's body frame rotates with the detour and
    # a real lateral avoidance velocity is reclassified as forward TRACK.
    # ``execute_trajectory`` and ``render_trajectory`` consume these attitude
    # states, so the physical controller, rendered images, and action labels
    # share this exact frame convention.
    course_delta = waypoints[-1] - waypoints[0]
    course_yaw = float(math.atan2(course_delta[1], course_delta[0]))
    yaws = np.full(len(velocities), course_yaw, dtype=float)
    navigation_waypoints = trajectory.mission_waypoints(track)
    states, previous_yaw = [], None
    for index, (time_s, position, velocity, acceleration, yaw) in enumerate(zip(times, positions, velocities, accelerations, yaws)):
        dt_s = 0.0 if index == 0 else float(dts[index - 1])
        yaw_rate = 0.0 if previous_yaw is None else float(math.atan2(math.sin(yaw - previous_yaw), math.cos(yaw - previous_yaw)) / dt_s)
        previous_yaw = yaw
        waypoint = trajectory.active_waypoint(position, navigation_waypoints)
        states.append({"frame_index": index, "time_s": float(time_s), "delta_t_s": dt_s,
                       "position_m": position.tolist(), "velocity_mps": velocity.tolist(),
                       "acceleration_mps2": acceleration.tolist(), "attitude_rpy_rad": [0.0, 0.0, float(yaw)],
                       "angular_velocity_rps": [0.0, 0.0, yaw_rate,
                       ], "active_waypoint_m": waypoint["position_m"], "active_waypoint_kind": waypoint["kind"],
                       "active_waypoint_source": waypoint["source"]})
    return {"layout_id": track["layout_id"], "split": track["split"], "trajectory_id": "trajectory_00000",
            "trajectory_index": 0, "trajectory_type": "joint_safe_expert", "deterministic_seed": int(trajectory.trajectory_seed(seed, layout_index, 0)),
            "nominal_speed_mps": nominal_speed,
            "capture_timing": {"nominal_frame_rate_hz": 30.0, "jitter_fraction": 0.05, "frame_skip_probability": 0.02},
            "mission_goal_m": navigation_waypoints[-1]["position_m"], "navigation_waypoints": navigation_waypoints,
            "waypoints_m": [point.tolist() for point in waypoints], "avoidance_time_window_s": avoidance_window,
            "action_label_frame": "executed body-local lateral velocity; body heading held to nominal course axis",
            "states": states}


def main() -> None:
    args = parse_args()
    global cv2, np
    import cv2
    import numpy as np
    if args.geometry_smoke:
        smoke()
        return
    if args.episodes < (1 if args.render_smoke else 3) or args.frames < 3:
        raise ValueError("insufficient episodes or frames")
    if not args.render_smoke and not args.compact_only and (args.episodes != 90 or args.frames != 210):
        raise ValueError("v3 physical export is fixed at 90 episodes (30/family) and 210 frames")
    if args.compact_only:
        if not args.output_dir.is_dir():
            raise FileNotFoundError(args.output_dir)
        output_dir = args.output_dir.expanduser().resolve()
        compact_summary = compact_dataset(output_dir)
        records = [json.loads(line) for line in (output_dir / "transitions.jsonl").read_text().splitlines() if line]
        counts = {family: len({row["episode_id"] for row in records if row["family"] == family})
                  for family in FAMILIES}
        layouts = []
        for episode_id in sorted({row["episode_id"] for row in records}):
            episode = [row for row in records if row["episode_id"] == episode_id]
            first = episode[0]
            layouts.append({"layout_id": episode_id, "family": first["family"],
                            "split": first["split"], "release_cluster_id": first["release_cluster_id"],
                            "transition_count": len(episode)})
        manifest = {"schema_version": SCHEMA_VERSION, "resolution": [160, 160], "color_space": "grayscale_u8",
                    "temporal_input": "frame_t_minus_1, frame_t", "split_policy": "release_cluster_sha256_70_15_15; appearances never choose split",
                    "families": counts, "layouts": layouts, "compaction": compact_summary,
                    "splits": compact_summary["splits"],
                    "training_contract": {"frames": "uint8[N,2,160,160]", "action": "int64 TRACK=0 LEFT=1 RIGHT=2",
                                          "gate_corners": "float32[N,4,2] rail-center TL/TR/BR/BL normalized", "gate_visible": "float32",
                                          "obstacle_risk": "float32", "reward": "float32"},
                    "reward_weights": REWARD_WEIGHTS,
                    "camera_calibration": json.loads((Path(__file__).parents[1] / "gap8_perception/configs/hm01b0_calibration.json").read_text()),
                    "newbee_geometry_label_appearance_contract": NEWBEE_GATE,
                    "easy_transition_course": EASY_TRANSITION_COURSE,
                    "source_sha256": source_hashes(),
                    "small_gate_coverage": {
                        "maximum_span_px": SMALL_GATE_MAX_SPAN_PX,
                        "rail_center_span_m": NEWBEE_GATE["rail_center_span_m"][0],
                        "ideal_4m_span_px": CALIBRATED_RAIL_SPAN_AT_4M_PX,
                        "v3_farthest_camera_range_m": V3_FARTHEST_CAMERA_RANGE_M,
                        "v3_farthest_span_px": CALIBRATED_V3_FARTHEST_RAIL_SPAN_PX,
                    },
                    "finalization": "compact-only from complete validated physical transitions"}
        write_canonical_manifest(output_dir, manifest)
        (output_dir / "_SUCCESS").write_text(json.dumps({
            "schema_version": SCHEMA_VERSION, "episodes": sum(counts.values()),
            "families": counts, "source_sha256": source_hashes(),
        }, sort_keys=True) + "\n")
        print(json.dumps(compact_summary, indent=2))
        return

    output_dir = args.output_dir.expanduser().resolve()
    if output_dir.exists():
        raise FileExistsError(f"refusing to overwrite existing output: {output_dir}")
    output_dir.mkdir(parents=True)
    (output_dir / "frames").mkdir()
    course = import_course(output_dir, args)
    import generate_flight_trajectories as trajectory
    from crazyflie_rollout import execute_trajectory

    transitions_path = output_dir / "transitions.jsonl"
    layouts, counts = [], {family: 0 for family in FAMILIES}
    # Alternate side per split/family, rather than trusting a random sample to
    # cover both labels in the small held-out partitions.
    side_counts: dict[tuple[str, str], int] = {}
    with transitions_path.open("w") as transitions:
        for episode_index in range(args.episodes):
            family = FAMILIES[episode_index % len(FAMILIES)]
            layout_index = args.layout_start_index + episode_index
            # Each layout is its own release cluster. This preserves strict
            # split isolation while avoiding a 7-layout cluster accidentally
            # coupling an attempted re-render to a previously released scene.
            cluster_id = layout_index
            split = split_for_cluster(args.seed, cluster_id)
            side_key = (split, family)
            avoidance_side = -1.0 if side_counts.get(side_key, 0) % 2 == 0 else 1.0
            side_counts[side_key] = side_counts.get(side_key, 0) + 1
            track = family_track(course, layout_index, family, avoidance_side)
            track["split"] = split
            track["release_cluster_id"] = f"cluster_{cluster_id:05d}"
            layout_id = f"joint_{layout_index:05d}"
            track["layout_id"] = layout_id
            layout_dir = output_dir / "layouts" / layout_id
            layout_dir.mkdir(parents=True)
            (layout_dir / "scene_geometry.json").write_text(json.dumps(track, indent=2) + "\n")
            manifest = expert_manifest(trajectory, track, layout_index, args.seed, args.frames)
            execution = execute_trajectory(manifest)
            physical_acceptance = validate_physical_execution(execution, track)
            manifest["physical_contact_acceptance"] = physical_acceptance
            (layout_dir / "physical_acceptance.json").write_text(
                json.dumps(physical_acceptance, indent=2) + "\n"
            )
            rendered = course.render_trajectory(layout_dir, track, manifest, execution=execution)
            if args.render_smoke:
                (output_dir / "_RENDER_SMOKE_SUCCESS").write_text(json.dumps({
                    "layout_id": layout_id,
                    "visual_asset": track["gates"][0]["visual_asset"],
                    "rendered_frames": rendered["rendered_frame_count"],
                }, indent=2) + "\n")
                return
            frames = [json.loads(line) for line in (layout_dir / "trajectories" / rendered["trajectory_id"] / "frames.jsonl").read_text().splitlines()]
            image_dir = layout_dir / "trajectories" / rendered["trajectory_id"]
            passed = False
            previous_action = None
            for frame_index in range(1, len(frames)):
                prior, current = frames[frame_index - 1], frames[frame_index]
                previous_position = np.asarray(prior["vehicle_state"]["position_m"], dtype=float)
                position = np.asarray(current["vehicle_state"]["position_m"], dtype=float)
                obstacle_clearances = [point_box_clearance(position, item) for item in track["obstacles"]]
                gate = gate_label(track["gates"][0], previous_position, position, passed) if track["gates"] else {"present": False}
                passed = passed or bool(gate.get("ordered_shrunken_opening_pass", False))
                camera_state = current["camera_state"]
                gate_corners, gate_visible = (
                    normalized_gate_corners(track["gates"][0], camera_state, course.camera_calibration)
                    if track["gates"] else (np.zeros((4, 2), dtype=np.float32), 0.0)
                )
                source_a = image_dir / f"rgb_{frame_index - 1:04d}.png"
                source_b = image_dir / f"rgb_{frame_index:04d}.png"
                if not source_a.is_file() or not source_b.is_file():
                    raise RuntimeError(f"missing paired render in {image_dir}")
                relative_a = Path("frames") / f"{layout_id}_{frame_index - 1:04d}.png"
                relative_b = Path("frames") / f"{layout_id}_{frame_index:04d}.png"
                for source, relative in ((source_a, relative_a), (source_b, relative_b)):
                    target = output_dir / relative
                    if not target.exists():
                        image = cv2.imread(str(source), cv2.IMREAD_GRAYSCALE)
                        if image is None or image.shape != (160, 160):
                            raise RuntimeError(f"invalid HM01B0 frame: {source}")
                        cv2.imwrite(str(target), image)
                current_progress = route_progress(position, track["route_waypoints_m"])
                prior_progress = route_progress(previous_position, track["route_waypoints_m"])
                action = (position - previous_position) / max(float(current["delta_t_s"]), 1.0e-6)
                avoidance_window = manifest["avoidance_time_window_s"]
                in_explicit_dodge = (
                    avoidance_window is not None
                    and avoidance_window[0] <= float(current["time_s"]) <= avoidance_window[1]
                )
                # Gate approach/passage, side traversal, and recovery are
                # explicitly TRACK.  Only the executed velocity on the
                # planned outward dodge leg determines LEFT/RIGHT.
                expert_action = (
                    executed_action(np.asarray(current["vehicle_state"]["velocity_mps"], dtype=float), camera_state)
                    if in_explicit_dodge else ACTION_TRACK
                )
                ordinary_clearance = min(obstacle_clearances) if obstacle_clearances else None
                reward = reward_scalar(current_progress - prior_progress, gate, ordinary_clearance,
                                       expert_action, previous_action)
                previous_action = expert_action
                record = {
                    "schema_version": SCHEMA_VERSION, "episode_id": layout_id, "layout_id": layout_id,
                    "family": family, "split": split, "release_cluster_id": track["release_cluster_id"],
                    "course_geometry": EASY_TRANSITION_COURSE,
                    "frame_t_minus_1": str(relative_a), "frame_t": str(relative_b),
                    "action_velocity_world_mps": action.tolist(),
                    "camera_state": camera_state,
                    "gate_corners_normalized_tl_tr_br_bl": gate_corners.tolist(),
                    "gate_visible": gate_visible,
                    "gate_label_semantics": "rail_center_tl_tr_br_bl" if track["gates"] else "not_present",
                    "gate_rail_center_span_px": gate_span_px(gate_corners, gate_visible, course.camera_calibration),
                    "executed_expert_action": expert_action,
                    "executed_expert_action_names": ["TRACK", "LEFT", "RIGHT"],
                    "executed_expert_lateral_threshold_mps": ACTION_LATERAL_THRESHOLD_MPS,
                    "transition_index": frame_index - 1,
                    "layout_seed": int(track["deterministic_seeds"]["scene_geometry"]),
                    "reward": reward,
                    "obstacle_risk": float(max(
                        0.0, REWARD_CLEARANCE_MARGIN_M - (ordinary_clearance if ordinary_clearance is not None else math.inf),
                        REWARD_CLEARANCE_MARGIN_M - gate.get("frame_clearance_m", math.inf),
                    ) / REWARD_CLEARANCE_MARGIN_M),
                    "reward_inputs": {
                        "course_progress_m": current_progress,
                        "course_progress_delta_m": current_progress - prior_progress,
                        "gate_pass_reward_event": bool(gate.get("ordered_shrunken_opening_pass", False)),
                        "gate_frame_penalty_event": bool(gate.get("frame_contact", False)),
                        "ordinary_obstacle_penalty_event": bool(any(value <= 0.0 for value in obstacle_clearances)),
                    },
                    "privileged": {
                        "gate": gate,
                        # ``null`` explicitly means this family has no ordinary
                        # obstacle, avoiding a non-standard JSON infinity.
                        "ordinary_obstacle_clearance_m": ordinary_clearance,
                        "ordinary_obstacle_contact": bool(any(value <= 0.0 for value in obstacle_clearances)),
                        "vehicle_position_m": position.tolist(),
                    },
                }
                transitions.write(json.dumps(record, allow_nan=False) + "\n")
            layouts.append({"layout_id": layout_id, "family": family, "split": split,
                            "release_cluster_id": track["release_cluster_id"], "transition_count": max(0, len(frames) - 1)})
            counts[family] += 1
    compact_summary = compact_dataset(output_dir)
    if counts != {family: 30 for family in FAMILIES}:
        raise RuntimeError(f"v3 export requires exactly 30 accepted physical episodes per family: {counts}")
    manifest = {"schema_version": SCHEMA_VERSION, "resolution": [160, 160], "color_space": "grayscale_u8",
                "temporal_input": "frame_t_minus_1, frame_t", "split_policy": "release_cluster_sha256_70_15_15; appearances never choose split",
                "families": counts, "layouts": layouts, "compaction": compact_summary,
                "splits": compact_summary["splits"],
                "training_contract": {"frames": "uint8[N,2,160,160]", "action": "int64 TRACK=0 LEFT=1 RIGHT=2",
                                      "gate_corners": "float32[N,4,2] rail-center TL/TR/BR/BL normalized", "gate_visible": "float32",
                                      "obstacle_risk": "float32", "reward": "float32"},
                "reward_weights": REWARD_WEIGHTS,
                "camera_calibration": json.loads((Path(__file__).parents[1] / "gap8_perception/configs/hm01b0_calibration.json").read_text()),
                "newbee_geometry_label_appearance_contract": NEWBEE_GATE,
                "easy_transition_course": EASY_TRANSITION_COURSE,
                "source_sha256": source_hashes(),
                "small_gate_coverage": {
                    "maximum_span_px": SMALL_GATE_MAX_SPAN_PX,
                    "rail_center_span_m": NEWBEE_GATE["rail_center_span_m"][0],
                    "ideal_4m_span_px": CALIBRATED_RAIL_SPAN_AT_4M_PX,
                    "v3_farthest_camera_range_m": V3_FARTHEST_CAMERA_RANGE_M,
                    "v3_farthest_span_px": CALIBRATED_V3_FARTHEST_RAIL_SPAN_PX,
                }}
    write_canonical_manifest(output_dir, manifest)
    (output_dir / "_SUCCESS").write_text(json.dumps({
        "schema_version": SCHEMA_VERSION,
        "episodes": args.episodes,
        "families": counts,
        "physical_acceptance": "all exported transitions have zero gate-frame and ordinary-obstacle contacts",
        "source_sha256": source_hashes(),
    }, sort_keys=True) + "\n")


if __name__ == "__main__":
    main()
