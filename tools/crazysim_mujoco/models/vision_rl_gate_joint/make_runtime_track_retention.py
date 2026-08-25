#!/usr/bin/env python3
"""Build corrective TRACK-only pairs from one or more held-out smoke failures.

The source video is the bridge's complete post-HM01B0 160x160 stream.  Frames
are selected only while the measured vehicle remains at least a fixed distance
before the known gate plane, where the course contains no ordinary obstacle and
the privileged corrective action is unambiguously TRACK.  The H.264-decoded
frames are training-only DAgger retention, never evaluation evidence.
"""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
from pathlib import Path

import cv2
import numpy as np

RAIL_CENTER_SPAN_M = 0.555
CAMERA_OFFSET_BODY_M = np.asarray([0.0, 0.0, 0.01], dtype=np.float64)
CORNER_ORDER = ["TL", "TR", "BR", "BL"]


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as source:
        for block in iter(lambda: source.read(1 << 20), b""):
            digest.update(block)
    return digest.hexdigest()


def canonical_json(value) -> str:
    return json.dumps(value, sort_keys=True, separators=(",", ":"))


def rotate_body_to_world(quaternion: np.ndarray, vector: np.ndarray) -> np.ndarray:
    """Rotate body vectors with scalar-first body-to-world quaternions."""
    quaternion = np.asarray(quaternion, dtype=np.float64)
    vector = np.asarray(vector, dtype=np.float64)
    norm = np.linalg.norm(quaternion)
    if not np.isfinite(norm) or norm <= 1.0e-12:
        raise RuntimeError("state contains an invalid body-to-world quaternion")
    quaternion = quaternion / norm
    xyz = quaternion[1:]
    return vector + 2.0 * np.cross(xyz, np.cross(xyz, vector) + quaternion[0] * vector)


def interpolate_pose(time_s: float, times: np.ndarray, positions: np.ndarray,
                     quaternions: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Interpolate position and shortest-path normalized quaternion at one time."""
    if time_s < times[0] or time_s > times[-1]:
        raise RuntimeError("selected camera time falls outside state.csv")
    upper = int(np.searchsorted(times, time_s, side="right"))
    if upper == 0:
        return positions[0].copy(), quaternions[0] / np.linalg.norm(quaternions[0])
    if upper == len(times):
        return positions[-1].copy(), quaternions[-1] / np.linalg.norm(quaternions[-1])
    lower = upper - 1
    alpha = float((time_s - times[lower]) / (times[upper] - times[lower]))
    position = positions[lower] + alpha * (positions[upper] - positions[lower])
    first, second = quaternions[lower].copy(), quaternions[upper].copy()
    first /= np.linalg.norm(first)
    second /= np.linalg.norm(second)
    if float(np.dot(first, second)) < 0.0:
        second = -second
    quaternion = (1.0 - alpha) * first + alpha * second
    quaternion /= np.linalg.norm(quaternion)
    return position, quaternion


def calibration_contract(config: dict) -> dict:
    """Extract the projection-relevant HM01B0 run_config contract exactly."""
    calibration = config.get("camera_calibration")
    if not isinstance(calibration, dict):
        raise RuntimeError("run_config lacks camera_calibration")
    required = ("model", "resolution", "fx_px", "fy_px", "cx_px", "cy_px",
                "distortion_model", "simulation_distortion_coefficients", "acquisition")
    if any(key not in calibration for key in required):
        raise RuntimeError("run_config camera_calibration is incomplete")
    acquisition = calibration["acquisition"]
    offset = acquisition.get("ai_deck_optical_center_body_m") if isinstance(acquisition, dict) else None
    if calibration["model"] != "Himax HM01B0" or calibration["resolution"] != [160, 160]:
        raise RuntimeError("runtime retention requires the 160x160 Himax HM01B0 calibration")
    if calibration["distortion_model"] != "opencv_plumb_bob":
        raise RuntimeError("runtime retention requires OpenCV plumb-bob distortion")
    if offset != CAMERA_OFFSET_BODY_M.tolist():
        raise RuntimeError("unexpected AI-deck optical-center body offset")
    numeric = [calibration[key] for key in ("fx_px", "fy_px", "cx_px", "cy_px")]
    distortion = calibration["simulation_distortion_coefficients"]
    if (len(distortion) != 5 or not np.all(np.isfinite(numeric + distortion)) or
            float(calibration["fx_px"]) <= 0.0 or float(calibration["fy_px"]) <= 0.0):
        raise RuntimeError("invalid HM01B0 projection calibration")
    return {
        "model": calibration["model"],
        "resolution": list(calibration["resolution"]),
        "fx_px": float(calibration["fx_px"]),
        "fy_px": float(calibration["fy_px"]),
        "cx_px": float(calibration["cx_px"]),
        "cy_px": float(calibration["cy_px"]),
        "distortion_model": calibration["distortion_model"],
        "simulation_distortion_coefficients": [float(value) for value in distortion],
    }


def gate_world_corners(gate: dict) -> np.ndarray:
    """Return authoritative NewBee rail centers in TL/TR/BR/BL gate order."""
    center = np.asarray(gate.get("center"), dtype=np.float64)
    normal_xy = np.asarray(gate.get("normal"), dtype=np.float64)
    opening = np.asarray(gate.get("opening"), dtype=np.float64)
    if (center.shape != (3,) or normal_xy.shape != (2,) or opening.shape != (2,) or
            np.any(~np.isfinite(center)) or np.any(~np.isfinite(normal_xy)) or
            np.any(~np.isfinite(opening)) or np.any(opening <= 0.0)):
        raise RuntimeError("course gate center/normal/opening contract is invalid")
    normal_norm = np.linalg.norm(normal_xy)
    if normal_norm <= 1.0e-12:
        raise RuntimeError("course gate normal must be nonzero")
    normal_xy /= normal_norm
    lateral = np.asarray([-normal_xy[1], normal_xy[0], 0.0])
    up = np.asarray([0.0, 0.0, 1.0])
    half = RAIL_CENTER_SPAN_M / 2.0
    return np.asarray([
        center + half * lateral + half * up,
        center - half * lateral + half * up,
        center - half * lateral - half * up,
        center + half * lateral - half * up,
    ])


def project_gate_corners(position: np.ndarray, quaternion: np.ndarray, gate: dict,
                         calibration: dict) -> tuple[np.ndarray, np.float32]:
    """Project world rail centers into the distorted runtime camera image."""
    camera_world = position + rotate_body_to_world(quaternion, CAMERA_OFFSET_BODY_M)
    world_delta = gate_world_corners(gate) - camera_world
    # Inverse unit-quaternion rotation: world -> body.
    inverse = np.asarray([quaternion[0], -quaternion[1], -quaternion[2], -quaternion[3]])
    body_points = np.asarray([rotate_body_to_world(inverse, vector) for vector in world_delta])
    # OpenCV camera coordinates: X is image-right, Y image-down, Z depth.
    camera_points = np.column_stack((-body_points[:, 1], -body_points[:, 2], body_points[:, 0]))
    matrix = np.asarray([
        [calibration["fx_px"], 0.0, calibration["cx_px"]],
        [0.0, calibration["fy_px"], calibration["cy_px"]],
        [0.0, 0.0, 1.0],
    ], dtype=np.float64)
    distortion = np.asarray(calibration["simulation_distortion_coefficients"], dtype=np.float64)
    pixels = cv2.projectPoints(camera_points, np.zeros(3), np.zeros(3), matrix, distortion)[0][:, 0]
    width, height = calibration["resolution"]
    visible = bool(
        np.all(np.isfinite(camera_points)) and np.all(np.isfinite(pixels)) and
        np.all(camera_points[:, 2] > 0.0) and
        np.all((pixels[:, 0] >= 0.0) & (pixels[:, 0] < width)) and
        np.all((pixels[:, 1] >= 0.0) & (pixels[:, 1] < height))
    )
    if not visible:
        return np.zeros((4, 2), dtype=np.float32), np.float32(0.0)
    return (pixels / np.asarray([width, height])).astype(np.float32), np.float32(1.0)


def collect_run(run: Path, course: dict, minimum_gate_plane_distance_m: float):
    """Validate and collect one smoke run without conflating its frame clock."""
    run = run.resolve()
    video, state_path, config_path = run / "fpv_camera.mp4", run / "state.csv", run / "run_config.json"
    for path in (video, state_path, config_path):
        if not path.is_file():
            raise FileNotFoundError(path)
    config = json.loads(config_path.read_text())
    if config.get("course") != "gate_obstacle_poc" or config.get("random_seed") != 3199:
        raise RuntimeError("corrective retention must come from the isolated seed-3199 gate smoke")
    if config.get("camera_fps") != 30.0 or config.get("vision_adapter") != "joint_gate_rl":
        raise RuntimeError("unexpected corrective camera/runtime contract")
    if len(course.get("gates", [])) != 1:
        raise RuntimeError("corrective course must contain exactly one known gate")
    gate = course["gates"][0]
    gate_x = float(gate["center"][0])
    calibration = calibration_contract(config)
    with state_path.open(newline="") as source:
        state_rows = list(csv.DictReader(source))
    if len(state_rows) < 2:
        raise RuntimeError("corrective state.csv is too short")
    times = np.asarray([float(row["time_s"]) for row in state_rows], dtype=np.float64)
    positions = np.asarray([[float(row[key]) for key in ("x_m", "y_m", "z_m")]
                            for row in state_rows], dtype=np.float64)
    quaternions = np.asarray([[float(row[key]) for key in ("qw", "qx", "qy", "qz")]
                              for row in state_rows], dtype=np.float64)
    contacts = np.asarray([int(row["contacts"]) for row in state_rows], dtype=np.int64)
    if (np.any(~np.isfinite(times)) or np.any(np.diff(times) <= 0.0) or
            np.any(~np.isfinite(positions)) or np.any(~np.isfinite(quaternions)) or
            np.any(np.linalg.norm(quaternions, axis=1) <= 1.0e-12)):
        raise RuntimeError("corrective state pose/time contract is invalid")
    x_position = positions[:, 0]
    cutoff_x = gate_x - minimum_gate_plane_distance_m
    if float(np.max(x_position)) < cutoff_x:
        raise RuntimeError("corrective source ended before covering the pre-gate cutoff")

    capture = cv2.VideoCapture(str(video))
    fps = float(capture.get(cv2.CAP_PROP_FPS))
    width = int(capture.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
    if abs(fps - 30.0) > 1.0e-6 or (width, height) != (160, 160):
        raise RuntimeError(f"unexpected corrective video contract: {width}x{height} at {fps:g} Hz")
    decoded = []
    while True:
        ok, frame = capture.read()
        if not ok:
            break
        decoded.append(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY))
    capture.release()
    if len(decoded) < 100:
        raise RuntimeError(f"corrective camera stream is too short: {len(decoded)} frames")
    decoded = np.asarray(decoded, dtype=np.uint8)
    frame_indices, frame_times, pairs, gate_corners, gate_visible = [], [], [], [], []
    for current in range(1, len(decoded)):
        time_s = current / fps
        x_m = float(np.interp(time_s, times, x_position))
        if x_m > cutoff_x:
            continue
        if float(np.interp(time_s, times, contacts)) != 0.0:
            raise RuntimeError("a selected pre-gate corrective frame contains contact")
        pairs.append(np.stack((decoded[current - 1], decoded[current])))
        frame_indices.append(current)
        frame_times.append(time_s)
        position, quaternion = interpolate_pose(time_s, times, positions, quaternions)
        corners, visible = project_gate_corners(position, quaternion, gate, calibration)
        gate_corners.append(corners)
        gate_visible.append(visible)
    if len(pairs) < 100:
        raise RuntimeError("too few unambiguous pre-gate corrective frames")
    return {
        "frames": np.asarray(pairs, dtype=np.uint8),
        "frame_indices": np.asarray(frame_indices, dtype=np.int64),
        "frame_times": np.asarray(frame_times, dtype=np.float64),
        "gate_corners": np.asarray(gate_corners, dtype=np.float32),
        "gate_visible": np.asarray(gate_visible, dtype=np.float32),
        "maximum_source_x_m": float(np.max(np.interp(frame_times, times, x_position))),
        "gate_x_m": gate_x,
        "calibration": calibration,
        "provenance": {
            "path": str(run),
            "video_sha256": sha256(video),
            "state_sha256": sha256(state_path),
            "run_config_sha256": sha256(config_path),
        },
    }


def write_retention(output: Path, course_path: Path, collected: list[dict],
                    minimum_gate_plane_distance_m: float) -> dict:
    """Concatenate independently-clocked runs into one cumulative artifact."""
    if not collected:
        raise RuntimeError("at least one collected runtime correction is required")
    source_run_index = np.concatenate([
        np.full(len(item["frames"]), index, dtype=np.int64)
        for index, item in enumerate(collected)
    ])
    selection_rule = (
        "post-HM01B0 H.264-decoded adjacent frames while state.x <= "
        f"gate_x-{minimum_gate_plane_distance_m:g}m; privileged corrective action TRACK"
    )
    course = json.loads(course_path.read_text())
    gate = course["gates"][0]
    projection = {
        "schema": "runtime_gate_projection_v1",
        "source": "interpolated state.csv body pose + run_config HM01B0 calibration + course gate geometry",
        "corner_order": CORNER_ORDER,
        "rail_center_span_m": RAIL_CENTER_SPAN_M,
        "course_gate": {"center": gate["center"], "normal": gate["normal"], "opening": gate["opening"]},
        "ai_deck_optical_center_body_m": CAMERA_OFFSET_BODY_M.tolist(),
        "camera_axes": {"u": "-body_y", "v": "-body_z", "depth": "+body_x"},
        "projection": "cv2.projectPoints with zero extrinsics and simulation distortion",
        "normalization_divisor_px": [160, 160],
        "runs": [item["calibration"] for item in collected],
    }
    output.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(
        output,
        frames=np.concatenate([item["frames"] for item in collected]),
        expert_action=np.zeros(len(source_run_index), dtype=np.int64),
        source_run_index=source_run_index,
        source_frame_index=np.concatenate([item["frame_indices"] for item in collected]),
        source_time_s=np.concatenate([item["frame_times"] for item in collected]),
        gate_corners=np.concatenate([item["gate_corners"] for item in collected]).astype(np.float32),
        gate_visible=np.concatenate([item["gate_visible"] for item in collected]).astype(np.float32),
        source_runs_json=np.asarray(canonical_json([item["provenance"] for item in collected])),
        gate_projection_json=np.asarray(canonical_json(projection)),
        source_course_sha256=np.asarray(sha256(course_path)),
        selection_rule=np.asarray(selection_rule),
    )
    return {
        "output": str(output.resolve()),
        "samples": int(len(source_run_index)),
        "runs": [{"path": item["provenance"]["path"],
                  "samples": int(len(item["frames"])),
                  "time_range_s": [float(item["frame_times"][0]), float(item["frame_times"][-1])],
                  "maximum_source_x_m": item["maximum_source_x_m"]}
                 for item in collected],
        "gate_x_m": collected[0]["gate_x_m"],
        "source_course_sha256": sha256(course_path),
        "sha256": sha256(output),
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run", type=Path, required=True, action="append",
                        help="repeat for each iterative smoke run, oldest first")
    parser.add_argument("--course", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--minimum-gate-plane-distance-m", type=float, default=0.40)
    args = parser.parse_args()
    course_path = args.course.resolve()
    if not course_path.is_file():
        raise FileNotFoundError(course_path)
    course = json.loads(course_path.read_text())
    if len(course.get("gates", [])) != 1:
        raise RuntimeError("corrective course must contain exactly one known gate")
    resolved_runs = [path.resolve() for path in args.run]
    if len(set(resolved_runs)) != len(resolved_runs):
        raise RuntimeError("duplicate --run would silently overweight one correction")
    collected = [collect_run(run, course, args.minimum_gate_plane_distance_m)
                 for run in resolved_runs]
    print(json.dumps(write_retention(args.output, course_path, collected,
                                     args.minimum_gate_plane_distance_m), indent=2))


if __name__ == "__main__":
    main()
