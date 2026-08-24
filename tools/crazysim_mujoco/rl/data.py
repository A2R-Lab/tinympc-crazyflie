"""Load retained CrazySim run directories into privileged transition data."""

from __future__ import annotations

import csv
import json
import math
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np

from . import ACTION_LEFT, ACTION_RIGHT, ACTION_TRACK
from .reward import transition_reward

LATENT_NAMES = (
    "progress_m", "cross_track_m", "tangent_speed_mps", "normal_speed_mps",
    "yaw_error_rad", "obstacle_clearance_m", "wall_clearance_m",
)
FRAME_COUNT_TOLERANCE_FRACTION = 0.02
FRAME_COUNT_TOLERANCE_MINIMUM = 2


@dataclass
class RunTransitions:
    frames: np.ndarray
    latent: np.ndarray
    next_latent: np.ndarray
    action: np.ndarray
    reward: np.ndarray
    done: np.ndarray


def _numeric_csv(path: Path) -> dict[str, np.ndarray]:
    with path.open(newline="") as stream:
        rows = list(csv.DictReader(stream))
    if not rows:
        raise ValueError(f"empty CSV: {path}")
    numeric = {}
    for key in rows[0]:
        try:
            numeric[key] = np.asarray(
                [float(row[key]) for row in rows], dtype=np.float64)
        except (TypeError, ValueError):
            continue
    return numeric


def _video_frames(path: Path) -> np.ndarray:
    capture = cv2.VideoCapture(str(path))
    frames = []
    while True:
        ok, image = capture.read()
        if not ok:
            break
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        frames.append(cv2.resize(gray, (160, 160), interpolation=cv2.INTER_AREA))
    capture.release()
    if len(frames) < 3:
        raise ValueError(f"fewer than three frames in {path}")
    return np.asarray(frames, dtype=np.uint8)


def _course_projection(points: np.ndarray, centerline: np.ndarray):
    start = centerline[:-1]
    delta = centerline[1:] - start
    length = np.linalg.norm(delta, axis=1)
    tangent = delta / np.maximum(length[:, None], 1.0e-9)
    cumulative = np.concatenate(([0.0], np.cumsum(length)))
    relative = points[:, None, :] - start[None, :, :]
    fraction = np.clip(
        np.einsum("nsi,si->ns", relative, delta)
        / np.maximum(np.square(length)[None, :], 1.0e-9), 0.0, 1.0)
    projected = start[None, :, :] + fraction[..., None] * delta[None, :, :]
    distance2 = np.sum(np.square(points[:, None, :] - projected), axis=2)
    segment = np.argmin(distance2, axis=1)
    row = np.arange(points.shape[0])
    chosen_tangent = tangent[segment]
    chosen_point = projected[row, segment]
    displacement = points - chosen_point
    signed_cross = np.einsum(
        "ni,ni->n", displacement,
        np.column_stack((-chosen_tangent[:, 1], chosen_tangent[:, 0])))
    progress = cumulative[segment] + fraction[row, segment] * length[segment]
    return progress, signed_cross, chosen_tangent


def _box_clearance(points: np.ndarray, item: dict, radius: float = 0.10):
    center = np.asarray(item["center"], dtype=float)
    half = np.asarray(item["half_size"], dtype=float)
    q = np.abs(points - center) - half
    outside = np.linalg.norm(np.maximum(q, 0.0), axis=1)
    inside = np.minimum(np.maximum(q[:, 0], q[:, 1]), 0.0)
    return outside + inside - radius


def _actions(vision: dict[str, np.ndarray], count: int) -> np.ndarray:
    if "rl_action" in vision:
        candidate = vision["rl_action"][:count]
        rounded = np.rint(candidate)
        if (np.all(np.isfinite(candidate)) and
                np.all(np.abs(candidate - rounded) < 1.0e-6) and
                np.all((rounded >= ACTION_TRACK) & (rounded <= ACTION_RIGHT))):
            return rounded.astype(np.int64)
    collision = vision["collision"][:count]
    steering = vision["steering"][:count]
    return np.where(collision < 0.5, ACTION_TRACK,
                    np.where(steering >= 0.0, ACTION_LEFT, ACTION_RIGHT)).astype(np.int64)


def load_run(run_dir: Path, course_path: Path, camera_fps: float = 30.0):
    run_config = json.loads((run_dir / "run_config.json").read_text())
    delivery_latency_frames = int(run_config.get("vision_latency_frames", 0))
    if delivery_latency_frames < 0:
        raise ValueError(f"negative vision latency in {run_dir}")
    state = _numeric_csv(run_dir / "state.csv")
    vision = _numeric_csv(run_dir / "vision.csv")
    images = _video_frames(run_dir / "fpv_camera.mp4")
    course = json.loads(course_path.read_text())
    vision_count = len(vision["time_s"])
    allowed_difference = max(
        FRAME_COUNT_TOLERANCE_MINIMUM,
        int(math.ceil(FRAME_COUNT_TOLERANCE_FRACTION * vision_count)))
    if abs(images.shape[0] - vision_count) > allowed_difference:
        raise ValueError(
            f"video/vision frame mismatch in {run_dir}: {images.shape[0]} vs "
            f"{vision_count}, tolerance={allowed_difference}")
    count = min(images.shape[0], vision_count)
    if count < 3:
        raise ValueError(f"insufficient aligned samples in {run_dir}")
    # The bridge timestamp is authoritative: it preserves the first-frame
    # offset and any future capture jitter/drop behavior.
    image_time = vision["time_s"][:count]
    state_index = np.searchsorted(state["time_s"], image_time, side="left")
    state_index = np.clip(state_index, 0, len(state["time_s"]) - 1)
    points = np.column_stack((state["x_m"][state_index], state["y_m"][state_index]))
    velocity = np.column_stack((state["vx_mps"][state_index], state["vy_mps"][state_index]))
    centerline = np.asarray(course["centerline"], dtype=float)
    progress, cross, tangent = _course_projection(points, centerline)
    normal = np.column_stack((-tangent[:, 1], tangent[:, 0]))
    tangent_speed = np.einsum("ni,ni->n", velocity, tangent)
    normal_speed = np.einsum("ni,ni->n", velocity, normal)
    qw, qx, qy, qz = (state[name][state_index] for name in ("qw", "qx", "qy", "qz"))
    yaw = np.arctan2(2.0 * (qw * qz + qx * qy),
                     1.0 - 2.0 * (qy * qy + qz * qz))
    path_yaw = np.arctan2(tangent[:, 1], tangent[:, 0])
    yaw_error = np.arctan2(np.sin(yaw - path_yaw), np.cos(yaw - path_yaw))
    obstacle = np.min(np.column_stack([
        _box_clearance(points, item) for item in course.get("obstacles", [])
    ]), axis=1)
    wall = np.min(np.column_stack([
        _box_clearance(points, item) for item in course.get("walls", [])
    ]), axis=1)
    latent = np.column_stack((progress, cross, tangent_speed, normal_speed,
                              yaw_error, obstacle, wall)).astype(np.float32)
    actions = _actions(vision, count)
    contacts = state.get("contacts", np.zeros_like(state["time_s"]))[state_index] > 0
    total_length = float(np.sum(np.linalg.norm(np.diff(centerline, axis=0), axis=1)))
    complete = progress >= 0.99 * total_length
    # An inference computed at t is released after the configured frame delay.
    # Model its consequence at t + latency + 1 rather than attributing it to
    # the immediate transition. Behavior cloning still uses the action at t.
    stride = delivery_latency_frames + 1
    transition_count = count - stride
    if transition_count < 2:
        raise ValueError(f"run too short for latency={delivery_latency_frames}: {run_dir}")
    current_index = np.arange(transition_count)
    next_index = current_index + stride
    current_actions = actions[current_index]
    previous_actions = np.concatenate(([current_actions[0]], current_actions[:-1]))
    reward = transition_reward(
        progress[next_index] - progress[current_index], cross[next_index],
        tangent_speed[next_index],
        np.minimum(obstacle[next_index], wall[next_index]), current_actions,
        previous_actions, contacts[next_index], complete[next_index])
    done = np.logical_or(contacts[next_index], complete[next_index]).astype(np.float32)
    # A retained run boundary is terminal even when it ended by timeout.
    done[-1] = 1.0
    paired = np.stack((np.concatenate((images[:1], images[:-1])), images), axis=1)
    return RunTransitions(paired[current_index], latent[current_index],
                          latent[next_index], current_actions,
                          reward, done)


def concatenate_runs(run_dirs: list[Path], course_path: Path):
    runs = [load_run(path, course_path) for path in run_dirs]
    return RunTransitions(*(np.concatenate([getattr(run, field) for run in runs])
                            for field in RunTransitions.__dataclass_fields__))
