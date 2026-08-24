#!/usr/bin/env python3
"""Generate seed-reproducible paired-frame corridor avoidance demonstrations.

This tool deliberately uses MuJoCo directly.  It does not start CrazySim, build
firmware, or require Docker.  Analytic trajectories provide privileged labels;
the rendered HM01B0-like image pairs are the observation available to learning.

Analytic labels are byte-deterministic.  GPU rasterization is semantically
deterministic but may vary at a tiny number of edge pixels by one 5-bit level;
artifact hashes therefore record provenance rather than predict a fixed digest.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Dict, Iterable, List, Mapping, Sequence, Tuple
import zipfile

import numpy as np

try:
    from . import ACTION_COUNT, ACTION_LEFT, ACTION_RIGHT, ACTION_TRACK
except ImportError:  # Direct script execution.
    ACTION_TRACK, ACTION_LEFT, ACTION_RIGHT, ACTION_COUNT = 0, 1, 2, 3


IMAGE_SIZE = 160
CAMERA_FOV_Y_DEG = 83.6091095
LATENT_NAMES = (
    "progress_m",
    "cross_track_m",
    "tangent_speed_mps",
    "normal_speed_mps",
    "yaw_error_rad",
    "obstacle_clearance_m",
    "wall_clearance_m",
)
ACTION_NAMES = ("TRACK", "LEFT", "RIGHT")
OUTCOMES = ("success", "contact", "near_miss", "timeout")


@dataclass(frozen=True)
class DatasetConfig:
    output: str
    seed: int = 20260824
    episodes: int = 60
    samples_per_episode: int = 720
    validation_fraction: float = 0.2
    layout_variants: int = 12
    appearance_variants: int = 6
    noise_variants: int = 3
    corridor_length_m: float = 10.0
    corridor_half_width_m: float = 1.0
    camera_height_m: float = 0.50
    drone_radius_m: float = 0.10

    def validate(self) -> None:
        if self.episodes < 4:
            raise ValueError("episodes must be at least 4")
        if self.samples_per_episode < 12:
            raise ValueError("samples_per_episode must be at least 12")
        if not 0.0 < self.validation_fraction < 1.0:
            raise ValueError("validation_fraction must be between zero and one")
        if min(self.layout_variants, self.appearance_variants, self.noise_variants) < 1:
            raise ValueError("variant counts must be positive")
        if self.corridor_half_width_m <= self.drone_radius_m:
            raise ValueError("corridor is too narrow for the drone radius")


@dataclass(frozen=True)
class Layout:
    layout_id: int
    corridor_half_width_m: float
    corridor_height_m: float
    obstacle_x_m: float
    obstacle_side: int  # -1: lower wall, +1: upper wall
    obstacle_half_x_m: float
    obstacle_intrusion_m: float
    obstacle_height_m: float

    @property
    def obstacle_center_y_m(self) -> float:
        return self.obstacle_side * (
            self.corridor_half_width_m - 0.5 * self.obstacle_intrusion_m
        )

    @property
    def obstacle_half_y_m(self) -> float:
        return 0.5 * self.obstacle_intrusion_m

    @property
    def pass_sign(self) -> float:
        return float(-self.obstacle_side)


def _rng(*parts: int) -> np.random.Generator:
    return np.random.default_rng(np.random.SeedSequence([int(p) for p in parts]))


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1 << 20), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _save_npz_deterministic(path: Path, arrays: Mapping[str, np.ndarray]) -> None:
    """Write a byte-reproducible, np.load-compatible compressed archive."""
    with zipfile.ZipFile(path, "w", compression=zipfile.ZIP_DEFLATED, compresslevel=6) as archive:
        for name in sorted(arrays):
            info = zipfile.ZipInfo(f"{name}.npy", date_time=(1980, 1, 1, 0, 0, 0))
            info.compress_type = zipfile.ZIP_DEFLATED
            info.external_attr = 0o600 << 16
            # Stream large frame arrays instead of materializing another copy in
            # memory; fixed ZIP metadata still makes the bytes reproducible.
            with archive.open(info, "w", force_zip64=True) as member:
                np.lib.format.write_array(
                    member, np.ascontiguousarray(arrays[name]), allow_pickle=False
                )


def _smoothstep(u: np.ndarray) -> np.ndarray:
    u = np.clip(u, 0.0, 1.0)
    return u * u * u * (u * (6.0 * u - 15.0) + 10.0)


def _bump(x: np.ndarray, obstacle_x: float, amplitude: float) -> np.ndarray:
    start, finish = obstacle_x - 1.9, obstacle_x + 1.35
    y = np.zeros_like(x)
    rising = (x > start) & (x <= obstacle_x)
    falling = (x > obstacle_x) & (x < finish)
    y[rising] = amplitude * _smoothstep((x[rising] - start) / (obstacle_x - start))
    y[falling] = amplitude * (1.0 - _smoothstep((x[falling] - obstacle_x) / (finish - obstacle_x)))
    return y


def _box_clearance(x: np.ndarray, y: np.ndarray, layout: Layout, radius: float) -> np.ndarray:
    dx = np.abs(x - layout.obstacle_x_m) - layout.obstacle_half_x_m
    dy = np.abs(y - layout.obstacle_center_y_m) - layout.obstacle_half_y_m
    outside = np.hypot(np.maximum(dx, 0.0), np.maximum(dy, 0.0))
    inside = np.minimum(np.maximum(dx, dy), 0.0)
    return outside + inside - radius


def _layout(
    seed: int, layout_id: int, variants: int, corridor_half_width_m: float = 1.0
) -> Layout:
    local = _rng(seed, 11, layout_id % variants)
    side = -1 if layout_id % 2 == 0 else 1
    return Layout(
        layout_id=layout_id % variants,
        corridor_half_width_m=float(
            corridor_half_width_m + local.uniform(-0.10, 0.10)
        ),
        corridor_height_m=float(local.uniform(1.85, 2.20)),
        obstacle_x_m=float(local.uniform(3.0, 4.6)),
        obstacle_side=side,
        obstacle_half_x_m=float(local.uniform(0.13, 0.24)),
        obstacle_intrusion_m=float(local.uniform(0.62, 1.02)),
        obstacle_height_m=float(local.uniform(0.9, 1.35)),
    )


def _amplitude_for_clearance(
    x: np.ndarray, layout: Layout, radius: float, target_clearance: float
) -> float:
    """Bisection finds a repeatable path with the requested closest clearance."""
    lo, hi = 0.0, 1.55
    for _ in range(48):
        mid = 0.5 * (lo + hi)
        y = _bump(x, layout.obstacle_x_m, layout.pass_sign * mid)
        clearance = float(np.min(_box_clearance(x, y, layout, radius)))
        if clearance < target_clearance:
            lo = mid
        else:
            hi = mid
    return layout.pass_sign * hi


def _euler_wxyz(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return np.asarray(
        [
            cr * cp * cy + sr * sp * sy,
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
        ],
        dtype=np.float64,
    )


def build_corridor_xml(config: DatasetConfig, layout: Layout, appearance_id: int) -> str:
    palette = (
        ((0.43, 0.44, 0.45), (0.82, 0.80, 0.75), (0.75, 0.18, 0.10)),
        ((0.31, 0.35, 0.38), (0.70, 0.76, 0.78), (0.12, 0.35, 0.76)),
        ((0.47, 0.42, 0.36), (0.89, 0.84, 0.73), (0.74, 0.55, 0.08)),
        ((0.25, 0.28, 0.25), (0.72, 0.78, 0.69), (0.65, 0.12, 0.55)),
        ((0.38, 0.35, 0.42), (0.83, 0.76, 0.86), (0.05, 0.58, 0.50)),
        ((0.35, 0.39, 0.45), (0.85, 0.87, 0.90), (0.82, 0.29, 0.08)),
    )
    floor, wall, obstacle = palette[appearance_id % len(palette)]
    light = 0.72 + 0.08 * (appearance_id % 4)
    rgba = lambda rgb: " ".join(f"{c:.3f}" for c in (*rgb, 1.0))
    half_length = 0.5 * config.corridor_length_m
    wall_y = layout.corridor_half_width_m + 0.06
    return f"""<mujoco model="procedural_avoidance">
  <compiler angle="degree"/>
  <option gravity="0 0 -9.81"/>
  <visual><global offwidth="{IMAGE_SIZE}" offheight="{IMAGE_SIZE}"/></visual>
  <worldbody>
    <light pos="{half_length:.4f} 0 2.7" dir="0 0 -1" diffuse="{light:.3f} {light:.3f} {light:.3f}"/>
    <geom name="floor" type="box" pos="{half_length:.4f} 0 -0.055" size="{half_length:.4f} {wall_y + 0.2:.4f} 0.05" rgba="{rgba(floor)}"/>
    <geom name="left_wall" type="box" pos="{half_length:.4f} {wall_y:.4f} 1.0" size="{half_length:.4f} 0.06 1.0" rgba="{rgba(wall)}"/>
    <geom name="right_wall" type="box" pos="{half_length:.4f} {-wall_y:.4f} 1.0" size="{half_length:.4f} 0.06 1.0" rgba="{rgba(wall)}"/>
    <geom name="end_wall" type="box" pos="{config.corridor_length_m:.4f} 0 1.0" size="0.06 {wall_y:.4f} 1.0" rgba="{rgba(wall)}"/>
    <geom name="ceiling" type="box" pos="{half_length:.4f} 0 {layout.corridor_height_m:.4f}" size="{half_length:.4f} {wall_y:.4f} 0.04" rgba="{rgba(wall)}"/>
    <geom name="obstacle" type="box" pos="{layout.obstacle_x_m:.4f} {layout.obstacle_center_y_m:.4f} {layout.obstacle_height_m / 2:.4f}" size="{layout.obstacle_half_x_m:.4f} {layout.obstacle_half_y_m:.4f} {layout.obstacle_height_m / 2:.4f}" rgba="{rgba(obstacle)}"/>
    <body name="camera_mount" mocap="true" pos="0 0 {config.camera_height_m:.4f}">
      <camera name="fpv" pos="0 0 0" euler="0 -90 -90" fovy="{CAMERA_FOV_Y_DEG}"/>
    </body>
  </worldbody>
</mujoco>"""


class CorridorRenderer:
    def __init__(self, xml: str):
        os.environ.setdefault("MUJOCO_GL", "egl")
        import mujoco

        self._mujoco = mujoco
        self._model = mujoco.MjModel.from_xml_string(xml)
        self._data = mujoco.MjData(self._model)
        self._renderer = mujoco.Renderer(self._model, height=IMAGE_SIZE, width=IMAGE_SIZE)

    def render(self, position: Sequence[float], euler: Sequence[float]) -> np.ndarray:
        self._data.mocap_pos[0] = position
        self._data.mocap_quat[0] = _euler_wxyz(*euler)
        self._mujoco.mj_forward(self._model, self._data)
        self._renderer.update_scene(self._data, camera="fpv")
        rgb = np.asarray(self._renderer.render(), dtype=np.float32)
        gray = rgb[..., 0] * 0.299 + rgb[..., 1] * 0.587 + rgb[..., 2] * 0.114
        return np.clip(np.rint(gray), 0, 255).astype(np.uint8)

    def close(self) -> None:
        self._renderer.close()


def _episode_arrays(config: DatasetConfig, episode_id: int) -> Tuple[Dict[str, np.ndarray], Dict[str, object]]:
    outcome = OUTCOMES[episode_id % len(OUTCOMES)]
    layout_id = episode_id % config.layout_variants
    # The default 60 episodes exercise every configured variant rather than only
    # declaring unused domain-randomization settings in the manifest.
    appearance_stride = max(1, math.ceil(config.episodes / config.appearance_variants))
    noise_stride = max(1, math.ceil(config.episodes / config.noise_variants))
    appearance_id = (episode_id // appearance_stride) % config.appearance_variants
    noise_id = (episode_id // noise_stride) % config.noise_variants
    layout = _layout(
        config.seed, layout_id, config.layout_variants, config.corridor_half_width_m
    )
    local = _rng(config.seed, 101, episode_id)

    finish_x = config.corridor_length_m - 0.25
    if outcome == "timeout":
        finish_x = max(layout.obstacle_x_m + 1.1, 0.72 * finish_x)
    # Put half of the temporal samples around the actual decision/clearance
    # region.  Uniform sampling over a long empty corridor made dangerous
    # states too rare even with many nominal frames.
    near_start = max(0.25, layout.obstacle_x_m - 1.0)
    near_finish = min(finish_x, layout.obstacle_x_m + 1.0)
    before_count = config.samples_per_episode // 4
    near_count = config.samples_per_episode // 2
    after_count = config.samples_per_episode - before_count - near_count
    x = np.concatenate((
        np.linspace(0.25, near_start, before_count, endpoint=False),
        np.linspace(near_start, near_finish, near_count, endpoint=False),
        np.linspace(near_finish, finish_x, after_count, endpoint=True),
    )).astype(np.float64)
    if outcome == "contact":
        # Deliberately steer toward the wall-attached obstacle.  This creates a
        # true contact label even for shallow obstacle intrusions.
        amplitude = layout.obstacle_side * abs(layout.obstacle_center_y_m)
    else:
        target = 0.035 if outcome == "near_miss" else 0.22
        amplitude = _amplitude_for_clearance(x, layout, config.drone_radius_m, target)
    y = _bump(x, layout.obstacle_x_m, amplitude)

    # Low-frequency, deterministic pose errors cover estimation/camera variation
    # without invalidating analytic clearance guarantees.
    phase = float(local.uniform(0.0, 2.0 * math.pi))
    lateral_noise = 0.0025 * (1 + noise_id) * np.sin(0.73 * x + phase)
    if outcome == "near_miss":
        lateral_noise *= 0.2
    y += lateral_noise
    clearance = _box_clearance(x, y, layout, config.drone_radius_m)

    if outcome == "contact":
        hits = np.flatnonzero(clearance <= 0.0)
        if len(hits) == 0:
            raise RuntimeError("contact episode did not intersect the obstacle")
        stop = min(int(hits[0]) + 1, len(x) - 1)
        x, y, clearance = x[: stop + 1], y[: stop + 1], clearance[: stop + 1]
    elif float(np.min(clearance)) <= 0.0:
        raise RuntimeError(f"non-contact episode {episode_id} has non-positive clearance")

    count = len(x)
    dt = 0.04
    vx = np.gradient(x, dt)
    vy = np.gradient(y, dt)
    yaw = np.arctan2(vy, vx)
    yaw_error = 0.018 * np.sin(0.91 * x + phase)
    wall_clearance = layout.corridor_half_width_m - np.abs(y) - config.drone_radius_m
    progress = x.copy()
    latent = np.stack((progress, y, vx, vy, yaw_error, clearance, wall_clearance), axis=1).astype(np.float32)
    next_latent = np.concatenate((latent[1:], latent[-1:]), axis=0)

    forward = layout.obstacle_x_m - x
    recovery_mask = ((x > layout.obstacle_x_m) & (np.abs(y) >= 0.10)).astype(np.uint8)
    decision_mask = (
        (forward >= -0.05) & (forward <= 2.25) & (recovery_mask == 0)
    ).astype(np.uint8)
    # Recovery/rejoin is intentionally supervised as hard TRACK: the obstacle
    # remains visually close, but continuing to steer would cause over-avoidance.
    # Hard TRACK means the obstacle is still visually relevant but the correct
    # command is to wait or rejoin.  Preserve separate easy, distant TRACK
    # examples so validation can measure false/unnecessary dodges.
    near_but_not_decision = (((forward > 2.25) & (forward <= 3.50)) |
                             ((forward < -0.05) & (forward >= -1.00)))
    hard_track_mask = (near_but_not_decision | (recovery_mask != 0)).astype(np.uint8)
    expert = np.full(count, ACTION_TRACK, dtype=np.int64)
    expert[decision_mask.astype(bool)] = ACTION_LEFT if layout.pass_sign > 0 else ACTION_RIGHT
    expert[recovery_mask.astype(bool)] = ACTION_TRACK

    behavior = expert.copy()
    exploration = local.random(count) < 0.08
    behavior[exploration] = local.integers(0, ACTION_COUNT, size=int(exploration.sum()))

    scores = np.empty((count, ACTION_COUNT), dtype=np.float32)
    # Candidate scores approximate path alignment, wall margin, and obstacle side.
    scores[:, ACTION_TRACK] = -1.5 * np.abs(y) + 0.4 * np.clip(clearance, -0.5, 1.0)
    scores[:, ACTION_LEFT] = scores[:, ACTION_TRACK] - 0.18 - 0.75 * np.maximum(y, 0.0)
    scores[:, ACTION_RIGHT] = scores[:, ACTION_TRACK] - 0.18 - 0.75 * np.maximum(-y, 0.0)
    rows = np.arange(count)
    scores[rows, expert] = np.max(scores, axis=1) + 1.0

    reward = (0.08 * vx - 0.35 * np.abs(y) + 0.03 * np.clip(clearance, -1.0, 1.0)).astype(np.float32)
    done = np.zeros(count, dtype=np.uint8)
    terminal = np.zeros(count, dtype=np.uint8)
    success = np.zeros(count, dtype=np.uint8)
    contact = np.zeros(count, dtype=np.uint8)
    timeout = np.zeros(count, dtype=np.uint8)
    done[-1] = terminal[-1] = 1
    if outcome == "contact":
        contact[-1] = 1
        reward[-1] = -100.0
    elif outcome == "timeout":
        timeout[-1] = 1
        reward[-1] = -5.0
    else:
        success[-1] = 1
        reward[-1] = 20.0

    band = np.full(count, 2, dtype=np.uint8)  # safe
    band[clearance < 0.08] = 1  # near miss
    band[clearance <= 0.0] = 0  # contact

    xml = build_corridor_xml(config, layout, appearance_id)
    renderer = CorridorRenderer(xml)
    images = np.empty((count, IMAGE_SIZE, IMAGE_SIZE), dtype=np.uint8)
    pixel_rng = _rng(config.seed, 303, noise_id, episode_id)
    try:
        for index in range(count):
            roll = float(np.clip(-0.24 * vy[index] + 0.025 * math.sin(0.4 * x[index] + phase), -0.30, 0.30))
            pitch = float(0.015 * math.sin(0.63 * x[index] + 0.3 * phase))
            z = config.camera_height_m + 0.012 * math.sin(0.52 * x[index] + phase)
            image = renderer.render((x[index], y[index], z), (roll, pitch, yaw[index] + yaw_error[index]))
            # Suppress isolated edge-pixel rasterization jitter before applying
            # the deterministic sensor-noise model.
            padded = np.pad(image, 1, mode="edge")
            neighbors = np.stack(
                [
                    padded[row : row + IMAGE_SIZE, col : col + IMAGE_SIZE]
                    for row in range(3)
                    for col in range(3)
                ],
                axis=0,
            )
            image = np.median(neighbors, axis=0).astype(np.uint8)
            contrast = 0.94 + 0.025 * (appearance_id % 5)
            sigma = 0.8 + 0.8 * noise_id
            noisy = (image.astype(np.float32) - 127.5) * contrast + 127.5
            noisy += pixel_rng.normal(0.0, sigma, image.shape)
            image_u8 = np.clip(np.rint(noisy), 0, 255).astype(np.uint8)
            # EGL rasterization can differ by one least-significant bit at an
            # isolated edge pixel.  HM01B0-like 5-bit quantization removes that
            # irrelevant driver variation and makes generated artifacts stable.
            images[index] = (image_u8 // 8) * 8
    finally:
        renderer.close()
    previous = np.concatenate((images[:1], images[:-1]), axis=0)
    frames = np.stack((previous, images), axis=1)

    arrays = {
        "frames": frames,
        "latent": latent,
        "next_latent": next_latent,
        "behavior_action": behavior,
        "expert_action": expert,
        "reward": reward,
        "done": done,
        "terminal": terminal,
        "success": success,
        "complete": success.copy(),
        "contact": contact,
        "timeout": timeout,
        "decision_mask": decision_mask,
        "hard_track_mask": hard_track_mask,
        "recovery_mask": recovery_mask,
        "expert_scores": scores,
        "episode_id": np.full(count, episode_id, dtype=np.int32),
        "scenario_id": np.full(count, episode_id, dtype=np.int32),
        "layout_id": np.full(count, layout.layout_id, dtype=np.int16),
        "appearance_id": np.full(count, appearance_id, dtype=np.int16),
        "noise_id": np.full(count, noise_id, dtype=np.int16),
        "clearance_band": band,
        "clearance": clearance.astype(np.float32),
    }
    metadata = {
        "episode_id": episode_id,
        "seed": int(np.random.SeedSequence([config.seed, 101, episode_id]).generate_state(1)[0]),
        "outcome": outcome,
        "transitions": count,
        "layout_id": layout.layout_id,
        "appearance_id": appearance_id,
        "noise_id": noise_id,
        "obstacle_side": "lower" if layout.obstacle_side < 0 else "upper",
        "corridor_half_width_m": layout.corridor_half_width_m,
        "corridor_height_m": layout.corridor_height_m,
        "expert_avoidance": "LEFT" if layout.pass_sign > 0 else "RIGHT",
        "min_obstacle_clearance_m": float(np.min(clearance)),
        "min_wall_clearance_m": float(np.min(wall_clearance)),
        "terminal_reward": float(reward[-1]),
        "terminal_success": bool(success[-1]),
        "terminal_contact": bool(contact[-1]),
        "terminal_timeout": bool(timeout[-1]),
    }
    return arrays, metadata


def _concatenate(episodes: Iterable[Dict[str, np.ndarray]]) -> Dict[str, np.ndarray]:
    episodes = list(episodes)
    return {key: np.concatenate([episode[key] for episode in episodes], axis=0) for key in episodes[0]}


def _counts(values: np.ndarray, names: Sequence[str]) -> Dict[str, int]:
    return {name: int(np.sum(values == index)) for index, name in enumerate(names)}


def _split_manifest(
    split: str, filename: str, arrays: Mapping[str, np.ndarray], episodes: List[Dict[str, object]]
) -> Dict[str, object]:
    outcomes = {name: sum(e["outcome"] == name for e in episodes) for name in OUTCOMES}
    return {
        "schema_version": 1,
        "split": split,
        "file": filename,
        "transitions": int(len(arrays["reward"])),
        "episodes": len(episodes),
        "episode_ids": [int(e["episode_id"]) for e in episodes],
        "action_counts": _counts(arrays["expert_action"], ACTION_NAMES),
        "clearance_band_counts": _counts(arrays["clearance_band"], ("contact", "near_miss", "safe")),
        "outcome_episode_counts": outcomes,
        "terminal_counts": {
            "success": int(arrays["success"].sum()),
            "contact": int(arrays["contact"].sum()),
            "timeout": int(arrays["timeout"].sum()),
        },
        "episodes_detail": episodes,
    }


def generate_dataset(config: DatasetConfig) -> Dict[str, object]:
    config.validate()
    output = Path(config.output).expanduser().resolve()
    output.mkdir(parents=True, exist_ok=True)

    episode_arrays: Dict[int, Dict[str, np.ndarray]] = {}
    episode_metadata: Dict[int, Dict[str, object]] = {}
    for episode_id in range(config.episodes):
        arrays, metadata = _episode_arrays(config, episode_id)
        episode_arrays[episode_id] = arrays
        episode_metadata[episode_id] = metadata

    # Assign whole episodes.  Stratification by outcome avoids a validation split
    # that accidentally omits contacts or recovery behavior.
    validation_ids = set()
    for outcome_index in range(len(OUTCOMES)):
        ids = [i for i in range(config.episodes) if i % len(OUTCOMES) == outcome_index]
        count = max(1, int(round(len(ids) * config.validation_fraction)))
        validation_ids.update(ids[-count:])
    train_ids = [i for i in range(config.episodes) if i not in validation_ids]
    validation_ids_sorted = sorted(validation_ids)
    if not train_ids or not validation_ids_sorted:
        raise RuntimeError("episode split produced an empty partition")

    manifests: Dict[str, Dict[str, object]] = {}
    for split, ids in (("train", train_ids), ("validation", validation_ids_sorted)):
        arrays = _concatenate(episode_arrays[i] for i in ids)
        npz_path = output / f"{split}.npz"
        _save_npz_deterministic(npz_path, arrays)
        detail = [episode_metadata[i] for i in ids]
        split_manifest = _split_manifest(split, npz_path.name, arrays, detail)
        split_manifest["sha256"] = _sha256(npz_path)
        manifest_path = output / f"{split}_manifest.json"
        manifest_path.write_text(json.dumps(split_manifest, indent=2, sort_keys=True) + "\n")
        manifests[split] = split_manifest

    all_transitions = manifests["train"]["transitions"] + manifests["validation"]["transitions"]
    all_action_counts = {
        name: manifests["train"]["action_counts"][name] + manifests["validation"]["action_counts"][name]
        for name in ACTION_NAMES
    }
    outcome_counts = {
        name: manifests["train"]["outcome_episode_counts"][name]
        + manifests["validation"]["outcome_episode_counts"][name]
        for name in OUTCOMES
    }
    fractions = {name: count / config.episodes for name, count in outcome_counts.items()}
    action_fractions = {name: count / all_transitions for name, count in all_action_counts.items()}
    targets = {
        "transitions_at_least_30000": all_transitions >= 30_000,
        "episodes_at_least_60": config.episodes >= 60,
        "layouts_at_least_10": config.layout_variants >= 10,
        "appearances_at_least_5": config.appearance_variants >= 5,
        "noise_variants_at_least_3": config.noise_variants >= 3,
        "success_contact_near_miss_each_at_least_20pct": all(
            fractions[name] >= 0.20 for name in ("success", "contact", "near_miss")
        ),
        "every_action_at_least_10pct": all(value >= 0.10 for value in action_fractions.values()),
    }
    source = Path(__file__).resolve()
    manifest = {
        "schema_version": 1,
        "generator": source.name,
        "generator_sha256": _sha256(source),
        "config": asdict(config) | {"output": "."},
        "camera": {
            "width": IMAGE_SIZE,
            "height": IMAGE_SIZE,
            "channels": 1,
            "temporal_frames": 2,
            "fovy_degrees": CAMERA_FOV_Y_DEG,
            "mount_euler_degrees": [0, -90, -90],
        },
        "determinism": {
            "analytic_arrays": "byte-exact for a fixed seed and configuration",
            "rendered_frames": "semantic; EGL edge pixels may differ by one 5-bit level",
            "hashes": "integrity and provenance of this generation, not a cross-GPU golden digest",
        },
        "latent_names": list(LATENT_NAMES),
        "action_names": list(ACTION_NAMES),
        "transitions": all_transitions,
        "episodes": config.episodes,
        "action_counts": all_action_counts,
        "action_fractions": action_fractions,
        "outcome_episode_counts": outcome_counts,
        "outcome_episode_fractions": fractions,
        "acceptance_targets": targets,
        "meets_acceptance_targets": all(targets.values()),
        "files": {},
    }
    for filename in ("train.npz", "validation.npz", "train_manifest.json", "validation_manifest.json"):
        manifest["files"][filename] = {"sha256": _sha256(output / filename)}
    manifest_path = output / "manifest.json"
    manifest_path.write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n")
    return manifest


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True)
    parser.add_argument("--seed", type=int, default=DatasetConfig.seed)
    parser.add_argument("--episodes", type=int, default=DatasetConfig.episodes)
    parser.add_argument("--samples-per-episode", type=int, default=DatasetConfig.samples_per_episode)
    parser.add_argument("--validation-fraction", type=float, default=DatasetConfig.validation_fraction)
    parser.add_argument("--layout-variants", type=int, default=DatasetConfig.layout_variants)
    parser.add_argument("--appearance-variants", type=int, default=DatasetConfig.appearance_variants)
    parser.add_argument("--noise-variants", type=int, default=DatasetConfig.noise_variants)
    return parser


def main() -> None:
    args = _parser().parse_args()
    manifest = generate_dataset(DatasetConfig(**vars(args)))
    print(json.dumps({
        "output": str(Path(args.output).resolve()),
        "transitions": manifest["transitions"],
        "episodes": manifest["episodes"],
        "meets_acceptance_targets": manifest["meets_acceptance_targets"],
    }, indent=2))


if __name__ == "__main__":
    main()
