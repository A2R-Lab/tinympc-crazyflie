#!/usr/bin/env python3
"""Generate deterministic privileged U-course expert labels and augmentations."""

from __future__ import annotations

import argparse
import hashlib
import io
import json
import zipfile
from pathlib import Path

import numpy as np

from . import ACTION_LEFT, ACTION_RIGHT, ACTION_TRACK
from .data import RunTransitions, _course_projection, concatenate_datasets, load_run

TRAINING_SEEDS = frozenset((11, 22, 33))
HELDOUT_SEEDS = frozenset((101, 202, 303))
ACTION_COUNT = 3


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def save_npz_deterministic(path: Path, arrays: dict[str, np.ndarray]) -> None:
    """Write a NumPy archive with sorted members and fixed ZIP metadata."""
    with zipfile.ZipFile(path, "w", compression=zipfile.ZIP_DEFLATED,
                         compresslevel=6) as archive:
        for name in sorted(arrays):
            buffer = io.BytesIO()
            np.lib.format.write_array(
                buffer, np.asarray(arrays[name]), allow_pickle=False)
            member = zipfile.ZipInfo(f"{name}.npy", date_time=(1980, 1, 1, 0, 0, 0))
            member.compress_type = zipfile.ZIP_DEFLATED
            member.external_attr = 0o600 << 16
            archive.writestr(member, buffer.getvalue(), compress_type=zipfile.ZIP_DEFLATED,
                             compresslevel=6)


def validate_source_seeds(seeds) -> None:
    observed = {int(seed) for seed in seeds}
    if observed & HELDOUT_SEEDS:
        raise ValueError(f"held-out evaluation seeds are forbidden: {sorted(observed & HELDOUT_SEEDS)}")
    if observed != TRAINING_SEEDS:
        raise ValueError(
            f"oracle generation requires exactly seeds {sorted(TRAINING_SEEDS)}, "
            f"received {sorted(observed)}")


def obstacle_path_geometry(course: dict):
    centerline = np.asarray(course["centerline"], dtype=np.float64)
    result = []
    for obstacle in course.get("obstacles", []):
        center = np.asarray(obstacle["center"], dtype=np.float64)[None, :]
        progress, signed_cross, tangent = _course_projection(center, centerline)
        half = np.asarray(obstacle["half_size"], dtype=np.float64)
        along_extent = abs(tangent[0, 0]) * half[0] + abs(tangent[0, 1]) * half[1]
        result.append({
            "name": obstacle["name"], "near_progress_m": float(progress[0] - along_extent),
            "signed_cross_track_m": float(signed_cross[0]),
        })
    return result


def oracle_labels(progress_m: np.ndarray, obstacle_geometry: list[dict],
                  lookahead_m: float = 2.25, recovery_m: float = 0.75):
    """Choose the nearest upcoming obstacle and pass opposite its intrusion."""
    progress = np.asarray(progress_m, dtype=np.float64)
    count = len(progress)
    action = np.full(count, ACTION_TRACK, dtype=np.int64)
    scores = np.zeros((count, ACTION_COUNT), dtype=np.float32)
    scores[:, ACTION_TRACK] = 1.0
    decision = np.zeros(count, dtype=bool)
    hard_track = np.zeros(count, dtype=bool)
    selected_obstacle = np.full(count, -1, dtype=np.int64)
    for sample, measured_progress in enumerate(progress):
        distances = np.asarray([
            item["near_progress_m"] - measured_progress for item in obstacle_geometry])
        upcoming = np.flatnonzero((distances >= 0.0) & (distances <= lookahead_m))
        if upcoming.size:
            obstacle_index = int(upcoming[np.argmin(distances[upcoming])])
            item = obstacle_geometry[obstacle_index]
            chosen = (ACTION_RIGHT if item["signed_cross_track_m"] > 0.0
                      else ACTION_LEFT)
            wrong = ACTION_LEFT if chosen == ACTION_RIGHT else ACTION_RIGHT
            proximity = 1.0 - float(distances[obstacle_index]) / lookahead_m
            action[sample] = chosen
            scores[sample] = 0.0
            scores[sample, chosen] = 1.0
            scores[sample, ACTION_TRACK] = -proximity
            scores[sample, wrong] = -1.0
            decision[sample] = True
            selected_obstacle[sample] = obstacle_index
        else:
            recently_passed = np.flatnonzero(
                (distances < 0.0) & (distances >= -recovery_m))
            hard_track[sample] = recently_passed.size > 0
    return action, scores, decision, hard_track, selected_obstacle


def _copy_dataset(source: RunTransitions, frames: np.ndarray, latent: np.ndarray,
                  next_latent: np.ndarray, behavior_action: np.ndarray,
                  expert_action: np.ndarray, expert_scores: np.ndarray,
                  episode_id: int) -> RunTransitions:
    return RunTransitions(
        frames, latent, next_latent, behavior_action, expert_action, expert_scores,
        source.decision_mask.copy(), source.hard_track_mask.copy(),
        source.reward.copy(), source.done.copy(),
        np.full(len(expert_action), episode_id, dtype=np.int64))


def swap_left_right(actions: np.ndarray) -> np.ndarray:
    return np.where(actions == ACTION_LEFT, ACTION_RIGHT,
                    np.where(actions == ACTION_RIGHT, ACTION_LEFT, actions)).astype(np.int64)


def mirror_dataset(source: RunTransitions, episode_id: int) -> RunTransitions:
    latent = source.latent.copy()
    next_latent = source.next_latent.copy()
    for index in (1, 3, 4):
        latent[:, index] *= -1.0
        next_latent[:, index] *= -1.0
    scores = source.expert_scores[:, [0, 2, 1]].copy()
    return _copy_dataset(
        source, source.frames[:, :, :, ::-1].copy(), latent, next_latent,
        swap_left_right(source.behavior_action),
        swap_left_right(source.expert_action), scores, episode_id)


def photometric_dataset(source: RunTransitions, episode_id: int, seed: int,
                        kind: str) -> RunTransitions:
    if kind == "brightness":
        frames = np.clip(source.frames.astype(np.float32) * 0.92 + 6.0,
                         0.0, 255.0).astype(np.uint8)
    elif kind == "noise":
        generator = np.random.default_rng(seed)
        noise = generator.normal(0.0, 2.0, source.frames.shape)
        frames = np.clip(source.frames.astype(np.float32) + noise,
                         0.0, 255.0).astype(np.uint8)
    elif kind == "contrast":
        frames = np.clip(
            (source.frames.astype(np.float32) - 128.0) * 1.12 + 128.0,
            0.0, 255.0).astype(np.uint8)
    elif kind == "darkness":
        frames = np.clip(source.frames.astype(np.float32) * 0.78 + 2.0,
                         0.0, 255.0).astype(np.uint8)
    else:
        raise ValueError(kind)
    return _copy_dataset(
        source, frames, source.latent.copy(), source.next_latent.copy(),
        source.behavior_action.copy(), source.expert_action.copy(),
        source.expert_scores.copy(), episode_id)


def labeled_source(source: RunTransitions, course: dict, seed: int,
                   lookahead_m: float, recovery_m: float) -> RunTransitions:
    action, scores, decision, hard_track, _ = oracle_labels(
        source.latent[:, 0], obstacle_path_geometry(course), lookahead_m, recovery_m)
    result = _copy_dataset(
        source, source.frames.copy(), source.latent.copy(), source.next_latent.copy(),
        source.behavior_action.copy(), action, scores, seed)
    result.decision_mask = decision
    result.hard_track_mask = hard_track
    return result


def generate(run_dirs: list[Path], course_path: Path, output_path: Path,
             manifest_path: Path, lookahead_m: float = 2.25,
             recovery_m: float = 0.75) -> None:
    course = json.loads(course_path.read_text())
    sources = []
    source_records = []
    configured_sources = []
    for run_dir in run_dirs:
        config_path = run_dir / "run_config.json"
        config = json.loads(config_path.read_text())
        seed = int(config["random_seed"])
        configured_sources.append((run_dir, config_path, seed))
    # Reject held-out or incomplete source sets before reading any frame/state
    # artifact from them.
    validate_source_seeds([item[2] for item in configured_sources])
    for run_dir, config_path, seed in configured_sources:
        source = labeled_source(
            load_run(run_dir, course_path), course, seed, lookahead_m, recovery_m)
        episode_group = 100000 + seed
        source.episode_id[:] = episode_group
        # Every augmentation keeps one source-stable, namespaced episode group.
        # This prevents augmentation leakage without colliding with procedural
        # episode IDs from another expert shard.
        sources.extend((
            source, mirror_dataset(source, episode_group),
            photometric_dataset(source, episode_group, seed * 100 + 1, "brightness"),
            photometric_dataset(source, episode_group, seed * 100 + 2, "noise"),
        ))
        source_records.append({
            "seed": seed, "run_directory": str(run_dir.resolve()),
            "samples": len(source.expert_action),
            "files": {name: sha256(run_dir / name) for name in
                      ("run_config.json", "state.csv", "vision.csv", "fpv_camera.mp4")},
        })
    combined = concatenate_datasets(sources)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    save_npz_deterministic(output_path, {
        field: getattr(combined, field) for field in (
            "frames", "latent", "next_latent", "behavior_action", "expert_action",
            "expert_scores", "decision_mask", "hard_track_mask", "reward", "done",
            "episode_id")})
    manifest = {
        "format": "tinympc-u-course-oracle-expert-v1",
        "training_seeds": sorted(TRAINING_SEEDS),
        "excluded_heldout_seeds": sorted(HELDOUT_SEEDS),
        "lookahead_m": lookahead_m, "recovery_m": recovery_m,
        "pass_side_rule": "opposite signed obstacle cross-track",
        "augmentations": ["original", "horizontal_mirror", "brightness_0p92_plus_6",
                          "gaussian_noise_sigma_2"],
        "episode_group_rule": "all variants use namespaced group 100000 + source seed",
        "course": {"path": str(course_path.resolve()), "sha256": sha256(course_path)},
        "sources": sorted(source_records, key=lambda item: item["seed"]),
        "output": {"path": str(output_path.resolve()), "sha256": sha256(output_path),
                   "samples": len(combined.expert_action)},
    }
    manifest_path.write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--runs", type=Path, nargs="+", required=True)
    parser.add_argument("--course", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--lookahead-m", type=float, default=2.25)
    parser.add_argument("--recovery-m", type=float, default=0.75)
    args = parser.parse_args()
    if args.lookahead_m <= 0.0 or args.recovery_m < 0.0:
        parser.error("lookahead must be positive and recovery nonnegative")
    generate(args.runs, args.course, args.output, args.manifest,
             args.lookahead_m, args.recovery_m)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
