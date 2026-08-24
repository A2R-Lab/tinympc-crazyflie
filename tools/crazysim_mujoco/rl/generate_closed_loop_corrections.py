#!/usr/bin/env python3
"""Relabel failed closed-loop U-course runs as correction demonstrations."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

from .data import (RunTransitions, concatenate_datasets, load_run,
                   subset_dataset)
from .generate_oracle_expert import (
    labeled_source, mirror_dataset, photometric_dataset,
    save_npz_deterministic, sha256)

CORRECTION_SEEDS = frozenset((404, 505, 606))
RESERVED_EVALUATION_SEEDS = frozenset((707, 808, 909))


def validate_source_configs(configs: list[dict], source_policy_sha256: str) -> None:
    observed = {int(config["random_seed"]) for config in configs}
    if observed & RESERVED_EVALUATION_SEEDS:
        raise ValueError(
            "reserved evaluation seeds are forbidden: "
            f"{sorted(observed & RESERVED_EVALUATION_SEEDS)}")
    if observed != CORRECTION_SEEDS:
        raise ValueError(
            f"correction generation requires exactly seeds "
            f"{sorted(CORRECTION_SEEDS)}, received {sorted(observed)}")
    for config in configs:
        seed = int(config["random_seed"])
        if config.get("vision_adapter") != "hybrid_rl":
            raise ValueError(f"seed {seed} did not use the hybrid_rl adapter")
        if not config.get("vision_control_enabled", False):
            raise ValueError(f"seed {seed} was not closed-loop")
        if not config.get("stop_on_contact", False):
            raise ValueError(f"seed {seed} did not use stop-on-contact")
        observed_sha = config.get("vision_model", {}).get("sha256")
        if observed_sha != source_policy_sha256:
            raise ValueError(
                f"seed {seed} policy hash {observed_sha} does not match "
                f"{source_policy_sha256}")


def correction_focus_mask(source: RunTransitions, transition_radius: int = 15,
                          terminal_frames: int = 30) -> np.ndarray:
    """Select disagreements, action transitions, recovery, and crash approach."""
    count = len(source.expert_action)
    focus = source.behavior_action != source.expert_action
    focus = np.logical_or(focus, source.hard_track_mask)
    transitions = np.flatnonzero(source.expert_action[1:] != source.expert_action[:-1]) + 1
    for index in transitions:
        first = max(0, int(index) - transition_radius)
        last = min(count, int(index) + transition_radius + 1)
        focus[first:last] = True
    focus[max(0, count - terminal_frames):] = True
    return focus


def _counts(values: np.ndarray) -> list[int]:
    return np.bincount(values.astype(np.int64), minlength=3).astype(int).tolist()


def generate(run_dirs: list[Path], course_path: Path, source_policy: Path,
             output_path: Path, manifest_path: Path,
             lookahead_m: float = 2.25, recovery_m: float = 0.75) -> None:
    policy_sha = sha256(source_policy)
    configured = []
    for run_dir in run_dirs:
        config_path = run_dir / "run_config.json"
        summary_path = run_dir / "summary.json"
        config = json.loads(config_path.read_text())
        summary = json.loads(summary_path.read_text())
        configured.append((run_dir, config_path, summary_path, config, summary))
    # Reject seed leakage and non-closed-loop sources before decoding frames.
    validate_source_configs([item[3] for item in configured], policy_sha)

    course = json.loads(course_path.read_text())
    shards = []
    source_records = []
    for run_dir, config_path, summary_path, config, summary in configured:
        seed = int(config["random_seed"])
        if summary.get("course_success", False) or not summary.get("crashed", False):
            raise ValueError(f"seed {seed} is not a retained failed/contact run")
        source = labeled_source(
            load_run(run_dir, course_path), course, seed, lookahead_m, recovery_m)
        episode_group = 200000 + seed
        source.episode_id[:] = episode_group
        focus = correction_focus_mask(source)
        focused = subset_dataset(source, focus)
        shards.extend((
            source,
            mirror_dataset(source, episode_group),
            photometric_dataset(source, episode_group, seed * 100 + 1, "brightness"),
            photometric_dataset(source, episode_group, seed * 100 + 2, "noise"),
            photometric_dataset(focused, episode_group, seed * 100 + 3, "contrast"),
            photometric_dataset(focused, episode_group, seed * 100 + 4, "darkness"),
        ))
        source_records.append({
            "seed": seed,
            "run_directory": str(run_dir.resolve()),
            "samples": len(source.expert_action),
            "focus_samples": int(np.count_nonzero(focus)),
            "behavior_action_counts_track_left_right": _counts(source.behavior_action),
            "expert_action_counts_track_left_right": _counts(source.expert_action),
            "behavior_expert_disagreements": int(np.count_nonzero(
                source.behavior_action != source.expert_action)),
            "course_success": bool(summary["course_success"]),
            "first_crash_after_launch_s": summary["first_crash_after_launch_s"],
            "files": {
                "run_config.json": sha256(config_path),
                "summary.json": sha256(summary_path),
                "state.csv": sha256(run_dir / "state.csv"),
                "vision.csv": sha256(run_dir / "vision.csv"),
                "fpv_camera.mp4": sha256(run_dir / "fpv_camera.mp4"),
            },
        })

    combined = concatenate_datasets(shards)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    save_npz_deterministic(output_path, {
        field: getattr(combined, field) for field in (
            "frames", "latent", "next_latent", "behavior_action",
            "expert_action", "expert_scores", "decision_mask",
            "hard_track_mask", "reward", "done", "episode_id")
    })
    manifest = {
        "format": "tinympc-closed-loop-corrections-v1",
        "correction_seeds": sorted(CORRECTION_SEEDS),
        "reserved_evaluation_seeds": sorted(RESERVED_EVALUATION_SEEDS),
        "source_policy": {
            "path": str(source_policy.resolve()), "sha256": policy_sha},
        "lookahead_m": lookahead_m,
        "recovery_m": recovery_m,
        "pass_side_rule": "opposite signed obstacle cross-track",
        "focus_rule": (
            "behavior/expert disagreement OR hard TRACK OR +/-15 frames around "
            "expert transitions OR final 30 frames"),
        "augmentations": [
            "original", "horizontal_mirror", "brightness_0p92_plus_6",
            "gaussian_noise_sigma_2", "focused_contrast_1p12",
            "focused_darkness_0p78_plus_2"],
        "episode_group_rule": (
            "all variants use namespaced group 200000 + source seed"),
        "course": {"path": str(course_path.resolve()),
                   "sha256": sha256(course_path)},
        "sources": sorted(source_records, key=lambda item: item["seed"]),
        "output": {
            "path": str(output_path.resolve()), "sha256": sha256(output_path),
            "samples": len(combined.expert_action),
            "expert_action_counts_track_left_right": _counts(
                combined.expert_action),
        },
    }
    manifest_path.write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--runs", type=Path, nargs="+", required=True)
    parser.add_argument("--course", type=Path, required=True)
    parser.add_argument("--source-policy", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--lookahead-m", type=float, default=2.25)
    parser.add_argument("--recovery-m", type=float, default=0.75)
    args = parser.parse_args()
    if args.lookahead_m <= 0.0 or args.recovery_m < 0.0:
        parser.error("lookahead must be positive and recovery nonnegative")
    generate(args.runs, args.course, args.source_policy, args.output,
             args.manifest, args.lookahead_m, args.recovery_m)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
