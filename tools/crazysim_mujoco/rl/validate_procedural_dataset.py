#!/usr/bin/env python3
"""Validate procedural vision-RL NPZ datasets and split isolation.

The validator is intentionally independent of the generator and trainer.  A
manifest may describe one shard directly or contain a ``shards``/``files``
list.  Relative NPZ paths are resolved relative to the manifest that names
them.
"""

from __future__ import annotations

import argparse
import hashlib
import json
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any, Iterable

import numpy as np

from .reward import transition_reward


REQUIRED_ARRAYS = {
    "frames", "latent", "next_latent", "behavior_action", "expert_action",
    "expert_scores", "decision_mask", "hard_track_mask", "done", "contact",
    "complete", "timeout", "episode_id", "clearance",
}
VALID_SPLITS = {"train", "validation", "test"}


class DatasetValidationError(ValueError):
    """Raised when an artifact cannot be interpreted safely."""


@dataclass(frozen=True)
class Thresholds:
    min_transitions: int = 30_000
    min_episodes: int = 60
    min_scenarios: int = 10
    min_layouts: int = 10
    min_appearances: int = 5
    min_noise_variants: int = 3
    min_action_count: int = 500
    min_action_fraction: float = 0.10
    min_decision_direction_count: int = 200
    min_outcome_fraction: float = 0.20
    min_clearance_lt_0p4_fraction: float = 0.20
    min_clearance_lt_0p2_fraction: float = 0.05
    min_clearance_gt_0p6_fraction: float = 0.10
    min_progress_decile_count: int = 100
    require_all_splits: bool = True
    require_hashes: bool = True
    require_scenario_id: bool = True
    require_provenance: bool = True
    require_no_cross_split_frame_duplicates: bool = True

    @classmethod
    def smoke(cls) -> "Thresholds":
        return cls(
            min_transitions=1, min_episodes=1, min_scenarios=1,
            min_layouts=0, min_appearances=0, min_noise_variants=0,
            min_action_count=0, min_action_fraction=0.0,
            min_decision_direction_count=0, min_outcome_fraction=0.0,
            min_clearance_lt_0p4_fraction=0.0,
            min_clearance_lt_0p2_fraction=0.0,
            min_clearance_gt_0p6_fraction=0.0,
            min_progress_decile_count=0, require_all_splits=False,
            require_hashes=False, require_scenario_id=False,
            require_provenance=False,
            require_no_cross_split_frame_duplicates=False,
        )


@dataclass
class Shard:
    manifest: Path
    path: Path
    split: str
    metadata: dict[str, Any]
    arrays: dict[str, np.ndarray]
    sha256: str


def file_sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _jsonable(value: Any) -> Any:
    if value is None or isinstance(value, (str, int, float, bool, list, dict, tuple)):
        return value
    if isinstance(value, np.bool_):
        return bool(value)
    if isinstance(value, (np.integer, np.floating)):
        return value.item()
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, Path):
        return str(value)
    if isinstance(value, set):
        return sorted(value)
    raise TypeError(f"not JSON serializable: {type(value).__name__}")


def _entries(document: dict[str, Any]) -> list[dict[str, Any]]:
    for key in ("shards", "files", "episodes"):
        if key in document:
            if isinstance(document[key], list):
                return document[key]
            # The procedural generator's aggregate manifest uses a filename to
            # hash mapping.  Only NPZ entries are dataset shards; JSON entries
            # are retained provenance documents.
            if key == "files" and isinstance(document[key], dict):
                result = []
                for filename, metadata in document[key].items():
                    if not str(filename).endswith(".npz"):
                        continue
                    detail = metadata if isinstance(metadata, dict) else {}
                    result.append({"file": filename,
                                   "split": Path(filename).stem,
                                   **detail})
                return result
            raise DatasetValidationError(f"manifest {key} must be a list")
    if any(key in document for key in ("path", "npz", "file")):
        return [document]
    raise DatasetValidationError("manifest has no shards/files or NPZ path")


def load_shards(manifest_paths: Iterable[Path]) -> tuple[list[Shard], list[dict]]:
    shards: list[Shard] = []
    documents: list[dict] = []
    for manifest_path in manifest_paths:
        manifest_path = manifest_path.resolve()
        document = json.loads(manifest_path.read_text())
        if not isinstance(document, dict):
            raise DatasetValidationError(f"{manifest_path}: root must be an object")
        documents.append(document)
        if isinstance(document.get("files"), dict):
            for filename, detail in document["files"].items():
                if not isinstance(detail, dict) or "sha256" not in detail:
                    continue
                retained = (manifest_path.parent / filename).resolve()
                if not retained.is_file():
                    raise DatasetValidationError(f"missing retained artifact: {retained}")
                actual = file_sha256(retained)
                if actual != str(detail["sha256"]).lower():
                    raise DatasetValidationError(
                        f"{retained}: SHA-256 mismatch, expected "
                        f"{detail['sha256']}, got {actual}")
        defaults = {key: value for key, value in document.items()
                    if key not in ("shards", "files", "episodes")}
        for raw_entry in _entries(document):
            if not isinstance(raw_entry, dict):
                raise DatasetValidationError(f"{manifest_path}: shard must be an object")
            entry = {**defaults, **raw_entry}
            # Aggregate manifests retain richer per-split outcome metadata in
            # adjacent sidecars.  Merge it only after its hash was checked.
            split_hint = str(entry.get("split", ""))
            sidecar = manifest_path.parent / f"{split_hint}_manifest.json"
            if isinstance(document.get("files"), dict) and sidecar.is_file():
                sidecar_document = json.loads(sidecar.read_text())
                if isinstance(sidecar_document, dict):
                    entry = {**entry, **sidecar_document}
            path_value = entry.get("path", entry.get("npz", entry.get("file")))
            if path_value is None:
                raise DatasetValidationError(f"{manifest_path}: shard has no path")
            path = Path(path_value)
            if not path.is_absolute():
                path = manifest_path.parent / path
            path = path.resolve()
            if not path.is_file():
                raise DatasetValidationError(f"missing NPZ shard: {path}")
            split = str(entry.get("split", document.get("split", ""))).lower()
            if split == "val":
                split = "validation"
            if split not in VALID_SPLITS:
                raise DatasetValidationError(f"{path}: invalid split {split!r}")
            digest = file_sha256(path)
            expected = entry.get("sha256", entry.get("file_sha256"))
            if expected is not None and str(expected).lower() != digest:
                raise DatasetValidationError(
                    f"{path}: SHA-256 mismatch, expected {expected}, got {digest}")
            with np.load(path, allow_pickle=False) as archive:
                arrays = {name: archive[name] for name in archive.files}
            # Accept the generator's descriptive names while exposing one
            # canonical validator schema.
            if "expert_scores" not in arrays and "counterfactual_action_scores" in arrays:
                arrays["expert_scores"] = arrays["counterfactual_action_scores"]
            if "complete" not in arrays and "success" in arrays:
                arrays["complete"] = arrays["success"]
            if "clearance" not in arrays and "latent" in arrays and arrays["latent"].ndim == 2 \
                    and arrays["latent"].shape[1] >= 6:
                arrays["clearance"] = arrays["latent"][:, 5]
            if "scenario_id" not in arrays and all(
                    name in arrays for name in ("layout_id", "appearance_id", "noise_id")):
                arrays["scenario_id"] = np.asarray([
                    f"layout={layout}|appearance={appearance}|noise={noise}"
                    for layout, appearance, noise in zip(
                        arrays["layout_id"], arrays["appearance_id"], arrays["noise_id"])
                ])
            shards.append(Shard(manifest_path, path, split, entry, arrays, digest))
    if not shards:
        raise DatasetValidationError("no NPZ shards loaded")
    return shards, documents


def _binary(name: str, value: np.ndarray) -> np.ndarray:
    if value.ndim != 1:
        raise DatasetValidationError(f"{name} must be one-dimensional")
    if not np.all(np.isin(value, (0, 1, False, True))):
        raise DatasetValidationError(f"{name} must contain only zero/one values")
    return value.astype(bool)


def _actions(name: str, value: np.ndarray) -> np.ndarray:
    if value.ndim != 1 or not np.issubdtype(value.dtype, np.number):
        raise DatasetValidationError(f"{name} must be a numeric vector")
    rounded = np.rint(value)
    if (not np.all(np.isfinite(value)) or
            not np.allclose(value, rounded, rtol=0.0, atol=1.0e-6) or
            not np.all((rounded >= 0) & (rounded <= 2))):
        raise DatasetValidationError(f"{name} must contain actions 0, 1, or 2")
    return rounded.astype(np.int64)


def _identity_values(value: np.ndarray, name: str) -> np.ndarray:
    if value.ndim != 1:
        raise DatasetValidationError(f"{name} must be one-dimensional")
    if value.dtype.kind in "fc" and not np.all(np.isfinite(value)):
        raise DatasetValidationError(f"{name} contains non-finite identifiers")
    return np.asarray([str(item) for item in value])


def _check(checks: list[dict], name: str, passed: bool, actual: Any,
           required: Any, detail: str | None = None) -> None:
    item = {"name": name, "passed": bool(passed), "actual": _jsonable(actual),
            "required": _jsonable(required)}
    if detail:
        item["detail"] = detail
    checks.append(item)


def _variant_values(shard: Shard, keys: tuple[str, ...]) -> set[str]:
    result: set[str] = set()
    for key in keys:
        if key in shard.arrays:
            result.update(_identity_values(shard.arrays[key], key))
        elif key in shard.metadata:
            value = shard.metadata[key]
            if isinstance(value, list):
                result.update(str(item) for item in value)
            else:
                result.add(str(value))
    return result


def validate_dataset(manifest_paths: Iterable[Path],
                     thresholds: Thresholds | None = None) -> dict[str, Any]:
    """Validate shards and return a JSON-serializable acceptance report."""
    thresholds = thresholds or Thresholds()
    shards, documents = load_shards(manifest_paths)
    checks: list[dict] = []
    records: list[dict[str, Any]] = []
    split_seeds: dict[str, set[str]] = {split: set() for split in VALID_SPLITS}
    split_scenarios: dict[str, set[str]] = {split: set() for split in VALID_SPLITS}
    split_episodes: dict[str, set[str]] = {split: set() for split in VALID_SPLITS}
    split_frame_hashes: dict[str, set[str]] = {split: set() for split in VALID_SPLITS}
    seen_file_hashes: dict[str, str] = {}
    layouts: set[str] = set()
    appearances: set[str] = set()
    noises: set[str] = set()

    for shard in shards:
        missing = sorted(REQUIRED_ARRAYS.difference(shard.arrays))
        if thresholds.require_scenario_id and "scenario_id" not in shard.arrays:
            missing.append("scenario_id")
        if missing:
            raise DatasetValidationError(f"{shard.path}: missing arrays {missing}")
        frames = shard.arrays["frames"]
        if frames.ndim != 4 or frames.shape[1:] != (2, 160, 160):
            raise DatasetValidationError(
                f"{shard.path}: frames must have shape [N,2,160,160]")
        if frames.dtype != np.uint8:
            raise DatasetValidationError(f"{shard.path}: frames must be uint8")
        count = len(frames)
        if count == 0:
            raise DatasetValidationError(f"{shard.path}: empty shard")
        for name in REQUIRED_ARRAYS.difference({"frames"}):
            if len(shard.arrays[name]) != count:
                raise DatasetValidationError(
                    f"{shard.path}: {name} length differs from frames")
        if "scenario_id" in shard.arrays and len(shard.arrays["scenario_id"]) != count:
            raise DatasetValidationError(f"{shard.path}: scenario_id length mismatch")
        for name in ("latent", "next_latent"):
            value = shard.arrays[name]
            if value.shape != (count, 7) or not np.issubdtype(value.dtype, np.number):
                raise DatasetValidationError(f"{shard.path}: {name} must be [N,7]")
            if not np.all(np.isfinite(value)):
                raise DatasetValidationError(f"{shard.path}: {name} is non-finite")
        scores = shard.arrays["expert_scores"]
        if scores.shape != (count, 3) or not np.all(np.isfinite(scores)):
            raise DatasetValidationError(
                f"{shard.path}: expert_scores must be finite [N,3]")
        behavior = _actions("behavior_action", shard.arrays["behavior_action"])
        expert = _actions("expert_action", shard.arrays["expert_action"])
        decision = _binary("decision_mask", shard.arrays["decision_mask"])
        hard_track = _binary("hard_track_mask", shard.arrays["hard_track_mask"])
        done = _binary("done", shard.arrays["done"])
        contact = _binary("contact", shard.arrays["contact"])
        complete = _binary("complete", shard.arrays["complete"])
        timeout = _binary("timeout", shard.arrays["timeout"])
        clearance = np.asarray(shard.arrays["clearance"], dtype=float)
        if clearance.ndim != 1 or not np.all(np.isfinite(clearance)):
            raise DatasetValidationError(f"{shard.path}: clearance must be finite [N]")
        episode = _identity_values(shard.arrays["episode_id"], "episode_id")
        scenario = (_identity_values(shard.arrays["scenario_id"], "scenario_id")
                    if "scenario_id" in shard.arrays else episode.copy())
        terminal_sum = contact.astype(int) + complete.astype(int) + timeout.astype(int)
        _check(checks, f"{shard.path.name}:terminal_equivalence",
               np.array_equal(done, terminal_sum == 1),
               int(np.count_nonzero(done != (terminal_sum == 1))), 0)
        _check(checks, f"{shard.path.name}:terminal_exclusivity",
               bool(np.all(terminal_sum <= 1)), int(terminal_sum.max()), "<=1")
        _check(checks, f"{shard.path.name}:decision_track_disjoint",
               not bool(np.any(decision & hard_track)),
               int(np.count_nonzero(decision & hard_track)), 0)
        preference = str(shard.metadata.get("expert_score_preference", "max"))
        score_action = (np.argmin(scores, axis=1) if preference == "min"
                        else np.argmax(scores, axis=1))
        score_agreement = float(np.mean(score_action == expert))
        _check(checks, f"{shard.path.name}:expert_score_action_agreement",
               score_agreement >= 0.99, score_agreement, ">=0.99")

        terminal_episode_counts = []
        terminal_at_end = True
        for identifier in np.unique(episode):
            indices = np.flatnonzero(episode == identifier)
            terminals = indices[done[indices]]
            terminal_episode_counts.append(len(terminals))
            terminal_at_end &= len(terminals) == 1 and terminals[0] == indices[-1]
        _check(checks, f"{shard.path.name}:one_terminal_per_episode",
               all(value == 1 for value in terminal_episode_counts),
               terminal_episode_counts, "exactly one")
        _check(checks, f"{shard.path.name}:terminal_is_episode_end",
               terminal_at_end, terminal_at_end, True)

        previous = np.empty_like(behavior)
        for identifier in np.unique(episode):
            indices = np.flatnonzero(episode == identifier)
            previous[indices] = np.concatenate((behavior[indices[:1]], behavior[indices[:-1]]))
        expected_reward = transition_reward(
            shard.arrays["next_latent"][:, 0] - shard.arrays["latent"][:, 0],
            shard.arrays["next_latent"][:, 1], shard.arrays["next_latent"][:, 2],
            clearance, behavior, previous, contact, complete)
        contact_counterfactual = transition_reward(
            shard.arrays["next_latent"][:, 0] - shard.arrays["latent"][:, 0],
            shard.arrays["next_latent"][:, 1], shard.arrays["next_latent"][:, 2],
            clearance, behavior, previous, np.zeros(count), complete)
        complete_counterfactual = transition_reward(
            shard.arrays["next_latent"][:, 0] - shard.arrays["latent"][:, 0],
            shard.arrays["next_latent"][:, 1], shard.arrays["next_latent"][:, 2],
            clearance, behavior, previous, contact, np.zeros(count))
        contact_delta_ok = (not np.any(contact) or np.allclose(
            expected_reward[contact] - contact_counterfactual[contact], -100.0,
            rtol=0.0, atol=1.0e-4))
        complete_delta_ok = (not np.any(complete) or np.allclose(
            expected_reward[complete] - complete_counterfactual[complete], 20.0,
            rtol=0.0, atol=1.0e-4))
        _check(checks, f"{shard.path.name}:derived_contact_reward",
               contact_delta_ok, contact_delta_ok, "-100 delta")
        _check(checks, f"{shard.path.name}:derived_completion_reward",
               complete_delta_ok, complete_delta_ok, "+20 delta")
        if "reward" in shard.arrays:
            reward = np.asarray(shard.arrays["reward"], dtype=float)
            reward_shape_ok = reward.shape == (count,) and np.all(np.isfinite(reward))
            _check(checks, f"{shard.path.name}:stored_reward_finite",
                   reward_shape_ok, reward.shape, f"finite [{count}]")
            terminal_reward_ok = reward_shape_ok and \
                (not np.any(contact) or np.allclose(reward[contact], -100.0)) and \
                (not np.any(complete) or np.allclose(reward[complete], 20.0)) and \
                (not np.any(timeout) or np.allclose(reward[timeout], -5.0))
            _check(checks, f"{shard.path.name}:stored_terminal_rewards",
                   terminal_reward_ok,
                   {"contact": reward[contact].tolist() if reward_shape_ok else None,
                    "complete": reward[complete].tolist() if reward_shape_ok else None,
                    "timeout": reward[timeout].tolist() if reward_shape_ok else None},
                   {"contact": -100.0, "complete": 20.0, "timeout": -5.0})

        expected_hash = shard.metadata.get("sha256", shard.metadata.get("file_sha256"))
        _check(checks, f"{shard.path.name}:manifest_hash",
               expected_hash is not None or not thresholds.require_hashes,
               expected_hash, "present" if thresholds.require_hashes else "optional")
        duplicate_file_split = seen_file_hashes.get(shard.sha256)
        _check(checks, f"{shard.path.name}:unique_file_hash",
               duplicate_file_split in (None, shard.split), duplicate_file_split,
               "not reused across splits")
        seen_file_hashes[shard.sha256] = shard.split
        provenance = any(key in shard.metadata for key in
                         ("source_hash", "source_sha256", "source_commit", "provenance",
                          "generator_sha256"))
        _check(checks, f"{shard.path.name}:provenance",
               provenance or not thresholds.require_provenance, provenance,
               "present" if thresholds.require_provenance else "optional")
        seed = shard.metadata.get("seed", shard.metadata.get("random_seed"))
        if seed is not None:
            split_seeds[shard.split].add(str(seed))
        for episode_detail in shard.metadata.get("episodes_detail", []):
            if isinstance(episode_detail, dict) and "seed" in episode_detail:
                split_seeds[shard.split].add(str(episode_detail["seed"]))
        split_scenarios[shard.split].update(scenario)
        split_episodes[shard.split].update(episode)
        if thresholds.require_no_cross_split_frame_duplicates:
            split_frame_hashes[shard.split].update(
                hashlib.sha256(frame.tobytes()).hexdigest() for frame in frames)
        layouts.update(_variant_values(shard, ("layout_id", "layout_ids")))
        appearances.update(_variant_values(
            shard, ("appearance_id", "appearance_ids", "texture_id", "lighting_id")))
        noises.update(_variant_values(
            shard, ("noise_id", "noise_ids", "dynamics_id", "dynamics_variant")))
        records.append({
            "shard": str(shard.path), "split": shard.split, "count": count,
            "behavior": behavior, "expert": expert, "decision": decision,
            "hard_track": hard_track, "done": done, "contact": contact,
            "complete": complete, "timeout": timeout, "episode": episode,
            "scenario": scenario, "clearance": clearance,
            "outcome_by_episode": {
                str(detail["episode_id"]): str(detail["outcome"])
                for detail in shard.metadata.get("episodes_detail", [])
                if isinstance(detail, dict) and "episode_id" in detail
                and "outcome" in detail
            },
            "progress": np.asarray(shard.arrays["latent"][:, 0], dtype=float),
            "sha256": shard.sha256,
        })

    present_splits = {item["split"] for item in records}
    required_splits = {"train", "validation"}
    _check(checks, "required_splits", not thresholds.require_all_splits or
           required_splits.issubset(present_splits), sorted(present_splits),
           sorted(required_splits))
    for first, second in (("train", "validation"), ("train", "test"),
                          ("validation", "test")):
        for label, values in (("seed", split_seeds), ("scenario", split_scenarios),
                              ("episode", split_episodes)):
            overlap = sorted(values[first].intersection(values[second]))
            _check(checks, f"split_isolation:{first}:{second}:{label}",
                   not overlap, overlap, [])
        if thresholds.require_no_cross_split_frame_duplicates:
            overlap_count = len(split_frame_hashes[first].intersection(
                split_frame_hashes[second]))
            _check(checks, f"split_isolation:{first}:{second}:exact_frames",
                   overlap_count == 0, overlap_count, 0)

    count = sum(item["count"] for item in records)
    episodes = set().union(*(split_episodes.values()))
    scenarios = set().union(*(split_scenarios.values()))
    expert = np.concatenate([item["expert"] for item in records])
    decision = np.concatenate([item["decision"] for item in records])
    clearance = np.concatenate([item["clearance"] for item in records])
    progress = np.concatenate([item["progress"] for item in records])
    class_counts = np.bincount(expert, minlength=3)
    class_fractions = class_counts / max(1, count)
    decision_counts = np.bincount(expert[decision], minlength=3)
    _check(checks, "minimum_transitions", count >= thresholds.min_transitions,
           count, f">={thresholds.min_transitions}")
    _check(checks, "minimum_episodes", len(episodes) >= thresholds.min_episodes,
           len(episodes), f">={thresholds.min_episodes}")
    _check(checks, "minimum_scenarios", len(scenarios) >= thresholds.min_scenarios,
           len(scenarios), f">={thresholds.min_scenarios}")
    for name, values, minimum in (("layouts", layouts, thresholds.min_layouts),
                                  ("appearances", appearances, thresholds.min_appearances),
                                  ("noise_variants", noises, thresholds.min_noise_variants)):
        _check(checks, f"minimum_{name}", len(values) >= minimum,
               len(values), f">={minimum}")
    _check(checks, "expert_action_counts",
           bool(np.all(class_counts >= thresholds.min_action_count)), class_counts,
           f"each >={thresholds.min_action_count}")
    _check(checks, "expert_action_fractions",
           bool(np.all(class_fractions >= thresholds.min_action_fraction)),
           class_fractions, f"each >={thresholds.min_action_fraction}")
    _check(checks, "decision_direction_counts",
           bool(np.all(decision_counts[1:] >= thresholds.min_decision_direction_count)),
           decision_counts, f"LEFT/RIGHT each >={thresholds.min_decision_direction_count}")
    danger = clearance < 0.4
    danger_counts = np.bincount(expert[danger], minlength=3)
    _check(checks, "danger_direction_counts",
           bool(np.all(danger_counts[1:] >= thresholds.min_decision_direction_count)),
           danger_counts, f"LEFT/RIGHT each >={thresholds.min_decision_direction_count}")
    clearance_fractions = {
        "lt_0p4": float(np.mean(clearance < 0.4)),
        "lt_0p2": float(np.mean(clearance < 0.2)),
        "gt_0p6": float(np.mean(clearance > 0.6)),
    }
    for name, minimum in (("lt_0p4", thresholds.min_clearance_lt_0p4_fraction),
                          ("lt_0p2", thresholds.min_clearance_lt_0p2_fraction),
                          ("gt_0p6", thresholds.min_clearance_gt_0p6_fraction)):
        _check(checks, f"clearance_fraction_{name}",
               clearance_fractions[name] >= minimum, clearance_fractions[name],
               f">={minimum}")
    if np.ptp(progress) > 1.0e-9:
        bins = np.linspace(float(progress.min()), float(progress.max()), 11)
        progress_counts = np.histogram(progress, bins=bins)[0]
    else:
        progress_counts = np.zeros(10, dtype=int)
    _check(checks, "progress_decile_counts",
           bool(np.all(progress_counts >= thresholds.min_progress_decile_count)),
           progress_counts, f"each >={thresholds.min_progress_decile_count}")

    episode_outcomes = {"success": 0, "contact": 0, "near_miss": 0, "timeout": 0}
    for item in records:
        for identifier in np.unique(item["episode"]):
            indices = np.flatnonzero(item["episode"] == identifier)
            last = indices[-1]
            recorded_outcome = item["outcome_by_episode"].get(str(identifier))
            if recorded_outcome in episode_outcomes:
                episode_outcomes[recorded_outcome] += 1
            elif item["contact"][last]:
                episode_outcomes["contact"] += 1
            elif item["timeout"][last]:
                episode_outcomes["timeout"] += 1
            elif item["complete"][last]:
                if float(np.min(item["clearance"][indices])) < 0.08:
                    episode_outcomes["near_miss"] += 1
                else:
                    episode_outcomes["success"] += 1
    outcome_fractions = {name: value / max(1, len(episodes))
                         for name, value in episode_outcomes.items()}
    for name in ("success", "contact", "near_miss"):
        _check(checks, f"outcome_fraction_{name}",
               outcome_fractions[name] >= thresholds.min_outcome_fraction,
               outcome_fractions[name], f">={thresholds.min_outcome_fraction}")

    failed = [item["name"] for item in checks if not item["passed"]]
    return {
        "format": "tinympc-procedural-dataset-validation-v1",
        "accepted": not failed,
        "manifest_paths": [str(Path(path).resolve()) for path in manifest_paths],
        "thresholds": asdict(thresholds),
        "summary": {
            "shards": len(shards), "transitions": count,
            "episodes": len(episodes), "scenarios": len(scenarios),
            "splits": sorted(present_splits), "expert_action_counts": class_counts,
            "expert_action_fractions": class_fractions,
            "decision_action_counts": decision_counts,
            "danger_action_counts": danger_counts,
            "clearance_fractions": clearance_fractions,
            "progress_decile_counts": progress_counts,
            "episode_outcomes": episode_outcomes,
            "episode_outcome_fractions": outcome_fractions,
            "layout_count": len(layouts), "appearance_count": len(appearances),
            "noise_variant_count": len(noises),
        },
        "shards": [{"path": str(item.path), "split": item.split,
                    "sha256": item.sha256, "samples": len(item.arrays["frames"])}
                   for item in shards],
        "checks": checks, "failed_checks": failed,
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--manifest", type=Path, action="append", required=True)
    parser.add_argument("--profile", choices=("acceptance", "smoke"),
                        default="acceptance")
    parser.add_argument("--out", type=Path)
    args = parser.parse_args()
    thresholds = Thresholds() if args.profile == "acceptance" else Thresholds.smoke()
    try:
        report = validate_dataset(args.manifest, thresholds)
    except (DatasetValidationError, OSError, json.JSONDecodeError) as error:
        report = {"format": "tinympc-procedural-dataset-validation-v1",
                  "accepted": False, "fatal_error": str(error)}
    payload = json.dumps(report, indent=2, allow_nan=False, default=_jsonable) + "\n"
    if args.out:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        args.out.write_text(payload)
    print(payload, end="")
    return 0 if report.get("accepted") else 1


if __name__ == "__main__":
    raise SystemExit(main())
