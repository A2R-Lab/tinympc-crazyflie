#!/usr/bin/env python3
"""Off-policy U-course safe-side diagnostic for temporal ONNX policies.

This tool replays retained camera frames.  Candidate actions cannot affect the
recorded vehicle trajectory, so its results are classification diagnostics and
must never be presented as closed-loop course completion or collision evidence.

Privileged labels are derived in path coordinates.  The vehicle and each
obstacle center are projected onto the course centerline.  In the default
decision window, an obstacle center may be 0.05 m behind through 2.25 m ahead
in path progress.  The nearest eligible obstacle is selected, and the safe-side
label is the direction opposite its signed path-normal displacement.  Outside
that window the label is TRACK.  Thus LEFT/RIGHT rotate with the U course.
"""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path
from typing import Any, Iterable

import numpy as np

from . import ACTION_LEFT, ACTION_RIGHT, ACTION_TRACK
from .data import _course_projection, _numeric_csv, _video_frames
from .evaluate_procedural_policy import _run_onnx, classification_metrics
from .validate_procedural_dataset import DatasetValidationError, _jsonable, file_sha256


ACTION_NAMES = ("TRACK", "LEFT", "RIGHT")
DEFAULT_DECISION_BEHIND_M = 0.05
DEFAULT_DECISION_AHEAD_M = 2.25
FRAME_COUNT_TOLERANCE_FRACTION = 0.02
FRAME_COUNT_TOLERANCE_MINIMUM = 2


def _read_csv_times(path: Path) -> np.ndarray:
    with path.open(newline="") as stream:
        rows = list(csv.DictReader(stream))
    if not rows or "time_s" not in rows[0]:
        raise DatasetValidationError(f"{path}: missing timestamped rows")
    result = np.asarray([float(row["time_s"]) for row in rows], dtype=float)
    if not np.all(np.isfinite(result)) or np.any(np.diff(result) <= 0.0):
        raise DatasetValidationError(f"{path}: timestamps must be finite and increasing")
    return result


def obstacle_path_coordinates(course: dict[str, Any]) -> list[dict[str, Any]]:
    centerline = np.asarray(course["centerline"], dtype=float)
    obstacles = course.get("obstacles", [])
    if not obstacles:
        raise DatasetValidationError("course has no obstacles")
    centers = np.asarray([item["center"][:2] for item in obstacles], dtype=float)
    progress, signed_cross, _ = _course_projection(centers, centerline)
    result = []
    for index, (item, along, across) in enumerate(
            zip(obstacles, progress, signed_cross)):
        if abs(float(across)) < 1.0e-6:
            raise DatasetValidationError(
                f"obstacle {item.get('name', index)} lies on the centerline; safe side ambiguous")
        # Positive signed cross is path-left, so the safe side is path-right.
        action = ACTION_RIGHT if across > 0.0 else ACTION_LEFT
        result.append({
            "index": index, "name": item.get("name", f"obstacle_{index}"),
            "path_progress_m": float(along), "signed_cross_track_m": float(across),
            "safe_action": action, "safe_action_name": ACTION_NAMES[action],
        })
    return result


def privileged_safe_side_labels(
    points: np.ndarray, course: dict[str, Any], *,
    decision_behind_m: float = DEFAULT_DECISION_BEHIND_M,
    decision_ahead_m: float = DEFAULT_DECISION_AHEAD_M,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, list[dict[str, Any]]]:
    """Return labels, selected obstacle indices, and projected progress."""
    if decision_behind_m < 0.0 or decision_ahead_m <= 0.0:
        raise ValueError("decision window distances must be positive")
    centerline = np.asarray(course["centerline"], dtype=float)
    progress, _, _ = _course_projection(np.asarray(points, dtype=float), centerline)
    obstacles = obstacle_path_coordinates(course)
    obstacle_progress = np.asarray(
        [item["path_progress_m"] for item in obstacles], dtype=float)
    delta = obstacle_progress[None, :] - progress[:, None]
    eligible = (delta >= -decision_behind_m) & (delta <= decision_ahead_m)
    ordered = np.where(eligible, delta, np.inf)
    selected = np.argmin(ordered, axis=1)
    has_decision = np.any(eligible, axis=1)
    selected = np.where(has_decision, selected, -1)
    labels = np.full(len(points), ACTION_TRACK, dtype=np.int64)
    for index, item in enumerate(obstacles):
        labels[selected == index] = int(item["safe_action"])
    return labels, selected.astype(np.int64), progress, obstacles


def _active_window(run: Path, state: dict[str, np.ndarray], config: dict,
                   summary: dict) -> tuple[float, float]:
    launch = float(summary.get("launch_time_s", config.get("launch_time_s", 0.0)))
    end = summary.get("course_evaluation_end_time_s")
    if end is None:
        after = state["time_s"] >= launch
        contact = np.flatnonzero(after & (state.get("contacts", np.zeros_like(
            state["time_s"])) > 0.0))
        end = float(state["time_s"][contact[0]]) if contact.size else float(state["time_s"][-1])
    end = float(end)
    if end <= launch:
        raise DatasetValidationError(f"{run}: empty launch-to-terminal active window")
    return launch, end


def load_retained_run(
    run: Path, course: dict[str, Any], *, decision_behind_m: float,
    decision_ahead_m: float,
) -> dict[str, Any]:
    run = run.resolve()
    required = ("run_config.json", "summary.json", "state.csv", "vision.csv",
                "fpv_camera.mp4")
    missing = [name for name in required if not (run / name).is_file()]
    if missing:
        raise DatasetValidationError(f"{run}: missing {missing}")
    config = json.loads((run / "run_config.json").read_text())
    summary = json.loads((run / "summary.json").read_text())
    state = _numeric_csv(run / "state.csv")
    vision_time = _read_csv_times(run / "vision.csv")
    images = _video_frames(run / "fpv_camera.mp4")
    allowed = max(FRAME_COUNT_TOLERANCE_MINIMUM,
                  int(np.ceil(FRAME_COUNT_TOLERANCE_FRACTION * len(vision_time))))
    if abs(len(images) - len(vision_time)) > allowed:
        raise DatasetValidationError(
            f"{run}: video/vision frame mismatch {len(images)} vs {len(vision_time)}")
    count = min(len(images), len(vision_time))
    images, vision_time = images[:count], vision_time[:count]
    state_index = np.searchsorted(state["time_s"], vision_time, side="left")
    state_index = np.clip(state_index, 0, len(state["time_s"]) - 1)
    points = np.column_stack((state["x_m"][state_index], state["y_m"][state_index]))
    labels, selected, progress, obstacles = privileged_safe_side_labels(
        points, course, decision_behind_m=decision_behind_m,
        decision_ahead_m=decision_ahead_m)
    previous = np.concatenate((images[:1], images[:-1]), axis=0)
    frames = np.stack((previous, images), axis=1)
    launch, end = _active_window(run, state, config, summary)
    active = (vision_time >= launch) & (vision_time <= end)
    if not np.any(active):
        raise DatasetValidationError(f"{run}: no camera frames in active window")
    seed = config.get("random_seed")
    if seed is None:
        raise DatasetValidationError(f"{run}: run_config lacks random_seed")
    return {
        "path": run, "seed": int(seed), "frames": frames[active],
        "labels": labels[active], "selected_obstacle": selected[active],
        "progress": progress[active], "times": vision_time[active],
        "launch_time_s": launch, "end_time_s": end, "obstacles": obstacles,
        "artifact_hashes": {
            name: file_sha256(run / name) for name in
            ("run_config.json", "state.csv", "vision.csv", "fpv_camera.mp4")
        },
    }


def _extract_training_seeds(value: Any, parent_key: str = "") -> set[int]:
    seeds: set[int] = set()
    if isinstance(value, dict):
        for key, item in value.items():
            if key in ("training_seeds", "episode_seeds", "source_seeds") and \
                    isinstance(item, list):
                seeds.update(int(seed) for seed in item)
            elif key in ("seed", "random_seed") and parent_key not in (
                    "optimizer", "optimization"):
                try:
                    seeds.add(int(item))
                except (TypeError, ValueError):
                    pass
            elif key != "optimizer_seed":
                seeds.update(_extract_training_seeds(item, key))
    elif isinstance(value, list):
        for item in value:
            seeds.update(_extract_training_seeds(item, parent_key))
    return seeds


def training_seed_evidence(paths: Iterable[Path]) -> tuple[set[int], list[dict]]:
    seeds: set[int] = set()
    evidence = []
    for path in paths:
        path = path.resolve()
        document = json.loads(path.read_text())
        current = _extract_training_seeds(document)
        seeds.update(current)
        evidence.append({"path": str(path), "sha256": file_sha256(path),
                         "extracted_training_seeds": sorted(current)})
    return seeds, evidence


def _policy_specs(values: Iterable[str]) -> dict[str, Path]:
    result: dict[str, Path] = {}
    for value in values:
        if "=" not in value:
            raise DatasetValidationError("policy must be NAME=PATH")
        name, raw_path = value.split("=", 1)
        if not name or name in result:
            raise DatasetValidationError(f"invalid or duplicate policy name {name!r}")
        path = Path(raw_path).expanduser().resolve()
        if not path.is_file():
            raise DatasetValidationError(f"missing policy: {path}")
        result[name] = path
    return result


def _bundle_specs(values: Iterable[str]) -> dict[str, Path]:
    return _policy_specs(values)


def _bundle_policy_hash(document: dict) -> str | None:
    policy = document.get("artifacts", {}).get("policy_onnx", {})
    return policy.get("sha256") if isinstance(policy, dict) else None


def evaluate_transfer(
    run_paths: Iterable[Path], policies: dict[str, Path], course_path: Path, *,
    training_manifests: Iterable[Path] = (), bundles: dict[str, Path] | None = None,
    decision_behind_m: float = DEFAULT_DECISION_BEHIND_M,
    decision_ahead_m: float = DEFAULT_DECISION_AHEAD_M,
) -> dict[str, Any]:
    """Replay candidate policies on retained frames and aggregate diagnostics."""
    try:
        import onnxruntime as ort
    except ImportError as error:
        raise DatasetValidationError("onnxruntime is required") from error
    course_path = course_path.resolve()
    course = json.loads(course_path.read_text())
    runs = [load_retained_run(
        Path(path), course, decision_behind_m=decision_behind_m,
        decision_ahead_m=decision_ahead_m) for path in run_paths]
    if not runs:
        raise DatasetValidationError("at least one run is required")
    eval_seeds = {item["seed"] for item in runs}
    training_seeds, seed_evidence = training_seed_evidence(training_manifests)
    for name, path in (bundles or {}).items():
        if name not in policies:
            raise DatasetValidationError(f"bundle {name!r} has no matching policy")
        document = json.loads(path.read_text())
        expected = _bundle_policy_hash(document)
        actual = file_sha256(policies[name])
        if expected is not None and expected != actual:
            raise DatasetValidationError(
                f"bundle/model hash mismatch for {name}: {expected} != {actual}")
        bundle_seeds = _extract_training_seeds(document)
        training_seeds.update(bundle_seeds)
        seed_evidence.append({"path": str(path.resolve()), "sha256": file_sha256(path),
                              "policy": name,
                              "extracted_training_seeds": sorted(bundle_seeds)})
    overlap = sorted(eval_seeds.intersection(training_seeds))
    if overlap:
        raise DatasetValidationError(
            f"training/evaluation seed overlap: {overlap}")

    sessions = {}
    for name, path in policies.items():
        session = ort.InferenceSession(str(path), providers=["CPUExecutionProvider"])
        inputs, outputs = session.get_inputs(), session.get_outputs()
        if len(inputs) != 1 or len(outputs) != 1 or len(inputs[0].shape) != 4 or \
                inputs[0].shape[1:] != [2, 160, 160] or outputs[0].shape[-1] != 3:
            raise DatasetValidationError(
                f"{name}: expected temporal categorical ONNX [N,2,160,160] -> [N,3]")
        sessions[name] = (session, inputs[0].name)

    policy_reports = {}
    for name, path in policies.items():
        session, input_name = sessions[name]
        per_run = []
        aggregate_truth, aggregate_prediction = [], []
        for run in runs:
            logits = _run_onnx(session, input_name, run["frames"])
            prediction = np.argmax(logits, axis=1).astype(np.int64)
            truth = run["labels"]
            decision = truth != ACTION_TRACK
            metrics = classification_metrics(truth, prediction)
            decision_metrics = classification_metrics(
                truth[decision], prediction[decision])
            selected_counts = {
                item["name"]: int(np.count_nonzero(run["selected_obstacle"] == item["index"]))
                for item in run["obstacles"]
            }
            per_run.append({
                "run": str(run["path"]), "random_seed": run["seed"],
                "active_window_s": [run["launch_time_s"], run["end_time_s"]],
                "frames": len(truth), "label_action_counts_track_left_right":
                    np.bincount(truth, minlength=3),
                "predicted_action_counts_track_left_right":
                    np.bincount(prediction, minlength=3),
                "decision_frames": int(np.count_nonzero(decision)),
                "decision_accuracy": decision_metrics["accuracy"],
                "decision_left_recall": decision_metrics[
                    "recall_track_left_right"][ACTION_LEFT],
                "decision_right_recall": decision_metrics[
                    "recall_track_left_right"][ACTION_RIGHT],
                "decision_direction_balanced_accuracy": float(np.mean([
                    decision_metrics["recall_track_left_right"][ACTION_LEFT],
                    decision_metrics["recall_track_left_right"][ACTION_RIGHT],
                ])),
                "all_frame_metrics": metrics,
                "decision_metrics": decision_metrics,
                "selected_obstacle_frame_counts": selected_counts,
                "retained_run_artifact_hashes": run["artifact_hashes"],
            })
            aggregate_truth.append(truth)
            aggregate_prediction.append(prediction)
        truth = np.concatenate(aggregate_truth)
        prediction = np.concatenate(aggregate_prediction)
        decision = truth != ACTION_TRACK
        overall = classification_metrics(truth, prediction)
        decision_metrics = classification_metrics(truth[decision], prediction[decision])
        policy_reports[name] = {
            "model": str(path.resolve()), "model_sha256": file_sha256(path),
            "runs": per_run,
            "aggregate": {
                "runs": len(per_run), "frames": len(truth),
                "label_action_counts_track_left_right": np.bincount(truth, minlength=3),
                "predicted_action_counts_track_left_right":
                    np.bincount(prediction, minlength=3),
                "decision_frames": int(np.count_nonzero(decision)),
                "decision_accuracy": decision_metrics["accuracy"],
                "decision_left_recall": decision_metrics[
                    "recall_track_left_right"][ACTION_LEFT],
                "decision_right_recall": decision_metrics[
                    "recall_track_left_right"][ACTION_RIGHT],
                "decision_direction_balanced_accuracy": float(np.mean([
                    decision_metrics["recall_track_left_right"][ACTION_LEFT],
                    decision_metrics["recall_track_left_right"][ACTION_RIGHT],
                ])),
                "all_frame_metrics": overall, "decision_metrics": decision_metrics,
            },
        }

    return {
        "format": "tinympc-u-course-off-policy-transfer-v1",
        "diagnostic_scope": "off-policy classification on retained camera/state streams",
        "closed_loop_success_claim_permitted": False,
        "interpretation_warning": (
            "Candidate actions did not affect retained trajectories. Accuracy and recall "
            "do not establish collision avoidance, stability, or course completion."),
        "action_contract": list(ACTION_NAMES),
        "course": str(course_path), "course_sha256": file_sha256(course_path),
        "decision_window": {
            "minimum_obstacle_delta_progress_m": -decision_behind_m,
            "maximum_obstacle_delta_progress_m": decision_ahead_m,
            "selection": "minimum eligible obstacle delta progress",
            "safe_side": "opposite sign of obstacle center signed path-normal offset",
            "outside_window": "TRACK",
        },
        "obstacle_path_coordinates": runs[0]["obstacles"],
        "evaluation_seeds": sorted(eval_seeds),
        "training_seeds": sorted(training_seeds),
        "training_seed_overlap": overlap,
        "training_seed_evidence": seed_evidence,
        "policies": policy_reports,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run", type=Path, action="append", required=True)
    parser.add_argument("--policy", action="append", required=True,
                        help="candidate in NAME=PATH form; repeatable")
    parser.add_argument("--course", type=Path, required=True)
    parser.add_argument("--training-manifest", type=Path, action="append", default=[])
    parser.add_argument("--bundle", action="append", default=[],
                        help="optional NAME=PATH bundle; verifies model hash and seeds")
    parser.add_argument("--decision-behind-m", type=float,
                        default=DEFAULT_DECISION_BEHIND_M)
    parser.add_argument("--decision-ahead-m", type=float,
                        default=DEFAULT_DECISION_AHEAD_M)
    parser.add_argument("--out", type=Path)
    args = parser.parse_args()
    try:
        report = evaluate_transfer(
            args.run, _policy_specs(args.policy), args.course,
            training_manifests=args.training_manifest,
            bundles=_bundle_specs(args.bundle),
            decision_behind_m=args.decision_behind_m,
            decision_ahead_m=args.decision_ahead_m)
    except (DatasetValidationError, OSError, ValueError, json.JSONDecodeError) as error:
        report = {"format": "tinympc-u-course-off-policy-transfer-v1",
                  "closed_loop_success_claim_permitted": False,
                  "accepted": False, "fatal_error": str(error)}
    payload = json.dumps(report, indent=2, allow_nan=False, default=_jsonable) + "\n"
    if args.out:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        args.out.write_text(payload)
    print(payload, end="")
    return 0 if "fatal_error" not in report else 1


if __name__ == "__main__":
    raise SystemExit(main())
