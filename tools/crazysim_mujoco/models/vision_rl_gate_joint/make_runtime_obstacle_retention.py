#!/usr/bin/env python3
"""Build actor-only obstacle retention from the pinned frozen-teacher smoke."""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
from pathlib import Path

import cv2
import numpy as np

COURSE_NAME = "gate_obstacle_poc_obstacle_only"
RANDOM_SEED = 3201
TEACHER_SHA256 = "291d1de3a7152f09cc2e96f4a6973d322c95bade4e5c8249c7bc14f7b656187e"
COURSE_SHA256 = "1fcc8d41f70a13528de9ea8be92ed507156b39d932f748b1c2a13fca1302e14c"
EXPECTED_SOURCE_HASHES = {
    "run_config_sha256": "e0bf5eedca94f374f123bb7a86c633e3663080d9a0add1a381c19d593567aa76",
    "summary_sha256": "4b30c1e6567c141fc2687876ae3beaf97c5cbe1ba6ace76690a54c8a64dfb946",
    "state_sha256": "b8baf5f9abb27a62a03fb91e9b95dd445edfab99ff606bd169addb756ff61a72",
    "vision_sha256": "f7010ea8951de4a4f857e7685b37d8fc9767cc476b66f76793c375658a1b36d6",
    "video_sha256": "91da516dbc92c22e7393683c3c61d5dbf57702e97a78944c159f443e0c665d4b",
}
EXPECTED_SOURCE_ACTION_COUNTS = np.asarray([619, 0, 46], dtype=np.int64)
EXPECTED_ARTIFACT_ACTION_COUNTS = np.asarray([618, 0, 46], dtype=np.int64)
SELECTION_RULE = (
    "post-HM01B0 H.264 sequence N uses adjacent decoded frames [N-2,N-1] "
    "and vision.csv rl_action at N; select N>=2 and time_s<=summary course_completion_time_s"
)


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as source:
        for block in iter(lambda: source.read(1 << 20), b""):
            digest.update(block)
    return digest.hexdigest()


def canonical_json(value) -> str:
    return json.dumps(value, sort_keys=True, separators=(",", ":"))


def array_sha256(value: np.ndarray) -> str:
    array = np.ascontiguousarray(value)
    digest = hashlib.sha256()
    digest.update(array.dtype.str.encode())
    digest.update(str(array.shape).encode())
    digest.update(array.tobytes())
    return digest.hexdigest()


def write_retention(output: Path, frames: np.ndarray, actions: np.ndarray,
                    sequences: np.ndarray, frame_indices: np.ndarray,
                    times: np.ndarray, source: dict) -> dict:
    """Write the exact schema after source validation and frame/action alignment."""
    frames=np.asarray(frames,dtype=np.uint8);actions=np.asarray(actions,dtype=np.int64)
    sequences=np.asarray(sequences,dtype=np.int64);frame_indices=np.asarray(frame_indices,dtype=np.int64)
    times=np.asarray(times,dtype=np.float64)
    counts = np.bincount(actions, minlength=3).astype(np.int64)
    array_hashes={key:array_sha256(value) for key,value in (
        ("frames",frames),("expert_action",actions),("source_sequence",sequences),
        ("source_frame_index",frame_indices),("source_time_s",times))}
    output.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(
        output,
        frames=frames,
        expert_action=actions,
        source_sequence=sequences,
        source_frame_index=frame_indices,
        source_time_s=times,
        action_counts=counts,
        array_hashes_json=np.asarray(canonical_json(array_hashes)),
        source_run_json=np.asarray(canonical_json(source)),
        teacher_policy_sha256=np.asarray(TEACHER_SHA256),
        source_course_sha256=np.asarray(COURSE_SHA256),
        selection_rule=np.asarray(SELECTION_RULE),
    )
    return {
        "output": str(output.resolve()), "samples": int(len(actions)),
        "action_counts_track_left_right": counts.tolist(),
        "sequence_range": [int(sequences[0]), int(sequences[-1])],
        "time_range_s": [float(times[0]), float(times[-1])],
        "sha256": sha256(output), "source": source,
    }


def build(run: Path, course_path: Path, output: Path) -> dict:
    run = run.resolve()
    course_path = course_path.resolve()
    paths = {name: run / filename for name, filename in (
        ("run_config", "run_config.json"), ("summary", "summary.json"),
        ("state", "state.csv"), ("vision", "vision.csv"),
        ("video", "fpv_camera.mp4"))}
    for path in (*paths.values(), course_path):
        if not path.is_file():
            raise FileNotFoundError(path)
    actual_hashes = {f"{name}_sha256": sha256(path) for name, path in paths.items()}
    if actual_hashes != EXPECTED_SOURCE_HASHES:
        raise RuntimeError("runtime obstacle retention source hashes do not match the pinned seed-3201 run")
    if sha256(course_path) != COURSE_SHA256:
        raise RuntimeError("runtime obstacle retention course hash mismatch")

    config = json.loads(paths["run_config"].read_text())
    model = config.get("vision_model")
    calibration = config.get("camera_calibration")
    if (config.get("course") != COURSE_NAME or config.get("random_seed") != RANDOM_SEED or
            config.get("vision_adapter") != "hybrid_rl" or not isinstance(model, dict) or
            model.get("sha256") != TEACHER_SHA256):
        raise RuntimeError("runtime obstacle retention run/course/teacher contract mismatch")
    teacher_path = Path(model.get("path", ""))
    if (not teacher_path.is_absolute() or not teacher_path.is_file() or
            sha256(teacher_path) != TEACHER_SHA256):
        raise RuntimeError("runtime obstacle retention frozen teacher artifact mismatch")
    if (config.get("camera_fps") != 30.0 or not config.get("camera_capture_enabled") or
            not config.get("camera_inference_enabled") or not isinstance(calibration, dict) or
            calibration.get("model") != "Himax HM01B0" or
            calibration.get("resolution") != [160, 160] or
            calibration.get("poc_transform") != "principal_point+plumb_bob+isaac_gray_response+seeded_noise" or
            calibration.get("poc_sensor_seed") != RANDOM_SEED or
            config.get("course_manifest_sha256") != COURSE_SHA256):
        raise RuntimeError("runtime obstacle retention post-HM01B0 camera contract mismatch")

    summary = json.loads(paths["summary"].read_text())
    completion = summary.get("course_completion_time_s")
    if (summary.get("course") != COURSE_NAME or summary.get("contact_count_max") != 0 or
            summary.get("course_contact_before_completion") is not False or
            summary.get("course_pass_point_reached") is not True or summary.get("crashed") is not False or
            not isinstance(completion, (int, float)) or not np.isfinite(completion) or completion <= 0.0):
        raise RuntimeError("runtime obstacle retention summary is not a contact-free pass-point run")

    with paths["state"].open(newline="") as source_file:
        state_rows = list(csv.DictReader(source_file))
    state_times = np.asarray([float(row["time_s"]) for row in state_rows], dtype=np.float64)
    contacts = np.asarray([int(row["contacts"]) for row in state_rows], dtype=np.int64)
    if (len(state_rows) < 2 or np.any(~np.isfinite(state_times)) or
            np.any(np.diff(state_times) <= 0.0) or state_times[-1] < completion or np.any(contacts != 0)):
        raise RuntimeError("runtime obstacle retention state.csv is incomplete or contains contact")

    with paths["vision"].open(newline="") as source_file:
        vision_rows = list(csv.DictReader(source_file))
    if len(vision_rows) != 900:
        raise RuntimeError("pinned runtime obstacle vision.csv must contain exactly 900 frames")
    sequences = np.asarray([int(row["sequence"]) for row in vision_rows], dtype=np.int64)
    times = np.asarray([float(row["time_s"]) for row in vision_rows], dtype=np.float64)
    actions = np.asarray([int(row["rl_action"]) for row in vision_rows], dtype=np.int64)
    adapters = [row["adapter"] for row in vision_rows]
    scheduled = np.asarray([int(row["scheduled_delivery_sequence"]) for row in vision_rows], dtype=np.int64)
    expected_sequences = np.arange(1, 901, dtype=np.int64)
    expected_times = np.round(expected_sequences / 30.0, 3)
    if (not np.array_equal(sequences, expected_sequences) or
            not np.array_equal(scheduled, sequences + 1) or adapters != ["hybrid_rl"] * 900 or
            np.any(~np.isfinite(times)) or np.any(np.abs(times - expected_times) > 5.01e-4) or
            np.any((actions < 0) | (actions > 2))):
        raise RuntimeError("runtime obstacle vision sequence/time/action contract mismatch")
    through_completion = times <= float(completion)
    source_counts = np.bincount(actions[through_completion], minlength=3).astype(np.int64)
    summary_counts = np.asarray(
        summary.get("vision_inference_latency_ms", {}).get("rl_action_counts_track_left_right"),
        dtype=np.int64)
    if (not np.array_equal(source_counts, EXPECTED_SOURCE_ACTION_COUNTS) or
            not np.array_equal(summary_counts, EXPECTED_SOURCE_ACTION_COUNTS)):
        raise RuntimeError("runtime obstacle source/summary action counts mismatch")

    capture = cv2.VideoCapture(str(paths["video"]))
    fps = float(capture.get(cv2.CAP_PROP_FPS))
    width = int(capture.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
    decoded = []
    while True:
        ok, frame = capture.read()
        if not ok:
            break
        decoded.append(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY))
    capture.release()
    if abs(fps - 30.0) > 1.0e-6 or (width, height) != (160, 160) or len(decoded) != 900:
        raise RuntimeError(f"pinned runtime obstacle video must decode as 900 160x160 frames at 30 Hz, got {len(decoded)} {width}x{height} at {fps:g}")
    decoded = np.asarray(decoded, dtype=np.uint8)

    selected = (sequences >= 2) & through_completion
    selected_sequences = sequences[selected]
    selected_indices = selected_sequences - 1
    selected_actions = actions[selected]
    selected_frames = np.asarray([
        np.stack((decoded[sequence - 2], decoded[sequence - 1]))
        for sequence in selected_sequences
    ], dtype=np.uint8)
    if not np.array_equal(np.bincount(selected_actions, minlength=3), EXPECTED_ARTIFACT_ACTION_COUNTS):
        raise RuntimeError("runtime obstacle adjacent-frame action counts mismatch")
    source = {"path": str(run), "course_completion_time_s": float(completion), **actual_hashes}
    return write_retention(output, selected_frames, selected_actions, selected_sequences,
                           selected_indices, times[selected], source)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run", type=Path, required=True)
    parser.add_argument("--course", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    print(json.dumps(build(args.run, args.course, args.output), indent=2))


if __name__ == "__main__":
    main()
