from __future__ import annotations

import json
import tempfile
import unittest
from pathlib import Path
from unittest import mock

import numpy as np

from tools.crazysim_mujoco.rl import ACTION_LEFT, ACTION_RIGHT, ACTION_TRACK
from tools.crazysim_mujoco.rl.evaluate_u_course_transfer import (
    DatasetValidationError, _extract_training_seeds, evaluate_transfer,
    privileged_safe_side_labels,
)


def course() -> dict:
    return {
        "centerline": [[0, 0], [3, 0], [3, 2], [0, 2]],
        "obstacles": [
            {"name": "lower_first", "center": [1.0, -0.5]},
            {"name": "upper_first", "center": [2.0, 0.5]},
            {"name": "lower_return", "center": [2.0, 1.5]},
            {"name": "upper_return", "center": [1.0, 2.5]},
        ],
    }


class FakeInput:
    shape = [None, 2, 160, 160]
    type = "tensor(float)"
    name = "frames"


class FakeOutput:
    shape = [None, 3]


class FakeSession:
    def __init__(self, *_args, **_kwargs):
        pass

    def get_inputs(self):
        return [FakeInput()]

    def get_outputs(self):
        return [FakeOutput()]

    def run(self, _outputs, inputs):
        count = len(inputs["frames"])
        # TRACK, LEFT, RIGHT, LEFT for the four retained frames.
        actions = np.resize(np.asarray([0, 1, 2, 1]), count)
        logits = np.full((count, 3), -1.0, dtype=np.float32)
        logits[np.arange(count), actions] = 2.0
        return [logits]


def retained_run(seed: int = 101) -> dict:
    obstacle = [
        {"index": 0, "name": "first", "path_progress_m": 1.0,
         "signed_cross_track_m": -0.5, "safe_action": 1,
         "safe_action_name": "LEFT"},
    ]
    return {
        "path": Path("/retained/seed_101"), "seed": seed,
        "frames": np.zeros((4, 2, 160, 160), dtype=np.uint8),
        "labels": np.asarray([0, 1, 2, 1]),
        "selected_obstacle": np.asarray([-1, 0, 0, 0]),
        "progress": np.arange(4, dtype=float), "times": np.arange(4, dtype=float),
        "launch_time_s": 1.0, "end_time_s": 2.0, "obstacles": obstacle,
        "artifact_hashes": {"state.csv": "state-hash"},
    }


class PrivilegedLabelTest(unittest.TestCase):
    def test_safe_side_rotates_with_path_normal(self):
        points = np.asarray([
            [0.7, 0.0], [1.7, 0.0], [2.3, 2.0], [1.3, 2.0], [0.0, 0.0],
        ])
        labels, selected, _, obstacles = privileged_safe_side_labels(
            points, course(), decision_behind_m=0.05, decision_ahead_m=0.4)
        np.testing.assert_array_equal(
            labels, [ACTION_LEFT, ACTION_RIGHT, ACTION_RIGHT, ACTION_LEFT, ACTION_TRACK])
        np.testing.assert_array_equal(selected, [0, 1, 2, 3, -1])
        self.assertEqual([item["safe_action_name"] for item in obstacles],
                         ["LEFT", "RIGHT", "RIGHT", "LEFT"])

    def test_optimizer_seed_is_not_data_leakage(self):
        seeds = _extract_training_seeds({
            "training": {"training_seeds": [11, 22]},
            "optimizer_seed": 101,
            "episodes_detail": [{"seed": 33}],
        })
        self.assertEqual(seeds, {11, 22, 33})


class TransferReportTest(unittest.TestCase):
    def test_report_is_explicitly_off_policy_and_has_counts(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            model = root / "candidate.onnx"
            model.write_bytes(b"model")
            course_path = root / "course.json"
            course_path.write_text(json.dumps(course()))
            with mock.patch(
                    "tools.crazysim_mujoco.rl.evaluate_u_course_transfer.load_retained_run",
                    return_value=retained_run()), mock.patch(
                    "onnxruntime.InferenceSession", FakeSession):
                report = evaluate_transfer(
                    [Path("unused")], {"candidate": model}, course_path)
            self.assertFalse(report["closed_loop_success_claim_permitted"])
            aggregate = report["policies"]["candidate"]["aggregate"]
            np.testing.assert_array_equal(
                aggregate["predicted_action_counts_track_left_right"], [1, 2, 1])
            self.assertEqual(aggregate["decision_accuracy"], 1.0)
            self.assertEqual(aggregate["decision_left_recall"], 1.0)
            self.assertEqual(aggregate["decision_right_recall"], 1.0)

    def test_training_evaluation_seed_overlap_fails_closed(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            model = root / "candidate.onnx"
            model.write_bytes(b"model")
            course_path = root / "course.json"
            course_path.write_text(json.dumps(course()))
            manifest = root / "training.json"
            manifest.write_text(json.dumps({"training_seeds": [11, 101]}))
            with mock.patch(
                    "tools.crazysim_mujoco.rl.evaluate_u_course_transfer.load_retained_run",
                    return_value=retained_run()):
                with self.assertRaisesRegex(
                        DatasetValidationError, "training/evaluation seed overlap"):
                    evaluate_transfer(
                        [Path("unused")], {"candidate": model}, course_path,
                        training_manifests=[manifest])


if __name__ == "__main__":
    unittest.main()
