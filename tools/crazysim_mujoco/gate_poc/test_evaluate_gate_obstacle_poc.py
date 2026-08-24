#!/usr/bin/env python3
"""Focused synthetic coverage for gate_poc/evaluate_gate_obstacle_poc.py."""

from __future__ import annotations

import csv
import importlib.util
import json
import tempfile
import unittest
from pathlib import Path


MODULE_PATH = Path(__file__).with_name("evaluate_gate_obstacle_poc.py")
SPEC = importlib.util.spec_from_file_location("gate_poc_evaluation", MODULE_PATH)
assert SPEC and SPEC.loader
evaluation = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(evaluation)


def make_run(root: Path, name: str, seed: int, *, complete: bool,
             crashed: bool, clearance: float | None, gate: bool | None,
             with_summary: bool = True, course: str = "gate_obstacle_poc",
             adapter: str = "combined_gate_rl") -> Path:
    directory = root / name
    directory.mkdir()
    (directory / "run_config.json").write_text(json.dumps({
        "random_seed": seed, "course": course,
        "vision_adapter": adapter}))
    if with_summary:
        summary = {
            "course": course, "course_success": complete,
            "course_pass_point_reached": complete, "crashed": crashed,
            "course_contact_before_completion": crashed,
            "contact_count_max": 2 if crashed else 0,
            "first_crash_time_s": 3.0 if crashed else None,
            "first_crash_after_launch_s": 2.0 if crashed else None,
            "course_obstacle_clearance_min_m": clearance,
            "course_gates_passed_in_order": gate,
            "course_gate_results": ([{"name": "gate", "crossed": gate,
                                      "time_s": 2.0, "lateral_error_m": 0.01,
                                      "vertical_error_m": 0.02,
                                      "clearance_margin_m": 0.1}]
                                    if gate is not None else []),
        }
        (directory / "summary.json").write_text(json.dumps(summary))
    with (directory / "vision.csv").open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=(
            "time_s", "gate_valid", "gate_confidence", "gate_reason",
            "rl_action", "inference_ms"))
        writer.writeheader()
        writer.writerows((
            {"time_s": "1", "gate_valid": "1", "gate_confidence": ".8",
             "gate_reason": "accepted", "rl_action": "0", "inference_ms": "1"},
            {"time_s": "2", "gate_valid": "0", "gate_confidence": ".2",
             "gate_reason": "ordering", "rl_action": "2", "inference_ms": "3"},
        ))
    (directory / "firmware.log").write_text("gate associated\nother\n")
    (directory / "state.csv").write_text("time_s\n3.0\n")
    return directory


class GatePocEvaluationTest(unittest.TestCase):
    def test_reports_ground_truth_and_pairs_without_hiding_incomplete(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            candidate_gate = make_run(root, "candidate_gate", 7, complete=True,
                                      crashed=False, clearance=.2, gate=True)
            candidate_obstacle = make_run(root, "candidate_obstacle", 7, complete=False,
                                          crashed=True, clearance=-.1, gate=None,
                                          course="gate_obstacle_poc_obstacle_only")
            incomplete = make_run(root, "incomplete", 8, complete=False, crashed=False,
                                  clearance=None, gate=None, with_summary=False,
                                  course="gate_obstacle_poc_obstacle_only")
            baseline = make_run(root, "baseline", 7, complete=True, crashed=False,
                                clearance=.05, gate=None,
                                course="gate_obstacle_poc_obstacle_only",
                                adapter="hybrid_rl")
            report = evaluation.evaluate({
                "candidate_gate_obstacle": [candidate_gate],
                "candidate_obstacle_only": [candidate_obstacle, incomplete],
                "baseline_obstacle_only": [baseline],
            })
        row = report["groups"]["candidate_gate_obstacle"]["runs"][0]
        self.assertTrue(row["ground_truth_gate"]["passed_in_order"])
        self.assertEqual(row["vision_gate_association"]["gate_valid_samples"], 1)
        aggregate = report["groups"]["candidate_obstacle_only"]["aggregate"]
        self.assertEqual(aggregate["trials"], 2)
        self.assertEqual(aggregate["incomplete_trials"], 1)
        self.assertEqual(aggregate["course_completion_rate_all_trials"], 0.0)
        self.assertEqual(aggregate["sum_contact_count_max"], 2.0)
        self.assertEqual(aggregate["maximum_contact_count_max"], 2.0)
        pair = report["obstacle_only_regression"]["paired_runs"][0]
        self.assertAlmostEqual(pair["clearance_delta_m_candidate_minus_baseline"], -.15)

    def test_no_detection_is_promoted_to_gate_success(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            run = make_run(root, "run", 1, complete=False, crashed=False,
                           clearance=.1, gate=False)
            row = evaluation.make_row(run, "candidate_gate_obstacle")
        self.assertFalse(row["ground_truth_gate"]["passed_in_order"])
        self.assertEqual(row["vision_gate_association"]["gate_valid_samples"], 1)


if __name__ == "__main__":
    unittest.main()
