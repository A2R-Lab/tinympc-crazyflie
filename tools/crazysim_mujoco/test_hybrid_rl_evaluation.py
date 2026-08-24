#!/usr/bin/env python3

from __future__ import annotations

import importlib.util
import json
import math
from pathlib import Path
import tempfile
import unittest

import numpy as np


HERE = Path(__file__).resolve().parent


def load_module(name: str, path: Path):
    spec = importlib.util.spec_from_file_location(name, path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


analyzer = load_module("analyze_run", HERE / "analyze_run.py")
comparison = load_module("compare_hybrid_rl", HERE / "compare_hybrid_rl.py")


def synthetic_state(contact_index: int = 5) -> dict[str, np.ndarray]:
    t = np.arange(6, dtype=float)
    result = {
        "time_s": t,
        "x_m": np.asarray([0.0, 0.0, 1.0, 2.0, 3.0, 3.0]),
        "y_m": np.asarray([0.0, 0.1, 0.2, 0.3, 0.4, 0.9]),
        "z_m": np.full(6, 0.5),
        "qw": np.ones(6), "qx": np.zeros(6), "qy": np.zeros(6),
        "qz": np.zeros(6),
        "vx_mps": np.asarray([0.0, 0.0, 1.0, -0.1, 1.0, 0.0]),
        "vy_mps": np.zeros(6), "vz_mps": np.zeros(6),
        "wx_radps": np.zeros(6), "wy_radps": np.zeros(6),
        "wz_radps": np.zeros(6),
        "contacts": np.zeros(6),
    }
    result["contacts"][contact_index] = 1.0
    for motor in range(1, 5):
        result[f"rpm_{motor}"] = np.full(6, 1000.0)
    return result


def synthetic_course() -> dict:
    return {
        "name": "test", "description": "synthetic", "centerline": [[0, 0], [4, 0]],
        "pass_point": [3, 0.4], "pass_radius_m": 0.01,
        "maximum_final_cross_track_m": 0.5, "required_heading_change_deg": 0.0,
        "obstacles": [], "walls": [], "segments": [], "gates": [],
    }


def synthetic_vision() -> dict[str, np.ndarray]:
    return {
        "time_s": np.arange(6, dtype=float),
        "inference_ms": np.arange(1, 7, dtype=float),
        "emulated_latency_ms": np.full(6, 1000.0 / 30.0),
    }


class AnalyzeRunMetricsTest(unittest.TestCase):
    def test_metrics_stop_at_contact_free_completion(self) -> None:
        summary = analyzer.build_summary(
            synthetic_state(), 1.0, course=synthetic_course(),
            vision=synthetic_vision())
        self.assertTrue(summary["crashed"])
        self.assertTrue(summary["course_success"])
        self.assertFalse(summary["course_contact_before_completion"])
        self.assertEqual(summary["course_evaluation_end_time_s"], 4.0)
        self.assertAlmostEqual(
            summary["course_cross_track_error_rmse_m"], math.sqrt(0.075))
        self.assertAlmostEqual(summary[
            "course_reverse_tangential_motion_fraction"], 1.0 / 3.0)
        self.assertIsNotNone(summary["course_mean_horizontal_speed_active_mps"])
        self.assertEqual(summary["course_tangential_speed_min_mps"], -0.1)
        self.assertAlmostEqual(summary["vision_inference_latency_ms"]["p50"], 3.5)
        self.assertAlmostEqual(
            summary["vision_inference_latency_ms"]["emulated_delivery_mean"],
            1000.0 / 30.0)

    def test_contact_before_finish_truncates_and_fails(self) -> None:
        summary = analyzer.build_summary(
            synthetic_state(contact_index=3), 1.0, course=synthetic_course())
        self.assertFalse(summary["course_success"])
        self.assertTrue(summary["course_contact_before_completion"])
        self.assertIsNone(summary["course_completion_time_s"])
        self.assertEqual(summary["course_evaluation_end_time_s"], 3.0)

    def test_polyline_tangent_follows_curve(self) -> None:
        points = np.asarray([[0.8, 0.1], [1.1, 0.8]])
        distance, tangent = analyzer.point_to_polyline_metrics(
            points, np.asarray([[0, 0], [1, 0], [1, 1]], dtype=float))
        np.testing.assert_allclose(distance, [0.1, 0.1])
        np.testing.assert_allclose(tangent, [[1, 0], [0, 1]])


def comparison_summary(success: bool = True) -> dict:
    return {
        "course_success": success,
        "course_completion_time_s": 8.0 if success else None,
        "course_mean_horizontal_speed_mps": 0.5 if success else None,
        "course_obstacle_clearance_min_m": 0.2,
        "course_wall_clearance_min_m": 0.3,
        "course_cross_track_error_rmse_m": 0.1,
        "course_cross_track_error_p95_m": 0.2,
        "course_reverse_tangential_motion_fraction": 0.01,
        "vision_inference_latency_ms": {
            "p50": 1.0, "p95": 2.0, "maximum": 3.0,
            "emulated_delivery_mean": 1000.0 / 30.0,
        },
    }


class PairedComparisonTest(unittest.TestCase):
    def write_run(
        self, root: Path, controller: str, seed: int, *, duration: float = 35.0,
    ) -> Path:
        run = root / controller / f"seed_{seed}"
        run.mkdir(parents=True)
        config = {
            "format": "tinympc-crazysim-run-config-v1", "random_seed": seed,
            "duration_s": duration, "trajectory": "dronet_u", "course": "dronet_u",
            "firmware_time_factor": 0.18, "firmware_sha256": "same",
            "vision_model": {"sha256": controller},
            "vision_adapter": "dronet" if controller == "baseline" else "hybrid_rl",
        }
        (run / "run_config.json").write_text(json.dumps(config))
        (run / "summary.json").write_text(json.dumps(comparison_summary()))
        return run

    def make_matrix(self, root: Path):
        baseline = [self.write_run(root, "baseline", seed)
                    for seed in comparison.EXPECTED_SEEDS]
        hybrid = [self.write_run(root, "hybrid", seed)
                  for seed in comparison.EXPECTED_SEEDS]
        manifest = root / "training.json"
        manifest.write_text(json.dumps({"training_seeds": [1, 2, 3]}))
        return baseline, hybrid, manifest

    def test_accepts_exact_held_out_pairs_and_aggregates(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            baseline, hybrid, manifest = self.make_matrix(Path(directory))
            report, rows = comparison.compare(baseline, hybrid, manifest)
            self.assertEqual(len(rows), 6)
            self.assertEqual(report["held_out_seeds"], [101, 202, 303])
            self.assertEqual(
                report["controllers"]["hybrid_rl"]["contact_free_completions"], 3)

    def test_rejects_training_overlap(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            baseline, hybrid, manifest = self.make_matrix(Path(directory))
            manifest.write_text(json.dumps({"training_seeds": [1, 101]}))
            with self.assertRaisesRegex(ValueError, "held-out seeds"):
                comparison.compare(baseline, hybrid, manifest)

    def test_accepts_exported_bundle_training_section(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            baseline, hybrid, manifest = self.make_matrix(Path(directory))
            manifest.write_text(json.dumps({
                "format": "tinympc-vision-rl-mpc-poc-v1",
                "training": {"training_seeds": [1, 2, 3]},
            }))
            report, _ = comparison.compare(baseline, hybrid, manifest)
            self.assertEqual(report["training_seed_count"], 3)

    def test_rejects_config_drift(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            baseline, hybrid, manifest = self.make_matrix(Path(directory))
            config_path = hybrid[1] / "run_config.json"
            config = json.loads(config_path.read_text())
            config["duration_s"] = 34.0
            config_path.write_text(json.dumps(config))
            with self.assertRaisesRegex(ValueError, "configuration drift"):
                comparison.compare(baseline, hybrid, manifest)

    def test_rejects_checkpoint_drift_between_seeds(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            baseline, hybrid, manifest = self.make_matrix(Path(directory))
            config_path = hybrid[1] / "run_config.json"
            config = json.loads(config_path.read_text())
            config["vision_model"] = {"sha256": "different-checkpoint"}
            config_path.write_text(json.dumps(config))
            with self.assertRaisesRegex(ValueError, "identity drift"):
                comparison.compare(baseline, hybrid, manifest)

    def test_rejects_missing_seed(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            baseline, hybrid, manifest = self.make_matrix(Path(directory))
            with self.assertRaisesRegex(ValueError, "exactly seeds"):
                comparison.compare(baseline[:-1], hybrid, manifest)


if __name__ == "__main__":
    unittest.main()
