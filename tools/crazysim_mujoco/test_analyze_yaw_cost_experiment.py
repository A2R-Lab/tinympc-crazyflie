#!/usr/bin/env python3
"""Focused synthetic tests for analyze_yaw_cost_experiment.py."""

from __future__ import annotations

import csv
import json
import math
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

import numpy as np

import analyze_yaw_cost_experiment as analyzer


def write_header(path: Path, samples: int = 41) -> None:
    rows = []
    for theta in np.linspace(0.0, 2.0 * math.pi, samples):
        x = math.sin(theta)
        y = 1.0 - math.cos(theta)
        yaw = theta
        rows.append(
            "  {" + ", ".join(f"{value:.9g}f" for value in (
                x, y, 0.4, math.cos(yaw / 2.0), 0.0, 0.0,
                math.sin(yaw / 2.0), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            )) + "},"
        )
    path.write_text(
        "static const float trajectory_reference_data[41][13] = {\n"
        + "\n".join(rows) + "\n};\n"
    )


def write_run(root: Path, name: str, yaw_error: float, lateral_error: float,
              altitude_error: float, contact: bool = False) -> Path:
    run = root / name
    run.mkdir()
    config = {
        "launch_time_s": 1.0, "trajectory": "circle", "reference_mode": "progress",
        "yaw_cost_mode": name,
    }
    (run / "run_config.json").write_text(json.dumps(config))
    (run / "summary.json").write_text(json.dumps({"launch_time_s": 1.0, "crashed": contact}))
    (run / "firmware.log").write_text(
        "TINYMPC-E: Trajectory origin=(2.00,3.00,1.10) yaw=0.0deg hold=0.4s\n"
        "TINYMPC-E: Progress path sample=20.00/40\n"
    )
    fields = [
        "time_s", "x_m", "y_m", "z_m", "qw", "qx", "qy", "qz",
        "wz_radps", "contacts", "airborne",
    ]
    with (run / "state.csv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for index, theta in enumerate(np.linspace(0.0, math.pi, 101)):
            desired_yaw = theta
            actual_yaw = desired_yaw + yaw_error
            normal_x, normal_y = -math.sin(theta), math.cos(theta)
            writer.writerow({
                "time_s": 1.0 + index * 0.05,
                "x_m": 2.0 + math.sin(theta) + lateral_error * normal_x,
                "y_m": 3.0 + 1.0 - math.cos(theta) + lateral_error * normal_y,
                "z_m": 1.5 + altitude_error,
                "qw": math.cos(actual_yaw / 2.0), "qx": 0.0, "qy": 0.0,
                "qz": math.sin(actual_yaw / 2.0), "wz_radps": 0.15,
                "contacts": int(contact and index == 80), "airborne": 1,
            })
    return run


class YawCostAnalysisTest(unittest.TestCase):
    def test_semicircle_is_half_horizontal_arc(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            header = Path(directory) / "circle.h"
            write_header(header)
            raw = analyzer.load_trajectory_header(header)
            half = analyzer.semicircle_reference(raw)
            self.assertAlmostEqual(half[0, 0], 0.0, places=6)
            self.assertAlmostEqual(half[-1, 0], 0.0, places=6)
            self.assertAlmostEqual(half[-1, 1], 2.0, places=6)

    def test_acceptance_rule_and_cli_artifacts(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            header = root / "circle.h"
            write_header(header)
            runs = {
                "baseline": write_run(root, "baseline", 0.20, 0.10, 0.05),
                "yaw_angle": write_run(root, "yaw_angle", 0.10, 0.11, 0.055),
                "yaw_rate": write_run(root, "yaw_rate", 0.20, 0.13, 0.05),
                "motor_differential": write_run(
                    root, "motor_differential", 0.05, 0.10, 0.05, contact=True
                ),
            }
            output = root / "output"
            subprocess.run([
                sys.executable, str(Path(analyzer.__file__)),
                "--baseline", str(runs["baseline"]),
                "--yaw-angle", str(runs["yaw_angle"]),
                "--yaw-rate", str(runs["yaw_rate"]),
                "--motor-differential", str(runs["motor_differential"]),
                "--trajectory-header", str(header), "--out", str(output),
            ], check=True, capture_output=True, text=True)
            report = json.loads((output / "comparison.json").read_text())
            by_mode = {item["mode"]: item for item in report["results"]}
            self.assertTrue(by_mode["baseline"]["valid_launch"])
            self.assertTrue(by_mode["baseline"]["semicircle_complete"])
            self.assertTrue(by_mode["yaw_angle"]["promising"])
            self.assertFalse(by_mode["yaw_rate"]["promising"])
            self.assertFalse(by_mode["motor_differential"]["promising"])
            self.assertTrue(by_mode["motor_differential"]["contact"])
            self.assertEqual(by_mode["yaw_angle"]["cost_metadata"]["run_config_fields"]["yaw_cost_mode"], "yaw_angle")
            self.assertTrue((output / "comparison.csv").is_file())
            self.assertGreater((output / "comparison.png").stat().st_size, 1000)

    def test_zero_baseline_error_does_not_emit_nonfinite_json(self) -> None:
        baseline = {
            "mode": "baseline", "tangent_yaw_mae_deg": 0.0,
            "yaw_rate_rmse_rad_s": 0.0, "cross_track_rmse_m": 0.0,
            "altitude_error_rmse_m": 0.0, "contact": False,
        }
        candidate = {
            "mode": "yaw_angle", "tangent_yaw_mae_deg": 0.0,
            "yaw_rate_rmse_rad_s": 0.0, "cross_track_rmse_m": 0.01,
            "altitude_error_rmse_m": 0.0, "contact": False,
        }
        analyzer.evaluate_promising([baseline, candidate])
        self.assertIsNone(candidate["comparison_to_baseline"]["cross_track_degradation_fraction"])
        self.assertFalse(candidate["promising"])
        json.dumps([baseline, candidate], allow_nan=False)


if __name__ == "__main__":
    unittest.main()
