#!/usr/bin/env python3
"""Synthetic tests for analyze_progress_speed_ladder.py."""

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

import analyze_progress_speed_ladder as analyzer


def write_header(path: Path, *, curvature_spike: bool = False) -> None:
    rows = []
    for theta in np.linspace(0.0, 2.0 * math.pi, 751):
        x = math.sin(theta) + (0.08 if curvature_spike and len(rows) == 9 else 0.0)
        values = (
            x, 1.0 - math.cos(theta), 0.4,
            math.cos(theta / 2.0), 0.0, 0.0, math.sin(theta / 2.0),
            math.cos(theta), math.sin(theta), 0.0, 0.0, 0.0, 1.0,
        )
        rows.append("  {" + ", ".join(f"{value:.9g}f" for value in values) + "},")
    path.write_text("static const float trajectory_reference_data[751][13] = {\n" + "\n".join(rows) + "\n};\n")


def write_run(root: Path, speed: float, contact: bool, *, uncapped: bool = False,
              sustained_altitude_error: bool = False) -> Path:
    run = root / f"speed_{speed:.3f}"
    run.mkdir()
    (run / "run_config.json").write_text(json.dumps({
        "trajectory": "circle", "reference_mode": "progress",
        "launch_time_s": 1.0, "progress_speed_mps": speed,
        "progress_reference_limits": "uncapped" if uncapped else "default",
    }))
    (run / "summary.json").write_text(json.dumps({"launch_time_s": 1.0, "crashed": contact}))
    final_progress = 600.0 if contact else 750.0
    completion = "TINYMPC-E: Progress path complete sample=750.00/750\n" if not contact else ""
    limits_banner = (
        "TINYMPC-E: Progress reference limits=uncapped projection_advance=finite-unbounded "
        "yaw_phase_slew=uncapped roll_pitch=uncapped local_yaw=+/-15deg\n"
        if uncapped else
        "TINYMPC-E: Progress reference limits=default projection_advance=0.02m "
        "yaw_phase_slew=90deg/s roll_pitch=+/-10deg local_yaw=+/-15deg\n")
    diagnostics = (
        "TINYMPC-E: Progress reference diagnostic t=0.20 sample=10 kappa=1 kappa_v=1 "
        "kappa_v2=1 bank_deg=5.82 thrust_scale=0.005 ref_rod=0.05 jump=10\n"
        "TINYMPC-E: Progress reference diagnostic t=0.22 sample=20 kappa=1 kappa_v=1 "
        "kappa_v2=1 bank_deg=5.82 thrust_scale=0.005 ref_rod=0.05 jump=10\n"
        if uncapped else "")
    (run / "firmware.log").write_text(
        limits_banner
        +
        f"TINYMPC-E: Progress path ready samples=751 speed={speed:.3f}..{speed:.3f}m/s curvature_gain=0.75m projection_step=0.02m\n"
        + diagnostics
        + f"TINYMPC-E: Progress path sample={final_progress:.2f}/750\n" + completion
        + "TINYMPC-E: MPC: iterations=5\n"
    )
    (run / "simulator.log").write_text("[crazysim] max_rpm     : 22000\n")
    fields = [
        "time_s", "x_m", "y_m", "z_m", "qw", "qx", "qy", "qz",
        "vx_mps", "vy_mps", "vz_mps", "wx_radps", "wy_radps", "wz_radps",
        "rpm_1", "rpm_2", "rpm_3", "rpm_4", "contacts", "airborne",
    ]
    maximum_theta = 1.6 * math.pi if contact else 2.0 * math.pi
    with (run / "state.csv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields); writer.writeheader()
        for index, theta in enumerate(np.linspace(0.0, maximum_theta, 301)):
            crash_sample = contact and index == 300
            writer.writerow({
                "time_s": 1.0 + index * 0.02,
                "x_m": math.sin(theta), "y_m": 1.0 - math.cos(theta),
                "z_m": 1.8 if sustained_altitude_error and index >= 20 else 1.5,
                "qw": math.cos(theta / 2.0), "qx": 0.0, "qy": 0.0, "qz": math.sin(theta / 2.0),
                "vx_mps": speed * math.cos(theta), "vy_mps": speed * math.sin(theta), "vz_mps": 0.0,
                "wx_radps": 0.0, "wy_radps": 0.0, "wz_radps": speed,
                "rpm_1": 16000, "rpm_2": 16000, "rpm_3": 16000, "rpm_4": 16000,
                "contacts": int(crash_sample), "airborne": 1,
            })
    return run


class ProgressSpeedLadderTest(unittest.TestCase):
    def test_first_sustained_crossing(self) -> None:
        time = np.arange(0.0, 2.0, 0.1)
        values = np.zeros_like(time)
        values[5:12] = 2.0
        self.assertAlmostEqual(analyzer.first_sustained(time, values, 1.0, 0.5), 0.5)
        self.assertIsNone(analyzer.first_sustained(time, values, 3.0, 0.5))

    def test_cli_reports_pass_failure_and_skipped_speed(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            header = root / "circle.h"; write_header(header)
            experiment = root / "experiment"; experiment.mkdir()
            write_run(experiment, 0.10, False)
            write_run(experiment, 0.20, True)
            (experiment / "experiment_plan.json").write_text(json.dumps({
                "planned_speeds_mps": [0.10, 0.20, 0.30],
            }))
            output = root / "output"
            subprocess.run([
                sys.executable, str(Path(analyzer.__file__)),
                "--experiment-root", str(experiment), "--circle-header", str(header),
                "--out", str(output),
            ], check=True, capture_output=True, text=True)
            report = json.loads((output / "comparison.json").read_text())
            self.assertTrue(report["runs"][0]["passed"])
            self.assertFalse(report["runs"][1]["passed"])
            self.assertTrue(report["runs"][0]["progress"]["firmware_complete_750_of_750"])
            self.assertEqual(report["skipped"][0]["requested_speed_mps"], 0.30)
            self.assertEqual(report["skipped"][0]["status"], "skipped_after_first_failure")
            corrected = report["runs"][0]["tracking"]["corrected_yaw_rate_50hz"]
            self.assertEqual(corrected["sample_rate_hz"], 50.0)
            self.assertIn("commanded_curvature_speed_rate_radps", corrected)
            self.assertIn("body_wz_minus_euler_yaw_rate_radps", corrected)
            self.assertEqual(report["runs"][0]["progress_reference_limits"], "default")
            self.assertTrue((output / "comparison.csv").exists())
            self.assertGreater((output / "comparison.png").stat().st_size, 1000)
            self.assertEqual(len(list((output / "diagnostics").glob("*.png"))), 2)
            self.assertEqual(len(list((output / "diagnostics").glob("*_corrected_rate_50hz.csv"))), 2)
            self.assertEqual(len(list((output / "diagnostics").glob("*_raw_reference_by_sample.csv"))), 2)

    def test_curvature_artifact_and_uncapped_safety_classification(self) -> None:
        route = np.zeros((101, 13))
        route[:, 0] = np.arange(101)
        curvature = np.ones(100)
        curvature[8] = 10.0
        geometry = {
            "curvature": curvature,
            "segment": np.diff(route[:, :3], axis=0),
            "length": np.ones(100),
            "tangent": np.tile([1.0, 0.0, 0.0], (100, 1)),
        }
        severity = analyzer.reference_severity(route, geometry, 1.0)["summary"]
        self.assertTrue(severity["startup_curvature_artifact"]["exposed"])
        self.assertEqual(severity["startup_curvature_artifact"]["first_segment_sample"], 8)
        self.assertGreater(severity["raw_bank_tilt_deg"]["max_abs"], 45.0)
        invalid_jump = analyzer.progress_jump_evidence([
            {"progress_sample": 0.0}, {"progress_sample": 61.0},
        ])
        self.assertTrue(invalid_jump["invalid"])
        self.assertTrue(invalid_jump["over_60_segment_window"])
        nonmonotonic = analyzer.progress_jump_evidence([
            {"progress_sample": 10.0}, {"progress_sample": 9.0},
        ])
        self.assertTrue(nonmonotonic["nonmonotonic"])

        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            header = root / "circle.h"; write_header(header, curvature_spike=True)
            run = write_run(root, 1.0, False, uncapped=True,
                            sustained_altitude_error=True)
            result, _ = analyzer.analyze_run(run, None, analyzer.load_header(header))
            self.assertFalse(result["passed"])
            self.assertEqual(result["progress_reference_limits"], "uncapped")
            self.assertEqual(result["evidence_label"], "artifact_exposed")
            self.assertIn("altitude_error_over_0p20_m",
                          result["safety"]["uncapped_failure_reasons"])
            self.assertFalse(result["progress"]["offline_completion_used_as_authority"])
            self.assertEqual(result["progress"]["jump_evidence"]["validation"],
                             "proven_valid")
            self.assertEqual(result["reference_evidence"]["firmware_diagnostics"]["record_count"], 2)

    def test_corrected_rate_known_circle_and_tilted_quaternion(self) -> None:
        time = np.arange(0.0, 4.0, 0.001)
        speed = 0.8
        radius = 2.0
        expected_rate = speed / radius
        yaw = expected_rate * time
        roll = math.radians(60.0)
        croll, sroll = math.cos(roll / 2.0), math.sin(roll / 2.0)
        cyaw, syaw = np.cos(yaw / 2.0), np.sin(yaw / 2.0)
        # ZYX Euler attitude q = q_z(yaw) * q_x(roll).
        quaternion = np.column_stack([
            cyaw * croll, cyaw * sroll, syaw * sroll, syaw * croll,
        ])
        measured_roll, measured_pitch, measured_yaw = analyzer.quaternion_rpy(quaternion)
        self.assertTrue(np.allclose(measured_roll, roll, atol=1e-10))
        self.assertTrue(np.allclose(measured_pitch, 0.0, atol=1e-10))
        body_wz = np.full(len(time), expected_rate * math.cos(roll))
        grid = analyzer.corrected_rate_grid(
            time, measured_yaw, body_wz, np.full(len(time), speed),
            np.full(len(time), 1.0 / radius), speed,
            np.zeros(len(time)), np.full(len(time), roll),
        )
        interior = slice(5, -5)
        self.assertLess(np.sqrt(np.mean(grid["euler_yaw_rate_minus_geometric_radps"][interior] ** 2)), 1e-6)
        expected_wz_difference = expected_rate * (math.cos(roll) - 1.0)
        self.assertAlmostEqual(
            float(np.mean(grid["body_wz_minus_euler_yaw_rate_radps"][interior])),
            expected_wz_difference, places=6,
        )
        self.assertAlmostEqual(
            float(np.mean(grid["inferred_measured_speed_geometric_rate_radps"])),
            expected_rate, places=8,
        )


if __name__ == "__main__":
    unittest.main()
