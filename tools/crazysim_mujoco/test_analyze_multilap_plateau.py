#!/usr/bin/env python3
"""Deterministic synthetic tests for analyze_multilap_plateau.py."""

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

import analyze_multilap_plateau as analyzer


def write_circle_header(path: Path, radius: float = 0.75) -> None:
    rows = []
    for theta in np.linspace(0.0, 2.0 * math.pi, 751):
        values = (
            radius * math.sin(theta), radius * (1.0 - math.cos(theta)), 0.5,
            math.cos(theta / 2.0), 0.0, 0.0, math.sin(theta / 2.0),
            math.cos(theta), math.sin(theta), 0.0, 0.0, 0.0, 1.0,
        )
        rows.append("  {" + ", ".join(f"{value:.9g}f" for value in values) + "},")
    path.write_text("static const float circle[751][13] = {\n" + "\n".join(rows) + "\n};\n")


def write_run(root: Path, seed: int, lap_speeds: list[float], *, contact: bool = False) -> Path:
    run = root / f"seed_{seed}"
    run.mkdir()
    config = {
        "trajectory": "circle", "reference_mode": "progress",
        "progress_speed_mps": 1.0, "progress_laps": len(lap_speeds),
        "progress_entry_acceleration_mps2": 0.25,
        "launch_time_s": 1.0, "random_seed": seed,
        "actuator_lti": True, "rate_cascade": False,
        "stop_on_contact": True, "progress_reference_limits": "uncapped",
        "trajectory_header_sha256": analyzer.sha256(root / "circle.h"),
        "controller_source_sha256": {
            "apps/controller_tinympc_eigen/src/controller_tinympc.cpp": "1" * 64,
        },
        "acceptance_contract": {
            "controller_sha256": "1" * 64, "firmware_binary_sha256": "2" * 64,
        },
    }
    (run / "run_config.json").write_text(json.dumps(config))
    (run / "summary.json").write_text(json.dumps({"launch_time_s": 1.0, "crashed": contact}))
    firmware_lines = [
        "TINYMPC-E: Progress path ready samples=751 laps=4 speed=constant 1.000m/s "
        "entry_acceleration=0.250m/s2",
    ]
    for elapsed in range(0, 30):
        firmware_lines.append(
            f"TINYMPC-E: PROGRESS multilap measured_lap=1/4 "
            f"measured_lap_progress=0.1 target_lap=1/4 target_lap_progress=0.1 "
            f"commanded_speed_mps={min(1.0, .25 * elapsed):.5f} "
            f"entry_acceleration_mps2=.25000 elapsed_s={elapsed:.3f}")
    for lap in range(1, len(lap_speeds) + 1):
        firmware_lines.append(
            f"TINYMPC-E: PROGRESS lap_complete lap={lap}/{len(lap_speeds)} "
            f"measured={lap * 750:.8f} target={lap * 750:.8f} elapsed_s={4 * lap:.3f}")
    firmware_lines.append(
        "TINYMPC-E: PROGRESS counters projection_violation_count=0 "
        "command_violation_count=0 lead_violation_count=0")
    (run / "firmware.log").write_text("\n".join(firmware_lines) + "\n")
    (run / "simulator.log").write_text("[crazysim] max_rpm     : 22000\n")
    fields = [
        "time_s", "x_m", "y_m", "z_m", "qw", "qx", "qy", "qz",
        "vx_mps", "vy_mps", "vz_mps", "wx_radps", "wy_radps", "wz_radps",
        "rpm_1", "rpm_2", "rpm_3", "rpm_4", "contacts", "airborne",
        "rpm_ref_1", "rpm_ref_2", "rpm_ref_3", "rpm_ref_4",
    ]
    samples_per_lap = 200
    active_samples = samples_per_lap * len(lap_speeds) + 1
    total = active_samples + 30  # terminal hover/braking must be excluded
    with (run / "state.csv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for index in range(total):
            active_index = min(index, active_samples - 1)
            lap_index = min(active_index // samples_per_lap, len(lap_speeds) - 1)
            theta = 2.0 * math.pi * active_index / samples_per_lap
            speed = lap_speeds[lap_index] if index < active_samples else 0.0
            row = {
                "time_s": 1.0 + 0.02 * index,
                "x_m": 0.75 * math.sin(theta),
                "y_m": 0.75 * (1.0 - math.cos(theta)), "z_m": 0.5,
                "qw": math.cos(theta / 2.0), "qx": 0.0, "qy": 0.0,
                "qz": math.sin(theta / 2.0),
                "vx_mps": speed * math.cos(theta), "vy_mps": speed * math.sin(theta),
                "vz_mps": 0.0, "wx_radps": 0.0, "wy_radps": 0.0,
                "wz_radps": speed / 0.75,
                "contacts": int(contact and index == active_samples - 20), "airborne": 1,
            }
            for motor in range(1, 5):
                row[f"rpm_{motor}"] = 15000 + 100 * motor
                row[f"rpm_ref_{motor}"] = 15500 + 100 * motor
            writer.writerow(row)
    return run


class MultilapPlateauTest(unittest.TestCase):
    def test_wrap_projection_retains_backward_motion(self) -> None:
        theta = np.asarray([0.0, 0.1, 0.2, 0.15, 2 * math.pi - .02, .03])
        route_theta = np.linspace(0.0, 2 * math.pi, 751)
        route = np.zeros((751, 13))
        route[:, 0] = .75 * np.sin(route_theta)
        route[:, 1] = .75 * (1 - np.cos(route_theta))
        points = np.column_stack([.75 * np.sin(theta), .75 * (1 - np.cos(theta)), np.zeros(len(theta))])
        projection = analyzer.closed_route_projection(points, route)
        delta = np.diff(projection["cumulative_sample"])
        self.assertTrue(np.any(delta < 0.0))
        self.assertLess(np.max(np.abs(delta)), 100.0)

    def test_plateau_requires_two_changes_and_two_slopes(self) -> None:
        def lap(median: float, slope: float) -> dict[str, object]:
            return {"tangential_speed_mps": {"median": median},
                    "within_lap_slope_mps2": slope}
        passed = analyzer.detect_plateau([lap(.78, .2), lap(.80, .02), lap(.82, -.01)])
        self.assertTrue(passed["established"])
        self.assertEqual(passed["first_established_through_lap"], 3)
        failed_change = analyzer.detect_plateau([lap(.70, 0), lap(.80, 0), lap(.82, 0)])
        self.assertFalse(failed_change["established"])
        failed_slope = analyzer.detect_plateau([lap(.78, 0), lap(.80, .06), lap(.82, 0)])
        self.assertFalse(failed_slope["established"])

    def test_analyze_run_uses_active_complete_laps_and_motor_references(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            header = root / "circle.h"; write_circle_header(header)
            run = write_run(root, 1, [.78, .80, .82, .81])
            result, plot = analyzer.analyze_run(run, analyzer.ladder.load_header(header))
            self.assertEqual(result["completed_measured_laps"], 4)
            self.assertTrue(result["plateau"]["established"])
            self.assertEqual(result["plateau"]["first_established_through_lap"], 3)
            self.assertTrue(result["experiment_valid"])
            self.assertEqual(result["command_speed_source"], "firmware_timestamped_diagnostic")
            self.assertEqual(result["identity"]["mode_fields"]["rate_cascade"], False)
            self.assertLess(float(np.max(plot["rpm_ref_fraction"])), .98)
            raw_state = analyzer.ladder.load_state(run / "state.csv")
            self.assertLess(result["active_window"]["end_time_s"], raw_state["time_s"][-1])
            self.assertTrue(result["active_window"]["post_completion_braking_excluded"])
            self.assertEqual(result["progress_invariants"]["firmware_counter_maxima"], {
                "projection_violation_count": 0.0,
                "command_violation_count": 0.0,
                "lead_violation_count": 0.0,
            })
            self.assertTrue(result["progress_invariants"][
                "firmware_measured_laps_match_offline_projection"])

    def test_contact_and_fewer_than_three_laps_is_measured_failure(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            header = root / "circle.h"; write_circle_header(header)
            run = write_run(root, 2, [.7, .75], contact=True)
            result, _ = analyzer.analyze_run(run, analyzer.ladder.load_header(header))
            self.assertLess(result["completed_measured_laps"], 3)
            self.assertTrue(result["safety"]["contact"])
            self.assertFalse(result["experiment_valid"])

    def test_cli_writes_machine_readable_outputs_and_plots(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            header = root / "circle.h"; write_circle_header(header)
            runs = [write_run(root, seed, [.78, .80, .82, .81]) for seed in (1, 2, 3)]
            out = root / "analysis"
            command = [sys.executable, str(Path(analyzer.__file__))]
            for run in runs:
                command += ["--run", str(run)]
            command += ["--circle-header", str(header), "--out", str(out)]
            subprocess.run(command, check=True, capture_output=True, text=True)
            report = json.loads((out / "plateau_report.json").read_text())
            self.assertTrue(report["aggregate"]["all_three_valid_plateaus"])
            self.assertTrue(report["aggregate"]["matrix_identity_consistent"])
            self.assertEqual(len(list((out / "diagnostics").glob("*.png"))), 3)
            self.assertGreater((out / "plateau_summary.png").stat().st_size, 1000)
            with (out / "per_lap.csv").open(newline="") as stream:
                rows = list(csv.DictReader(stream))
            self.assertEqual(len(rows), 12)
            self.assertIn("within_lap_slope_mps2", rows[0])


if __name__ == "__main__":
    unittest.main()
