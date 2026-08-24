#!/usr/bin/env python3

import copy
import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).parent))
import analyze_progress_acceptance as acceptance


class AcceptanceTest(unittest.TestCase):
    HASH = "a" * 64

    def fixtures(self):
        result = {
            "requested_speed_mps": 1.5, "random_seed": 2, "run_directory": "/tmp/run",
            "launch": {"calibrated": True, "timing_error_s": 0.1},
            "progress": {"firmware_complete_750_of_750": True, "bounded_diagnostics": {
                "present": True, "parseable": True, "valid": True, "record_count": 20,
                "projection_violation_count_max": 0, "command_violation_count_max": 0,
                "lead_violation_count_max": 0}},
            "safety": {"contact_or_crash": False,
                       "contact_or_crash_active_route": False,
                       "contact_or_crash_post_completion": False},
            "tracking": {"cross_track_m": {"rmse": .1, "p95_abs": .2},
                         "altitude_error_m": {"rmse": .05}},
        }
        route = np.column_stack((np.arange(11), np.zeros(11), np.ones(11)))
        position = route[:6].copy()
        plot = {"route": route, "progress": np.arange(6) * 2.,
                "tangent_velocity": np.asarray([0., 1., 2., 3., 4., 5.]),
                "position": position, "projected": position.copy()}
        return result, plot

    def matrix_results(self):
        results = []
        for speed in acceptance.EXPECTED_SPEEDS_MPS:
            for seed in acceptance.EXPECTED_SEEDS:
                config = {
                    "runner_sha256": self.HASH,
                    "trajectory_header_sha256": "b" * 64,
                    "trajectory": "circle",
                    "reference_mode": "progress",
                    "progress_speed_mps": speed,
                    "progress_reference_limits": "uncapped",
                    "progress_sample_limit": 0,
                    "level_cost_mode": "baseline",
                    "duration_s": 25.0,
                    "launch_time_s": 2.0,
                    "spawn_z_m": 1.5,
                    "model": "cf21B_500_combined",
                    "mass": 0.045,
                    "pwm_thrust_full_n": 0.312852,
                    "launch_prespin": False,
                    "random_seed": seed,
                    "inertia_scale": 1.0,
                    "motor_tau_scale": 1.0,
                    "thrust_scale": 1.0,
                    "realtime_factor": 1.0,
                    "firmware_time_factor": 0.8,
                    "actuator_lti": True,
                    "stop_on_contact": True,
                    "camera_capture_enabled": True,
                    "extra_simulator_arguments": ["--flowdeck"],
                    "acceptance_contract": {
                        field: "0123456789abcdef"[index + 2] * 64
                        for index, field in enumerate(
                            acceptance.CONTRACT_HASH_FIELDS)
                    },
                }
                config["acceptance_contract"].update({
                    "circle_radius_m": 0.75,
                    "flowdeck_enabled": True,
                    "passive_camera_capture_enabled": True,
                    "camera_inference_enabled": False,
                    "vision_control_enabled": False,
                    "resolved_plant_profile": {
                        "name": "cf21bl_battery_guards_ai_flow",
                        "mass_kg": 0.045,
                        "diagonal_inertia_kg_m2": [2.3951e-5, 2.3951e-5, 3.2347e-5],
                        "drag_matrix_n_s_per_m": [
                            [-0.021, 0.0, 0.0],
                            [0.0, -0.021, 0.0],
                            [0.0, 0.0, -0.023],
                        ],
                    },
                })
                config["acceptance_contract"][
                    acceptance.FIRMWARE_BINARY_HASH_FIELD] = {
                        1.0: "1" * 64,
                        1.5: "2" * 64,
                        2.0: "3" * 64,
                    }[speed]
                results.append({
                    "requested_speed_mps": speed,
                    "random_seed": seed,
                    "run_directory": f"/tmp/speed_{speed}_seed_{seed}",
                    "run_config": config,
                })
        return results

    def test_accepts_complete_safe_bounded_run_and_reports_median(self):
        result, plot = self.fixtures()
        row = acceptance.assess_run(result, plot, acceptance.DEFAULTS)
        self.assertTrue(row["accepted"])
        self.assertEqual(row["median_post_entry_tangential_speed_mps"], 3.0)
        self.assertEqual(row["progress_invariant_counts"]["lead_violation_count_max"], 0)

    def test_rejects_contact_flyaway_and_invariant_violation(self):
        result, plot = self.fixtures()
        result["safety"]["contact_or_crash"] = True
        result["safety"]["contact_or_crash_active_route"] = True
        result["progress"]["bounded_diagnostics"]["lead_violation_count_max"] = 1
        result["progress"]["bounded_diagnostics"]["valid"] = False
        plot["position"][2, 1] = 3.0
        row = acceptance.assess_run(result, plot, acceptance.DEFAULTS)
        self.assertFalse(row["accepted"])
        self.assertTrue(row["contact_or_crash"])
        self.assertTrue(row["flyaway"])
        self.assertFalse(row["gates"]["progress_invariants_valid"])

    def test_rejects_unavailable_or_slow_post_entry_speed_explicitly(self):
        result, plot = self.fixtures()
        plot["progress"][:] = 0.0
        row = acceptance.assess_run(result, plot, acceptance.DEFAULTS)
        self.assertFalse(row["accepted"])
        self.assertIsNone(row["median_post_entry_tangential_speed_mps"])
        self.assertFalse(row["gates"]["post_entry_speed_available"])
        self.assertFalse(row["gates"]["post_entry_speed_at_least_fraction_of_request"])

        result, plot = self.fixtures()
        plot["tangent_velocity"][:] = 1.0
        row = acceptance.assess_run(result, plot, acceptance.DEFAULTS)
        self.assertFalse(row["accepted"])
        self.assertAlmostEqual(row["minimum_post_entry_tangential_speed_mps"], 1.275)
        self.assertFalse(row["gates"]["post_entry_speed_at_least_fraction_of_request"])

    def test_accepts_exact_homogeneous_nine_trial_contract(self):
        contract = acceptance.validate_matrix_contract(self.matrix_results())
        self.assertTrue(contract["valid"], contract["errors"])
        self.assertEqual(len(contract["observed_trials"]), 9)
        self.assertEqual(len(contract["firmware_identity_by_speed"]), 3)
        self.assertEqual(
            len({item["firmware_binary_sha256"]
                 for item in contract["firmware_identity_by_speed"]}), 3)
        self.assertEqual(contract["errors"], [])

    def test_rejects_firmware_binary_drift_between_seeds_at_one_speed(self):
        results = self.matrix_results()
        results[1]["run_config"]["acceptance_contract"][
            acceptance.FIRMWARE_BINARY_HASH_FIELD] = "f" * 64
        contract = acceptance.validate_matrix_contract(results)
        self.assertFalse(contract["valid"])
        self.assertTrue(any(
            error["code"] == "firmware_binary_seed_drift"
            for error in contract["errors"]))
        speed_one = next(
            item for item in contract["firmware_identity_by_speed"]
            if item["requested_speed_mps"] == 1.0)
        self.assertIsNone(speed_one["firmware_binary_sha256"])

    def test_rejects_missing_duplicate_and_extra_trials(self):
        missing = self.matrix_results()[:-1]
        contract = acceptance.validate_matrix_contract(missing)
        self.assertFalse(contract["valid"])
        self.assertIn("trial_count", {error["code"] for error in contract["errors"]})
        self.assertIn("missing_trial", {error["code"] for error in contract["errors"]})

        duplicate = self.matrix_results()
        duplicate[-1] = copy.deepcopy(duplicate[0])
        duplicate[-1]["run_directory"] = "/tmp/duplicate"
        contract = acceptance.validate_matrix_contract(duplicate)
        codes = {error["code"] for error in contract["errors"]}
        self.assertIn("duplicate_trial", codes)
        self.assertIn("missing_trial", codes)

        extra = self.matrix_results()
        unexpected = copy.deepcopy(extra[0])
        unexpected["requested_speed_mps"] = 3.0
        unexpected["run_config"]["progress_speed_mps"] = 3.0
        unexpected["run_directory"] = "/tmp/extra"
        extra.append(unexpected)
        contract = acceptance.validate_matrix_contract(extra)
        codes = {error["code"] for error in contract["errors"]}
        self.assertIn("trial_count", codes)
        self.assertIn("unexpected_speed", codes)
        self.assertIn("extra_trial", codes)

    def test_rejects_missing_or_malformed_contract_metadata(self):
        results = self.matrix_results()
        del results[0]["run_config"]["acceptance_contract"]["frenet_error_sha256"]
        contract = acceptance.validate_matrix_contract(results)
        self.assertFalse(contract["valid"])
        self.assertTrue(any(
            error["code"] == "invalid_contract_hash"
            and error.get("field") == "frenet_error_sha256"
            for error in contract["errors"]))

        results = self.matrix_results()
        results[0]["run_config"]["acceptance_contract"]["controller_sha256"] = "not-a-hash"
        contract = acceptance.validate_matrix_contract(results)
        self.assertTrue(any(error["code"] == "invalid_contract_hash"
                            for error in contract["errors"]))

    def test_rejects_heterogeneous_hash_plant_or_timing_identity(self):
        for field, value in (
            ("bank_header_sha256", "f" * 64),
            ("resolved_plant_profile", {
                "name": "different", "mass_kg": 0.046,
                "diagonal_inertia_kg_m2": [1e-5, 1e-5, 1e-5],
                "drag_matrix_n_s_per_m": [[0.0] * 3 for _ in range(3)],
            }),
        ):
            with self.subTest(field=field):
                results = self.matrix_results()
                results[-1]["run_config"]["acceptance_contract"][field] = value
                contract = acceptance.validate_matrix_contract(results)
                self.assertTrue(any(
                    error["code"] == "heterogeneous_matrix_identity"
                    for error in contract["errors"]))

        results = self.matrix_results()
        results[-1]["run_config"]["firmware_time_factor"] = 0.7
        contract = acceptance.validate_matrix_contract(results)
        self.assertTrue(any(
            error["code"] == "heterogeneous_matrix_identity"
            and error.get("field") == "firmware_time_factor"
            for error in contract["errors"]))

    def test_rejects_wrong_mode_geometry_flow_or_passive_camera_contract(self):
        mutations = (
            ("top", "progress_reference_limits", "default"),
            ("top", "camera_capture_enabled", False),
            ("contract", "circle_radius_m", 0.8),
            ("contract", "flowdeck_enabled", False),
            ("contract", "passive_camera_capture_enabled", False),
            ("contract", "camera_inference_enabled", True),
            ("contract", "vision_control_enabled", True),
        )
        for location, field, value in mutations:
            with self.subTest(field=field):
                results = self.matrix_results()
                target = (results[0]["run_config"] if location == "top" else
                          results[0]["run_config"]["acceptance_contract"])
                target[field] = value
                contract = acceptance.validate_matrix_contract(results)
                self.assertFalse(contract["valid"])
                self.assertTrue(any(error.get("field") == field
                                    for error in contract["errors"]))

    def test_rejects_invalid_resolved_plant_shape(self):
        results = self.matrix_results()
        profile = results[0]["run_config"]["acceptance_contract"][
            "resolved_plant_profile"]
        profile["diagonal_inertia_kg_m2"] = [1.0, 2.0]
        contract = acceptance.validate_matrix_contract(results)
        self.assertTrue(any(
            error["code"] == "invalid_resolved_plant_profile"
            for error in contract["errors"]))


if __name__ == "__main__":
    unittest.main()
