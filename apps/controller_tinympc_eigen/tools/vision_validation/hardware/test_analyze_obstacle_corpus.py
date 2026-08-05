#!/usr/bin/env python3

import csv
import json
import tempfile
import unittest
from pathlib import Path

import analyze_obstacle_corpus as analyzer


class CorpusAnalyzerTest(unittest.TestCase):
    def test_positive_setup_must_match_measured_manifest_pose(self):
        case = {
            "label": "positive",
            "distance_m": 0.5,
            "lateral_offset_m": 0.3,
            "orientation_deg": -45,
            "truth_start_range_m": (0.5 ** 2 + 0.3 ** 2) ** 0.5,
            "truth_start_bearing_deg": 30.9637565,
            "truth_obstacle_width_m": 0.5,
            "truth_orientation_deg": -45,
        }
        rows = [{"state_x": "1", "state_y": "2", "state_yaw": "20"}]
        truth_x, truth_y, _source = analyzer.resolve_truth_world(case, rows)
        evidence = analyzer.setup_evidence(case, rows, truth_x, truth_y)
        self.assertTrue(evidence["label_supported"])
        self.assertAlmostEqual(evidence["measured_forward_m"], 0.5, places=6)
        self.assertAlmostEqual(evidence["measured_lateral_m"], 0.3, places=6)

        case["truth_orientation_deg"] = -30
        self.assertFalse(
            analyzer.setup_evidence(case, rows, truth_x, truth_y)[
                "label_supported"
            ]
        )

    def test_bias_audit_requires_full_symmetric_corpus(self):
        positives = []
        for orientation in (-45, -20, 0, 20, 45):
            for offset in (-0.3, 0.0, 0.3):
                for _repeat in range(12):
                    positives.append({
                        "orientation_deg": orientation,
                        "lateral_offset_m": offset,
                        "detected": True,
                        "world_position_error_m": 0.1,
                        "signed_bearing_error_deg": 1.0,
                        "signed_body_lateral_error_m": 0.02,
                    })
        audit = analyzer.bias_audit(positives)
        self.assertTrue(audit["evaluable"])
        self.assertTrue(audit["passed"])

        for item in positives:
            if item["orientation_deg"] == -45:
                item["detected"] = False
        self.assertFalse(analyzer.bias_audit(positives)["passed"])
        self.assertIsNone(analyzer.bias_audit(positives[:-1])["passed"])

    def test_initial_polar_truth_is_anchored_to_first_logged_pose(self):
        case = {
            "truth_start_range_m": 2.0,
            "truth_start_bearing_deg": 30.0,
        }
        rows = [
            {"state_x": "", "state_y": "", "state_yaw": ""},
            {"state_x": "1.0", "state_y": "2.0", "state_yaw": "60.0"},
        ]
        x, y, source = analyzer.resolve_truth_world(case, rows)
        self.assertAlmostEqual(x, 1.0, places=6)
        self.assertAlmostEqual(y, 4.0, places=6)
        self.assertEqual(source, "measured_start_polar")

    def test_synchronization_is_wrap_safe_and_flow_period_gated(self):
        rows = []
        modulus = 1 << 32
        for base in (modulus - 10, 100):
            row = {
                field: str((base + index * 2) % modulus)
                for index, field in enumerate(analyzer.LOG_TIMESTAMP_FIELDS)
            }
            rows.append(row)
        evidence = analyzer.synchronization_evidence(rows)
        self.assertEqual(evidence["complete_rows"], 2)
        self.assertEqual(evidence["skew_max_ms"], 30)
        self.assertTrue(evidence["within_one_flow_period"])

        rows[0][analyzer.LOG_TIMESTAMP_FIELDS[-1]] = str(1000)
        self.assertFalse(
            analyzer.synchronization_evidence(rows)["within_one_flow_period"]
        )

    def test_motion_evidence_distinguishes_stationary_translation_and_yaw(self):
        def rows(speed, yaw):
            return [
                {
                    "body_vx": str(speed),
                    "body_vy": "0",
                    "yaw_rate": str(yaw),
                    "state_yaw": str(index * yaw),
                }
                for index in range(20)
            ]

        self.assertTrue(
            analyzer.motion_evidence(rows(0.005, 0.01), "stationary")[
                "label_supported"
            ]
        )
        self.assertTrue(
            analyzer.motion_evidence(rows(0.12, 0.10), "translation")[
                "label_supported"
            ]
        )
        self.assertTrue(
            analyzer.motion_evidence(rows(0.12, 0.80), "translation_yaw")[
                "label_supported"
            ]
        )
        self.assertTrue(
            analyzer.motion_evidence(rows(0.01, 0.80), "pure_yaw")[
                "label_supported"
            ]
        )
        self.assertFalse(
            analyzer.motion_evidence(rows(0.005, 0.01), "pure_yaw")[
                "label_supported"
            ]
        )
        self.assertFalse(
            analyzer.motion_evidence(rows(1.6, 0.10), "translation")[
                "label_supported"
            ]
        )

    def test_first_detection_distance_dropout_and_new_sample_invariant(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            csv_path = root / "case.csv"
            fields = [
                "t_s", "cyl_valid", "cyl_wx", "cyl_wy", "state_x", "state_y",
                "rx_ok", "new_samples", "crc_err", "bad_rx", "dup_rx",
                "invalid_rx", "sequence_gaps", "sequence_resets",
                "uart_queue_drops", "map_votes",
            ]
            rows = [
                [0.0, 0, 0, 0, 0.0, 0.0, 10, 10, 0, 0, 0, 0, 0, 0, 0, 0],
                [0.1, 1, 2, 0, 0.5, 0.0, 11, 11, 0, 0, 0, 0, 0, 0, 0, 1],
                [0.2, 0, 0, 0, 0.6, 0.0, 12, 12, 0, 0, 0, 0, 0, 0, 0, 1],
                [0.3, 0, 0, 0, 0.7, 0.0, 12, 12, 0, 0, 0, 0, 0, 0, 0, 2],
                [0.4, 1, 2, 0, 0.8, 0.0, 13, 13, 0, 0, 0, 0, 0, 0, 0, 2],
            ]
            with csv_path.open("w", newline="") as stream:
                writer = csv.writer(stream)
                writer.writerow(fields)
                writer.writerows(rows)
            sidecar = root / "case.csv.json"
            sidecar.write_text(json.dumps({
                "csv": str(csv_path),
                "case": {
                    "id": "f27-test",
                    "configuration_features": 27,
                    "label": "positive",
                    "distance_m": 2.0,
                    "orientation_deg": 0,
                    "lateral_offset_m": 0,
                    "texture": "high",
                    "lighting": "nominal",
                    "motion": "translation",
                    "truth_world_x_m": 2.0,
                    "truth_world_y_m": 0.0,
                },
                "run": {"snapshot_hz": 10},
            }))

            result = analyzer.summarize_run(sidecar)

            self.assertTrue(result["detected"])
            self.assertAlmostEqual(result["first_detection_time_s"], 0.1)
            self.assertAlmostEqual(result["first_detection_distance_m"], 1.5)
            self.assertAlmostEqual(result["maximum_dropout_s"], 0.2)
            self.assertEqual(
                result["transport"]["map_vote_without_new_sample_violations"], 1
            )
            self.assertEqual(result["transport"]["received_samples"], 3)
            self.assertEqual(result["transport"]["new_samples"], 3)

    def test_flow_delivery_detects_silent_stream_stall(self):
        healthy = [
            {"t_s": str(index / 30), "new_samples": str(index // 2)}
            for index in range(901)
        ]
        evidence = analyzer.flow_delivery_evidence(healthy)
        self.assertTrue(evidence["delivery_stable"])
        self.assertAlmostEqual(evidence["delivery_rate_hz"], 15.0)

        stalled = [
            {
                "t_s": str(index / 30),
                "new_samples": str(min(index // 2, 49)),
            }
            for index in range(901)
        ]
        evidence = analyzer.flow_delivery_evidence(stalled)
        self.assertFalse(evidence["delivery_stable"])
        self.assertGreater(evidence["maximum_no_new_sample_interval_s"], 20)


if __name__ == "__main__":
    unittest.main()
