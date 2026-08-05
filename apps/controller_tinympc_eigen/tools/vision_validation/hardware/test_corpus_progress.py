#!/usr/bin/env python3

import csv
import json
import tempfile
import unittest
from pathlib import Path

import corpus_progress


class CorpusProgressTest(unittest.TestCase):
    def test_missing_duplicate_outside_and_unreadable_are_distinct(self):
        manifest = {
            "cases": [
                {"id": "f27-a", "configuration_features": 27},
                {"id": "f27-b", "configuration_features": 27},
                {"id": "f36-a", "configuration_features": 36},
            ]
        }
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            sidecars = []
            for name, case_id in (
                ("one.json", "f27-a"),
                ("two.json", "f27-a"),
                ("outside.json", "ad-hoc"),
            ):
                path = root / name
                path.write_text(json.dumps({"case": {"id": case_id}}))
                sidecars.append(path)
            broken = root / "broken.json"
            broken.write_text("{")
            sidecars.append(broken)

            report = corpus_progress.audit(manifest, sidecars)

            self.assertEqual(report["completed_cases"], 0)
            self.assertEqual(report["missing_cases"], 3)
            self.assertIn("f27-a", report["duplicate_case_ids"])
            self.assertIn("ad-hoc", report["outside_manifest"])
            self.assertEqual(len(report["unreadable_sidecars"]), 1)
            self.assertEqual(
                report["by_configuration"]["36"],
                {"expected": 1, "completed": 0, "missing": 1},
            )

    def test_motion_and_manifest_evidence_gate_completion(self):
        manifest = {
            "cases": [
                {
                    "id": "f27-stationary",
                    "configuration_features": 27,
                    "motion": "stationary",
                },
                {
                    "id": "f27-yaw",
                    "configuration_features": 27,
                    "motion": "pure_yaw",
                },
            ]
        }
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            sidecars = []
            for case_id, motion in (
                ("f27-stationary", "stationary"),
                ("f27-yaw", "pure_yaw"),
            ):
                csv_path = root / f"{case_id}.csv"
                with csv_path.open("w", newline="") as stream:
                    timestamp_fields = (
                        corpus_progress.analyze_obstacle_corpus
                        .LOG_TIMESTAMP_FIELDS
                    )
                    writer = csv.DictWriter(stream, fieldnames=[
                        "t_s", "cyl_valid", "body_vx", "body_vy", "yaw_rate",
                        "state_yaw", "rx_ok", "new_samples", "crc_err",
                        "bad_rx", "dup_rx", "invalid_rx", "sequence_gaps",
                        "sequence_resets", "uart_queue_drops", "map_votes",
                    ] + list(timestamp_fields))
                    writer.writeheader()
                    for index in range(10):
                        row = {
                            "t_s": index / 15,
                            "cyl_valid": 0,
                            "body_vx": 0.005,
                            "body_vy": 0,
                            "yaw_rate": 0.01,
                            "state_yaw": index * 0.01,
                            "rx_ok": index,
                            "new_samples": index,
                            "crc_err": 0,
                            "bad_rx": 0,
                            "dup_rx": 0,
                            "invalid_rx": 0,
                            "sequence_gaps": 0,
                            "sequence_resets": 0,
                            "uart_queue_drops": 0,
                            "map_votes": 0,
                        }
                        row.update({
                            field: index * 100 + offset
                            for offset, field in enumerate(timestamp_fields)
                        })
                        writer.writerow(row)
                sidecar = root / f"{case_id}.csv.json"
                sidecar.write_text(json.dumps({
                    "csv": str(csv_path),
                    "case": {
                        "id": case_id,
                        "configuration_features": 27,
                        "motion": motion,
                        "label": "negative",
                        "distance_m": None,
                        "orientation_deg": None,
                        "lateral_offset_m": None,
                        "texture": "low",
                        "lighting": "nominal",
                    },
                }))
                sidecars.append(sidecar)

            report = corpus_progress.audit(manifest, sidecars)

            self.assertEqual(report["completed"], ["f27-stationary"])
            self.assertIn("f27-yaw", report["invalid_motion"])
            self.assertEqual(report["completed_cases"], 1)
            self.assertEqual(report["missing_cases"], 1)


if __name__ == "__main__":
    unittest.main()
