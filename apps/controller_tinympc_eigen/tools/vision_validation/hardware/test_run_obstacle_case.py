#!/usr/bin/env python3

import types
import unittest

import run_obstacle_case as runner


class ObstacleCaseRunnerTest(unittest.TestCase):
    def args(self, **updates):
        values = {
            "uri": "radio://test",
            "logs_dir": runner.Path("/tmp/logs"),
            "duration": 30.0,
            "start_delay": 5.0,
            "period_ms": 30,
            "sample_hz": 30.0,
            "truth_world_x": None,
            "truth_world_y": None,
            "truth_start_range_m": None,
            "truth_start_bearing_deg": None,
            "truth_obstacle_width_m": None,
            "truth_orientation_deg": None,
            "notes": "",
        }
        values.update(updates)
        return types.SimpleNamespace(**values)

    def test_positive_requires_measured_world_truth(self):
        case = {
            "id": "f27-p-001", "configuration_features": 27,
            "label": "positive", "distance_m": 0.5,
            "orientation_deg": -45, "lateral_offset_m": -0.3,
            "obstacle_shape": "planar", "obstacle_width": "narrow",
            "depth_configuration": "single", "texture": "high",
            "lighting": "nominal", "motion": "translation",
        }
        with self.assertRaises(SystemExit):
            runner.command_for(case, self.args())
        output, command = runner.command_for(
            case, self.args(
                truth_world_x=0.5,
                truth_world_y=-0.3,
                truth_obstacle_width_m=0.15,
                truth_orientation_deg=-45,
            )
        )
        self.assertEqual(output.name, "f27-p-001.csv")
        self.assertIn("--depth-configuration", command)
        self.assertIn("--truth-world-x", command)
        self.assertIn("--start-delay", command)
        self.assertEqual(command[command.index("--period-ms") + 1], "30")
        self.assertEqual(command[command.index("--sample-hz") + 1], "30.0")

        _output, relative_command = runner.command_for(
            case,
            self.args(
                truth_start_range_m=0.58,
                truth_start_bearing_deg=31.0,
                truth_obstacle_width_m=0.15,
                truth_orientation_deg=-45,
            ),
        )
        self.assertIn("--truth-start-range-m", relative_command)
        self.assertIn("--truth-start-bearing-deg", relative_command)

        with self.assertRaises(SystemExit):
            runner.command_for(
                case,
                self.args(
                    truth_world_x=0.5,
                    truth_world_y=-0.3,
                    truth_start_range_m=0.58,
                    truth_start_bearing_deg=31.0,
                    truth_obstacle_width_m=0.15,
                    truth_orientation_deg=-45,
                ),
            )

    def test_negative_does_not_require_truth(self):
        case = {
            "id": "f27-n-001", "configuration_features": 27,
            "label": "negative", "distance_m": None,
            "orientation_deg": None, "lateral_offset_m": None,
            "obstacle_shape": "none", "obstacle_width": "none",
            "depth_configuration": "stationary_scene", "texture": "high",
            "lighting": "nominal", "motion": "stationary",
        }
        output, _command = runner.command_for(case, self.args())
        self.assertEqual(output.name, "f27-n-001.csv")


if __name__ == "__main__":
    unittest.main()
