#!/usr/bin/env python3
import importlib.util
from pathlib import Path
import unittest

import numpy as np


GENERATOR = (Path(__file__).resolve().parents[1] / "tools" /
             "firmware_codegen" / "generate_level_actuator_lti.py")
SPEC = importlib.util.spec_from_file_location("generate_level_actuator_lti", GENERATOR)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(MODULE)


class LevelActuatorCostModesTest(unittest.TestCase):
    def test_exact_mode_set_and_q_changes(self):
        modes = MODULE._cost_modes()
        self.assertEqual(set(modes), {
            "baseline", "yaw_angle_4x", "yaw_rate_4x", "yaw_diff_r_quarter"
        })
        baseline_q, baseline_r = modes["baseline"]
        angle_q, angle_r = modes["yaw_angle_4x"]
        rate_q, rate_r = modes["yaw_rate_4x"]
        diff_q, diff_r = modes["yaw_diff_r_quarter"]
        np.testing.assert_array_equal(angle_q, baseline_q + np.eye(1, 16, 5)[0] * 1200.0)
        np.testing.assert_array_equal(rate_q, baseline_q + np.eye(1, 16, 11)[0] * 120.0)
        np.testing.assert_array_equal(diff_q, baseline_q)
        np.testing.assert_array_equal(angle_r, baseline_r)
        np.testing.assert_array_equal(rate_r, baseline_r)

    def test_yaw_differential_mode_is_the_only_reduced_r_eigenmode(self):
        r = MODULE._cost_modes()["yaw_diff_r_quarter"][1]
        yaw = MODULE.YAW_MOTOR_MODE
        self.assertAlmostEqual(float(yaw @ r @ yaw), MODULE.BASE_R_EIGENVALUE / 4.0)
        for other in (
            np.asarray([1.0, 1.0, 1.0, 1.0]) / 2.0,
            np.asarray([-1.0, -1.0, 1.0, 1.0]) / 2.0,
            np.asarray([-1.0, 1.0, 1.0, -1.0]) / 2.0,
        ):
            self.assertAlmostEqual(float(other @ r @ other), MODULE.BASE_R_EIGENVALUE)
            self.assertAlmostEqual(float(other @ r @ yaw), 0.0, places=10)


if __name__ == "__main__":
    unittest.main()
