#!/usr/bin/env python3
"""Crazyflow closed-loop trajectory and safety regressions."""

import importlib.util
import unittest

crazyflow_available = importlib.util.find_spec("crazyflow") is not None

if crazyflow_available:
    import sim_crazyflow_racing_pipeline as pipeline


@unittest.skipIf(not crazyflow_available, "run with .venv-crazyflow/bin/python")
class CrazyflowPipelineTest(unittest.TestCase):
    def test_speed_ramp_is_continuous_and_reaches_requested_speed(self):
        before = pipeline.speed_ramp(0.75 - 1e-7, 3.0)
        after = pipeline.speed_ramp(0.75 + 1e-7, 3.0)
        self.assertAlmostEqual(before[0], after[0], places=5)
        self.assertAlmostEqual(before[1], after[1], places=5)
        self.assertEqual(after[1], 3.0)

    def test_three_mps_hybrid_pipeline_safely_clears_box(self):
        result = pipeline.run(3.0, True, 1000, 0.02, 2.0)
        self.assertIsNotNone(result["first_detection"])
        self.assertFalse(result["collision"])
        self.assertFalse(result["ground_contact"])
        self.assertTrue(result["safe_pass"])
        self.assertGreater(float(result["minimum_clearance_m"]), 0.20)
        self.assertGreater(float(result["minimum_altitude_m"]), 0.40)


if __name__ == "__main__":
    unittest.main()
