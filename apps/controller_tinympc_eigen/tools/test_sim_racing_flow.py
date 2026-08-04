#!/usr/bin/env python3
"""Racing-rate regression tests."""

import unittest

import sim_racing_flow_benchmark as racing


class RacingFlowTest(unittest.TestCase):
    def test_deployed_five_hz_misses_three_mps_approach(self):
        self.assertIsNone(racing.run(3.0, 5, False, False)["first_detection"])

    def test_hybrid_thirty_hz_detects_three_mps_with_margin(self):
        result = racing.run(3.0, 30, True, True, True)
        first = result["first_detection"]
        self.assertIsNotNone(first)
        self.assertGreater(float(first["distance_remaining"]), 1.0)
        self.assertLess(float(first["error"]), 0.40)


if __name__ == "__main__":
    unittest.main()
