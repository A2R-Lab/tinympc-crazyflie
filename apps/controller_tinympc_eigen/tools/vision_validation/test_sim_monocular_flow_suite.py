#!/usr/bin/env python3
"""End-to-end regression for rendered monocular sector flow."""

import argparse
import unittest

import sim_monocular_flow_suite as suite


class MonocularSuiteTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        args = argparse.Namespace(width=120, height=72, fov_deg=70,
                                  sectors=9, frames=35, dt=0.1,
                                  max_shift=4, pass_error=0.25, out=None)
        cls.results = {scene.name: suite.run_scene(scene, args)
                       for scene in suite.SCENES}

    def test_all_expected_outcomes(self):
        self.assertTrue(all(result["pass"] for result in self.results.values()))

    def test_front_box_accuracy(self):
        result = self.results["box_1m"]
        self.assertLess(float(result["error_m"]), 0.10)

    def test_textureless_box_does_not_create_false_depth(self):
        self.assertEqual(self.results["low_texture"]["valid_frames"], 0)


if __name__ == "__main__":
    unittest.main()
