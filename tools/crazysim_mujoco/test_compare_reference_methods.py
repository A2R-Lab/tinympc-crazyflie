#!/usr/bin/env python3

import math
import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from compare_reference_methods import (
    anchored_reference,
    load_header,
    nearest_route,
    ordered_waypoint_completion,
    progress_speed_profile,
    waypoint_route_indices,
)


class CompareReferenceMethodsTest(unittest.TestCase):
    def test_generated_benchmark_headers_are_tangent_and_closed(self):
        headers = Path(__file__).resolve().parents[2] / (
            "apps/controller_tinympc_eigen/src/trajectories/50hz"
        )
        for shape in ("figure8", "oval", "circle"):
            defines, reference = load_header(headers / f"traj_{shape}_50hz.h")
            self.assertEqual(defines["TRAJECTORY_TANGENT_HEADING"], "1")
            self.assertEqual(defines["TRAJECTORY_DURATION_S"], "15.0f")
            np.testing.assert_allclose(reference[0, :3], reference[-1, :3], atol=1e-6)

    def test_anchor_rotates_and_translates_position_velocity_and_yaw(self):
        reference = np.zeros((2, 13))
        reference[:, 3] = 1.0
        reference[1, 0] = 1.0
        reference[1, 7] = 2.0
        anchored = anchored_reference(
            reference, np.asarray([3.0, 4.0, 1.5]), math.pi / 2.0
        )
        np.testing.assert_allclose(anchored[1, :3], [3.0, 5.0, 1.5], atol=1e-7)
        np.testing.assert_allclose(anchored[1, 7:10], [0.0, 2.0, 0.0], atol=1e-7)

    def test_waypoint_completion_requires_order(self):
        angle = np.linspace(0.0, 2.0 * math.pi, 201)
        route = np.zeros((len(angle), 13))
        route[:, 0] = np.sin(angle)
        route[:, 1] = 1.0 - np.cos(angle)
        points = route[:, :3].copy()
        self.assertGreater(len(waypoint_route_indices(route)), 10)
        self.assertIsNotNone(ordered_waypoint_completion(points, route, 0.10))
        self.assertIsNone(ordered_waypoint_completion(points[:80], route, 0.10))

    def test_nearest_route_uses_segments(self):
        route = np.zeros((3, 13))
        route[:, 0] = [0.0, 1.0, 2.0]
        points = np.asarray([[0.5, 0.2, 0.0], [1.5, -0.3, 0.0]])
        distance, _ = nearest_route(points, route)
        np.testing.assert_allclose(distance, [0.2, 0.3], atol=1e-7)

    def test_progress_speed_profile_slows_curvature(self):
        reference = np.zeros((5, 13))
        reference[:, :2] = np.asarray([
            [0.0, 0.0], [1.0, 0.0], [2.0, 0.0],
            [2.0, 1.0], [2.0, 2.0],
        ])
        speed = progress_speed_profile(reference)
        self.assertLess(speed[1], speed[0])
        self.assertGreaterEqual(float(np.min(speed)), 0.05)
        self.assertLessEqual(float(np.max(speed)), 0.15)


if __name__ == "__main__":
    unittest.main()
