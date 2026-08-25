#!/usr/bin/env python3
"""Pure geometry/compaction coverage for the NewBee joint-rollout contract."""

import importlib.util
import sys
import unittest
from pathlib import Path

import cv2
import numpy as np


PATH = Path(__file__).with_name("export_joint_gate_obstacle_rollouts.py")
SPEC = importlib.util.spec_from_file_location("joint_export", PATH)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)
MODULE.np = np
MODULE.cv2 = cv2


class NewBeeGeometryTest(unittest.TestCase):
    def test_v3_easy_transition_geometry_is_exact_and_self_consistent(self):
        course = MODULE.EASY_TRANSITION_COURSE
        self.assertEqual(course["gate_center_m"], [4.00, 0.30, 1.50])
        self.assertEqual(course["clear_opening_m"], [0.45, 0.45])
        self.assertEqual(course["post_gate_box_center_m"], [6.20, 0.00, 1.10])
        self.assertEqual(course["post_gate_box_full_dimensions_m"], [0.30, 1.00, 2.20])
        leading_face = course["post_gate_box_center_m"][0] - course["post_gate_box_full_dimensions_m"][0] / 2.0
        self.assertAlmostEqual(leading_face - course["gate_center_m"][0], 2.05)
        self.assertEqual(course["gate_to_box_leading_face_gap_m"], 2.05)

    def test_v3_small_gate_limit_matches_calibrated_farthest_camera_geometry(self):
        self.assertAlmostEqual(MODULE.CALIBRATED_RAIL_SPAN_AT_4M_PX, 12.413, places=3)
        self.assertAlmostEqual(MODULE.CALIBRATED_V3_FARTHEST_RAIL_SPAN_PX, 15.516, places=3)
        self.assertGreaterEqual(MODULE.SMALL_GATE_MAX_SPAN_PX,
                                MODULE.CALIBRATED_V3_FARTHEST_RAIL_SPAN_PX)
        self.assertLess(MODULE.SMALL_GATE_MAX_SPAN_PX,
                        MODULE.CALIBRATED_V3_FARTHEST_RAIL_SPAN_PX + 1.0)

    def test_high_rate_physical_contact_acceptance_rejects_box_contact(self):
        course = MODULE.EASY_TRANSITION_COURSE
        track = {
            "gates": [MODULE.newbee_gate_geometry(np.asarray(course["gate_center_m"]), 0.0)],
            "obstacles": [{"center_m": course["post_gate_box_center_m"],
                           "size_m": course["post_gate_box_full_dimensions_m"], "yaw_rad": 0.0}],
        }
        safe = {"_physical_rollout_state_f32": np.asarray([[0.8, 0.3, 1.5], [3.8, 0.3, 1.5]])}
        accepted = MODULE.validate_physical_execution(safe, track)
        self.assertEqual(accepted["sample_count"], 2)
        collided = {"_physical_rollout_state_f32": np.asarray([[6.2, 0.0, 1.1]])}
        with self.assertRaisesRegex(RuntimeError, "high-rate contact"):
            MODULE.validate_physical_execution(collided, track)

    def test_exact_clearance_and_rail_center_contract(self):
        gate = MODULE.newbee_gate_geometry(np.asarray([2.0, 0.0, 0.8]), 0.0)
        self.assertEqual(gate["opening_m"], [0.45, 0.45])
        self.assertEqual(gate["geometry_contract"]["rail_center_span_m"], [0.555, 0.555])
        corners = np.asarray(gate["rail_center_corners_world_m_tl_tr_br_bl"])
        self.assertAlmostEqual(float(np.ptp(corners[:, 1])), 0.555)
        self.assertAlmostEqual(float(np.ptp(corners[:, 2])), 0.555)
        self.assertEqual([Path(path).name for path in gate["visual_asset"]["mesh_parts"]], [
            "newbeedrone_gate_top.obj", "newbeedrone_gate_bottom.obj",
            "newbeedrone_gate_left.obj", "newbeedrone_gate_right.obj",
        ])
        self.assertEqual(Path(gate["visual_asset"]["texture"]).name,
                         "newbeedrone_gate_front_rgba_v1.png")
        for path in (*gate["visual_asset"]["mesh_parts"], gate["visual_asset"]["texture"]):
            self.assertTrue(Path(path).is_file())
        parts = {part["object_id"].rsplit("_", 1)[-1]: part for part in gate["parts"]}
        np.testing.assert_allclose(parts["left"]["size_m"], [0.05, 0.108, 0.45])
        np.testing.assert_allclose(parts["top"]["size_m"], [0.05, 0.666, 0.108])
        # Pass labels use the shrunken *clear* opening, not visual rail centres.
        before, after = np.asarray([1.9, 0.0, 0.8]), np.asarray([2.1, 0.0, 0.8])
        label = MODULE.gate_label(gate, before, after, False)
        self.assertTrue(label["ordered_shrunken_opening_pass"])
        self.assertEqual(label["opening_m"], [0.45, 0.45])

    def test_projection_is_rail_centres_and_small_span_is_counted(self):
        gate = MODULE.newbee_gate_geometry(np.asarray([5.0, 0.0, 0.8]), 0.0)
        calibration = {"resolution": [160, 160], "camera_matrix": [[90, 0, 80], [0, 90, 80], [0, 0, 1]],
                       "distortion_coefficients": [0, 0, 0, 0, 0]}
        state = {"position_m": [0, 0, .8], "right_world": [0, -1, 0],
                 "up_world": [0, 0, 1], "forward_world": [1, 0, 0]}
        corners, visible = MODULE.normalized_gate_corners(gate, state, calibration)
        self.assertTrue(visible)
        self.assertLessEqual(MODULE.gate_span_px(corners, visible, calibration), 12.0)
        self.assertGreater(MODULE.gate_span_px(corners, visible, calibration), 9.0)


if __name__ == "__main__":
    unittest.main()
