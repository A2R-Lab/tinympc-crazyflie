#!/usr/bin/env python3
"""Host tests for the four-direction binary waypoint planner."""

import ctypes
import subprocess
import tempfile
import unittest
from pathlib import Path

APP = Path(__file__).resolve().parents[2]
SRC = APP / "src"
F4 = ctypes.c_float * 4
F3 = ctypes.c_float * 3


class Config(ctypes.Structure):
    _fields_ = [(name, ctypes.c_float) for name in (
        "confidence_min", "drone_radius_m", "tracking_margin_m",
        "latency_s", "perception_margin_m", "confidence_margin_gain_m",
        "conservative_default_offset_m", "maximum_range_m",
        "trigger_distance_m", "distance_weight",
        "goal_weight", "hysteresis_weight", "dynamic_weight",
        "direction_safe_min_m",
    )]


class Result(ctypes.Structure):
    _fields_ = [
        ("valid", ctypes.c_bool),
        ("stop", ctypes.c_bool),
        ("reliable_mask", ctypes.c_uint8),
        ("chosen_direction", ctypes.c_int8),
        ("body_normal", (ctypes.c_float * 3) * 4),
        ("effective_offset_m", F4),
        ("margin_m", F4),
        ("chosen_score", ctypes.c_float),
        ("avoidance_pressure", ctypes.c_float),
    ]


class DangerAverage(ctypes.Structure):
    _fields_ = [
        ("samples", ctypes.c_float * 32),
        ("sum", ctypes.c_float),
        ("next", ctypes.c_uint8),
        ("count", ctypes.c_uint8),
        ("window", ctypes.c_uint8),
    ]


class ClearanceAverage(ctypes.Structure):
    _fields_ = [
        ("samples", (ctypes.c_float * 32) * 4),
        ("sum", ctypes.c_float * 4),
        ("next", ctypes.c_uint8),
        ("count", ctypes.c_uint8),
        ("window", ctypes.c_uint8),
    ]


def build_library(output: Path):
    subprocess.run([
        "gcc", "-std=c11", "-shared", "-fPIC", "-Wall", "-Wextra",
        "-Werror", "-I", str(SRC), str(SRC / "sequential_obstacle_control.c"),
        "-lm", "-o", str(output),
    ], check=True)
    lib = ctypes.CDLL(str(output))
    lib.sequentialObstacleControlPlan.argtypes = [
        ctypes.POINTER(ctypes.c_float), ctypes.POINTER(ctypes.c_float),
        ctypes.POINTER(ctypes.c_float), ctypes.POINTER(ctypes.c_float),
        ctypes.c_int, ctypes.POINTER(Config), ctypes.POINTER(Result),
    ]
    lib.sequentialDangerAverageReset.argtypes = [ctypes.POINTER(DangerAverage)]
    lib.sequentialDangerAverageUpdate.argtypes = [
        ctypes.POINTER(DangerAverage), ctypes.c_uint8, ctypes.c_uint8,
    ]
    lib.sequentialDangerAverageUpdate.restype = ctypes.c_float
    lib.sequentialClearanceAverageReset.argtypes = [ctypes.POINTER(ClearanceAverage)]
    lib.sequentialClearanceAverageUpdate.argtypes = [
        ctypes.POINTER(ClearanceAverage), ctypes.POINTER(ctypes.c_float),
        ctypes.c_uint8, ctypes.POINTER(ctypes.c_float),
    ]
    lib.sequentialSelectEvasionSide.argtypes = [
        ctypes.POINTER(ctypes.c_float), ctypes.c_float, ctypes.c_int,
        ctypes.POINTER(ctypes.c_float), ctypes.POINTER(ctypes.c_float),
    ]
    lib.sequentialSelectEvasionSide.restype = ctypes.c_int8
    lib.sequentialLateralBarrierRow.argtypes = [
        ctypes.POINTER(ctypes.c_float), ctypes.POINTER(ctypes.c_float),
        ctypes.POINTER(ctypes.c_float), ctypes.POINTER(ctypes.c_float),
    ]
    lib.sequentialLateralBarrierRow.restype = ctypes.c_bool
    lib.sequentialReturnScanYawDeg.argtypes = [
        ctypes.c_float, ctypes.c_int, ctypes.c_float,
    ]
    lib.sequentialReturnScanYawDeg.restype = ctypes.c_float
    lib.sequentialSlewYawDeg.argtypes = [
        ctypes.c_float, ctypes.c_float, ctypes.c_float,
    ]
    lib.sequentialSlewYawDeg.restype = ctypes.c_float
    lib.sequentialClearanceSpeedMps.argtypes = [ctypes.c_float] * 4
    lib.sequentialClearanceSpeedMps.restype = ctypes.c_float
    lib.sequentialRateLimitSpeedMps.argtypes = [ctypes.c_float] * 5
    lib.sequentialRateLimitSpeedMps.restype = ctypes.c_float
    return lib


def config(**changes) -> Config:
    values = dict(
        confidence_min=0.0, drone_radius_m=0.10, tracking_margin_m=0.08,
        latency_s=0.08, perception_margin_m=0.03,
        confidence_margin_gain_m=0.05, maximum_range_m=6.0,
        conservative_default_offset_m=0.25,
        trigger_distance_m=1.2, distance_weight=0.45, goal_weight=0.40,
        hysteresis_weight=0.15, dynamic_weight=0.10,
        direction_safe_min_m=0.30,
    )
    values.update(changes)
    return Config(**values)


def plan(library, distances, confidences, goal=(1, 0, 0), velocity=(0, 0, 0),
         previous=-1, cfg=None) -> Result:
    result = Result()
    chosen_config = cfg or config()
    library.sequentialObstacleControlPlan(
        F4(*distances), F4(*confidences), F3(*goal), F3(*velocity), previous,
        ctypes.byref(chosen_config), ctypes.byref(result),
    )
    return result


class SequentialObstacleControlTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tempdir = tempfile.TemporaryDirectory(prefix="sequential-control-")
        cls.library = build_library(Path(cls.tempdir.name) / "libseq.so")

    @classmethod
    def tearDownClass(cls):
        cls.tempdir.cleanup()

    def test_confidence_is_ignored(self) -> None:
        result = plan(self.library, [0.1] * 4, [-1] * 4)
        self.assertFalse(result.stop)
        self.assertTrue(result.valid)
        self.assertEqual(result.reliable_mask, 0)
        self.assertEqual(result.avoidance_pressure, 1.0)

    def test_metric_margins_do_not_change_classification(self) -> None:
        result = plan(self.library, [0.4] * 4, [2] * 4,
                      velocity=(2, 0, 0))
        self.assertEqual(result.reliable_mask, 0b1111)
        self.assertAlmostEqual(result.effective_offset_m[0], 0.4)
        self.assertAlmostEqual(result.margin_m[0], 0.0)

    def test_threshold_value_is_safe(self) -> None:
        result = plan(self.library, [0.30] * 4, [-100, -1, 1, 100])
        self.assertEqual(result.reliable_mask, 0b1111)
        self.assertEqual(result.avoidance_pressure, 0.0)

    def test_confidence_does_not_change_open_mask(self) -> None:
        result = plan(self.library, [0.1, 2, 2, 2], [-1, 2, 2, 2])
        self.assertTrue(result.valid)
        self.assertEqual(result.reliable_mask, 0b1110)
        self.assertAlmostEqual(result.effective_offset_m[0], 0.1)

    def test_partial_blockage_does_not_trigger_avoidance(self) -> None:
        result = plan(self.library, [3, 0.1, 0.1, 3], [2, -1, -1, 2])
        self.assertTrue(result.valid)
        self.assertEqual(result.reliable_mask, 0b1001)
        self.assertAlmostEqual(result.avoidance_pressure, 0.0)

    def test_blocked_direction_does_not_hide_other_escape_directions(self) -> None:
        result = plan(self.library, [0.1, 2, 2, 2], [2, 2, 2, 2])
        self.assertTrue(result.valid)
        self.assertEqual(result.reliable_mask, 0b1110)
        blocked = plan(self.library, [0.1] * 4, [2] * 4)
        self.assertFalse(blocked.stop)
        self.assertTrue(blocked.valid)
        self.assertEqual(blocked.avoidance_pressure, 1.0)

    def test_goal_alignment_selects_matching_side(self) -> None:
        left = plan(self.library, [2] * 4, [2] * 4, goal=(1, -1, 0))
        right = plan(self.library, [2] * 4, [2] * 4, goal=(1, 1, 0))
        self.assertEqual(left.chosen_direction, 0)
        self.assertEqual(right.chosen_direction, 3)

    def test_hysteresis_breaks_an_ambiguous_tie(self) -> None:
        result = plan(
            self.library, [2] * 4, [2] * 4, goal=(1, 0, 0), previous=3,
            cfg=config(goal_weight=0.0, hysteresis_weight=1.0),
        )
        self.assertEqual(result.chosen_direction, 3)

    def test_center_obstacle_raises_avoidance_pressure(self) -> None:
        far = plan(self.library, [3, 3, 3, 3], [2] * 4)
        near = plan(self.library, [0.2, 0.2, 0.2, 0.2], [2] * 4)
        self.assertEqual(far.avoidance_pressure, 0.0)
        self.assertEqual(near.avoidance_pressure, 1.0)

    def test_danger_average_fills_then_rolls(self) -> None:
        average = DangerAverage()
        values = [self.library.sequentialDangerAverageUpdate(
            ctypes.byref(average), 4, 5) for _ in range(5)]
        self.assertEqual(values, [4.0] * 5)
        self.assertEqual(average.count, 5)
        rolled = self.library.sequentialDangerAverageUpdate(
            ctypes.byref(average), 0, 5)
        self.assertAlmostEqual(rolled, 3.2, places=5)
        self.assertEqual(average.count, 5)

    def test_danger_average_window_change_resets_history(self) -> None:
        average = DangerAverage()
        self.library.sequentialDangerAverageUpdate(
            ctypes.byref(average), 4, 5)
        changed = self.library.sequentialDangerAverageUpdate(
            ctypes.byref(average), 1, 3)
        self.assertEqual(changed, 1.0)
        self.assertEqual(average.count, 1)
        self.assertEqual(average.window, 3)

    def test_directional_clearance_average_preserves_spatial_profile(self) -> None:
        average, mean = ClearanceAverage(), F4()
        self.library.sequentialClearanceAverageUpdate(
            ctypes.byref(average), F4(0.2, 0.3, 0.4, 0.5), 3, mean)
        self.library.sequentialClearanceAverageUpdate(
            ctypes.byref(average), F4(0.4, 0.5, 0.6, 0.7), 3, mean)
        self.assertEqual(average.count, 2)
        for actual, expected in zip(mean, (0.3, 0.4, 0.5, 0.6)):
            self.assertAlmostEqual(actual, expected)

    def test_directional_clearance_average_rolls_each_direction(self) -> None:
        average, mean = ClearanceAverage(), F4()
        for values in ([0.1, 0.2, 0.3, 0.4],
                       [0.2, 0.3, 0.4, 0.5],
                       [0.3, 0.4, 0.5, 0.6],
                       [0.4, 0.5, 0.6, 0.7]):
            self.library.sequentialClearanceAverageUpdate(
                ctypes.byref(average), F4(*values), 3, mean)
        self.assertEqual(average.count, 3)
        self.assertAlmostEqual(mean[0], 0.3)
        self.assertAlmostEqual(mean[3], 0.6)

    def test_binary_open_frequency_uses_outer_slices_only(self) -> None:
        score0, score3 = ctypes.c_float(), ctypes.c_float()
        side = self.library.sequentialSelectEvasionSide(
            F4(0.2, 1.0, 1.0, 0.8), 0.03, -1,
            ctypes.byref(score0), ctypes.byref(score3))
        self.assertEqual(side, 3)
        self.assertAlmostEqual(score0.value, 0.2)
        self.assertAlmostEqual(score3.value, 0.8)

    def test_binary_open_frequency_tie_alternates_side(self) -> None:
        scores = F4(0.4, 0.4, 0.4, 0.4)
        self.assertEqual(self.library.sequentialSelectEvasionSide(
            scores, 0.03, 0, None, None), 3)
        self.assertEqual(self.library.sequentialSelectEvasionSide(
            scores, 0.03, 3, None, None), 0)

    def test_right_evasion_barrier_blocks_leftward_return(self) -> None:
        row, boundary = F3(), ctypes.c_float()
        self.assertTrue(self.library.sequentialLateralBarrierRow(
            F3(0, -1, 0), F3(1, -0.35, 0.5), row,
            ctypes.byref(boundary)))
        self.assertAlmostEqual(row[0], 0.0)
        self.assertAlmostEqual(row[1], 1.0)
        self.assertAlmostEqual(boundary.value, -0.35)
        self.assertLessEqual(row[1] * -0.50, boundary.value)
        self.assertGreater(row[1] * 0.0, boundary.value)

    def test_left_evasion_barrier_blocks_rightward_return(self) -> None:
        row, boundary = F3(), ctypes.c_float()
        self.assertTrue(self.library.sequentialLateralBarrierRow(
            F3(0, 1, 0), F3(1, 0.35, 0.5), row,
            ctypes.byref(boundary)))
        self.assertAlmostEqual(row[1], -1.0)
        self.assertAlmostEqual(boundary.value, -0.35)
        self.assertLessEqual(row[1] * 0.50, boundary.value)
        self.assertGreater(row[1] * 0.0, boundary.value)

    def test_right_evasion_scans_left(self) -> None:
        yaw = self.library.sequentialReturnScanYawDeg(0.0, 0, 50.0)
        self.assertAlmostEqual(yaw, 50.0)

    def test_left_evasion_scans_right(self) -> None:
        yaw = self.library.sequentialReturnScanYawDeg(0.0, 3, 50.0)
        self.assertAlmostEqual(yaw, -50.0)

    def test_return_scan_yaw_wraps(self) -> None:
        yaw = self.library.sequentialReturnScanYawDeg(160.0, 0, 50.0)
        self.assertAlmostEqual(yaw, -150.0)

    def test_return_scan_yaw_slews_without_a_step(self) -> None:
        yaw = self.library.sequentialSlewYawDeg(0.0, 50.0, 0.9)
        self.assertAlmostEqual(yaw, 0.9)

    def test_return_scan_yaw_slew_uses_short_wrapped_path(self) -> None:
        yaw = self.library.sequentialSlewYawDeg(179.0, -179.0, 1.0)
        self.assertAlmostEqual(yaw, 180.0)

    def test_return_scan_yaw_slew_stops_exactly_at_goal(self) -> None:
        yaw = self.library.sequentialSlewYawDeg(49.5, 50.0, 0.9)
        self.assertAlmostEqual(yaw, 50.0)

    def test_clearance_speed_uses_stopping_distance(self) -> None:
        speed = self.library.sequentialClearanceSpeedMps(
            0.30, 0.25, 0.35, 0.40)
        self.assertAlmostEqual(speed, 0.20, places=5)

    def test_clearance_speed_is_capped_by_cruise_speed(self) -> None:
        speed = self.library.sequentialClearanceSpeedMps(
            1.0, 0.25, 0.35, 0.40)
        self.assertAlmostEqual(speed, 0.35, places=5)

    def test_clearance_speed_stops_at_safety_distance(self) -> None:
        self.assertEqual(self.library.sequentialClearanceSpeedMps(
            0.25, 0.25, 0.35, 0.40), 0.0)

    def test_nonfinite_clearance_stops(self) -> None:
        self.assertEqual(self.library.sequentialClearanceSpeedMps(
            float("nan"), 0.25, 0.35, 0.40), 0.0)

    def test_speed_rate_limit_uses_separate_accel_and_braking(self) -> None:
        accelerated = self.library.sequentialRateLimitSpeedMps(
            0.10, 0.35, 0.25, 0.40, 0.02)
        braked = self.library.sequentialRateLimitSpeedMps(
            0.30, 0.0, 0.25, 0.40, 0.02)
        self.assertAlmostEqual(accelerated, 0.105, places=5)
        self.assertAlmostEqual(braked, 0.292, places=5)


if __name__ == "__main__":
    unittest.main()
