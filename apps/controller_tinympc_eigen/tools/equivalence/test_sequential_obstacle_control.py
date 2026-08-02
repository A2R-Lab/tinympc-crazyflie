#!/usr/bin/env python3
"""Host tests for the four-direction carrot-and-stick planner."""

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
    return lib


def config(**changes) -> Config:
    values = dict(
        confidence_min=0.0, drone_radius_m=0.10, tracking_margin_m=0.08,
        latency_s=0.08, perception_margin_m=0.03,
        confidence_margin_gain_m=0.05, maximum_range_m=6.0,
        conservative_default_offset_m=0.25,
        trigger_distance_m=1.2, distance_weight=0.45, goal_weight=0.40,
        hysteresis_weight=0.15, dynamic_weight=0.10,
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

    def test_low_confidence_is_never_free_space(self) -> None:
        result = plan(self.library, [5, 5, 5, 5], [-1, -1, -1, -1])
        self.assertTrue(result.stop)
        self.assertFalse(result.valid)
        self.assertEqual(result.reliable_mask, 0)

    def test_confidence_and_latency_reduce_effective_clearance(self) -> None:
        low_conf = plan(self.library, [2] * 4, [0] * 4)
        high_conf = plan(self.library, [2] * 4, [4] * 4)
        moving = plan(self.library, [2] * 4, [4] * 4,
                      velocity=(2, 0, 0))
        self.assertGreater(low_conf.margin_m[0], high_conf.margin_m[0])
        self.assertGreater(moving.margin_m[0], high_conf.margin_m[0])
        self.assertLess(moving.effective_offset_m[0],
                        high_conf.effective_offset_m[0])

    def test_unreliable_direction_gets_conservative_boundary(self) -> None:
        result = plan(self.library, [5, 2, 2, 2], [-1, 2, 2, 2])
        self.assertTrue(result.valid)
        self.assertEqual(result.reliable_mask, 0b1110)
        self.assertAlmostEqual(result.effective_offset_m[0], 0.25)

    def test_close_plane_does_not_hide_other_escape_directions(self) -> None:
        result = plan(self.library, [0.1, 2, 2, 2], [2, 2, 2, 2])
        self.assertTrue(result.valid)
        self.assertEqual(result.effective_offset_m[0], 0.0)
        blocked = plan(self.library, [0.1] * 4, [2] * 4)
        self.assertTrue(blocked.stop)

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
        near = plan(self.library, [3, 0.8, 0.8, 3], [2] * 4)
        self.assertGreater(near.avoidance_pressure, far.avoidance_pressure)


if __name__ == "__main__":
    unittest.main()
