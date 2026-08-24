#!/usr/bin/env python3
"""Focused contract tests for the manifest-only combined vision adapter."""

from __future__ import annotations

import importlib.util
import json
import sys
import tempfile
import unittest
from pathlib import Path

import numpy as np


HERE = Path(__file__).resolve().parent
SPEC = importlib.util.spec_from_file_location("combined_vision_bridge", HERE / "vision_bridge.py")
BRIDGE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
sys.modules[SPEC.name] = BRIDGE
SPEC.loader.exec_module(BRIDGE)


class _NavigationTeacher:
    def __init__(self, prediction):
        self.prediction = prediction
        self.frames = []
        self.last_input = None
        self.last_input_raw = None

    def predict(self, frame):
        self.frames.append(frame)
        self.last_input = frame
        self.last_input_raw = np.stack((frame, frame), axis=-1)
        return self.prediction

    def reset(self):
        pass


class _GateTeacher:
    def __init__(self, prediction):
        self.prediction = prediction
        self.frames = []

    def predict(self, frame):
        self.frames.append(frame)
        return self.prediction


class CombinedAdapterTest(unittest.TestCase):
    def test_preserves_navigation_and_uses_gate_fields(self):
        navigation = BRIDGE.Prediction(
            metric=False, sector_danger=False, navigation=True,
            clearance=np.asarray([6.0, 6.0, 6.0, 6.0]),
            confidence=np.zeros(4), danger=np.asarray([1.0, 1.0, 1.0, 1.0]),
            steering=-1.0, collision=1.0, gate_valid=False,
            corners=np.zeros((4, 2)), gate_confidence=0.0,
            gate_reason="not_provided", raw_danger=np.asarray([.1, .2, .3, .4]),
            danger_threshold=.6, action=2, action_logits=(1.25, -2.0, 3.0))
        gate = BRIDGE.Prediction(
            metric=False, sector_danger=False, navigation=True,
            clearance=np.zeros(4), confidence=np.zeros(4), danger=np.zeros(4),
            steering=0.0, collision=0.0, gate_valid=True,
            corners=np.asarray(((.1, .2), (.8, .2), (.8, .9), (.1, .9))),
            gate_confidence=.85, gate_reason="accepted",
            gate_intrinsics=(.55, .56, .51, .52))
        adapter = BRIDGE.CombinedGateRlAdapter.__new__(BRIDGE.CombinedGateRlAdapter)
        adapter.obstacle = _NavigationTeacher(navigation)
        adapter.gate = _GateTeacher(gate)
        adapter.last_input = None
        adapter.last_input_raw = None
        frame = np.full((160, 160), 17, dtype=np.uint8)

        actual = adapter.predict(frame)

        self.assertIs(actual.clearance, navigation.clearance)
        self.assertIs(actual.confidence, navigation.confidence)
        self.assertIs(actual.danger, navigation.danger)
        self.assertIs(actual.raw_danger, navigation.raw_danger)
        self.assertEqual((actual.navigation, actual.steering, actual.collision,
                          actual.action, actual.action_logits),
                         (navigation.navigation, navigation.steering,
                          navigation.collision, navigation.action,
                          navigation.action_logits))
        self.assertEqual((actual.gate_valid, actual.gate_confidence,
                          actual.gate_reason, actual.gate_intrinsics),
                         (gate.gate_valid, gate.gate_confidence,
                          gate.gate_reason, gate.gate_intrinsics))
        np.testing.assert_array_equal(actual.corners, gate.corners)
        self.assertEqual(len(adapter.obstacle.frames), 1)
        self.assertEqual(len(adapter.gate.frames), 1)
        self.assertIs(adapter.obstacle.frames[0], adapter.gate.frames[0])

    def test_manifest_paths_are_relative_and_auto_detected(self):
        with tempfile.TemporaryDirectory() as directory:
            bundle = Path(directory) / "bundle"
            obstacle = bundle / "teachers" / "policy.onnx"
            gate = bundle / "teachers" / "gate.onnx"
            obstacle.parent.mkdir(parents=True)
            obstacle.touch()
            gate.touch()
            (bundle / "bundle.json").write_text(json.dumps({
                "runtime_adapter": "combined_gate_rl",
                "teacher_models": {
                    "obstacle_rl": {"path": "teachers/policy.onnx"},
                    "gate_espnet": {"path": "teachers/gate.onnx"},
                },
            }))
            self.assertEqual(BRIDGE._combined_gate_rl_teacher_paths(bundle),
                             (obstacle.resolve(), gate.resolve()))
            with self.assertRaisesRegex(ValueError, "manifest-relative"):
                (bundle / "bundle.json").write_text(json.dumps({
                    "runtime_adapter": "combined_gate_rl",
                    "teacher_models": {
                        "obstacle_rl": {"path": str(obstacle.resolve())},
                        "gate_espnet": {"path": "teachers/gate.onnx"},
                    },
                }))
                BRIDGE._combined_gate_rl_teacher_paths(bundle)


if __name__ == "__main__":
    unittest.main()
