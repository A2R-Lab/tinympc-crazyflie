#!/usr/bin/env python3
"""Focused host tests for the explicit single-ONNX joint policy adapter."""

from __future__ import annotations

import hashlib
import importlib.util
import json
import sys
import tempfile
import types
import unittest
from pathlib import Path

import numpy as np


HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
SPEC = importlib.util.spec_from_file_location("joint_vision_bridge", HERE / "vision_bridge.py")
BRIDGE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
sys.modules[SPEC.name] = BRIDGE
SPEC.loader.exec_module(BRIDGE)


class _Meta:
    def __init__(self, name, shape):
        self.name = name
        self.shape = shape


class _Session:
    output_values = None
    output_metadata = (
        _Meta("action_logits", [1, 3]), _Meta("gate_corners", [1, 4, 2]),
        _Meta("gate_confidence_logit", [1]),
    )

    def __init__(self, path, providers):
        self.path = path
        self.providers = providers
        self.calls = []

    def get_inputs(self):
        return [_Meta("frames", [1, 2, 160, 160])]

    def get_outputs(self):
        return list(self.output_metadata)

    def run(self, names, feed):
        self.calls.append((names, feed))
        return self.output_values


class JointGateRlAdapterTest(unittest.TestCase):
    def setUp(self):
        self.module = types.SimpleNamespace(InferenceSession=_Session)
        self.previous_ort = sys.modules.get("onnxruntime")
        sys.modules["onnxruntime"] = self.module
        _Session.output_metadata = (
            _Meta("action_logits", [1, 3]), _Meta("gate_corners", [1, 4, 2]),
            _Meta("gate_confidence_logit", [1]),
        )
        self.good_corners = np.asarray([[[.2, .2], [.8, .2], [.8, .8], [.2, .8]]], np.float32)
        _Session.output_values = [np.asarray([[0., 3., 1.]], np.float32), self.good_corners,
                                 np.asarray([2.], np.float32)]

    def tearDown(self):
        if self.previous_ort is None:
            del sys.modules["onnxruntime"]
        else:
            sys.modules["onnxruntime"] = self.previous_ort

    def _bundle(self, mutate=None):
        directory = tempfile.TemporaryDirectory()
        bundle = Path(directory.name)
        policy = bundle / "policy.onnx"
        policy.write_bytes(b"joint-policy")
        manifest = {
            "format": "tinympc-joint-gate-obstacle-student-v1",
            "runtime_adapter": "joint_gate_rl",
            "deployment": {
                "input": {"name": "frames", "shape": [1, 2, 160, 160], "dtype": "float32",
                          "range": [0, 1], "temporal_order": ["previous", "current"]},
                "outputs": {"action_logits": [1, 3],
                            "gate_corners": {"shape": [1, 4, 2], "range": [0, 1],
                                             "order": ["TL", "TR", "BR", "BL"]},
                            "gate_confidence_logit": [1]},
                "actions": ["TRACK", "LEFT", "RIGHT"],
                "fixed_intrinsics": {"fx_normalized": .56, "fy_normalized": .57,
                                     "cx_normalized": .51, "cy_normalized": .46},
            },
            "artifacts": {"policy_onnx": {"path": "policy.onnx",
                          "sha256": hashlib.sha256(policy.read_bytes()).hexdigest()}},
        }
        if mutate is not None:
            mutate(manifest)
        (bundle / "bundle.json").write_text(json.dumps(manifest))
        self.addCleanup(directory.cleanup)
        return bundle

    def test_bundle_hash_contract_and_explicit_resolution(self):
        bundle = self._bundle()
        adapter, kind = BRIDGE.make_adapter("joint_gate_rl", bundle, .3)
        self.assertIsInstance(adapter, BRIDGE.JointGateRlAdapter)
        self.assertEqual(kind, "joint_gate_rl")
        self.assertEqual(adapter.gate_intrinsics, (.56, .57, .51, .46))
        with self.assertRaisesRegex(ValueError, "joint_gate_rl"):
            BRIDGE.JointGateRlAdapter(self._bundle(
                lambda document: document.__setitem__("runtime_adapter", "combined_gate_rl")))
        with self.assertRaisesRegex(ValueError, "checksum"):
            BRIDGE.JointGateRlAdapter(self._bundle(
                lambda document: document["artifacts"]["policy_onnx"].__setitem__("sha256", "0" * 64)))

    def test_model_and_inference_shape_nonfinite_fail_explicitly(self):
        bundle = self._bundle()
        _Session.output_metadata = (_Meta("action_logits", [1, 3]),)
        with self.assertRaisesRegex(ValueError, "ONNX contract"):
            BRIDGE.JointGateRlAdapter(bundle)
        _Session.output_metadata = (
            _Meta("action_logits", [1, 3]), _Meta("gate_corners", [1, 4, 2]),
            _Meta("gate_confidence_logit", [1]),
        )
        adapter = BRIDGE.JointGateRlAdapter(bundle)
        _Session.output_values = [np.zeros((1, 2), np.float32), self.good_corners,
                                 np.zeros((1,), np.float32)]
        with self.assertRaisesRegex(ValueError, "malformed output shape"):
            adapter.predict(np.zeros((160, 160), np.uint8))
        _Session.output_values = [np.asarray([[np.nan, 0., 0.]], np.float32), self.good_corners,
                                 np.zeros((1,), np.float32)]
        with self.assertRaisesRegex(ValueError, "non-finite"):
            adapter.predict(np.zeros((160, 160), np.uint8))

    def test_action_mapping_valid_gate_and_packet_parity(self):
        adapter = BRIDGE.JointGateRlAdapter(self._bundle())
        prediction = adapter.predict(np.full((160, 160), 5, np.uint8))
        self.assertEqual((prediction.action, prediction.steering, prediction.collision), (1, 1.0, 1.0))
        self.assertTrue(prediction.gate_valid)
        self.assertEqual(prediction.gate_reason, "accepted")
        packet = BRIDGE.packet_bytes(prediction, 7, 123)
        expected = BRIDGE.Prediction(
            metric=False, sector_danger=False, navigation=True,
            clearance=np.full(4, 6.0), confidence=np.zeros(4),
            danger=np.full(4, 1.0), steering=1.0, collision=1.0,
            gate_valid=True, corners=self.good_corners[0],
            gate_confidence=float(BRIDGE.sigmoid(np.float32(2.0))), gate_reason="accepted",
            gate_intrinsics=(.56, .57, .51, .46), action=1,
            action_logits=(0.0, 3.0, 1.0))
        self.assertEqual(packet, BRIDGE.packet_bytes(expected, 7, 123))
        parsed = BRIDGE.VISION_BODY.unpack(packet[:-4])
        self.assertEqual(parsed[3] & BRIDGE.FLAG_NAVIGATION, BRIDGE.FLAG_NAVIGATION)
        self.assertEqual(parsed[3] & BRIDGE.FLAG_GATE, BRIDGE.FLAG_GATE)
        self.assertAlmostEqual(parsed[16], 1.0)  # steering in the stable packet layout

    def test_invalid_gate_clears_only_gate_fields_not_navigation(self):
        adapter = BRIDGE.JointGateRlAdapter(self._bundle())
        _Session.output_values = [np.asarray([[.5, .2, 3.]], np.float32),
                                 np.asarray([[[.8, .2], [.2, .2], [.2, .8], [.8, .8]]], np.float32),
                                 np.asarray([8.], np.float32)]
        invalid = adapter.predict(np.zeros((160, 160), np.uint8))
        self.assertEqual((invalid.action, invalid.steering, invalid.collision), (2, -1.0, 1.0))
        np.testing.assert_allclose(invalid.action_logits, (.5, .2, 3.0))
        self.assertFalse(invalid.gate_valid)
        self.assertEqual(invalid.gate_reason, "ordering")
        self.assertEqual(invalid.gate_confidence, 0.0)
        np.testing.assert_array_equal(invalid.corners, np.zeros((4, 2), np.float32))
        self.assertEqual(invalid.gate_intrinsics, (.56, .57, .51, .46))

    def test_joint_controller_gate_opening_guard_is_short_and_opt_in(self):
        source = (HERE.parents[1] /
                  "apps/controller_tinympc_eigen/src/controller_tinympc.cpp").read_text()
        self.assertIn("joint_gate_maximum_geometry_dropout_samples = 20u", source)
        self.assertIn("joint_gate_maximum_association_samples = 300u", source)
        self.assertIn("++gate_poc_association_samples", source)
        self.assertIn("Joint gate association completed after hard lifetime", source)
        self.assertIn("Joint gate association completed after geometry timeout", source)
        self.assertIn("gate_poc_geometry_dropout_samples <=", source)
        self.assertIn("static bool gate_poc_completed = false", source)
        self.assertIn("!gate_poc_completed &&", source)
        self.assertIn("gate_poc_completed = true", source)
        self.assertIn("gate_corner_span_m = TINYMPC_GATE_CORNER_SPAN_M", source)
        self.assertIn("#elif TINYMPC_JOINT_GATE_RL_ENABLE", source)
        self.assertIn("Joint gate restoring obstacle authority", source)
        self.assertIn("Validate quadrilateral structure, not image-axis alignment", source)
        self.assertNotIn("maximum_roll_slope", source)

    def test_gate_servo_uses_rotating_path_normal_and_shifts_tunnel(self):
        source = (HERE.parents[1] /
                  "apps/controller_tinympc_eigen/src/controller_tinympc.cpp").read_text()
        servo = source.split("static void __attribute__((unused)) applyGateVisualServo(", 1)[1]
        servo = servo.split("static void maybeFuseGatePose(", 1)[0]
        self.assertIn("requested_gate_displacement_local - Xref[0].head<3>()", servo)
        self.assertIn("gate_path_normal_local.dot(", servo)
        self.assertIn("knot_path_normal_local", servo)
        self.assertIn("Xref[k].head<3>() +=", servo)
        self.assertIn("previous_gate_shifted_tunnel_frame", servo)
        self.assertIn("setPathTunnelHalfspaces(", servo)
        self.assertNotIn("requested_lateral_m - Xref[0](1)", servo)
        self.assertNotIn("Xref[k](1) += ramp * gate_lateral_offset_m", servo)


if __name__ == "__main__":
    unittest.main()
