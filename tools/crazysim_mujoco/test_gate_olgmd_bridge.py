import importlib.util
from pathlib import Path
import struct
import sys
import types
import unittest
from unittest import mock
import zlib

import numpy as np


TOOLS = Path(__file__).resolve().parent
sys.path.insert(0, str(TOOLS))
SPEC = importlib.util.spec_from_file_location("gate_olgmd_vision_bridge", TOOLS / "vision_bridge.py")
BRIDGE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = BRIDGE
SPEC.loader.exec_module(BRIDGE)


class _Tensor:
    def __init__(self, name, shape):
        self.name = name
        self.shape = shape


class _Session:
    def __init__(self, *_args, **_kwargs):
        self.last_input = None

    def get_inputs(self):
        return [_Tensor("0", [1, 1, 96, 160])]

    def get_outputs(self):
        return [_Tensor("127", [1, 8])]

    def run(self, names, inputs):
        self.last_input = inputs["0"]
        self.last_names = names
        return [np.zeros((1, 8), dtype=np.float32)]


class GateOlgmdBridgeTest(unittest.TestCase):
    def _adapter(self):
        runtime = types.SimpleNamespace(InferenceSession=_Session)
        bundle = Path("/home/cchen/pulp-frontnet/PyTorch")
        with mock.patch.dict(sys.modules, {"onnxruntime": runtime}):
            return BRIDGE.GateFrontnetOlgmdAdapter(bundle)

    def test_adapter_runs_olgmd_before_gate_and_emits_two_packets(self):
        adapter = self._adapter()
        prediction = adapter.predict(np.full((160, 160), 128, dtype=np.uint8))
        self.assertFalse(prediction.olgmd_valid)
        self.assertFalse(prediction.imminent_threat)
        self.assertTrue(prediction.gate_valid)
        self.assertEqual(adapter.session.last_input.shape, (1, 1, 96, 160))

        packets = BRIDGE.packet_datagrams(prediction, 11, 456)
        self.assertEqual(tuple(map(len, packets)), (16, 48))
        self.assertEqual(packets[0][:4], b"\x90\x19\x08\x40")
        self.assertEqual(packets[1][:4], b"\x90\x19\x08\x41")
        for packet in packets:
            self.assertEqual(
                zlib.crc32(packet[:-4]) & 0xFFFFFFFF,
                struct.unpack("<I", packet[-4:])[0],
            )

    def test_canonical_obstacle_adapter_is_repo_owned_olgmd_only(self):
        bundle = TOOLS / "models" / "olgmd_obstacle_v1"
        with mock.patch.dict(sys.modules, {"onnxruntime": None}):
            adapter, kind = BRIDGE.make_adapter(
                "auto", bundle, threshold=0.0, camera_fps=30.0)
        self.assertEqual(
            kind, BRIDGE.GateFrontnetOlgmdObstacleOnlyAdapter._RUNTIME_ADAPTER)
        prediction = adapter.predict(np.full((160, 160), 128, dtype=np.uint8))
        self.assertFalse(prediction.gate_valid)
        self.assertEqual(prediction.gate_reason, "canonical_olgmd_obstacle")
        self.assertEqual(adapter.last_input.shape, (160, 160))

    def test_existing_prediction_retains_single_packet(self):
        prediction = BRIDGE.Prediction(
            False, False, False, np.full(4, 6.0), np.zeros(4), np.zeros(3),
            0.0, 0.0, False, np.zeros((4, 2)), 0.0, "none")
        packets = BRIDGE.packet_datagrams(prediction, 1, 1)
        self.assertEqual(len(packets), 1)
        self.assertEqual(packets[0], BRIDGE.packet_bytes(prediction, 1, 1))


if __name__ == "__main__":
    unittest.main()
