from __future__ import annotations

import importlib.util
import sys
import tempfile
import unittest
from pathlib import Path

import numpy as np

from tools.crazysim_mujoco.rl.model import TemporalPolicy
from tools.crazysim_mujoco.rl.reward import transition_reward
from tools.crazysim_mujoco.rl.data import _actions


ROOT = Path(__file__).resolve().parents[4]
BRIDGE_PATH = ROOT / "tools/crazysim_mujoco/vision_bridge.py"
SPEC = importlib.util.spec_from_file_location("vision_bridge_test", BRIDGE_PATH)
BRIDGE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
sys.modules[SPEC.name] = BRIDGE
SPEC.loader.exec_module(BRIDGE)


class FakeSession:
    def __init__(self, logits):
        self.logits = np.asarray([logits], dtype=np.float32)
        self.inputs = []

    def run(self, _outputs, inputs):
        self.inputs.append(inputs["frames"].copy())
        return [self.logits]


def fake_adapter(logits):
    adapter = BRIDGE.VisionRlAdapter.__new__(BRIDGE.VisionRlAdapter)
    adapter.session = FakeSession(logits)
    adapter.input_name = "frames"
    adapter.previous_frame = None
    adapter.last_input = None
    adapter.last_input_raw = None
    return adapter


class VisionRlContractTest(unittest.TestCase):
    def test_previous_current_order_and_deterministic_reset(self):
        adapter = fake_adapter([1.0, 0.0, 0.0])
        first = np.full((160, 160), 11, dtype=np.uint8)
        second = np.full((160, 160), 29, dtype=np.uint8)
        adapter.predict(first)
        adapter.predict(second)
        first_input = np.rint(adapter.session.inputs[0] * 255.0).astype(np.uint8)
        second_input = np.rint(adapter.session.inputs[1] * 255.0).astype(np.uint8)
        self.assertTrue(np.all(first_input[0, 0] == first))
        self.assertTrue(np.all(first_input[0, 1] == first))
        self.assertTrue(np.all(second_input[0, 0] == first))
        self.assertTrue(np.all(second_input[0, 1] == second))
        adapter.reset()
        adapter.predict(second)
        reset_input = np.rint(adapter.session.inputs[2] * 255.0).astype(np.uint8)
        self.assertTrue(np.all(reset_input[0, 0] == second))

    def test_action_packet_mapping(self):
        expected = ((0, 0.0, 0.0), (1, 1.0, 1.0), (2, -1.0, 1.0))
        frame = np.zeros((160, 160), dtype=np.uint8)
        for action, steering, collision in expected:
            logits = [-1.0, -1.0, -1.0]
            logits[action] = 2.0
            prediction = fake_adapter(logits).predict(frame)
            self.assertEqual(prediction.action, action)
            self.assertEqual(prediction.steering, steering)
            self.assertEqual(prediction.collision, collision)

    def test_reward_penalizes_reverse_contact_and_proximity(self):
        safe = transition_reward([0.1], [0.0], [0.5], [1.0], [0], [0], [0], [0])
        unsafe = transition_reward([0.1], [0.0], [-0.5], [0.0], [1], [0], [1], [0])
        self.assertGreater(float(safe[0]), float(unsafe[0]))

    def test_legacy_rows_ignore_placeholder_rl_action(self):
        vision = {
            "rl_action": np.asarray([-1.0, -1.0, -1.0]),
            "collision": np.asarray([0.1, 0.9, 0.9]),
            "steering": np.asarray([0.0, 0.4, -0.3]),
        }
        np.testing.assert_array_equal(_actions(vision, 3), [0, 1, 2])

    def test_policy_is_deterministic_for_fixed_seed(self):
        import torch
        torch.manual_seed(7)
        first = TemporalPolicy().eval()
        torch.manual_seed(7)
        second = TemporalPolicy().eval()
        frames = torch.linspace(0.0, 1.0, 2 * 160 * 160).reshape(1, 2, 160, 160)
        torch.testing.assert_close(first(frames), second(frames))

    def test_torch_onnx_parity_when_available(self):
        try:
            import onnxruntime as ort
            import torch
        except ImportError:
            self.skipTest("torch/onnxruntime unavailable")
        torch.manual_seed(9)
        policy = TemporalPolicy().eval()
        frames = torch.rand(1, 2, 160, 160)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "policy.onnx"
            torch.onnx.export(policy, frames, path, input_names=["frames"],
                              output_names=["action_logits"], opset_version=17,
                              dynamo=False)
            session = ort.InferenceSession(str(path), providers=["CPUExecutionProvider"])
            actual = session.run(None, {"frames": frames.numpy()})[0]
        np.testing.assert_allclose(actual, policy(frames).detach().numpy(),
                                   rtol=1.0e-5, atol=1.0e-5)


if __name__ == "__main__":
    unittest.main()
