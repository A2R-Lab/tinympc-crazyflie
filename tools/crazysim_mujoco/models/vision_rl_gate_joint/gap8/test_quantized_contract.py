import unittest

from tools.crazysim_mujoco.models.vision_rl_gate_joint.gap8.quantized_contract import build_quantized_contract


class QuantizedContractTest(unittest.TestCase):
    def test_contract_carries_authoritative_epsilon_decode(self):
        source = {"schema": "float", "output": {"layout": ["x"]},
                  "terminal_affine": {"output_scale": [1.0], "output_shift": [2.0]}}
        result = build_quantized_contract(source, 0.03125)
        self.assertEqual(result["terminal_quantization"]["terminal_epsilon"], 0.03125)
        self.assertIn("encoded*terminal_epsilon", result["output"]["decode"])
        self.assertEqual(result["output"]["dtype"], "uint8_encoded")
        self.assertEqual(source["schema"], "float")

    def test_invalid_epsilon_fails_closed(self):
        with self.assertRaises(ValueError):
            build_quantized_contract({"output": {}}, 0.0)


if __name__ == "__main__": unittest.main()
