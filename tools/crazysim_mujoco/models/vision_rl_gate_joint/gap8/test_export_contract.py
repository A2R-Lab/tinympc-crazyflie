import unittest

import numpy as np
from onnx import TensorProto, helper, numpy_helper

from .export_contract import signed_int8_weight_ranges


def graph_with_weight(values):
    initializer = numpy_helper.from_array(np.asarray(values, dtype=np.float32), "conv.weight")
    return helper.make_model(helper.make_graph([], "weight-test", [], [], [initializer]))


class SignedWeightContractTest(unittest.TestCase):
    def test_accepts_integral_signed_int8_weights(self):
        self.assertEqual(signed_int8_weight_ranges(graph_with_weight([-128, 0, 127])),
                         {"conv.weight": [-128, 127]})

    def test_rejects_coefficients_that_dory_would_wrap(self):
        with self.assertRaisesRegex(RuntimeError, "exceeds GAP8 signed-int8"):
            signed_int8_weight_ranges(graph_with_weight([0, 139]))

    def test_rejects_nonintegral_weights(self):
        with self.assertRaisesRegex(RuntimeError, "non-integral"):
            signed_int8_weight_ranges(graph_with_weight([0, 1.5]))


if __name__ == "__main__":
    unittest.main()
