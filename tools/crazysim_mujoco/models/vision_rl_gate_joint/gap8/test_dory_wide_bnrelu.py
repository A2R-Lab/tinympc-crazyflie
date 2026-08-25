#!/usr/bin/env python3
"""Focused regression tests for the 64-bit NeMO/DORY BNRelu lowering."""
from __future__ import print_function

import types
import unittest

import numpy as np

from tools.crazysim_mujoco.models.vision_rl_gate_joint.gap8 import dory_wide_bnrelu as wide


def _constant(value):
    return types.SimpleNamespace(value={"value": np.asarray(value)},
                                 input_indexes=["x"], output_index="constant",
                                 number_of_input_nodes=1, number_of_input_constants=1,
                                 min=0, max=255)


def _endpoint():
    return types.SimpleNamespace(input_indexes=["x"], output_index="y",
                                 number_of_input_nodes=1, number_of_input_constants=0,
                                 min=0, max=255)


class _Rewriter(object):
    def __init__(self, graph):
        self.graph = graph


class WideBNReluTest(unittest.TestCase):
    def _lower(self, k, l, outmul, divisor):
        rewriter = _Rewriter([_constant(k), _constant(l), _constant(outmul),
                              _constant(divisor), _endpoint()])
        wide._wide_bnrelu_pattern_rewriter(rewriter, [0, 1, 2, 3, 4])
        self.assertEqual(len(rewriter.graph), 1)
        return rewriter.graph[0]

    def test_non_power_of_two_multiplier_widens_without_wrap(self):
        node = self._lower([3, -4], [50000000, -50000000], 51, 1024)
        np.testing.assert_array_equal(node.k["value"], np.asarray([153, -204], dtype=np.int64))
        np.testing.assert_array_equal(node.l["value"],
                                      np.asarray([2550000000, -2550000000], dtype=np.int64))
        self.assertEqual(node.outshift["value"], 10)
        self.assertEqual(node.wide_requantization_strategy, "widened_int64_coefficients")

    def test_power_of_two_multiplier_uses_exact_divisor_fold(self):
        node = self._lower([7], [-9], 32, 1024)
        np.testing.assert_array_equal(node.k["value"], np.asarray([7], dtype=np.int64))
        np.testing.assert_array_equal(node.l["value"], np.asarray([-9], dtype=np.int64))
        self.assertEqual(node.outshift["value"], 5)
        self.assertEqual(node.wide_requantization_strategy, "power_of_two_divisor_fold")

    def test_out_of_range_wide_coefficient_fails_closed(self):
        with self.assertRaisesRegex(ValueError, "exceeds signed int64"):
            self._lower([1], [1 << 62], 3, 1024)

    def test_unsafe_bn_kernel_intermediate_fails_closed(self):
        with self.assertRaisesRegex(ValueError, "kernel intermediate"):
            self._lower([1 << 32], [0], 3, 1024)

    def test_invalid_divisor_fails_closed(self):
        with self.assertRaisesRegex(ValueError, "positive power of two"):
            self._lower([1], [1], 3, 1000)


if __name__ == "__main__":
    unittest.main()
