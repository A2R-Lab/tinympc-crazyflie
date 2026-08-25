#!/usr/bin/env python3
"""Opt-in 64-bit lowering for NeMO BN/ReLU constants on GAP8.

DORY's GAP8 backend already ships matching ``pulp-nn/64bit`` kernels. The
stock NeMO frontend rejects a non-power-of-two post multiplier once the legacy
32-bit packed coefficient would overflow. This adapter installs a strict
lowering only when the caller has selected 64-bit BN/ReLU constants.

It preserves NeMO's integer expression exactly. For a power-of-two post
multiplier it folds that multiplier into the divisor. Otherwise it widens both
coefficients to signed int64 before multiplying and retains the original
power-of-two divisor. Every scalar, divisor, and int64 bound is checked before
code generation; no coefficient is clipped or allowed to wrap.
"""
from __future__ import print_function

import numpy as np


INT64_MIN = -(1 << 63)
INT64_MAX = (1 << 63) - 1


def _integer_scalar(value, name):
    values = np.asarray(value)
    if values.size != 1:
        raise ValueError("%s must be a scalar, got shape %s" % (name, values.shape))
    scalar = values.reshape(()).item()
    integer = int(scalar)
    if scalar != integer:
        raise ValueError("%s must be integral, got %r" % (name, scalar))
    return integer


def _power_of_two_exponent(value, name):
    value = _integer_scalar(value, name)
    if value <= 0 or value & (value - 1):
        raise ValueError("%s must be a positive power of two, got %d" % (name, value))
    return value.bit_length() - 1


def _as_int64_exact(values, name):
    """Convert an integer tensor without relying on NumPy overflow behavior."""
    values = np.asarray(values)
    flattened = []
    for value in values.reshape(-1):
        scalar = value.item() if hasattr(value, "item") else value
        integer = int(scalar)
        if scalar != integer:
            raise ValueError("%s must contain only integral values, got %r" % (name, scalar))
        if integer < INT64_MIN or integer > INT64_MAX:
            raise ValueError("%s contains value outside signed int64: %d" % (name, integer))
        flattened.append(integer)
    return np.asarray(flattened, dtype=np.int64).reshape(values.shape)


def _multiply_int64_exact(values, multiplier, name):
    values = _as_int64_exact(values, name)
    flattened = []
    for value in values.reshape(-1):
        scaled = int(value) * multiplier
        if scaled < INT64_MIN or scaled > INT64_MAX:
            raise ValueError("%s * %d exceeds signed int64" % (name, multiplier))
        flattened.append(scaled)
    return np.asarray(flattened, dtype=np.int64).reshape(values.shape)


def _check_bn_kernel_intermediate(k, l):
    """Reject coefficients that could overflow GAP8's signed-int64 BN kernel.

    ``pulp_nn_bn_quant_u8`` receives a signed int32 accumulator and evaluates
    ``k * phi + lambda`` in signed int64.  Checking the complete int32 domain
    is conservative, but makes this lowering fail closed independently of a
    particular calibration corpus.
    """
    if np.asarray(k).shape != np.asarray(l).shape:
        raise ValueError("BNRelu k/lambda shapes differ: %s vs %s" %
                         (np.asarray(k).shape, np.asarray(l).shape))
    for coefficient, offset in zip(np.asarray(k).reshape(-1), np.asarray(l).reshape(-1)):
        maximum = abs(int(coefficient)) * ((1 << 31) - 1) + abs(int(offset))
        if maximum > INT64_MAX:
            raise ValueError("BNRelu signed-int64 kernel intermediate may overflow")


def _constant_value(node):
    candidates = [value["value"] for value in node.__dict__.values()
                  if isinstance(value, dict) and "value" in value]
    if len(candidates) != 1:
        raise ValueError("expected exactly one constant in DORY pattern node, got %d" % len(candidates))
    return candidates[0]


def _wide_bnrelu_pattern_rewriter(self, indexes):
    # Local import keeps algebra tests independent from the DORY install.
    from dory.Parsers import DORY_node

    node = DORY_node.DORY_node()
    node.name = "BNRelu"
    node.op_type = "BNRelu"
    node.input_indexes = self.graph[indexes[0]].input_indexes
    node.output_index = self.graph[indexes[-1]].output_index
    node.number_of_input_nodes = self.graph[indexes[0]].number_of_input_nodes
    node.number_of_input_constants = sum(self.graph[index].number_of_input_constants
                                         for index in indexes)
    node.branch_out = None
    node.branch_in = None
    node.branch_change = None
    node.branch_last = None

    k = _constant_value(self.graph[indexes[0]])
    l = _constant_value(self.graph[indexes[1]])
    outmul = _integer_scalar(_constant_value(self.graph[indexes[2]]), "BNRelu post multiplier")
    divisor_shift = _power_of_two_exponent(
        _constant_value(self.graph[indexes[3]]), "BNRelu divisor")
    if outmul <= 0:
        raise ValueError("BNRelu post multiplier must be positive, got %d" % outmul)

    if outmul & (outmul - 1) == 0:
        multiplier_shift = outmul.bit_length() - 1
        if multiplier_shift > divisor_shift:
            raise ValueError("BNRelu post multiplier 2**%d exceeds divisor 2**%d" %
                             (multiplier_shift, divisor_shift))
        packed_k = _as_int64_exact(k, "BNRelu k")
        packed_l = _as_int64_exact(l, "BNRelu lambda")
        packed_shift = divisor_shift - multiplier_shift
        strategy = "power_of_two_divisor_fold"
    else:
        packed_k = _multiply_int64_exact(k, outmul, "BNRelu k")
        packed_l = _multiply_int64_exact(l, outmul, "BNRelu lambda")
        packed_shift = divisor_shift
        strategy = "widened_int64_coefficients"

    _check_bn_kernel_intermediate(packed_k, packed_l)

    node.k = {"value": packed_k, "layout": ""}
    node.l = {"value": packed_l, "layout": ""}
    node.outshift = {"value": packed_shift, "layout": ""}
    node.min = self.graph[indexes[-1]].min
    node.max = self.graph[indexes[-1]].max
    node.constant_names = ["k", "l", "outshift"]
    node.wide_requantization_strategy = strategy
    for index in sorted(indexes, reverse=True):
        del self.graph[index]
    self.graph.insert(indexes[0], node)


def install_wide_bnrelu_lowering(config):
    """Install the 64-bit lowering and validate its matching GAP8 configuration."""
    if int(config.get("BNRelu_bits", 0)) != 64:
        raise ValueError("wide BNRelu lowering requires BNRelu_bits=64")
    from dory.Frontend_frameworks.NEMO.Pattern_rewriter import Pattern_rewriter
    Pattern_rewriter.BNRelu_pattern_rewriter = _wide_bnrelu_pattern_rewriter
    return {"enabled": True, "constant_bits": 64,
            "backend": "DORY GAP8 pulp-nn/64bit",
            "non_power_of_two_strategy": "exact widened signed-int64 coefficients",
            "power_of_two_strategy": "exact divisor fold",
            "overflow_policy": "fail closed before int64 packing"}
