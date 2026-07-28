#!/usr/bin/env python3
"""Host equivalence tests for STM32 motion-conditioned danger decoding."""

import ctypes
import math
import subprocess
from pathlib import Path

import pytest


APP = Path(__file__).resolve().parents[2]
SRC = APP / "src"
CELLS = 100
U8_MAP = ctypes.c_uint8 * CELLS
F32_MAP = ctypes.c_float * CELLS


class State(ctypes.Structure):
    _fields_ = [
        ("body_velocity_mps", ctypes.c_float * 3),
        ("horizon_s", ctypes.c_float),
        ("perception_control_latency_s", ctypes.c_float),
        ("maximum_range_m", ctypes.c_float),
        ("nominal_target_speed_mps", ctypes.c_float),
    ]


class Output(ctypes.Structure):
    _fields_ = [
        ("probability", F32_MAP),
        ("time_to_contact_s", F32_MAP),
        ("range_m", F32_MAP),
        ("uncertainty", F32_MAP),
    ]


@pytest.fixture(scope="module")
def library(tmp_path_factory):
    output = tmp_path_factory.mktemp("perception-danger") / "libdanger.so"
    subprocess.run(
        [
            "gcc",
            "-std=c11",
            "-shared",
            "-fPIC",
            "-Wall",
            "-Wextra",
            "-Werror",
            "-I",
            str(SRC),
            str(SRC / "perception_danger.c"),
            "-lm",
            "-o",
            str(output),
        ],
        check=True,
    )
    result = ctypes.CDLL(str(output))
    result.perceptionDangerCompute.argtypes = [
        ctypes.POINTER(ctypes.c_uint8),
        ctypes.POINTER(ctypes.c_uint8),
        ctypes.POINTER(ctypes.c_uint8),
        ctypes.c_uint32,
        ctypes.POINTER(State),
        ctypes.POINTER(Output),
    ]
    result.perceptionDangerApplyGateOpening.argtypes = [
        ctypes.POINTER(ctypes.c_uint8),
        ctypes.POINTER(ctypes.c_float),
        ctypes.c_float,
        ctypes.c_float,
        ctypes.c_float,
        ctypes.c_float,
        ctypes.c_float,
        ctypes.c_float,
        ctypes.POINTER(Output),
    ]
    result.perceptionDangerApplyGateOpening.restype = ctypes.c_int
    return result


def compute(library, speed: float, age_ms: int = 0) -> Output:
    # Chosen against the packaged model affine constants to represent a
    # non-saturated, roughly 3 m obstacle. Saturated q=255 means immediate
    # collision and cannot demonstrate motion conditioning.
    presence = U8_MAP(*([63] * CELLS))
    inverse_range = U8_MAP(*([30] * CELLS))
    uncertainty = U8_MAP(*([20] * CELLS))
    state = State((ctypes.c_float * 3)(speed, 0.0, 0.0), 1.0, 0.08, 6.0, 1.0)
    output = Output()
    library.perceptionDangerCompute(
        presence,
        inverse_range,
        uncertainty,
        age_ms,
        ctypes.byref(state),
        ctypes.byref(output),
    )
    return output


def test_same_image_map_is_more_dangerous_at_high_speed(library) -> None:
    slow = compute(library, 0.5)
    fast = compute(library, 5.0)
    assert fast.probability[0] > slow.probability[0]
    assert fast.time_to_contact_s[0] < slow.time_to_contact_s[0]


def test_map_age_is_included_in_latency(library) -> None:
    fresh = compute(library, 2.0, age_ms=0)
    stale = compute(library, 2.0, age_ms=200)
    assert stale.time_to_contact_s[0] < fresh.time_to_contact_s[0]
    assert stale.probability[0] > fresh.probability[0]


def test_stationary_time_to_contact_is_infinite(library) -> None:
    stationary = compute(library, 0.0)
    assert math.isinf(stationary.time_to_contact_s[0])
    assert 0.0 <= stationary.probability[0] <= 1.0


def _gate_opening_case(library, center_range: float) -> tuple[Output, int]:
    gate = U8_MAP(*([255] * CELLS))
    corners = (ctypes.c_float * 8)(
        32.0, 32.0, 128.0, 32.0, 128.0, 128.0, 32.0, 128.0
    )
    output = Output()
    for cell in range(CELLS):
        output.probability[cell] = 0.9
        output.range_m[cell] = 3.0
        output.uncertainty[cell] = 0.1
    output.range_m[5 * 10 + 5] = center_range
    changed = library.perceptionDangerApplyGateOpening(
        gate, corners, 2.0, 8.0, 0.5, 0.2, 0.15, 1.0,
        ctypes.byref(output),
    )
    return output, changed


def test_confident_gate_opening_reduces_only_inset_interior(library) -> None:
    output, changed = _gate_opening_case(library, center_range=3.0)
    assert changed > 0
    assert output.probability[5 * 10 + 5] == pytest.approx(0.2)
    assert output.probability[0] == pytest.approx(0.9)


def test_near_obstacle_inside_gate_opening_is_not_suppressed(library) -> None:
    output, _ = _gate_opening_case(library, center_range=1.5)
    assert output.probability[5 * 10 + 5] == pytest.approx(0.9)
