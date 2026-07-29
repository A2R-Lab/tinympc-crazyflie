#!/usr/bin/env python3
"""Host tests for firmware safe-corridor fitting and K^T line planes."""

import ctypes
import subprocess
from pathlib import Path

import numpy as np
import pytest


APP = Path(__file__).resolve().parents[2]
SRC = APP / "src"
CELLS = 100


class Config(ctypes.Structure):
    _fields_ = [
        ("danger_threshold", ctypes.c_float),
        ("inward_margin_px", ctypes.c_float),
        ("fx", ctypes.c_float),
        ("fy", ctypes.c_float),
        ("cx", ctypes.c_float),
        ("cy", ctypes.c_float),
    ]


class Corridor(ctypes.Structure):
    _fields_ = [
        ("valid", ctypes.c_bool),
        ("selected_cells", ctypes.c_int),
        ("pixel_line", (ctypes.c_float * 3) * 2),
        ("camera_normal", (ctypes.c_float * 3) * 2),
    ]


@pytest.fixture(scope="module")
def library(tmp_path_factory):
    output = tmp_path_factory.mktemp("corridor") / "libcorridor.so"
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
            str(SRC / "perception_corridor.c"),
            "-lm",
            "-o",
            str(output),
        ],
        check=True,
    )
    result = ctypes.CDLL(str(output))
    result.perceptionCorridorFit.restype = ctypes.c_bool
    vector = ctypes.c_float * 3
    result.perceptionAngularConstraintRow.argtypes = [
        vector, vector, ctypes.c_float, vector, vector,
        ctypes.POINTER(ctypes.c_float)
    ]
    return result


def fitted(library) -> Corridor:
    danger = (ctypes.c_float * CELLS)(*([1.0] * CELLS))
    # Central six columns form a connected safe vertical corridor.
    for y in range(10):
        for x in range(2, 8):
            danger[y * 10 + x] = 0.0
    corners = (ctypes.c_float * 8)(
        64.0, 56.0, 96.0, 56.0, 96.0, 104.0, 64.0, 104.0
    )
    config = Config(0.5, 4.0, 89.1558392549, 89.4608171623,
                    81.1038105230, 73.3473030288)
    result = Corridor()
    assert library.perceptionCorridorFit(
        danger, corners, ctypes.byref(config), ctypes.byref(result)
    )
    return result


def test_fitted_gate_center_is_on_both_feasible_sides(library) -> None:
    result = fitted(library)
    gate = np.array([80.0, 80.0, 1.0])
    for side in range(2):
        line = np.array(result.pixel_line[side])
        assert line @ gate > 0


def test_k_transpose_line_plane_contains_line_rays(library) -> None:
    result = fitted(library)
    intrinsic = np.array([
        [89.1558392549, 0.0, 81.1038105230],
        [0.0, 89.4608171623, 73.3473030288],
        [0.0, 0.0, 1.0],
    ])
    for side in range(2):
        line = np.array(result.pixel_line[side])
        normal = np.array(result.camera_normal[side])
        # Pick a point exactly on a*l_u+b*l_v+c=0.
        v = 64.0
        u = -(line[1] * v + line[2]) / line[0]
        ray = np.linalg.solve(intrinsic, np.array([u, v, 1.0]))
        assert normal @ ray == pytest.approx(0.0, abs=1e-6)
        reversed_normal = -normal
        gate_ray = np.linalg.solve(intrinsic, np.array([80.0, 80.0, 1.0]))
        assert normal @ gate_ray > 0
        assert reversed_normal @ gate_ray < 0


def test_pixel_margin_shrinks_the_corridor(library) -> None:
    result = fitted(library)
    # Original safe boundaries are x=32 and x=128; four-pixel shrink.
    left = np.array(result.pixel_line[0])
    right = np.array(result.pixel_line[1])
    assert -(left[1] * 80.0 + left[2]) / left[0] == pytest.approx(36.0)
    assert -(right[1] * 80.0 + right[2]) / right[0] == pytest.approx(124.0)


def test_tinympc_row_uses_position_then_velocity_and_rejects_danger_side(
    library,
) -> None:
    vector = ctypes.c_float * 3
    normal = vector(0.0, 1.0, 0.0)
    camera = vector(1.0, 2.0, 3.0)
    a_position, a_velocity = vector(), vector()
    upper = ctypes.c_float()
    library.perceptionAngularConstraintRow(
        normal, camera, 0.25, a_position, a_velocity, ctypes.byref(upper)
    )
    assert list(a_position) == pytest.approx([0.0, -1.0, 0.0])
    assert list(a_velocity) == pytest.approx([0.0, -0.25, 0.0])
    assert upper.value == pytest.approx(-2.0)

    # q=p+tau*v below the camera's safe y half-space violates a^T x <= b.
    p = np.array([1.0, 1.0, 3.0])
    v = np.array([0.0, 0.0, 0.0])
    lhs = np.array(a_position) @ p + np.array(a_velocity) @ v
    assert lhs > upper.value

    # Orthogonal half-space projection moves q toward the safe angular side.
    coefficients = np.r_[np.array(a_position), np.array(a_velocity)]
    state = np.r_[p, v]
    violation = lhs - upper.value
    projected = state - violation * coefficients / (coefficients @ coefficients)
    old_q = p + 0.25 * v
    new_q = projected[:3] + 0.25 * projected[3:]
    assert normal[1] * (new_q[1] - camera[1]) > (
        normal[1] * (old_q[1] - camera[1])
    )
