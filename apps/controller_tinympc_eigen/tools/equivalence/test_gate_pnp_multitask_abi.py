#!/usr/bin/env python3
"""Regression test for the native-160 TL/TR/BR/BL corner ABI."""

import subprocess
from pathlib import Path

import pytest


APP = Path(__file__).resolve().parents[2]
SRC = APP / "src"


def test_native_160_raster_corner_order(tmp_path: Path) -> None:
    harness = tmp_path / "gate_pnp_harness.c"
    harness.write_text(
        r'''
#include "gate_pnp.h"
#include <stdio.h>

int main(void) {
  /* TL, TR, BR, BL: 40x40 pixels centered near the calibrated principal point. */
  const float corners[8] = {61.0f, 53.0f, 101.0f, 53.0f,
                            101.0f, 93.0f, 61.0f, 93.0f};
  const DroneState state = {
    .x = 0.0f, .y = 0.0f, .z = 0.0f,
    .qx = 0.0f, .qy = 0.0f, .qz = 0.0f, .qw = 1.0f
  };
  GateVisionPacket packet;
  const int valid = gate_pnp_project(corners, 0, &state, &packet);
  printf("%d %.6f %.6f %.6f\n", valid, g_gate_dbg_width,
         g_gate_dbg_height, g_gate_dbg_range);
  return 0;
}
'''
    )
    executable = tmp_path / "gate_pnp_harness"
    subprocess.run(
        [
            "gcc",
            "-std=c11",
            "-Wall",
            "-Wextra",
            "-Werror",
            "-I",
            str(SRC),
            str(harness),
            str(SRC / "gate_pnp.c"),
            "-lm",
            "-o",
            str(executable),
        ],
        check=True,
    )
    values = subprocess.check_output([str(executable)], text=True).split()
    valid, width, height, range_m = (
        int(values[0]),
        float(values[1]),
        float(values[2]),
        float(values[3]),
    )
    assert valid == 1
    assert width == pytest.approx(40.0)
    assert height == pytest.approx(40.0)
    expected = 0.5 * (
        89.1558392549 * 0.4826 / 40.0
        + 89.4608171623 * 0.4826 / 40.0
    )
    assert range_m == pytest.approx(expected, rel=1e-5)
