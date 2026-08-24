#!/usr/bin/env python3
"""Focused deterministic checks for the opt-in HM01B0 gate-POC transform."""

from __future__ import annotations

import hashlib
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).parent))
from hm01b0_poc_camera import (  # noqa: E402
    CX_PX, CY_PX, FX_PX, FY_PX, Hm01b0PocCamera, SOURCE_FOVY_DEG,
    SOURCE_HEIGHT, SOURCE_WIDTH, TARGET_HEIGHT, TARGET_WIDTH, calibrated_remap,
)
from vision_bridge import CombinedGateRlAdapter, Prediction  # noqa: E402


def main() -> int:
    map_x, map_y = calibrated_remap()
    assert map_x.shape == (TARGET_HEIGHT, TARGET_WIDTH)
    assert map_y.shape == (TARGET_HEIGHT, TARGET_WIDTH)
    assert float(map_x.min()) >= 0.0 and float(map_x.max()) <= SOURCE_WIDTH - 1.0
    assert float(map_y.min()) >= 0.0 and float(map_y.max()) <= SOURCE_HEIGHT - 1.0
    assert Hm01b0PocCamera(0).remap_sha256() == (
        "1c5590ba3d35c898ff3a6786857e2c42911772a1a520a921a3617a96548be08b")
    # The calibrated optical-axis pixel maps to the centered source optical
    # axis, proving that both principal-point coordinates take effect.
    optical_x, optical_y = int(round(CX_PX)), int(round(CY_PX))
    assert abs(float(map_x[optical_y, optical_x]) - SOURCE_WIDTH / 2.0) < 1.5
    assert abs(float(map_y[optical_y, optical_x]) - SOURCE_HEIGHT / 2.0) < 1.5
    assert 94.0 < SOURCE_FOVY_DEG < 94.1

    source = np.tile(np.arange(SOURCE_WIDTH, dtype=np.uint8), (SOURCE_HEIGHT, 1))
    first = Hm01b0PocCamera(712).transform(source)
    second = Hm01b0PocCamera(712).transform(source)
    assert np.array_equal(first, second)
    assert first.shape == (TARGET_HEIGHT, TARGET_WIDTH)
    assert first.dtype == np.uint8
    assert hashlib.sha256(first.tobytes()).hexdigest() == hashlib.sha256(second.tobytes()).hexdigest()
    # A centered source gradient cannot retain its source-centered optical axis
    # after the target principal-point shift.
    assert int(first[80, 80]) != int(first[73, 81])

    class Recorder:
        def __init__(self, prediction):
            self.prediction = prediction
            self.frames = []
            self.last_input = None
            self.last_input_raw = None

        def predict(self, frame):
            self.frames.append(np.asarray(frame).copy())
            self.last_input = self.frames[-1]
            self.last_input_raw = self.frames[-1]
            return self.prediction

    navigation = Prediction(False, False, True, np.full(4, 6.0),
                            np.zeros(4), np.zeros(4), 0.0, 0.0, False,
                            np.zeros((4, 2)), 0.0, "not_provided")
    gate = Prediction(False, False, True, np.full(4, 6.0), np.zeros(4),
                      np.zeros(4), 0.0, 0.0, True, np.zeros((4, 2)),
                      0.9, "valid")
    combined = object.__new__(CombinedGateRlAdapter)
    combined.obstacle, combined.gate = Recorder(navigation), Recorder(gate)
    combined.last_input = combined.last_input_raw = None
    combined.predict(first)
    assert len(combined.obstacle.frames) == len(combined.gate.frames) == 1
    assert np.array_equal(combined.obstacle.frames[0], combined.gate.frames[0])
    print("HM01B0 POC camera checks passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
