"""CPU reference for the GAP8 gate-FrontNet + oLGMD1 perception path.

This module intentionally owns no flight-control policy.  It mirrors the
stateful 80 x 80 oLGMD1 front end and the two firmware wire packets so recorded
camera frames and CrazySim can exercise the exact perception/controller ABI.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
import struct
import zlib

import numpy as np


THREAT_HEADER = b"\x90\x19\x08\x40"
GATE_CORNERS_HEADER = b"\x90\x19\x08\x41"
THREAT_BODY = struct.Struct("<4sIHBB")
GATE_CORNERS_BODY = struct.Struct("<4sIHBB8f")


@dataclass(frozen=True)
class OlgmdResult:
    imminent_threat: bool
    warmup: bool
    membrane_potential: float
    ffi_on: float
    ffi_off: float
    spike: bool
    spike_count: int = 0
    accumulated_spikes: int = 0


def downsample_box_2x2(frame: np.ndarray) -> np.ndarray:
    """Convert one 160 x 160 uint8 image to the GAP8 80 x 80 input."""
    image = np.asarray(frame)
    if image.shape != (160, 160) or image.dtype != np.uint8:
        raise ValueError("oLGMD input must be uint8 with shape (160, 160)")
    wide = image.astype(np.uint16)
    # Integer round-to-nearest, shared with the GAP8 implementation.
    return ((wide[0::2, 0::2] + wide[0::2, 1::2] +
             wide[1::2, 0::2] + wide[1::2, 1::2] + 2) // 4).astype(np.uint8)


def resize_vertical_area_160_to_96(frame: np.ndarray) -> np.ndarray:
    """Exact integer form of the bring-up firmware's vertical INTER_AREA."""
    image = np.asarray(frame)
    if image.shape != (160, 160) or image.dtype != np.uint8:
        raise ValueError("gate input must be uint8 with shape (160, 160)")
    source = image.astype(np.uint16)
    output = np.empty((96, 160), dtype=np.uint8)
    for block in range(32):
        rows = source[5 * block:5 * block + 5]
        output[3 * block] = ((3 * rows[0] + 2 * rows[1] + 2) // 5).astype(np.uint8)
        output[3 * block + 1] = (
            (rows[1] + 3 * rows[2] + rows[3] + 2) // 5).astype(np.uint8)
        output[3 * block + 2] = (
            (2 * rows[3] + 3 * rows[4] + 2) // 5).astype(np.uint8)
    return output


def _neighbor_sum(image: np.ndarray, diagonal: bool) -> np.ndarray:
    padded = np.pad(image, 1, mode="constant")
    if diagonal:
        return (padded[:-2, :-2] + padded[:-2, 2:] +
                padded[2:, :-2] + padded[2:, 2:])
    return (padded[:-2, 1:-1] + padded[2:, 1:-1] +
            padded[1:-1, :-2] + padded[1:-1, 2:])


def _mean_3x3(image: np.ndarray) -> np.ndarray:
    padded = np.pad(image, 1, mode="constant")
    total = np.zeros_like(image, dtype=np.float32)
    for row in range(3):
        for column in range(3):
            total += padded[row:row + image.shape[0],
                            column:column + image.shape[1]]
    return total / 9.0


class Olgmd1Reference:
    """Equation-faithful floating reference for the fixed-point GAP8 port.

    Coefficients come from the authors' 30 Hz oLGMD1 implementation.  The
    reference corrects two apparent copy/paste errors in that source: the OFF
    diagonal time constant and use of the OFF channel/weight in ``S_off``.
    Threshold calibration may replace ``spike_threshold`` without changing the
    state or packet ABI.
    """

    def __init__(self, *, fps: float = 30.0, spike_threshold: float = 0.70,
                 collision_window: int = 6, collision_spikes: int = 6):
        if fps <= 0.0 or not math.isfinite(fps):
            raise ValueError("fps must be positive and finite")
        if not 0.5 <= spike_threshold <= 1.0:
            raise ValueError("spike_threshold must lie in [0.5, 1]")
        if not 1 <= collision_window <= 8:
            raise ValueError("collision_window must lie in [1, 8]")
        if collision_spikes < 1:
            raise ValueError("collision_spikes must be positive")
        self.dt_ms = 1000.0 / fps
        self.spike_threshold = float(spike_threshold)
        self.collision_window = int(collision_window)
        self.collision_spikes_required = int(collision_spikes)
        self.reset()

    def reset(self) -> None:
        self.previous_gray = None
        self.previous_on = np.zeros((80, 80), dtype=np.float32)
        self.previous_off = np.zeros((80, 80), dtype=np.float32)
        self.previous_smp = 0.5
        self.previous_output = 0.5
        self.ffi_on = 0.0
        self.ffi_off = 0.0
        self.spike_history: list[int] = []

    def _inhibition(self, current: np.ndarray, previous: np.ndarray,
                    time_constants_ms: tuple[float, float, float]) -> np.ndarray:
        alphas = tuple(self.dt_ms / (self.dt_ms + tau)
                       for tau in time_constants_ms)
        current_adjacent = _neighbor_sum(current, False) * 0.25
        previous_adjacent = _neighbor_sum(previous, False) * 0.25
        current_diagonal = _neighbor_sum(current, True) * 0.125
        previous_diagonal = _neighbor_sum(previous, True) * 0.125
        return (alphas[0] * current + (1.0 - alphas[0]) * previous +
                alphas[1] * current_adjacent +
                (1.0 - alphas[1]) * previous_adjacent +
                alphas[2] * current_diagonal +
                (1.0 - alphas[2]) * previous_diagonal)

    def step(self, frame: np.ndarray) -> OlgmdResult:
        gray = downsample_box_2x2(frame)
        if self.previous_gray is None:
            self.previous_gray = gray
            return OlgmdResult(False, True, 0.5, 0.0, 0.0, False)

        difference = gray.astype(np.int16) - self.previous_gray.astype(np.int16)
        current_on = np.maximum(difference, 0).astype(np.float32)
        current_off = np.maximum(-difference, 0).astype(np.float32)
        current_on += 0.1 * self.previous_on
        current_off += 0.1 * self.previous_off

        ffi_alpha = self.dt_ms / (self.dt_ms + 10.0)
        self.ffi_on = (ffi_alpha * float(np.mean(current_on)) +
                       (1.0 - ffi_alpha) * self.ffi_on)
        self.ffi_off = (ffi_alpha * float(np.mean(current_off)) +
                        (1.0 - ffi_alpha) * self.ffi_off)

        summed = np.zeros((80, 80), dtype=np.float32)
        if self.ffi_on >= 1.0:
            inhibition = self._inhibition(
                current_on, self.previous_on, (15.0, 30.0, 45.0))
            on_weight = max(1.0, self.ffi_on / 8.0)
            summed += np.maximum(current_on - on_weight * inhibition, 0.0)
        if self.ffi_off >= 1.0:
            inhibition = self._inhibition(
                current_off, self.previous_off, (60.0, 120.0, 180.0))
            off_weight = max(0.3, self.ffi_off / 8.0)
            summed += np.maximum(current_off - off_weight * inhibition, 0.0)

        grouped_context = _mean_3x3(summed)
        grouping_scale = 0.01 + float(np.max(grouped_context)) / 4.0
        grouped = summed * grouped_context / grouping_scale
        grouped[grouped < 35.0] = 0.0
        mp = float(np.sum(grouped))
        smp = 1.0 / (1.0 + math.exp(-mp / (80.0 * 80.0 * 0.5)))
        sfa_alpha = 500.0 / (self.dt_ms + 500.0)
        difference_smp = smp - self.previous_smp
        if difference_smp <= 0.003:
            output = sfa_alpha * (self.previous_output + difference_smp)
        else:
            output = sfa_alpha * smp
        output = max(0.5, output)

        spike_count = min(
            8, math.floor(math.exp(4.0 * (output - self.spike_threshold))))
        self.spike_history.append(spike_count)
        self.spike_history = self.spike_history[-self.collision_window:]
        accumulated_spikes = sum(self.spike_history)
        collision = accumulated_spikes >= self.collision_spikes_required

        self.previous_gray = gray
        self.previous_on = current_on
        self.previous_off = current_off
        self.previous_smp = smp
        self.previous_output = output
        return OlgmdResult(
            collision, False, output, self.ffi_on, self.ffi_off,
            spike_count > 0, spike_count, accumulated_spikes)


def threat_packet(sequence: int, timestamp_ms: int, threat: bool) -> bytes:
    if not 1 <= sequence <= 0xFFFF:
        raise ValueError("sequence must lie in [1, 65535]")
    body = THREAT_BODY.pack(
        THREAT_HEADER, timestamp_ms & 0xFFFFFFFF, sequence, int(bool(threat)), 0)
    return body + struct.pack("<I", zlib.crc32(body) & 0xFFFFFFFF)


def gate_corners_packet(sequence: int, timestamp_ms: int,
                        inference_valid: bool, corners: np.ndarray) -> bytes:
    if not 1 <= sequence <= 0xFFFF:
        raise ValueError("sequence must lie in [1, 65535]")
    values = np.asarray(corners, dtype=np.float32).reshape(-1)
    valid = (values.size == 8 and np.all(np.isfinite(values)) and
             np.all(values >= -1.0) and np.all(values <= 2.0))
    if not valid:
        raise ValueError("corners must be eight finite values in [-1, 2]")
    body = GATE_CORNERS_BODY.pack(
        GATE_CORNERS_HEADER, timestamp_ms & 0xFFFFFFFF, sequence,
        int(bool(inference_valid)), 0, *values)
    return body + struct.pack("<I", zlib.crc32(body) & 0xFFFFFFFF)
