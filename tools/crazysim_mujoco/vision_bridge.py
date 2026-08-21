#!/usr/bin/env python3
"""Run a repository-local ONNX vision head on CrazySim FPV frames.

The bridge deliberately carries metric clearance, collision risk, navigation,
and gate geometry as different signals. It sends the same versioned packet
that the AI deck can send to the STM32 firmware.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import signal
import socket
import struct
import time
import zlib
from dataclasses import dataclass
from pathlib import Path

import numpy as np


CAMERA_HEADER = struct.Struct("<HHHH")
VISION_BODY = struct.Struct("<4sIHH4f4f4f2f8f1f4f")
VISION_MAGIC = b"\x90\x19\x08\x3a"
LEGACY_GATE_INTRINSICS = (
    89.15584 / 160.0, 89.46082 / 120.0, 0.5, 0.5)
FLAG_METRIC = 1 << 0
FLAG_DANGER = 1 << 1
FLAG_GATE = 1 << 2
FLAG_NAVIGATION = 1 << 3
SECTOR_BEARING = np.asarray([1.0, 1.0 / 3.0, -1.0 / 3.0, -1.0])


class CameraFrameAssembler:
    """Reassemble CrazySim grayscale camera chunks into one image.

    CrazySim's camera protocol has no frame sequence number, but it sends all
    chunks for one frame consecutively and always starts a frame with chunk 0.
    A new chunk 0 therefore safely discards any partially received old frame.
    """

    def __init__(self):
        self._key = None
        self._parts = None

    def feed(self, datagram: bytes) -> np.ndarray | None:
        if len(datagram) < CAMERA_HEADER.size:
            return None
        chunk, total, width, height = CAMERA_HEADER.unpack_from(datagram)
        if total < 1 or total > 16 or chunk >= total or width < 1 or height < 1:
            return None
        key = (total, width, height)
        if chunk == 0:
            self._key = key
            self._parts = [None] * total
        elif self._key != key or self._parts is None:
            return None
        self._parts[chunk] = datagram[CAMERA_HEADER.size:]
        if any(part is None for part in self._parts):
            return None
        payload = b"".join(self._parts)
        self._key = None
        self._parts = None
        pixels = np.frombuffer(payload, dtype=np.uint8)
        if pixels.size != width * height:
            return None
        return pixels.reshape(height, width)


@dataclass
class Prediction:
    metric: bool
    sector_danger: bool
    navigation: bool
    clearance: np.ndarray
    confidence: np.ndarray
    danger: np.ndarray
    steering: float
    collision: float
    gate_valid: bool
    corners: np.ndarray
    gate_confidence: float
    gate_reason: str
    raw_danger: np.ndarray | None = None
    danger_threshold: float = math.nan
    # fx/width, fy/height, cx/width, cy/height for gate projection.
    gate_intrinsics: tuple[float, float, float, float] = LEGACY_GATE_INTRINSICS


class FixedFrameLatencyQueue:
    """Release inference results after an exact number of camera frames.

    The queue is indexed by camera sequence rather than host time, so a faster
    or slower ONNX call cannot change the simulated sensor-to-controller delay.
    """

    def __init__(self, latency_frames: int):
        if latency_frames < 0:
            raise ValueError("latency_frames must be nonnegative")
        self.latency_frames = latency_frames
        self._pending = []

    def push(self, sequence: int, timestamp_ms: int, prediction: Prediction):
        self._pending.append((sequence, timestamp_ms, prediction))
        if len(self._pending) <= self.latency_frames:
            return None
        return self._pending.pop(0)


def sigmoid(value):
    return 1.0 / (1.0 + np.exp(-np.clip(value, -60.0, 60.0)))


def cross2d(first: np.ndarray, second: np.ndarray) -> float:
    return float(first[0] * second[1] - first[1] * second[0])


def resize_nearest(frame: np.ndarray, height: int, width: int) -> np.ndarray:
    ys = np.minimum((np.arange(height) * frame.shape[0] / height).astype(int),
                    frame.shape[0] - 1)
    xs = np.minimum((np.arange(width) * frame.shape[1] / width).astype(int),
                    frame.shape[1] - 1)
    return frame[np.ix_(ys, xs)]


def bottom_center_crop(frame: np.ndarray, height: int, width: int) -> np.ndarray:
    """Match the deployed DroNet 324x244 Himax -> 200x200 crop."""
    if frame.ndim != 2:
        raise ValueError(f"DroNet expects one grayscale plane, received {frame.shape}")
    if frame.shape[0] < height or frame.shape[1] < width:
        raise ValueError(
            f"DroNet cannot crop {height}x{width} from camera frame {frame.shape}")
    left = (frame.shape[1] - width) // 2
    top = frame.shape[0] - height
    return np.ascontiguousarray(frame[top:top + height, left:left + width])


def hm01b0_center_crop(frame: np.ndarray) -> np.ndarray:
    """Apply the deployed Tiny Racer 160x160 -> 160x120 center crop."""
    if frame.ndim != 2:
        raise ValueError(
            f"Tiny Racer models expect one grayscale plane, received {frame.shape}")
    if frame.shape == (120, 160):
        return np.ascontiguousarray(frame)
    if frame.shape == (160, 160):
        return np.ascontiguousarray(frame[20:140, :])
    raise ValueError(
        "Tiny Racer models require a 160x120 crop or 160x160 HM01B0 frame; "
        f"received {frame.shape[1]}x{frame.shape[0]}")


def hm01b0_full_frame(frame: np.ndarray) -> np.ndarray:
    """Require the complete frame used by the two-frame ESPNet release."""
    if frame.ndim != 2 or frame.shape != (160, 160):
        raise ValueError(
            "two-frame ESPNet requires a complete 160x160 HM01B0 frame; "
            f"received {frame.shape}"
        )
    return np.ascontiguousarray(frame)


def file_sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def navigation_from_sectors(danger: np.ndarray, previous: float) -> tuple[float, float]:
    danger = np.clip(np.asarray(danger, dtype=float), 0.0, 1.0)
    collision = float(np.max(danger))
    steering = float(np.dot(1.0 - danger, SECTOR_BEARING) /
                     max(np.sum(1.0 - danger), 1e-6))
    # A centered obstacle is an ambiguous tie. Keep the previous chosen side
    # instead of alternating on inference noise; start with a left pass.
    if collision >= 0.25 and abs(steering) < 0.08:
        steering = math.copysign(0.60, previous if abs(previous) > 0.08 else 1.0)
    return float(np.clip(steering, -1.0, 1.0)), collision


def validate_gate(corners: np.ndarray) -> tuple[bool, str]:
    points = np.asarray(corners, dtype=float).reshape(4, 2)
    if not np.all(np.isfinite(points)) or np.any(points < 0.0) or np.any(points > 1.0):
        return False, "bounds"
    tl, tr, br, bl = points
    if not (tl[0] < tr[0] and bl[0] < br[0] and tl[1] < bl[1] and tr[1] < br[1]):
        return False, "ordering"
    cross = []
    for index in range(4):
        a, b, c = points[index], points[(index + 1) % 4], points[(index + 2) % 4]
        cross.append(cross2d(b - a, c - b))
    if not (np.all(np.asarray(cross) > 0.0) or np.all(np.asarray(cross) < 0.0)):
        return False, "nonconvex"
    area = 0.5 * abs(np.dot(points[:, 0], np.roll(points[:, 1], -1)) -
                     np.dot(points[:, 1], np.roll(points[:, 0], -1)))
    width = 0.5 * ((tr[0] - tl[0]) + (br[0] - bl[0]))
    height = 0.5 * ((bl[1] - tl[1]) + (br[1] - tr[1]))
    if area < 0.005 or area > 0.90:
        return False, "area"
    if height <= 0.0 or not 0.35 <= width / height <= 2.85:
        return False, "aspect"
    return True, "accepted"


def _polygon_geometry_reason(
        points: np.ndarray, minimum_area: float,
        maximum_side_ratio: float) -> str:
    """Mirror NanoCockpit's deployed ordered-quadrilateral checks."""
    points = np.asarray(points, dtype=np.float32).reshape(4, 2)
    cross = []
    for edge in range(4):
        first = points[edge]
        second = points[(edge + 1) & 3]
        third = points[(edge + 2) & 3]
        cross.append(cross2d(second - first, third - second))
    if cross[0] == 0.0 or any(value * cross[0] <= 0.0 for value in cross):
        return "quad_convexity"
    area = 0.5 * abs(float(
        np.dot(points[:, 0], np.roll(points[:, 1], -1)) -
        np.dot(points[:, 1], np.roll(points[:, 0], -1))))
    if area < minimum_area:
        return "quad_area"
    sides = np.linalg.norm(points - np.roll(points, -1, axis=0), axis=1)
    if (float(np.min(sides)) <= 0.0 or
            float(np.max(sides) / np.min(sides)) > maximum_side_ratio):
        return "quad_ratio"
    return "accepted"


def validate_sequential_gate(
        corners: np.ndarray, peaks: np.ndarray,
        ambiguity: np.ndarray) -> tuple[bool, str]:
    """Match the gate policy in the deployed NanoCockpit decoder."""
    corners = np.asarray(corners, dtype=np.float32).reshape(4, 2)
    peaks = np.asarray(peaks, dtype=np.float32)
    ambiguity = np.asarray(ambiguity, dtype=np.float32)
    confident = (peaks >= -0.5) & (ambiguity >= 0.06)
    confident_indices = np.flatnonzero(confident)
    if confident_indices.size < 3:
        geometry = _polygon_geometry_reason(corners, 400.0, 4.0)
        rescued = bool(np.all(peaks >= -1.0) and geometry == "accepted")
        return rescued, "accepted_geometry" if rescued else "confidence"
    if confident_indices.size == 3:
        triangle = corners[confident_indices]
        cross = cross2d(
            triangle[1] - triangle[0], triangle[2] - triangle[0])
        if 0.5 * abs(cross) < 50.0:
            return False, "triangle_area"
        sides = np.linalg.norm(
            triangle - np.roll(triangle, -1, axis=0), axis=1)
        if (float(np.min(sides)) <= 0.0 or
                float(np.max(sides) / np.min(sides)) > 8.0):
            return False, "triangle_ratio"
        return True, "accepted_three_corners"
    reason = _polygon_geometry_reason(corners, 100.0, 6.0)
    return reason == "accepted", reason


class SequentialAdapter:
    def __init__(self, model_path: Path, clearance_threshold: float):
        import onnxruntime as ort
        if model_path.is_dir():
            model_path = model_path / "sequential_int.onnx"
        if not model_path.is_file():
            raise FileNotFoundError(model_path)
        self.session = ort.InferenceSession(str(model_path), providers=["CPUExecutionProvider"])
        self.input_name = self.session.get_inputs()[0].name
        if self.session.get_inputs()[0].shape != [1, 1, 120, 160]:
            raise ValueError("sequential model input must be [1,1,120,160]")
        if self.session.get_outputs()[0].shape != [1, 12, 15, 20]:
            raise ValueError("sequential model output must be [1,12,15,20]")
        manifest_path = model_path.with_name("quantization_manifest.json")
        if not manifest_path.is_file():
            raise FileNotFoundError(f"missing {manifest_path}")
        manifest = json.loads(manifest_path.read_text())
        self.scale = float(manifest["scale"])
        if manifest.get("shape") != [1, 12, 15, 20]:
            raise ValueError(
                "sequential quantization manifest shape does not match the model")
        bundle_path = model_path.with_name("bundle_manifest.json")
        if bundle_path.is_file():
            expected = json.loads(bundle_path.read_text())["model_sha256"]
            actual = file_sha256(model_path)
            if actual != expected:
                raise ValueError(f"sequential model checksum mismatch: {actual}")
        self.threshold = clearance_threshold
        self.previous_steering = 0.0
        self.last_input = None

    def predict(self, frame: np.ndarray) -> Prediction:
        crop = hm01b0_center_crop(frame)
        self.last_input = crop
        raw = np.asarray(self.session.run(
            None, {self.input_name: crop.astype(np.float32)[None, None]})[0])
        if raw.shape != (1, 12, 15, 20):
            raise ValueError(f"sequential-v1 expects [1,12,15,20], received {raw.shape}")
        logical = raw[0].astype(np.float32) * self.scale - 6.0
        clearance = (np.clip(logical[4:8].mean((1, 2)), -6.0, 6.0) + 6.0) * 0.5
        confidence = logical[8:12].mean((1, 2))
        danger = sigmoid((self.threshold - clearance) / 0.05)
        steering, collision = navigation_from_sectors(danger, self.previous_steering)
        self.previous_steering = steering
        corners_px = []
        corner_peaks = []
        corner_ambiguity = []
        for heatmap in logical[:4]:
            y, x = np.unravel_index(np.argmax(heatmap), heatmap.shape)
            competing = heatmap.copy()
            competing[max(0, y - 1):y + 2,
                      max(0, x - 1):x + 2] = -np.inf
            corners_px.append((8.0 * (x + 0.5) - 0.5,
                               8.0 * (y + 0.5) - 0.5))
            corner_peaks.append(float(heatmap[y, x]))
            corner_ambiguity.append(
                float(heatmap[y, x] - np.max(competing)))
        corners_px = np.asarray(corners_px, dtype=np.float32)
        gate_valid, reason = validate_sequential_gate(
            corners_px, np.asarray(corner_peaks),
            np.asarray(corner_ambiguity))
        corners = corners_px / np.asarray([160.0, 120.0], dtype=np.float32)
        gate_confidence = float(np.mean(sigmoid(np.asarray(corner_peaks))))
        # The deployed 48-byte UART packet carries metric clearances and the
        # gate-valid bit, but no DroNet command or dense danger-map claim.
        return Prediction(True, False, False, clearance, confidence, danger,
                          steering, collision,
                          gate_valid, corners, gate_confidence, reason)


class StdcAdapter:
    CORNER_Q_THRESHOLDS = np.asarray([136, 141, 187, 131], dtype=np.float32)

    def __init__(self, release: Path):
        import onnxruntime as ort
        integer = release / "integer" if (release / "integer").is_dir() else release
        manifest_path = integer.parent / "nanocockpit" / "manifest.json"
        if not manifest_path.is_file():
            raise FileNotFoundError(f"missing {manifest_path}")
        self.manifest = json.loads(manifest_path.read_text())
        self.sessions = {
            name: ort.InferenceSession(str(integer / f"{name}_int.onnx"),
                                       providers=["CPUExecutionProvider"])
            for name in ("encoder", "corner_head", "danger_head")
        }
        expected_io = {
            "encoder": ([1, 1, 120, 160], [1, 32, 30, 40]),
            "corner_head": ([1, 32, 30, 40], [1, 4, 30, 40]),
            "danger_head": ([1, 32, 30, 40], [1, 1, 8, 10]),
        }
        for name, (input_shape, output_shape) in expected_io.items():
            session = self.sessions[name]
            if (session.get_inputs()[0].shape != input_shape or
                    session.get_outputs()[0].shape != output_shape):
                raise ValueError(f"unexpected {name} ONNX I/O")
        bundle_path = integer.parent / "bundle_manifest.json"
        if bundle_path.is_file():
            expected_hashes = json.loads(bundle_path.read_text())["model_sha256"]
            for name, expected in expected_hashes.items():
                actual = file_sha256(integer / f"{name}_int.onnx")
                if actual != expected:
                    raise ValueError(f"{name} model checksum mismatch: {actual}")
        self.previous_steering = 0.0
        self.last_input = None

    def run(self, name: str, values: np.ndarray) -> np.ndarray:
        session = self.sessions[name]
        return session.run(None, {session.get_inputs()[0].name: values})[0]

    def probability(self, values: np.ndarray, name: str) -> np.ndarray:
        affine = self.manifest["integer_affine"][name]
        offset = np.asarray(affine["offset"], dtype=np.float32)
        bias = np.asarray(affine["learned_bias"], dtype=np.float32)
        logits = values * float(affine["epsilon"])
        if values.ndim == 3:
            logits = logits - offset[:, None, None] + bias[:, None, None]
        else:
            logits = logits - offset[0] + bias[0]
        return sigmoid(logits)

    def predict(self, frame: np.ndarray) -> Prediction:
        crop = hm01b0_center_crop(frame)
        self.last_input = crop
        encoded = self.run("encoder", crop.astype(np.float32)[None, None])
        corner_q = np.clip(np.rint(self.run("corner_head", encoded)[0]), 0, 255)
        danger_q = np.clip(np.rint(self.run("danger_head", encoded)[0, 0]), 0, 255)
        corner_probability = self.probability(corner_q, "corner")
        danger_map = self.probability(danger_q, "danger")
        # Four symmetric image regions, ordered outer-left through outer-right.
        # np.array_split(10, 4) is asymmetric (3,3,2,2), so use 3,2,2,3.
        danger = np.asarray([
            np.max(danger_map[:, 0:3]), np.max(danger_map[:, 3:5]),
            np.max(danger_map[:, 5:7]), np.max(danger_map[:, 7:10]),
        ])
        steering, collision = navigation_from_sectors(danger, self.previous_steering)
        self.previous_steering = steering
        corners_px = []
        scores = []
        confident = np.max(corner_q, axis=(1, 2)) >= self.CORNER_Q_THRESHOLDS
        for heatmap in corner_probability:
            y, x = np.unravel_index(np.argmax(heatmap), heatmap.shape)
            corners_px.append(((x + 0.5) * 4.0, (y + 0.5) * 4.0))
            scores.append(float(heatmap[y, x]))
        corners_px = np.asarray(corners_px, dtype=np.float32)
        observed_corners_px = corners_px.copy()
        reason = "confidence"
        if int(np.sum(confident)) == 3:
            missing = int(np.flatnonzero(~confident)[0])
            recovered = (corners_px[(missing + 3) & 3] +
                         corners_px[(missing + 1) & 3] -
                         corners_px[(missing + 2) & 3])
            if (0.0 <= recovered[0] < 160.0 and
                    0.0 <= recovered[1] < 120.0):
                corners_px[missing] = recovered
                reason = "accepted_three_corners"
            else:
                reason = "recovered_out_of_bounds"
        elif int(np.sum(confident)) >= 4:
            reason = "accepted"
        if int(np.sum(confident)) >= 3 and reason.startswith("accepted"):
            tl, tr, br, bl = corners_px
            ordered = bool(tl[0] < tr[0] and bl[0] < br[0] and
                           tl[1] < bl[1] and tr[1] < br[1])
            geometry = _polygon_geometry_reason(
                corners_px, 128.0, float("inf"))
            area = 0.5 * abs(float(
                np.dot(corners_px[:, 0], np.roll(corners_px[:, 1], -1)) -
                np.dot(corners_px[:, 1], np.roll(corners_px[:, 0], -1))))
            width = 0.5 * ((tr[0] - tl[0]) + (br[0] - bl[0]))
            height = 0.5 * ((bl[1] - tl[1]) + (br[1] - tr[1]))
            if not ordered:
                reason = "ordering"
            elif geometry == "quad_convexity":
                reason = "nonconvex"
            elif geometry == "quad_area" or area > 23000.0:
                reason = "area"
            elif (height <= 0.0 or
                  not 0.35 <= float(width / height) <= 2.85):
                reason = "aspect"
        gate_valid = reason.startswith("accepted")
        if not gate_valid:
            # NanoCockpit only publishes a reconstructed corner after every
            # post-completion geometry check passes.
            corners_px = observed_corners_px
        corners = corners_px / np.asarray([160.0, 120.0], dtype=np.float32)
        gate_confidence = float(np.mean(scores))
        return Prediction(False, True, False, np.full(4, 6.0), np.zeros(4), danger,
                          steering, collision, gate_valid, corners,
                          gate_confidence, reason)


class EspnetAdapter:
    """Newest two-frame Tiny Racer ESPNet float-release adapter.

    The final NanoCockpit graph is a hybrid integer GAP8 package. Upstream
    publishes generated C for that graph, but no portable integer ONNX. This
    adapter therefore runs the coordinated release's float student and keeps
    its threshold distinct from the hardware threshold in the bundle metadata.
    """

    def __init__(self, release: Path):
        import onnxruntime as ort

        if release.is_file():
            model_path = release
            release = release.parent
        else:
            model_path = release / "espnet_two_frame_float.onnx"
        bundle_path = release / "bundle_manifest.json"
        hardware_path = release / "nanocockpit_manifest.json"
        float_metrics_path = release / "metrics/float_student.json"
        for required in (model_path, bundle_path, hardware_path,
                         float_metrics_path):
            if not required.is_file():
                raise FileNotFoundError(required)

        self.bundle = json.loads(bundle_path.read_text())
        self.hardware = json.loads(hardware_path.read_text())
        self.float_metrics = json.loads(float_metrics_path.read_text())
        expected = self.bundle["files"]["portable_model"]["sha256"]
        actual = file_sha256(model_path)
        if actual != expected:
            raise ValueError(f"ESPNet model checksum mismatch: {actual}")

        self.session = ort.InferenceSession(
            str(model_path), providers=["CPUExecutionProvider"])
        model_input = self.session.get_inputs()[0]
        expected_outputs = {
            "corner_logits": [1, 4, 40, 40],
            "gate_logits": [1, 1, 40, 40],
            "danger_logits": [1, 1, 10, 10],
        }
        if model_input.shape != [1, 2, 160, 160]:
            raise ValueError("two-frame ESPNet input must be [1,2,160,160]")
        if {output.name: output.shape for output in self.session.get_outputs()} != \
                expected_outputs:
            raise ValueError("unexpected two-frame ESPNet output contract")
        self.input_name = model_input.name

        self.danger_threshold = float(
            self.float_metrics["obstacle_test"]["collision_threshold"])
        corner_affine = self.hardware["integer_affine"]["corner"]
        corner_q = np.asarray(
            self.hardware["thresholds"]["corner_uint8"], dtype=np.float32)
        corner_logits = (
            corner_q * float(corner_affine["output_epsilon"])
            - np.asarray(corner_affine["output_offset"], dtype=np.float32)
            + np.asarray(corner_affine["learned_bias"], dtype=np.float32)
        )
        self.corner_peak_thresholds = sigmoid(corner_logits)
        self.gate_threshold = float(
            self.hardware["thresholds"]["gate_probability"])
        self.previous_frame = None
        self.previous_steering = 0.0
        self.last_input = None
        self.last_input_raw = None

    @staticmethod
    def _gate_geometry(corners_px: np.ndarray) -> tuple[bool, str]:
        tl, tr, br, bl = corners_px
        if not (tl[0] < tr[0] and bl[0] < br[0] and
                tl[1] < bl[1] and tr[1] < br[1]):
            return False, "ordering"
        geometry = _polygon_geometry_reason(
            corners_px, 128.0, float("inf"))
        if geometry == "quad_convexity":
            return False, "nonconvex"
        area = 0.5 * abs(float(
            np.dot(corners_px[:, 0], np.roll(corners_px[:, 1], -1)) -
            np.dot(corners_px[:, 1], np.roll(corners_px[:, 0], -1))))
        if geometry == "quad_area" or area > 23000.0:
            return False, "area"
        width = 0.5 * ((tr[0] - tl[0]) + (br[0] - bl[0]))
        height = 0.5 * ((bl[1] - tl[1]) + (br[1] - tr[1]))
        if height <= 0.0 or not 0.35 <= float(width / height) <= 2.85:
            return False, "aspect"
        return True, "accepted"

    def _decode_gate(
            self, corner_probability: np.ndarray,
            gate_probability: np.ndarray) -> tuple[bool, np.ndarray, float, str]:
        corners_px = []
        peaks = []
        for heatmap in corner_probability:
            y, x = np.unravel_index(np.argmax(heatmap), heatmap.shape)
            corners_px.append(((x + 0.5) * 4.0, (y + 0.5) * 4.0))
            peaks.append(float(heatmap[y, x]))
        corners_px = np.asarray(corners_px, dtype=np.float32)
        observed = corners_px.copy()
        peaks = np.asarray(peaks, dtype=np.float32)
        confident = peaks >= self.corner_peak_thresholds
        confident_count = int(np.sum(confident))
        reason = "confidence"
        if confident_count == 3:
            missing = int(np.flatnonzero(~confident)[0])
            recovered = (corners_px[(missing + 3) & 3] +
                         corners_px[(missing + 1) & 3] -
                         corners_px[(missing + 2) & 3])
            if np.all((recovered >= 0.0) & (recovered < 160.0)):
                corners_px[missing] = recovered
                reason = "accepted_three_corners"
            else:
                reason = "recovered_out_of_bounds"
        elif confident_count == 4:
            reason = "accepted"

        if reason.startswith("accepted"):
            valid, geometry_reason = self._gate_geometry(corners_px)
            if not valid:
                reason = geometry_reason
        else:
            valid = False

        gate_coverage = 0.0
        if valid:
            # Mirror gap8_pool_control_maps: require all four fine mask pixels
            # in a 2x2 block and retain only cells inside an inset quadrilateral.
            center = np.mean(corners_px, axis=0)
            inset = 0.86 * corners_px + 0.14 * center
            sign = cross2d(inset[1] - inset[0], inset[2] - inset[0])
            inside_count = 0
            open_count = 0
            for y in range(20):
                for x in range(20):
                    point = np.asarray(((x + 0.5) * 8.0,
                                        (y + 0.5) * 8.0))
                    inside = all(
                        cross2d(inset[(edge + 1) & 3] - inset[edge],
                                point - inset[edge]) * sign >= 0.0
                        for edge in range(4)
                    )
                    if not inside:
                        continue
                    inside_count += 1
                    block = gate_probability[2 * y:2 * y + 2,
                                             2 * x:2 * x + 2]
                    open_count += int(np.all(block >= self.gate_threshold))
            gate_coverage = open_count / max(inside_count, 1)
            if open_count == 0:
                valid = False
                reason = "gate_mask"
                corners_px = observed
        else:
            corners_px = observed

        return (valid, corners_px / 160.0,
                float(gate_coverage), reason)

    def predict_pair(
            self, previous_frame: np.ndarray,
            current_frame: np.ndarray) -> Prediction:
        """Run the float student on one explicit hardware-ordered frame pair.

        This does not mutate temporal history or steering state, which makes it
        suitable for exact comparison with a captured GAP8 HWC uint8 tensor.
        """
        previous = hm01b0_full_frame(previous_frame)
        current = hm01b0_full_frame(current_frame)
        frames = np.stack((previous, current), axis=0).astype(np.float32) / 255.0
        self.last_input = current
        self.last_input_raw = np.stack((previous, current), axis=-1)
        corner_logits, gate_logits, danger_logits = self.session.run(
            None, {self.input_name: frames[None]})
        corner_probability = sigmoid(corner_logits[0])
        gate_probability = sigmoid(gate_logits[0, 0])
        danger_probability = sigmoid(danger_logits[0, 0])

        slices = ((0, 3), (3, 5), (5, 7), (7, 10))
        raw_sector_danger = np.asarray([
            np.max(danger_probability[:, first:last])
            for first, last in slices
        ], dtype=np.float32)
        # NanoCockpit's postprocessor thresholds the dense map before handing
        # it to control. Preserve that binary control-domain contract here.
        obstacle_map = danger_probability >= self.danger_threshold
        danger = np.asarray([
            float(np.any(obstacle_map[:, first:last]))
            for first, last in slices
        ], dtype=np.float32)
        steering, _ = navigation_from_sectors(danger, self.previous_steering)
        collision = float(np.count_nonzero(danger >= 0.5) >= 2)
        gate_valid, corners, gate_confidence, reason = self._decode_gate(
            corner_probability, gate_probability)
        return Prediction(
            False, True, False, np.full(4, 6.0), np.zeros(4), danger,
            steering, collision, gate_valid, corners, gate_confidence, reason,
            raw_danger=raw_sector_danger,
            danger_threshold=self.danger_threshold,
            gate_intrinsics=(
                float(self.bundle["camera"]["fx_px"]) / 160.0,
                float(self.bundle["camera"]["fy_px"]) / 160.0,
                float(self.bundle["camera"]["cx_px"]) / 160.0,
                float(self.bundle["camera"]["cy_px"]) / 160.0,
            ),
        )

    def predict(self, frame: np.ndarray) -> Prediction:
        current = hm01b0_full_frame(frame)
        previous = current if self.previous_frame is None else self.previous_frame
        prediction = self.predict_pair(previous, current)
        self.previous_frame = current.copy()
        self.previous_steering = prediction.steering
        return prediction


class EspnetDronetGateAdapter:
    """Portable float adapter for perception release 193fa12.

    The model directly supplies DroNet-compatible yaw/collision navigation and
    a separate gate branch. Unlike the earlier ESPNet candidate, it does not
    claim spatial danger sectors or metric clearance.
    """

    def __init__(self, model_path: Path):
        import onnxruntime as ort

        self.session = ort.InferenceSession(
            str(model_path), providers=["CPUExecutionProvider"])
        model_input = self.session.get_inputs()[0]
        outputs = {output.name: output.shape for output in self.session.get_outputs()}
        expected = {
            "corner_heatmaps": ["batch", 4, 20, 20],
            "gate_mask_logits": ["batch", 1, 20, 20],
            "gate_presence_logit": ["batch"],
            "navigation_logits": ["batch", 2],
        }
        if list(model_input.shape[1:]) != [2, 160, 160] or outputs != expected:
            raise ValueError("unexpected ESPNet DroNet/gate ONNX contract")
        self.input_name = model_input.name
        self.previous_frame = None
        self.last_input = None
        self.last_input_raw = None

    @staticmethod
    def _decode_gate(corner_logits, mask_logits, presence_logit):
        corner_probability = sigmoid(corner_logits)
        mask_probability = sigmoid(mask_logits)
        presence = float(sigmoid(presence_logit))
        corners_px = []
        corner_peaks = []
        for heatmap in corner_probability:
            y, x = np.unravel_index(np.argmax(heatmap), heatmap.shape)
            corners_px.append(((x + 0.5) * 8.0, (y + 0.5) * 8.0))
            corner_peaks.append(float(heatmap[y, x]))
        corners_px = np.asarray(corners_px, dtype=np.float32)
        geometry_valid, reason = EspnetAdapter._gate_geometry(corners_px)
        mask_score = float(np.mean(np.partition(
            mask_probability.reshape(-1), -16)[-16:]))
        corner_score = float(np.mean(corner_peaks))
        features = np.asarray((presence, mask_score, corner_score))
        confidence_logit = float(
            ((features - np.asarray((0.7985369852, 0.266170449, 0.6138960815)))
             / np.asarray((0.2844120154, 0.3834623682, 0.149421324)))
            @ np.asarray((1.214142425, 0.2318861439, 1.305251985))
            + 1.114017256
        )
        confidence = float(sigmoid(confidence_logit))
        valid = bool(
            geometry_valid and confidence >= 0.6174671283
            and sum(score >= 0.5 for score in corner_peaks) >= 3
        )
        if not valid and geometry_valid:
            reason = "presence_mask_confidence"
        return valid, corners_px / 160.0, confidence, reason

    def predict(self, frame: np.ndarray) -> Prediction:
        current = hm01b0_full_frame(frame)
        previous = current if self.previous_frame is None else self.previous_frame
        frames = np.stack((previous, current), axis=0).astype(np.float32) / 255.0
        outputs = self.session.run(None, {self.input_name: frames[None]})
        values = {
            output.name: value
            for output, value in zip(self.session.get_outputs(), outputs)
        }
        navigation = values["navigation_logits"][0]
        steering = float(np.clip(navigation[0], -1.0, 1.0))
        collision = float(sigmoid(navigation[1]))
        gate_valid, corners, gate_confidence, reason = self._decode_gate(
            values["corner_heatmaps"][0], values["gate_mask_logits"][0, 0],
            values["gate_presence_logit"][0])
        self.previous_frame = current.copy()
        self.last_input = current
        self.last_input_raw = np.stack((previous, current), axis=-1)
        return Prediction(
            False, False, True, np.full(4, 6.0), np.zeros(4),
            np.full(4, collision, dtype=float), steering, collision,
            gate_valid, corners, gate_confidence, reason,
            gate_intrinsics=(89.1558392549 / 160.0, 89.4608171623 / 160.0,
                             81.1038105230 / 160.0, 73.3473030288 / 160.0),
        )


class DronetAdapter:
    """Adapter for the deployed PULP-DroNet steering/collision contract."""
    def __init__(self, model_path: Path):
        import onnxruntime as ort
        self.session = ort.InferenceSession(str(model_path), providers=["CPUExecutionProvider"])
        self.input = self.session.get_inputs()[0]
        self.last_input = None

    def predict(self, frame: np.ndarray) -> Prediction:
        shape = self.input.shape
        height = int(shape[-2]) if isinstance(shape[-2], int) else 120
        width = int(shape[-1]) if isinstance(shape[-1], int) else 160
        image_u8 = bottom_center_crop(frame, height, width)
        self.last_input = image_u8
        image = image_u8.astype(np.float32) / 255.0
        outputs = self.session.run(None, {self.input.name: image[None, None]})
        values = [float(np.asarray(output).reshape(-1)[0]) for output in outputs]
        if len(values) < 2:
            raise ValueError("dronet adapter needs separate steering and collision outputs")
        steering = float(np.clip(values[0], -1.0, 1.0))
        collision = values[1] if 0.0 <= values[1] <= 1.0 else float(sigmoid(values[1]))
        danger = np.full(4, collision, dtype=float)
        return Prediction(False, False, True, np.full(4, 6.0), np.zeros(4), danger,
                          steering, collision, False, np.zeros((4, 2)), 0.0,
                          "not_provided")


def make_adapter(kind: str, model: Path, threshold: float):
    if kind == "auto":
        if model.is_dir() and (model / "espnet_two_frame_float.onnx").is_file():
            kind = "espnet"
        else:
            kind = "stdc" if model.is_dir() else "sequential"
    if kind == "sequential":
        return SequentialAdapter(model, threshold), kind
    if kind == "stdc":
        return StdcAdapter(model), kind
    if kind == "espnet":
        if model.is_file():
            import onnxruntime as ort
            names = {output.name for output in ort.InferenceSession(
                str(model), providers=["CPUExecutionProvider"]).get_outputs()}
            if "navigation_logits" in names:
                return EspnetDronetGateAdapter(model), kind
        return EspnetAdapter(model), kind
    if kind == "dronet":
        return DronetAdapter(model), kind
    raise ValueError(kind)


def packet_bytes(prediction: Prediction, sequence: int, timestamp_ms: int) -> bytes:
    flags = FLAG_NAVIGATION if prediction.navigation else 0
    if prediction.sector_danger:
        flags |= FLAG_DANGER
    if prediction.metric:
        flags |= FLAG_METRIC
    if prediction.gate_valid:
        flags |= FLAG_GATE
    body = VISION_BODY.pack(
        VISION_MAGIC, timestamp_ms & 0xFFFFFFFF, sequence, flags,
        *np.asarray(prediction.clearance, dtype=float),
        *np.asarray(prediction.confidence, dtype=float),
        *np.asarray(prediction.danger, dtype=float),
        float(prediction.steering), float(prediction.collision),
        *np.asarray(prediction.corners, dtype=float).reshape(-1),
        float(prediction.gate_confidence),
        *prediction.gate_intrinsics,
    )
    return body + struct.pack("<I", zlib.crc32(body) & 0xFFFFFFFF)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", required=True, type=Path)
    parser.add_argument("--adapter",
                        choices=("auto", "espnet", "sequential", "stdc", "dronet"),
                        default="auto")
    parser.add_argument("--camera-port", type=int, default=5200)
    parser.add_argument("--firmware-port", type=int, default=19960)
    parser.add_argument("--camera-fps", type=float, default=20.0)
    parser.add_argument("--delivery-latency-frames", type=int, default=1,
                        help="Fixed capture-to-firmware latency in camera frames")
    parser.add_argument("--clearance-threshold", type=float, default=0.30)
    parser.add_argument("--log", required=True, type=Path)
    parser.add_argument("--frames-dir", type=Path,
                        help="Save the first, first critical, and maximum-risk inputs")
    args = parser.parse_args()
    if args.camera_fps <= 0.0:
        parser.error("--camera-fps must be positive")
    if args.delivery_latency_frames < 0:
        parser.error("--delivery-latency-frames must be nonnegative")
    adapter, adapter_name = make_adapter(args.adapter, args.model.resolve(),
                                         args.clearance_threshold)

    camera = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    camera.bind(("127.0.0.1", args.camera_port))
    camera.settimeout(0.25)
    sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    running = True
    def stop(_signum, _frame):
        nonlocal running
        running = False
    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)
    args.log.parent.mkdir(parents=True, exist_ok=True)
    fields = ["time_s", "sequence", "scheduled_delivery_sequence",
              "emulated_latency_ms", "adapter", "inference_ms", "steering",
              "collision", "metric", "spatial_danger", "navigation",
              "gate_valid", "gate_confidence", "gate_reason",
              "danger_threshold", "gate_fx_normalized",
              "gate_fy_normalized", "gate_cx_normalized",
              "gate_cy_normalized"] + [
              f"clearance_{i}_m" for i in range(4)] + [
              f"confidence_{i}" for i in range(4)] + [f"danger_{i}" for i in range(4)] + [
              f"raw_danger_{i}" for i in range(4)] + [
              f"gate_{corner}_{axis}" for corner in ("tl", "tr", "br", "bl")
              for axis in ("x", "y")]
    sequence = 0
    assembler = CameraFrameAssembler()
    delivery_queue = FixedFrameLatencyQueue(args.delivery_latency_frames)
    maximum_risk = -math.inf
    minimum_risk = math.inf
    critical_saved = False
    gate_saved = False
    frame_metadata = {}
    if args.frames_dir is not None:
        from PIL import Image
        args.frames_dir.mkdir(parents=True, exist_ok=True)
    print(f"vision bridge ready: udp://127.0.0.1:{args.camera_port} -> "
          f"udp://127.0.0.1:{args.firmware_port} ({adapter_name})", flush=True)
    with args.log.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        while running:
            try:
                datagram, _ = camera.recvfrom(65535)
            except socket.timeout:
                continue
            frame = assembler.feed(datagram)
            if frame is None:
                continue
            inference_start = time.perf_counter()
            prediction = adapter.predict(frame)
            raw_danger = (prediction.danger if prediction.raw_danger is None
                          else prediction.raw_danger)
            risk_score = (float(np.max(raw_danger))
                          if prediction.sector_danger
                          else float(prediction.collision))
            inference_ms = 1000.0 * (time.perf_counter() - inference_start)
            sequence = sequence % 65535 + 1
            timestamp_ms = int(round(1000.0 * sequence / args.camera_fps))
            if args.frames_dir is not None:
                exact_input = getattr(adapter, "last_input", frame)
                raw_input = getattr(adapter, "last_input_raw", exact_input)
                def save_input(stem: str) -> None:
                    np.asarray(raw_input, dtype=np.uint8).tofile(
                        args.frames_dir / f"{stem}.raw")
                    if np.asarray(raw_input).ndim == 3 and raw_input.shape[-1] == 2:
                        Image.fromarray(raw_input[:, :, 0], mode="L").save(
                            args.frames_dir / f"{stem}_previous.png")
                        Image.fromarray(raw_input[:, :, 1], mode="L").save(
                            args.frames_dir / f"{stem}_current.png")
                    else:
                        Image.fromarray(exact_input, mode="L").save(
                            args.frames_dir / f"{stem}.png")
                if sequence == 1:
                    Image.fromarray(frame, mode="L").save(
                        args.frames_dir / "first_camera_frame.png")
                    Image.fromarray(exact_input, mode="L").save(
                        args.frames_dir / "first_network_input.png")
                    save_input("first_network_input_hardware")
                    frame_metadata["first"] = {
                        "sequence": sequence, "collision": prediction.collision,
                        "camera_shape": list(frame.shape),
                        "input_shape": list(exact_input.shape),
                        "hardware_input_shape": list(np.asarray(raw_input).shape),
                        "hardware_input_layout": (
                            "HWC previous,current" if np.asarray(raw_input).ndim == 3
                            else "HW"
                        ),
                        "raw_danger": np.asarray(raw_danger).tolist(),
                    }
                if risk_score > maximum_risk:
                    maximum_risk = risk_score
                    Image.fromarray(exact_input, mode="L").save(
                        args.frames_dir / "maximum_risk_input.png")
                    save_input("maximum_risk_input_hardware")
                    frame_metadata["maximum_risk"] = {
                        "sequence": sequence, "collision": prediction.collision,
                        "risk_score": risk_score,
                        "raw_danger": np.asarray(raw_danger).tolist(),
                    }
                if risk_score < minimum_risk:
                    minimum_risk = risk_score
                    Image.fromarray(exact_input, mode="L").save(
                        args.frames_dir / "minimum_risk_input.png")
                    save_input("minimum_risk_input_hardware")
                    frame_metadata["minimum_risk"] = {
                        "sequence": sequence, "collision": prediction.collision,
                        "risk_score": risk_score,
                        "raw_danger": np.asarray(raw_danger).tolist(),
                    }
                if prediction.collision >= 0.70 and not critical_saved:
                    critical_saved = True
                    Image.fromarray(exact_input, mode="L").save(
                        args.frames_dir / "first_critical_input.png")
                    save_input("first_critical_input_hardware")
                    frame_metadata["first_critical"] = {
                        "sequence": sequence, "collision": prediction.collision,
                        "raw_danger": np.asarray(raw_danger).tolist(),
                    }
                if prediction.gate_valid and not gate_saved:
                    gate_saved = True
                    Image.fromarray(exact_input, mode="L").save(
                        args.frames_dir / "first_gate_input.png")
                    save_input("first_gate_input_hardware")
                    frame_metadata["first_gate"] = {
                        "sequence": sequence,
                        "gate_confidence": prediction.gate_confidence,
                        "gate_reason": prediction.gate_reason,
                        "corners": np.asarray(prediction.corners).tolist(),
                        "hardware_input_shape": list(np.asarray(raw_input).shape),
                    }
                (args.frames_dir / "metadata.json").write_text(
                    json.dumps(frame_metadata, indent=2) + "\n")
            ready = delivery_queue.push(sequence, timestamp_ms, prediction)
            if ready is not None:
                ready_sequence, ready_timestamp_ms, ready_prediction = ready
                sender.sendto(packet_bytes(
                    ready_prediction, ready_sequence, ready_timestamp_ms),
                    ("127.0.0.1", args.firmware_port))
            row = {
                "time_s": timestamp_ms / 1000.0, "sequence": sequence,
                "scheduled_delivery_sequence": (
                    sequence + args.delivery_latency_frames),
                "emulated_latency_ms": (
                    1000.0 * args.delivery_latency_frames / args.camera_fps),
                "adapter": adapter_name, "inference_ms": inference_ms,
                "steering": prediction.steering, "collision": prediction.collision,
                "metric": int(prediction.metric), "gate_valid": int(prediction.gate_valid),
                "spatial_danger": int(prediction.sector_danger),
                "navigation": int(prediction.navigation),
                "gate_confidence": prediction.gate_confidence,
                "gate_reason": prediction.gate_reason,
                "danger_threshold": prediction.danger_threshold,
                "gate_fx_normalized": prediction.gate_intrinsics[0],
                "gate_fy_normalized": prediction.gate_intrinsics[1],
                "gate_cx_normalized": prediction.gate_intrinsics[2],
                "gate_cy_normalized": prediction.gate_intrinsics[3],
            }
            row.update({f"clearance_{i}_m": prediction.clearance[i] for i in range(4)})
            row.update({f"confidence_{i}": prediction.confidence[i] for i in range(4)})
            row.update({f"danger_{i}": prediction.danger[i] for i in range(4)})
            row.update({f"raw_danger_{i}": raw_danger[i] for i in range(4)})
            for index, corner in enumerate(("tl", "tr", "br", "bl")):
                row[f"gate_{corner}_x"] = prediction.corners[index, 0]
                row[f"gate_{corner}_y"] = prediction.corners[index, 1]
            writer.writerow(row)
            stream.flush()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
