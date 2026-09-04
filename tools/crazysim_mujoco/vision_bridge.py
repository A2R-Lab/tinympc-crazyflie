#!/usr/bin/env python3
"""Capture CrazySim FPV frames or run a repository-local ONNX vision head.

The bridge deliberately carries metric clearance, collision risk, navigation,
and gate geometry as different signals. It sends the same versioned packet
that the AI deck can send to the STM32 firmware.

Camera-only mode is a passive evidence recorder. It creates no firmware
socket, performs no inference, and cannot emit observations or commands.
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
import subprocess
import time
import zlib
from dataclasses import dataclass, replace
from pathlib import Path

import numpy as np

from hm01b0_poc_camera import Hm01b0PocCamera
from gate_olgmd_reference import (
    Olgmd1Reference,
    gate_corners_packet,
    resize_vertical_area_160_to_96,
    threat_packet,
)


CAMERA_HEADER = struct.Struct("<HHHH")
VISION_BODY = struct.Struct("<4sIHH4f4f4f2f8f1f4f")
VISION_MAGIC = b"\x90\x19\x08\x3a"
VISION_V4_BODY = struct.Struct("<4sIHH4f4f4f2f8f1f4f3f")
VISION_V4_MAGIC = b"\x90\x19\x08\x3b"
VISION_V5_BODY = struct.Struct("<4sIHH4f4f3f2f8f1f4f3f")
VISION_V5_MAGIC = b"\x90\x19\x08\x3c"
VISION_V7_BODY = struct.Struct("<4sIHH3f1f")
VISION_V7_MAGIC = b"\x90\x19\x08\x3e"
VISION_V8_BODY = struct.Struct("<4sIHH3f")
VISION_V8_MAGIC = b"\x90\x19\x08\x3f"
ACTOR_STATE_PACKET = struct.Struct("<8sHHII24fI")
ACTOR_STATE_MAGIC = b"TMAS24V1"
LEGACY_GATE_INTRINSICS = (
    89.15584 / 160.0, 89.46082 / 120.0, 0.5, 0.5)
FLAG_METRIC = 1 << 0
FLAG_DANGER = 1 << 1
FLAG_GATE = 1 << 2
FLAG_NAVIGATION = 1 << 3
FLAG_RESIDUAL_REFERENCE = 1 << 4
VISION_V7_HAS_COLLISION = 1 << 0
VISION_V7_HAS_SQUARE_OPENING = 1 << 1
VISION_V8_HAS_NAVIGATION = 1 << 0
VISION_V8_HAS_SQUARE_OPENING = 1 << 1
SECTOR_BEARING = np.asarray([1.0, 1.0 / 3.0, -1.0 / 3.0, -1.0])


def danger3(values: np.ndarray) -> np.ndarray:
    """Return canonical LEFT/CENTER/RIGHT risks.

    Four-value inputs are accepted only for compatibility with older adapters
    and recordings, where the two middle values represented the same center
    output.
    """
    values = np.asarray(values, dtype=np.float32).reshape(-1)
    if values.size == 3:
        return values
    if values.size == 4:
        return np.asarray((values[0], max(values[1], values[2]), values[3]),
                          dtype=np.float32)
    raise ValueError(f"danger output must have 3 values, got {values.size}")


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


class CameraVideoWriter:
    """Stream raw grayscale camera frames into an H.264 MP4."""

    def __init__(self, path: Path | None, fps: float):
        self.path = path
        self.fps = fps
        self.process = None

    def write(self, frame: np.ndarray) -> None:
        if self.path is None:
            return
        if self.process is None:
            height, width = frame.shape
            self.path.parent.mkdir(parents=True, exist_ok=True)
            self.process = subprocess.Popen(
                [
                    "ffmpeg", "-loglevel", "error", "-y",
                    "-f", "rawvideo", "-pixel_format", "gray",
                    "-video_size", f"{width}x{height}",
                    "-framerate", f"{self.fps:g}", "-i", "-",
                    "-an", "-c:v", "libx264", "-pix_fmt", "yuv420p",
                    "-movflags", "+faststart", str(self.path),
                ],
                stdin=subprocess.PIPE,
                stderr=subprocess.DEVNULL,
            )
        self.process.stdin.write(np.ascontiguousarray(frame).tobytes())

    def close(self) -> None:
        if self.process is None:
            return
        self.process.stdin.close()
        return_code = self.process.wait()
        if return_code != 0:
            raise RuntimeError(f"FPV video encoder exited with {return_code}")


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
    action: int = -1
    action_logits: tuple[float, float, float] = (math.nan, math.nan, math.nan)
    residual_reference: bool = False
    # lateral reference rate [m/s], vertical rate [m/s], progress scale.
    residual_action: tuple[float, float, float] = (0.0, 0.0, 1.0)
    # None selects the legacy V5 packet. A finite probability selects the
    # collision[3] + square-opening[1] V7 ABI.
    square_opening_probability: float | None = None
    tinyvpc_scalar_navigation: bool = False
    # Non-None selects the two-packet gate-FrontNet/oLGMD transport.  The
    # binary threat datagram is emitted before the independently fresh gate
    # datagram, matching the GAP8 schedule.
    imminent_threat: bool | None = None
    olgmd_valid: bool = False


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


def center_crop(frame: np.ndarray, height: int, width: int) -> np.ndarray:
    """Match PULP-DroNet v3's torchvision/Himax center crop."""
    if frame.ndim != 2:
        raise ValueError(f"DroNet expects one grayscale plane, received {frame.shape}")
    if frame.shape[0] < height or frame.shape[1] < width:
        raise ValueError(
            f"DroNet cannot crop {height}x{width} from camera frame {frame.shape}")
    left = (frame.shape[1] - width) // 2
    top = (frame.shape[0] - height) // 2
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
    if collision >= 0.50 and abs(steering) < 0.08:
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
        self.gate_mask_center_px = None
        self.gate_mask_span_px = None
        self.gate_mask_streak = 0
        self.gate_filtered_corners_px = None
        self.gate_corner_velocity_px_per_frame = None
        self.gate_frames_since_valid = 0

    @staticmethod
    def _largest_mask_component(mask: np.ndarray) -> np.ndarray:
        """Return row/column coordinates of the largest 4-connected region."""
        active = np.asarray(mask, dtype=bool)
        visited = np.zeros(active.shape, dtype=bool)
        largest = []
        for row, column in np.argwhere(active):
            if visited[row, column]:
                continue
            component = []
            pending = [(int(row), int(column))]
            visited[row, column] = True
            while pending:
                current_row, current_column = pending.pop()
                component.append((current_row, current_column))
                for next_row, next_column in (
                        (current_row - 1, current_column),
                        (current_row + 1, current_column),
                        (current_row, current_column - 1),
                        (current_row, current_column + 1)):
                    if (0 <= next_row < active.shape[0]
                            and 0 <= next_column < active.shape[1]
                            and active[next_row, next_column]
                            and not visited[next_row, next_column]):
                        visited[next_row, next_column] = True
                        pending.append((next_row, next_column))
            if len(component) > len(largest):
                largest = component
        return np.asarray(largest, dtype=np.int32).reshape(-1, 2)

    @classmethod
    def _mask_opening_center(cls, mask_probability: np.ndarray):
        """Return the center of a resolved gate-opening mask component."""
        active = np.asarray(mask_probability, dtype=float) >= 0.5
        component = cls._largest_mask_component(active)
        if len(component) < 5:
            return None, "mask_component"
        rows, columns = component[:, 0], component[:, 1]
        center = np.asarray((8.0 * columns.mean() + 4.0,
                             8.0 * rows.mean() + 4.0), dtype=np.float32)
        return center, "mask_supported"

    @staticmethod
    def _square_pose_supported(corners_px: np.ndarray) -> bool:
        """Check whether a quadrilateral is a credible projection of a square."""
        source = np.asarray(((0.0, 0.0), (1.0, 0.0),
                             (1.0, 1.0), (0.0, 1.0)))
        system = []
        target = []
        for (x, y), (u, v) in zip(source, corners_px):
            system.extend(((x, y, 1.0, 0.0, 0.0, 0.0, -u * x, -u * y),
                           (0.0, 0.0, 0.0, x, y, 1.0, -v * x, -v * y)))
            target.extend((u, v))
        try:
            homography = np.append(
                np.linalg.solve(np.asarray(system), np.asarray(target)),
                1.0).reshape(3, 3)
        except np.linalg.LinAlgError:
            return False
        camera_inverse = np.linalg.inv(np.asarray((
            (89.1558392549, 0.0, 81.1038105230),
            (0.0, 89.4608171623, 73.3473030288),
            (0.0, 0.0, 1.0))))
        first_axis = camera_inverse @ homography[:, 0]
        second_axis = camera_inverse @ homography[:, 1]
        first_norm = float(np.linalg.norm(first_axis))
        second_norm = float(np.linalg.norm(second_axis))
        if min(first_norm, second_norm) < 1e-6:
            return False
        axis_scale_ratio = max(first_norm, second_norm) / min(
            first_norm, second_norm)
        axis_cosine = abs(float(np.dot(first_axis, second_axis)
                                / (first_norm * second_norm)))
        return axis_scale_ratio <= 1.8 and axis_cosine <= 0.40

    def _decode_gate(self, corner_logits, mask_logits, presence_logit):
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
        mask_center, mask_reason = self._mask_opening_center(
            mask_probability)
        mask_supported = mask_center is not None
        if mask_supported:
            quadrilateral_center = corners_px.mean(axis=0)
            quadrilateral_diagonal = float(np.linalg.norm(
                corners_px[2] - corners_px[0]))
            mask_supported = bool(np.linalg.norm(
                mask_center - quadrilateral_center)
                <= max(18.0, 0.22 * quadrilateral_diagonal))
            if not mask_supported:
                mask_reason = "mask_center_disagreement"

        pose_supported = geometry_valid and self._square_pose_supported(
            corners_px)

        temporal_consistent = False
        if mask_supported and pose_supported:
            mask_span = corners_px[2] - corners_px[0]
            if self.gate_mask_center_px is None:
                temporal_consistent = True
            else:
                center_motion_px = float(np.linalg.norm(
                    mask_center - self.gate_mask_center_px))
                previous_scale = float(np.linalg.norm(self.gate_mask_span_px))
                current_scale = float(np.linalg.norm(mask_span))
                scale_ratio = current_scale / max(previous_scale, 1.0)
                temporal_consistent = bool(
                    center_motion_px <= max(16.0, 0.40 * previous_scale)
                    and 0.60 <= scale_ratio <= 1.67)
            if temporal_consistent:
                self.gate_mask_streak += 1
            else:
                self.gate_mask_streak = 1
            self.gate_mask_center_px = mask_center
            self.gate_mask_span_px = mask_span
        else:
            self.gate_mask_streak = 0
        # Independent agreement between the mask and planar-square pose is
        # sufficient for the first observation. Subsequent observations are
        # bounded against the last credible center and scale; requiring an
        # uninterrupted streak discarded good views whenever one intervening
        # heatmap frame was weak.
        temporal_supported = temporal_consistent
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
            geometry_valid and pose_supported and mask_supported
            and temporal_supported
            and confidence >= 0.6174671283
            and sum(score >= 0.5 for score in corner_peaks) >= 3
        )
        if not geometry_valid:
            pass
        elif not pose_supported:
            reason = "square_pose"
        elif not mask_supported:
            reason = mask_reason
        elif not temporal_supported:
            reason = "mask_temporal"
        elif not valid:
            reason = "presence_mask_confidence"

        published_corners_px = corners_px
        self.gate_frames_since_valid += 1
        if valid:
            gap_frames = self.gate_frames_since_valid
            if (self.gate_filtered_corners_px is None
                    or self.gate_corner_velocity_px_per_frame is None
                    or gap_frames > 12):
                self.gate_filtered_corners_px = corners_px.copy()
                self.gate_corner_velocity_px_per_frame = np.zeros_like(
                    corners_px)
            else:
                predicted = (self.gate_filtered_corners_px
                             + gap_frames
                             * self.gate_corner_velocity_px_per_frame)
                residual = corners_px - predicted
                self.gate_filtered_corners_px = predicted + 0.65 * residual
                self.gate_corner_velocity_px_per_frame += (
                    0.10 / max(1, gap_frames)) * residual
                self.gate_filtered_corners_px = np.clip(
                    self.gate_filtered_corners_px, 0.0, 160.0)
            published_corners_px = self.gate_filtered_corners_px
            self.gate_frames_since_valid = 0
        elif self.gate_frames_since_valid > 12:
            self.gate_filtered_corners_px = None
            self.gate_corner_velocity_px_per_frame = None
        return (valid, published_corners_px / 160.0,
                confidence, reason)

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
    """Adapter for the full PULP-DroNet v3 steering/collision contract."""
    def __init__(self, model_path: Path):
        import onnxruntime as ort
        self.session = ort.InferenceSession(str(model_path), providers=["CPUExecutionProvider"])
        self.input = self.session.get_inputs()[0]
        self.last_input = None

    def predict(self, frame: np.ndarray) -> Prediction:
        shape = self.input.shape
        height = int(shape[-2]) if isinstance(shape[-2], int) else 120
        width = int(shape[-1]) if isinstance(shape[-1], int) else 160
        image_u8 = (center_crop(frame, height, width)
                    if frame.shape[0] >= height and frame.shape[1] >= width
                    else resize_nearest(frame, height, width))
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


class VisionRlAdapter:
    """Two-frame HM01B0 policy with TRACK/LEFT/RIGHT categorical actions."""

    TRACK = 0
    LEFT = 1
    RIGHT = 2

    def __init__(self, model_path: Path):
        import onnxruntime as ort
        self.session = ort.InferenceSession(
            str(model_path), providers=["CPUExecutionProvider"])
        inputs = self.session.get_inputs()
        outputs = self.session.get_outputs()
        if len(inputs) != 1 or inputs[0].name != "frames" or \
                list(inputs[0].shape) != [1, 2, 160, 160]:
            raise ValueError(
                "vision RL input must be frames with shape [1,2,160,160]")
        if len(outputs) != 1 or outputs[0].name != "action_logits" or \
                list(outputs[0].shape) != [1, 3]:
            raise ValueError(
                "vision RL output must be action_logits with shape [1,3]")
        self.input_name = inputs[0].name
        self.previous_frame = None
        self.last_input = None
        self.last_input_raw = None

    def reset(self) -> None:
        self.previous_frame = None
        self.last_input = None
        self.last_input_raw = None

    def predict(self, frame: np.ndarray) -> Prediction:
        current = hm01b0_full_frame(frame)
        previous = current if self.previous_frame is None else self.previous_frame
        ordered = np.stack((previous, current), axis=0)
        logits = np.asarray(self.session.run(
            ["action_logits"],
            {self.input_name: ordered[None].astype(np.float32) / 255.0},
        )[0], dtype=np.float32).reshape(3)
        if not np.all(np.isfinite(logits)):
            raise ValueError("vision RL produced non-finite action logits")
        action = int(np.argmax(logits))
        steering = 1.0 if action == self.LEFT else (-1.0 if action == self.RIGHT else 0.0)
        collision = 0.0 if action == self.TRACK else 1.0
        self.previous_frame = current.copy()
        self.last_input = current
        self.last_input_raw = np.stack((previous, current), axis=-1)
        return Prediction(
            False, False, True, np.full(4, 6.0), np.zeros(4),
            np.full(4, collision), steering, collision, False,
            np.zeros((4, 2)), 0.0, "not_provided", action=action,
            action_logits=tuple(float(value) for value in logits),
        )


class JointGateRlAdapter:
    """One jointly trained policy with frozen action and gate contracts.

    This adapter intentionally has no per-frame fallback.  A bad ONNX model or
    inference result is an operational error; a finite but implausible gate is
    merely withheld while the policy's navigation decision remains intact.
    """

    TRACK = 0
    LEFT = 1
    RIGHT = 2
    _FORMAT = "tinympc-joint-gate-obstacle-student-v1"
    _EXPECTED_INPUT = {
        "name": "frames", "shape": [1, 2, 160, 160], "dtype": "float32",
        "range": [0, 1], "temporal_order": ["previous", "current"],
    }
    _EXPECTED_OUTPUTS = {
        "action_logits": [1, 3],
        "gate_corners": {
            "shape": [1, 4, 2], "range": [0, 1],
            "order": ["TL", "TR", "BR", "BL"],
        },
        "gate_confidence_logit": [1],
    }

    def __init__(self, bundle: Path):
        import onnxruntime as ort

        manifest_path = bundle / "bundle.json"
        if not manifest_path.is_file():
            raise FileNotFoundError(f"missing joint model manifest: {manifest_path}")
        try:
            manifest = json.loads(manifest_path.read_text())
            artifact = manifest["artifacts"]["policy_onnx"]
            deployment = manifest["deployment"]
            model_relative = Path(artifact["path"])
            expected_hash = artifact["sha256"]
        except (json.JSONDecodeError, KeyError, TypeError) as error:
            raise ValueError("joint model manifest is incomplete") from error
        if (manifest.get("format") != self._FORMAT or
                manifest.get("runtime_adapter") != "joint_gate_rl"):
            raise ValueError("bundle is not a joint_gate_rl manifest")
        if (deployment.get("input") != self._EXPECTED_INPUT or
                deployment.get("outputs") != self._EXPECTED_OUTPUTS or
                deployment.get("actions") != ["TRACK", "LEFT", "RIGHT"]):
            raise ValueError("unexpected joint model deployment contract")
        if model_relative.is_absolute():
            raise ValueError("joint model artifact path must be manifest-relative")
        root = bundle.resolve()
        model_path = (root / model_relative).resolve()
        if root not in model_path.parents or not model_path.is_file():
            raise FileNotFoundError(f"joint model artifact missing: {model_path}")
        if not isinstance(expected_hash, str) or file_sha256(model_path) != expected_hash:
            raise ValueError(f"joint model checksum mismatch: {model_path}")
        try:
            intrinsics = deployment["fixed_intrinsics"]
            self.gate_intrinsics = tuple(float(intrinsics[key]) for key in (
                "fx_normalized", "fy_normalized", "cx_normalized", "cy_normalized"))
        except (KeyError, TypeError, ValueError) as error:
            raise ValueError("joint model manifest has invalid fixed intrinsics") from error
        if (not np.all(np.isfinite(self.gate_intrinsics)) or
                self.gate_intrinsics[0] <= 0.0 or self.gate_intrinsics[1] <= 0.0 or
                not all(0.0 <= value <= 1.0 for value in self.gate_intrinsics[2:])):
            raise ValueError("joint model manifest has invalid fixed intrinsics")

        self.session = ort.InferenceSession(str(model_path), providers=["CPUExecutionProvider"])
        inputs = self.session.get_inputs()
        outputs = self.session.get_outputs()
        expected_shapes = {
            "action_logits": [1, 3], "gate_corners": [1, 4, 2],
            "gate_confidence_logit": [1],
        }
        if (len(inputs) != 1 or inputs[0].name != "frames" or
                list(inputs[0].shape) != [1, 2, 160, 160] or
                {output.name: list(output.shape) for output in outputs} != expected_shapes):
            raise ValueError("unexpected joint gate RL ONNX contract")
        self.input_name = inputs[0].name
        self.previous_frame = None
        self.last_input = None
        self.last_input_raw = None

    def reset(self) -> None:
        self.previous_frame = None
        self.last_input = None
        self.last_input_raw = None

    @staticmethod
    def _gate_geometry(corners: np.ndarray) -> tuple[bool, str]:
        """Validate normalized TL/TR/BR/BL corners without changing actions."""
        if corners.shape != (4, 2) or not np.all(np.isfinite(corners)):
            return False, "nonfinite"
        if np.any(corners < 0.0) or np.any(corners > 1.0):
            return False, "range"
        # The existing gate packet uses normalized full-frame coordinates.
        return EspnetAdapter._gate_geometry(corners * 160.0)

    @classmethod
    def _decode_gate(cls, corners: np.ndarray, confidence: float):
        valid, reason = cls._gate_geometry(corners)
        if not valid:
            return False, np.zeros((4, 2), dtype=np.float32), 0.0, reason
        if confidence < 0.5:
            return False, np.zeros((4, 2), dtype=np.float32), 0.0, "confidence"
        return True, corners.astype(np.float32, copy=True), confidence, "accepted"

    def predict(self, frame: np.ndarray) -> Prediction:
        current = hm01b0_full_frame(frame)
        previous = current if self.previous_frame is None else self.previous_frame
        ordered = np.stack((previous, current), axis=0)
        outputs = self.session.run(
            ["action_logits", "gate_corners", "gate_confidence_logit"],
            {self.input_name: ordered[None].astype(np.float32) / 255.0},
        )
        if len(outputs) != 3:
            raise ValueError("joint gate RL inference returned wrong output count")
        logits = np.asarray(outputs[0], dtype=np.float32)
        corners = np.asarray(outputs[1], dtype=np.float32)
        confidence_logit = np.asarray(outputs[2], dtype=np.float32)
        if logits.shape != (1, 3) or corners.shape != (1, 4, 2) or confidence_logit.shape != (1,):
            raise ValueError("joint gate RL inference returned malformed output shape")
        if not (np.all(np.isfinite(logits)) and np.all(np.isfinite(corners)) and
                np.all(np.isfinite(confidence_logit))):
            raise ValueError("joint gate RL inference produced non-finite output")
        action_logits = logits[0]
        action = int(np.argmax(action_logits))
        steering = 1.0 if action == self.LEFT else (-1.0 if action == self.RIGHT else 0.0)
        collision = 0.0 if action == self.TRACK else 1.0
        confidence = float(sigmoid(confidence_logit[0]))
        gate_valid, gate_corners, gate_confidence, gate_reason = self._decode_gate(
            corners[0], confidence)
        self.previous_frame = current.copy()
        self.last_input = current
        self.last_input_raw = np.stack((previous, current), axis=-1)
        return Prediction(
            False, False, True, np.full(4, 6.0), np.zeros(4),
            np.full(4, collision), steering, collision, gate_valid,
            gate_corners, gate_confidence, gate_reason,
            gate_intrinsics=self.gate_intrinsics, action=action,
            action_logits=tuple(float(value) for value in action_logits),
        )


class JointResidualRlAdapter:
    """Opt-in physical residual-reference policy with retained gate geometry."""

    _FORMAT = "tinympc-joint-gate-obstacle-residual-poc-v1"

    def __init__(self, bundle: Path):
        import onnxruntime as ort

        manifest_path = bundle / "bundle.json"
        if not manifest_path.is_file():
            raise FileNotFoundError(f"missing residual model manifest: {manifest_path}")
        manifest = json.loads(manifest_path.read_text())
        if (manifest.get("format") != self._FORMAT or
                manifest.get("runtime_adapter") != "joint_residual_rl"):
            raise ValueError("bundle is not a joint_residual_rl manifest")
        deployment = manifest.get("deployment", {})
        expected_input = {
            "name": "frames", "shape": [1, 2, 160, 160], "dtype": "float32",
            "range": [0, 1], "temporal_order": ["previous", "current"],
        }
        if deployment.get("input") != expected_input:
            raise ValueError("unexpected residual policy input contract")
        try:
            action_contract = deployment["outputs"]["residual_action"]
            artifact = manifest["artifacts"]["policy_onnx"]
            model_relative = Path(artifact["path"])
            expected_hash = artifact["sha256"]
            intrinsics = deployment["fixed_intrinsics"]
            self.gate_intrinsics = tuple(float(intrinsics[key]) for key in (
                "fx_normalized", "fy_normalized", "cx_normalized", "cy_normalized"))
        except (KeyError, TypeError, ValueError) as error:
            raise ValueError("residual model manifest is incomplete") from error
        if (action_contract.get("shape") != [1, 3] or
                action_contract.get("order") != [
                    "lateral_reference_rate_mps", "vertical_reference_rate_mps",
                    "progress_speed_scale"] or
                action_contract.get("bounds") != [[-0.8, 0.8], [-0.2, 0.2], [0.2, 1.0]]):
            raise ValueError("unexpected residual action contract")
        if model_relative.is_absolute():
            raise ValueError("residual model artifact path must be manifest-relative")
        root = bundle.resolve()
        model_path = (root / model_relative).resolve()
        if root not in model_path.parents or not model_path.is_file():
            raise FileNotFoundError(f"residual model artifact missing: {model_path}")
        if not isinstance(expected_hash, str) or file_sha256(model_path) != expected_hash:
            raise ValueError(f"residual model checksum mismatch: {model_path}")
        self.session = ort.InferenceSession(str(model_path), providers=["CPUExecutionProvider"])
        expected_shapes = {
            "residual_action": [1, 3], "gate_corners": [1, 4, 2],
            "gate_confidence_logit": [1],
        }
        inputs, outputs = self.session.get_inputs(), self.session.get_outputs()
        if (len(inputs) != 1 or inputs[0].name != "frames" or
                list(inputs[0].shape) != [1, 2, 160, 160] or
                {output.name: list(output.shape) for output in outputs} != expected_shapes):
            raise ValueError("unexpected residual ONNX contract")
        self.previous_frame = None
        self.last_input = None
        self.last_input_raw = None

    def reset(self) -> None:
        self.previous_frame = None
        self.last_input = None
        self.last_input_raw = None

    def predict(self, frame: np.ndarray) -> Prediction:
        current = hm01b0_full_frame(frame)
        previous = current if self.previous_frame is None else self.previous_frame
        ordered = np.stack((previous, current), axis=0)
        action, corners, confidence_logit = self.session.run(
            ["residual_action", "gate_corners", "gate_confidence_logit"],
            {"frames": ordered[None].astype(np.float32) / 255.0},
        )
        action = np.asarray(action, dtype=np.float32)
        corners = np.asarray(corners, dtype=np.float32)
        confidence_logit = np.asarray(confidence_logit, dtype=np.float32)
        if (action.shape != (1, 3) or corners.shape != (1, 4, 2) or
                confidence_logit.shape != (1,) or
                not np.all(np.isfinite(action)) or not np.all(np.isfinite(corners)) or
                not np.all(np.isfinite(confidence_logit))):
            raise ValueError("residual policy produced malformed output")
        lower = np.asarray((-0.35, -0.20, 0.20), dtype=np.float32)
        upper = np.asarray((0.35, 0.20, 1.00), dtype=np.float32)
        if np.any(action[0] < lower - 1e-6) or np.any(action[0] > upper + 1e-6):
            raise ValueError("residual policy violated its physical action bounds")
        # Quantization to float32 can land a tanh-derived endpoint a few ULPs
        # outside the C receiver's inclusive physical bound. Canonicalize the
        # transport value after the explicit tolerance check.
        action = np.clip(action, lower, upper)
        confidence = float(sigmoid(confidence_logit[0]))
        gate_valid, gate_corners, gate_confidence, gate_reason = \
            JointGateRlAdapter._decode_gate(corners[0], confidence)
        self.previous_frame = current.copy()
        self.last_input = current
        self.last_input_raw = np.stack((previous, current), axis=-1)
        return Prediction(
            False, False, False, np.full(4, 6.0), np.zeros(4), np.zeros(4),
            0.0, 0.0, gate_valid, gate_corners, gate_confidence, gate_reason,
            gate_intrinsics=self.gate_intrinsics, residual_reference=True,
            residual_action=tuple(float(value) for value in action[0]),
        )


class EspNetV7ResidualAdapter:
    """Promoted ESPNet perception plus the sealed v7 residual PPO actor."""

    _FORMAT = "tinympc-espnetv2-imav22-residual-v7"

    def __init__(self, bundle: Path):
        import onnxruntime as ort

        manifest_path = bundle / "bundle.json"
        manifest = json.loads(manifest_path.read_text())
        if (manifest.get("format") != self._FORMAT or
                manifest.get("runtime_adapter") != "espnet_v7_residual"):
            raise ValueError("bundle is not an espnet_v7_residual manifest")
        if manifest.get("state_abi") != "espnetv2_state24_nominal_preview_body_v1":
            raise ValueError("unexpected ESPNet v7 state ABI")
        self.sessions = {}
        for name in ("perception_onnx", "actor_onnx"):
            artifact = manifest["artifacts"][name]
            relative = Path(artifact["path"])
            if relative.is_absolute():
                raise ValueError("ESPNet v7 artifact paths must be relative")
            path = (bundle / relative).resolve()
            if bundle.resolve() not in path.parents or not path.is_file():
                raise FileNotFoundError(path)
            if file_sha256(path) != artifact["sha256"]:
                raise ValueError(f"ESPNet v7 artifact checksum mismatch: {name}")
            self.sessions[name] = ort.InferenceSession(
                str(path), providers=["CPUExecutionProvider"])
        perception = self.sessions["perception_onnx"]
        actor = self.sessions["actor_onnx"]
        if ([(item.name, list(item.shape)) for item in perception.get_inputs()] !=
                [("frames", [1, 2, 160, 160])] or
                {item.name: list(item.shape) for item in perception.get_outputs()} !=
                {"collision_logits": [1, 3], "affordance_logits": [1, 3]}):
            raise ValueError("unexpected ESPNet v7 perception ONNX contract")
        if ([(item.name, list(item.shape)) for item in actor.get_inputs()] !=
                [("obs", [1, 33])] or
                [(item.name, list(item.shape)) for item in actor.get_outputs()] !=
                [("actions", [1, 2])]):
            raise ValueError("unexpected ESPNet v7 actor ONNX contract")
        self.state_receiver = ActorStateReceiver(
            int(manifest.get("actor_state_port", 19961)))
        self.previous_frame = None
        self.previous_normalized_action = np.asarray((0.0, 0.0, 1.0), np.float32)
        self.affordance_history = []
        self.last_input = None
        self.last_input_raw = None
        self.last_collision_probabilities = np.zeros(3, np.float32)
        self.last_affordance_probabilities = np.asarray((1.0, 0.0, 0.0), np.float32)
        self.last_state_sequence = 0
        self.last_state_tick_ms = 0
        self.state_available = False

    @staticmethod
    def _softmax(logits: np.ndarray) -> np.ndarray:
        shifted = logits - np.max(logits)
        values = np.exp(shifted)
        return values / np.sum(values)

    def reset(self) -> None:
        self.previous_frame = None
        self.previous_normalized_action[:] = (0.0, 0.0, 1.0)
        self.affordance_history.clear()

    def commit_delivery(self, prediction: Prediction) -> None:
        lateral, vertical, progress = prediction.residual_action
        normalized = np.asarray(
            (lateral / 0.35, vertical / 0.20, (progress - 0.60) / 0.40),
            dtype=np.float32,
        )
        self.previous_normalized_action = np.clip(normalized, -1.0, 1.0)

    def predict(self, frame: np.ndarray) -> Prediction:
        current = hm01b0_full_frame(frame)
        previous = current if self.previous_frame is None else self.previous_frame
        ordered = np.stack((previous, current), axis=0)
        collision_logits, affordance_logits = self.sessions["perception_onnx"].run(
            ["collision_logits", "affordance_logits"],
            {"frames": ordered[None].astype(np.float32) / 255.0},
        )
        collision = sigmoid(np.asarray(collision_logits[0], dtype=np.float32))
        affordance = self._softmax(np.asarray(affordance_logits[0], dtype=np.float32))
        self.affordance_history.append(affordance)
        self.affordance_history = self.affordance_history[-7:]
        if len(self.affordance_history) < 7:
            filtered_affordance = np.asarray((1.0, 0.0, 0.0), np.float32)
        else:
            filtered_affordance = np.median(
                np.stack(self.affordance_history), axis=0).astype(np.float32)
            filtered_affordance /= np.sum(filtered_affordance)
        state = self.state_receiver.latest()
        self.state_available = state is not None
        if state is None:
            physical_action = np.asarray((0.0, 0.0, 0.20), np.float32)
        else:
            observation = np.concatenate((
                state, self.previous_normalized_action,
                collision, filtered_affordance,
            )).astype(np.float32)
            normalized_action = np.asarray(
                self.sessions["actor_onnx"].run(
                    ["actions"], {"obs": observation[None]})[0][0],
                dtype=np.float32,
            )
            if (normalized_action.shape != (2,) or
                    not np.all(np.isfinite(normalized_action)) or
                    np.any(np.abs(normalized_action) > 1.000001)):
                raise ValueError("ESPNet v7 actor produced an invalid normalized action")
            normalized_action = np.clip(normalized_action, -1.0, 1.0)
            physical_action = np.asarray((
                0.35 * normalized_action[0], 0.0,
                0.60 + 0.40 * normalized_action[1],
            ), dtype=np.float32)
            self.last_state_sequence = self.state_receiver.sequence
            self.last_state_tick_ms = self.state_receiver.firmware_tick_ms
        self.previous_frame = current.copy()
        self.last_input = current
        self.last_input_raw = np.stack((previous, current), axis=-1)
        self.last_collision_probabilities = collision
        self.last_affordance_probabilities = filtered_affordance
        return Prediction(
            False, True, False, np.full(4, 6.0), np.zeros(4),
            collision.copy(),
            0.0, float(collision[1]), False, np.zeros((4, 2)), 0.0,
            "affordance_only", residual_reference=True,
            residual_action=tuple(float(value) for value in physical_action),
        )


class EspNetV7DronetAdapter:
    """Newer ESPNet perception driving the firmware's DroNet maneuver.

    The center collision head decides when an obstacle blocks the route.  The
    left/right heads choose the safer pass side; they do not continuously move
    the reference.  Once triggered, the firmware latches that side and owns
    the complete SIDESTEP -> PASS -> REJOIN state machine.
    """

    _FORMAT = "tinympc-espnetv2-imav22-dronet-v7"

    def __init__(self, bundle: Path):
        import onnxruntime as ort

        manifest = json.loads((bundle / "bundle.json").read_text())
        if (manifest.get("format") != self._FORMAT or
                manifest.get("runtime_adapter") != "espnet_v7_dronet"):
            raise ValueError("bundle is not an espnet_v7_dronet manifest")
        artifact = manifest["artifacts"]["perception_onnx"]
        relative = Path(artifact["path"])
        if relative.is_absolute():
            raise ValueError("ESPNet v7 DroNet artifact path must be relative")
        path = (bundle / relative).resolve()
        models_root = bundle.resolve().parent
        if models_root not in path.parents or not path.is_file():
            raise FileNotFoundError(path)
        if file_sha256(path) != artifact["sha256"]:
            raise ValueError("ESPNet v7 DroNet perception checksum mismatch")
        self.session = ort.InferenceSession(
            str(path), providers=["CPUExecutionProvider"])
        output_contract = {
            item.name: list(item.shape) for item in self.session.get_outputs()}
        base_outputs = {
            "collision_logits": [1, 3], "affordance_logits": [1, 3]}
        gate_outputs = {
            **base_outputs,
            "rail_visibility_logits": [1, 2],
            "rail_corner_heatmap_logits": [1, 4, 20, 20],
        }
        if ([(item.name, list(item.shape)) for item in self.session.get_inputs()] !=
                [("frames", [1, 2, 160, 160])] or
                output_contract not in (base_outputs, gate_outputs)):
            raise ValueError("unexpected ESPNet v7 perception ONNX contract")
        self.has_gate_geometry = output_contract == gate_outputs
        self.previous_frame = None
        self.previous_steering = 0.0
        self.last_input = None
        self.last_input_raw = None
        self.last_collision_probabilities = np.zeros(3, np.float32)
        self.last_raw_collision_probabilities = np.zeros(3, np.float32)
        self.collision_history = []
        self.last_affordance_probabilities = np.asarray(
            (1.0, 0.0, 0.0), np.float32)

    @staticmethod
    def _softmax(logits: np.ndarray) -> np.ndarray:
        shifted = logits - np.max(logits)
        values = np.exp(shifted)
        return values / np.sum(values)

    @staticmethod
    def dronet_command(
            collision_probabilities: np.ndarray,
            previous_steering: float) -> tuple[float, float]:
        left, center, right = np.clip(
            np.asarray(collision_probabilities, dtype=float), 0.0, 1.0)
        # Positive steering means a left pass in tinyRacerDodgeUpdate.  A high
        # risk on the right therefore produces a positive command, and vice
        # versa.  Preserve DroNet's committed side for an ambiguous centered
        # obstacle instead of allowing frame noise to alternate the maneuver.
        steering = float(right - left)
        if center >= 0.50 and abs(steering) < 0.08:
            steering = math.copysign(
                0.60, previous_steering if abs(previous_steering) > 0.08 else 1.0)
        return float(np.clip(steering, -1.0, 1.0)), float(center)

    def reset(self) -> None:
        self.previous_frame = None
        self.previous_steering = 0.0
        self.collision_history.clear()

    def _decode_gate_geometry(
            self, visibility_logits: np.ndarray,
            heatmap_logits: np.ndarray) -> tuple[bool, np.ndarray, float, str]:
        visibility = sigmoid(np.asarray(visibility_logits, dtype=np.float32))
        thresholds = np.asarray((0.5, 0.625), dtype=np.float32)
        raw_corners = []
        for heatmap in np.asarray(heatmap_logits, dtype=np.float32):
            y, x = np.unravel_index(np.argmax(heatmap), heatmap.shape)
            raw_corners.append(((float(x) + 0.5) / 20.0,
                                (float(y) + 0.5) / 20.0))
        # Network order is LT, RT, LB, RB; the firmware packet is TL, TR,
        # BR, BL so its quadrilateral validator receives a perimeter loop.
        corners = np.asarray(raw_corners, dtype=np.float32)[[0, 1, 3, 2]]
        geometry_reason = _polygon_geometry_reason(corners * 160.0, 100.0, 6.0)
        both_rails = bool(np.all(visibility >= thresholds))
        gate_valid = both_rails and geometry_reason == "accepted"
        confidence = float(np.min(np.clip(visibility / thresholds, 0.0, 1.0)))
        reason = "accepted_both_rails" if gate_valid else (
            "rail_visibility" if not both_rails else geometry_reason)
        return gate_valid, corners, confidence, reason

    def predict(self, frame: np.ndarray) -> Prediction:
        current = hm01b0_full_frame(frame)
        previous = current if self.previous_frame is None else self.previous_frame
        ordered = np.stack((previous, current), axis=0)
        outputs = self.session.run(
            None, {"frames": ordered[None].astype(np.float32) / 255.0})
        collision_logits, affordance_logits = outputs[:2]
        model_collision_probabilities = sigmoid(np.asarray(
            collision_logits[0], dtype=np.float32))
        # The promoted encoder and the native-left/right HM01B0 simulation
        # raster both use the firmware ABI's left, center, right order.
        raw_collision_probabilities = model_collision_probabilities
        self.collision_history.append(raw_collision_probabilities)
        self.collision_history = self.collision_history[-3:]
        collision_probabilities = np.median(
            np.stack(self.collision_history), axis=0).astype(np.float32)
        steering, collision = self.dronet_command(
            collision_probabilities, self.previous_steering)
        affordance_probabilities = self._softmax(np.asarray(
            affordance_logits[0], dtype=np.float32)).astype(np.float32)
        self.previous_frame = current.copy()
        self.previous_steering = steering
        self.last_input = current
        self.last_input_raw = np.stack((previous, current), axis=-1)
        self.last_raw_collision_probabilities = raw_collision_probabilities
        self.last_collision_probabilities = collision_probabilities
        self.last_affordance_probabilities = affordance_probabilities
        danger = collision_probabilities.copy()
        if self.has_gate_geometry:
            gate_valid, gate_corners, gate_confidence, gate_reason = \
                self._decode_gate_geometry(outputs[2][0], outputs[3][0])
        else:
            gate_valid = False
            gate_corners = np.zeros((4, 2), dtype=np.float32)
            gate_confidence = 0.0
            gate_reason = "not_provided"
        return Prediction(
            # Preserve the network's native LEFT/CENTER/RIGHT risks. Mark it
            # authoritative so any risky sector can trigger
            # the avoidance state machine, not only the legacy center scalar.
            False, True, True, np.full(4, 6.0), np.zeros(4), danger,
            steering, collision, gate_valid, gate_corners, gate_confidence,
            gate_reason,
        )


class EspNetFrameAblationAdapter(EspNetV7DronetAdapter):
    """Collision-only ESPNet runtime for the matched frame-count ablation.

    The bundle declares whether the network consumes the current frame alone
    or the previous/current pair. Everything after the collision logits is
    deliberately shared with the v7 DroNet path: sigmoid decoding, causal
    median-three filtering, steering/tie-breaking, and the firmware-owned
    avoidance maneuver.
    """

    _FORMAT = "tinympc-espnetv2-imav22-frame-ablation-v1"
    _RUNTIME_ADAPTER = "espnet_frame_ablation"

    def __init__(self, bundle: Path):
        import onnxruntime as ort

        bundle = bundle.resolve()
        manifest = json.loads((bundle / "bundle.json").read_text())
        if (manifest.get("format") != self._FORMAT or
                manifest.get("runtime_adapter") != self._RUNTIME_ADAPTER):
            raise ValueError("bundle is not an ESPNet frame-ablation manifest")
        frame_count = manifest.get("frame_count")
        if type(frame_count) is not int or frame_count not in (1, 2):
            raise ValueError("ESPNet frame-ablation frame_count must be 1 or 2")

        artifact = manifest.get("artifacts", {}).get("perception_onnx", {})
        relative = Path(artifact.get("path", ""))
        if not relative.parts or relative.is_absolute():
            raise ValueError("ESPNet frame-ablation artifact path must be relative")
        path = (bundle / relative).resolve()
        if bundle not in path.parents or not path.is_file():
            raise FileNotFoundError(path)
        if file_sha256(path) != artifact.get("sha256"):
            raise ValueError("ESPNet frame-ablation perception checksum mismatch")

        self.session = ort.InferenceSession(
            str(path), providers=["CPUExecutionProvider"])
        input_contract = [
            (item.name, list(item.shape)) for item in self.session.get_inputs()]
        output_contract = [
            (item.name, list(item.shape)) for item in self.session.get_outputs()]
        if (input_contract != [("frames", [1, frame_count, 160, 160])] or
                output_contract != [("collision_logits", [1, 3])]):
            raise ValueError("unexpected ESPNet frame-ablation ONNX contract")

        self.frame_count = frame_count
        self.previous_frame = None
        self.previous_steering = 0.0
        self.last_input = None
        self.last_input_raw = None
        self.last_collision_probabilities = np.zeros(3, np.float32)
        self.last_raw_collision_probabilities = np.zeros(3, np.float32)
        self.collision_history = []
        # Keep existing vision.csv columns deterministic even though this
        # collision-only bundle has no affordance or rail outputs.
        self.last_affordance_probabilities = np.asarray(
            (1.0, 0.0, 0.0), np.float32)
        self.last_rail_probabilities = np.zeros(3, np.float32)

    def reset(self) -> None:
        self.previous_frame = None
        self.previous_steering = 0.0
        self.collision_history.clear()

    def predict(self, frame: np.ndarray) -> Prediction:
        current = hm01b0_full_frame(frame)
        if self.frame_count == 1:
            ordered = current[None]
            raw_input = current
        else:
            previous = current if self.previous_frame is None else self.previous_frame
            ordered = np.stack((previous, current), axis=0)
            raw_input = np.stack((previous, current), axis=-1)

        collision_logits = self.session.run(
            ["collision_logits"],
            {"frames": ordered[None].astype(np.float32) / 255.0},
        )[0]
        raw_collision_probabilities = sigmoid(np.asarray(
            collision_logits[0], dtype=np.float32))
        self.collision_history.append(raw_collision_probabilities)
        self.collision_history = self.collision_history[-3:]
        collision_probabilities = np.median(
            np.stack(self.collision_history), axis=0).astype(np.float32)
        steering, collision = self.dronet_command(
            collision_probabilities, self.previous_steering)

        self.previous_frame = current.copy()
        self.previous_steering = steering
        self.last_input = current
        self.last_input_raw = raw_input
        self.last_raw_collision_probabilities = raw_collision_probabilities
        self.last_collision_probabilities = collision_probabilities
        return Prediction(
            False, True, True, np.full(4, 6.0), np.zeros(4),
            collision_probabilities.copy(), steering, collision,
            False, np.zeros((4, 2), dtype=np.float32), 0.0,
            "collision_only_ablation",
        )


class EspNetV7PidAdapter(EspNetV7DronetAdapter):
    """Synthetic matched DroNetV3 law: right-minus-left and center risk."""

    def predict(self, frame: np.ndarray) -> Prediction:
        prediction = super().predict(frame)
        left, center, right = np.clip(
            self.last_collision_probabilities, 0.0, 1.0)
        steering = float(np.clip(right - left, -1.0, 1.0))
        self.previous_steering = steering
        return replace(
            prediction, steering=steering, collision=float(center),
            gate_reason="espnet_v7_matched_dronet_v3_pid")


class EspNetV2Gap8DronetAdapter(EspNetV7DronetAdapter):
    """Execute the split NeMO integer student through the DroNet packet ABI."""

    _FORMAT = "tinympc-espnetv2-gap8-int8-v1"

    def __init__(self, bundle: Path):
        import onnxruntime as ort

        manifest = json.loads((bundle / "bundle.json").read_text())
        if (manifest.get("format") != self._FORMAT or
                manifest.get("runtime_adapter") != "espnet_v7_dronet_gap8"):
            raise ValueError("bundle is not an ESPNetV2 GAP8 integer manifest")
        artifacts = manifest["artifacts"]
        self.sessions = {}
        for name in ("encoder", "global_head", "corner_head"):
            artifact = artifacts[name]
            relative = Path(artifact["path"])
            if relative.is_absolute():
                raise ValueError("GAP8 integer artifact paths must be relative")
            path = (bundle / relative).resolve()
            if bundle.resolve() not in path.parents or not path.is_file():
                raise FileNotFoundError(path)
            if file_sha256(path) != artifact["sha256"]:
                raise ValueError(f"{name} integer ONNX checksum mismatch")
            self.sessions[name] = ort.InferenceSession(
                str(path), providers=["CPUExecutionProvider"])
        expected = {
            "encoder": ([1, 3, 160, 160], [1, 32, 40, 40]),
            "global_head": ([1, 32, 40, 40], [1, 8, 1, 1]),
            "corner_head": ([1, 32, 40, 40], [1, 4, 20, 20]),
        }
        for name, (input_shape, output_shape) in expected.items():
            session = self.sessions[name]
            if (list(session.get_inputs()[0].shape) != input_shape or
                    list(session.get_outputs()[0].shape) != output_shape):
                raise ValueError(f"unexpected {name} integer ONNX contract")
        report_artifact = artifacts["nemo_report"]
        report_path = (bundle / report_artifact["path"]).resolve()
        if (bundle.resolve() not in report_path.parents or
                file_sha256(report_path) != report_artifact["sha256"]):
            raise ValueError("NeMO report checksum mismatch")
        report = json.loads(report_path.read_text())
        self.decode = {part["graph"]: part for part in report["partitions"]}
        if set(self.decode) != {"encoder", "global_head", "corner_head"}:
            raise ValueError("NeMO report partition contract mismatch")
        self.previous_frame = None
        self.previous_steering = 0.0
        self.collision_history = []
        self.last_input = None
        self.last_input_raw = None
        self.last_collision_probabilities = np.zeros(3, np.float32)
        self.last_raw_collision_probabilities = np.zeros(3, np.float32)
        self.last_affordance_probabilities = np.asarray(
            (1.0, 0.0, 0.0), np.float32)

    def reset(self) -> None:
        self.previous_frame = None
        self.previous_steering = 0.0
        self.collision_history.clear()

    def _run(self, name: str, values: np.ndarray) -> np.ndarray:
        session = self.sessions[name]
        return session.run(None, {session.get_inputs()[0].name: values})[0]

    def _decode(self, name: str, raw: np.ndarray) -> np.ndarray:
        metadata = self.decode[name]
        channels = raw.shape[1]
        shape = (1, channels) + (1,) * (raw.ndim - 2)
        offset = np.asarray(metadata["output_offset"], np.float32).reshape(shape)
        learned = np.asarray(metadata.get("learned_bias", [0.0] * channels),
                             np.float32).reshape(shape)
        teacher = np.asarray(metadata.get("teacher_logit_offset",
                                          [0.0] * channels),
                             np.float32).reshape(shape)
        return (raw.astype(np.float32) * float(metadata["output_epsilon"])
                - offset + learned - teacher)

    @staticmethod
    def deployment_input(previous: np.ndarray, current: np.ndarray) -> np.ndarray:
        previous_u8 = np.asarray(previous, dtype=np.uint8)
        current_u8 = np.asarray(current, dtype=np.uint8)
        difference = ((current_u8.astype(np.int16)
                       - previous_u8.astype(np.int16) + 255) // 2).astype(np.uint8)
        return np.stack((previous_u8, current_u8, difference), axis=0)

    def predict(self, frame: np.ndarray) -> Prediction:
        current = hm01b0_full_frame(frame)
        previous = current if self.previous_frame is None else self.previous_frame
        network_input = self.deployment_input(previous, current)
        encoded = self._run("encoder", network_input[None].astype(np.float32))
        packed = self._decode(
            "global_head", self._run("global_head", encoded))[0, :, 0, 0]
        corners = self._decode(
            "corner_head", self._run("corner_head", encoded))[0]
        model_collision = sigmoid(packed[:3].astype(np.float32))
        raw_collision = model_collision[[2, 1, 0]]
        self.collision_history.append(raw_collision)
        self.collision_history = self.collision_history[-3:]
        collision_probabilities = np.median(
            np.stack(self.collision_history), axis=0).astype(np.float32)
        steering, collision = self.dronet_command(
            collision_probabilities, self.previous_steering)
        affordance = self._softmax(packed[3:6].astype(np.float32))
        gate_valid, gate_corners, gate_confidence, gate_reason = \
            self._decode_gate_geometry(packed[6:8], corners)
        self.previous_frame = current.copy()
        self.previous_steering = steering
        self.last_input = current
        self.last_input_raw = np.stack((previous, current), axis=-1)
        self.last_raw_collision_probabilities = raw_collision
        self.last_collision_probabilities = collision_probabilities
        self.last_affordance_probabilities = affordance
        return Prediction(
            False, True, True, np.full(4, 6.0), np.zeros(4),
            collision_probabilities.copy(), steering, collision,
            gate_valid, gate_corners, gate_confidence, gate_reason,
        )


class EspNetV2Gap8TaskBranchedAdapter(EspNetV2Gap8DronetAdapter):
    """Exact-INT8 collision branch plus the promoted float gate partitions."""

    _FORMAT = "tinympc-espnetv2-gap8-task-branched-v1"

    def __init__(self, bundle: Path):
        import onnxruntime as ort

        manifest = json.loads((bundle / "bundle.json").read_text())
        if (manifest.get("format") != self._FORMAT or
                manifest.get("runtime_adapter") != "espnet_v7_reactive_gap8"):
            raise ValueError("bundle is not the promoted task-branched GAP8 model")
        self.sessions = {}
        for name in ("encoder", "global_head", "gate_encoder", "gate_head",
                     "corner_head"):
            artifact = manifest["artifacts"][name]
            path = (bundle / artifact["path"]).resolve()
            if bundle.resolve() not in path.parents or not path.is_file():
                raise FileNotFoundError(path)
            if file_sha256(path) != artifact["sha256"]:
                raise ValueError(f"{name} artifact checksum mismatch")
            self.sessions[name] = ort.InferenceSession(
                str(path), providers=["CPUExecutionProvider"])
        expected = {
            "encoder": ([1, 3, 160, 160], [1, 32, 40, 40]),
            "global_head": ([1, 32, 40, 40], [1, 3, 1, 1]),
            "gate_encoder": ([1, 3, 160, 160], [1, 32, 40, 40]),
            "gate_head": ([1, 32, 40, 40], [1, 5, 1, 1]),
            "corner_head": ([1, 32, 40, 40], [1, 4, 20, 20]),
        }
        for name, (input_shape, output_shape) in expected.items():
            session = self.sessions[name]
            if (list(session.get_inputs()[0].shape) != input_shape or
                    list(session.get_outputs()[0].shape) != output_shape):
                raise ValueError(f"unexpected {name} task-branched contract")
        report_artifact = manifest["artifacts"]["nemo_report"]
        report_path = (bundle / report_artifact["path"]).resolve()
        if file_sha256(report_path) != report_artifact["sha256"]:
            raise ValueError("task-branched NeMO report checksum mismatch")
        report = json.loads(report_path.read_text())
        self.decode = {part["graph"]: part for part in report["partitions"]}
        if set(self.decode) != {"encoder", "global_head"}:
            raise ValueError("unexpected task-branched integer partitions")
        offset_artifact = manifest["artifacts"]["output_decode"]
        offset_path = (bundle / offset_artifact["path"]).resolve()
        if file_sha256(offset_path) != offset_artifact["sha256"]:
            raise ValueError("task-branched output decode checksum mismatch")
        offsets = np.load(offset_path)
        self.gate_offset = offsets["gate_offset"].astype(np.float32)
        self.corner_offset = offsets["corner_offset"].astype(np.float32)
        self.previous_frame = None
        self.previous_steering = 0.0
        self.collision_history = []
        self.last_input = None
        self.last_input_raw = None
        self.last_collision_probabilities = np.zeros(3, np.float32)
        self.last_raw_collision_probabilities = np.zeros(3, np.float32)
        self.last_affordance_probabilities = np.asarray((1.0, 0.0, 0.0), np.float32)

    def predict(self, frame: np.ndarray) -> Prediction:
        current = hm01b0_full_frame(frame)
        previous = current if self.previous_frame is None else self.previous_frame
        network_input = self.deployment_input(previous, current)
        collision_features = self._run(
            "encoder", network_input[None].astype(np.float32))
        collision_logits = self._decode(
            "global_head", self._run("global_head", collision_features)
        )[0, :, 0, 0]
        raw_collision = sigmoid(collision_logits.astype(np.float32))
        self.collision_history.append(raw_collision)
        self.collision_history = self.collision_history[-3:]
        collision_probabilities = np.median(
            np.stack(self.collision_history), axis=0).astype(np.float32)
        steering, collision = self.dronet_command(
            collision_probabilities, self.previous_steering)

        float_input = network_input[None].astype(np.float32) / 255.0
        gate_features = self._run("gate_encoder", float_input)
        gate_logits = (self._run("gate_head", gate_features)[0, :, 0, 0]
                       - self.gate_offset)
        corners = self._run("corner_head", gate_features) - self.corner_offset
        affordance = self._softmax(gate_logits[:3].astype(np.float32))
        gate_valid, gate_corners, gate_confidence, gate_reason = \
            self._decode_gate_geometry(gate_logits[3:5], corners[0])
        self.previous_frame = current.copy()
        self.previous_steering = steering
        self.last_input = current
        self.last_input_raw = np.stack((previous, current), axis=-1)
        self.last_raw_collision_probabilities = raw_collision
        self.last_collision_probabilities = collision_probabilities
        self.last_affordance_probabilities = affordance
        return Prediction(
            False, True, True, np.full(4, 6.0), np.zeros(4),
            collision_probabilities.copy(), steering, collision,
            gate_valid, gate_corners, gate_confidence, gate_reason,
        )


class EspNetV2Gap8CloseRailAdapter(EspNetV2Gap8DronetAdapter):
    """Official v8 collision path plus v9 sector rail-recovery output."""

    _FORMAT = "tinympc-espnetv2-gap8-close-rail-v9"

    def __init__(self, bundle: Path):
        import onnxruntime as ort

        manifest = json.loads((bundle / "bundle.json").read_text())
        if (manifest.get("format") != self._FORMAT or
                manifest.get("runtime_adapter") != "espnet_v7_reactive_gap8"):
            raise ValueError("bundle is not the official close-rail v9 model")
        self.sessions = {}
        for name in ("encoder", "global_head", "gate_encoder", "gate_head"):
            artifact = manifest["artifacts"][name]
            path = (bundle / artifact["path"]).resolve()
            if bundle.resolve() not in path.parents or not path.is_file():
                raise FileNotFoundError(path)
            if file_sha256(path) != artifact["sha256"]:
                raise ValueError(f"{name} artifact checksum mismatch")
            self.sessions[name] = ort.InferenceSession(
                str(path), providers=["CPUExecutionProvider"])
        expected = {
            "encoder": ([1, 3, 160, 160], [1, 32, 40, 40]),
            "global_head": ([1, 32, 40, 40], [1, 3, 1, 1]),
            "gate_encoder": ([1, 3, 160, 160], [1, 32, 40, 40]),
            "gate_head": ([1, 32, 40, 40], [1, 6, 1, 1]),
        }
        for name, (input_shape, output_shape) in expected.items():
            session = self.sessions[name]
            if (list(session.get_inputs()[0].shape) != input_shape or
                    list(session.get_outputs()[0].shape) != output_shape):
                raise ValueError(f"unexpected {name} close-rail contract")
        self.collision_decode = self._load_decode_report(
            bundle, manifest, "collision_nemo_report", {"encoder", "global_head"})
        self.gate_decode = self._load_decode_report(
            bundle, manifest, "gate_nemo_report", {"gate_encoder", "gate_head"})
        recovery = manifest["rail_recovery"]
        self.rail_threshold = float(recovery["rail_present_threshold"])
        self.pass_right_threshold = float(recovery["pass_right_threshold"])
        self.previous_frame = None
        self.previous_steering = 0.0
        self.collision_history = []
        self.last_input = None
        self.last_input_raw = None
        self.last_collision_probabilities = np.zeros(3, np.float32)
        self.last_raw_collision_probabilities = np.zeros(3, np.float32)
        self.last_affordance_probabilities = np.full(3, np.nan, np.float32)
        self.last_rail_probabilities = np.zeros(3, np.float32)
        self.last_pass_right_probabilities = np.zeros(3, np.float32)
        self.last_rail_sector = -1

    @staticmethod
    def _load_decode_report(bundle: Path, manifest: dict, artifact_name: str,
                            expected: set[str]) -> dict:
        artifact = manifest["artifacts"][artifact_name]
        path = (bundle / artifact["path"]).resolve()
        if (bundle.resolve() not in path.parents or
                file_sha256(path) != artifact["sha256"]):
            raise ValueError(f"{artifact_name} checksum mismatch")
        decode = {part["graph"]: part
                  for part in json.loads(path.read_text())["partitions"]}
        if set(decode) != expected:
            raise ValueError(f"unexpected {artifact_name} partition contract")
        return decode

    @staticmethod
    def _decode_partition(metadata: dict, raw: np.ndarray) -> np.ndarray:
        channels = raw.shape[1]
        shape = (1, channels) + (1,) * (raw.ndim - 2)
        offset = np.asarray(metadata["output_offset"], np.float32).reshape(shape)
        learned = np.asarray(metadata.get("learned_bias", [0.0] * channels),
                             np.float32).reshape(shape)
        teacher = np.asarray(metadata.get("teacher_logit_offset",
                                          [0.0] * channels),
                             np.float32).reshape(shape)
        return (raw.astype(np.float32) * float(metadata["output_epsilon"])
                - offset + learned - teacher)

    def predict(self, frame: np.ndarray) -> Prediction:
        current = hm01b0_full_frame(frame)
        previous = current if self.previous_frame is None else self.previous_frame
        network_input = self.deployment_input(previous, current)

        collision_features = self._run(
            "encoder", network_input[None].astype(np.float32))
        collision_logits = self._decode_partition(
            self.collision_decode["global_head"],
            self._run("global_head", collision_features))[0, :, 0, 0]
        raw_collision = sigmoid(collision_logits.astype(np.float32))
        self.collision_history.append(raw_collision)
        self.collision_history = self.collision_history[-3:]
        collision = np.median(
            np.stack(self.collision_history), axis=0).astype(np.float32)

        # V9 was trained for post-brake recovery with current=current. Keep
        # the temporal pair for collision inference, but give the rail branch
        # its exact static-pair deployment contract even while the camera has
        # small residual hover motion.
        gate_input = self.deployment_input(current, current)
        gate_features = self._run(
            "gate_encoder", gate_input[None].astype(np.float32))
        recovery_logits = self._decode_partition(
            self.gate_decode["gate_head"],
            self._run("gate_head", gate_features))[0, :, 0, 0]
        rail_probability = sigmoid(recovery_logits[:3].astype(np.float32))
        pass_right_probability = sigmoid(recovery_logits[3:6].astype(np.float32))
        selected_sector = int(np.argmax(collision))
        selected_rail = float(rail_probability[selected_sector])
        rail_present = selected_rail >= self.rail_threshold
        pass_right = bool(
            pass_right_probability[selected_sector] >= self.pass_right_threshold)
        fallback_steering, center_collision = self.dronet_command(
            collision, self.previous_steering)
        opening_steering = (-1.0 if pass_right else 1.0)
        steering = opening_steering if rail_present else fallback_steering

        self.previous_frame = current.copy()
        self.previous_steering = steering
        self.last_input = current
        self.last_input_raw = np.stack((previous, current), axis=-1)
        self.last_raw_collision_probabilities = raw_collision
        self.last_collision_probabilities = collision
        self.last_rail_probabilities = rail_probability
        self.last_pass_right_probabilities = pass_right_probability
        self.last_rail_sector = selected_sector
        gate_reason = ("rail_pass_right" if pass_right else "rail_pass_left") \
            if rail_present else "no_rail_in_highest_risk_sector"
        return Prediction(
            False, True, True, np.full(4, 6.0), np.zeros(4),
            collision.copy(), steering, center_collision,
            rail_present, np.zeros((4, 2), np.float32), selected_rail,
            gate_reason,
        )


class EspNetV2Gap8SquareOpeningAdapter(EspNetV2Gap8CloseRailAdapter):
    """Collision-v8 plus the advisory v11 square-opening detector."""

    _FORMAT = "tinympc-espnetv2-gap8-square-opening-v11"
    _SEMANTIC_ABI = "espnetv2_collision3_square_opening_probability1_v1"

    def __init__(self, bundle: Path):
        manifest = json.loads((bundle / "bundle.json").read_text())
        if (manifest.get("format") != self._FORMAT or
                manifest.get("runtime_adapter") != "espnet_v7_reactive_gap8" or
                manifest.get("semantic_abi") != self._SEMANTIC_ABI):
            raise ValueError("bundle is not the official square-opening v11 model")
        opening = manifest.get("square_opening", {})
        self.opening_threshold = float(opening.get("operating_threshold", math.nan))
        if (not 0.0 < self.opening_threshold < 1.0 or
                opening.get("confirmation_frames") != 2):
            raise ValueError("invalid square-opening threshold/confirmation contract")
        import onnxruntime as ort
        self.sessions = {}
        for name in ("encoder", "global_head", "gate_encoder", "gate_head"):
            artifact = manifest["artifacts"][name]
            path = (bundle / artifact["path"]).resolve()
            if bundle.resolve() not in path.parents or not path.is_file():
                raise FileNotFoundError(path)
            if file_sha256(path) != artifact["sha256"]:
                raise ValueError(f"{name} artifact checksum mismatch")
            self.sessions[name] = ort.InferenceSession(
                str(path), providers=["CPUExecutionProvider"])
        expected = {
            "encoder": ([1, 3, 160, 160], [1, 32, 40, 40]),
            "global_head": ([1, 32, 40, 40], [1, 3, 1, 1]),
            "gate_encoder": ([1, 3, 160, 160], [1, 32, 40, 40]),
            "gate_head": ([1, 32, 40, 40], [1, 1, 1, 1]),
        }
        for name, (input_shape, output_shape) in expected.items():
            session = self.sessions[name]
            if (list(session.get_inputs()[0].shape) != input_shape or
                    list(session.get_outputs()[0].shape) != output_shape):
                raise ValueError(f"unexpected {name} square-opening contract")
        self.collision_decode = self._load_decode_report(
            bundle, manifest, "collision_nemo_report", {"encoder", "global_head"})
        self.gate_decode = self._load_decode_report(
            bundle, manifest, "gate_nemo_report", {"gate_encoder", "gate_head"})
        self.previous_frame = None
        self.previous_steering = 0.0
        self.collision_history = []
        self.last_input = None
        self.last_input_raw = None
        self.last_collision_probabilities = np.zeros(3, np.float32)
        self.last_raw_collision_probabilities = np.zeros(3, np.float32)
        self.last_affordance_probabilities = np.full(3, np.nan, np.float32)
        self.last_rail_probabilities = np.full(3, np.nan, np.float32)
        self.last_pass_right_probabilities = np.full(3, np.nan, np.float32)
        self.last_rail_sector = -1
        self.last_square_opening_probability = 0.0

    def predict(self, frame: np.ndarray) -> Prediction:
        current = hm01b0_full_frame(frame)
        previous = current if self.previous_frame is None else self.previous_frame
        collision_input = self.deployment_input(previous, current)
        collision_features = self._run(
            "encoder", collision_input[None].astype(np.float32))
        collision_logits = self._decode_partition(
            self.collision_decode["global_head"],
            self._run("global_head", collision_features))[0, :, 0, 0]
        raw_collision = sigmoid(collision_logits.astype(np.float32))
        self.collision_history.append(raw_collision)
        self.collision_history = self.collision_history[-3:]
        collision = np.median(
            np.stack(self.collision_history), axis=0).astype(np.float32)

        opening_input = self.deployment_input(current, current)
        opening_features = self._run(
            "gate_encoder", opening_input[None].astype(np.float32))
        opening_logit = self._decode_partition(
            self.gate_decode["gate_head"],
            self._run("gate_head", opening_features))[0, 0, 0, 0]
        opening_probability = float(sigmoid(np.asarray(opening_logit)))
        steering, center_collision = self.dronet_command(
            collision, self.previous_steering)

        self.previous_frame = current.copy()
        self.previous_steering = steering
        self.last_input = current
        self.last_input_raw = np.stack((previous, current), axis=-1)
        self.last_raw_collision_probabilities = raw_collision
        self.last_collision_probabilities = collision
        self.last_square_opening_probability = opening_probability
        return Prediction(
            False, True, False, np.full(4, 6.0), np.zeros(4),
            collision.copy(), steering, center_collision,
            False, np.zeros((4, 2), np.float32), 0.0,
            "square_opening_probability",
            square_opening_probability=opening_probability,
        )


class TinyVPCSquareOpeningAdapter:
    """200x200 TinyVPC navigation with a private static-pair opening branch."""

    _FORMAT = "tinympc-tinyvpc-square-opening-v1"
    _RUNTIME_ADAPTER = "tinyvpc_square_opening"
    _SEMANTIC_ABI = (
        "tinyvpc_normalized_yaw_collision_square_opening_probability_v1")
    _NEMO_FORMAT = "tinyvpc-square-opening-nemo-int8-v1"

    def __init__(self, bundle: Path):
        manifest = json.loads((bundle / "bundle.json").read_text())
        if (manifest.get("format") != self._FORMAT or
                manifest.get("runtime_adapter") != self._RUNTIME_ADAPTER or
                manifest.get("semantic_abi") != self._SEMANTIC_ABI):
            raise ValueError("bundle is not the canonical TinyVPC opening model")
        opening = manifest.get("square_opening", {})
        self.opening_threshold = float(opening.get("operating_threshold", math.nan))
        if (not 0.0 < self.opening_threshold < 1.0 or
                opening.get("confirmation_frames") != 2):
            raise ValueError("invalid TinyVPC opening threshold/confirmation contract")

        import onnxruntime as ort
        self.sessions = {}
        model_names = (
            "navigation_encoder", "navigation_head",
            "square_encoder", "square_opening_head")
        artifacts = manifest.get("artifacts", {})
        if set(artifacts) != set(model_names) | {"nemo_report"}:
            raise ValueError("TinyVPC runtime artifact inventory mismatch")
        for name in model_names:
            artifact = artifacts.get(name, {})
            path = (bundle / artifact.get("path", "")).resolve()
            if bundle.resolve() not in path.parents or not path.is_file():
                raise FileNotFoundError(path)
            if file_sha256(path) != artifact.get("sha256"):
                raise ValueError(f"{name} artifact checksum mismatch")
            self.sessions[name] = ort.InferenceSession(
                str(path), providers=["CPUExecutionProvider"])
        expected = {
            "navigation_encoder": ([1, 2, 200, 200], [1, 32, 50, 50]),
            "navigation_head": ([1, 32, 50, 50], [1, 2, 1, 1]),
            "square_encoder": ([1, 2, 200, 200], [1, 32, 50, 50]),
            "square_opening_head": ([1, 32, 50, 50], [1, 1, 1, 1]),
        }
        for name, (input_shape, output_shape) in expected.items():
            session = self.sessions[name]
            if (list(session.get_inputs()[0].shape) != input_shape or
                    list(session.get_outputs()[0].shape) != output_shape):
                raise ValueError(f"unexpected {name} TinyVPC contract")

        report_artifact = manifest.get("artifacts", {}).get("nemo_report", {})
        report_path = (bundle / report_artifact.get("path", "")).resolve()
        if (bundle.resolve() not in report_path.parents or
                not report_path.is_file() or
                file_sha256(report_path) != report_artifact.get("sha256")):
            raise ValueError("TinyVPC NeMO report checksum mismatch")
        report = json.loads(report_path.read_text())
        if report.get("format") != self._NEMO_FORMAT:
            raise ValueError("unexpected TinyVPC NeMO report format")
        partitions = report.get("partitions", [])
        if not isinstance(partitions, list):
            raise ValueError("TinyVPC NeMO partitions must be a list")
        self.decode = {}
        for partition in partitions:
            if not isinstance(partition, dict) or not isinstance(
                    partition.get("graph"), str):
                raise ValueError("invalid TinyVPC NeMO partition")
            graph = partition["graph"]
            if graph in self.decode:
                raise ValueError("duplicate TinyVPC NeMO partition")
            for field in ("input_epsilon", "output_epsilon",
                          "output_offset", "learned_bias"):
                if field not in partition:
                    raise ValueError(f"TinyVPC partition missing {field}")
            self.decode[graph] = partition
        if set(self.decode) != set(expected):
            raise ValueError("TinyVPC NeMO partition inventory mismatch")
        output_channels = {
            "navigation_encoder": 32,
            "navigation_head": 2,
            "square_encoder": 32,
            "square_opening_head": 1,
        }
        for graph, channels in output_channels.items():
            partition = self.decode[graph]
            input_epsilon = float(partition["input_epsilon"])
            output_epsilon = float(partition["output_epsilon"])
            offset = np.asarray(partition["output_offset"], dtype=float).reshape(-1)
            learned = np.asarray(partition["learned_bias"], dtype=float).reshape(-1)
            if (not math.isfinite(input_epsilon) or input_epsilon <= 0.0 or
                    not math.isfinite(output_epsilon) or output_epsilon <= 0.0 or
                    offset.size != channels or learned.size != channels or
                    not np.all(np.isfinite(offset)) or
                    not np.all(np.isfinite(learned))):
                raise ValueError(f"invalid {graph} TinyVPC decode metadata")
        scale_pairs = (
            ("navigation_encoder", "navigation_head"),
            ("square_encoder", "square_opening_head"),
        )
        if any(not math.isclose(
                float(self.decode[head]["input_epsilon"]),
                float(self.decode[encoder]["output_epsilon"]),
                rel_tol=1e-7, abs_tol=0.0)
               for encoder, head in scale_pairs):
            raise ValueError("TinyVPC encoder/head feature scales do not match")

        self.previous_frame = None
        self.last_input = None
        self.last_input_raw = None
        self.last_normalized_yaw_rate = 0.0
        self.last_collision_probability = 0.0
        self.last_square_opening_probability = 0.0

    def _run(self, name: str, values: np.ndarray) -> np.ndarray:
        session = self.sessions[name]
        return session.run(None, {session.get_inputs()[0].name: values})[0]

    def _decode(self, name: str, raw: np.ndarray) -> np.ndarray:
        metadata = self.decode[name]
        channels = raw.shape[1]
        shape = (1, channels) + (1,) * (raw.ndim - 2)
        offset = np.asarray(metadata["output_offset"], np.float32).reshape(shape)
        learned = np.asarray(metadata["learned_bias"], np.float32).reshape(shape)
        return raw.astype(np.float32) * float(
            metadata["output_epsilon"]) - offset + learned

    @staticmethod
    def _pair(previous: np.ndarray, current: np.ndarray) -> np.ndarray:
        return np.stack((previous, current), axis=0).astype(np.float32)

    def reset(self) -> None:
        self.previous_frame = None

    def predict(self, frame: np.ndarray) -> Prediction:
        current = (center_crop(frame, 200, 200)
                   if frame.shape[0] >= 200 and frame.shape[1] >= 200
                   else resize_bilinear_half_pixel_u8(frame, 200, 200))
        previous = current if self.previous_frame is None else self.previous_frame

        navigation_input = self._pair(previous, current)
        navigation_features = self._run(
            "navigation_encoder", navigation_input[None])
        navigation = self._decode(
            "navigation_head", self._run("navigation_head", navigation_features))
        normalized_yaw_rate = float(np.clip(navigation[0, 0, 0, 0], -1.0, 1.0))
        collision_probability = float(sigmoid(navigation[0, 1, 0, 0]))

        opening_input = self._pair(current, current)
        opening_features = self._run("square_encoder", opening_input[None])
        opening = self._decode(
            "square_opening_head",
            self._run("square_opening_head", opening_features))
        opening_probability = float(sigmoid(opening[0, 0, 0, 0]))

        self.previous_frame = current.copy()
        self.last_input = current
        self.last_input_raw = np.stack((previous, current), axis=-1)
        self.last_normalized_yaw_rate = normalized_yaw_rate
        self.last_collision_probability = collision_probability
        self.last_square_opening_probability = opening_probability
        return Prediction(
            False, False, True, np.full(4, 6.0), np.zeros(4),
            np.full(3, collision_probability), normalized_yaw_rate,
            collision_probability, False, np.zeros((4, 2)), 0.0,
            "square_opening_probability",
            square_opening_probability=opening_probability,
            tinyvpc_scalar_navigation=True,
        )


class TinyVPCNavigationAdapter:
    """Canonical float TinyVPC two-frame navigation frontend."""

    _FORMAT = "tinympc-tinyvpc-navigation-float-v1"
    _RUNTIME_ADAPTER = "tinyvpc_navigation"
    _SEMANTIC_ABI = "tinyvpc_normalized_yaw_collision_probability_v1"

    def __init__(self, bundle: Path):
        manifest = json.loads((bundle / "bundle.json").read_text())
        if (manifest.get("format") != self._FORMAT or
                manifest.get("runtime_adapter") != self._RUNTIME_ADAPTER or
                manifest.get("semantic_abi") != self._SEMANTIC_ABI):
            raise ValueError("bundle is not canonical float TinyVPC navigation")
        artifact = manifest.get("artifacts", {}).get("perception", {})
        path = (bundle / artifact.get("path", "")).resolve()
        if bundle.resolve() not in path.parents or not path.is_file():
            raise FileNotFoundError(path)
        if file_sha256(path) != artifact.get("sha256"):
            raise ValueError("TinyVPC artifact checksum mismatch")
        import onnxruntime as ort
        self.session = ort.InferenceSession(
            str(path), providers=["CPUExecutionProvider"])
        if (list(self.session.get_inputs()[0].shape) != [1, 2, 200, 200] or
                list(self.session.get_outputs()[0].shape) != [1, 2]):
            raise ValueError("unexpected TinyVPC float graph contract")
        self.previous_frame = None
        self.last_input = None
        self.last_input_raw = None
        self.last_normalized_yaw_rate = 0.0
        self.last_collision_probability = 0.0

    def reset(self) -> None:
        self.previous_frame = None

    def predict(self, frame: np.ndarray) -> Prediction:
        current = (center_crop(frame, 200, 200)
                   if frame.shape[0] >= 200 and frame.shape[1] >= 200
                   else resize_bilinear_half_pixel_u8(frame, 200, 200))
        previous = current if self.previous_frame is None else self.previous_frame
        pair = np.stack((previous, current), axis=0).astype(np.float32) / 255.0
        output = self.session.run(
            None, {self.session.get_inputs()[0].name: pair[None]})[0][0]
        yaw = float(np.clip(output[0], -1.0, 1.0))
        collision = float(sigmoid(np.asarray(output[1], np.float32)))
        self.previous_frame = current.copy()
        self.last_input = current
        self.last_input_raw = np.stack((previous, current), axis=-1)
        self.last_normalized_yaw_rate = yaw
        self.last_collision_probability = collision
        return Prediction(
            False, False, True, np.full(4, 6.0), np.zeros(4),
            np.full(3, collision), yaw, collision, False,
            np.zeros((4, 2)), 0.0, "tinyvpc_navigation",
            square_opening_probability=0.0,
            tinyvpc_scalar_navigation=True,
        )


class TinyVPCPidAdapter(TinyVPCNavigationAdapter):
    """TinyVPC native scalar outputs for the stock Crazyflie PID arm."""


class TinyVPCEmergencyStopAdapter:
    """Latch TinyVPC's native p>=0.5 classifier decision for cached braking."""

    COLLISION_THRESHOLD = 0.5

    def __init__(self, bundle: Path, reveal_gate=None):
        self.runtime = TinyVPCNavigationAdapter(bundle)
        self.reveal_gate = reveal_gate
        self.emergency_signal = 0.0
        self.emergency_threshold = self.COLLISION_THRESHOLD
        self.emergency_triggered = False

    def reset(self) -> None:
        self.runtime.reset()
        self.emergency_signal = 0.0
        self.emergency_triggered = False

    def predict(self, frame: np.ndarray) -> Prediction:
        native = self.runtime.predict(frame)
        probability = self.runtime.last_collision_probability
        enabled = self.reveal_gate is None or self.reveal_gate.enabled()
        self.emergency_triggered |= enabled and probability >= self.COLLISION_THRESHOLD
        self.emergency_signal = probability
        self.last_input = self.runtime.last_input
        self.last_input_raw = self.runtime.last_input_raw
        binary = 1.0 if self.emergency_triggered else 0.0
        return Prediction(
            False, True, True, np.full(4, 6.0), np.zeros(4),
            np.asarray((0.0, binary, 0.0), dtype=float), 0.0, binary, False,
            np.zeros((4, 2)), 0.0, "tinyvpc_binary_emergency_stop",
            raw_danger=np.asarray((0.0, probability, 0.0), dtype=float),
        )


class OracleBrakeAdapter:
    """Model-free binary brake trigger driven by measured CrazySim position."""

    def __init__(self, reveal_gate):
        if reveal_gate is None:
            raise ValueError("oracle_brake requires --control-enable-at-x-m")
        self.reveal_gate = reveal_gate
        self.emergency_signal = 0.0
        self.emergency_threshold = 1.0
        self.emergency_triggered = False
        self.last_input = None
        self.last_input_raw = None

    def reset(self) -> None:
        self.emergency_signal = 0.0
        self.emergency_triggered = False

    def predict(self, frame: np.ndarray) -> Prediction:
        self.emergency_triggered |= self.reveal_gate.enabled()
        binary = 1.0 if self.emergency_triggered else 0.0
        self.emergency_signal = binary
        self.last_input = frame
        self.last_input_raw = frame
        return Prediction(
            False, True, self.emergency_triggered,
            np.full(4, 6.0), np.zeros(4),
            np.asarray((0.0, binary, 0.0), dtype=float), 0.0, binary,
            False, np.zeros((4, 2)), 0.0,
            "ground_truth_position_binary_center_stop",
            raw_danger=np.asarray((0.0, binary, 0.0), dtype=float),
        )


class OracleBrakePidAdapter(OracleBrakeAdapter):
    """Expose the oracle latch continuously to the stock velocity PID arm."""

    def predict(self, frame: np.ndarray) -> Prediction:
        prediction = super().predict(frame)
        prediction.navigation = True
        return prediction


def _combined_gate_rl_teacher_paths(bundle: Path) -> tuple[Path, Path]:
    """Resolve the two existing teachers named by a combined POC manifest.

    The POC deliberately contains no copied ONNX artifact.  Keeping the
    paths relative to the manifest makes the combination explicit while
    retaining the independently-versioned obstacle and gate teachers.
    """
    manifest_path = bundle / "bundle.json"
    if not manifest_path.is_file():
        raise FileNotFoundError(f"missing combined model manifest: {manifest_path}")
    manifest = json.loads(manifest_path.read_text())
    if manifest.get("runtime_adapter") != "combined_gate_rl":
        raise ValueError("bundle is not a combined_gate_rl manifest")
    teachers = manifest.get("teacher_models", {})
    try:
        obstacle_entry = teachers["obstacle_rl"]
        gate_entry = teachers["gate_espnet"]
        obstacle_relative = Path(obstacle_entry["path"])
        gate_relative = Path(gate_entry["path"])
    except (KeyError, TypeError) as error:
        raise ValueError("combined model manifest has incomplete teacher paths") from error
    if obstacle_relative.is_absolute() or gate_relative.is_absolute():
        raise ValueError("combined model teacher paths must be manifest-relative")
    obstacle = (bundle / obstacle_relative).resolve()
    gate = (bundle / gate_relative).resolve()
    if not obstacle.is_file() or not gate.is_file():
        raise FileNotFoundError(
            "combined model teacher artifact missing: "
            f"obstacle={obstacle}, gate={gate}")
    for label, path, entry in (("obstacle", obstacle, obstacle_entry),
                               ("gate", gate, gate_entry)):
        expected = entry.get("sha256")
        if expected is not None and file_sha256(path) != expected:
            raise ValueError(f"combined {label} teacher hash mismatch: {path}")
    return obstacle, gate


class CombinedGateRlAdapter:
    """Pair the unchanged RL obstacle action with ESPNet gate geometry.

    This is intentionally an adapter composition rather than a fused model:
    the navigation packet remains byte-for-byte the existing RL policy's
    decision, while gate fields and camera intrinsics come solely from the
    existing ESPNet gate detector.  Both teachers receive the same camera
    frame on each call and maintain their own identical two-frame history.
    """

    def __init__(self, bundle: Path):
        obstacle_model, gate_model = _combined_gate_rl_teacher_paths(bundle)
        self.obstacle = VisionRlAdapter(obstacle_model)
        self.gate = EspnetDronetGateAdapter(gate_model)
        self.last_input = None
        self.last_input_raw = None

    def reset(self) -> None:
        self.obstacle.reset()
        # ESPNet has no public reset because it was previously process-scoped.
        # Clear only its temporal image and gate filter state for deterministic
        # reuse in host tests without changing the established ESPNet decoder.
        self.gate.previous_frame = None
        self.gate.last_input = None
        self.gate.last_input_raw = None
        self.gate.gate_mask_center_px = None
        self.gate.gate_mask_span_px = None
        self.gate.gate_mask_streak = 0
        self.gate.gate_filtered_corners_px = None
        self.gate.gate_corner_velocity_px_per_frame = None
        self.gate.gate_frames_since_valid = 0
        self.last_input = None
        self.last_input_raw = None

    def predict(self, frame: np.ndarray) -> Prediction:
        navigation = self.obstacle.predict(frame)
        gate = self.gate.predict(frame)
        self.last_input = self.obstacle.last_input
        self.last_input_raw = self.obstacle.last_input_raw
        return Prediction(
            navigation.metric, navigation.sector_danger, navigation.navigation,
            navigation.clearance, navigation.confidence, navigation.danger,
            navigation.steering, navigation.collision, gate.gate_valid,
            gate.corners, gate.gate_confidence, gate.gate_reason,
            raw_danger=navigation.raw_danger,
            danger_threshold=navigation.danger_threshold,
            gate_intrinsics=gate.gate_intrinsics,
            action=navigation.action,
            action_logits=navigation.action_logits,
        )


class NanoFlowBridgeAdapter:
    """Expose the pinned direct-flow controller through the common bridge ABI."""

    def __init__(self, model: Path, target_speed_mps: float, camera_fps: float):
        from nanoflow_crazysim_adapter import (
            NanoFlowCrazySimAdapter,
            PinnedNanoFlowLiteRT,
            RegionalBalanceConfig,
            RegionalBalanceController,
        )

        if target_speed_mps <= 0.0:
            raise ValueError("NanoFlow requires a positive target speed")
        estimator = PinnedNanoFlowLiteRT(model)
        controller = RegionalBalanceController(RegionalBalanceConfig(
            policy_dt_s=1.0 / camera_fps,
            nominal_forward_speed_mps=target_speed_mps,
            desired_forward_speed_mps=target_speed_mps,
        ))
        self.runtime = NanoFlowCrazySimAdapter(estimator, controller)
        self.sequence = 0
        self.last_input = None
        self.last_input_raw = None

    def reset(self) -> None:
        self.runtime.reset()
        self.sequence = 0

    def predict(self, frame: np.ndarray) -> Prediction:
        self.sequence = self.sequence % 65535 + 1
        action, _packet, _diagnostics = self.runtime.push_frame(
            frame, sequence=self.sequence, timestamp_ms=self.sequence)
        self.last_input = np.asarray(frame, dtype=np.uint8)
        self.last_input_raw = self.last_input
        return Prediction(
            False, False, False, np.full(4, 6.0), np.zeros(4), np.zeros(4),
            0.0, 0.0, False, np.zeros((4, 2)), 0.0,
            "nanoflow_direct_flow",
            residual_reference=True,
            residual_action=(
                action.lateral_reference_rate_mps,
                action.vertical_reference_rate_mps,
                action.progress_speed_scale,
            ),
        )


class PulpDronetV2BridgeAdapter:
    """Official one-frame GAPflow model for the paper's braking experiment."""

    def __init__(self, model: Path, camera_fps: float,
                 control_enable_after_s: float = 0.0,
                 reveal_gate=None):
        from paper_vision_adapters import PinnedPulpDronetV2

        self.runtime = PinnedPulpDronetV2(model)
        self.last_input = None
        self.last_input_raw = None
        self.frame_count = 0
        self.enable_frame = int(math.ceil(control_enable_after_s * camera_fps))
        self.reveal_gate = reveal_gate

    def reset(self) -> None:
        self.last_input = None
        self.last_input_raw = None
        self.frame_count = 0

    def predict(self, frame: np.ndarray) -> Prediction:
        self.frame_count += 1
        _steering, collision = self.runtime.infer(frame)
        self.last_input = self.runtime.last_input
        self.last_input_raw = self.last_input
        # Niculescu et al. explicitly silence steering in the straight-on
        # collision experiment. Firmware applies their braking law to TinyMPC.
        enabled = (self.frame_count >= self.enable_frame and
                   (self.reveal_gate is None or self.reveal_gate.enabled()))
        applied_collision = collision if enabled else 0.0
        return Prediction(
            False, False, True, np.full(4, 6.0), np.zeros(4),
            np.full(4, applied_collision, dtype=float), 0.0, applied_collision,
            False, np.zeros((4, 2)), 0.0,
            ("pulp_dronet_v2_paper_braking" if enabled else
             "paper_headon_pre_reveal_control_suppressed"),
        )


class PulpDronetV2EmergencyStopBridgeAdapter(PulpDronetV2BridgeAdapter):
    """Turn the paper collision head into one binary emergency-stop request."""

    COLLISION_THRESHOLD = 0.30

    def __init__(self, model: Path, camera_fps: float,
                 control_enable_after_s: float = 0.0,
                 reveal_gate=None):
        super().__init__(model, camera_fps, control_enable_after_s, reveal_gate)
        self.emergency_signal = 0.0
        self.emergency_threshold = self.COLLISION_THRESHOLD
        self.emergency_triggered = False
        self.last_collision_probabilities = np.zeros(3, dtype=np.float32)

    def reset(self) -> None:
        super().reset()
        self.emergency_signal = 0.0
        self.emergency_threshold = self.COLLISION_THRESHOLD
        self.emergency_triggered = False
        self.last_collision_probabilities = np.zeros(3, dtype=np.float32)

    def predict(self, frame: np.ndarray) -> Prediction:
        self.frame_count += 1
        _steering, collision = self.runtime.infer(frame)
        self.last_input = self.runtime.last_input
        self.last_input_raw = self.last_input
        enabled = (self.frame_count >= self.enable_frame and
                   (self.reveal_gate is None or self.reveal_gate.enabled()))
        triggered = enabled and collision >= self.COLLISION_THRESHOLD
        self.emergency_signal = collision
        self.emergency_threshold = self.COLLISION_THRESHOLD
        self.emergency_triggered = self.emergency_triggered or triggered
        self.last_collision_probabilities = np.asarray(
            (collision, collision, collision), dtype=np.float32)
        binary = 1.0 if self.emergency_triggered else 0.0
        return Prediction(
            False, True, True, np.full(4, 6.0), np.zeros(4),
            np.asarray((0.0, binary, 0.0), dtype=float), 0.0, binary,
            False, np.zeros((4, 2)), 0.0,
            "dronet_v2_binary_emergency_stop",
            raw_danger=np.asarray((0.0, collision, 0.0), dtype=float),
        )


class DronetV3EmergencyStopBridgeAdapter:
    """Preserve the full v3 frontend while exposing its paper decision boundary."""

    COLLISION_THRESHOLD = 0.50

    def __init__(self, model: Path, reveal_gate=None):
        self.runtime = PinnedPulpDronetV3BridgeAdapter(model)
        self.reveal_gate = reveal_gate
        self.emergency_signal = 0.0
        self.emergency_threshold = self.COLLISION_THRESHOLD
        self.emergency_triggered = False

    def __getattr__(self, name):
        return getattr(self.runtime, name)

    def reset(self) -> None:
        self.emergency_signal = 0.0
        self.emergency_triggered = False

    def predict(self, frame: np.ndarray) -> Prediction:
        prediction = self.runtime.predict(frame)
        collision = float(np.clip(prediction.collision, 0.0, 1.0))
        enabled = self.reveal_gate is None or self.reveal_gate.enabled()
        triggered = enabled and collision >= self.COLLISION_THRESHOLD
        self.emergency_signal = collision
        self.emergency_triggered = self.emergency_triggered or triggered
        binary = 1.0 if self.emergency_triggered else 0.0
        return replace(
            prediction,
            sector_danger=True,
            steering=0.0,
            collision=binary,
            danger=np.asarray((0.0, binary, 0.0), dtype=float),
            raw_danger=np.asarray((0.0, collision, 0.0), dtype=float),
            gate_reason="dronet_v3_binary_emergency_stop",
        )


class PaperNanoFlowBridgeAdapter:
    """Official NanoFlow graph and regional controller at the TinyMPC ABI."""

    def __init__(self, model: Path, target_speed_mps: float, camera_fps: float,
                 control_enable_after_s: float = 0.0,
                 reveal_gate=None):
        from paper_vision_adapters import (
            PaperNanoFlowController, PinnedPaperNanoFlow,
            nanoflow_sensor_crop,
        )

        if target_speed_mps <= 0.0:
            raise ValueError("paper NanoFlow requires a positive target speed")
        self.runtime = PinnedPaperNanoFlow(model)
        self.controller = PaperNanoFlowController(1.0 / camera_fps)
        self.crop = nanoflow_sensor_crop
        self.previous_frame = None
        self.last_input = None
        self.last_input_raw = None
        self.last_diagnostics = None
        self.frame_count = 0
        self.enable_frame = int(math.ceil(control_enable_after_s * camera_fps))
        self.reveal_gate = reveal_gate

    def reset(self) -> None:
        self.previous_frame = None
        self.controller.reset()
        self.last_diagnostics = None
        self.frame_count = 0

    def predict(self, frame: np.ndarray) -> Prediction:
        self.frame_count += 1
        current = self.crop(frame)
        yaw_rate = 0.0
        enabled = (self.frame_count >= self.enable_frame and
                   (self.reveal_gate is None or self.reveal_gate.enabled()))
        if not enabled:
            # The paper obstacle is introduced only at the reveal point. Keep
            # no pre-reveal temporal state so the first exposed pair starts
            # from the same image, without a synthetic appearance-flow spike.
            self.previous_frame = None
            self.controller.reset()
        elif self.previous_frame is not None:
            flow = self.runtime.infer(self.previous_frame, current)
            yaw_rate, self.last_diagnostics = self.controller.step(flow)
        self.previous_frame = current.copy()
        self.last_input = current
        self.last_input_raw = current
        # PID ablations consume steering as the paper's body-yaw rate in rad/s.
        # The legacy nanoflow_paper mode retains its lateral-reference mapping.
        lateral = 0.35 * float(np.clip(yaw_rate, -1.0, 1.0))
        self.last_yaw_rate_rad_s = yaw_rate
        return Prediction(
            False, False, False, np.full(4, 6.0), np.zeros(4), np.zeros(4),
            0.0, 0.0, False, np.zeros((4, 2)), 0.0,
            ("paper_nanoflow_yaw_to_lateral_reference" if enabled else
             "paper_headon_pre_reveal_control_suppressed"),
            residual_reference=True, residual_action=(lateral, 0.0, 1.0),
        )


class PaperNanoFlowYawEmergencyStopBridgeAdapter(PaperNanoFlowBridgeAdapter):
    """Stop when the published NanoFlow yaw command leaves its deadband."""

    YAW_DEADBAND_RAD_S = 0.10

    def __init__(self, model: Path, target_speed_mps: float, camera_fps: float,
                 control_enable_after_s: float = 0.0, reveal_gate=None):
        super().__init__(model, target_speed_mps, camera_fps,
                         control_enable_after_s, reveal_gate)
        self.emergency_signal = 0.0
        self.emergency_threshold = self.YAW_DEADBAND_RAD_S
        self.emergency_triggered = False

    def reset(self) -> None:
        super().reset()
        self.emergency_signal = 0.0
        self.emergency_triggered = False

    def predict(self, frame: np.ndarray) -> Prediction:
        self.frame_count += 1
        current = self.crop(frame)
        enabled = (self.frame_count >= self.enable_frame and
                   (self.reveal_gate is None or self.reveal_gate.enabled()))
        yaw_rate = 0.0
        if not enabled:
            self.previous_frame = None
            self.controller.reset()
        elif self.previous_frame is not None:
            flow = self.runtime.infer(self.previous_frame, current)
            yaw_rate, self.last_diagnostics = self.controller.step(flow)
        self.previous_frame = current.copy()
        self.last_input = current
        self.last_input_raw = current
        triggered = enabled and abs(yaw_rate) > self.YAW_DEADBAND_RAD_S
        self.emergency_signal = abs(yaw_rate)
        self.emergency_triggered = self.emergency_triggered or triggered
        binary = 1.0 if self.emergency_triggered else 0.0
        return Prediction(
            False, True, True, np.full(4, 6.0), np.zeros(4),
            np.asarray((0.0, binary, 0.0), dtype=float), 0.0, binary,
            False, np.zeros((4, 2)), 0.0,
            "nanoflow_published_yaw_deadband_emergency_stop",
            raw_danger=np.asarray((0.0, abs(yaw_rate), 0.0), dtype=float),
        )


class PaperNanoFlowPidBridgeAdapter(PaperNanoFlowBridgeAdapter):
    """Send the paper's regional-flow PD yaw command to the stock PID path."""

    def predict(self, frame: np.ndarray) -> Prediction:
        prediction = super().predict(frame)
        raw_yaw_rate = float(getattr(self, "last_yaw_rate_rad_s", 0.0))
        yaw_rate = raw_yaw_rate if abs(raw_yaw_rate) > 0.10 else 0.0
        return replace(
            prediction,
            navigation=True,
            steering=yaw_rate,
            collision=0.0,
            danger=np.zeros(4, dtype=float),
            residual_reference=False,
            residual_action=(0.0, 0.0, 1.0),
            gate_reason="paper_nanoflow_direct_yaw_pid",
        )


class PaperNanoFlowEmergencyStopBridgeAdapter(PaperNanoFlowBridgeAdapter):
    """Legacy looming trigger retained for existing experiment aliases."""

    EXPANSION_THRESHOLD_PER_S = 0.50
    CONFIRMATION_FRAMES = 2

    def __init__(self, model: Path, target_speed_mps: float, camera_fps: float,
                 control_enable_after_s: float = 0.0, reveal_gate=None):
        super().__init__(model, target_speed_mps, camera_fps,
                         control_enable_after_s, reveal_gate)
        self.camera_fps = camera_fps
        self.confirmation_count = 0
        self.emergency_signal = 0.0
        self.emergency_threshold = self.EXPANSION_THRESHOLD_PER_S
        self.emergency_triggered = False

    def reset(self) -> None:
        super().reset()
        self.confirmation_count = 0
        self.emergency_signal = 0.0
        self.emergency_triggered = False

    def predict(self, frame: np.ndarray) -> Prediction:
        from paper_vision_adapters import nanoflow_radial_expansion_rate

        self.frame_count += 1
        current = self.crop(frame)
        enabled = (self.frame_count >= self.enable_frame and
                   (self.reveal_gate is None or self.reveal_gate.enabled()))
        expansion = 0.0
        if not enabled:
            self.previous_frame = None
            self.confirmation_count = 0
        elif self.previous_frame is not None:
            flow = self.runtime.infer(self.previous_frame, current)
            expansion = nanoflow_radial_expansion_rate(flow, self.camera_fps)
            if expansion >= self.EXPANSION_THRESHOLD_PER_S:
                self.confirmation_count += 1
            else:
                self.confirmation_count = 0
        self.previous_frame = current.copy()
        self.last_input = current
        self.last_input_raw = current
        triggered = self.confirmation_count >= self.CONFIRMATION_FRAMES
        self.emergency_signal = expansion
        self.emergency_triggered = triggered
        binary = 1.0 if triggered else 0.0
        return Prediction(
            False, True, True, np.full(4, 6.0), np.zeros(4),
            np.asarray((0.0, binary, 0.0), dtype=float), 0.0, binary,
            False, np.zeros((4, 2)), 0.0,
            "nanoflow_radial_expansion_binary_emergency_stop",
            raw_danger=np.asarray((0.0, expansion, 0.0), dtype=float),
        )


class EspNetV7EmergencyStopAdapter:
    """Preserve ESPNetV2 inference, but expose only a binary center stop."""

    COLLISION_THRESHOLD = 0.80

    def __init__(self, bundle: Path, reveal_gate=None):
        self.runtime = EspNetV7DronetAdapter(bundle)
        self.reveal_gate = reveal_gate
        self.emergency_signal = 0.0
        self.emergency_threshold = self.COLLISION_THRESHOLD
        self.emergency_triggered = False

    def reset(self) -> None:
        self.runtime.reset()
        self.emergency_signal = 0.0
        self.emergency_triggered = False

    def predict(self, frame: np.ndarray) -> Prediction:
        prediction = self.runtime.predict(frame)
        collision_probabilities = self.runtime.last_collision_probabilities
        center = float(collision_probabilities[1])
        enabled = self.reveal_gate is None or self.reveal_gate.enabled()
        triggered = enabled and center >= self.COLLISION_THRESHOLD
        self.emergency_signal = center
        self.emergency_triggered = self.emergency_triggered or triggered
        self.last_input = self.runtime.last_input
        self.last_input_raw = self.runtime.last_input_raw
        self.last_collision_probabilities = collision_probabilities
        self.last_raw_collision_probabilities = \
            self.runtime.last_raw_collision_probabilities
        self.last_affordance_probabilities = \
            self.runtime.last_affordance_probabilities
        binary = 1.0 if self.emergency_triggered else 0.0
        return Prediction(
            False, True, True, np.full(4, 6.0), np.zeros(4),
            np.asarray((0.0, binary, 0.0), dtype=float), 0.0, binary,
            False, np.zeros((4, 2)), 0.0,
            "espnet_v7_center_binary_emergency_stop",
            raw_danger=np.asarray((0.0, center, 0.0), dtype=float),
        )


class PredictionControlGateAdapter:
    """Suppress commands until an experimenter-defined approach position."""

    def __init__(self, runtime, reveal_gate):
        self.runtime = runtime
        self.reveal_gate = reveal_gate

    def __getattr__(self, name):
        return getattr(self.runtime, name)

    def reset(self) -> None:
        self.runtime.reset()

    def predict(self, frame: np.ndarray) -> Prediction:
        prediction = self.runtime.predict(frame)
        if self.reveal_gate.enabled():
            return prediction
        return replace(
            prediction,
            danger=np.zeros(3, dtype=float),
            steering=0.0,
            collision=0.0,
            gate_reason="approach_control_not_yet_enabled",
        )


class PrivilegedOracleBridgeAdapter:
    """Run the learned privileged-geometry actor on oracle-only SITL state."""

    def __init__(self, bundle: Path):
        from privileged_oracle_adapter import GeometryObservation, load_policy

        self.GeometryObservation = GeometryObservation
        self.policy = load_policy(bundle)
        self.state_receiver = ActorStateReceiver(19961)
        self.last_input = None
        self.last_input_raw = None
        self.state_available = False
        self.last_state_sequence = 0
        self.last_state_tick_ms = 0

    @staticmethod
    def _relative_corners(x: float, y: float, x0: float, x1: float,
                          y0: float, y1: float):
        return ((x0 - x, y0 - y), (x1 - x, y0 - y),
                (x1 - x, y1 - y), (x0 - x, y1 - y))

    def predict(self, frame: np.ndarray) -> Prediction:
        state = self.state_receiver.latest()
        self.last_input = np.asarray(frame, dtype=np.uint8)
        self.last_input_raw = self.last_input
        self.state_available = state is not None
        if state is None:
            action = (0.0, 0.0, 0.20)
            reason = "oracle_state_unavailable"
        else:
            # This benchmark-only firmware build places exact SITL estimator
            # position in state[21:24].  Geometry remains external privileged
            # course truth and never enters the visual baselines.
            x, y = float(state[21]), float(state[22])
            obstacle = self._relative_corners(x, y, 5.85, 6.15, -0.34, 0.34)
            gate = self._relative_corners(x, y, 8.40, 8.90, -0.20, 0.20)
            observation = self.GeometryObservation(
                (8.65 - x, -y), obstacle, gate, True,
                max(0.0, 5.85 - x), self.state_receiver.firmware_tick_ms / 1000.0)
            result = self.policy.predict(observation)
            action = tuple(float(value) for value in result["action"])
            reason = "privileged_geometry_learned_policy"
            self.last_state_sequence = self.state_receiver.sequence
            self.last_state_tick_ms = self.state_receiver.firmware_tick_ms
        return Prediction(
            False, False, False, np.full(4, 6.0), np.zeros(4), np.zeros(4),
            0.0, 0.0, False, np.zeros((4, 2)), 0.0, reason,
            residual_reference=True, residual_action=action)


class GateFrontnetOlgmdAdapter:
    """Bring-up mirror of the sequential GAP8 gate/oLGMD application.

    The bundle is deliberately marked bring-up-only: it executes the existing
    160x96 integer gate graph and the equation-faithful CPU oLGMD1 reference.
    Replacing the bundle with the qualified 160x160 graph leaves the two packet
    ABIs unchanged.
    """

    _FORMAT = "tinympc-gate-frontnet-olgmd-v1"
    _RUNTIME_ADAPTER = "gate_frontnet_olgmd"

    def __init__(self, bundle: Path):
        import onnxruntime as ort

        bundle = bundle.resolve()
        manifest_path = bundle / "bundle.json"
        manifest = json.loads(manifest_path.read_text())
        if (manifest.get("format") != self._FORMAT or
                manifest.get("runtime_adapter") != self._RUNTIME_ADAPTER):
            raise ValueError("bundle is not a gate-FrontNet/oLGMD bundle")
        if manifest.get("camera_abi") != "hm01b0_gray_160_native_forward_left_up_v2":
            raise ValueError("unexpected gate/oLGMD camera ABI")

        artifact = manifest["artifacts"]["gate_integer_onnx"]
        relative = Path(artifact["path"])
        if relative.is_absolute():
            raise ValueError("gate ONNX path must be bundle-relative")
        model_path = (bundle / relative).resolve()
        if bundle not in model_path.parents or not model_path.is_file():
            raise FileNotFoundError(model_path)
        if file_sha256(model_path) != artifact["sha256"]:
            raise ValueError("gate ONNX checksum mismatch")

        output = manifest["gate_output"]
        if output.get("order") != [
                "TL_x", "TL_y", "TR_x", "TR_y",
                "BR_x", "BR_y", "BL_x", "BL_y"]:
            raise ValueError("unexpected gate corner order")
        self.output_scale = float(output["dequantization_scale"])
        self.output_bias = np.asarray(
            output["dequantization_bias"], dtype=np.float32)
        self.output_divisor = np.asarray(
            output["normalization_divisor_xy"] * 4, dtype=np.float32)
        if (self.output_bias.shape != (8,) or self.output_divisor.shape != (8,) or
                not np.all(np.isfinite(self.output_bias)) or
                not math.isfinite(self.output_scale) or self.output_scale <= 0.0):
            raise ValueError("invalid gate output dequantization")

        self.session = ort.InferenceSession(
            str(model_path), providers=["CPUExecutionProvider"])
        inputs = self.session.get_inputs()
        outputs = self.session.get_outputs()
        if (len(inputs) != 1 or list(inputs[0].shape) != [1, 1, 96, 160] or
                len(outputs) != 1 or list(outputs[0].shape) != [1, 8]):
            raise ValueError("unexpected gate integer ONNX contract")
        self.input_name = inputs[0].name
        self.output_name = outputs[0].name

        olgmd = manifest["olgmd"]
        if (olgmd.get("variant") != "oLGMD1" or
                olgmd.get("polarity") != ["ON", "OFF"] or
                olgmd.get("input_shape") != [80, 80]):
            raise ValueError("unexpected oLGMD contract")
        self.olgmd = Olgmd1Reference(
            fps=float(olgmd["fps"]),
            spike_threshold=float(olgmd["spike_threshold"]),
            collision_window=int(olgmd.get(
                "collision_window", olgmd.get("consecutive_spikes", 4))),
            collision_spikes=int(olgmd.get(
                "collision_spikes", olgmd.get("consecutive_spikes", 4))),
        )
        self.last_input = None
        self.last_input_raw = None
        self.last_olgmd_result = None

    def reset(self) -> None:
        self.olgmd.reset()
        self.last_input = None
        self.last_input_raw = None
        self.last_olgmd_result = None

    def predict(self, frame: np.ndarray) -> Prediction:
        current = hm01b0_full_frame(frame)
        looming = self.olgmd.step(current)
        network_input = resize_vertical_area_160_to_96(current)
        raw = self.session.run(
            [self.output_name],
            {self.input_name: network_input[None, None].astype(np.float32)},
        )[0]
        raw = np.asarray(raw, dtype=np.float32).reshape(-1)
        inference_valid = raw.shape == (8,) and np.all(np.isfinite(raw))
        if inference_valid:
            corners = ((raw * self.output_scale + self.output_bias) /
                       self.output_divisor)
            inference_valid = bool(
                np.all(corners >= -1.0) and np.all(corners <= 2.0))
        if not inference_valid:
            corners = np.zeros(8, dtype=np.float32)
        corners = corners.reshape(4, 2).astype(np.float32)
        self.last_input = network_input
        self.last_input_raw = current
        self.last_olgmd_result = looming
        return Prediction(
            False, False, False, np.full(4, 6.0), np.zeros(4), np.zeros(3),
            0.0, float(looming.imminent_threat), inference_valid, corners,
            1.0 if inference_valid else 0.0,
            "inference_valid" if inference_valid else "invalid_output",
            gate_intrinsics=(89.1558392549 / 160.0, 89.4608171623 / 160.0,
                             81.1038105230 / 160.0, 73.3473030288 / 160.0),
            imminent_threat=looming.imminent_threat,
            olgmd_valid=not looming.warmup,
        )


class GateFrontnetOlgmdObstacleOnlyAdapter:
    """Canonical oLGMD obstacle head; legacy class name preserves the ABI."""

    _FORMAT = "tinympc-olgmd-obstacle-v1"
    _RUNTIME_ADAPTER = "gate_frontnet_olgmd_obstacle_only"

    def __init__(self, bundle: Path, reveal_gate=None):
        bundle = bundle.resolve()
        manifest = json.loads((bundle / "bundle.json").read_text())
        if (manifest.get("format") != self._FORMAT or
                manifest.get("runtime_adapter") != self._RUNTIME_ADAPTER):
            raise ValueError("bundle is not the canonical oLGMD obstacle bundle")
        if manifest.get("camera_abi") != "hm01b0_gray_160_native_forward_left_up_v2":
            raise ValueError("unexpected oLGMD camera ABI")
        olgmd = manifest["olgmd"]
        if (olgmd.get("variant") != "oLGMD1" or
                olgmd.get("polarity") != ["ON", "OFF"] or
                olgmd.get("input_shape") != [80, 80]):
            raise ValueError("unexpected oLGMD contract")
        self.olgmd = Olgmd1Reference(
            fps=float(olgmd["fps"]),
            spike_threshold=float(olgmd["spike_threshold"]),
            collision_window=int(olgmd["collision_window"]),
            collision_spikes=int(olgmd["collision_spikes"]),
        )
        self.reveal_gate = reveal_gate
        self.last_input = None
        self.last_input_raw = None
        self.last_olgmd_result = None

    def reset(self) -> None:
        self.olgmd.reset()
        self.last_input = None
        self.last_input_raw = None
        self.last_olgmd_result = None

    def predict(self, frame: np.ndarray) -> Prediction:
        current = hm01b0_full_frame(frame)
        looming = self.olgmd.step(current)
        threat_enabled = self.reveal_gate is None or self.reveal_gate.enabled()
        self.last_input = current
        self.last_input_raw = current
        self.last_olgmd_result = looming
        return Prediction(
            False, False, False, np.full(4, 6.0), np.zeros(4), np.zeros(3),
            0.0, float(looming.imminent_threat), False,
            np.zeros((4, 2), dtype=np.float32), 0.0,
            "canonical_olgmd_obstacle",
            imminent_threat=(looming.imminent_threat if threat_enabled else False),
            olgmd_valid=not looming.warmup,
        )


class StateCsvPositionRevealGate:
    """Experimenter switch: reveal at a CrazySim position, not wall time."""

    def __init__(self, state_log: Path, x_m: float):
        self.state_log = state_log
        self.x_m = float(x_m)
        self.revealed = False

    def enabled(self) -> bool:
        if self.revealed:
            return True
        try:
            with self.state_log.open("rb") as stream:
                stream.seek(0, 2)
                size = stream.tell()
                stream.seek(max(0, size - 8192))
                lines = stream.read().decode("utf-8", errors="ignore").splitlines()
        except FileNotFoundError:
            return False
        for line in reversed(lines):
            fields = line.split(",")
            if len(fields) < 2 or fields[0] == "time_s":
                continue
            try:
                self.revealed = float(fields[1]) >= self.x_m
            except ValueError:
                continue
            return self.revealed
        return False


def make_adapter(kind: str, model: Path, threshold: float,
                 target_speed_mps: float = 0.0, camera_fps: float = 20.0,
                 control_enable_after_s: float = 0.0,
                 reveal_gate=None):
    if kind == "auto":
        manifest = model / "bundle.json" if model.is_dir() else None
        runtime_adapter = (json.loads(manifest.read_text()).get("runtime_adapter")
                           if manifest is not None and manifest.is_file() else None)
        if runtime_adapter == "joint_residual_rl":
            kind = "joint_residual_rl"
        elif runtime_adapter == "espnet_v7_residual":
            kind = "espnet_v7_residual"
        elif runtime_adapter == "espnet_v7_dronet":
            kind = "espnet_v7_dronet"
        elif runtime_adapter == "espnet_frame_ablation":
            kind = "espnet_frame_ablation"
        elif runtime_adapter == "espnet_v7_dronet_gap8":
            kind = "espnet_v7_dronet_gap8"
        elif runtime_adapter == "espnet_v7_reactive_gap8":
            kind = "espnet_v7_reactive_gap8"
        elif runtime_adapter == TinyVPCSquareOpeningAdapter._RUNTIME_ADAPTER:
            kind = TinyVPCSquareOpeningAdapter._RUNTIME_ADAPTER
        elif runtime_adapter == TinyVPCNavigationAdapter._RUNTIME_ADAPTER:
            kind = TinyVPCNavigationAdapter._RUNTIME_ADAPTER
        elif runtime_adapter == GateFrontnetOlgmdAdapter._RUNTIME_ADAPTER:
            kind = GateFrontnetOlgmdAdapter._RUNTIME_ADAPTER
        elif runtime_adapter == GateFrontnetOlgmdObstacleOnlyAdapter._RUNTIME_ADAPTER:
            kind = GateFrontnetOlgmdObstacleOnlyAdapter._RUNTIME_ADAPTER
        elif runtime_adapter == "combined_gate_rl":
            kind = "combined_gate_rl"
        elif model.is_dir() and (model / "espnet_two_frame_float.onnx").is_file():
            kind = "espnet"
        else:
            kind = "stdc" if model.is_dir() else "sequential"
    if kind == "sequential":
        return SequentialAdapter(model, threshold), kind
    if kind == GateFrontnetOlgmdAdapter._RUNTIME_ADAPTER:
        return GateFrontnetOlgmdAdapter(model), kind
    if kind == GateFrontnetOlgmdObstacleOnlyAdapter._RUNTIME_ADAPTER:
        return GateFrontnetOlgmdObstacleOnlyAdapter(model, reveal_gate), kind
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
    if kind == "tiny_dronet_v3_direct":
        return DronetAdapter(model), kind
    if kind == "pulp_dronet_v3_paper":
        return DronetAdapter(model), kind
    if kind == "pulp_dronet_v3_pid":
        return PinnedPulpDronetV3BridgeAdapter(model), kind
    if kind == "pulp_dronet_v3_emergency":
        return DronetV3EmergencyStopBridgeAdapter(model, reveal_gate), kind
    if kind == "pulp_dronet_v3_straight_brake":
        return StraightBrakeAdapter(
            DronetAdapter(model), "pulp_dronet_v3", reveal_gate), kind
    if kind in ("dronet_v2_paper", "dronet_v2_pid"):
        return PulpDronetV2BridgeAdapter(
            model, camera_fps, control_enable_after_s, reveal_gate), kind
    if kind == "dronet_v2_emergency":
        return PulpDronetV2EmergencyStopBridgeAdapter(
            model, camera_fps, control_enable_after_s, reveal_gate), kind
    if kind == "nanoflow":
        return NanoFlowBridgeAdapter(model, target_speed_mps, camera_fps), kind
    if kind == "nanoflow_paper":
        return PaperNanoFlowBridgeAdapter(
            model, target_speed_mps, camera_fps, control_enable_after_s,
            reveal_gate), kind
    if kind == "nanoflow_paper_pid":
        return PaperNanoFlowPidBridgeAdapter(
            model, target_speed_mps, camera_fps, control_enable_after_s,
            reveal_gate), kind
    if kind == "nanoflow_paper_emergency":
        return PaperNanoFlowEmergencyStopBridgeAdapter(
            model, target_speed_mps, camera_fps, control_enable_after_s,
            reveal_gate), kind
    if kind == "nanoflow_paper_emergency_yaw":
        return PaperNanoFlowYawEmergencyStopBridgeAdapter(
            model, target_speed_mps, camera_fps, control_enable_after_s,
            reveal_gate), kind
    if kind == "oracle_action":
        return PrivilegedOracleBridgeAdapter(model), kind
    if kind in ("rl", "hybrid_rl"):
        return VisionRlAdapter(model), kind
    if kind == "joint_gate_rl":
        return JointGateRlAdapter(model), kind
    if kind == "joint_residual_rl":
        return JointResidualRlAdapter(model), kind
    if kind == "espnet_v7_residual":
        return EspNetV7ResidualAdapter(model), kind
    if kind == "espnet_v7_pid":
        adapter = EspNetV7PidAdapter(model)
        if reveal_gate is not None:
            adapter = PredictionControlGateAdapter(adapter, reveal_gate)
        return adapter, kind
    if kind in ("espnet_v7_dronet", "espnet_v7_reactive"):
        adapter = EspNetV7DronetAdapter(model)
        if reveal_gate is not None:
            adapter = PredictionControlGateAdapter(adapter, reveal_gate)
        return adapter, kind
    if kind == "espnet_frame_ablation":
        adapter = EspNetFrameAblationAdapter(model)
        if reveal_gate is not None:
            adapter = PredictionControlGateAdapter(adapter, reveal_gate)
        return adapter, kind
    if kind == "espnet_v7_straight_brake":
        return StraightBrakeAdapter(
            EspNetV7DronetAdapter(model), "espnet_v7", reveal_gate), kind
    if kind == "espnet_v7_emergency":
        return EspNetV7EmergencyStopAdapter(model, reveal_gate), kind
    if kind == "espnet_v7_dronet_gap8":
        return EspNetV2Gap8DronetAdapter(model), kind
    if kind == "espnet_v7_reactive_gap8":
        manifest = json.loads((model / "bundle.json").read_text())
        if manifest.get("format") == EspNetV2Gap8SquareOpeningAdapter._FORMAT:
            return EspNetV2Gap8SquareOpeningAdapter(model), kind
        if manifest.get("format") == EspNetV2Gap8CloseRailAdapter._FORMAT:
            return EspNetV2Gap8CloseRailAdapter(model), kind
        return EspNetV2Gap8TaskBranchedAdapter(model), kind
    if kind == TinyVPCSquareOpeningAdapter._RUNTIME_ADAPTER:
        return TinyVPCSquareOpeningAdapter(model), kind
    if kind == TinyVPCNavigationAdapter._RUNTIME_ADAPTER:
        return TinyVPCNavigationAdapter(model), kind
    if kind == "tinyvpc_pid":
        adapter = TinyVPCPidAdapter(model)
        if reveal_gate is not None:
            adapter = PredictionControlGateAdapter(adapter, reveal_gate)
        return adapter, kind
    if kind == "tinyvpc_emergency":
        return TinyVPCEmergencyStopAdapter(model, reveal_gate), kind
    if kind == "oracle_brake":
        return OracleBrakeAdapter(reveal_gate), kind
    if kind == "oracle_brake_pid":
        return OracleBrakePidAdapter(reveal_gate), kind
    if kind == "combined_gate_rl":
        return CombinedGateRlAdapter(model), kind
    raise ValueError(kind)


def packet_bytes(prediction: Prediction, sequence: int, timestamp_ms: int) -> bytes:
    if prediction.tinyvpc_scalar_navigation:
        probability = float(prediction.square_opening_probability)
        values = (float(prediction.steering), float(prediction.collision),
                  probability)
        if (not all(math.isfinite(value) for value in values) or
                not -1.0 <= values[0] <= 1.0 or
                not 0.0 <= values[1] <= 1.0 or
                not 0.0 <= values[2] <= 1.0):
            raise ValueError("TinyVPC V8 outputs violate their bounded ABI")
        body = VISION_V8_BODY.pack(
            VISION_V8_MAGIC, timestamp_ms & 0xFFFFFFFF, sequence,
            VISION_V8_HAS_NAVIGATION | VISION_V8_HAS_SQUARE_OPENING,
            *values)
        return body + struct.pack("<I", zlib.crc32(body) & 0xFFFFFFFF)
    if prediction.square_opening_probability is not None:
        probability = float(prediction.square_opening_probability)
        if not math.isfinite(probability) or not 0.0 <= probability <= 1.0:
            raise ValueError("square-opening probability must lie in [0,1]")
        risks = danger3(prediction.danger)
        body = VISION_V7_BODY.pack(
            VISION_V7_MAGIC, timestamp_ms & 0xFFFFFFFF, sequence,
            VISION_V7_HAS_COLLISION | VISION_V7_HAS_SQUARE_OPENING,
            *risks, probability)
        return body + struct.pack("<I", zlib.crc32(body) & 0xFFFFFFFF)
    flags = FLAG_NAVIGATION if prediction.navigation else 0
    if prediction.sector_danger:
        flags |= FLAG_DANGER
    if prediction.metric:
        flags |= FLAG_METRIC
    if prediction.gate_valid:
        flags |= FLAG_GATE
    if prediction.residual_reference:
        flags |= FLAG_RESIDUAL_REFERENCE
    risks = danger3(prediction.danger)
    common = (
        timestamp_ms & 0xFFFFFFFF, sequence, flags,
        *np.asarray(prediction.clearance, dtype=float),
        *np.asarray(prediction.confidence, dtype=float),
        *risks,
        float(prediction.steering), float(prediction.collision),
        *np.asarray(prediction.corners, dtype=float).reshape(-1),
        float(prediction.gate_confidence),
        *prediction.gate_intrinsics,
    )
    residual_action = (prediction.residual_action if prediction.residual_reference
                       else (0.0, 0.0, 1.0))
    body = VISION_V5_BODY.pack(
        VISION_V5_MAGIC, *common, *residual_action)
    return body + struct.pack("<I", zlib.crc32(body) & 0xFFFFFFFF)


def packet_datagrams(
        prediction: Prediction, sequence: int, timestamp_ms: int) -> tuple[bytes, ...]:
    """Return ordered firmware datagrams for one completed frame.

    Existing adapters retain their single-packet transport.  The gate/oLGMD
    path deliberately sends the small threat result first so the gate network
    cannot add latency to emergency braking.
    """
    if prediction.imminent_threat is None:
        return (packet_bytes(prediction, sequence, timestamp_ms),)
    return (
        threat_packet(sequence, timestamp_ms, prediction.imminent_threat),
        gate_corners_packet(
            sequence, timestamp_ms, prediction.gate_valid, prediction.corners),
    )


def record_camera_only(args: argparse.Namespace) -> int:
    """Record reassembled camera frames without any firmware-facing channel."""
    camera = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    camera.bind(("127.0.0.1", args.camera_port))
    camera.settimeout(0.25)
    running = True

    def stop(_signum, _frame):
        nonlocal running
        running = False

    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)
    args.log.parent.mkdir(parents=True, exist_ok=True)
    if args.frames_dir is not None:
        from PIL import Image
        args.frames_dir.mkdir(parents=True, exist_ok=True)
    assembler = CameraFrameAssembler()
    sequence = 0
    metadata = {
        "mode": "camera_only",
        "inference_enabled": False,
        "firmware_output_enabled": False,
        "frames_received": 0,
    }
    video = CameraVideoWriter(args.camera_video, args.camera_fps)
    print(f"camera-only bridge ready: udp://127.0.0.1:{args.camera_port} "
          "(passive; no firmware output)", flush=True)
    with args.log.open("w", newline="") as stream:
        fields = ["time_s", "sequence", "width", "height", "bytes", "sha256"]
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
            video.write(frame)
            sequence += 1
            timestamp_ms = int(round(1000.0 * sequence / args.camera_fps))
            frame_bytes = np.ascontiguousarray(frame).tobytes()
            writer.writerow({
                "time_s": timestamp_ms / 1000.0,
                "sequence": sequence,
                "width": frame.shape[1],
                "height": frame.shape[0],
                "bytes": len(frame_bytes),
                "sha256": hashlib.sha256(frame_bytes).hexdigest(),
            })
            stream.flush()
            metadata["frames_received"] = sequence
            if sequence == 1:
                metadata["first_frame_shape"] = list(frame.shape)
                metadata["first_frame_sha256"] = hashlib.sha256(frame_bytes).hexdigest()
                if args.frames_dir is not None:
                    Image.fromarray(frame, mode="L").save(
                        args.frames_dir / "first_camera_frame.png")
                    np.asarray(frame, dtype=np.uint8).tofile(
                        args.frames_dir / "first_camera_frame.raw")
            if args.frames_dir is not None:
                (args.frames_dir / "metadata.json").write_text(
                    json.dumps(metadata, indent=2) + "\n")
    video.close()
    return 0


def main() -> int:
    parser = argparse.ArgumentParser()
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--model", type=Path)
    mode.add_argument("--camera-only", action="store_true",
                      help="Capture/log frames without inference or firmware I/O")
    parser.add_argument("--adapter",
                        choices=("auto", "espnet", "sequential", "stdc", "dronet",
                                 "rl", "hybrid_rl", "combined_gate_rl", "joint_gate_rl",
                                 "joint_residual_rl", "espnet_v7_residual",
                                 "espnet_v7_dronet", "espnet_v7_reactive",
                                 "espnet_frame_ablation",
                                 "espnet_v7_dronet_gap8",
                                 "espnet_v7_reactive_gap8",
                                 "tinyvpc_square_opening",
                                 "tinyvpc_navigation", "tinyvpc_pid",
                                 "tinyvpc_emergency",
                                 "oracle_brake", "oracle_brake_pid",
                                 "gate_frontnet_olgmd",
                                 "gate_frontnet_olgmd_obstacle_only",
                                 "tiny_dronet_v3_direct", "pulp_dronet_v3_paper",
                                 "pulp_dronet_v3_pid", "pulp_dronet_v3_emergency",
                                 "pulp_dronet_v3_straight_brake",
                                 "dronet_v2_paper", "dronet_v2_pid",
                                 "dronet_v2_emergency", "nanoflow",
                                 "nanoflow_paper", "nanoflow_paper_pid",
                                 "nanoflow_paper_emergency", "nanoflow_paper_emergency_yaw",
                                 "espnet_v7_pid", "espnet_v7_emergency", "espnet_v7_straight_brake",
                                 "oracle_action"),
                        default="auto")
    parser.add_argument("--camera-port", type=int, default=5200)
    parser.add_argument("--firmware-port", type=int, default=19960)
    parser.add_argument("--camera-fps", type=float, default=20.0)
    parser.add_argument("--target-speed-mps", type=float, default=0.0,
                        help="Benchmark forward speed supplied to speed-aware adapters")
    parser.add_argument("--control-enable-after-s", type=float, default=0.0,
                        help="Paper head-on obstacle reveal time; suppress control before it")
    parser.add_argument("--control-enable-at-x-m", type=float,
                        help="Reveal paper obstacle when CrazySim state reaches this x position")
    parser.add_argument("--hm01b0-poc", action="store_true",
                        help="apply the calibrated HM01B0 path used only by the gate POC")
    parser.add_argument("--sensor-seed", type=int,
                        help="deterministic HM01B0 sensor-noise seed (requires --hm01b0-poc)")
    parser.add_argument("--delivery-latency-frames", type=int, default=1,
                        help="Fixed capture-to-firmware latency in camera frames")
    parser.add_argument("--passive", action="store_true",
                        help="Run and log inference without sending firmware packets")
    parser.add_argument("--clearance-threshold", type=float, default=0.30)
    parser.add_argument("--log", required=True, type=Path)
    parser.add_argument("--frames-dir", type=Path,
                        help="Save the first, first critical, and maximum-risk inputs")
    parser.add_argument("--camera-video", type=Path,
                        help="Encode the complete simulated onboard-camera stream")
    args = parser.parse_args()
    if args.camera_fps <= 0.0:
        parser.error("--camera-fps must be positive")
    if args.delivery_latency_frames < 0:
        parser.error("--delivery-latency-frames must be nonnegative")
    if args.sensor_seed is not None and not args.hm01b0_poc:
        parser.error("--sensor-seed requires --hm01b0-poc")
    if args.hm01b0_poc and args.sensor_seed is None:
        parser.error("--hm01b0-poc requires --sensor-seed")
    if args.camera_only:
        return record_camera_only(args)
    adapter, adapter_name = make_adapter(args.adapter, args.model.resolve(),
                                         args.clearance_threshold)
    hm01b0_camera = Hm01b0PocCamera(args.sensor_seed) if args.hm01b0_poc else None

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
              "collision", "rl_action", "rl_logit_track", "rl_logit_left",
              "rl_logit_right", "metric", "spatial_danger", "navigation",
              "gate_valid", "gate_confidence", "gate_reason",
              "danger_threshold", "gate_fx_normalized",
              "gate_fy_normalized", "gate_cx_normalized",
              "gate_cy_normalized", "residual_reference",
              "lateral_reference_rate_mps", "vertical_reference_rate_mps",
              "progress_speed_scale", "actor_state_available",
              "actor_state_sequence", "actor_state_firmware_tick_ms",
              "collision_probability_left", "collision_probability_center",
              "collision_probability_right", "gate_affordance_none",
              "gate_affordance_opening_left", "gate_affordance_opening_right",
              "rail_present_left", "rail_present_center", "rail_present_right",
              "pass_right_left", "pass_right_center", "pass_right_right",
              "rail_selected_sector", "square_opening_probability",
              "normalized_yaw_rate", "scalar_collision_probability",
              "emergency_signal", "emergency_threshold",
              "emergency_triggered", "olgmd_valid", "imminent_threat",
              "olgmd_membrane", "olgmd_ffi_on", "olgmd_ffi_off"] + [
              f"clearance_{i}_m" for i in range(4)] + [
              f"confidence_{i}" for i in range(4)] + [f"danger_{i}" for i in range(3)] + [
              f"raw_danger_{i}" for i in range(3)] + [
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
    video = CameraVideoWriter(args.camera_video, args.camera_fps)
    if args.frames_dir is not None:
        from PIL import Image
        args.frames_dir.mkdir(parents=True, exist_ok=True)
    destination = ("passive log only" if args.passive else
                   f"udp://127.0.0.1:{args.firmware_port}")
    print(f"vision bridge ready: udp://127.0.0.1:{args.camera_port} -> "
          f"{destination} ({adapter_name})", flush=True)
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
            if hm01b0_camera is not None:
                # Transform exactly once per captured frame.  The resulting
                # uint8 frame is then handed unchanged to both teachers, so
                # their temporal histories are exactly identical.
                frame = hm01b0_camera.transform(frame)
            video.write(frame)
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
            if ready is not None and not args.passive:
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
                "rl_action": prediction.action,
                "rl_logit_track": prediction.action_logits[0],
                "rl_logit_left": prediction.action_logits[1],
                "rl_logit_right": prediction.action_logits[2],
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
                "residual_reference": int(prediction.residual_reference),
                "lateral_reference_rate_mps": prediction.residual_action[0],
                "vertical_reference_rate_mps": prediction.residual_action[1],
                "progress_speed_scale": prediction.residual_action[2],
                "actor_state_available": int(getattr(adapter, "state_available", False)),
                "actor_state_sequence": getattr(adapter, "last_state_sequence", 0),
                "actor_state_firmware_tick_ms": getattr(adapter, "last_state_tick_ms", 0),
                "emergency_signal": getattr(adapter, "emergency_signal", math.nan),
                "emergency_threshold": getattr(adapter, "emergency_threshold", math.nan),
                "emergency_triggered": int(getattr(
                    adapter, "emergency_triggered", False)),
                "olgmd_valid": int(prediction.olgmd_valid),
                "imminent_threat": ("" if prediction.imminent_threat is None
                                    else int(prediction.imminent_threat)),
                "olgmd_membrane": getattr(
                    getattr(adapter, "last_olgmd_result", None),
                    "membrane_potential", math.nan),
                "olgmd_ffi_on": getattr(
                    getattr(adapter, "last_olgmd_result", None),
                    "ffi_on", math.nan),
                "olgmd_ffi_off": getattr(
                    getattr(adapter, "last_olgmd_result", None),
                    "ffi_off", math.nan),
            }
            collision_probabilities = getattr(
                adapter, "last_collision_probabilities", np.full(3, math.nan))
            affordance_probabilities = getattr(
                adapter, "last_affordance_probabilities", np.full(3, math.nan))
            rail_probabilities = getattr(
                adapter, "last_rail_probabilities", np.full(3, math.nan))
            pass_right_probabilities = getattr(
                adapter, "last_pass_right_probabilities", np.full(3, math.nan))
            row.update({
                "collision_probability_left": collision_probabilities[0],
                "collision_probability_center": collision_probabilities[1],
                "collision_probability_right": collision_probabilities[2],
                "gate_affordance_none": affordance_probabilities[0],
                "gate_affordance_opening_left": affordance_probabilities[1],
                "gate_affordance_opening_right": affordance_probabilities[2],
                "rail_present_left": rail_probabilities[0],
                "rail_present_center": rail_probabilities[1],
                "rail_present_right": rail_probabilities[2],
                "pass_right_left": pass_right_probabilities[0],
                "pass_right_center": pass_right_probabilities[1],
                "pass_right_right": pass_right_probabilities[2],
                "rail_selected_sector": getattr(adapter, "last_rail_sector", -1),
                "square_opening_probability": getattr(
                    adapter, "last_square_opening_probability", math.nan),
                "normalized_yaw_rate": getattr(
                    adapter, "last_normalized_yaw_rate", math.nan),
                "scalar_collision_probability": getattr(
                    adapter, "last_collision_probability", math.nan),
            })
            row.update({f"clearance_{i}_m": prediction.clearance[i] for i in range(4)})
            row.update({f"confidence_{i}": prediction.confidence[i] for i in range(4)})
            row.update({f"danger_{i}": prediction_danger[i] for i in range(3)})
            row.update({f"raw_danger_{i}": raw_danger[i] for i in range(3)})
            for index, corner in enumerate(("tl", "tr", "br", "bl")):
                row[f"gate_{corner}_x"] = prediction.corners[index, 0]
                row[f"gate_{corner}_y"] = prediction.corners[index, 1]
            writer.writerow(row)
            stream.flush()
    video.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
