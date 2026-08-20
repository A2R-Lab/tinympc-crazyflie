#!/usr/bin/env python3
"""Run a repository-local ONNX vision head on CrazySim FPV frames.

The bridge deliberately carries metric clearance, collision risk, navigation,
and gate geometry as different signals. It sends the same versioned packet
that the AI deck can send to the STM32 firmware.
"""

from __future__ import annotations

import argparse
import csv
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
VISION_BODY = struct.Struct("<4sIHH4f4f4f2f8f1f")
VISION_MAGIC = b"\x90\x19\x08\x39"
FLAG_METRIC = 1 << 0
FLAG_DANGER = 1 << 1
FLAG_GATE = 1 << 2
FLAG_NAVIGATION = 1 << 3
SECTOR_BEARING = np.asarray([1.0, 1.0 / 3.0, -1.0 / 3.0, -1.0])


@dataclass
class Prediction:
    metric: bool
    sector_danger: bool
    clearance: np.ndarray
    confidence: np.ndarray
    danger: np.ndarray
    steering: float
    collision: float
    gate_valid: bool
    corners: np.ndarray
    gate_confidence: float
    gate_reason: str


def sigmoid(value):
    return 1.0 / (1.0 + np.exp(-np.clip(value, -60.0, 60.0)))


def resize_nearest(frame: np.ndarray, height: int, width: int) -> np.ndarray:
    ys = np.minimum((np.arange(height) * frame.shape[0] / height).astype(int),
                    frame.shape[0] - 1)
    xs = np.minimum((np.arange(width) * frame.shape[1] / width).astype(int),
                    frame.shape[1] - 1)
    return frame[np.ix_(ys, xs)]


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
        cross.append(np.cross(b - a, c - b))
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


class SequentialAdapter:
    def __init__(self, model_path: Path, clearance_threshold: float):
        import onnxruntime as ort
        self.session = ort.InferenceSession(str(model_path), providers=["CPUExecutionProvider"])
        self.input_name = self.session.get_inputs()[0].name
        manifest_path = model_path.with_name("quantization_manifest.json")
        if not manifest_path.is_file():
            raise FileNotFoundError(f"missing {manifest_path}")
        self.scale = float(json.loads(manifest_path.read_text())["scale"])
        self.threshold = clearance_threshold
        self.previous_steering = 0.0

    def predict(self, frame: np.ndarray) -> Prediction:
        crop = frame if frame.shape == (120, 160) else frame[20:140]
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
        corners = []
        corner_scores = []
        for heatmap in logical[:4]:
            y, x = np.unravel_index(np.argmax(heatmap), heatmap.shape)
            corners.append(((x + 0.5) / heatmap.shape[1],
                            (y + 0.5) / heatmap.shape[0]))
            corner_scores.append(float(sigmoid(heatmap[y, x])))
        corners = np.asarray(corners, dtype=np.float32)
        gate_valid, reason = validate_gate(corners)
        gate_confidence = float(np.mean(corner_scores))
        gate_valid &= gate_confidence >= 0.25
        return Prediction(True, True, clearance, confidence, danger, steering, collision,
                          gate_valid, corners, gate_confidence, reason)


class StdcAdapter:
    CORNER_Q_THRESHOLDS = np.asarray([136, 141, 187, 131], dtype=np.float32)

    def __init__(self, release: Path):
        import onnxruntime as ort
        integer = release / "integer" if (release / "integer").is_dir() else release
        manifest_path = integer.parent / "nanocockpit" / "manifest.json"
        self.manifest = json.loads(manifest_path.read_text())
        self.sessions = {
            name: ort.InferenceSession(str(integer / f"{name}_int.onnx"),
                                       providers=["CPUExecutionProvider"])
            for name in ("encoder", "corner_head", "danger_head")
        }
        self.previous_steering = 0.0

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
        crop = frame if frame.shape == (120, 160) else frame[20:140]
        encoded = self.run("encoder", crop.astype(np.float32)[None, None])
        corner_q = np.clip(np.rint(self.run("corner_head", encoded)[0]), 0, 255)
        danger_q = np.clip(np.rint(self.run("danger_head", encoded)[0, 0]), 0, 255)
        corner_probability = self.probability(corner_q, "corner")
        danger_map = self.probability(danger_q, "danger")
        danger = np.asarray([np.max(part) for part in np.array_split(danger_map, 4, axis=1)])
        steering, collision = navigation_from_sectors(danger, self.previous_steering)
        self.previous_steering = steering
        corners = []
        scores = []
        confident = np.max(corner_q, axis=(1, 2)) >= self.CORNER_Q_THRESHOLDS
        for heatmap in corner_probability:
            y, x = np.unravel_index(np.argmax(heatmap), heatmap.shape)
            corners.append(((x + 0.5) / heatmap.shape[1],
                            (y + 0.5) / heatmap.shape[0]))
            scores.append(float(heatmap[y, x]))
        corners = np.asarray(corners, dtype=np.float32)
        if int(np.sum(confident)) == 3:
            missing = int(np.flatnonzero(~confident)[0])
            corners[missing] = (corners[(missing + 3) & 3] +
                                corners[(missing + 1) & 3] -
                                corners[(missing + 2) & 3])
        gate_valid, reason = validate_gate(corners)
        gate_valid &= int(np.sum(confident)) >= 3
        gate_confidence = float(np.mean(scores))
        return Prediction(False, True, np.full(4, 6.0), np.zeros(4), danger,
                          steering, collision, gate_valid, corners,
                          gate_confidence, reason)


class DronetAdapter:
    """Adapter for a future two-output steering/collision ONNX head."""
    def __init__(self, model_path: Path):
        import onnxruntime as ort
        self.session = ort.InferenceSession(str(model_path), providers=["CPUExecutionProvider"])
        self.input = self.session.get_inputs()[0]

    def predict(self, frame: np.ndarray) -> Prediction:
        shape = self.input.shape
        height = int(shape[-2]) if isinstance(shape[-2], int) else 120
        width = int(shape[-1]) if isinstance(shape[-1], int) else 160
        image = resize_nearest(frame, height, width).astype(np.float32) / 255.0
        outputs = self.session.run(None, {self.input.name: image[None, None]})
        values = [float(np.asarray(output).reshape(-1)[0]) for output in outputs]
        if len(values) < 2:
            raise ValueError("dronet adapter needs separate steering and collision outputs")
        steering = float(np.clip(values[0], -1.0, 1.0))
        collision = values[1] if 0.0 <= values[1] <= 1.0 else float(sigmoid(values[1]))
        danger = np.full(4, collision, dtype=float)
        return Prediction(False, False, np.full(4, 6.0), np.zeros(4), danger,
                          steering, collision, False, np.zeros((4, 2)), 0.0,
                          "not_provided")


def make_adapter(kind: str, model: Path, threshold: float):
    if kind == "auto":
        kind = "stdc" if model.is_dir() else "sequential"
    if kind == "sequential":
        return SequentialAdapter(model, threshold), kind
    if kind == "stdc":
        return StdcAdapter(model), kind
    if kind == "dronet":
        return DronetAdapter(model), kind
    raise ValueError(kind)


def packet_bytes(prediction: Prediction, sequence: int, timestamp_ms: int) -> bytes:
    flags = FLAG_NAVIGATION
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
    )
    return body + struct.pack("<I", zlib.crc32(body) & 0xFFFFFFFF)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", required=True, type=Path)
    parser.add_argument("--adapter", choices=("auto", "sequential", "stdc", "dronet"),
                        default="auto")
    parser.add_argument("--camera-port", type=int, default=5200)
    parser.add_argument("--firmware-port", type=int, default=19960)
    parser.add_argument("--camera-fps", type=float, default=20.0)
    parser.add_argument("--clearance-threshold", type=float, default=0.30)
    parser.add_argument("--log", required=True, type=Path)
    args = parser.parse_args()
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
    fields = ["time_s", "sequence", "adapter", "inference_ms", "steering",
              "collision", "metric", "spatial_danger", "gate_valid", "gate_confidence", "gate_reason"] + [
              f"clearance_{i}_m" for i in range(4)] + [
              f"confidence_{i}" for i in range(4)] + [f"danger_{i}" for i in range(4)] + [
              f"gate_{corner}_{axis}" for corner in ("tl", "tr", "br", "bl")
              for axis in ("x", "y")]
    sequence = 0
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
            if len(datagram) < CAMERA_HEADER.size:
                continue
            chunk, total, width, height = CAMERA_HEADER.unpack_from(datagram)
            if chunk != 0 or total != 1:
                raise RuntimeError("use 160x120 camera frames (one UDP chunk)")
            pixels = np.frombuffer(datagram[CAMERA_HEADER.size:], dtype=np.uint8)
            if pixels.size != width * height:
                continue
            frame = pixels.reshape(height, width)
            inference_start = time.perf_counter()
            prediction = adapter.predict(frame)
            inference_ms = 1000.0 * (time.perf_counter() - inference_start)
            sequence = sequence % 65535 + 1
            timestamp_ms = int(round(1000.0 * sequence / args.camera_fps))
            sender.sendto(packet_bytes(prediction, sequence, timestamp_ms),
                          ("127.0.0.1", args.firmware_port))
            row = {
                "time_s": timestamp_ms / 1000.0, "sequence": sequence,
                "adapter": adapter_name, "inference_ms": inference_ms,
                "steering": prediction.steering, "collision": prediction.collision,
                "metric": int(prediction.metric), "gate_valid": int(prediction.gate_valid),
                "spatial_danger": int(prediction.sector_danger),
                "gate_confidence": prediction.gate_confidence,
                "gate_reason": prediction.gate_reason,
            }
            row.update({f"clearance_{i}_m": prediction.clearance[i] for i in range(4)})
            row.update({f"confidence_{i}": prediction.confidence[i] for i in range(4)})
            row.update({f"danger_{i}": prediction.danger[i] for i in range(4)})
            for index, corner in enumerate(("tl", "tr", "br", "bl")):
                row[f"gate_{corner}_x"] = prediction.corners[index, 0]
                row[f"gate_{corner}_y"] = prediction.corners[index, 1]
            writer.writerow(row)
            stream.flush()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
