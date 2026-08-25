#!/usr/bin/env python3
"""Strict final acceptance gate for an instrumented 200-clip GVSOC harness."""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np


L1_CAPACITY_BYTES = 64_000
L2_CAPACITY_BYTES = 512_000
FLASH_CAPACITY_BYTES = 8 * 1024 * 1024
GATE_CONFIDENCE_THRESHOLD = 0.6174671283
BRIDGE_GATE_CONFIDENCE_THRESHOLD = 0.5


def sigmoid(values: np.ndarray) -> np.ndarray:
    return 1.0 / (1.0 + np.exp(-np.clip(values, -30.0, 30.0)))


def binary_counts(prediction: np.ndarray, visible: np.ndarray) -> dict:
    """Counts against the labeled gate-presence target, not model agreement."""
    prediction = np.asarray(prediction, dtype=bool)
    visible = np.asarray(visible, dtype=bool)
    return {
        "true_positive": int(np.sum(prediction & visible)),
        "false_positive": int(np.sum(prediction & ~visible)),
        "false_negative": int(np.sum(~prediction & visible)),
        "true_negative": int(np.sum(~prediction & ~visible)),
    }


def visible_recall(prediction: np.ndarray, visible: np.ndarray) -> float:
    positives = int(np.sum(visible))
    return (float(np.sum(np.asarray(prediction, dtype=bool) & visible) / positives)
            if positives else 1.0)


def bridge_gate_geometry(corners: np.ndarray) -> np.ndarray:
    """Exactly mirror JointGateRlAdapter._gate_geometry().

    The adapter first checks normalized bounds, then passes pixels to
    EspnetAdapter._gate_geometry().  Keeping this here makes the GVSOC check
    validate the decoder that actually produces observation.gate_valid.
    """
    corners = np.asarray(corners, dtype=np.float64)
    if corners.ndim != 3 or corners.shape[1:] != (4, 2):
        raise ValueError("gate corners must be [N,4,2]")
    result = np.zeros(len(corners), dtype=bool)
    for index, normalized in enumerate(corners):
        if (not np.all(np.isfinite(normalized)) or np.any(normalized < 0.0) or
                np.any(normalized > 1.0)):
            continue
        tl, tr, br, bl = normalized * 160.0
        if not (tl[0] < tr[0] and bl[0] < br[0] and
                tl[1] < bl[1] and tr[1] < br[1]):
            continue
        points = np.asarray((tl, tr, br, bl))
        turns = []
        for edge in range(4):
            first, second, third = (points[edge], points[(edge + 1) & 3],
                                    points[(edge + 2) & 3])
            turns.append((second[0] - first[0]) * (third[1] - second[1]) -
                         (second[1] - first[1]) * (third[0] - second[0]))
        if turns[0] == 0.0 or any(turn * turns[0] <= 0.0 for turn in turns):
            continue
        area = 0.5 * abs(float(
            np.dot(points[:, 0], np.roll(points[:, 1], -1)) -
            np.dot(points[:, 1], np.roll(points[:, 0], -1))))
        sides = np.linalg.norm(points - np.roll(points, -1, axis=0), axis=1)
        if area < 128.0 or np.min(sides) <= 0.0 or area > 23000.0:
            continue
        width = 0.5 * ((tr[0] - tl[0]) + (br[0] - bl[0]))
        height = 0.5 * ((bl[1] - tl[1]) + (br[1] - tr[1]))
        if height > 0.0 and 0.35 <= width / height <= 2.85:
            result[index] = True
    return result


def controller_gate_geometry(corners: np.ndarray) -> np.ndarray:
    """Exactly mirror gateObservationFreshAndGeometric() after freshness."""
    corners = np.asarray(corners, dtype=np.float64)
    if corners.ndim != 3 or corners.shape[1:] != (4, 2):
        raise ValueError("gate corners must be [N,4,2]")
    result = np.zeros(len(corners), dtype=bool)
    for index, point in enumerate(corners):
        signed_area_twice = sum(
            point[i, 0] * point[(i + 1) % 4, 1] -
            point[i, 1] * point[(i + 1) % 4, 0] for i in range(4))
        top = point[1] - point[0]
        right = point[2] - point[1]
        bottom = point[2] - point[3]
        left = point[3] - point[0]
        bottom_loop, left_loop = -bottom, -left
        diagonal_tl_br, diagonal_tr_bl = point[2] - point[0], point[3] - point[1]
        top_norm, right_norm, bottom_norm, left_norm, diagonal_a, diagonal_b = (
            np.linalg.norm(top), np.linalg.norm(right), np.linalg.norm(bottom),
            np.linalg.norm(left), np.linalg.norm(diagonal_tl_br),
            np.linalg.norm(diagonal_tr_bl))
        if (not np.isfinite(signed_area_twice) or abs(signed_area_twice) < 0.01 or
                min(top_norm, right_norm, bottom_norm, left_norm,
                    diagonal_a, diagonal_b) < 0.05):
            continue
        parallel_top_bottom = abs(float(np.dot(top / top_norm, bottom / bottom_norm)))
        parallel_left_right = abs(float(np.dot(left / left_norm, right / right_norm)))
        ratios = (top_norm / bottom_norm, left_norm / right_norm,
                  diagonal_a / diagonal_b)
        turns = (
            top[0] * right[1] - top[1] * right[0],
            right[0] * bottom_loop[1] - right[1] * bottom_loop[0],
            bottom_loop[0] * left_loop[1] - bottom_loop[1] * left_loop[0],
            left_loop[0] * top[1] - left_loop[1] * top[0])
        convex = all(turn > 0.0 for turn in turns) or all(turn < 0.0 for turn in turns)
        if (parallel_top_bottom >= 0.75 and parallel_left_right >= 0.75 and
                all(0.35 <= ratio <= 2.85 for ratio in ratios) and convex):
            result[index] = True
    return result


def runtime_gate_admission(corners: np.ndarray, confidence: np.ndarray) -> np.ndarray:
    """Mirror the joint bridge followed by the controller gate admission."""
    confidence = np.asarray(confidence, dtype=np.float64)
    bridge_valid = ((confidence >= BRIDGE_GATE_CONFIDENCE_THRESHOLD) &
                    bridge_gate_geometry(corners))
    return (bridge_valid & (confidence >= GATE_CONFIDENCE_THRESHOLD) &
            controller_gate_geometry(corners))


def semantic_metrics(decoded: np.ndarray, reference: np.ndarray,
                     visible: np.ndarray, target_corners: np.ndarray) -> dict:
    got_confident = sigmoid(decoded[:, 11]) >= GATE_CONFIDENCE_THRESHOLD
    ref_confident = sigmoid(reference[:, 11]) >= GATE_CONFIDENCE_THRESHOLD
    got_corners = sigmoid(decoded[:, 3:11]).reshape(-1, 4, 2) * 160.0
    ref_corners = sigmoid(reference[:, 3:11]).reshape(-1, 4, 2) * 160.0
    target = np.asarray(target_corners, dtype=np.float64) * 160.0
    positives = int(visible.sum())
    if positives:
        got_corner_error = float(np.linalg.norm(
            got_corners[visible] - target[visible], axis=2).mean())
        ref_corner_error = float(np.linalg.norm(
            ref_corners[visible] - target[visible], axis=2).mean())
        got_coordinate_mae = float(np.abs(got_corners[visible] - target[visible]).mean())
        ref_coordinate_mae = float(np.abs(ref_corners[visible] - target[visible]).mean())
    else:
        got_corner_error = ref_corner_error = got_coordinate_mae = ref_coordinate_mae = 0.0
    got_admitted = runtime_gate_admission(got_corners / 160.0, sigmoid(decoded[:, 11]))
    ref_admitted = runtime_gate_admission(ref_corners / 160.0, sigmoid(reference[:, 11]))
    return {
        "confidence_threshold": GATE_CONFIDENCE_THRESHOLD,
        "confidence_classification_agreement": float(
            np.mean(got_confident == ref_confident)),
        "float_confidence_counts": binary_counts(ref_confident, visible),
        "gvsoc_confidence_counts": binary_counts(got_confident, visible),
        "gvsoc_visible_gate_recall": visible_recall(got_confident, visible),
        "float_visible_gate_recall": visible_recall(ref_confident, visible),
        "visible_gate_recall_degradation": max(
            0.0, visible_recall(ref_confident, visible) - visible_recall(got_confident, visible)),
        "gvsoc_visible_per_corner_error_px": got_corner_error,
        "float_visible_per_corner_error_px": ref_corner_error,
        "visible_per_corner_error_increase_px": got_corner_error - ref_corner_error,
        "gvsoc_visible_corner_coordinate_mae_px": got_coordinate_mae,
        "float_visible_corner_coordinate_mae_px": ref_coordinate_mae,
        "visible_corner_coordinate_mae_increase_px": got_coordinate_mae - ref_coordinate_mae,
        "runtime_gate_admission": {
            "bridge_confidence_threshold": BRIDGE_GATE_CONFIDENCE_THRESHOLD,
            "controller_confidence_threshold": GATE_CONFIDENCE_THRESHOLD,
            "agreement": float(np.mean(got_admitted == ref_admitted)),
            "float_counts": binary_counts(ref_admitted, visible),
            "gvsoc_counts": binary_counts(got_admitted, visible),
            "float_visible_recall": visible_recall(ref_admitted, visible),
            "gvsoc_visible_recall": visible_recall(got_admitted, visible),
            "visible_recall_degradation": max(
                0.0, visible_recall(ref_admitted, visible) - visible_recall(got_admitted, visible)),
        },
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--semantic-corpus", type=Path, required=True,
                        help="The exact labeled calibration/parity NPZ executed by GVSOC.")
    parser.add_argument("--quantized-contract", type=Path, required=True)
    parser.add_argument("--gvsoc-corpus", type=Path, required=True,
                        help="NPZ from the generated-C harness: uint8 encoded[200,12], int64 cycles[200].")
    parser.add_argument("--memory-report", type=Path, required=True,
                        help="JSON emitted from generated GAP8 build/map with L1/L2 and complete-image flash accounting.")
    parser.add_argument("--model-provenance", type=Path, required=True,
                        help="Validated bundle/checkpoint/policy identity JSON.")
    parser.add_argument("--clock-hz", type=int, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.clock_hz <= 0:
        raise ValueError("GAP8 clock must be positive")
    contract = json.loads(args.quantized_contract.read_text())
    if contract.get("schema") != "tinympc-joint-gate-dory-quantized-v1":
        raise RuntimeError("unexpected quantized deployment contract")
    source = np.load(args.semantic_corpus, allow_pickle=False)
    gvsoc = np.load(args.gvsoc_corpus, allow_pickle=False)
    encoded = np.asarray(gvsoc["encoded"])
    cycles = np.asarray(gvsoc["cycles"])
    if encoded.dtype != np.uint8 or encoded.shape != (200, 12):
        raise RuntimeError("GVSOC harness must retain exactly uint8[200,12] outputs")
    if cycles.dtype.kind not in "iu" or cycles.shape != (200,) or np.any(cycles <= 0):
        raise RuntimeError("GVSOC harness must retain positive per-clip integer cycles[200]")
    scale = np.asarray(contract["terminal_affine"]["output_scale"], dtype=np.float64)
    shift = np.asarray(contract["terminal_affine"]["output_shift"], dtype=np.float64)
    # The generated integer ONNX determines epsilon; keeping it in the harness
    # avoids accidentally comparing a host float runtime to GVSOC output.
    epsilon = float(contract["terminal_quantization"]["terminal_epsilon"])
    corpus_epsilon = float(np.asarray(gvsoc["terminal_epsilon"]).item())
    if not np.isclose(corpus_epsilon, epsilon, rtol=0.0, atol=1.0e-12):
        raise RuntimeError("GVSOC corpus epsilon disagrees with quantized contract")
    decoded = (encoded.astype(np.float64) * epsilon - shift[None, :]) / scale[None, :]
    reference = np.asarray(source["reference_logits"], dtype=np.float64)
    agreement = float((decoded[:, :3].argmax(1) == reference[:, :3].argmax(1)).mean())
    visible = np.asarray(source["gate_visible"]).astype(bool)
    target = np.asarray(source["gate_corners"], dtype=np.float64).mean(1) * 160.0
    ref_center = sigmoid(reference[:, 3:11]).reshape(-1, 4, 2).mean(1) * 160.0
    got_center = sigmoid(decoded[:, 3:11]).reshape(-1, 4, 2).mean(1) * 160.0
    increase = (float(np.mean(np.linalg.norm(got_center[visible] - target[visible], axis=1))) -
                float(np.mean(np.linalg.norm(ref_center[visible] - target[visible], axis=1)))) if visible.any() else 0.0
    semantics = semantic_metrics(
        decoded, reference, visible, np.asarray(source["gate_corners"]))
    latency_ms = cycles.astype(np.float64) * 1000.0 / args.clock_hz
    memory = json.loads(args.memory_report.read_text())
    required_memory = {
        "l1_bytes", "l2_bytes", "deployment_flash_upper_bound_bytes"}
    if not required_memory <= set(memory):
        raise RuntimeError("generated build memory report missing %s" % sorted(required_memory))
    memory_values = {key: int(memory[key]) for key in sorted(required_memory)}
    memory_capacity = {
        "l1_bytes": L1_CAPACITY_BYTES, "l2_bytes": L2_CAPACITY_BYTES,
        "deployment_flash_upper_bound_bytes": FLASH_CAPACITY_BYTES,
    }
    memory_fits = all(memory_values[key] <= memory_capacity[key]
                      for key in required_memory)
    primary_performance_passed = (agreement >= .95 and increase <= 5.0 and
                          float(np.percentile(latency_ms, 95)) < 33.0)
    supplemental_semantics_passed = (
        semantics["confidence_classification_agreement"] >= .94 and
        semantics["visible_gate_recall_degradation"] <= .12 and
        semantics["visible_corner_coordinate_mae_increase_px"] <= 2.0 and
        semantics["runtime_gate_admission"]["agreement"] >= .94 and
        semantics["runtime_gate_admission"]["visible_recall_degradation"] <= .10 and
        semantics["runtime_gate_admission"]["gvsoc_counts"]["false_positive"] <=
        semantics["runtime_gate_admission"]["float_counts"]["false_positive"])
    performance_passed = primary_performance_passed and supplemental_semantics_passed
    provenance = json.loads(args.model_provenance.read_text())
    required_provenance = {
        "bundle_sha256", "checkpoint_sha256", "policy_onnx_sha256"}
    if not required_provenance <= set(provenance):
        raise RuntimeError("model provenance report is incomplete")
    report = {"clips": 200, "clock_hz": args.clock_hz,
              "cycles": {"p50": float(np.percentile(cycles, 50)), "p95": float(np.percentile(cycles, 95))},
              "inference_latency_ms": {"p50": float(np.percentile(latency_ms, 50)), "p95": float(np.percentile(latency_ms, 95))},
              "action_agreement": agreement, "visible_gate_center_error_increase_px": increase,
              "gate_semantics": semantics,
              "supplemental_post_audit_thresholds": {
                  "confidence_classification_agreement_minimum": .94,
                  "visible_gate_recall_degradation_maximum": .12,
                  "visible_corner_coordinate_mae_increase_px_maximum": 2.0,
                  "runtime_gate_admission_agreement_minimum": .94,
                  "runtime_gate_admission_visible_recall_degradation_maximum": .10,
                  "runtime_gate_admission_gvsoc_false_positives_no_greater_than_float": True,
              },
              "memory": memory_values, "memory_capacity": memory_capacity,
              "model_provenance": {key: provenance[key]
                                   for key in sorted(required_provenance)},
              "memory_fits": memory_fits,
              "primary_performance_passed": primary_performance_passed,
              "supplemental_semantics_passed": supplemental_semantics_passed,
              "performance_passed": performance_passed,
              "passed": performance_passed and memory_fits}
    args.output.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n")
    print(json.dumps(report, indent=2, sort_keys=True))
    if not report["passed"]:
        raise SystemExit("GVSOC acceptance gate failed")


if __name__ == "__main__":
    main()
