#!/usr/bin/env python3
"""Replay Himax frames and prototype probability/bounding-box obstacle logic.

The report preserves the deployed GAP8 threshold for comparison, but the
laptop proposal considers only yellow-or-hotter probability cells dangerous.
It boxes the target-containing connected component (or the largest component
when the target is safe) and selects the nearer viable vertical box edge as a
candidate image-space avoidance line.
"""

from __future__ import annotations

import argparse
import csv
import importlib.util
import json
import sys
import time
from pathlib import Path

import cv2
import numpy as np


APP_ROOT = Path(__file__).resolve().parents[1]
NANOCOCKPIT_ROOT = APP_ROOT.parents[2] / "tinympc-nanocockpit"
ADAPTER_PATH = NANOCOCKPIT_ROOT / "tools/stdc_shared_onnx_adapter.py"
DEFAULT_MODEL = (
    NANOCOCKPIT_ROOT / "gap8_stdc_release_shared_real_v1"
)
DEPLOYED_DANGER_Q_THRESHOLD = 42
YELLOW_DANGER_PROBABILITY = 0.60


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("capture", type=Path,
                        help="capture directory containing frames.csv, or image directory")
    parser.add_argument("--out", type=Path, default=None,
                        help="report directory (default: CAPTURE/danger_replay)")
    parser.add_argument("--model", type=Path, default=DEFAULT_MODEL,
                        help="deployed release root or integer model directory")
    parser.add_argument("--stride", type=int, default=1,
                        help="process every Nth captured frame")
    parser.add_argument("--limit", type=int, default=0,
                        help="maximum frames to process; 0 is unlimited")
    parser.add_argument("--q-threshold", type=int,
                        default=DEPLOYED_DANGER_Q_THRESHOLD,
                        help="integer GAP8 threshold; firmware default is the original q>=42")
    parser.add_argument("--danger-threshold", type=float,
                        default=YELLOW_DANGER_PROBABILITY,
                        help="proposed raw probability threshold; 0.60 is yellow in Turbo")
    parser.add_argument("--no-render", action="store_true",
                        help="do not save source/heatmap/mask diagnostic panels")
    return parser.parse_args()


def load_adapter():
    if not ADAPTER_PATH.is_file():
        raise SystemExit(
            f"Missing deployed-model adapter: {ADAPTER_PATH}. "
            "Keep tinympc-nanocockpit beside tinympc-crazyflie."
        )
    spec = importlib.util.spec_from_file_location(
        "tinympc_stdc_shared_adapter", ADAPTER_PATH
    )
    if spec is None or spec.loader is None:
        raise SystemExit(f"Could not load {ADAPTER_PATH}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def image_rows(capture: Path) -> list[dict[str, str]]:
    csv_path = capture / "frames.csv"
    if csv_path.is_file():
        with csv_path.open(newline="") as stream:
            rows = list(csv.DictReader(stream))
        for row in rows:
            row["_path"] = str(capture / row["image"])
        return rows
    patterns = ("*.pgm", "*.png", "*.jpg", "*.jpeg")
    paths = sorted(path for pattern in patterns for path in capture.glob(pattern))
    if not paths and (capture / "frames").is_dir():
        paths = sorted(
            path
            for pattern in patterns
            for path in (capture / "frames").glob(pattern)
        )
    return [{"image": str(path.relative_to(capture)), "_path": str(path)}
            for path in paths]


def connected_components(mask: np.ndarray) -> list[list[tuple[int, int]]]:
    height, width = mask.shape
    visited = np.zeros(mask.shape, dtype=bool)
    components: list[list[tuple[int, int]]] = []
    for seed_y in range(height):
        for seed_x in range(width):
            if not mask[seed_y, seed_x] or visited[seed_y, seed_x]:
                continue
            stack = [(seed_y, seed_x)]
            visited[seed_y, seed_x] = True
            component: list[tuple[int, int]] = []
            while stack:
                y, x = stack.pop()
                component.append((y, x))
                for next_y, next_x in (
                    (y - 1, x), (y + 1, x), (y, x - 1), (y, x + 1)
                ):
                    if (
                        0 <= next_y < height
                        and 0 <= next_x < width
                        and mask[next_y, next_x]
                        and not visited[next_y, next_x]
                    ):
                        visited[next_y, next_x] = True
                        stack.append((next_y, next_x))
            components.append(component)
    return components


def obstacle_box(
    mask: np.ndarray, target_y: int, target_x: int
) -> dict[str, int | str] | None:
    components = connected_components(mask)
    if not components:
        return None
    target_component = next(
        (
            component for component in components
            if (target_y, target_x) in component
        ),
        None,
    )
    selected = target_component or max(components, key=len)
    ys = [cell[0] for cell in selected]
    xs = [cell[1] for cell in selected]
    x0, x1 = min(xs), max(xs) + 1
    y0, y1 = min(ys), max(ys) + 1

    line_side = "none"
    line_grid_x = -1
    if target_component is not None:
        candidates: list[tuple[float, str, int]] = []
        target_center_x = target_x + 0.5
        if x0 > 0:
            candidates.append((target_center_x - x0, "left", x0))
        if x1 < mask.shape[1]:
            candidates.append((x1 - target_center_x, "right", x1))
        if candidates:
            _distance, line_side, line_grid_x = min(
                candidates, key=lambda candidate: (candidate[0], candidate[1])
            )
    return {
        "x0": x0, "y0": y0, "x1": x1, "y1": y1,
        "cells": len(selected),
        "source": "target" if target_component is not None else "largest",
        "line_side": line_side,
        "line_grid_x": line_grid_x,
    }


def flattened(values: np.ndarray, precision: int | None = None) -> str:
    if precision is None:
        return " ".join(str(int(value)) for value in values.reshape(-1))
    return " ".join(
        f"{float(value):.{precision}f}" for value in values.reshape(-1)
    )


def render_panel(
    gray: np.ndarray,
    probability: np.ndarray,
    unsafe: np.ndarray,
    box: dict[str, int | str] | None,
    crop_y_offset: int,
    danger_threshold: float,
    obstacle_cells: int,
    cluster: int,
    center_probability: float,
) -> np.ndarray:
    source = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
    height, width = gray.shape
    probability_large = cv2.resize(
        probability, (width, height), interpolation=cv2.INTER_NEAREST
    )
    heat = cv2.applyColorMap(
        np.clip(probability_large * 255.0, 0, 255).astype(np.uint8),
        cv2.COLORMAP_TURBO,
    )
    mask_large = cv2.resize(
        unsafe.astype(np.uint8), (width, height), interpolation=cv2.INTER_NEAREST
    )
    overlay = source.copy()
    overlay[mask_large > 0] = (0, 0, 255)
    overlay = cv2.addWeighted(source, 0.65, overlay, 0.35, 0)
    if box is not None:
        cell_width = width / probability.shape[1]
        crop_height = (
            height if height == probability.shape[0] * 15 else 120
        )
        cell_height = crop_height / probability.shape[0]
        x0 = int(round(int(box["x0"]) * cell_width))
        x1 = int(round(int(box["x1"]) * cell_width))
        y0 = int(round(crop_y_offset + int(box["y0"]) * cell_height))
        y1 = int(round(crop_y_offset + int(box["y1"]) * cell_height))
        cv2.rectangle(
            overlay, (x0, y0), (max(x0, x1 - 1), max(y0, y1 - 1)),
            (0, 255, 255), 2, cv2.LINE_AA,
        )
        line_grid_x = int(box["line_grid_x"])
        if line_grid_x >= 0:
            line_x = int(round(line_grid_x * cell_width))
            line_x = min(max(line_x, 0), width - 1)
            cv2.line(
                overlay, (line_x, crop_y_offset),
                (line_x, min(height - 1, crop_y_offset + crop_height - 1)),
                (255, 255, 0), 2, cv2.LINE_AA,
            )
    text = (
        f"cells={obstacle_cells}/80 cluster={cluster} "
        f"center={center_probability:.3f}"
    )
    for panel, label in (
        (source, "Himax input"),
        (heat, "raw danger probability"),
        (overlay, f"proposed p>={danger_threshold:.2f} box+edge"),
    ):
        cv2.rectangle(panel, (0, 0), (width, 34), (0, 0, 0), -1)
        cv2.putText(
            panel, label, (4, 13), cv2.FONT_HERSHEY_SIMPLEX,
            0.34, (255, 255, 255), 1, cv2.LINE_AA,
        )
        cv2.putText(
            panel, text, (4, 29), cv2.FONT_HERSHEY_SIMPLEX,
            0.30, (255, 255, 255), 1, cv2.LINE_AA,
        )
    return np.hstack((source, heat, overlay))


def render_timeline(
    report_rows: list[dict[str, object]],
    published_probability_threshold: float,
    proposed_probability_threshold: float,
) -> np.ndarray:
    width, height = 1100, 560
    left, right = 70, width - 25
    canvas = np.full((height, width, 3), 248, dtype=np.uint8)

    def x_position(index: int) -> int:
        denominator = max(len(report_rows) - 1, 1)
        return int(round(left + index * (right - left) / denominator))

    def draw_axes(top: int, bottom: int, maximum: float, label: str) -> None:
        cv2.line(canvas, (left, top), (left, bottom), (40, 40, 40), 1)
        cv2.line(canvas, (left, bottom), (right, bottom), (40, 40, 40), 1)
        for fraction in (0.0, 0.25, 0.5, 0.75, 1.0):
            y = int(round(bottom - fraction * (bottom - top)))
            cv2.line(canvas, (left, y), (right, y), (220, 220, 220), 1)
            cv2.putText(
                canvas, f"{maximum * fraction:g}", (8, y + 4),
                cv2.FONT_HERSHEY_SIMPLEX, 0.42, (70, 70, 70), 1,
                cv2.LINE_AA,
            )
        cv2.putText(
            canvas, label, (left, top - 10), cv2.FONT_HERSHEY_SIMPLEX,
            0.52, (30, 30, 30), 1, cv2.LINE_AA,
        )

    def points(key: str, top: int, bottom: int, maximum: float) -> np.ndarray:
        result = []
        for index, row in enumerate(report_rows):
            value = min(max(float(row[key]), 0.0), maximum)
            y = int(round(bottom - value * (bottom - top) / maximum))
            result.append((x_position(index), y))
        return np.asarray(result, dtype=np.int32).reshape(-1, 1, 2)

    draw_axes(
        55, 285, 80.0,
        f"Proposed yellow threshold p>={proposed_probability_threshold:.2f}",
    )
    cv2.polylines(
        canvas, [points("obstacle_cells", 55, 285, 80.0)], False,
        (210, 80, 20), 2, cv2.LINE_AA,
    )
    cv2.polylines(
        canvas, [points("largest_cluster", 55, 285, 80.0)], False,
        (20, 130, 230), 2, cv2.LINE_AA,
    )
    cv2.putText(
        canvas, "obstacle cells", (left + 10, 78),
        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (210, 80, 20), 1, cv2.LINE_AA,
    )
    cv2.putText(
        canvas, "largest cluster", (left + 150, 78),
        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (20, 130, 230), 1, cv2.LINE_AA,
    )

    probability_top, probability_bottom = 350, 520
    draw_axes(
        probability_top, probability_bottom, 1.0,
        "Continuous network probability before GAP8 binarization",
    )
    cv2.polylines(
        canvas,
        [points("max_probability", probability_top, probability_bottom, 1.0)],
        False, (160, 50, 170), 2, cv2.LINE_AA,
    )
    cv2.polylines(
        canvas,
        [points("center_probability", probability_top, probability_bottom, 1.0)],
        False, (20, 155, 40), 2, cv2.LINE_AA,
    )
    threshold_y = int(round(
        probability_bottom
        - proposed_probability_threshold * (probability_bottom - probability_top)
    ))
    cv2.line(
        canvas, (left, threshold_y), (right, threshold_y),
        (80, 80, 80), 1, cv2.LINE_AA,
    )
    cv2.putText(
        canvas, "max", (left + 10, probability_top + 22),
        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (160, 50, 170), 1, cv2.LINE_AA,
    )
    cv2.putText(
        canvas, "center", (left + 75, probability_top + 22),
        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (20, 155, 40), 1, cv2.LINE_AA,
    )
    cv2.putText(
        canvas, f"proposed p threshold={proposed_probability_threshold:.2f}",
        (right - 215, threshold_y - 5), cv2.FONT_HERSHEY_SIMPLEX,
        0.40, (80, 80, 80), 1, cv2.LINE_AA,
    )
    published_y = int(round(
        probability_bottom
        - published_probability_threshold
        * (probability_bottom - probability_top)
    ))
    cv2.line(
        canvas, (left, published_y), (right, published_y),
        (170, 170, 170), 1, cv2.LINE_AA,
    )
    cv2.putText(
        canvas, f"old published p={published_probability_threshold:.3f}",
        (right - 190, published_y - 5), cv2.FONT_HERSHEY_SIMPLEX,
        0.38, (130, 130, 130), 1, cv2.LINE_AA,
    )
    cv2.putText(
        canvas, f"frame 1", (left, height - 12),
        cv2.FONT_HERSHEY_SIMPLEX, 0.42, (70, 70, 70), 1, cv2.LINE_AA,
    )
    cv2.putText(
        canvas, f"frame {len(report_rows)}", (right - 85, height - 12),
        cv2.FONT_HERSHEY_SIMPLEX, 0.42, (70, 70, 70), 1, cv2.LINE_AA,
    )
    return canvas


def main() -> None:
    args = parse_args()
    if args.stride < 1:
        raise SystemExit("--stride must be at least 1")
    if not 0 <= args.q_threshold <= 255:
        raise SystemExit("--q-threshold must be in [0, 255]")
    if not 0.0 <= args.danger_threshold <= 1.0:
        raise SystemExit("--danger-threshold must be in [0, 1]")
    rows = image_rows(args.capture)
    if not rows:
        raise SystemExit(f"No captured images found under {args.capture}")
    selected = rows[::args.stride]
    if args.limit:
        selected = selected[:args.limit]

    adapter = load_adapter()
    model = adapter.load_model(str(args.model))
    manifest = model["manifest"]
    probability_threshold = float(manifest["danger_probability_threshold"])
    out = args.out or args.capture / "danger_replay"
    if out.exists() and any(out.iterdir()):
        raise SystemExit(f"Refusing to overwrite non-empty output directory: {out}")
    panels = out / "panels"
    out.mkdir(parents=True, exist_ok=True)
    if not args.no_render:
        panels.mkdir()

    sessions = model["sessions"]
    report_fields = [
        "replay_index", "image", "frame_id", "host_monotonic_s",
        "frame_gap8_timestamp", "inference_ms", "max_q", "max_probability",
        "mean_probability", "center_q", "center_probability",
        "center_unsafe", "obstacle_cells", "largest_cluster",
        "deployed_obstacle_cells", "deployed_largest_cluster",
        "box_source", "box_cells", "box_x0", "box_y0", "box_x1", "box_y1",
        "avoidance_side", "line_grid_x", "line_pixel_x",
        "q_map_8x10", "probability_map_8x10", "unsafe_map_8x10",
    ]
    report_rows: list[dict[str, object]] = []
    with (out / "danger.csv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=report_fields)
        writer.writeheader()
        for index, source in enumerate(selected, start=1):
            path = Path(source["_path"])
            gray = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
            if gray is None:
                raise SystemExit(f"Could not decode {path}")
            crop, display_y_offset = adapter._center_crop(gray)
            started = time.perf_counter()
            encoder = adapter._run(
                sessions["encoder"], crop.astype(np.float32)[None, None]
            )
            danger_q = adapter._quantized(
                adapter._run(sessions["danger_head"], encoder)[0, 0]
            )
            probability = adapter._probability(
                danger_q, manifest["integer_affine"]["danger"]
            )
            inference_ms = (time.perf_counter() - started) * 1000.0

            deployed_unsafe = danger_q >= args.q_threshold
            unsafe = probability >= args.danger_threshold
            cells = int(np.count_nonzero(unsafe))
            components = connected_components(unsafe)
            cluster = max((len(component) for component in components), default=0)
            deployed_components = connected_components(deployed_unsafe)
            deployed_cells = int(np.count_nonzero(deployed_unsafe))
            deployed_cluster = max(
                (len(component) for component in deployed_components), default=0
            )
            center_y, center_x = danger_q.shape[0] // 2, danger_q.shape[1] // 2
            center_q = int(danger_q[center_y, center_x])
            center_probability = float(probability[center_y, center_x])
            box = obstacle_box(unsafe, center_y, center_x)
            line_grid_x = -1 if box is None else int(box["line_grid_x"])
            line_pixel_x = (
                -1 if line_grid_x < 0
                else int(round(line_grid_x * crop.shape[1] / unsafe.shape[1]))
            )
            result = {
                "replay_index": index,
                "image": source.get("image", path.name),
                "frame_id": source.get("frame_id", ""),
                "host_monotonic_s": source.get("host_monotonic_s", ""),
                "frame_gap8_timestamp": source.get("frame_gap8_timestamp", ""),
                "inference_ms": f"{inference_ms:.3f}",
                "max_q": int(np.max(danger_q)),
                "max_probability": f"{float(np.max(probability)):.6f}",
                "mean_probability": f"{float(np.mean(probability)):.6f}",
                "center_q": center_q,
                "center_probability": f"{center_probability:.6f}",
                "center_unsafe": int(unsafe[center_y, center_x]),
                "obstacle_cells": cells,
                "largest_cluster": cluster,
                "deployed_obstacle_cells": deployed_cells,
                "deployed_largest_cluster": deployed_cluster,
                "box_source": "none" if box is None else box["source"],
                "box_cells": 0 if box is None else box["cells"],
                "box_x0": -1 if box is None else box["x0"],
                "box_y0": -1 if box is None else box["y0"],
                "box_x1": -1 if box is None else box["x1"],
                "box_y1": -1 if box is None else box["y1"],
                "avoidance_side": "none" if box is None else box["line_side"],
                "line_grid_x": line_grid_x,
                "line_pixel_x": line_pixel_x,
                "q_map_8x10": flattened(danger_q),
                "probability_map_8x10": flattened(probability, precision=6),
                "unsafe_map_8x10": flattened(unsafe),
            }
            writer.writerow(result)
            report_rows.append(result)

            if not args.no_render:
                panel = render_panel(
                    gray, probability, unsafe, box, display_y_offset,
                    args.danger_threshold, cells, cluster,
                    center_probability,
                )
                cv2.imwrite(str(panels / f"frame_{index:06d}.png"), panel)
            if index == 1 or index % 30 == 0:
                print(
                    f"  {index}/{len(selected)}: cells={cells:2d}/80 "
                    f"cluster={cluster:2d} old_cells={deployed_cells:2d} "
                    f"center={center_probability:.3f}",
                    flush=True,
                )

    counts = np.asarray(
        [int(row["obstacle_cells"]) for row in report_rows], dtype=np.int32
    )
    clusters = np.asarray(
        [int(row["largest_cluster"]) for row in report_rows], dtype=np.int32
    )
    center_hits = sum(int(row["center_unsafe"]) for row in report_rows)
    summary = {
        "format": "tinympc-handheld-danger-replay-v1",
        "capture": str(args.capture),
        "model": str(args.model),
        "model_checkpoint": model["checkpoint"],
        "processed_frames": len(report_rows),
        "stride": args.stride,
        "deployed_integer_threshold": args.q_threshold,
        "published_probability_threshold": probability_threshold,
        "proposed_probability_threshold": args.danger_threshold,
        "map_shape": [8, 10],
        "controller_mapping": (
            "These 80 cells become rows 1-8 of the controller 10x10 map; "
            "controller rows 0 and 9 are forced unsafe and excluded from "
            "perceptMass.cells."
        ),
        "obstacle_cells": {
            "minimum": int(np.min(counts)),
            "median": float(np.median(counts)),
            "maximum": int(np.max(counts)),
            "frames_nonzero": int(np.count_nonzero(counts)),
        },
        "deployed_obstacle_cells": {
            "minimum": min(int(row["deployed_obstacle_cells"])
                           for row in report_rows),
            "median": float(np.median([
                int(row["deployed_obstacle_cells"]) for row in report_rows
            ])),
            "maximum": max(int(row["deployed_obstacle_cells"])
                           for row in report_rows),
        },
        "largest_cluster": {
            "minimum": int(np.min(clusters)),
            "median": float(np.median(clusters)),
            "maximum": int(np.max(clusters)),
        },
        "center_unsafe_frames": center_hits,
    }
    (out / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    cv2.imwrite(
        str(out / "timeline.png"),
        render_timeline(
            report_rows, probability_threshold, args.danger_threshold
        ),
    )
    print(
        f"Processed {len(report_rows)} frames. "
        f"Peak cells={int(np.max(counts))}/80, "
        f"peak cluster={int(np.max(clusters))}; report: {out}"
    )


if __name__ == "__main__":
    main()
