#!/usr/bin/env python3
"""Offline sparse-flow inverse-depth harness for sim or hardware datasets."""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path

import numpy as np


R_CB = np.asarray(
    [
        [0.0, -1.0, 0.0],
        [0.0, 0.0, -1.0],
        [1.0, 0.0, 0.0],
    ],
    dtype=np.float64,
)


def parse_args() -> argparse.Namespace:
    ap = argparse.ArgumentParser()
    ap.add_argument("dataset", type=Path)
    ap.add_argument("--out", type=Path, default=None)
    ap.add_argument("--max-corners", type=int, default=80)
    ap.add_argument("--quality", type=float, default=0.01)
    ap.add_argument("--min-distance", type=float, default=5.0)
    ap.add_argument("--fb-thresh-px", type=float, default=0.75)
    ap.add_argument("--min-depth", type=float, default=0.15)
    ap.add_argument("--max-depth", type=float, default=8.0)
    ap.add_argument("--min-a-norm", type=float, default=1e-3)
    ap.add_argument("--sectors", type=int, default=7)
    ap.add_argument("--redetect-every", type=int, default=15,
                    help="replenish Shi-Tomasi tracks every N frame pairs; 0 disables periodic replenishment")
    ap.add_argument("--min-track-count", type=int, default=40,
                    help="replenish when surviving persistent tracks fall below this count")
    ap.add_argument("--depth-smooth-window", type=int, default=5,
                    help="median window per track for depth visualization/sector aggregation")
    ap.add_argument("--depth-hold-frames", type=int, default=4,
                    help="reuse a recent smoothed depth this many frames when raw depth flickers")
    ap.add_argument("--near-depth-thresh", type=float, default=0.0,
                    help="enable near-range smoothing/hold settings below this depth; <=0 disables")
    ap.add_argument("--near-depth-smooth-window", type=int, default=None,
                    help="median window below --near-depth-thresh; defaults to --depth-smooth-window")
    ap.add_argument("--near-depth-hold-frames", type=int, default=None,
                    help="hold frames below --near-depth-thresh; defaults to --depth-hold-frames")
    ap.add_argument("--depth-jump-ratio", type=float, default=2.5,
                    help="treat larger frame-to-frame depth ratio jumps as flicker when history exists")
    ap.add_argument("--depth-jump-up-ratio", type=float, default=None,
                    help="suppress raw depth increases above this ratio; defaults to --depth-jump-ratio")
    ap.add_argument("--depth-jump-down-ratio", type=float, default=None,
                    help="suppress raw depth decreases above this ratio; defaults to --depth-jump-ratio")
    ap.add_argument("--max-model-residual", type=float, default=0.25,
                    help="reject raw depths whose 2D flow model residual exceeds this normalized-flow rate")
    ap.add_argument("--min-flow-cos", type=float, default=-1.0,
                    help="directional gate: require cos(derot_flow, A*v) >= this; -1 disables")
    ap.add_argument("--min-flow-norm", type=float, default=1e-4,
                    help="minimum derotated flow norm before directional gating is applied")
    ap.add_argument("--sector-mad-thresh", type=float, default=10.0,
                    help="reject sector depth outliers beyond this scaled MAD; <=0 disables")
    ap.add_argument("--sector-min-inliers", type=int, default=4,
                    help="minimum sector feature count before MAD filtering is applied")
    ap.add_argument("--visualize", action="store_true")
    return ap.parse_args()


def main() -> int:
    args = parse_args()
    try:
        import cv2
    except ImportError as exc:
        raise SystemExit("offline harness requires opencv-python") from exc

    dataset = args.dataset
    out = args.out or (dataset / "flow_harness")
    out.mkdir(parents=True, exist_ok=True)
    overlay_dir = out / "overlays"
    if args.visualize:
        overlay_dir.mkdir(parents=True, exist_ok=True)
        (overlay_dir / "all_tracks").mkdir(parents=True, exist_ok=True)
        (overlay_dir / "depth_tracks").mkdir(parents=True, exist_ok=True)

    calib = _load_calibration(dataset)
    frames = _read_csv(dataset / "frames.csv")
    telemetry = _read_csv(dataset / "telemetry.csv")
    if len(frames) < 2:
        raise SystemExit("dataset needs at least two frames")

    tracks_rows: list[list[object]] = []
    sector_rows: list[list[object]] = []
    sector_raw_rows: list[list[object]] = []
    rejection_rows: list[list[object]] = []
    depth_history: dict[int, list[float]] = {}
    depth_last_seen: dict[int, int] = {}

    prev_gray = _load_gray(cv2, dataset / frames[0]["filename"], calib)
    prev_truth_depth = _load_truth_depth(dataset, frames[0])
    prev_t = float(frames[0]["t"])
    prev_tel = _interp_telemetry(telemetry, prev_t)
    prev_points = _detect_features(cv2, prev_gray, args)
    track_ids = np.arange(0, len(prev_points), dtype=np.int64)
    next_track_id = int(len(prev_points))

    for pair_idx, frame in enumerate(frames[1:], start=1):
        cur_t = float(frame["t"])
        dt = cur_t - prev_t
        if dt <= 1e-6:
            prev_t = cur_t
            prev_gray = _load_gray(cv2, dataset / frame["filename"], calib)
            prev_truth_depth = _load_truth_depth(dataset, frame)
            prev_tel = _interp_telemetry(telemetry, cur_t)
            continue

        cur_gray = _load_gray(cv2, dataset / frame["filename"], calib)
        cur_tel = _interp_telemetry(telemetry, cur_t)
        if len(prev_points) == 0:
            prev_points = _detect_features(cv2, prev_gray, args)
            track_ids = np.arange(next_track_id, next_track_id + len(prev_points), dtype=np.int64)
            next_track_id += int(len(prev_points))
        if len(prev_points) == 0:
            prev_gray, prev_t, prev_tel = cur_gray, cur_t, cur_tel
            continue

        next_pts, status, err = cv2.calcOpticalFlowPyrLK(
            prev_gray,
            cur_gray,
            prev_points.reshape(-1, 1, 2).astype(np.float32),
            None,
            winSize=(15, 15),
            maxLevel=2,
            criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 20, 0.03),
        )
        back_pts, back_status, _ = cv2.calcOpticalFlowPyrLK(
            cur_gray,
            prev_gray,
            next_pts,
            None,
            winSize=(15, 15),
            maxLevel=2,
            criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 20, 0.03),
        )

        p0 = prev_points.reshape(-1, 2)
        p1 = next_pts.reshape(-1, 2)
        pb = back_pts.reshape(-1, 2)
        ok = (status.reshape(-1) == 1) & (back_status.reshape(-1) == 1)
        fb_err = np.linalg.norm(pb - p0, axis=1)
        ok &= fb_err <= float(args.fb_thresh_px)

        q_mid = _slerp_quat(
            np.asarray([prev_tel["qx"], prev_tel["qy"], prev_tel["qz"], prev_tel["qw"]], dtype=np.float64),
            np.asarray([cur_tel["qx"], cur_tel["qy"], cur_tel["qz"], cur_tel["qw"]], dtype=np.float64),
            0.5,
        )
        rot_wb = _quat_xyzw_to_rot(q_mid)
        v_world = 0.5 * (_vec(prev_tel, "v") + _vec(cur_tel, "v"))
        gyro_body = 0.5 * (_vec(prev_tel, "gyro_") + _vec(cur_tel, "gyro_"))
        v_cam = R_CB @ (rot_wb.T @ v_world)
        gyro_cam = R_CB @ gyro_body

        pair_depths: list[tuple[float, float, float, float, float, float, float, int, float, str]] = []
        pair_raw_depths: list[tuple[float, float, float, float, float, float, float, int, float, str]] = []
        raw_counts = {
            "lk_rejected": int(len(ok) - int(np.count_nonzero(ok))),
            "low_a": 0,
            "bad_rho": 0,
            "out_of_range": 0,
            "valid_raw": 0,
            "jump_suppressed": 0,
            "direction": 0,
            "model_residual": 0,
            "sector_outlier": 0,
            "held": 0,
            "dropped_no_hold": 0,
        }
        for j, (xy0, xy1, good) in enumerate(zip(p0, p1, ok)):
            track_id = int(track_ids[j])
            if not good:
                continue
            true_depth = _sample_depth(prev_truth_depth, xy0)
            x0, y0 = _pixel_to_norm(xy0, calib)
            x1, y1 = _pixel_to_norm(xy1, calib)
            flow = np.asarray([(x1 - x0) / dt, (y1 - y0) / dt], dtype=np.float64)
            a_vec = np.asarray([-v_cam[0] + x0 * v_cam[2], -v_cam[1] + y0 * v_cam[2]], dtype=np.float64)
            b_omega = np.asarray(
                [
                    x0 * y0 * gyro_cam[0] - (1.0 + x0 * x0) * gyro_cam[1] + y0 * gyro_cam[2],
                    (1.0 + y0 * y0) * gyro_cam[0] - x0 * y0 * gyro_cam[1] - x0 * gyro_cam[2],
                ],
                dtype=np.float64,
            )
            flow_derot = flow - b_omega
            a2 = float(np.dot(a_vec, a_vec))
            raw_depth = math.nan
            model_residual = math.nan
            flow_cos = math.nan
            source = "raw"
            if a2 < float(args.min_a_norm) ** 2:
                raw_counts["low_a"] += 1
                smoothed = _held_depth(depth_history, depth_last_seen, track_id, pair_idx, args)
                if smoothed is None:
                    raw_counts["dropped_no_hold"] += 1
                    continue
                depth = smoothed
                source = "held_low_a"
                raw_counts["held"] += 1
                rho = 1.0 / depth
            else:
                flow_norm = float(np.linalg.norm(flow_derot))
                a_norm = math.sqrt(a2)
                if float(args.min_flow_cos) > -1.0 and flow_norm >= float(args.min_flow_norm):
                    flow_cos = float(np.dot(flow_derot, a_vec) / max(1e-12, flow_norm * a_norm))
                    if flow_cos < float(args.min_flow_cos):
                        raw_counts["direction"] += 1
                        smoothed = _held_depth(depth_history, depth_last_seen, track_id, pair_idx, args)
                        if smoothed is None:
                            raw_counts["dropped_no_hold"] += 1
                            continue
                        depth = smoothed
                        source = "held_direction"
                        raw_counts["held"] += 1
                        rho = 1.0 / depth
                        azimuth = math.atan2(x0, 1.0)
                        ttc = depth / max(0.05, float(v_cam[2]))
                        pair_depths.append(
                            (azimuth, depth, ttc, x0, y0, float(xy0[0]), float(xy0[1]),
                             track_id, raw_depth, source)
                        )
                        tracks_rows.append(
                            [
                                pair_idx,
                                f"{cur_t:.9f}",
                                track_id,
                                f"{xy0[0]:.6f}",
                                f"{xy0[1]:.6f}",
                                f"{xy1[0]:.6f}",
                                f"{xy1[1]:.6f}",
                                f"{x0:.9f}",
                                f"{y0:.9f}",
                                f"{flow[0]:.9f}",
                                f"{flow[1]:.9f}",
                                f"{flow_derot[0]:.9f}",
                                f"{flow_derot[1]:.9f}",
                                f"{rho:.9f}",
                                "",
                                f"{depth:.9f}",
                                f"{ttc:.9f}",
                                f"{fb_err[j]:.6f}",
                                f"{a_norm:.9f}",
                                "",
                                f"{flow_cos:.9f}",
                                "" if not math.isfinite(true_depth) else f"{true_depth:.9f}",
                                "" if not math.isfinite(true_depth) else f"{depth - true_depth:.9f}",
                                source,
                            ]
                        )
                        continue
                rho = float(np.dot(a_vec, flow_derot) / (a2 + 1e-8))
                if rho <= 0.0 or not math.isfinite(rho):
                    raw_counts["bad_rho"] += 1
                    smoothed = _held_depth(depth_history, depth_last_seen, track_id, pair_idx, args)
                    if smoothed is None:
                        raw_counts["dropped_no_hold"] += 1
                        continue
                    depth = smoothed
                    source = "held_bad_rho"
                    raw_counts["held"] += 1
                else:
                    raw_depth = 1.0 / rho
                    if not (float(args.min_depth) <= raw_depth <= float(args.max_depth)):
                        raw_counts["out_of_range"] += 1
                        smoothed = _held_depth(depth_history, depth_last_seen, track_id, pair_idx, args)
                        if smoothed is None:
                            raw_counts["dropped_no_hold"] += 1
                            continue
                        depth = smoothed
                        source = "held_range"
                        raw_counts["held"] += 1
                    else:
                        raw_counts["valid_raw"] += 1
                        depth = _smooth_depth(depth_history, depth_last_seen, track_id, raw_depth, pair_idx, args)
                        source = "smoothed"
                        model_residual = float(np.linalg.norm(flow_derot - rho * a_vec))
                        if model_residual > float(args.max_model_residual):
                            raw_counts["model_residual"] += 1
                            smoothed = _held_depth(depth_history, depth_last_seen, track_id, pair_idx, args)
                            if smoothed is None:
                                raw_counts["dropped_no_hold"] += 1
                                continue
                            depth = smoothed
                            source = "held_residual"
                            raw_counts["held"] += 1
                        else:
                            pair_raw_depths.append(
                                (math.atan2(x0, 1.0), raw_depth, raw_depth / max(0.05, float(v_cam[2])),
                                 x0, y0, float(xy0[0]), float(xy0[1]), track_id, raw_depth, "raw")
                            )
                        if source == "smoothed" and _is_depth_jump(raw_depth, depth, args):
                            source = "smoothed_jump"
                            raw_counts["jump_suppressed"] += 1
            azimuth = math.atan2(x0, 1.0)
            ttc = depth / max(0.05, float(v_cam[2]))
            pair_depths.append((azimuth, depth, ttc, x0, y0, float(xy0[0]), float(xy0[1]), track_id, raw_depth, source))
            tracks_rows.append(
                [
                    pair_idx,
                    f"{cur_t:.9f}",
                    track_id,
                    f"{xy0[0]:.6f}",
                    f"{xy0[1]:.6f}",
                    f"{xy1[0]:.6f}",
                    f"{xy1[1]:.6f}",
                    f"{x0:.9f}",
                    f"{y0:.9f}",
                    f"{flow[0]:.9f}",
                    f"{flow[1]:.9f}",
                    f"{flow_derot[0]:.9f}",
                    f"{flow_derot[1]:.9f}",
                    f"{rho:.9f}",
                    "" if not math.isfinite(raw_depth) else f"{raw_depth:.9f}",
                    f"{depth:.9f}",
                    f"{ttc:.9f}",
                    f"{fb_err[j]:.6f}",
                    f"{np.linalg.norm(a_vec):.9f}",
                    "" if not math.isfinite(model_residual) else f"{model_residual:.9f}",
                    "" if not math.isfinite(flow_cos) else f"{flow_cos:.9f}",
                    "" if not math.isfinite(true_depth) else f"{true_depth:.9f}",
                    "" if not math.isfinite(true_depth) else f"{depth - true_depth:.9f}",
                    source,
                ]
            )

        pair_depths, n_sector_outliers = _filter_sector_depth_outliers(pair_depths, args)
        pair_raw_depths, n_raw_sector_outliers = _filter_sector_depth_outliers(pair_raw_depths, args)
        raw_counts["sector_outlier"] += n_sector_outliers + n_raw_sector_outliers
        _append_sector_rows(sector_rows, pair_idx, cur_t, pair_depths, args.sectors, frame)
        _append_sector_rows(sector_raw_rows, pair_idx, cur_t, pair_raw_depths, args.sectors, frame)
        rejection_rows.append(
            [
                pair_idx,
                f"{cur_t:.9f}",
                len(ok),
                *[raw_counts[key] for key in (
                    "lk_rejected",
                    "low_a",
                    "bad_rho",
                    "out_of_range",
                    "valid_raw",
                    "jump_suppressed",
                    "direction",
                    "model_residual",
                    "sector_outlier",
                    "held",
                    "dropped_no_hold",
                )],
            ]
        )
        if args.visualize:
            _write_track_overlay(
                cv2,
                cur_gray,
                p0,
                p1,
                ok,
                overlay_dir / "all_tracks" / f"tracks_{pair_idx:06d}.png",
            )
            _write_depth_overlay(
                cv2,
                cur_gray,
                p0,
                p1,
                ok,
                pair_depths,
                overlay_dir / "depth_tracks" / f"depth_{pair_idx:06d}.png",
            )

        prev_points = p1[ok].astype(np.float32)
        track_ids = track_ids[ok]
        do_periodic = int(args.redetect_every) > 0 and pair_idx % int(args.redetect_every) == 0
        if do_periodic or len(prev_points) < int(args.min_track_count):
            if int(args.min_track_count) > int(args.max_corners):
                print(
                    f"warning: --min-track-count ({args.min_track_count}) is greater than "
                    f"--max-corners ({args.max_corners}); replenishment will be requested "
                    "every frame but total tracks are capped",
                    flush=True,
                )
                args.min_track_count = int(args.max_corners)
            new_points = _detect_features(cv2, cur_gray, args, existing=prev_points)
            if len(new_points) > 0:
                remaining = max(0, int(args.max_corners) - len(prev_points))
                new_points = new_points[:remaining]
                new_ids = np.arange(next_track_id, next_track_id + len(new_points), dtype=np.int64)
                next_track_id += int(len(new_points))
                prev_points = np.vstack([prev_points, new_points]).astype(np.float32)
                track_ids = np.concatenate([track_ids, new_ids])
        prev_gray, prev_truth_depth, prev_t, prev_tel = cur_gray, _load_truth_depth(dataset, frame), cur_t, cur_tel

    with (out / "tracks.csv").open("w", newline="") as f:
        w = csv.writer(f)
        w.writerow(
            [
                "pair",
                "t",
                "track_id",
                "u0",
                "v0",
                "u1",
                "v1",
                "x",
                "y",
                "flow_x",
                "flow_y",
                "derot_flow_x",
                "derot_flow_y",
                "inv_depth",
                "raw_depth_m",
                "depth_m",
                "ttc_s",
                "fb_err_px",
                "a_norm",
                "model_residual",
                "flow_cos",
                "true_depth_m",
                "depth_error_m",
                "depth_source",
            ]
        )
        w.writerows(tracks_rows)
    with (out / "sectors.csv").open("w", newline="") as f:
        w = csv.writer(f)
        w.writerow(
            [
                "pair",
                "t",
                "sector",
                "azimuth_center_rad",
                "n_features",
                "depth_p20_m",
                "depth_median_m",
                "ttc_median_s",
                "confidence",
                "range_front_truth_m",
            ]
        )
        w.writerows(sector_rows)
    with (out / "sectors_raw.csv").open("w", newline="") as f:
        w = csv.writer(f)
        w.writerow(
            [
                "pair",
                "t",
                "sector",
                "azimuth_center_rad",
                "n_features",
                "depth_p20_m",
                "depth_median_m",
                "ttc_median_s",
                "confidence",
                "range_front_truth_m",
            ]
        )
        w.writerows(sector_raw_rows)
    with (out / "depth_rejections.csv").open("w", newline="") as f:
        w = csv.writer(f)
        w.writerow(
            [
                "pair",
                "t",
                "n_candidate_tracks",
                "lk_rejected",
                "low_a",
                "bad_rho",
                "out_of_range",
                "valid_raw",
                "jump_suppressed",
                "direction",
                "model_residual",
                "sector_outlier",
                "held",
                "dropped_no_hold",
            ]
        )
        w.writerows(rejection_rows)

    print(
        f"wrote {len(tracks_rows)} tracks, {len(sector_rows)} stabilized sector rows, "
        f"and {len(sector_raw_rows)} raw sector rows to {out}"
    )
    return 0


def _load_calibration(dataset: Path) -> dict[str, float | list[float]]:
    path = dataset / "calibration.json"
    if path.exists():
        return json.loads(path.read_text())
    return {"fx": 89.2, "fy": 89.5, "cx": 81.1, "cy": 73.4, "distortion": [0, 0, 0, 0, 0]}


def _read_csv(path: Path) -> list[dict[str, float | str]]:
    with path.open(newline="") as f:
        rows = list(csv.DictReader(f))
    parsed = []
    for row in rows:
        out = {}
        for key, val in row.items():
            if key == "filename":
                out[key] = val
            else:
                try:
                    out[key] = float(val)
                except (TypeError, ValueError):
                    out[key] = val
        parsed.append(out)
    return parsed


def _load_gray(cv2, path: Path, calib: dict[str, float | list[float]]) -> np.ndarray:
    image = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
    if image is None:
        raise FileNotFoundError(path)
    dist = np.asarray(calib.get("distortion", [0, 0, 0, 0, 0]), dtype=np.float64).reshape(-1)
    if dist.size >= 5 and float(np.max(np.abs(dist))) > 0.0:
        k = np.asarray(
            [[calib["fx"], 0.0, calib["cx"]], [0.0, calib["fy"], calib["cy"]], [0.0, 0.0, 1.0]],
            dtype=np.float64,
        )
        image = cv2.undistort(image, k, dist[:5])
    return image


def _load_truth_depth(dataset: Path, frame: dict[str, float | str]) -> np.ndarray | None:
    filename = frame.get("depth_filename")
    if not filename:
        return None
    path = dataset / str(filename)
    if not path.exists():
        return None
    depth = np.load(path)
    if depth.ndim != 2:
        return None
    return np.asarray(depth, dtype=np.float64)


def _sample_depth(depth: np.ndarray | None, xy: np.ndarray) -> float:
    if depth is None:
        return math.nan
    h, w = depth.shape
    x = int(round(float(xy[0])))
    y = int(round(float(xy[1])))
    if x < 0 or y < 0 or x >= w or y >= h:
        return math.nan
    value = float(depth[y, x])
    return value if math.isfinite(value) and value > 0.0 else math.nan


def _detect_features(cv2, gray: np.ndarray, args: argparse.Namespace, existing: np.ndarray | None = None) -> np.ndarray:
    mask = np.full(gray.shape[:2], 255, dtype=np.uint8)
    if existing is not None and len(existing) > 0:
        radius = max(1, int(round(float(args.min_distance))))
        for x, y in np.asarray(existing, dtype=np.float32).reshape(-1, 2):
            cv2.circle(mask, (int(round(float(x))), int(round(float(y)))), radius, 0, -1)
    pts = cv2.goodFeaturesToTrack(
        gray,
        maxCorners=int(args.max_corners),
        qualityLevel=float(args.quality),
        minDistance=float(args.min_distance),
        mask=mask,
        blockSize=5,
    )
    if pts is None:
        return np.zeros((0, 2), dtype=np.float32)
    return pts.reshape(-1, 2).astype(np.float32)


def _interp_telemetry(rows: list[dict[str, float | str]], t: float) -> dict[str, float]:
    if t <= float(rows[0]["t"]):
        return {k: float(v) for k, v in rows[0].items() if k != "filename"}
    for i in range(1, len(rows)):
        if t <= float(rows[i]["t"]):
            a, b = rows[i - 1], rows[i]
            ta, tb = float(a["t"]), float(b["t"])
            alpha = 0.0 if tb <= ta else (t - ta) / (tb - ta)
            out = {}
            for key in b.keys():
                if key == "filename":
                    continue
                out[key] = (1.0 - alpha) * float(a[key]) + alpha * float(b[key])
            q = _slerp_quat(
                np.asarray([a["qx"], a["qy"], a["qz"], a["qw"]], dtype=np.float64),
                np.asarray([b["qx"], b["qy"], b["qz"], b["qw"]], dtype=np.float64),
                alpha,
            )
            out.update({"qx": q[0], "qy": q[1], "qz": q[2], "qw": q[3]})
            return out
    return {k: float(v) for k, v in rows[-1].items() if k != "filename"}


def _pixel_to_norm(xy: np.ndarray, calib: dict[str, float | list[float]]) -> tuple[float, float]:
    return (
        (float(xy[0]) - float(calib["cx"])) / float(calib["fx"]),
        (float(xy[1]) - float(calib["cy"])) / float(calib["fy"]),
    )


def _vec(row: dict[str, float], prefix: str) -> np.ndarray:
    if prefix == "v":
        return np.asarray([row["vx"], row["vy"], row["vz"]], dtype=np.float64)
    return np.asarray([row[prefix + "x"], row[prefix + "y"], row[prefix + "z"]], dtype=np.float64)


def _quat_xyzw_to_rot(q: np.ndarray) -> np.ndarray:
    q = np.asarray(q, dtype=np.float64).reshape(4)
    q = q / max(1e-12, float(np.linalg.norm(q)))
    x, y, z, w = q
    return np.asarray(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def _slerp_quat(q0: np.ndarray, q1: np.ndarray, alpha: float) -> np.ndarray:
    q0 = q0 / max(1e-12, float(np.linalg.norm(q0)))
    q1 = q1 / max(1e-12, float(np.linalg.norm(q1)))
    dot = float(np.dot(q0, q1))
    if dot < 0.0:
        q1 = -q1
        dot = -dot
    if dot > 0.9995:
        out = q0 + float(alpha) * (q1 - q0)
        return out / max(1e-12, float(np.linalg.norm(out)))
    theta0 = math.acos(max(-1.0, min(1.0, dot)))
    theta = theta0 * float(alpha)
    sin_theta = math.sin(theta)
    sin_theta0 = math.sin(theta0)
    s0 = math.cos(theta) - dot * sin_theta / sin_theta0
    s1 = sin_theta / sin_theta0
    return s0 * q0 + s1 * q1


def _smooth_depth(
    history: dict[int, list[float]],
    last_seen: dict[int, int],
    track_id: int,
    raw_depth: float,
    pair_idx: int,
    args: argparse.Namespace,
) -> float:
    vals = history.get(track_id, [])
    if vals:
        median = float(np.median(np.asarray(vals, dtype=np.float64)))
        if _is_depth_jump(raw_depth, median, args):
            last_seen[track_id] = pair_idx
            return median
    window = _depth_smooth_window(float(raw_depth), args)
    vals = (vals + [float(raw_depth)])[-max(1, int(window)):]
    history[track_id] = vals
    last_seen[track_id] = pair_idx
    return float(np.median(np.asarray(vals, dtype=np.float64)))


def _held_depth(
    history: dict[int, list[float]],
    last_seen: dict[int, int],
    track_id: int,
    pair_idx: int,
    args: argparse.Namespace,
) -> float | None:
    vals = history.get(track_id)
    if not vals:
        return None
    age = pair_idx - int(last_seen.get(track_id, -10**9))
    median = float(np.median(np.asarray(vals, dtype=np.float64)))
    hold_frames = _depth_hold_frames(median, args)
    if age > int(hold_frames):
        return None
    return median


def _depth_smooth_window(depth: float, args: argparse.Namespace) -> int:
    if float(args.near_depth_thresh) > 0.0 and depth <= float(args.near_depth_thresh):
        if args.near_depth_smooth_window is not None:
            return int(args.near_depth_smooth_window)
    return int(args.depth_smooth_window)


def _depth_hold_frames(depth: float, args: argparse.Namespace) -> int:
    if float(args.near_depth_thresh) > 0.0 and depth <= float(args.near_depth_thresh):
        if args.near_depth_hold_frames is not None:
            return int(args.near_depth_hold_frames)
    return int(args.depth_hold_frames)


def _is_depth_jump(raw_depth: float, reference_depth: float, args: argparse.Namespace) -> bool:
    if not (math.isfinite(raw_depth) and math.isfinite(reference_depth)):
        return False
    raw = float(raw_depth)
    ref = max(1e-6, float(reference_depth))
    if raw >= ref:
        ratio = raw / ref
        threshold = args.depth_jump_up_ratio
    else:
        ratio = ref / max(1e-6, raw)
        threshold = args.depth_jump_down_ratio
    if threshold is None:
        threshold = args.depth_jump_ratio
    return ratio > float(threshold)


def _filter_sector_depth_outliers(
    depths: list[tuple[float, float, float, float, float, float, float, int, float, str]],
    args: argparse.Namespace,
) -> tuple[list[tuple[float, float, float, float, float, float, float, int, float, str]], int]:
    if float(args.sector_mad_thresh) <= 0.0 or not depths:
        return depths, 0
    edges = np.linspace(-0.5 * math.pi, 0.5 * math.pi, int(args.sectors) + 1)
    kept: list[tuple[float, float, float, float, float, float, float, int, float, str]] = []
    rejected = 0
    for sector in range(int(args.sectors)):
        lo, hi = float(edges[sector]), float(edges[sector + 1])
        vals = [d for d in depths if lo <= d[0] < hi]
        if len(vals) < int(args.sector_min_inliers):
            kept.extend(vals)
            continue
        z = np.asarray([d[1] for d in vals], dtype=np.float64)
        median = float(np.median(z))
        mad = float(np.median(np.abs(z - median)))
        scale = max(1e-6, 1.4826 * mad)
        for item in vals:
            if abs(float(item[1]) - median) <= float(args.sector_mad_thresh) * scale:
                kept.append(item)
            else:
                rejected += 1
    return kept, rejected


def _append_sector_rows(
    rows: list[list[object]],
    pair_idx: int,
    t: float,
    depths: list[tuple[float, float, float, float, float, float, float, int, float, str]],
    n_sectors: int,
    frame: dict[str, float | str],
) -> None:
    edges = np.linspace(-0.5 * math.pi, 0.5 * math.pi, int(n_sectors) + 1)
    range_truth = frame.get("range_front_truth_m", math.nan)
    for sector in range(int(n_sectors)):
        lo, hi = float(edges[sector]), float(edges[sector + 1])
        vals = [d for d in depths if lo <= d[0] < hi]
        center = 0.5 * (lo + hi)
        if vals:
            z = np.asarray([d[1] for d in vals], dtype=np.float64)
            ttc = np.asarray([d[2] for d in vals], dtype=np.float64)
            confidence = min(1.0, len(vals) / 8.0) * max(0.0, 1.0 - min(1.0, float(np.std(z)) / 2.0))
            rows.append(
                [
                    pair_idx,
                    f"{t:.9f}",
                    sector,
                    f"{center:.9f}",
                    len(vals),
                    f"{float(np.percentile(z, 20)):.9f}",
                    f"{float(np.median(z)):.9f}",
                    f"{float(np.median(ttc)):.9f}",
                    f"{confidence:.6f}",
                    f"{float(range_truth):.9f}",
                ]
            )
        else:
            rows.append([pair_idx, f"{t:.9f}", sector, f"{center:.9f}", 0, "", "", "", "0.000000", range_truth])


def _write_track_overlay(
    cv2,
    gray: np.ndarray,
    p0: np.ndarray,
    p1: np.ndarray,
    ok: np.ndarray,
    path: Path,
) -> None:
    vis = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
    for xy0, xy1, good in zip(p0, p1, ok):
        if not good:
            continue
        a = (int(round(float(xy0[0]))), int(round(float(xy0[1]))))
        b = (int(round(float(xy1[0]))), int(round(float(xy1[1]))))
        cv2.line(vis, a, b, (180, 180, 180), 1)
        cv2.circle(vis, b, 2, (0, 220, 255), -1)
    cv2.imwrite(str(path), vis)


def _write_depth_overlay(
    cv2,
    gray: np.ndarray,
    p0: np.ndarray,
    p1: np.ndarray,
    ok: np.ndarray,
    depths: list[tuple[float, float, float, float, float, float, float, int, float, str]],
    path: Path,
) -> None:
    vis = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
    for xy0, xy1, good in zip(p0, p1, ok):
        if not good:
            continue
        a = (int(round(float(xy0[0]))), int(round(float(xy0[1]))))
        b = (int(round(float(xy1[0]))), int(round(float(xy1[1]))))
        cv2.line(vis, a, b, (90, 90, 90), 1)
    for _, depth, _, _, _, u, v, _, _, source in depths:
        if source.startswith("held"):
            color = (0, 165, 255)
        else:
            color = (0, int(max(0, min(255, 255 - 30 * depth))), int(max(0, min(255, 30 * depth))))
        cv2.circle(vis, (int(round(u)), int(round(v))), 2, color, -1)
    cv2.imwrite(str(path), vis)


if __name__ == "__main__":
    raise SystemExit(main())
