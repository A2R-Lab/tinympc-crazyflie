#!/usr/bin/env python3
"""Generate TinyMPC-style obstacle half-spaces from simulated flow sectors.

This is a sim-side shadow path: it consumes the sector depth estimates produced by
offline_flow_harness.py and writes the per-horizon a_hs/b_hs/en_hs values that the
firmware controller would eventually populate.
"""

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
R_BC = R_CB.T


def parse_args() -> argparse.Namespace:
    ap = argparse.ArgumentParser(
        description=(
            "Convert flow-sector depth estimates into per-horizon position "
            "half-space constraints a^T p <= b."
        )
    )
    ap.add_argument(
        "path",
        type=Path,
        nargs="?",
        default=Path("flow_sim_dataset"),
        help="dataset dir or harness output dir; default: flow_sim_dataset",
    )
    ap.add_argument("--harness", default="flow_harness",
                    help="harness subdir when PATH is a dataset dir")
    ap.add_argument("--sectors-file", default="sectors.csv",
                    help="sector CSV inside the harness directory")
    ap.add_argument("--out", type=Path, default=None,
                    help="output directory; default: HARNESS/constraints")
    ap.add_argument("--depth-field", choices=["p20", "median"], default="p20",
                    help="sector depth statistic to turn into an obstacle plane")
    ap.add_argument("--horizon", type=int, default=25)
    ap.add_argument("--start-k", type=int, default=3,
                    help="first horizon knot where the half-space is enabled")
    ap.add_argument("--end-k", type=int, default=None,
                    help="last enabled horizon knot, inclusive; default horizon-1")
    ap.add_argument("--min-confidence", type=float, default=0.20)
    ap.add_argument("--min-features", type=int, default=4)
    ap.add_argument("--min-depth", type=float, default=0.20)
    ap.add_argument("--max-depth", type=float, default=15.0)
    ap.add_argument("--margin-min", type=float, default=0.35,
                    help="minimum standoff margin before the obstacle plane [m]")
    ap.add_argument("--margin-slack", type=float, default=0.35,
                    help="extra margin at zero confidence [m]")
    ap.add_argument("--max-margin-frac", type=float, default=0.80,
                    help="cap margin to this fraction of selected depth")
    ap.add_argument("--prefer-center", type=float, default=0.15,
                    help="score penalty per radian away from image center")
    ap.add_argument("--max-abs-azimuth", type=float, default=math.pi / 2.0,
                    help="ignore sectors farther than this absolute azimuth [rad]")
    ap.add_argument("--plot", action="store_true",
                    help="write a top-down PNG if matplotlib is available")
    return ap.parse_args()


def main() -> int:
    args = parse_args()
    dataset, harness = _resolve_paths(args.path, args.harness)
    sectors_path = harness / args.sectors_file
    if not sectors_path.exists():
        raise SystemExit(f"missing sector file: {sectors_path}")

    out = args.out or (harness / "constraints")
    out.mkdir(parents=True, exist_ok=True)

    telemetry = _read_csv(dataset / "telemetry.csv")
    sectors = _read_csv(sectors_path)
    grouped = _group_by_pair(sectors)

    horizon = int(args.horizon)
    end_k = horizon - 1 if args.end_k is None else int(args.end_k)
    if horizon <= 0:
        raise SystemExit("--horizon must be positive")
    if int(args.start_k) < 0 or end_k >= horizon or int(args.start_k) > end_k:
        raise SystemExit("--start-k/--end-k must define a valid inclusive range inside the horizon")

    summary_rows: list[list[object]] = []
    constraint_rows: list[list[object]] = []
    active_pairs = 0
    disabled_reasons: dict[str, int] = {}

    for pair in sorted(grouped):
        rows = grouped[pair]
        t = float(rows[0]["t"])
        tel = _interp_telemetry(telemetry, t)
        state_pos = np.asarray([tel["x"], tel["y"], tel["z"]], dtype=np.float64)
        rot_wb = _quat_xyzw_to_rot(
            np.asarray([tel["qx"], tel["qy"], tel["qz"], tel["qw"]], dtype=np.float64)
        )
        candidate, reason = _select_sector(rows, args)
        if candidate is None:
            disabled_reasons[reason] = disabled_reasons.get(reason, 0) + 1
            _append_disabled(pair, t, horizon, constraint_rows, reason)
            summary_rows.append(_summary_disabled(pair, t, reason, tel))
            continue

        az = float(candidate["azimuth_center_rad"])
        depth = _sector_depth(candidate, args.depth_field)
        confidence = float(candidate["confidence"])
        margin = _constraint_margin(depth, confidence, args)
        d_cam = _ray_from_azimuth(az)
        d_world = _normalized(rot_wb @ (R_BC @ d_cam))
        p_obst = state_pos + depth * d_world

        # TinyMPC projects against a^T p <= b. For a forward obstacle this uses
        # a=d_world, so positions before the obstacle remain feasible.
        a = d_world
        b = float(np.dot(a, p_obst) - margin)
        current_slack = float(b - np.dot(a, state_pos))
        active = current_slack > 0.02 and math.isfinite(current_slack)
        reason = "active" if active else "violates_current"
        if active:
            active_pairs += 1
        else:
            disabled_reasons[reason] = disabled_reasons.get(reason, 0) + 1

        summary_rows.append(
            [
                pair,
                f"{t:.9f}",
                int(candidate["sector"]),
                f"{az:.9f}",
                int(candidate["n_features"]),
                f"{depth:.9f}",
                f"{confidence:.6f}",
                f"{margin:.9f}",
                f"{a[0]:.9f}",
                f"{a[1]:.9f}",
                f"{a[2]:.9f}",
                f"{b:.9f}",
                f"{state_pos[0]:.9f}",
                f"{state_pos[1]:.9f}",
                f"{state_pos[2]:.9f}",
                f"{p_obst[0]:.9f}",
                f"{p_obst[1]:.9f}",
                f"{p_obst[2]:.9f}",
                f"{current_slack:.9f}",
                candidate.get("range_front_truth_m", ""),
                reason,
            ]
        )
        for k in range(horizon):
            enabled = active and int(args.start_k) <= k <= end_k
            constraint_rows.append(
                [
                    pair,
                    f"{t:.9f}",
                    k,
                    int(enabled),
                    f"{a[0]:.9f}" if enabled else "",
                    f"{a[1]:.9f}" if enabled else "",
                    f"{a[2]:.9f}" if enabled else "",
                    f"{b:.9f}" if enabled else "",
                    f"{depth:.9f}" if enabled else "",
                    f"{margin:.9f}" if enabled else "",
                    int(candidate["sector"]) if enabled else "",
                    f"{az:.9f}" if enabled else "",
                    f"{confidence:.6f}" if enabled else "",
                    reason,
                ]
            )

    _write_summary(out / "constraint_summary.csv", summary_rows)
    _write_constraints(out / "constraints.csv", constraint_rows)
    _write_metadata(out / "metadata.json", args, dataset, harness, sectors_path, active_pairs, disabled_reasons)
    if args.plot:
        _write_plot(out / "constraints_topdown.png", summary_rows)

    total = len(grouped)
    print(f"wrote {len(summary_rows)} summaries and {len(constraint_rows)} horizon rows to {out}")
    print(f"active pairs: {active_pairs}/{total} ({(active_pairs / total if total else 0.0):.1%})")
    _print_active_stats(summary_rows)
    if disabled_reasons:
        reasons = ", ".join(f"{k}={v}" for k, v in sorted(disabled_reasons.items()))
        print(f"disabled/rejected: {reasons}")
    return 0


def _resolve_paths(path: Path, harness_name: str) -> tuple[Path, Path]:
    path = path.resolve()
    if (path / "telemetry.csv").exists():
        dataset = path
        harness = dataset / harness_name
    else:
        harness = path
        dataset = harness.parent
    if not (dataset / "telemetry.csv").exists():
        raise SystemExit(f"could not find telemetry.csv for {path}")
    if not harness.exists():
        raise SystemExit(f"could not find harness directory: {harness}")
    return dataset, harness


def _read_csv(path: Path) -> list[dict[str, float | str]]:
    with path.open(newline="") as f:
        rows = list(csv.DictReader(f))
    parsed = []
    for row in rows:
        out = {}
        for key, val in row.items():
            try:
                out[key] = float(val)
            except (TypeError, ValueError):
                out[key] = val
        parsed.append(out)
    return parsed


def _group_by_pair(rows: list[dict[str, float | str]]) -> dict[int, list[dict[str, float | str]]]:
    grouped: dict[int, list[dict[str, float | str]]] = {}
    for row in rows:
        grouped.setdefault(int(row["pair"]), []).append(row)
    return grouped


def _select_sector(
    rows: list[dict[str, float | str]],
    args: argparse.Namespace,
) -> tuple[dict[str, float | str] | None, str]:
    candidates = []
    for row in rows:
        n_features = int(row.get("n_features", 0) or 0)
        confidence = float(row.get("confidence", 0.0) or 0.0)
        depth = _sector_depth(row, args.depth_field)
        if n_features < int(args.min_features):
            continue
        if confidence < float(args.min_confidence):
            continue
        if not (float(args.min_depth) <= depth <= float(args.max_depth)):
            continue
        az = abs(float(row["azimuth_center_rad"]))
        if az > float(args.max_abs_azimuth):
            continue
        score = depth * (1.0 + float(args.prefer_center) * az) / max(0.05, confidence)
        candidates.append((score, row))
    if candidates:
        candidates.sort(key=lambda item: item[0])
        return candidates[0][1], "active"

    any_depth = any(math.isfinite(_sector_depth(row, args.depth_field)) for row in rows)
    if not any_depth:
        return None, "no_depth"
    any_features = any(int(row.get("n_features", 0) or 0) >= int(args.min_features) for row in rows)
    if not any_features:
        return None, "too_few_features"
    any_conf = any(float(row.get("confidence", 0.0) or 0.0) >= float(args.min_confidence) for row in rows)
    if not any_conf:
        return None, "low_confidence"
    return None, "depth_out_of_range"


def _sector_depth(row: dict[str, float | str], depth_field: str) -> float:
    key = "depth_p20_m" if depth_field == "p20" else "depth_median_m"
    try:
        return float(row[key])
    except (KeyError, TypeError, ValueError):
        return math.nan


def _constraint_margin(depth: float, confidence: float, args: argparse.Namespace) -> float:
    confidence = max(0.0, min(1.0, float(confidence)))
    margin = float(args.margin_min) + (1.0 - confidence) * float(args.margin_slack)
    if float(args.max_margin_frac) > 0.0:
        margin = min(margin, float(args.max_margin_frac) * float(depth))
    return max(0.0, margin)


def _append_disabled(
    pair: int,
    t: float,
    horizon: int,
    rows: list[list[object]],
    reason: str,
) -> None:
    for k in range(horizon):
        rows.append([pair, f"{t:.9f}", k, 0, "", "", "", "", "", "", "", "", "", reason])


def _summary_disabled(
    pair: int,
    t: float,
    reason: str,
    tel: dict[str, float],
) -> list[object]:
    return [
        pair,
        f"{t:.9f}",
        "",
        "",
        "",
        "",
        "",
        "",
        "",
        "",
        "",
        "",
        f"{tel['x']:.9f}",
        f"{tel['y']:.9f}",
        f"{tel['z']:.9f}",
        "",
        "",
        "",
        "",
        "",
        reason,
    ]


def _write_summary(path: Path, rows: list[list[object]]) -> None:
    with path.open("w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(
            [
                "pair",
                "t",
                "sector",
                "azimuth_center_rad",
                "n_features",
                "depth_m",
                "confidence",
                "margin_m",
                "a_x",
                "a_y",
                "a_z",
                "b",
                "state_x",
                "state_y",
                "state_z",
                "obstacle_x",
                "obstacle_y",
                "obstacle_z",
                "current_slack_m",
                "range_front_truth_m",
                "status",
            ]
        )
        writer.writerows(rows)


def _write_constraints(path: Path, rows: list[list[object]]) -> None:
    with path.open("w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(
            [
                "pair",
                "t",
                "k",
                "en_hs",
                "a_x",
                "a_y",
                "a_z",
                "b",
                "depth_m",
                "margin_m",
                "sector",
                "azimuth_center_rad",
                "confidence",
                "status",
            ]
        )
        writer.writerows(rows)


def _write_metadata(
    path: Path,
    args: argparse.Namespace,
    dataset: Path,
    harness: Path,
    sectors_path: Path,
    active_pairs: int,
    disabled_reasons: dict[str, int],
) -> None:
    payload = {
        "dataset": str(dataset),
        "harness": str(harness),
        "sectors_file": str(sectors_path),
        "depth_field": args.depth_field,
        "horizon": int(args.horizon),
        "start_k": int(args.start_k),
        "end_k": int(args.horizon - 1 if args.end_k is None else args.end_k),
        "min_confidence": float(args.min_confidence),
        "min_features": int(args.min_features),
        "min_depth": float(args.min_depth),
        "max_depth": float(args.max_depth),
        "margin_min": float(args.margin_min),
        "margin_slack": float(args.margin_slack),
        "max_margin_frac": float(args.max_margin_frac),
        "prefer_center": float(args.prefer_center),
        "max_abs_azimuth": float(args.max_abs_azimuth),
        "active_pairs": int(active_pairs),
        "disabled_reasons": disabled_reasons,
        "constraint_sign": "a=d_world, b=a dot p_obst - margin, feasible side before obstacle",
    }
    path.write_text(json.dumps(payload, indent=2) + "\n")


def _print_active_stats(rows: list[list[object]]) -> None:
    active = [r for r in rows if r[-1] == "active"]
    if not active:
        return
    depths = [float(r[5]) for r in active]
    margins = [float(r[7]) for r in active]
    slacks = [float(r[18]) for r in active]
    truth_errors = []
    for row in active:
        try:
            truth = float(row[19])
        except (TypeError, ValueError):
            continue
        if math.isfinite(truth):
            truth_errors.append(float(row[5]) - truth)
    print(f"selected depth median/p10/p90: {_percentile(depths, 0.50):.3f} / {_percentile(depths, 0.10):.3f} / {_percentile(depths, 0.90):.3f} m")
    print(f"margin median: {_percentile(margins, 0.50):.3f} m")
    print(f"current slack median/min: {_percentile(slacks, 0.50):.3f} / {min(slacks):.3f} m")
    if truth_errors:
        abs_errors = [abs(e) for e in truth_errors]
        print(
            "selected depth vs front truth median error/median abs: "
            f"{_percentile(truth_errors, 0.50):+.3f} / {_percentile(abs_errors, 0.50):.3f} m"
        )


def _percentile(values: list[float], q: float) -> float:
    clean = sorted(float(v) for v in values if math.isfinite(float(v)))
    if not clean:
        return math.nan
    idx = min(len(clean) - 1, max(0, int(round(float(q) * (len(clean) - 1)))))
    return clean[idx]


def _write_plot(path: Path, rows: list[list[object]]) -> None:
    try:
        import matplotlib.pyplot as plt
    except Exception as exc:
        print(f"could not import matplotlib; skipping --plot output ({exc})")
        return
    active = [r for r in rows if r[-1] == "active"]
    if not active:
        return
    xs = [float(r[12]) for r in active]
    ys = [float(r[13]) for r in active]
    ox = [float(r[15]) for r in active]
    oy = [float(r[16]) for r in active]
    axv = [float(r[8]) for r in active]
    ayv = [float(r[9]) for r in active]
    try:
        fig, ax = plt.subplots(figsize=(8, 4.5))
        ax.plot(xs, ys, color="black", linewidth=1.5, label="drone")
        ax.scatter(ox, oy, s=12, color="tab:red", label="selected obstacle points")
        ax.quiver(ox, oy, axv, ayv, angles="xy", scale_units="xy", scale=5.0, color="tab:blue", width=0.003)
        ax.set_aspect("equal", adjustable="box")
        ax.set_xlabel("world x [m]")
        ax.set_ylabel("world y [m]")
        ax.legend(loc="best")
        fig.tight_layout()
        fig.savefig(path, dpi=160)
        plt.close(fig)
    except Exception as exc:
        print(f"could not write plot {path}; skipping ({exc})")


def _ray_from_azimuth(azimuth_rad: float) -> np.ndarray:
    # Camera frame: +z forward, +x right, +y down. Keep this first pass
    # horizontal-only because the current sector aggregator bins azimuth only.
    return _normalized(np.asarray([math.tan(float(azimuth_rad)), 0.0, 1.0], dtype=np.float64))


def _interp_telemetry(rows: list[dict[str, float | str]], t: float) -> dict[str, float]:
    if t <= float(rows[0]["t"]):
        return {k: float(v) for k, v in rows[0].items()}
    for i in range(1, len(rows)):
        if t <= float(rows[i]["t"]):
            a, b = rows[i - 1], rows[i]
            ta, tb = float(a["t"]), float(b["t"])
            alpha = 0.0 if tb <= ta else (t - ta) / (tb - ta)
            out = {}
            for key in b.keys():
                out[key] = (1.0 - alpha) * float(a[key]) + alpha * float(b[key])
            q = _slerp_quat(
                np.asarray([a["qx"], a["qy"], a["qz"], a["qw"]], dtype=np.float64),
                np.asarray([b["qx"], b["qy"], b["qz"], b["qw"]], dtype=np.float64),
                alpha,
            )
            out.update({"qx": q[0], "qy": q[1], "qz": q[2], "qw": q[3]})
            return out
    return {k: float(v) for k, v in rows[-1].items()}


def _quat_xyzw_to_rot(q: np.ndarray) -> np.ndarray:
    q = _normalized(np.asarray(q, dtype=np.float64).reshape(4))
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
    q0 = _normalized(q0)
    q1 = _normalized(q1)
    dot = float(np.dot(q0, q1))
    if dot < 0.0:
        q1 = -q1
        dot = -dot
    if dot > 0.9995:
        return _normalized(q0 + float(alpha) * (q1 - q0))
    theta0 = math.acos(max(-1.0, min(1.0, dot)))
    theta = theta0 * float(alpha)
    sin_theta = math.sin(theta)
    sin_theta0 = math.sin(theta0)
    s0 = math.cos(theta) - dot * sin_theta / sin_theta0
    s1 = sin_theta / sin_theta0
    return s0 * q0 + s1 * q1


def _normalized(v: np.ndarray) -> np.ndarray:
    norm = float(np.linalg.norm(v))
    if norm <= 1e-12:
        return np.zeros_like(v, dtype=np.float64)
    return np.asarray(v, dtype=np.float64) / norm


if __name__ == "__main__":
    raise SystemExit(main())
