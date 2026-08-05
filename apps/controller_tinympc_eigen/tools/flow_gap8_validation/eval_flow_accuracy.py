#!/usr/bin/env python3
"""Evaluate offline flow-sector depth accuracy against front-range truth."""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path


def parse_args() -> argparse.Namespace:
    ap = argparse.ArgumentParser()
    ap.add_argument(
        "path",
        type=Path,
        nargs="?",
        default=Path("flow_sim_dataset"),
        help="dataset dir or harness output dir; default: flow_sim_dataset",
    )
    ap.add_argument("--harness", default="flow_harness",
                    help="harness subdir when PATH is a dataset dir")
    ap.add_argument("--truth-min", type=float, default=0.75,
                    help="only evaluate rows with range_front_truth_m above this")
    ap.add_argument("--center-sector", type=int, default=3)
    ap.add_argument("--files", nargs="+", default=["sectors_raw.csv", "sectors.csv"])
    ap.add_argument("--buckets", default="1.5:9.0,1.0:1.5,0.75:1.0,0.5:0.75,0.0:0.5",
                    help="comma-separated range buckets as lo:hi")
    return ap.parse_args()


def main() -> int:
    args = parse_args()
    harness = _resolve_harness(args.path, args.harness)
    print(f"harness: {harness}")
    print(f"truth_min: {args.truth_min:.3f} m")

    for filename in args.files:
        path = harness / filename
        if not path.exists():
            print(f"\n{filename}: missing")
            continue
        rows = _read_rows(path)
        print(f"\n{filename}")
        _print_scope("all active sectors", rows, truth_min=args.truth_min)
        _print_scope(
            f"center sector {args.center_sector}",
            [r for r in rows if int(r["sector"]) == int(args.center_sector)],
            truth_min=args.truth_min,
        )
        _print_buckets(rows, int(args.center_sector), _parse_buckets(args.buckets))

    rejections = harness / "depth_rejections.csv"
    if rejections.exists():
        _print_rejections(rejections)
    tracks = harness / "tracks.csv"
    if tracks.exists():
        _print_feature_truth(tracks)
    return 0


def _resolve_harness(path: Path, harness_name: str) -> Path:
    if (path / "sectors.csv").exists():
        return path
    harness = path / harness_name
    if (harness / "sectors.csv").exists():
        return harness
    raise SystemExit(f"Could not find sectors.csv in {path} or {harness}")


def _read_rows(path: Path) -> list[dict[str, str]]:
    with path.open(newline="") as f:
        return list(csv.DictReader(f))


def _valid_rows(rows: list[dict[str, str]], truth_min: float) -> list[dict[str, str]]:
    out = []
    for row in rows:
        if int(row["n_features"] or 0) <= 0:
            continue
        truth = float(row["range_front_truth_m"])
        if truth <= truth_min:
            continue
        depth = _depth(row)
        if not math.isfinite(depth):
            continue
        out.append(row)
    return out


def _depth(row: dict[str, str]) -> float:
    if row.get("depth_median_m"):
        return float(row["depth_median_m"])
    if row.get("depth_p20_m"):
        return float(row["depth_p20_m"])
    return math.nan


def _print_scope(name: str, rows: list[dict[str, str]], truth_min: float) -> None:
    valid = _valid_rows(rows, truth_min)
    if not valid:
        print(f"  {name}: no valid rows")
        return
    errors = [_depth(r) - float(r["range_front_truth_m"]) for r in valid]
    counts = [int(r["n_features"]) for r in valid]
    print(f"  {name}:")
    print(f"    rows: {len(valid)}")
    print(f"    median n_features: {_median(counts):.0f}")
    print(f"    median error: {_median(errors):+.3f} m")
    print(f"    mean error: {sum(errors) / len(errors):+.3f} m")
    print(f"    median abs error: {_percentile([abs(e) for e in errors], 0.50):.3f} m")
    print(f"    p90 abs error: {_percentile([abs(e) for e in errors], 0.90):.3f} m")


def _print_buckets(
    rows: list[dict[str, str]],
    center_sector: int,
    buckets: list[tuple[float, float]],
) -> None:
    center = [r for r in rows if int(r["sector"]) == center_sector and int(r["n_features"] or 0) > 0]
    if not center:
        return
    print(f"  center sector {center_sector} by truth range:")
    for lo, hi in buckets:
        bucket = []
        for row in center:
            truth = float(row["range_front_truth_m"])
            if lo < truth <= hi:
                bucket.append(row)
        if not bucket:
            continue
        errors = [_depth(r) - float(r["range_front_truth_m"]) for r in bucket]
        counts = [int(r["n_features"]) for r in bucket]
        print(f"    {lo:.2f}-{hi:.2f} m:")
        print(f"      rows: {len(bucket)}")
        print(f"      median n_features: {_median(counts):.0f}")
        print(f"      median error: {_median(errors):+.3f} m")
        print(f"      median abs error: {_percentile([abs(e) for e in errors], 0.50):.3f} m")
        print(f"      p90 abs error: {_percentile([abs(e) for e in errors], 0.90):.3f} m")


def _print_rejections(path: Path) -> None:
    rows = _read_rows(path)
    if not rows:
        return
    keys = [
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
    total = sum(int(float(r["n_candidate_tracks"])) for r in rows)
    if total <= 0:
        return
    print("\ndepth_rejections.csv:")
    print(f"  candidate tracks: {total}")
    for key in keys:
        count = sum(int(float(r.get(key, 0) or 0)) for r in rows)
        print(f"  {key}: {count} ({count / total:.1%})")


def _print_feature_truth(path: Path) -> None:
    rows = _read_rows(path)
    if not rows or "true_depth_m" not in rows[0]:
        return
    valid = []
    for row in rows:
        if not row.get("true_depth_m") or not row.get("depth_m"):
            continue
        truth = float(row["true_depth_m"])
        estimate = float(row["depth_m"])
        if math.isfinite(truth) and math.isfinite(estimate) and truth > 0.0:
            valid.append((estimate - truth, row.get("depth_source", "")))
    if not valid:
        return
    print("\nfeature truth from tracks.csv:")
    _print_error_stats("  all tracked depths", [err for err, _ in valid])
    fresh = [err for err, source in valid if source in {"smoothed", "smoothed_jump"}]
    if fresh:
        _print_error_stats("  fresh/smoothed only", fresh)
    held = [err for err, source in valid if source.startswith("held")]
    if held:
        _print_error_stats("  held only", held)


def _print_error_stats(label: str, errors: list[float]) -> None:
    print(f"{label}:")
    print(f"    rows: {len(errors)}")
    print(f"    median error: {_median(errors):+.3f} m")
    print(f"    mean error: {sum(errors) / len(errors):+.3f} m")
    print(f"    median abs error: {_percentile([abs(e) for e in errors], 0.50):.3f} m")
    print(f"    p90 abs error: {_percentile([abs(e) for e in errors], 0.90):.3f} m")


def _parse_buckets(raw: str) -> list[tuple[float, float]]:
    buckets = []
    for item in raw.split(","):
        if not item:
            continue
        lo, hi = item.split(":", 1)
        buckets.append((float(lo), float(hi)))
    return buckets


def _median(values: list[float | int]) -> float:
    return _percentile([float(v) for v in values], 0.50)


def _percentile(values: list[float], q: float) -> float:
    clean = sorted(float(v) for v in values if math.isfinite(float(v)))
    if not clean:
        return math.nan
    idx = min(len(clean) - 1, max(0, int(round(q * (len(clean) - 1)))))
    return clean[idx]


if __name__ == "__main__":
    raise SystemExit(main())
