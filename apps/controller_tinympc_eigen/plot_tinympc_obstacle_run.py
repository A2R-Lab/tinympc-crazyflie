#!/usr/bin/env python3
"""Plot trajectory, modeled cylinder, and logged TinyMPC obstacle half-spaces."""

import argparse
import csv
import math
import os
from pathlib import Path

os.environ.setdefault(
    "MPLCONFIGDIR",
    str(Path(os.environ.get("TMPDIR", "/tmp")) / "tinympc_mpl_cache"),
)

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt


def parse_args():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("csv", nargs="?", default="obstacle_run.csv")
    p.add_argument("--out", default=None, help="Output image path. Defaults to <csv>.png")
    p.add_argument("--obs-x", type=float, default=1.0)
    p.add_argument("--obs-y", type=float, default=0.0)
    p.add_argument("--radius", type=float, default=None, help="Cylinder radius to draw. Defaults to logged obs_margin.")
    p.add_argument("--plane-stride", type=int, default=15, help="Plot every Nth logged active plane.")
    p.add_argument("--plane-alpha", type=float, default=0.22)
    p.add_argument("--xlim", nargs=2, type=float, default=None)
    p.add_argument("--ylim", nargs=2, type=float, default=None)
    return p.parse_args()


def f(row, key):
    val = row.get(key, "")
    if val is None or val == "":
        return None
    try:
        return float(val)
    except ValueError:
        return None


def load_rows(path):
    with open(path, newline="") as fp:
        return list(csv.DictReader(fp))


def finite_points(rows):
    pts = []
    for row in rows:
        x = f(row, "x")
        y = f(row, "y")
        if x is not None and y is not None and math.isfinite(x) and math.isfinite(y):
            pts.append((x, y))
    return pts


def halfspace_rows(rows):
    hs = []
    for row in rows:
        active = f(row, "obs_apply")
        if active is None:
            active = f(row, "obs_active")
        a0 = f(row, "obs_a0")
        a1 = f(row, "obs_a1")
        b = f(row, "obs_b")
        if active and a0 is not None and a1 is not None and b is not None:
            if math.hypot(a0, a1) > 1e-6:
                hs.append((a0, a1, b))
    return hs


def draw_halfspace(ax, a0, a1, b, xmin, xmax, ymin, ymax, **kwargs):
    if abs(a1) > 1e-6:
        xs = [xmin, xmax]
        ys = [(b - a0 * x) / a1 for x in xs]
        ax.plot(xs, ys, **kwargs)
    elif abs(a0) > 1e-6:
        x = b / a0
        ax.plot([x, x], [ymin, ymax], **kwargs)


def main():
    args = parse_args()
    csv_path = Path(args.csv)
    rows = load_rows(csv_path)
    pts = finite_points(rows)
    if not pts:
        raise SystemExit(f"no x/y trajectory samples found in {csv_path}")

    xs = [p[0] for p in pts]
    ys = [p[1] for p in pts]
    margins = [f(r, "obs_margin") for r in rows]
    margins = [m for m in margins if m is not None and math.isfinite(m) and m > 0.0]
    radius = args.radius if args.radius is not None else (max(margins) if margins else 0.35)

    xmin = min(xs + [args.obs_x - radius])
    xmax = max(xs + [args.obs_x + radius])
    ymin = min(ys + [args.obs_y - radius])
    ymax = max(ys + [args.obs_y + radius])
    pad = max(0.25, 0.15 * max(xmax - xmin, ymax - ymin, 1.0))
    xmin -= pad
    xmax += pad
    ymin -= pad
    ymax += pad
    if args.xlim:
        xmin, xmax = args.xlim
    if args.ylim:
        ymin, ymax = args.ylim

    fig, ax = plt.subplots(figsize=(9, 5.5))
    ax.plot(xs, ys, color="black", linewidth=2.0, label="trajectory")
    ax.scatter(xs[0], ys[0], s=80, color="forestgreen", zorder=4, label="start")
    ax.scatter(xs[-1], ys[-1], s=80, color="tab:red", zorder=4, label="end")

    circle = plt.Circle(
        (args.obs_x, args.obs_y),
        radius,
        fill=False,
        linestyle=":",
        linewidth=2.5,
        color="tab:blue",
        label="effective cylinder",
    )
    ax.add_patch(circle)
    ax.scatter([args.obs_x], [args.obs_y], marker="x", s=70, color="tab:blue", label="center")

    planes = halfspace_rows(rows)
    plotted = 0
    for i, (a0, a1, b) in enumerate(planes):
        if i % max(1, args.plane_stride) != 0:
            continue
        draw_halfspace(
            ax,
            a0,
            a1,
            b,
            xmin,
            xmax,
            ymin,
            ymax,
            color="tab:orange",
            alpha=args.plane_alpha,
            linewidth=1.4,
            label="applied half-spaces" if plotted == 0 else None,
        )
        plotted += 1

    ax.set_title(f"{csv_path.name}: {len(pts)} samples, {len(planes)} active/applied planes")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_xlim(xmin, xmax)
    ax.set_ylim(ymin, ymax)
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="best")

    out = Path(args.out) if args.out else csv_path.with_suffix(".png")
    fig.tight_layout()
    fig.savefig(out, dpi=180)
    print(f"wrote {out}")


if __name__ == "__main__":
    main()
