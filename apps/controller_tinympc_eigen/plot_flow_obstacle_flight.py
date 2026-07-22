#!/usr/bin/env python3
"""Plot a TinyMPC flow-obstacle flight CSV."""

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
    p.add_argument("csv", nargs="?", default="flow_obstacle_flight.csv")
    p.add_argument("--out", default=None, help="Output image path. Defaults to <csv>.png")
    p.add_argument("--radius", type=float, default=None,
                   help="Cylinder radius to draw. Defaults to logged frozen/effective radius.")
    p.add_argument("--safety", type=float, default=0.0,
                   help="Extra safety radius to draw around the frozen cylinder.")
    p.add_argument("--truth-x", type=float, default=None, help="Known obstacle x [m].")
    p.add_argument("--truth-y", type=float, default=None, help="Known obstacle y [m].")
    p.add_argument("--truth-radius", type=float, default=0.25, help="Known obstacle radius [m].")
    return p.parse_args()


def f(row, key):
    val = row.get(key, "")
    if val is None or val == "":
        return None
    try:
        out = float(val)
    except ValueError:
        return None
    return out if math.isfinite(out) else None


def series(rows, key):
    out = []
    for i, row in enumerate(rows):
        val = f(row, key)
        if val is not None:
            out.append((i, val))
    return out


def xy(rows, x_key, y_key):
    pts = []
    for row in rows:
        x = f(row, x_key)
        y = f(row, y_key)
        if x is not None and y is not None:
            pts.append((x, y, row.get("phase", "")))
    return pts


def latest_positive(rows, key):
    for row in reversed(rows):
        val = f(row, key)
        if val is not None and val > 0.0:
            return val
    return None


def main():
    args = parse_args()
    csv_path = Path(args.csv)
    with csv_path.open(newline="") as fp:
        rows = list(csv.DictReader(fp))
    if not rows:
        raise SystemExit(f"no rows in {csv_path}")

    traj = xy(rows, "state_x", "state_y")
    cmds = xy(rows, "cmd_x", "cmd_y")
    live = xy(rows, "flow_cyl_wx", "flow_cyl_wy")
    frozen_x = latest_positive(rows, "obs_frz_cx")
    frozen_y = latest_positive(rows, "obs_frz_cy")
    radius = args.radius or latest_positive(rows, "obs_eff_rad") or 0.25
    eff_radius = radius + args.safety

    fig, (ax_xy, ax_conf, ax_z) = plt.subplots(
        3, 1, figsize=(10, 10), gridspec_kw={"height_ratios": [2.2, 1.0, 1.0]}
    )

    if cmds:
        ax_xy.plot([p[0] for p in cmds], [p[1] for p in cmds],
                   color="0.70", linewidth=1.5, label="command")
    if traj:
        ax_xy.plot([p[0] for p in traj], [p[1] for p in traj],
                   color="black", linewidth=2.0, label="state")
        ax_xy.scatter([traj[0][0]], [traj[0][1]], s=60, color="forestgreen", label="start", zorder=5)
        ax_xy.scatter([traj[-1][0]], [traj[-1][1]], s=60, color="tab:red", label="end", zorder=5)

    live_pts = [(x, y) for x, y, _ in live if abs(x) > 1.0e-6 or abs(y) > 1.0e-6]
    if live_pts:
        ax_xy.scatter([p[0] for p in live_pts], [p[1] for p in live_pts],
                      s=12, alpha=0.35, color="tab:purple", label="live cyl estimate")

    if frozen_x is not None and frozen_y is not None:
        ax_xy.scatter([frozen_x], [frozen_y], marker="x", s=90, color="tab:blue", label="frozen center")
        ax_xy.add_patch(plt.Circle((frozen_x, frozen_y), radius, fill=False,
                                   linewidth=2.0, color="tab:blue", label="frozen radius"))
        if args.safety > 0.0:
            ax_xy.add_patch(plt.Circle((frozen_x, frozen_y), eff_radius, fill=False,
                                       linestyle=":", linewidth=2.0, color="tab:orange",
                                       label="radius + safety"))

    if args.truth_x is not None and args.truth_y is not None:
        ax_xy.scatter([args.truth_x], [args.truth_y], marker="+", s=120,
                      color="tab:red", label="known obstacle")
        ax_xy.add_patch(plt.Circle((args.truth_x, args.truth_y), args.truth_radius,
                                   fill=False, linestyle="--", linewidth=2.0,
                                   color="tab:red", label="known radius"))

    ax_xy.set_title(csv_path.name)
    ax_xy.set_xlabel("x [m]")
    ax_xy.set_ylabel("y [m]")
    ax_xy.grid(True, alpha=0.3)
    ax_xy.set_aspect("equal", adjustable="box")
    ax_xy.legend(loc="best")

    for key, label, color in [
        ("map_peak", "mapPeak", "tab:green"),
        ("flow_cyl_conf", "cylConf", "tab:purple"),
        ("flow_cyl_valid", "cylValid", "tab:blue"),
        ("obs_frz_valid", "frzValid", "tab:red"),
        ("obs_apply", "obsApply", "tab:orange"),
    ]:
        vals = series(rows, key)
        if vals:
            ax_conf.plot([i for i, _ in vals], [v for _, v in vals], label=label, color=color)
    ax_conf.set_ylabel("confidence / flags")
    ax_conf.set_ylim(-0.05, 1.05)
    ax_conf.grid(True, alpha=0.3)
    ax_conf.legend(loc="best", ncol=3)

    for key, label, color in [
        ("state_z", "z", "black"),
        ("cmd_z", "cmd z", "0.65"),
        ("obs_clearance", "obs clearance", "tab:red"),
    ]:
        vals = series(rows, key)
        if vals:
            ax_z.plot([i for i, _ in vals], [v for _, v in vals], label=label, color=color)
    ax_z.axhline(0.0, color="0.3", linewidth=1.0, alpha=0.5)
    ax_z.set_xlabel("CSV row")
    ax_z.set_ylabel("m")
    ax_z.grid(True, alpha=0.3)
    ax_z.legend(loc="best")

    out = Path(args.out) if args.out else csv_path.with_suffix(".png")
    fig.tight_layout()
    fig.savefig(out, dpi=180)
    print(f"wrote {out}")


if __name__ == "__main__":
    main()
