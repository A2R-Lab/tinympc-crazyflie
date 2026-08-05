#!/usr/bin/env python3
"""Plot one closed-loop depth-constraint run.

The input is a run directory produced by tools/pybullet_simulation/run_depth_constraint_sim.py, for
example flow_sim_dataset/depth_constraint_batch/case_010_avoid.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("run_dir", type=Path, help="directory containing closed_loop.csv and summary.json")
    ap.add_argument("--out", type=Path, default=None, help="output PNG path; default: RUN_DIR/trajectory_topdown.png")
    ap.add_argument("--stride", type=int, default=5, help="draw every Nth active half-space normal")
    args = ap.parse_args()

    run_dir = args.run_dir
    if not (run_dir / "closed_loop.csv").exists():
        children = sorted(path for path in run_dir.iterdir() if (path / "closed_loop.csv").exists())
        if not children:
            raise SystemExit(f"{run_dir} is neither a run dir nor a directory containing run dirs")
        for child in children:
            rows = _read_rows(child / "closed_loop.csv")
            plan_rows = _read_rows(child / "planned_horizon.csv") if (child / "planned_horizon.csv").exists() else []
            summary = json.loads((child / "summary.json").read_text())
            _plot(rows, plan_rows, summary, child / "trajectory_topdown.png", max(1, int(args.stride)))
        print(f"wrote {len(children)} plots under {run_dir}")
        return 0

    rows = _read_rows(run_dir / "closed_loop.csv")
    plan_rows = _read_rows(run_dir / "planned_horizon.csv") if (run_dir / "planned_horizon.csv").exists() else []
    summary = json.loads((run_dir / "summary.json").read_text())
    if not rows:
        raise SystemExit(f"no rows in {run_dir / 'closed_loop.csv'}")

    out = args.out or (run_dir / "trajectory_topdown.png")
    _plot(rows, plan_rows, summary, out, max(1, int(args.stride)))
    print(f"wrote {out}")
    return 0


def _read_rows(path: Path) -> list[dict[str, str]]:
    with path.open(newline="") as f:
        return list(csv.DictReader(f))


def _f(row: dict[str, str], key: str, default: float = math.nan) -> float:
    raw = row.get(key, "")
    if raw == "":
        return default
    return float(raw)


def _plot(
    rows: list[dict[str, str]],
    plan_rows: list[dict[str, str]],
    summary: dict,
    out: Path,
    stride: int,
) -> None:
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        from matplotlib.patches import Rectangle
    except Exception as exc:
        raise SystemExit(f"matplotlib is required for plotting: {exc}") from exc

    xs = [_f(r, "x") for r in rows]
    ys = [_f(r, "y") for r in rows]
    next_x = [_f(r, "next_x") for r in rows]
    next_y = [_f(r, "next_y") for r in rows]

    fig, ax = plt.subplots(figsize=(8.5, 5.0))
    ax.plot(xs, ys, color="black", linewidth=1.5, label="state")
    ax.scatter(xs[0], ys[0], color="green", s=45, zorder=3, label="start")
    ax.scatter(next_x[-1], next_y[-1], color="red" if summary.get("collision") else "blue", s=55, zorder=3, label="end")

    goal = summary.get("goal") or {}
    goal_x = float(goal.get("x", 3.0))
    goal_y = float(goal.get("y", 0.0))
    ax.scatter(goal_x, goal_y, marker="*", color="#7b2cff", s=120, zorder=4, label="destination")

    if plan_rows:
        _plot_full_horizon_plan(ax, plan_rows, stride)
    else:
        _plot_terminal_plan_trace(ax, rows)

    for obstacle in summary.get("obstacles", []):
        cx, cy, _ = obstacle["center"]
        hx, hy, _ = obstacle["half_extents"]
        ax.add_patch(
            Rectangle(
                (cx - hx, cy - hy),
                2.0 * hx,
                2.0 * hy,
                facecolor="#2d7ff9",
                edgecolor="#0b2f66",
                alpha=0.35,
                linewidth=1.2,
            )
        )
        ax.text(cx, cy, obstacle.get("name", "obstacle"), ha="center", va="center", fontsize=8)

    active_rows = [r for r in rows if int(float(r.get("constraint_active", "0") or 0)) != 0]
    for idx, row in enumerate(active_rows[::stride]):
        x = _f(row, "x")
        y = _f(row, "y")
        axv = _f(row, "a_x")
        ayv = _f(row, "a_y")
        b = _f(row, "b")
        if not all(math.isfinite(v) for v in (x, y, axv, ayv, b)):
            continue
        # Draw the outward normal from the current pose and a short segment of the
        # actual half-space boundary a_x*x + a_y*y = b in top-down projection.
        ax.arrow(x, y, 0.12 * axv, 0.12 * ayv, head_width=0.025, color="#ff8a00", alpha=0.55, length_includes_head=True)
        denom = axv * axv + ayv * ayv
        if denom > 1e-9:
            px = (b / denom) * axv
            py = (b / denom) * ayv
            tx = -ayv
            ty = axv
            scale = 0.35
            ax.plot([px - scale * tx, px + scale * tx], [py - scale * ty, py + scale * ty], color="#ff8a00", alpha=0.25)
        if idx > 80:
            break

    title = f"{out.parent.name}: collision={summary.get('collision')} clearance={summary.get('min_obstacle_clearance_m'):.3f} m"
    ax.set_title(title)
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.axis("equal")
    ax.grid(True, alpha=0.25)
    ax.legend(loc="best")
    fig.tight_layout()
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out, dpi=180)
    plt.close(fig)


def _plot_terminal_plan_trace(ax, rows: list[dict[str, str]]) -> None:
    tx = [_f(r, "admm_terminal_x") for r in rows]
    ty = [_f(r, "admm_terminal_y") for r in rows]
    pts = [(x, y) for x, y in zip(tx, ty) if math.isfinite(x) and math.isfinite(y)]
    if not pts:
        return
    ax.plot(
        [p[0] for p in pts],
        [p[1] for p in pts],
        color="#7b2cff",
        linestyle="--",
        linewidth=1.0,
        alpha=0.8,
        label="planned terminal trace",
    )


def _plot_full_horizon_plan(ax, plan_rows: list[dict[str, str]], stride: int) -> None:
    by_step: dict[int, list[dict[str, str]]] = {}
    for row in plan_rows:
        by_step.setdefault(int(float(row["step"])), []).append(row)
    if not by_step:
        return

    steps = sorted(by_step)
    plotted = 0
    for step in steps[::stride]:
        horizon = sorted(by_step[step], key=lambda r: int(float(r["k"])))
        xs = [_f(r, "x") for r in horizon]
        ys = [_f(r, "y") for r in horizon]
        if not xs or not all(math.isfinite(v) for v in xs + ys):
            continue
        ax.plot(
            xs,
            ys,
            color="#7b2cff",
            linewidth=0.8,
            alpha=0.18,
            label="planned horizons" if plotted == 0 else None,
        )
        plotted += 1

    latest = sorted(by_step[steps[-1]], key=lambda r: int(float(r["k"])))
    xs = [_f(r, "x") for r in latest]
    ys = [_f(r, "y") for r in latest]
    if xs and all(math.isfinite(v) for v in xs + ys):
        ax.plot(xs, ys, color="#7b2cff", linewidth=1.6, alpha=0.9, label="final planned horizon")


if __name__ == "__main__":
    raise SystemExit(main())
