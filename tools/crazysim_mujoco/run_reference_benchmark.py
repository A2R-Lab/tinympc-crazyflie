#!/usr/bin/env python3
"""Run the reproducible three-method CrazySim reference benchmark matrix."""

from __future__ import annotations

import argparse
import subprocess
from pathlib import Path


SHAPES = ("figure8", "oval", "circle")
METHODS = ("waypoint", "progress", "trajectory")
SEEDS = (1, 2, 3)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", type=Path, default=Path(
        "apps/controller_tinympc_eigen/sim_runs/crazysim/reference_method_benchmark_v2"
    ))
    parser.add_argument("--overwrite", action="store_true")
    parser.add_argument("--analyze-only", action="store_true")
    args = parser.parse_args()
    repo = Path(__file__).resolve().parents[2]
    runner = repo / "tools/crazysim_mujoco/run.sh"
    analyzer = repo / "tools/crazysim_mujoco/compare_reference_methods.py"
    root = args.root if args.root.is_absolute() else repo / args.root
    runs = []
    for shape in SHAPES:
        for method in METHODS:
            for seed in SEEDS:
                run = root / f"{shape}_{method}_seed{seed}"
                runs.append(run)
                if args.analyze_only or ((run / "summary.json").is_file() and not args.overwrite):
                    continue
                command = [
                    str(runner), "--trajectory", shape,
                    "--reference-mode", method,
                    "--stored-ltv", "0", "--actuator-lti", "1",
                    "--duration", (
                        "130" if method == "waypoint" else
                        "110" if method == "progress" else "23"
                    ),
                    "--launch-time", "2", "--random-seed", str(seed),
                    "--realtime-factor", "1", "--firmware-time-factor", "0.7",
                    "--stop-on-contact",
                    "--out", str(run.relative_to(repo)),
                ]
                if args.overwrite:
                    command.append("--overwrite")
                print("running", shape, method, f"seed={seed}", flush=True)
                subprocess.run(command, cwd=repo, check=True)
    subprocess.run([
        "python3", str(analyzer), *(str(run) for run in runs),
        "--headers", str(repo / "apps/controller_tinympc_eigen/src/trajectories/50hz"),
        "--out", str(root),
    ], cwd=repo, check=True)


if __name__ == "__main__":
    main()
