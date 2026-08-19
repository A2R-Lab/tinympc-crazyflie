#!/usr/bin/env python3
"""Vision-free TinyMPC maneuver simulation entry point."""

from __future__ import annotations

import sys

from run_depth_constraint_sim import main as run_simulation


def main() -> int:
    forbidden = {"--perception-mode", "--onnx-model", "--neural-danger-clearance", "--neural-plane-margin"}
    supplied = [argument.split("=", 1)[0] for argument in sys.argv[1:]]
    incompatible = sorted(forbidden.intersection(supplied))
    if incompatible:
        raise SystemExit(
            "run_maneuver_sim.py is vision-free; use the detached benchmark or legacy "
            f"obstacle runner for: {', '.join(incompatible)}"
        )
    arguments = [*sys.argv[1:]]
    if "--trajectory-handoff-hold-s" not in supplied:
        # controllerOutOfTree() freezes every reference knot at trajectory
        # sample zero for 2/5 second after activation.
        arguments.extend(["--trajectory-handoff-hold-s", "0.4"])
    return run_simulation([*arguments, "--perception-mode", "none"])


if __name__ == "__main__":
    raise SystemExit(main())
