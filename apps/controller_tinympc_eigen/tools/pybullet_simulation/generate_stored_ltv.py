#!/usr/bin/env python3
"""Generate an entirely offline LTV model/cache artifact for one maneuver."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from stored_ltv_controller import generate_artifact, render_firmware_header


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("trajectory", type=Path)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--rho", type=float, default=250.0)
    parser.add_argument("--firmware-header", type=Path)
    args = parser.parse_args()
    result = generate_artifact(args.trajectory, args.out, args.rho)
    if args.firmware_header is not None:
        render_firmware_header(args.out, args.firmware_header)
        result["firmware_header"] = str(args.firmware_header)
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
