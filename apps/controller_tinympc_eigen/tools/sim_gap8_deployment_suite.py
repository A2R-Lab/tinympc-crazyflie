#!/usr/bin/env python3
"""End-to-end scenes using the exact gate8-dory-package GAP8 flow frontend."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

from sim_flow_obstacle_sectors import FirmwareMirror
from sim_gap8_flow_frontend import CX, FX, H, W, Gap8FlowFrontend
from sim_monocular_flow_suite import SCENES, Scene, ray_hit, texture


def render_gap8(scene: Scene, x: float, y: float, yaw: float,
                exposure_scale: float = 1.0, brightness: float = 0.0) -> list[list[int]]:
    image = [[0] * W for _ in range(H)]
    for col in range(W):
        az = math.atan((col + 0.5 - CX) / FX)
        distance, box, face = ray_hit(x, y, yaw + az, scene.boxes)
        wx, wy = x + distance*math.cos(yaw+az), y + distance*math.sin(yaw+az)
        for row in range(H):
            z = 1.0 + (H/2-row-0.5)*distance/FX
            raw = texture(5 if box is None else box.texture, wx, wy, z, face)
            image[row][col] = max(0, min(255, round(raw*exposure_scale+brightness)))
    return image


def run_scene(scene: Scene, args: argparse.Namespace) -> dict[str, object]:
    gap8 = Gap8FlowFrontend(balanced_features=bool(getattr(args, "balanced_features", False)))
    stm32 = FirmwareMirror()
    valid: list[dict[str, float | int | bool]] = []
    track_counts: list[int] = []
    # The producer snapshots at 5 Hz even though the HM01B0 captures at 30 Hz.
    for frame in range(args.snapshots + 1):
        t = frame * 0.2
        y = scene.start_y + scene.vy*t
        image = render_gap8(scene, 0.0, y, 0.0)
        sectors, stats = gap8.process(image, 0.2)
        track_counts.append(stats["tracks"])
        if sectors is None:
            continue
        # 50 Hz STM32 controller reuses the latest 5 Hz payload ten times.
        for repeat in range(10):
            now_y = y + scene.vy*(0.014583 + repeat*0.02)
            result = stm32.update(sectors, 0.0, scene.vy, 0.0,
                                  0.0, now_y, 0.0, new_sample=(repeat == 0))
            if result["cylinder"]["valid"]:
                valid.append(result["cylinder"])
    final = valid[-1] if valid else None
    error = None
    if final and scene.truth:
        error = math.hypot(float(final["world_x"])-scene.truth[0],
                           float(final["world_y"])-scene.truth[1])
    passed = ((error is not None and error <= args.pass_error) if scene.detectable
              else final is None)
    return {"scene": scene.name, "detection_expected": scene.detectable,
            "valid_controller_ticks": len(valid),
            "mean_tracks_per_snapshot": sum(track_counts)/len(track_counts),
            "estimate": None if final is None else [final["world_x"], final["world_y"]],
            "truth": scene.truth, "error_m": error, "pass": passed}


def parse_args() -> argparse.Namespace:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--snapshots", type=int, default=15)
    ap.add_argument("--pass-error", type=float, default=0.35)
    ap.add_argument("--balanced-features", action="store_true",
                    help="evaluate proposed five-features-per-sector GAP8 allocation")
    ap.add_argument("--out", type=Path)
    return ap.parse_args()


def main() -> int:
    args = parse_args()
    results = [run_scene(scene, args) for scene in SCENES]
    summary = {"passed": sum(bool(r["pass"]) for r in results),
               "total": len(results), "results": results}
    print(json.dumps(summary, indent=2))
    if args.out:
        args.out.mkdir(parents=True, exist_ok=True)
        (args.out/"summary.json").write_text(json.dumps(summary, indent=2)+"\n")
    return 0 if summary["passed"] == summary["total"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
