#!/usr/bin/env python3
"""Dependency-free monocular render -> image flow -> firmware obstacle suite.

This is a deliberately small horizontal pinhole renderer, not a photorealistic
engine. Vertical box faces and a background wall are textured procedurally,
rendered to grayscale images, and matched with sector-wise SSD block flow.
Only the rendered pixels and known body velocity enter the firmware mirror.
"""

from __future__ import annotations

import argparse
import json
import math
from dataclasses import dataclass
from pathlib import Path

from sim_flow_obstacle_sectors import FirmwareMirror, Sector


@dataclass(frozen=True)
class Box:
    name: str
    xmin: float
    xmax: float
    ymin: float
    ymax: float
    texture: int = 1


@dataclass(frozen=True)
class Scene:
    name: str
    boxes: tuple[Box, ...]
    start_y: float = -0.12
    vy: float = 0.12
    truth: tuple[float, float] | None = None
    detectable: bool = True


SCENES = (
    Scene("box_1m", (Box("target", 1.0, 1.2, -0.42, 0.42, 1),), truth=(1.0, 0.0)),
    Scene("box_left", (Box("target", 1.0, 1.2, 0.05, 0.65, 2),), truth=(1.0, 0.25)),
    Scene("cluttered", (Box("target", 1.0, 1.2, -0.4, 0.4, 1),
                         Box("far", 2.0, 2.3, -1.0, -0.35, 3),
                         Box("side", 1.5, 1.8, 0.7, 1.1, 4)), truth=(1.0, 0.0)),
    Scene("two_depths", (Box("near", 1.0, 1.2, -0.55, -0.02, 2),
                          Box("far", 1.8, 2.0, 0.02, 0.65, 3)), truth=(1.0, -0.25)),
    Scene("narrow", (Box("target", 1.0, 1.2, -0.12, 0.12, 1),), truth=(1.0, 0.0)),
    Scene("far", (Box("target", 2.6, 2.8, -0.55, 0.55, 2),), truth=(2.6, 0.0)),
    Scene("low_texture", (Box("target", 1.0, 1.2, -0.42, 0.42, 0),),
          truth=(1.0, 0.0), detectable=False),
)


def _hash(x: int) -> int:
    x = ((x >> 16) ^ x) * 0x45D9F3B
    x = ((x >> 16) ^ x) * 0x45D9F3B
    return (x >> 16) ^ x


def texture(texture_id: int, x: float, y: float, z: float, face: int) -> int:
    if texture_id == 0:
        return 125
    a, b = (y, z) if face < 2 else (x, z)
    # Continuous multi-frequency texture avoids nearest-cell flicker while
    # retaining enough unique structure for subpixel patch matching.
    phase = texture_id * 1.713
    value = (128 + 46 * math.sin(17.0 * a + 9.0 * b + phase)
             + 31 * math.sin(31.0 * a - 13.0 * b + 0.7 * phase)
             + 18 * math.sin(7.0 * a + 27.0 * b - phase))
    return max(0, min(255, round(value)))


def ray_hit(px: float, py: float, angle: float, boxes: tuple[Box, ...]) -> tuple[float, Box | None, int]:
    dx, dy = math.cos(angle), math.sin(angle)
    best = 4.0 / max(0.15, dx)
    hit: Box | None = None
    hit_face = 0
    for box in boxes:
        t0, t1, face = -math.inf, math.inf, 0
        for axis, (origin, direction, lo, hi) in enumerate(((px, dx, box.xmin, box.xmax),
                                                            (py, dy, box.ymin, box.ymax))):
            if abs(direction) < 1e-12:
                if not lo <= origin <= hi:
                    t0, t1 = 1, 0
                continue
            a, b = (lo - origin) / direction, (hi - origin) / direction
            near_face = axis * 2 + (1 if a > b else 0)
            if a > b:
                a, b = b, a
            if a > t0:
                t0, face = a, near_face
            t1 = min(t1, b)
        if t1 >= max(0.0, t0) and 0.0 < t0 < best:
            best, hit, hit_face = t0, box, face
    return best, hit, hit_face


def render(scene: Scene, x: float, y: float, yaw: float,
           width: int, height: int, fov: float) -> list[list[int]]:
    image = [[0] * width for _ in range(height)]
    for col in range(width):
        az = -fov / 2 + (col + 0.5) * fov / width
        distance, box, face = ray_hit(x, y, yaw + az, scene.boxes)
        wx, wy = x + distance * math.cos(yaw + az), y + distance * math.sin(yaw + az)
        for row in range(height):
            z = 1.0 + (height / 2 - row - 0.5) * distance * 2 * math.tan(math.radians(25)) / height
            if box is None:
                image[row][col] = texture(5, wx, wy, z, 0)
            else:
                image[row][col] = texture(box.texture, wx, wy, z, face)
    return image


def sector_flow(prev: list[list[int]], cur: list[list[int]], sectors: int,
                fov: float, dt: float, max_shift: int = 4) -> list[Sector]:
    height, width = len(prev), len(prev[0])
    result: list[Sector] = []
    radius = 2
    margin = max_shift + radius + 1
    for sec in range(sectors):
        lo = max(margin, sec * width // sectors)
        hi = min(width - margin, (sec + 1) * width // sectors)
        tracks: list[tuple[float, float]] = []
        for row in range(margin, height - margin, 5):
            for col in range(lo, hi, 4):
                contrast = sum(abs(prev[rr][cc + 1] - prev[rr][cc - 1])
                               for rr in range(row - radius, row + radius + 1)
                               for cc in range(col - radius, col + radius + 1)) / 25
                if contrast < 12:
                    continue
                costs: list[float] = []
                for shift_i in range(-max_shift, max_shift + 1):
                    ssd = sum((cur[rr][cc + shift_i] - prev[rr][cc]) ** 2
                              for rr in range(row - radius, row + radius + 1)
                              for cc in range(col - radius, col + radius + 1)) / 25
                    costs.append(ssd)
                k = min(range(len(costs)), key=costs.__getitem__)
                if k == 0 or k == len(costs) - 1:
                    continue
                ordered = sorted(costs)
                uniqueness = (ordered[1] - ordered[0]) / max(20.0, ordered[1])
                if uniqueness < 0.015:
                    continue
                shift = float(k - max_shift)
                a, b, c = costs[k - 1], costs[k], costs[k + 1]
                denom = a - 2 * b + c
                if denom > 1e-9:
                    shift += max(-0.5, min(0.5, 0.5 * (a - c) / denom))
                tracks.append((shift, min(1.0, uniqueness * 4)))
        # Prefer the closer half of the tracked surfaces. With lateral motion,
        # inverse depth is proportional to absolute horizontal image motion.
        tracks.sort(key=lambda item: abs(item[0]), reverse=True)
        selected = tracks[:max(1, (len(tracks) + 1) // 2)]
        if selected:
            shifts = sorted(item[0] for item in selected)
            shift = shifts[len(shifts) // 2]
            confidence = min(1.0, len(selected) / 5.0) * sum(item[1] for item in selected) / len(selected)
        else:
            shift, confidence = 0.0, 0.0
        az = -fov / 2 + (sec + 0.5) * fov / sectors
        image_q = math.tan(az)
        angular_flow = shift * fov / width / dt
        result.append(Sector(image_q, angular_flow * (1.0 + image_q**2), confidence))
    return result


def run_scene(scene: Scene, args: argparse.Namespace) -> dict[str, object]:
    fov = math.radians(args.fov_deg)
    est = FirmwareMirror()
    prev = render(scene, 0.0, scene.start_y, 0.0, args.width, args.height, fov)
    valid: list[dict[str, float | int | bool]] = []
    for frame in range(1, args.frames + 1):
        t = frame * args.dt
        y = scene.start_y + scene.vy * t
        cur = render(scene, 0.0, y, 0.0, args.width, args.height, fov)
        sectors = sector_flow(prev, cur, args.sectors, fov, args.dt, args.max_shift)
        result = est.update(sectors, 0.0, scene.vy, 0.0, 0.0, y, 0.0)
        if result["cylinder"]["valid"]:
            valid.append(result["cylinder"])
        prev = cur
    final = valid[-1] if valid else None
    error = None
    if final and scene.truth:
        error = math.hypot(float(final["world_x"]) - scene.truth[0],
                           float(final["world_y"]) - scene.truth[1])
    passed = ((error is not None and error <= args.pass_error) if scene.detectable
              else final is None)
    return {"scene": scene.name, "valid_frames": len(valid), "frames": args.frames,
            "detection_expected": scene.detectable,
            "estimate": None if final is None else [final["world_x"], final["world_y"]],
            "truth": scene.truth, "error_m": error,
            "pass": passed}


def parse_args() -> argparse.Namespace:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--width", type=int, default=120)
    ap.add_argument("--height", type=int, default=72)
    ap.add_argument("--fov-deg", type=float, default=70)
    ap.add_argument("--sectors", type=int, default=9)
    ap.add_argument("--frames", type=int, default=35)
    ap.add_argument("--dt", type=float, default=0.1)
    ap.add_argument("--max-shift", type=int, default=4)
    ap.add_argument("--pass-error", type=float, default=0.25)
    ap.add_argument("--out", type=Path)
    return ap.parse_args()


def main() -> int:
    args = parse_args()
    results = [run_scene(scene, args) for scene in SCENES]
    passed = sum(bool(r["pass"]) for r in results)
    summary = {"passed": passed, "total": len(results), "results": results}
    print(json.dumps(summary, indent=2))
    if args.out:
        args.out.mkdir(parents=True, exist_ok=True)
        (args.out / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    return 0 if passed == len(results) else 2


if __name__ == "__main__":
    raise SystemExit(main())
