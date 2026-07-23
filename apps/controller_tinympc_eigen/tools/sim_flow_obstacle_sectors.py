#!/usr/bin/env python3
"""Offline mirror of flowdeck_obstacle_link.c's obstacle estimator.

Synthetic horizontal sector rays intersect a world-aligned box.  Their ideal
angular flow is then fed through the same inverse-depth, contiguous-cluster,
persistent evidence-map, and cylinder-extraction equations as the firmware.
No Crazyflie or radio APIs are used.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import random
from dataclasses import dataclass
from pathlib import Path


SECT_MAX = 9
MIN_CONFIDENCE = 0.02
MIN_TRANSLATION = 0.08
MAX_INV_DEPTH = 8.0
MAX_RANGE = 10.0
CANDIDATE_MAX_RANGE = 3.0
CANDIDATE_MAX_YAW_RATE = 0.6
CANDIDATE_MIN_SECTORS = 2
CANDIDATE_MAX_RANGE_JUMP = 0.45
CANDIDATE_ALPHA = 0.35
MAP_CELLS = 16
MAP_DECAY = 0.999
MAP_MIN_EVIDENCE = 0.03
MAP_VOTE = 0.18
MAP_MERGE_RADIUS = 0.55
MAP_EXTRACT_RADIUS = 0.55
MAP_VALID_EVIDENCE = 0.20
CYL_VALID_CONF = 0.20
CYL_VALID_ACCEPTS = 2


@dataclass
class Sector:
    azimuth: float
    flow_x: float
    confidence: float
    truth_range: float | None = None
    flow_y: float = 0.0


@dataclass
class Cell:
    x: float = 0.0
    y: float = 0.0
    evidence: float = 0.0
    hits: int = 0


class FirmwareMirror:
    """Stateful, equation-for-equation sector estimator mirror."""

    def __init__(self) -> None:
        self.cells = [Cell() for _ in range(MAP_CELLS)]
        self.obs_hits = 0
        self.obs_body_x = self.obs_body_y = 0.0
        self.cylinder: dict[str, float | int | bool] = {
            "valid": False, "world_x": 0.0, "world_y": 0.0,
            "body_x": 0.0, "body_y": 0.0, "confidence": 0.0,
            "accepts": 0,
        }

    def update(self, sectors: list[Sector], vx: float, vy: float,
               yaw_rate: float, world_x: float, world_y: float,
               yaw: float, new_sample: bool = True) -> dict[str, object]:
        c, s = math.cos(yaw), math.sin(yaw)
        derived: list[dict[str, float | bool]] = []
        candidate: list[bool] = []
        for i in range(SECT_MAX):
            if i >= len(sectors) or sectors[i].confidence < MIN_CONFIDENCE:
                derived.append({"valid": False})
                candidate.append(False)
                continue
            sec = sectors[i]
            # Deployed GAP8 fields are normalized image coordinate/rate.
            az = math.atan(sec.azimuth)
            measured_angular_flow = sec.flow_x / (1.0 + sec.azimuth**2)
            residual = measured_angular_flow - yaw_rate
            vel_eff = vx * math.sin(az) - vy * math.cos(az)
            use_looming = (abs(vel_eff) < MIN_TRANSLATION and abs(vx) >= 0.15 and
                           abs(yaw_rate) < 0.20 and abs(sec.flow_y) >= 1e-3)
            if use_looming:
                residual = sec.flow_y
                vel_eff = vx
            if abs(vel_eff) < MIN_TRANSLATION or abs(residual) < 1e-3:
                derived.append({"valid": False, "residual": residual,
                                "vel_eff": vel_eff})
                candidate.append(False)
                continue
            signed_inv_depth = residual / vel_eff
            if signed_inv_depth <= 0.0:
                derived.append({"valid": False, "residual": residual,
                                "vel_eff": vel_eff})
                candidate.append(False)
                continue
            inv_depth = min(signed_inv_depth, MAX_INV_DEPTH)
            distance = min(1.0 / inv_depth if inv_depth > 1e-3 else 0.0,
                           MAX_RANGE)
            bx = distance * math.cos(az)
            by = distance * math.sin(az)
            wx = world_x + c * bx - s * by
            wy = world_y + s * bx + c * by
            derived.append({"valid": True, "residual": residual,
                            "vel_eff": vel_eff, "inv_depth": inv_depth,
                            "range": distance, "body_x": bx, "body_y": by,
                            "world_x": wx, "world_y": wy})
            candidate.append(abs(yaw_rate) < CANDIDATE_MAX_YAW_RATE and
                             0.0 < distance < CANDIDATE_MAX_RANGE)

        best_start, best_count, best_sum = 0, 0, 0.0
        start = 0
        while start < SECT_MAX:
            while start < SECT_MAX and not candidate[start]:
                start += 1
            count, range_sum = 0, 0.0
            while start + count < SECT_MAX and candidate[start + count]:
                idx = start + count
                if count and abs(float(derived[idx]["range"]) -
                                 float(derived[idx - 1]["range"])) > CANDIDATE_MAX_RANGE_JUMP:
                    break
                range_sum += float(derived[start + count]["range"])
                count += 1
            if count > best_count or (count == best_count and count and range_sum < best_sum):
                best_start, best_count, best_sum = start, count, range_sum
            start += max(count, 1)

        fresh = best_count >= CANDIDATE_MIN_SECTORS
        obs: dict[str, float | int | bool] = {"fresh": fresh, "start": best_start,
                                              "count": best_count, "valid": False}
        if fresh:
            indexes = range(best_start, best_start + best_count)
            weights = [max(sectors[i].confidence, 1e-3) for i in indexes]
            weight_sum = sum(weights)
            raw_x = sum(w * float(derived[i]["body_x"]) for w, i in zip(weights, indexes)) / weight_sum
            raw_y = sum(w * float(derived[i]["body_y"]) for w, i in zip(weights, indexes)) / weight_sum
            if new_sample:
                if self.obs_hits == 0:
                    self.obs_body_x, self.obs_body_y = raw_x, raw_y
                else:
                    self.obs_body_x += CANDIDATE_ALPHA * (raw_x - self.obs_body_x)
                    self.obs_body_y += CANDIDATE_ALPHA * (raw_y - self.obs_body_y)
                self.obs_hits = min(255, self.obs_hits + 1)
            owx = world_x + c * self.obs_body_x - s * self.obs_body_y
            owy = world_y + s * self.obs_body_x + c * self.obs_body_y
            obs.update(valid=self.obs_hits >= 2, body_x=self.obs_body_x,
                       body_y=self.obs_body_y, world_x=owx, world_y=owy,
                       range=best_sum / best_count)
        elif new_sample:
            self.obs_hits = max(0, self.obs_hits - 1)
            obs["valid"] = self.obs_hits >= 2

        if new_sample:
            self._decay(MAP_DECAY)
            if fresh:
                self._vote(float(obs["world_x"]), float(obs["world_y"]),
                           min(1.0, 0.5 + 0.15 * best_count))
        self._extract(world_x, world_y, yaw)
        return {"sectors": derived, "cluster": obs,
                "cylinder": dict(self.cylinder)}

    def _decay(self, factor: float) -> None:
        for cell in self.cells:
            cell.evidence *= factor
            if cell.evidence < MAP_MIN_EVIDENCE:
                cell.evidence, cell.hits = 0.0, 0

    def _vote(self, x: float, y: float, weight: float) -> None:
        weakest = min(range(MAP_CELLS), key=lambda i: self.cells[i].evidence)
        merged = False
        r2 = MAP_MERGE_RADIUS ** 2
        for cell in self.cells:
            if cell.evidence <= 0.0:
                continue
            d2 = (x - cell.x) ** 2 + (y - cell.y) ** 2
            if d2 <= r2:
                proximity = 1.0 - d2 / r2
                cell.x += 0.12 * weight * proximity * (x - cell.x)
                cell.y += 0.12 * weight * proximity * (y - cell.y)
                cell.evidence = min(1.0, cell.evidence + MAP_VOTE * weight * proximity)
                cell.hits = min(255, cell.hits + 1)
                merged = True
        if not merged:
            self.cells[weakest] = Cell(x, y, MAP_VOTE * weight, 1)

    def _extract(self, world_x: float, world_y: float, yaw: float) -> None:
        active = [c for c in self.cells if c.evidence > 0.0]
        peak = max(active, key=lambda c: c.evidence) if active else None
        if peak is None or peak.evidence < MAP_VALID_EVIDENCE:
            self.cylinder.update(valid=False, confidence=peak.evidence if peak else 0.0)
            return
        support = [c for c in active if (c.x - peak.x) ** 2 + (c.y - peak.y) ** 2 <= MAP_EXTRACT_RADIUS ** 2]
        total = sum(c.evidence for c in support)
        wx = sum(c.evidence * c.x for c in support) / total
        wy = sum(c.evidence * c.y for c in support) / total
        co, si = math.cos(yaw), math.sin(yaw)
        dx, dy = wx - world_x, wy - world_y
        accepts = peak.hits
        self.cylinder.update(valid=peak.evidence >= CYL_VALID_CONF and accepts >= CYL_VALID_ACCEPTS,
                             world_x=wx, world_y=wy,
                             body_x=co * dx + si * dy,
                             body_y=-si * dx + co * dy,
                             confidence=peak.evidence, accepts=accepts)


def ray_box_range(x: float, y: float, yaw: float, az: float,
                  box: tuple[float, float, float, float]) -> float | None:
    """Return horizontal ray distance to (xmin, xmax, ymin, ymax)."""
    angle = yaw + az
    dx, dy = math.cos(angle), math.sin(angle)
    t0, t1 = -math.inf, math.inf
    for origin, direction, lo, hi in ((x, dx, box[0], box[1]),
                                       (y, dy, box[2], box[3])):
        if abs(direction) < 1e-12:
            if origin < lo or origin > hi:
                return None
            continue
        a, b = (lo - origin) / direction, (hi - origin) / direction
        if a > b:
            a, b = b, a
        t0, t1 = max(t0, a), min(t1, b)
    return t0 if t1 >= max(t0, 0.0) and t0 > 0.0 else None


def simulate(args: argparse.Namespace) -> tuple[list[dict[str, object]], FirmwareMirror]:
    rng = random.Random(args.seed)
    estimator = FirmwareMirror()
    azimuths = [math.radians(-args.fov_deg / 2 + (i + 0.5) * args.fov_deg / args.sectors)
                for i in range(args.sectors)]
    box = (args.box_x - args.box_depth / 2, args.box_x + args.box_depth / 2,
           args.box_y - args.box_width / 2, args.box_y + args.box_width / 2)
    rows: list[dict[str, object]] = []
    for frame in range(args.frames):
        t = frame * args.dt
        x = args.start_x + args.vx * t
        y = args.start_y + args.vy * t
        yaw = args.yaw + args.yaw_rate * t
        sectors: list[Sector] = []
        for az in azimuths:
            distance = ray_box_range(x, y, yaw, az, box)
            if distance is None or rng.random() < args.dropout:
                sectors.append(Sector(az, 0.0, 0.0, None))
                continue
            vel_eff = args.vx * math.sin(az) - args.vy * math.cos(az)
            angular_flow = args.yaw_rate + vel_eff / distance
            image_q = math.tan(az)
            image_flow = angular_flow * (1.0 + image_q**2)
            sectors.append(Sector(image_q, image_flow + rng.gauss(0.0, args.flow_noise),
                                  args.confidence, distance))
        result = estimator.update(sectors, args.vx, args.vy, args.yaw_rate,
                                  x, y, yaw)
        cyl = result["cylinder"]
        cluster = result["cluster"]
        rows.append({"frame": frame, "t": t, "x": x, "y": y,
                     "cluster_count": cluster["count"],
                     "cluster_fresh": cluster["fresh"],
                     "cyl_valid": cyl["valid"], "cyl_world_x": cyl["world_x"],
                     "cyl_world_y": cyl["world_y"], "cyl_body_x": cyl["body_x"],
                     "cyl_body_y": cyl["body_y"], "confidence": cyl["confidence"],
                     "accepts": cyl["accepts"]})
    return rows, estimator


def parse_args() -> argparse.Namespace:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--out", type=Path, default=None)
    ap.add_argument("--frames", type=int, default=60)
    ap.add_argument("--dt", type=float, default=1 / 30)
    ap.add_argument("--sectors", type=int, default=9, choices=range(2, 10))
    ap.add_argument("--fov-deg", type=float, default=70.0)
    ap.add_argument("--start-x", type=float, default=0.0)
    ap.add_argument("--start-y", type=float, default=-0.15)
    ap.add_argument("--vx", type=float, default=0.0)
    ap.add_argument("--vy", type=float, default=0.15)
    ap.add_argument("--yaw", type=float, default=0.0)
    ap.add_argument("--yaw-rate", type=float, default=0.0)
    ap.add_argument("--box-x", type=float, default=1.10,
                    help="box center; default front face is x=1.0 m")
    ap.add_argument("--box-y", type=float, default=0.0)
    ap.add_argument("--box-width", type=float, default=0.80)
    ap.add_argument("--box-depth", type=float, default=0.20)
    ap.add_argument("--confidence", type=float, default=0.8)
    ap.add_argument("--flow-noise", type=float, default=0.0)
    ap.add_argument("--dropout", type=float, default=0.0)
    ap.add_argument("--seed", type=int, default=7)
    return ap.parse_args()


def main() -> int:
    args = parse_args()
    rows, _ = simulate(args)
    valid = [r for r in rows if r["cyl_valid"]]
    summary = {"frames": len(rows), "valid_frames": len(valid)}
    if valid:
        last = valid[-1]
        summary.update(final_world_x=last["cyl_world_x"], final_world_y=last["cyl_world_y"],
                       final_body_x=last["cyl_body_x"], final_body_y=last["cyl_body_y"],
                       final_confidence=last["confidence"], final_accepts=last["accepts"])
    print(json.dumps(summary, indent=2))
    if args.out:
        args.out.mkdir(parents=True, exist_ok=True)
        with (args.out / "frames.csv").open("w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=rows[0].keys())
            writer.writeheader()
            writer.writerows(rows)
        (args.out / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    return 0 if valid else 2


if __name__ == "__main__":
    raise SystemExit(main())
