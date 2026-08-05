#!/usr/bin/env python3
"""Offline mirror of flowdeck_obstacle_link.c's obstacle estimator.

Synthetic horizontal sector rays intersect a world-aligned box.  Their ideal
angular flow is then fed through the same inverse-depth, bounded spatial-group,
N-of-M persistence, evidence-map, and cylinder-extraction equations as firmware.
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
CANDIDATE_MIN_SECTORS = 1
CANDIDATE_GROUP_RADIUS = 0.60
CANDIDATE_MAX_RANGE_DISPERSION = 0.55
CANDIDATE_SPATIAL_GATE = 0.55
PERSIST_M = 3
PERSIST_N = 2
MIN_AGGREGATE_DISPLACEMENT = 0.004
MAX_YAW_EXPLAINED_RATIO = 0.80
MAX_INV_DEPTH_DISAGREEMENT = 0.75
MAP_CELLS = 16
MAP_DECAY = 0.999
MAP_MIN_EVIDENCE = 0.03
MAP_VOTE = 0.18
MAP_MERGE_RADIUS = 0.30
MAP_EXTRACT_RADIUS = 0.30
MAP_VALID_EVIDENCE = 0.10
CYL_VALID_CONF = 0.10
CYL_VALID_ACCEPTS = 1
CYL_FAR_RANGE = 0.75
CYL_FAR_VALID_ACCEPTS = 4
CYL_GATE_BASE = 0.25
CYL_GATE_RANGE_FRAC = 0.20
CYL_GATE_MAX = 0.50
CYL_SWITCH_MARGIN = 0.50


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
        self.history: list[tuple[float, float] | None] = []
        self.cylinder: dict[str, float | int | bool] = {
            "valid": False, "world_x": 0.0, "world_y": 0.0,
            "body_x": 0.0, "body_y": 0.0, "confidence": 0.0,
            "accepts": 0,
        }

    @property
    def obs_hits(self) -> int:
        """Compatibility/debug view: candidate observations in the N-of-M window."""
        return sum(item is not None for item in self.history)

    def update(self, sectors: list[Sector], vx: float, vy: float,
               yaw_rate: float, world_x: float, world_y: float,
               yaw: float, new_sample: bool = True,
               dt: float = 1.0 / 15.0) -> dict[str, object]:
        if not new_sample:
            self._extract(world_x, world_y, yaw)
            return {"sectors": [], "cluster": {"fresh": False, "count": 0,
                    "valid": False, "reject": "duplicate"},
                    "cylinder": dict(self.cylinder)}
        self.history = self.history[-(PERSIST_M - 1):]
        c, s = math.cos(yaw), math.sin(yaw)
        derived: list[dict[str, float | bool]] = []
        candidate: list[bool] = []
        aggregate_displacement = 0.0
        yaw_displacement = 0.0
        for i in range(SECT_MAX):
            if i >= len(sectors) or sectors[i].confidence < MIN_CONFIDENCE:
                derived.append({"valid": False})
                candidate.append(False)
                continue
            sec = sectors[i]
            # Deployed GAP8 fields are normalized image coordinate/rate.
            az = math.atan(sec.azimuth)
            measured_angular_flow = sec.flow_x / (1.0 + sec.azimuth**2)
            aggregate_displacement += sec.confidence * abs(measured_angular_flow) * dt
            yaw_displacement += sec.confidence * abs(yaw_rate) * dt
            residual = measured_angular_flow - yaw_rate
            vel_eff = vx * math.sin(az) - vy * math.cos(az)
            parallax_inv = residual / vel_eff if abs(vel_eff) >= MIN_TRANSLATION else None
            looming_inv = (sec.flow_y / vx
                           if abs(vx) >= 0.15 and abs(sec.flow_y) >= 1e-3 else None)
            disagreement = (abs(parallax_inv - looming_inv)
                            if parallax_inv is not None and looming_inv is not None else 0.0)
            use_looming = (abs(vel_eff) < MIN_TRANSLATION and abs(vx) >= 0.15 and
                           abs(yaw_rate) < 0.20 and abs(sec.flow_y) >= 1e-3)
            if use_looming:
                residual = sec.flow_y
                vel_eff = vx
            if disagreement > MAX_INV_DEPTH_DISAGREEMENT:
                derived.append({"valid": False, "residual": residual,
                                "vel_eff": vel_eff, "reject": "depth_disagreement",
                                "depth_disagreement": disagreement})
                candidate.append(False)
                continue
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

        reject = ""
        yaw_ratio = yaw_displacement / max(aggregate_displacement, 1e-9)
        if aggregate_displacement < MIN_AGGREGATE_DISPLACEMENT:
            candidate = [False] * SECT_MAX
            reject = "low_displacement"
        elif yaw_ratio > MAX_YAW_EXPLAINED_RATIO:
            candidate = [False] * SECT_MAX
            reject = "yaw_explained"

        # Fixed-size union-find forms disjoint spatial/range components. Unlike
        # overlapping seed balls, each sector belongs to one unambiguous group.
        # Keep a singleton component so an FOV-edge foreground observation can
        # reach N-of-M persistence; it cannot vote from only one frame.
        parent = list(range(SECT_MAX))

        def root(index: int) -> int:
            while parent[index] != index:
                parent[index] = parent[parent[index]]
                index = parent[index]
            return index

        for i in range(SECT_MAX):
            if not candidate[i]:
                continue
            for j in range(i + 1, SECT_MAX):
                if not candidate[j]:
                    continue
                spatial = math.hypot(
                    float(derived[i]["body_x"]) - float(derived[j]["body_x"]),
                    float(derived[i]["body_y"]) - float(derived[j]["body_y"]))
                range_gap = abs(float(derived[i]["range"]) -
                                float(derived[j]["range"]))
                if (spatial <= CANDIDATE_GROUP_RADIUS and
                        range_gap <= CANDIDATE_MAX_RANGE_DISPERSION):
                    ri, rj = root(i), root(j)
                    if ri != rj:
                        parent[rj] = ri

        groups: list[tuple[float, list[int], float, float, float]] = []
        for component in range(SECT_MAX):
            if not candidate[component] or root(component) != component:
                continue
            members = [i for i in range(SECT_MAX)
                       if candidate[i] and root(i) == component]
            ranges = [float(derived[i]["range"]) for i in members]
            if len(members) < CANDIDATE_MIN_SECTORS or max(ranges) - min(ranges) > CANDIDATE_MAX_RANGE_DISPERSION:
                continue
            weights = [max(sectors[i].confidence, 1e-3) for i in members]
            total = sum(weights)
            bx = sum(w * float(derived[i]["body_x"]) for w, i in zip(weights, members)) / total
            by = sum(w * float(derived[i]["body_y"]) for w, i in zip(weights, members)) / total
            mean_range = sum(ranges) / len(ranges)
            temporal = 1.0
            prior = [p for p in self.history if p is not None]
            if prior:
                temporal += max(0.0, 1.0 - min(math.hypot(bx-p[0], by-p[1])
                                               for p in prior) / CANDIDATE_SPATIAL_GATE)
            score = (0.25 * len(members) + 2.0 * total +
                     2.0 / max(mean_range, 0.2) + temporal)
            groups.append((score, members, bx, by, mean_range))
        best = max(groups, default=None, key=lambda group: group[0])
        fresh = best is not None
        best_count = len(best[1]) if best else 0
        obs: dict[str, float | int | bool | str] = {
            "fresh": fresh, "start": min(best[1]) if best else 0,
            "count": best_count, "valid": False, "reject": reject or ("no_group" if not best else ""),
            "aggregate_displacement": aggregate_displacement, "yaw_ratio": yaw_ratio}
        if fresh:
            assert best is not None
            raw_x, raw_y = best[2], best[3]
            consistent = [p for p in self.history if p is not None and
                          math.hypot(raw_x-p[0], raw_y-p[1]) <= CANDIDATE_SPATIAL_GATE]
            persistent = 1 + len(consistent) >= PERSIST_N
            owx = world_x + c * raw_x - s * raw_y
            owy = world_y + s * raw_x + c * raw_y
            obs.update(valid=persistent, body_x=raw_x, body_y=raw_y,
                       world_x=owx, world_y=owy, range=best[4],
                       persistence=1 + len(consistent))
            self.history.append((raw_x, raw_y))
        else:
            self.history.append(None)
        self.history = self.history[-PERSIST_M:]

        self._decay(MAP_DECAY)
        if fresh and bool(obs["valid"]):
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
        eligible = [c for c in active if c.evidence >= MAP_VALID_EVIDENCE]
        if not eligible:
            self.cylinder.update(valid=False, confidence=peak.evidence if peak else 0.0)
            return
        # Each eligible cell already represents N-of-M temporal agreement.
        # Prefer the nearest validated hazard over a stronger distant wall.
        selected = min(eligible, key=lambda c:
                       ((c.x - world_x) ** 2 + (c.y - world_y) ** 2,
                        -c.evidence))
        if float(self.cylinder["confidence"]) >= CYL_VALID_CONF:
            old_x, old_y = (float(self.cylinder["world_x"]),
                            float(self.cylinder["world_y"]))
            tracked = min(eligible, key=lambda c:
                          (c.x - old_x) ** 2 + (c.y - old_y) ** 2)
            old_range = math.hypot(old_x - world_x, old_y - world_y)
            track_gate = min(CYL_GATE_MAX,
                             CYL_GATE_BASE + CYL_GATE_RANGE_FRAC * old_range)
            innovation = math.hypot(tracked.x - old_x, tracked.y - old_y)
            tracked_range = math.hypot(tracked.x - world_x,
                                       tracked.y - world_y)
            selected_range = math.hypot(selected.x - world_x,
                                        selected.y - world_y)
            if (innovation <= track_gate and
                    selected_range + CYL_SWITCH_MARGIN >= tracked_range):
                selected = tracked
        support = [c for c in active if (c.x - selected.x) ** 2 + (c.y - selected.y) ** 2 <= MAP_EXTRACT_RADIUS ** 2]
        total = sum(c.evidence for c in support)
        wx = sum(c.evidence * c.x for c in support) / total
        wy = sum(c.evidence * c.y for c in support) / total
        co, si = math.cos(yaw), math.sin(yaw)
        dx, dy = wx - world_x, wy - world_y
        accepts = selected.hits
        body_x = co * dx + si * dy
        body_y = -si * dx + co * dy
        required_accepts = (CYL_FAR_VALID_ACCEPTS
                            if math.hypot(body_x, body_y) > CYL_FAR_RANGE
                            else CYL_VALID_ACCEPTS)
        self.cylinder.update(valid=selected.evidence >= CYL_VALID_CONF and accepts >= required_accepts,
                             world_x=wx, world_y=wy,
                             body_x=body_x, body_y=body_y,
                             confidence=selected.evidence, accepts=accepts)


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
