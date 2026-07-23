#!/usr/bin/env python3
"""Python mirror of gate8-dory-package's GAP8 sparse-LK flow frontend."""

from __future__ import annotations

import math
from dataclasses import dataclass

from sim_flow_obstacle_sectors import Sector


W = H = 160
SECTORS = 9
HALF_W = HALF_H = 80
MAX_FEATURES = 48
FEATURE_STEP = 6
FEATURE_BORDER = 10
MIN_FEATURE_DIST = 8
ST_SCORE_THRESH = 1200.0
LK_WIN_R = 3
LK_ITERS = 3
LK_ERR_THRESH = 18.0
MIN_SAMPLES = 2
FX = 89.15584
CX = 81.10381
FY = FX
CY = 80.0
MAX_RAD_S = 20.0
SMOOTH_ALPHA = 0.35
HOLD_UPDATES = 3
HOLD_CONF_DECAY = 0.60


@dataclass
class Feature:
    x: float
    y: float
    score: float


def half_pyramid(src: list[list[int]]) -> list[list[int]]:
    return [[(src[2*y][2*x] + src[2*y][2*x+1] +
              src[2*y+1][2*x] + src[2*y+1][2*x+1] + 2) // 4
             for x in range(HALF_W)] for y in range(HALF_H)]


def shi_tomasi(img: list[list[int]], x: int, y: int) -> float:
    sxx = syy = sxy = 0
    for yy in range(-2, 3):
        for xx in range(-2, 3):
            gx = img[y+yy][x+xx+1] - img[y+yy][x+xx-1]
            gy = img[y+yy+1][x+xx] - img[y+yy-1][x+xx]
            sxx += gx * gx
            syy += gy * gy
            sxy += gx * gy
    return 0.5 * (sxx + syy - math.sqrt((sxx-syy)**2 + 4*sxy*sxy))


def select_features(img: list[list[int]]) -> list[Feature]:
    candidates: list[Feature] = []
    for y in range(FEATURE_BORDER, H - FEATURE_BORDER, FEATURE_STEP):
        for x in range(FEATURE_BORDER, W - FEATURE_BORDER, FEATURE_STEP):
            score = shi_tomasi(img, x, y)
            if score >= ST_SCORE_THRESH:
                candidates.append(Feature(float(x), float(y), score))
    candidates.sort(key=lambda f: f.score, reverse=True)
    selected: list[Feature] = []
    for feature in candidates:
        if all((feature.x-f.x)**2 + (feature.y-f.y)**2 >= MIN_FEATURE_DIST**2
               for f in selected):
            selected.append(feature)
            if len(selected) == MAX_FEATURES:
                break
    return selected


def select_features_balanced(img: list[list[int]]) -> list[Feature]:
    """Allocate five strong, separated features to every transmitted sector."""
    buckets: list[list[Feature]] = [[] for _ in range(SECTORS)]
    for y in range(FEATURE_BORDER, H - FEATURE_BORDER, FEATURE_STEP):
        for x in range(FEATURE_BORDER, W - FEATURE_BORDER, FEATURE_STEP):
            score = shi_tomasi(img, x, y)
            if score >= ST_SCORE_THRESH:
                buckets[min(SECTORS-1, x*SECTORS//W)].append(Feature(float(x), float(y), score))
    selected: list[Feature] = []
    for bucket in buckets:
        bucket.sort(key=lambda f: f.score, reverse=True)
        local: list[Feature] = []
        for feature in bucket:
            if all((feature.x-f.x)**2 + (feature.y-f.y)**2 >= MIN_FEATURE_DIST**2
                   for f in local):
                local.append(feature)
                if len(local) == 5:
                    break
        selected.extend(local)
    return selected[:MAX_FEATURES]


def bilinear(img: list[list[int]], x: float, y: float) -> float:
    xi, yi = int(x), int(y)
    if xi < 0 or yi < 0 or yi >= len(img)-1 or xi >= len(img[0])-1:
        return 0.0
    ax, ay = x-xi, y-yi
    return ((1-ax)*(1-ay)*img[yi][xi] + ax*(1-ay)*img[yi][xi+1] +
            (1-ax)*ay*img[yi+1][xi] + ax*ay*img[yi+1][xi+1])


def track_level(prev: list[list[int]], cur: list[list[int]], px: float, py: float,
                cx: float, cy: float) -> tuple[bool, float, float, float]:
    w, h = len(prev[0]), len(prev)
    if px < 4 or py < 4 or px >= w-5 or py >= h-5:
        return False, cx, cy, 0.0
    err = 0.0
    for _ in range(LK_ITERS):
        if cx < 4 or cy < 4 or cx >= w-5 or cy >= h-5:
            return False, cx, cy, err
        gxx = gyy = gxy = bx = by = err = 0.0
        samples = 0
        for yy in range(-LK_WIN_R, LK_WIN_R+1):
            for xx in range(-LK_WIN_R, LK_WIN_R+1):
                qx, qy, rx, ry = px+xx, py+yy, cx+xx, cy+yy
                i0, i1 = bilinear(prev, qx, qy), bilinear(cur, rx, ry)
                ix = 0.5 * (bilinear(prev, qx+1, qy) - bilinear(prev, qx-1, qy))
                iy = 0.5 * (bilinear(prev, qx, qy+1) - bilinear(prev, qx, qy-1))
                it = i0-i1
                gxx += ix*ix; gyy += iy*iy; gxy += ix*iy
                bx += ix*it; by += iy*it; err += abs(it); samples += 1
        det = gxx*gyy-gxy*gxy
        if det < 1e-3:
            return False, cx, cy, err/samples
        du, dv = (gyy*bx-gxy*by)/det, (gxx*by-gxy*bx)/det
        cx, cy, err = cx+du, cy+dv, err/samples
        if du*du+dv*dv < 0.0025:
            break
    return err <= LK_ERR_THRESH, cx, cy, err


def track_pyramid(prev: list[list[int]], cur: list[list[int]],
                  prev_half: list[list[int]], cur_half: list[list[int]],
                  x: float, y: float) -> tuple[bool, float, float]:
    ok, hx, hy, _ = track_level(prev_half, cur_half, x*.5, y*.5, x*.5, y*.5)
    if not ok:
        return False, x, y
    nx, ny = x + 2*(hx-x*.5), y + 2*(hy-y*.5)
    ok, nx, ny, _ = track_level(prev, cur, x, y, nx, ny)
    return ok, nx, ny


class Gap8FlowFrontend:
    def __init__(self, balanced_features: bool = False,
                 near_aggregation: bool = False) -> None:
        self.balanced_features = balanced_features
        self.near_aggregation = near_aggregation
        self.prev: list[list[int]] | None = None
        self.prev_half: list[list[int]] | None = None
        self.smooth_flow = [0.0] * SECTORS
        self.smooth_radial = [0.0] * SECTORS
        self.smooth_conf = [0.0] * SECTORS
        self.hold = [0] * SECTORS

    def process(self, cur: list[list[int]], dt: float) -> tuple[list[Sector] | None, dict[str, int]]:
        cur_half = half_pyramid(cur)
        if self.prev is None:
            self.prev, self.prev_half = cur, cur_half
            return None, {"features": 0, "tracks": 0}
        features = (select_features_balanced(self.prev) if self.balanced_features
                    else select_features(self.prev))
        sums, counts = [0.0]*SECTORS, [0]*SECTORS
        radial_sums, radial_counts = [0.0]*SECTORS, [0]*SECTORS
        flow_samples: list[list[float]] = [[] for _ in range(SECTORS)]
        radial_samples: list[list[float]] = [[] for _ in range(SECTORS)]
        tracked = 0
        for f in features:
            ok, nx, ny = track_pyramid(self.prev, cur, self.prev_half, cur_half, f.x, f.y)
            if not ok:
                continue
            dx = nx-f.x
            if abs(dx) < 0.05:
                continue
            sector = min(SECTORS-1, int(f.x)*SECTORS//W)
            sums[sector] += dx
            counts[sector] += 1
            flow_samples[sector].append(dx/(FX*dt))
            q, p = (f.x-CX)/FX, (f.y-CY)/FY
            qdot, pdot = dx/(FX*dt), (ny-f.y)/(FY*dt)
            radius2 = q*q+p*p
            if radius2 > 0.01:
                radial_sums[sector] += (q*qdot+p*pdot)/radius2
                radial_counts[sector] += 1
                radial_samples[sector].append((q*qdot+p*pdot)/radius2)
            tracked += 1
        output: list[Sector] = []
        for i in range(SECTORS):
            valid = counts[i] >= MIN_SAMPLES
            raw_flow = max(-MAX_RAD_S, min(MAX_RAD_S, sums[i]/counts[i]/(FX*dt))) if valid else 0.0
            raw_radial = (radial_sums[i]/radial_counts[i]
                          if radial_counts[i] >= MIN_SAMPLES else 0.0)
            if self.near_aggregation and valid:
                if len(radial_samples[i]) >= MIN_SAMPLES:
                    near_radial = sorted(radial_samples[i], key=abs, reverse=True)[:max(2, (len(radial_samples[i])+1)//2)]
                    raw_radial = sorted(near_radial)[len(near_radial)//2]
            raw_conf = min(1.0, counts[i]/32.0) if valid else 0.0
            if valid:
                if self.smooth_conf[i] <= 0:
                    self.smooth_flow[i], self.smooth_conf[i] = raw_flow, raw_conf
                    self.smooth_radial[i] = raw_radial
                else:
                    self.smooth_flow[i] += SMOOTH_ALPHA*(raw_flow-self.smooth_flow[i])
                    self.smooth_radial[i] += SMOOTH_ALPHA*(raw_radial-self.smooth_radial[i])
                    self.smooth_conf[i] += SMOOTH_ALPHA*(raw_conf-self.smooth_conf[i])
                self.hold[i] = HOLD_UPDATES
            elif self.hold[i] > 0:
                self.hold[i] -= 1
                self.smooth_conf[i] *= HOLD_CONF_DECAY
            else:
                self.smooth_flow[i] = self.smooth_conf[i] = 0.0
                self.smooth_radial[i] = 0.0
            center_x = (i+0.5)*W/SECTORS
            output.append(Sector((center_x-CX)/FX, self.smooth_flow[i], self.smooth_conf[i],
                                 flow_y=self.smooth_radial[i]))
        self.prev, self.prev_half = cur, cur_half
        return output, {"features": len(features), "tracks": tracked}
