#!/usr/bin/env python3
"""Python mirror of gate8-dory-package's GAP8 sparse-LK flow frontend."""

from __future__ import annotations

import math
import struct
from dataclasses import dataclass

from sim_flow_obstacle_sectors import Sector


W = H = 160
SECTORS = 9
HALF_W = HALF_H = 80
MAX_FEATURES = 27
FEATURES_PER_SECTOR = MAX_FEATURES // SECTORS
FEATURE_STEP = 6
FEATURE_BORDER = 10
MIN_FEATURE_DIST = 8
ST_SCORE_THRESH_PROXY = 600
LK_WIN_R = 2
LK_ITERS = 1
LK_ERR_THRESH = 18.0
MIN_SAMPLES = 2
FX = 89.15584
CX = 81.10381
FY = 89.46082
CY = 73.34730
MAX_RAD_S = 20.0
SMOOTH_ALPHA = 0.35
HOLD_UPDATES = 3
HOLD_CONF_DECAY = 0.60
K1, K2, P1, P2, K3 = (
    -0.01764488, 0.09941325, 0.00544322, -0.00604001, -0.19001899)


def f32(value: float) -> float:
    """Round at production float storage/operation boundaries."""
    return struct.unpack("<f", struct.pack("<f", value))[0]


def undistort_normalized(u: float, v: float) -> tuple[float, float]:
    xd, yd = f32((u-CX)/FX), f32((v-CY)/FY)
    x, y = xd, yd
    for _ in range(4):
        r2 = f32(x*x+y*y)
        radial = f32(1+r2*(K1+r2*(K2+r2*K3)))
        dx = f32(2*P1*x*y+P2*(r2+2*x*x))
        dy = f32(P1*(r2+2*y*y)+2*P2*x*y)
        x, y = f32((xd-dx)/radial), f32((yd-dy)/radial)
    return x, y


@dataclass
class Feature:
    x: float
    y: float
    score: float


def half_pyramid(src: list[list[int]]) -> list[list[int]]:
    return [[(src[2*y][2*x] + src[2*y][2*x+1] +
              src[2*y+1][2*x] + src[2*y+1][2*x+1] + 2) // 4
             for x in range(HALF_W)] for y in range(HALF_H)]


def corner_score_proxy(img: list[list[int]], x: int, y: int) -> int:
    sxx = syy = sxy = 0
    for yy in range(-2, 3):
        for xx in range(-2, 3):
            gx = img[y+yy][x+xx+1] - img[y+yy][x+xx-1]
            gy = img[y+yy+1][x+xx] - img[y+yy-1][x+xx]
            sxx += gx * gx
            syy += gy * gy
            sxy += gx * gy
    trace = sxx + syy
    det = sxx*syy-sxy*sxy
    if trace == 0 or det <= 0:
        return 0
    return det >> (trace.bit_length()-1)


def select_features_balanced(img: list[list[int]]) -> list[Feature]:
    """Mirror the deployed per-sector proxy scoring and suppression order."""
    scores: dict[tuple[int, int], int] = {}
    for y in range(FEATURE_BORDER, H - FEATURE_BORDER, FEATURE_STEP):
        for x in range(FEATURE_BORDER, W - FEATURE_BORDER, FEATURE_STEP):
            scores[(x, y)] = corner_score_proxy(img, x, y)
    selected: list[Feature] = []
    for sector in range(SECTORS):
        x0, x1 = sector*W//SECTORS, (sector+1)*W//SECTORS
        for _ in range(FEATURES_PER_SECTOR):
            best_score = ST_SCORE_THRESH_PROXY
            bx = by = -1
            # Production scans y-major/x-minor and replaces only on strictly
            # greater scores, so the first grid point wins ties.
            for (x, y), score in scores.items():
                if x0 <= x < x1 and score > best_score:
                    best_score, bx, by = score, x, y
            if bx < 0:
                break
            selected.append(Feature(float(bx), float(by), float(best_score)))
            for x, y in scores:
                if (x-bx)**2+(y-by)**2 < MIN_FEATURE_DIST**2:
                    scores[(x, y)] = 0
    return selected


def bilinear(img: list[list[int]], x: float, y: float) -> float:
    xi, yi = int(x), int(y)
    if xi < 0 or yi < 0 or yi >= len(img)-1 or xi >= len(img[0])-1:
        return 0.0
    ax, ay = f32(x-xi), f32(y-yi)
    return f32((1-ax)*(1-ay)*img[yi][xi] + ax*(1-ay)*img[yi][xi+1] +
               (1-ax)*ay*img[yi+1][xi] + ax*ay*img[yi+1][xi+1])


def track_level(prev: list[list[int]], cur: list[list[int]], px: float, py: float,
                cx: float, cy: float) -> tuple[bool, float, float, float]:
    w, h = len(prev[0]), len(prev)
    if (px < LK_WIN_R+1 or py < LK_WIN_R+1 or
            px >= w-LK_WIN_R-2 or py >= h-LK_WIN_R-2):
        return False, cx, cy, 0.0
    err = 0.0
    for _ in range(LK_ITERS):
        if (cx < LK_WIN_R+1 or cy < LK_WIN_R+1 or
                cx >= w-LK_WIN_R-2 or cy >= h-LK_WIN_R-2):
            return False, cx, cy, err
        gxx = gyy = gxy = bx = by = err = 0.0
        samples = 0
        for yy in range(-LK_WIN_R, LK_WIN_R+1):
            for xx in range(-LK_WIN_R, LK_WIN_R+1):
                qx, qy, rx, ry = px+xx, py+yy, cx+xx, cy+yy
                pxi, pyi = int(px), int(py)
                i0 = float(prev[pyi+yy][pxi+xx])
                i1 = bilinear(cur, rx, ry)
                ix = 0.5 * (prev[pyi+yy][pxi+xx+1] -
                            prev[pyi+yy][pxi+xx-1])
                iy = 0.5 * (prev[pyi+yy+1][pxi+xx] -
                            prev[pyi+yy-1][pxi+xx])
                it = f32(i0-i1)
                gxx = f32(gxx+f32(ix*ix))
                gyy = f32(gyy+f32(iy*iy))
                gxy = f32(gxy+f32(ix*iy))
                bx = f32(bx+f32(ix*it))
                by = f32(by+f32(iy*it))
                err = f32(err+abs(it))
                samples += 1
        det = f32(f32(gxx*gyy)-f32(gxy*gxy))
        if det < 1e-3:
            return False, cx, cy, err/samples
        du = f32(f32(f32(gyy*bx)-f32(gxy*by))/det)
        dv = f32(f32(f32(gxx*by)-f32(gxy*bx))/det)
        cx, cy, err = f32(cx+du), f32(cy+dv), f32(err/samples)
        if du*du+dv*dv < 0.0025:
            break
    return err <= LK_ERR_THRESH, cx, cy, err


def track_pyramid(prev: list[list[int]], cur: list[list[int]],
                  prev_half: list[list[int]], cur_half: list[list[int]],
                  x: float, y: float) -> tuple[bool, float, float]:
    ok, hx, hy, _ = track_level(prev_half, cur_half, x*.5, y*.5, x*.5, y*.5)
    if not ok:
        return False, x, y
    return True, f32(x + 2*(hx-x*.5)), f32(y + 2*(hy-y*.5))


def robust_near_sample(values: list[float]) -> float:
    near = sorted(values, key=abs, reverse=True)[:max(MIN_SAMPLES, (len(values)+1)//2)]
    near.sort()
    return near[len(near)//2]


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
        dt = f32(dt)
        cur_half = half_pyramid(cur)
        if self.prev is None:
            self.prev, self.prev_half = cur, cur_half
            return None, {"features": 0, "tracks": 0,
                          "feature_x_sum": 0, "feature_y_sum": 0,
                          "feature_score_sum": 0,
                          "track_dx_sum": 0.0, "track_dy_sum": 0.0,
                          "sector_counts": [0]*SECTORS}
        # Current production firmware always uses balanced per-sector selection.
        features = select_features_balanced(self.prev)
        sums, counts = [0.0]*SECTORS, [0]*SECTORS
        radial_sums, radial_counts = [0.0]*SECTORS, [0]*SECTORS
        flow_samples: list[list[float]] = [[] for _ in range(SECTORS)]
        radial_samples: list[list[float]] = [[] for _ in range(SECTORS)]
        tracked = 0
        track_dx_sum = 0.0
        track_dy_sum = 0.0
        for f in features:
            ok, nx, ny = track_pyramid(self.prev, cur, self.prev_half, cur_half, f.x, f.y)
            if not ok:
                continue
            dx = f32(nx-f.x)
            dy = f32(ny-f.y)
            if f32(f32(dx*dx)+f32(dy*dy)) < f32(0.0025):
                continue
            sector = min(SECTORS-1, int(f.x)*SECTORS//W)
            if counts[sector] < FEATURES_PER_SECTOR:
                sums[sector] += dx
                counts[sector] += 1
                flow_samples[sector].append(dx)
            q, p = undistort_normalized(f.x, f.y)
            q1, p1 = undistort_normalized(nx, ny)
            qdot = f32(f32(q1-q)/dt)
            pdot = f32(f32(p1-p)/dt)
            radius2 = f32(f32(q*q)+f32(p*p))
            if radius2 > 0.01 and radial_counts[sector] < FEATURES_PER_SECTOR:
                radial = f32(f32(f32(q*qdot)+f32(p*pdot))/radius2)
                radial_sums[sector] += radial
                radial_counts[sector] += 1
                radial_samples[sector].append(radial)
            tracked += 1
            track_dx_sum += dx
            track_dy_sum += dy
        output: list[Sector] = []
        for i in range(SECTORS):
            valid = counts[i] >= MIN_SAMPLES
            raw_flow = (max(-MAX_RAD_S, min(
                MAX_RAD_S,
                f32(robust_near_sample(flow_samples[i]) / dt),
            )) if valid else 0.0)
            raw_radial = (radial_sums[i]/radial_counts[i]
                          if radial_counts[i] >= MIN_SAMPLES else 0.0)
            if valid and len(radial_samples[i]) >= MIN_SAMPLES:
                raw_radial = robust_near_sample(radial_samples[i])
            raw_conf = min(1.0, counts[i]/32.0) if valid else 0.0
            if valid:
                if self.smooth_conf[i] <= 0:
                    self.smooth_flow[i], self.smooth_conf[i] = f32(raw_flow), f32(raw_conf)
                    self.smooth_radial[i] = raw_radial
                else:
                    self.smooth_flow[i] = f32(
                        f32(SMOOTH_ALPHA)*raw_flow +
                        f32(1.0-SMOOTH_ALPHA)*self.smooth_flow[i])
                    self.smooth_radial[i] = f32(
                        f32(SMOOTH_ALPHA)*raw_radial +
                        f32(1.0-SMOOTH_ALPHA)*self.smooth_radial[i])
                    self.smooth_conf[i] = f32(
                        f32(SMOOTH_ALPHA)*raw_conf +
                        f32(1.0-SMOOTH_ALPHA)*self.smooth_conf[i])
                self.hold[i] = HOLD_UPDATES
            elif self.hold[i] > 0:
                self.hold[i] -= 1
                self.smooth_conf[i] = f32(
                    self.smooth_conf[i]*f32(HOLD_CONF_DECAY))
            else:
                self.smooth_flow[i] = self.smooth_conf[i] = 0.0
                self.smooth_radial[i] = 0.0
            center_x = (i+0.5)*W/SECTORS
            center_q, _ = undistort_normalized(center_x, CY)
            output.append(Sector(center_q, self.smooth_flow[i], self.smooth_conf[i],
                                 flow_y=self.smooth_radial[i]))
        self.prev, self.prev_half = cur, cur_half
        return output, {
            "features": len(features),
            "tracks": tracked,
            "feature_x_sum": int(sum(f.x for f in features)),
            "feature_y_sum": int(sum(f.y for f in features)),
            "feature_score_sum": int(sum(f.score for f in features)),
            "track_dx_sum": track_dx_sum,
            "track_dy_sum": track_dy_sum,
            "sector_counts": counts,
        }
