#!/usr/bin/env python3
"""Monte Carlo for 30 Hz parallax+looming racing obstacle detection."""

from __future__ import annotations

import argparse
import json
import math
import random

from sim_flow_obstacle_sectors import FirmwareMirror, Sector, ray_box_range
from sim_gap8_flow_frontend import CX, FX, SECTORS, W, SMOOTH_ALPHA


Q = [((i+.5)*W/SECTORS-CX)/FX for i in range(SECTORS)]


def pct(v: list[float], p: float) -> float | None:
    if not v: return None
    v = sorted(v); return v[round((len(v)-1)*p)]


def one(rng: random.Random, obstacle: bool) -> dict[str, float | bool]:
    speed = rng.uniform(1.0, 3.0)
    front = rng.uniform(2.5, 5.0)
    width = rng.uniform(.18, 1.2)
    cy = rng.uniform(-.35, .35)
    box = (front, front+rng.uniform(.15,.35), cy-width/2, cy+width/2)
    vel_meas = speed*(1+rng.gauss(0,.015))+rng.gauss(0,.015)
    pixel_noise = rng.uniform(.04,.16)
    loss = rng.uniform(0,.04)
    smooth_x = [0.0]*SECTORS; smooth_r = [0.0]*SECTORS; smooth_c = [0.0]*SECTORS
    est = FirmwareMirror(); first = final = None
    frames = int((front-.45)/speed*30)
    for frame in range(1,frames+1):
        t=frame/30; x=speed*t
        packet=[]
        for i,q in enumerate(Q):
            az=math.atan(q)
            distance=ray_box_range(x,0,0,az,box) if obstacle else None
            if distance is None:
                distance=((8.0-x)/max(.15,math.cos(az))) if obstacle else 20.0
            angular=speed*math.sin(az)/distance
            qdot=angular*(1+q*q)+rng.gauss(0,pixel_noise/(FX/30*math.sqrt(4)))
            radial=speed/distance+rng.gauss(0,pixel_noise/(FX/30*math.sqrt(4)))
            conf=.125 if rng.random()>.04 else 0.0
            if conf:
                if smooth_c[i] <= 0: smooth_x[i],smooth_r[i],smooth_c[i]=qdot,radial,conf
                else:
                    smooth_x[i]+=SMOOTH_ALPHA*(qdot-smooth_x[i])
                    smooth_r[i]+=SMOOTH_ALPHA*(radial-smooth_r[i])
                    smooth_c[i]+=SMOOTH_ALPHA*(conf-smooth_c[i])
            packet.append(Sector(q,smooth_x[i],smooth_c[i],flow_y=smooth_r[i]))
        if rng.random()<loss: continue
        result=est.update(packet,vel_meas,0,0,x+speed*.014583,0,0,True)
        c=result["cylinder"]
        if c["valid"]:
            err=math.hypot(float(c["world_x"])-front,float(c["world_y"])-cy) if obstacle else math.nan
            rec={"remaining":front-(x+speed*.014583),"error":err}
            if first is None:first=rec
            final=rec
    accurate=bool(first and first["remaining"]>=.75 and first["error"]<=.50)
    return {"obstacle":obstacle,"detected":first is not None,"accurate":accurate,
            "speed":speed,"width":width,"range":front,
            "first_remaining":math.nan if first is None else first["remaining"],
            "first_error":math.nan if first is None else first["error"]}


def main()->int:
    ap=argparse.ArgumentParser(description=__doc__);ap.add_argument("--trials",type=int,default=10000);ap.add_argument("--seed",type=int,default=53)
    a=ap.parse_args();r=random.Random(a.seed);rows=[one(r,i%5!=0) for i in range(a.trials)]
    pos=[x for x in rows if x["obstacle"]];neg=[x for x in rows if not x["obstacle"]];det=[x for x in pos if x["detected"]]
    errors=[float(x["first_error"]) for x in det];remaining=[float(x["first_remaining"]) for x in det]
    out={"trials":a.trials,"detection_rate":len(det)/len(pos),"racing_success_rate":sum(bool(x["accurate"]) for x in pos)/len(pos),
         "false_positive_rate":sum(bool(x["detected"]) for x in neg)/len(neg),
         "first_error_m":{"median":pct(errors,.5),"p90":pct(errors,.9),"p95":pct(errors,.95)},
         "first_remaining_m":{"median":pct(remaining,.5),"p10":pct(remaining,.1)},"by_speed":{}}
    for name,lo,hi in (("1-1.67",1,1.67),("1.67-2.33",1.67,2.33),("2.33-3",2.33,3.01)):
        b=[x for x in pos if lo<=float(x["speed"])<hi];out["by_speed"][name]={"detected":sum(bool(x["detected"]) for x in b)/len(b),"success":sum(bool(x["accurate"]) for x in b)/len(b)}
    print(json.dumps(out,indent=2));return 0


if __name__=="__main__":raise SystemExit(main())
