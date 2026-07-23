#!/usr/bin/env python3
"""Source-matched racing benchmark for 1-3 m/s monocular obstacle approaches."""

from __future__ import annotations

import argparse
import json
import math

from sim_flow_obstacle_sectors import FirmwareMirror, Sector
from sim_gap8_deployment_suite import render_gap8
from sim_gap8_flow_frontend import Gap8FlowFrontend
from sim_monocular_flow_suite import Box, Scene


TARGET = Box("target", 3.0, 3.25, -0.45, 0.45, 1)
SCENE = Scene("racing_box", (TARGET,), start_y=0.0, vy=0.0, truth=(3.0, 0.0))


def run(speed: float, camera_hz: int, balanced: bool, looming: bool,
        near_aggregation: bool = False) -> dict[str, object]:
    gap8 = Gap8FlowFrontend(balanced_features=balanced, near_aggregation=near_aggregation)
    stm32 = FirmwareMirror()
    dt_camera = 1/30
    stride = 30//camera_hz
    first = None
    final = None
    processed = 0
    max_frames = int((3.0-0.45)/speed/dt_camera)
    for frame in range(max_frames+1):
        t = frame*dt_camera
        x = speed*t
        if frame % stride:
            continue
        image = render_gap8(SCENE, x, 0.0, 0.0)
        sectors, stats = gap8.process(image, stride*dt_camera)
        if sectors is None:
            continue
        processed += 1
        if not looming:
            sectors = [Sector(s.azimuth, s.flow_x, s.confidence, flow_y=0.0) for s in sectors]
        result = stm32.update(sectors, speed, 0.0, 0.0,
                              x + speed*0.014583, 0.0, 0.0, new_sample=True)
        cyl = result["cylinder"]
        if cyl["valid"]:
            record = {"t": t+0.014583, "distance_remaining": 3.0-(x+speed*0.014583),
                      "estimate": [cyl["world_x"], cyl["world_y"]],
                      "error": math.hypot(float(cyl["world_x"])-3.0,
                                          float(cyl["world_y"]))}
            if first is None:
                first = record
            final = record
    return {"speed_m_s": speed, "camera_hz": camera_hz,
            "balanced_features": balanced, "looming": looming,
            "near_aggregation": near_aggregation,
            "packets": processed, "first_detection": first, "final_detection": final,
            "success": first is not None and float(first["distance_remaining"]) >= 0.75 and
                       float(first["error"]) <= 0.35}


def main() -> int:
    argparse.ArgumentParser(description=__doc__).parse_args()
    variants = ((5, False, False, False, "deployed_5hz"),
                (30, True, False, False, "30hz_parallax"),
                (30, True, True, False, "30hz_hybrid"),
                (30, True, True, True, "30hz_hybrid_near"))
    results = []
    for speed in (1.0, 2.0, 3.0):
        for hz, balanced, looming, near, name in variants:
            row = run(speed, hz, balanced, looming, near); row["variant"] = name
            results.append(row)
    flow_wire = 168*10/115200
    gate_wire = 44*10/115200
    summary = {"results": results,
               "uart": {"flow_packet_ms": flow_wire*1000,
                        "flow_30hz_utilization": flow_wire*30,
                        "flow_plus_gate_30hz_utilization": (flow_wire+gate_wire)*30}}
    print(json.dumps(summary, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
