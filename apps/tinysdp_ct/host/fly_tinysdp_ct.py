#!/usr/bin/env python3
"""Fly the TinySDP-CT on-board planner (apps/tinysdp_ct) and log everything §4.7 needs.

Sequence:
  1. pre-flight checks: the tsdp app is on board (log/param TOC), battery, parameters read back;
  2. PID controller (tuned for the Crazyflie 2.1 Brushless; --controller 2 for Mellinger once tuned),
     Kalman reset, motor arming (the Brushless requires it), high-level takeoff to `start`;
  3. goal and limits (param group tsdp), obstacle streaming over the app channel at 20 Hz;
  4. tsdp.run = 1: the on-board planner commands the drone;
  5. 100 Hz logging of four blocks (state, track, frame, health) plus the true positions of the virtual
     obstacles at each row's host time;
  6. stop at the goal / timeout / geofence / Ctrl-C: tsdp.run = 0, land, disarm
     (a second Ctrl-C sends the emergency stop instead of landing).

Outputs (in --out-dir, named by scenario and time):
  <name>.csv        one row per state sample (100 Hz) with the latest value of every logged variable
  <name>.meta.json  scenario, arguments, parameters read back, firmware revision, outcome
Analyse with host/analyze_flight.py.

Scenario file (JSON): see host/scenarios/*.json. Obstacle centers are at the moment tsdp.run is set
and move with the given velocity (virtual obstacles; the drone never touches them).
"""
import argparse
import csv
import json
import math
import os
import signal
import struct
import sys
import time

import cflib
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.utils import uri_helper

# Each block <= 26 bytes (CRTP log packet). Types must match the firmware's LOG_ADD declarations.
LOG_BLOCKS = {
    "state": [("stateEstimate.x", "float"), ("stateEstimate.y", "float"), ("stateEstimate.z", "float"),
              ("stateEstimate.vx", "float"), ("stateEstimate.vy", "float"), ("stateEstimate.vz", "float")],
    "track": [("tsdp.spx", "float"), ("tsdp.spy", "float"), ("tsdp.spz", "float"),
              ("tsdp.trackErr", "float"), ("tsdp.rho", "float"), ("tsdp.decision", "uint8_t")],
    "frame": [("tsdp.frames", "uint32_t"), ("tsdp.planUs", "uint32_t"), ("tsdp.frameUs", "uint32_t"),
              ("tsdp.iters", "uint16_t"), ("tsdp.attempts", "uint8_t"), ("tsdp.reset", "uint8_t"),
              ("tsdp.obs", "uint8_t"), ("tsdp.obsAgeMs", "uint16_t"), ("tsdp.late", "uint32_t")],
    "health": [("tsdp.failures", "uint32_t"), ("tsdp.stackPl", "uint16_t"), ("tsdp.stackSp", "uint16_t"),
               ("pm.vbat", "float"), ("tsdp.base", "uint8_t")],
    # Commitment rule: committed arrival minus the switch time (ms, -1 none), certified plan rejected (1 later than
    # promised, 2 costlier than the tail), promises made. 9 bytes.
    "commit": [("tsdp.promiseMs", "int32_t"), ("tsdp.reject", "uint8_t"), ("tsdp.recommits", "uint32_t")],
}
PARAMS_READ_BACK = ["tsdp.run", "tsdp.gx", "tsdp.gy", "tsdp.gz", "tsdp.budgetMs", "tsdp.vmax", "tsdp.amax", "tsdp.cruise",
                    "tsdp.margin", "tsdp.robotR", "tsdp.resetD", "tsdp.obsTimeout", "tsdp.footprint", "tsdp.commit",
                    "stabilizer.controller"]
FIRMWARE_PARAMS = ["firmware.revision0", "firmware.revision1", "firmware.modified"]


def obstacle_packet(index, center, velocity, radius):
    return struct.pack("<B7f", index, *center, *velocity, radius)   # 29 bytes = tsdp_obstacle_packet_t


def obstacle_truth(obstacles, t):
    """Positions of the virtual obstacles t seconds after tsdp.run was set (None before)."""
    return [[o["center"][c] + t * o["velocity"][c] for c in range(3)] for o in obstacles]


def param_value(cf, name):
    try:
        return cf.param.get_value(name)
    except Exception as e:  # noqa: BLE001 - missing parameter
        return f"unavailable ({type(e).__name__})"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--uri", default=uri_helper.uri_from_env(default="radio://0/80/2M/E7E7E7E7E7"))
    ap.add_argument("--scenario", required=True)
    ap.add_argument("--out-dir", default="flights")
    ap.add_argument("--timeout", type=float, default=20.0)
    ap.add_argument("--fence", type=float, nargs=6, default=[-1.5, -1.5, 0.1, 3.0, 1.5, 1.6],
                    metavar=("XMIN", "YMIN", "ZMIN", "XMAX", "YMAX", "ZMAX"))
    ap.add_argument("--min-vbat", type=float, default=3.6, help="refuse to take off below this battery voltage")
    ap.add_argument("--controller", type=int, default=1, choices=[1, 2],
                    help="1 PID (default; tuned for the Brushless), 2 Mellinger (tune its gains/massThrust first)")
    ap.add_argument("--dry-run", action="store_true", help="no takeoff, run stays 0: checks, params and logging only")
    ap.add_argument("--bench", action="store_true",
                    help="props off: motors are never armed, tsdp.run = 1 for --timeout s. The planner runs on board against "
                         "the streamed obstacles and follows its own reference (tsdp.resetD is raised for the run and restored), "
                         "so planning time, stack headroom and the commitment logs are measured without flying")
    ap.add_argument("--hover", action="store_true",
                    help="hover test: the goal is the scenario's start, so after takeoff the on-board planner holds the drone "
                         "there for the whole --timeout (the run does not stop at the goal)")
    ap.add_argument("--commit", type=int, choices=[0, 1], help="set tsdp.commit (1 commitment rule, 0 first certified); "
                    "default: leave the firmware's value (1)")
    ap.add_argument("--build-commit", default=None, help="source commit the flashed firmware was built from (recorded in meta)")
    a = ap.parse_args()
    sc = json.load(open(a.scenario))
    if a.hover: sc["goal"] = list(sc["start"])   # hold the start point
    obstacles = sc.get("obstacles", [])[:8]
    os.makedirs(a.out_dir, exist_ok=True)
    if a.dry_run and a.bench: ap.error("--dry-run and --bench exclude each other")
    flying = not (a.dry_run or a.bench)
    name = (f"{os.path.splitext(os.path.basename(a.scenario))[0]}_{time.strftime('%Y%m%d_%H%M%S')}"
            + ("_dry" if a.dry_run else "_bench" if a.bench else "") + ("_hover" if a.hover else "")
            + ("" if a.commit is None else f"_commit{a.commit}"))
    csv_path, meta_path = os.path.join(a.out_dir, name + ".csv"), os.path.join(a.out_dir, name + ".meta.json")

    meta = {"scenario_file": a.scenario, "scenario": sc, "args": vars(a), "cflib": getattr(cflib, "__version__", "?"),
            "build_commit": a.build_commit,
            "mode": ("dry run" if a.dry_run else "bench (props off)" if a.bench else "flight") + (", hover" if a.hover else ""),
            "host_start": time.strftime("%Y-%m-%dT%H:%M:%S"), "outcome": None}
    stop = {"count": 0}
    signal.signal(signal.SIGINT, lambda *_: stop.__setitem__("count", stop["count"] + 1))

    cflib.crtp.init_drivers()
    rows, latest = [], {}
    state = {"run": 0, "t_run": None}

    with SyncCrazyflie(a.uri, cf=Crazyflie(rw_cache="./cache")) as scf:
        cf = scf.cf
        # ---- pre-flight checks
        toc = cf.log.toc
        missing = [n for block in LOG_BLOCKS.values() for n, _ in block if toc.get_element_by_complete_name(n) is None]
        if missing:
            print(f"ABORT: firmware lacks log variables {missing}: flash the current app (with the commitment rule) first"); return 2
        meta["firmware"] = {p: param_value(cf, p) for p in FIRMWARE_PARAMS}
        p = cf.param.set_value
        p("tsdp.run", "0")
        p("stabilizer.controller", str(a.controller))
        for pname, key in (("vmax", "vmax"), ("amax", "amax"), ("cruise", "cruise"), ("margin", "margin"),
                           ("robotR", "robot_radius"), ("budgetMs", "budget_ms")):
            if key in sc: p(f"tsdp.{pname}", str(sc[key]))
        for c, axis in enumerate("xyz"):
            p(f"tsdp.g{axis}", str(sc["goal"][c]))
        if a.commit is not None: p("tsdp.commit", str(a.commit))
        reset_d_saved = param_value(cf, "tsdp.resetD")
        if a.bench: p("tsdp.resetD", "100")   # the drone does not move: plan from the reference, never from the estimate
        time.sleep(0.5)
        meta["params"] = {n: param_value(cf, n) for n in PARAMS_READ_BACK}

        # ---- logging
        configs = []
        for block_name, variables in LOG_BLOCKS.items():
            lc = LogConfig(name=f"tsdp_{block_name}", period_in_ms=10)
            for n, typ in variables: lc.add_variable(n, typ)
            def cb(ts, data, _lc, block_name=block_name):
                latest.update(data)
                if block_name == "state":
                    t_run = None if state["t_run"] is None else time.time() - state["t_run"]
                    row = {"host_t": time.time(), "cf_ts_ms": ts, "run": state["run"], "t_run": t_run, **latest}
                    if t_run is not None:
                        for i, (pos, o) in enumerate(zip(obstacle_truth(obstacles, t_run), obstacles)):
                            row.update({f"obs{i}_x": pos[0], f"obs{i}_y": pos[1], f"obs{i}_z": pos[2], f"obs{i}_r": o["radius"]})
                    rows.append(row)
            lc.data_received_cb.add_callback(cb)
            cf.log.add_config(lc); lc.start(); configs.append(lc)
        time.sleep(0.5)
        vbat = latest.get("pm.vbat")
        meta["vbat_start"] = vbat
        print(f"firmware {meta['firmware']}; planner footprint {meta['params'].get('tsdp.footprint')} bytes; battery {vbat} V")
        if flying and (vbat is None or vbat < a.min_vbat):
            print(f"ABORT: battery {vbat} V below {a.min_vbat} V"); meta["outcome"] = "aborted: battery"
        else:
            p("kalman.resetEstimation", "1"); time.sleep(0.1); p("kalman.resetEstimation", "0"); time.sleep(2.0)
            hl = cf.high_level_commander
            start = sc["start"]
            if flying:
                cf.platform.send_arming_request(True); time.sleep(1.0)
                hl.takeoff(start[2], 2.0); time.sleep(2.5)
                hl.go_to(start[0], start[1], start[2], 0.0, 3.0); time.sleep(3.5)
            cf.appchannel.send_packet(struct.pack("<B", 0xFF) + bytes(28))   # clear obstacles
            state["t_run"] = time.time()
            if not a.dry_run:   # flight or bench
                p("tsdp.run", "1"); state["run"] = 1
            outcome = "hover completed" if a.hover else "timeout"
            while time.time() - state["t_run"] < a.timeout:
                if stop["count"]: outcome = "operator stop"; break
                t = time.time() - state["t_run"]
                for i, (pos, o) in enumerate(zip(obstacle_truth(obstacles, t), obstacles)):
                    cf.appchannel.send_packet(obstacle_packet(i, pos, o["velocity"], o["radius"]))
                x = [latest.get(f"stateEstimate.{k}", 0.0) for k in "xyz"]
                v = [latest.get(f"stateEstimate.v{k}", 0.0) for k in "xyz"]
                if flying and not all(a.fence[c] <= x[c] <= a.fence[c + 3] for c in range(3)):
                    outcome = f"geofence at {[round(c, 3) for c in x]}"; break
                if a.bench and not a.hover:   # the reference (setpoint) is what moves on the bench
                    sp = [latest.get(f"tsdp.sp{k}") for k in "xyz"]
                    if None not in sp and math.dist(sp, sc["goal"]) < 0.1: outcome = "reference reached goal"; break
                elif not a.hover and math.dist(x, sc["goal"]) < 0.1 and math.hypot(*v) < 0.1:
                    outcome = "reached"; break
                time.sleep(0.05)
            meta["outcome"] = outcome
            meta["time_to_outcome_s"] = time.time() - state["t_run"]
            p("tsdp.run", "0"); state["run"] = 0
            if a.bench and reset_d_saved is not None: p("tsdp.resetD", str(reset_d_saved))
            if flying:
                if stop["count"] > 1:
                    cf.commander.send_stop_setpoint()
                else:
                    pos = [latest.get(f"stateEstimate.{k}", 0.0) for k in "xyz"]
                    hl.go_to(pos[0], pos[1], pos[2], 0.0, 0.5); time.sleep(0.6)
                    hl.land(0.0, 2.5); time.sleep(3.0)
                    hl.stop()
                cf.platform.send_arming_request(False)
            time.sleep(0.3)
        for lc in configs: lc.stop()
        meta["vbat_end"] = latest.get("pm.vbat")

    fields = []
    for r in rows:
        for k in r:
            if k not in fields: fields.append(k)
    with open(csv_path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=fields); w.writeheader(); w.writerows(rows)
    meta["rows"] = len(rows)
    json.dump(meta, open(meta_path, "w"), indent=1, default=str)
    print(f"outcome: {meta['outcome']}; {len(rows)} rows -> {csv_path}")
    print(f"analyse: python3 {os.path.join(os.path.dirname(__file__), 'analyze_flight.py')} {csv_path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
