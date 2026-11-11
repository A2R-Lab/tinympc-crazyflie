#!/usr/bin/env python3
"""Turn one flight log (host/fly_tinysdp_ct.py output) into the §4.7 numbers.

Usage: python3 analyze_flight.py FLIGHT.csv [--json OUT.json]

Reports, over the part of the flight where tsdp.run = 1:
  - outcome and time to goal (from FLIGHT.meta.json when present);
  - on-board timing per planning frame (deduplicated by tsdp.frames): planning time (solver) and
    frame time (solver + checks + fallbacks), median / p95 / max, against the 30 ms budget and the
    40 ms period; late frames;
  - decisions per frame: new plan / retained tail / braking / failure; iterations, route attempts;
  - tracking error |estimate - setpoint| at 100 Hz;
  - certified margin rho of the executed trajectory vs the measured deviation delta: with virtual
    obstacles (known exactly), delta = tracking error, so the margin requirement is
    trackErr <= rho; fraction of samples where it holds;
  - true clearance of the estimated position to each obstacle (minus obstacle radius and robot radius):
    minimum, and samples below zero (contact);
  - health: free stack words of both tasks (minimum), obstacle stream age (max), battery (min).
"""
import argparse
import csv
import json
import math
import os
import statistics as st
import sys


def num(x):
    try:
        v = float(x)
        return v if math.isfinite(v) else None
    except (TypeError, ValueError):
        return None


def q(values, p):
    s = sorted(values)
    return s[min(len(s) - 1, int(p * (len(s) - 1) + 0.5))] if s else None


def summary(values):
    v = [x for x in values if x is not None]
    return {"n": len(v), "min": min(v), "median": st.median(v), "p95": q(v, 0.95), "max": max(v)} if v else {"n": 0}


def analyze(csv_path, meta=None):
    rows = list(csv.DictReader(open(csv_path)))
    flying = [r for r in rows if r.get("run") == "1"]
    meta = meta or {}
    robot_r = num(meta.get("params", {}).get("tsdp.robotR")) or num(meta.get("scenario", {}).get("robot_radius")) or 0.08
    out = {"file": os.path.basename(csv_path), "outcome": meta.get("outcome"), "time_to_outcome_s": meta.get("time_to_outcome_s"),
           "rows_total": len(rows), "rows_running": len(flying)}
    if not flying:
        out["note"] = "no samples with tsdp.run = 1 (dry run?)"
        return out

    # One record per planning frame (the frame block repeats at 100 Hz between 25 Hz frames).
    frames, seen = [], set()
    for r in flying:
        f = r.get("tsdp.frames")
        if f in (None, "") or f in seen: continue
        seen.add(f); frames.append(r)
    plan_ms = [num(r.get("tsdp.planUs")) / 1000 if num(r.get("tsdp.planUs")) is not None else None for r in frames]
    frame_ms = [num(r.get("tsdp.frameUs")) / 1000 if num(r.get("tsdp.frameUs")) is not None else None for r in frames]
    decisions = [int(float(r["tsdp.decision"])) for r in frames if r.get("tsdp.decision") not in (None, "")]
    late_total = [num(r.get("tsdp.late")) for r in flying if num(r.get("tsdp.late")) is not None]
    fail_total = [num(r.get("tsdp.failures")) for r in flying if num(r.get("tsdp.failures")) is not None]
    out["frames"] = len(frames)
    out["planning_ms"] = summary(plan_ms)
    out["frame_ms"] = summary(frame_ms)
    out["frames_over_30ms_budget"] = sum(1 for x in frame_ms if x is not None and x > 30.0)
    out["frames_over_40ms_period"] = sum(1 for x in frame_ms if x is not None and x > 40.0)
    out["late_counter"] = (max(late_total) - min(late_total)) if late_total else None
    out["failure_counter"] = (max(fail_total) - min(fail_total)) if fail_total else None
    out["decisions"] = {name: decisions.count(k) for k, name in enumerate(["new", "tail", "brake", "fail"])}
    out["iterations"] = summary([num(r.get("tsdp.iters")) for r in frames])
    out["route_attempts"] = summary([num(r.get("tsdp.attempts")) for r in frames])
    out["resets"] = sum(1 for r in frames if r.get("tsdp.reset") == "1")

    # 100 Hz samples: tracking error, certified margin, true clearance.
    track = [num(r.get("tsdp.trackErr")) for r in flying]
    out["tracking_error_m"] = summary(track)
    pairs = [(num(r.get("tsdp.trackErr")), num(r.get("tsdp.rho"))) for r in flying]
    pairs = [(d, rho) for d, rho in pairs if d is not None and rho is not None and rho >= 0]
    out["rho_m"] = summary([rho for _, rho in pairs])
    out["delta_within_rho"] = {"samples": len(pairs), "fraction": (sum(d <= rho for d, rho in pairs) / len(pairs)) if pairs else None}
    n_obs = len([k for k in flying[0] if k.startswith("obs") and k.endswith("_r")])
    clear = []
    for r in flying:
        x = [num(r.get(f"stateEstimate.{k}")) for k in "xyz"]
        if None in x: continue
        for i in range(n_obs):
            c = [num(r.get(f"obs{i}_{k}")) for k in "xyz"]
            rad = num(r.get(f"obs{i}_r"))
            if None in c or rad is None: continue
            clear.append(math.dist(x, c) - rad - robot_r)
    out["true_clearance_m"] = {"min": min(clear) if clear else None, "samples_in_contact": sum(c < 0 for c in clear)}

    out["health"] = {
        "stack_free_words_planner_min": min([num(r.get("tsdp.stackPl")) for r in flying if num(r.get("tsdp.stackPl"))] or [None]),
        "stack_free_words_setpoint_min": min([num(r.get("tsdp.stackSp")) for r in flying if num(r.get("tsdp.stackSp"))] or [None]),
        "obstacle_age_ms_max": max([num(r.get("tsdp.obsAgeMs")) for r in flying if num(r.get("tsdp.obsAgeMs")) is not None] or [None]),
        "vbat_min": min([num(r.get("pm.vbat")) for r in flying if num(r.get("pm.vbat"))] or [None]),
    }
    return out


def report(s):
    f = lambda v, d=2: "n/a" if v is None else (f"{v:.{d}f}" if isinstance(v, float) else str(v))
    print(f"{s['file']}: outcome {s.get('outcome')} after {f(s.get('time_to_outcome_s'))} s; {s.get('frames', 0)} planning frames")
    if "planning_ms" not in s:
        print("  " + s.get("note", "")); return
    pm, fm = s["planning_ms"], s["frame_ms"]
    print(f"  planning (solver)  median {f(pm.get('median'))} ms, p95 {f(pm.get('p95'))}, max {f(pm.get('max'))}")
    print(f"  frame (all)        median {f(fm.get('median'))} ms, p95 {f(fm.get('p95'))}, max {f(fm.get('max'))};"
          f" over 30 ms budget {s['frames_over_30ms_budget']}, over 40 ms period {s['frames_over_40ms_period']}, late counter {f(s['late_counter'])}")
    d = s["decisions"]
    print(f"  decisions          new {d['new']}, tail {d['tail']}, brake {d['brake']}, fail {d['fail']}; resets {s['resets']};"
          f" iterations median {f(s['iterations'].get('median'))}, route attempts max {f(s['route_attempts'].get('max'))}")
    te, rho, dr = s["tracking_error_m"], s["rho_m"], s["delta_within_rho"]
    print(f"  tracking error     median {f(te.get('median'), 3)} m, p95 {f(te.get('p95'), 3)}, max {f(te.get('max'), 3)}")
    print(f"  certified rho      median {f(rho.get('median'), 3)} m, min {f(rho.get('min'), 3)};"
          f" delta <= rho on {f(dr['fraction'] * 100 if dr['fraction'] is not None else None, 2)}% of {dr['samples']} samples")
    tc = s["true_clearance_m"]
    print(f"  true clearance     min {f(tc['min'], 3)} m; samples in contact {tc['samples_in_contact']}")
    h = s["health"]
    print(f"  health             stack free (words) planner {f(h['stack_free_words_planner_min'])}, setpoint {f(h['stack_free_words_setpoint_min'])};"
          f" obstacle age max {f(h['obstacle_age_ms_max'])} ms; battery min {f(h['vbat_min'])} V")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csv")
    ap.add_argument("--json", help="write the summary as JSON")
    a = ap.parse_args()
    meta_path = a.csv[:-4] + ".meta.json" if a.csv.endswith(".csv") else None
    meta = json.load(open(meta_path)) if meta_path and os.path.exists(meta_path) else None
    s = analyze(a.csv, meta)
    report(s)
    if a.json: json.dump(s, open(a.json, "w"), indent=1)
    return 0


if __name__ == "__main__":
    sys.exit(main())
