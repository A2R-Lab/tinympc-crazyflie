#!/usr/bin/env python3
"""compare_vision_truth.py -- how accurate is the camera's world-position estimate?

Joins a vision run CSV (from tinympc_circuit_run.py / tinympc_hover_run.py, with the
`host_t` column + a `*.meta.json` sidecar) against a CONCURRENT mocap log
(mocap_ros2_logger.py format) and writes a per-frame table of

    where the CAMERA says the drone is   vs   where the drone REALLY is (mocap),

both in the same frame, plus the error. This is the ground-truth check the estimator's
own `x/y/z` can't give you (that's the drifting Flow-deck odometry, not truth).

WHAT "the camera says" MEANS
    The firmware turns a sighting of a surveyed gate into an ABSOLUTE drone position
    (controller_tinympc.cpp: imp = surveyed_gate - measured_offset). The run CSV logs it
    as imp_x/imp_y/imp_z. It is independent of the drifting estimate -- it depends only
    on the camera measurement + the (surveyed) gate location -- so it is a real
    camera-based localization, and its error vs mocap is the accuracy of the camera +
    its calibration (intrinsics, the range/aperture scale, and the mount boresight trims).

FRAMES
    The run is in the "reset frame": the drone sits at the origin facing +x at the
    instant kalman.resetEstimation fires. mocap is in its own world frame. Both are
    gravity-aligned (z up), so the transform between them is a yaw rotation + a 3D
    translation (4 DOF). We recover it from the mocap DRONE pose at the reset instant
    (meta.reset_host_t): that pose maps to (0,0,0, yaw=0) in the reset frame. If there is
    no reset time, fall back to fitting the transform over an early low-drift window of
    the run against the estimator (--align fit).

TIME SYNC
    Both machines stamp host time.time(); NTP keeps them within a few ms (see the mocap
    runbook). Image/pipeline latency is a near-constant lag of the camera behind truth;
    pass --offset to shift the run's host_t before the join (positive = the camera row is
    treated as older than its host_t, i.e. it reflects where the drone was --offset s
    ago). Sweep it to minimise the error if you haven't calibrated it.

GATE-TRUTH MODE (--gate-truth)
    imp_* is anchored on the TAPE-SURVEYED gate (gAx.. in the run). If the mocap log also
    tracked the gate rigid body, --gate-truth recomputes the camera position against the
    MOCAP gate instead (imp' = mocap_gate - measured_offset, where the measured offset is
    recovered from the logged vg_g and the estimator). That removes survey error and
    leaves pure camera error. Needs vg_gx/gy/gz and gate columns in both CSVs.

USAGE
    # single run + its concurrent mocap log
    ./compare_vision_truth.py --run tinympc_circuit_run.csv --mocap flight_09.csv \
        --out vision_vs_truth.csv
    # sweep the latency offset to find the best sync
    ./compare_vision_truth.py --run r.csv --mocap m.csv --sweep -0.10 0.02 0.01
    # use the mocap gate as the anchor (removes tape-survey error)
    ./compare_vision_truth.py --run r.csv --mocap m.csv --gate-truth

Only depends on numpy.
"""
import argparse
import csv
import json
import math
import os
import sys

import numpy as np


# --------------------------------------------------------------------------- io
def read_run(path):
    """Run CSV -> dict of column -> np.array (float where possible; else object)."""
    with open(path, newline="") as f:
        rows = list(csv.reader(f))
    if not rows:
        sys.exit(f"empty run CSV: {path}")
    hdr = rows[0]
    cols = {h: [] for h in hdr}
    for r in rows[1:]:
        if len(r) != len(hdr):
            continue
        for h, v in zip(hdr, r):
            cols[h].append(v)

    def num(v):
        try:
            return float(v)
        except (ValueError, TypeError):
            return np.nan

    out = {}
    for h, vals in cols.items():
        arr = np.array([num(v) for v in vals], dtype=float)
        out[h] = arr
    if "host_t" not in out or np.all(np.isnan(out["host_t"])):
        sys.exit(f"{path} has no usable host_t column -- re-fly with the updated "
                 f"tinympc_circuit_run.py (it stamps host_t per row).")
    return out


def read_mocap(path):
    """mocap_ros2_logger.py CSV -> arrays. Returns (t, drone_p[N,3], drone_q[N,4 xyzw],
    gates={i: (p[N,3], q[N,4])}). Skips short/corrupt rows."""
    with open(path, newline="") as f:
        rows = list(csv.reader(f))
    hdr = rows[0]
    idx = {h: i for i, h in enumerate(hdr)}
    need = ["timestamp", "drone_px", "drone_py", "drone_pz",
            "drone_qx", "drone_qy", "drone_qz", "drone_qw"]
    for c in need:
        if c not in idx:
            sys.exit(f"{path} missing column '{c}' -- is it a mocap_ros2_logger CSV?")
    gate_ids = sorted({h.split("_")[0] for h in hdr if h.startswith("gate")
                       and h.endswith("_px")},
                      key=lambda g: g)  # 'gate1','gate2',...

    t, dp, dq = [], [], []
    gp = {g: [] for g in gate_ids}
    gq = {g: [] for g in gate_ids}
    for r in rows[1:]:
        if len(r) < len(hdr):
            continue
        try:
            t.append(float(r[idx["timestamp"]]))
            dp.append([float(r[idx["drone_px"]]), float(r[idx["drone_py"]]),
                       float(r[idx["drone_pz"]])])
            dq.append([float(r[idx["drone_qx"]]), float(r[idx["drone_qy"]]),
                       float(r[idx["drone_qz"]]), float(r[idx["drone_qw"]])])
            for g in gate_ids:
                gp[g].append([float(r[idx[g + "_px"]]), float(r[idx[g + "_py"]]),
                              float(r[idx[g + "_pz"]])])
                gq[g].append([float(r[idx[g + "_qx"]]), float(r[idx[g + "_qy"]]),
                              float(r[idx[g + "_qz"]]), float(r[idx[g + "_qw"]])])
        except (ValueError, IndexError):
            continue
    t = np.array(t)
    order = np.argsort(t)
    t = t[order]
    dp = np.array(dp)[order]
    dq = np.array(dq)[order]
    gates = {}
    for i, g in enumerate(gate_ids, start=1):
        gates[i] = (np.array(gp[g])[order], np.array(gq[g])[order])
    return t, dp, dq, gates


# ------------------------------------------------------------------- geometry
def yaw_from_quat(q):
    """Yaw (rotation about world z) from a quaternion [x,y,z,w]. Robust for small roll/
    pitch; we only need the heading to define the reset frame's +x."""
    x, y, z, w = q
    siny = 2.0 * (w * z + x * y)
    cosy = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny, cosy)


def interp_pose(t_query, t_src, p_src, q_src, max_dt):
    """Linear-interpolate position (and pick the nearer quaternion) at each t_query.
    Returns (p[M,3], q[M,4], dt[M]); rows whose nearest sample is > max_dt away come back
    as NaN so the caller can drop them."""
    M = len(t_query)
    p = np.full((M, 3), np.nan)
    q = np.full((M, 4), np.nan)
    dt = np.full(M, np.nan)
    if len(t_src) < 2:
        return p, q, dt
    lo = t_src[0]
    hi = t_src[-1]
    for i, tq in enumerate(t_query):
        if not np.isfinite(tq) or tq < lo - max_dt or tq > hi + max_dt:
            continue
        j = np.searchsorted(t_src, tq)
        if j <= 0:
            k0 = k1 = 0
        elif j >= len(t_src):
            k0 = k1 = len(t_src) - 1
        else:
            k0, k1 = j - 1, j
        # nearest sample distance (for the max_dt gate + reporting)
        near = min(abs(tq - t_src[k0]), abs(tq - t_src[k1]))
        if near > max_dt:
            continue
        dt[i] = near
        if k0 == k1 or t_src[k1] == t_src[k0]:
            a = 0.0
        else:
            a = (tq - t_src[k0]) / (t_src[k1] - t_src[k0])
            a = min(1.0, max(0.0, a))
        p[i] = (1 - a) * p_src[k0] + a * p_src[k1]
        q[i] = q_src[k0] if a < 0.5 else q_src[k1]   # heading only; no slerp needed
    return p, q, dt


def make_reset_transform(p_reset, yaw_reset):
    """Return a function mapping mocap-world positions -> reset frame. Reset frame: origin
    at the drone body at reset, +x along the drone's horizontal heading at reset, z up.
        p_reset[3], yaw_reset[rad]. """
    c, s = math.cos(-yaw_reset), math.sin(-yaw_reset)   # rotate by -yaw about z
    Rz = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
    p0 = np.asarray(p_reset, dtype=float)

    def to_reset(P):
        P = np.atleast_2d(np.asarray(P, dtype=float))
        return (Rz @ (P - p0).T).T
    return to_reset


def fit_yaw_translation(src_xyz, dst_xyz):
    """Best 4-DOF (yaw about z + 3D translation) transform mapping src -> dst, least
    squares. Used as the fallback frame alignment (mocap drone -> estimator over an early
    low-drift window). Returns a to_reset(P) callable. Horizontal rotation solved in xy;
    z is a pure offset."""
    src = np.asarray(src_xyz, float)
    dst = np.asarray(dst_xyz, float)
    sc = src[:, :2] - src[:, :2].mean(0)
    dc = dst[:, :2] - dst[:, :2].mean(0)
    # optimal 2D rotation (Kabsch in the plane)
    H = sc.T @ dc
    num = H[0, 1] - H[1, 0]     # sum(sx*dy - sy*dx)
    den = H[0, 0] + H[1, 1]     # sum(sx*dx + sy*dy)
    theta = math.atan2(num, den)
    c, s = math.cos(theta), math.sin(theta)
    Rz = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
    t = dst.mean(0) - Rz @ src.mean(0)

    def to_reset(P):
        P = np.atleast_2d(np.asarray(P, dtype=float))
        return (Rz @ P.T).T + t
    return to_reset, math.degrees(theta)


# ------------------------------------------------------------------- core join
def build_comparison(run, mocap, meta, offset, max_dt, align, fit_window,
                     gate_truth):
    t_m, dp_m, dq_m, gates_m = mocap
    host = run["host_t"] - offset          # shift the camera stream by the latency offset

    # --- 1. frame alignment: mocapWorld -> reset frame -----------------------
    reset_t = (meta or {}).get("reset_host_t")
    align_note = ""
    if align != "fit" and reset_t:
        p_r, q_r, dtr = interp_pose(np.array([reset_t]), t_m, dp_m, dq_m,
                                    max_dt=0.5)
        if not np.isfinite(p_r[0, 0]):
            sys.exit("no mocap sample near reset_host_t -- was the logger running at "
                     "reset? Use --align fit to bypass.")
        to_reset = make_reset_transform(p_r[0], yaw_from_quat(q_r[0]))
        align_note = (f"reset-pose @ t={reset_t:.3f} (mocap dt={dtr[0]*1e3:.0f} ms), "
                      f"drone at ({p_r[0,0]:+.3f},{p_r[0,1]:+.3f},{p_r[0,2]:+.3f}) "
                      f"yaw {math.degrees(yaw_from_quat(q_r[0])):+.0f}deg")
    else:
        # fallback: fit yaw+translation over the first `fit_window` s of the run, mocap
        # drone -> estimator (x/y/z). Valid while drift is still small.
        t0 = np.nanmin(host)
        m = (host - t0) <= fit_window
        m &= np.isfinite(run["x"]) & np.isfinite(host)
        if m.sum() < 10:
            sys.exit("too few early rows to fit the frame -- widen --fit-window.")
        p_e, _, _ = interp_pose(host[m], t_m, dp_m, dq_m, max_dt=max_dt)
        good = np.isfinite(p_e[:, 0])
        est = np.stack([run["x"][m], run["y"][m], run["z"][m]], axis=1)[good]
        to_reset, theta = fit_yaw_translation(p_e[good], est)
        align_note = (f"fit over first {fit_window:.0f}s ({good.sum()} pts), "
                      f"yaw {theta:+.1f}deg  [NO reset time -- accuracy limited by "
                      f"early odometry drift]")

    # --- 2. per-row truth: mocap drone pose at each run host_t ---------------
    p_d, q_d, dt = interp_pose(host, t_m, dp_m, dq_m, max_dt)
    true_reset = to_reset(p_d)             # NaN rows stay NaN

    # --- 3. the camera estimate ---------------------------------------------
    for c in ("imp_x", "imp_y", "imp_z"):
        if c not in run:
            sys.exit(f"run CSV has no '{c}' -- it must be a tinympc_circuit_run.py CSV "
                     f"(the one that logs the vision-implied position).")
    cam = np.stack([run["imp_x"], run["imp_y"], run["imp_z"]], axis=1)
    if gate_truth:
        cam = recompute_against_mocap_gate(run, host, t_m, gates_m, to_reset, max_dt)

    est = np.stack([run["x"], run["y"], run["z"]], axis=1)   # odometry, for reference

    # --- 4. validity mask ----------------------------------------------------
    valid = (np.isfinite(cam).all(1) & np.isfinite(true_reset).all(1)
             & np.isfinite(dt))
    if "vg_valid" in run:
        valid &= (run["vg_valid"] == 1)
    return {
        "host": host, "t": run.get("t"), "gate": run.get("vf_gate"),
        "range": run.get("vg_range"), "cam": cam, "true": true_reset,
        "est": est, "dt": dt, "valid": valid, "align_note": align_note,
    }


def recompute_against_mocap_gate(run, host, t_m, gates_m, to_reset, max_dt):
    """imp' = mocap_gate(reset frame) - measured_offset, where the measured drone->gate
    offset (world axes) is recovered from the logged vision gate: vg_g = estimator +
    offset  =>  offset = vg_g - estimator. Anchoring on the MOCAP gate instead of the
    tape survey removes survey error, leaving pure camera error. The run's vf_gate
    (1=A, 2=B) selects which mocap gate to use per row."""
    for c in ("vg_gx", "vg_gy", "vg_gz", "vf_gate"):
        if c not in run:
            sys.exit(f"--gate-truth needs '{c}' in the run CSV.")
    off = np.stack([run["vg_gx"] - run["x"], run["vg_gy"] - run["y"],
                    run["vg_gz"] - run["z"]], axis=1)   # measured drone->gate, world axes
    # mocap gate position in the reset frame at each run time, per gate id
    gate_reset = {}
    for gi, (gp, gq) in gates_m.items():
        p, _, _ = interp_pose(host, t_m, gp, gq, max_dt)
        gate_reset[gi] = to_reset(p)        # NaN where no synced sample
    N = len(host)
    cam = np.full((N, 3), np.nan)
    for i in range(N):
        gi = run["vf_gate"][i]
        if not np.isfinite(gi):
            continue
        gi = int(gi)
        if gi in gate_reset and np.isfinite(gate_reset[gi][i]).all() \
                and np.isfinite(off[i]).all():
            cam[i] = gate_reset[gi][i] - off[i]
    return cam


# ------------------------------------------------------------------- reporting
def summarize(cmp, label=""):
    v = cmp["valid"]
    n = int(v.sum())
    if n == 0:
        print("  no valid rows to compare (no synced mocap + valid vision overlap).")
        return None
    err = cmp["cam"][v] - cmp["true"][v]
    est_err = cmp["est"][v] - cmp["true"][v]
    norm = np.linalg.norm(err, axis=1)
    est_norm = np.linalg.norm(est_err, axis=1)
    bias = err.mean(0)
    rms = math.sqrt((norm ** 2).mean())
    print(f"  {label}n={n}   sync |dt| mean {cmp['dt'][v].mean()*1e3:.1f} ms "
          f"(max {cmp['dt'][v].max()*1e3:.0f})")
    print(f"  camera err |e|:  mean {norm.mean()*100:.1f}  median "
          f"{np.median(norm)*100:.1f}  RMS {rms*100:.1f}  p95 "
          f"{np.percentile(norm,95)*100:.1f}  (cm)")
    print(f"  bias (cam-true): x {bias[0]*100:+.1f}  y {bias[1]*100:+.1f}  "
          f"z {bias[2]*100:+.1f}  cm   [std x {err[:,0].std()*100:.1f} "
          f"y {err[:,1].std()*100:.1f} z {err[:,2].std()*100:.1f}]")
    print(f"  odometry err |e|: mean {est_norm.mean()*100:.1f}  RMS "
          f"{math.sqrt((est_norm**2).mean())*100:.1f} cm   "
          f"(camera {'beats' if rms < math.sqrt((est_norm**2).mean()) else 'WORSE than'}"
          f" odometry over this window)")
    # per-range breakdown
    rng = cmp["range"]
    if rng is not None and np.isfinite(rng[v]).any():
        rv = rng[v]
        print("  by range:")
        for lo, hi in [(0, 1), (1, 1.5), (1.5, 2), (2, 99)]:
            m = (rv >= lo) & (rv < hi)
            if m.sum() >= 3:
                print(f"    {lo:.1f}-{hi:.1f} m: n={m.sum():4d}  |e| mean "
                      f"{norm[m].mean()*100:5.1f}  RMS "
                      f"{math.sqrt((norm[m]**2).mean())*100:5.1f} cm")
    return {"n": n, "rms_cm": rms * 100, "bias_cm": bias * 100}


def write_csv(path, cmp):
    v = cmp["valid"]
    err = cmp["cam"] - cmp["true"]
    est_err = cmp["est"] - cmp["true"]
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t", "host_t", "gate", "range", "sync_dt_ms",
                    "cam_x", "cam_y", "cam_z", "true_x", "true_y", "true_z",
                    "est_x", "est_y", "est_z",
                    "err_x", "err_y", "err_z", "err_norm",
                    "est_err_norm"])
        t = cmp["t"] if cmp["t"] is not None else cmp["host"]
        for i in np.where(v)[0]:
            w.writerow([
                f"{t[i]:.3f}", f"{cmp['host'][i]:.3f}",
                int(cmp["gate"][i]) if cmp["gate"] is not None
                and np.isfinite(cmp["gate"][i]) else "",
                f"{cmp['range'][i]:.3f}" if cmp["range"] is not None
                and np.isfinite(cmp["range"][i]) else "",
                f"{cmp['dt'][i]*1e3:.1f}",
                *[f"{x:.4f}" for x in cmp["cam"][i]],
                *[f"{x:.4f}" for x in cmp["true"][i]],
                *[f"{x:.4f}" for x in cmp["est"][i]],
                *[f"{x:.4f}" for x in err[i]],
                f"{np.linalg.norm(err[i]):.4f}",
                f"{np.linalg.norm(est_err[i]):.4f}",
            ])
    print(f"wrote {path}  ({int(v.sum())} compared rows)")


# ------------------------------------------------------------------------ main
def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--run", required=True, help="vision run CSV (has host_t)")
    ap.add_argument("--mocap", required=True, help="concurrent mocap_ros2_logger CSV")
    ap.add_argument("--meta", default=None,
                    help="run sidecar (default: <run>.meta.json)")
    ap.add_argument("--out", default="vision_vs_truth.csv")
    ap.add_argument("--offset", type=float, default=0.0,
                    help="camera latency [s]: run host_t is shifted back by this before "
                         "the join (positive = camera lags truth)")
    ap.add_argument("--max-dt", type=float, default=0.030,
                    help="drop rows whose nearest mocap sample is > this [s] away")
    ap.add_argument("--align", choices=("reset", "fit"), default="reset",
                    help="reset = use the reset-instant mocap pose (default); "
                         "fit = least-squares fit vs early odometry (no reset time)")
    ap.add_argument("--fit-window", type=float, default=8.0,
                    help="seconds of early run used by --align fit")
    ap.add_argument("--gate-truth", action="store_true",
                    help="anchor the camera position on the MOCAP gate, not the survey "
                         "(removes tape-survey error; needs gate cols in both CSVs)")
    ap.add_argument("--sweep", nargs=3, type=float, metavar=("LO", "HI", "STEP"),
                    default=None,
                    help="sweep --offset over [LO,HI] step STEP and report the RMS at "
                         "each (find the best sync), then write the best")
    a = ap.parse_args()

    run = read_run(a.run)
    mocap = read_mocap(a.mocap)
    meta_path = a.meta or (a.run.rsplit(".", 1)[0] + ".meta.json")
    meta = None
    if os.path.exists(meta_path):
        meta = json.load(open(meta_path))
    elif a.align == "reset":
        print(f"!! no {meta_path}; falling back to --align fit")
        a.align = "fit"

    print(f"run   {a.run}: {len(run['host_t'])} rows")
    print(f"mocap {a.mocap}: {len(mocap[0])} poses, "
          f"{len(mocap[3])} tracked gate(s)")

    if a.sweep:
        lo, hi, step = a.sweep
        best = None
        offs = np.arange(lo, hi + 1e-9, step)
        print(f"\nsweeping offset {lo:+.3f}..{hi:+.3f} step {step}:")
        for off in offs:
            cmp = build_comparison(run, mocap, meta, off, a.max_dt, a.align,
                                   a.fit_window, a.gate_truth)
            v = cmp["valid"]
            if v.sum() == 0:
                print(f"  offset {off:+.3f}: no overlap")
                continue
            e = np.linalg.norm(cmp["cam"][v] - cmp["true"][v], axis=1)
            rms = math.sqrt((e ** 2).mean())
            print(f"  offset {off:+.3f}: n={int(v.sum()):4d}  RMS {rms*100:5.1f} cm")
            if best is None or rms < best[1]:
                best = (off, rms)
        if best:
            print(f"\nbest offset {best[0]:+.3f} s (RMS {best[1]*100:.1f} cm) "
                  f"-> writing that")
            a.offset = best[0]

    cmp = build_comparison(run, mocap, meta, a.offset, a.max_dt, a.align,
                           a.fit_window, a.gate_truth)
    print(f"\nframe alignment: {cmp['align_note']}")
    print(f"offset {a.offset:+.3f} s, max_dt {a.max_dt*1e3:.0f} ms"
          + ("  [gate-truth: anchored on mocap gate]" if a.gate_truth else ""))
    print("\nCAMERA-PREDICTED vs TRUE (mocap) drone position:")
    summarize(cmp)
    write_csv(a.out, cmp)


if __name__ == "__main__":
    main()
