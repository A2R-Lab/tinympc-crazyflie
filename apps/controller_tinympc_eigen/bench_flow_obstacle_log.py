#!/usr/bin/env python3
"""Bench-log AI-deck optical-flow obstacle depth data to CSV.

This logger does not arm or send setpoints. By default it switches to the OOT
controller so the AI-deck UART receiver and flow-depth update path run, then
restores PID on exit. ADMM obstacle constraints are opt-in and are restored to
safe defaults when this script exits.
"""

import argparse
import csv
import os
import math
import time
from pathlib import Path

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.utils import uri_helper


DEFAULT_URI = "radio://0/80/2M/E7E7E7E7E8"
SECTORS = range(9)


LOG_BLOCKS = [
    ("health", [("flowObsRx.rxOk", "uint32_t"), ("flowObsRx.crcErr", "uint32_t"), ("flowObsRx.badRx", "uint32_t")]),
    ("packet", [("flowObsRx.n", "uint8_t"), ("flowObsRx.dt", "float"), ("flowObsRx.nearIdx", "uint8_t")]),
    ("motion", [("flowObsRx.bodyVx", "float"), ("flowObsRx.bodyVy", "float"), ("flowObsRx.yawRate", "float")]),
    ("obs_a", [("flowObsRx.obsValid", "float"), ("flowObsRx.obsHits", "uint8_t"), ("flowObsRx.obsCount", "uint8_t")]),
    ("obs_b", [
        ("flowObsRx.obsRange", "float"),
        ("flowObsRx.obsBear", "float"),
        ("flowObsRx.obsBx", "float"),
        ("flowObsRx.obsBy", "float"),
        ("flowObsRx.obsWx", "float"),
        ("flowObsRx.obsWy", "float"),
    ]),
    ("cyl_a", [("flowObsRx.cylValid", "float"), ("flowObsRx.cylConf", "float"), ("flowObsRx.cylReject", "float"), ("flowObsRx.cylInnov", "float")]),
    ("cyl_b", [
        ("flowObsRx.cylBx", "float"),
        ("flowObsRx.cylBy", "float"),
        ("flowObsRx.cylWx", "float"),
        ("flowObsRx.cylWy", "float"),
    ]),
    ("cyl_cov", [
        ("flowObsRx.cylVarX", "float"),
        ("flowObsRx.cylVarY", "float"),
        ("flowObsRx.cylCovXY", "float"),
        ("flowObsRx.mapPeak", "float"),
        ("flowObsRx.mapActive", "uint8_t"),
        ("flowObsRx.mapBest", "uint8_t"),
    ]),
    ("admm_obs", [
        ("obs.source", "uint8_t"),
        ("obs.active", "uint8_t"),
        ("obs.apply", "uint8_t"),
        ("obs.count", "uint8_t"),
        ("obs.effCx", "float"),
        ("obs.effCy", "float"),
        ("obs.frzValid", "uint8_t"),
        ("obs.frzCx", "float"),
        ("obs.frzCy", "float"),
    ]),
    ("state_xy", [("stateEstimate.x", "float"), ("stateEstimate.y", "float"), ("stateEstimate.yaw", "float")]),
]

for i in range(6):
    LOG_BLOCKS.append((
        f"sector_{i}",
        [
            (f"flowObsRx.flowX{i}", "float"),
            (f"flowObsRx.resX{i}", "float"),
            (f"flowObsRx.vEff{i}", "float"),
            (f"flowObsRx.range{i}", "float"),
            (f"flowObsRx.valid{i}", "float"),
        ],
    ))


CSV_FIELDS = [
    "host_time",
    "t_s",
    "rx_ok",
    "crc_err",
    "bad_rx",
    "n",
    "dt",
    "body_vx",
    "body_vy",
    "yaw_rate",
    "near_valid",
    "near_idx",
    "near_range",
    "near_bx",
    "near_by",
    "near_wx",
    "near_wy",
    "obs_valid",
    "obs_hits",
    "obs_count",
    "obs_range",
    "obs_bearing",
    "obs_bx",
    "obs_by",
    "obs_wx",
    "obs_wy",
    "cyl_valid",
    "cyl_conf",
    "cyl_age",
    "cyl_reject",
    "cyl_innov",
    "cyl_bx",
    "cyl_by",
    "cyl_wx",
    "cyl_wy",
    "cyl_var_x",
    "cyl_var_y",
    "cyl_cov_xy",
    "map_peak",
    "map_active",
    "map_best",
    "admm_obs_source",
    "admm_obs_active",
    "admm_obs_apply",
    "admm_obs_count",
    "admm_obs_eff_cx",
    "admm_obs_eff_cy",
    "admm_obs_frz_valid",
    "admm_obs_frz_cx",
    "admm_obs_frz_cy",
    "state_x",
    "state_y",
    "state_yaw",
]

for name in ("flow_x", "res_x", "veff", "range", "valid", "bx", "by"):
    for i in SECTORS:
        CSV_FIELDS.append(f"{name}{i}")


def parse_args():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--uri", default=uri_helper.uri_from_env(default=DEFAULT_URI))
    p.add_argument("--out", default="flow_obstacle_bench.csv")
    p.add_argument("--duration", type=float, default=30.0, help="Seconds to record.")
    p.add_argument("--period-ms", type=int, default=100, help="Crazyflie log period for each small block.")
    p.add_argument("--sample-hz", type=float, default=20.0, help="CSV snapshot rate from latest received logs.")
    p.add_argument("--cache-dir", default=".cf_cache")
    p.add_argument("--no-controller-switch", action="store_true",
                   help="Do not switch stabilizer.controller to OOT/6 before logging.")
    p.add_argument("--restore-controller", type=int, default=1,
                   help="Controller id to restore on exit. Use -1 to leave unchanged.")
    mode = p.add_mutually_exclusive_group()
    mode.add_argument("--admm-log-only", action="store_true",
                      help="Set obs.useFlow=1 and obs.logOnly=1 so flow-derived halfspaces are logged but not applied.")
    mode.add_argument("--admm-apply", action="store_true",
                      help="Set obs.useFlow=1 and obs.logOnly=0 so valid flow-derived halfspaces are written into TinyMPC.")
    p.add_argument("--obs-safety", type=float, default=None,
                   help="Optional obs.safety value [m] to set before the run.")
    p.add_argument("--obs-delay-ms", type=int, default=None,
                   help="Optional obs.delayMs value to set before the run.")
    p.add_argument("--freeze-after-s", type=float, default=None,
                   help="Set obs.freeze=1 and latch the flow obstacle after this many seconds from OOT activation.")
    p.add_argument("--no-plot", action="store_true")
    return p.parse_args()


def set_param(cf, name, value, delay=0.1):
    cf.param.set_value(name, str(value))
    time.sleep(delay)


def latest_get(latest, key):
    return latest.get(key, "")


def make_row(latest, start_time):
    row = {
        "host_time": time.time(),
        "t_s": time.time() - start_time,
        "rx_ok": latest_get(latest, "flowObsRx.rxOk"),
        "crc_err": latest_get(latest, "flowObsRx.crcErr"),
        "bad_rx": latest_get(latest, "flowObsRx.badRx"),
        "n": latest_get(latest, "flowObsRx.n"),
        "dt": latest_get(latest, "flowObsRx.dt"),
        "body_vx": latest_get(latest, "flowObsRx.bodyVx"),
        "body_vy": latest_get(latest, "flowObsRx.bodyVy"),
        "yaw_rate": latest_get(latest, "flowObsRx.yawRate"),
        "near_valid": latest_get(latest, "flowObsRx.nearValid"),
        "near_idx": latest_get(latest, "flowObsRx.nearIdx"),
        "near_range": latest_get(latest, "flowObsRx.nearRange"),
        "near_bx": latest_get(latest, "flowObsRx.nearBx"),
        "near_by": latest_get(latest, "flowObsRx.nearBy"),
        "near_wx": latest_get(latest, "flowObsRx.nearWx"),
        "near_wy": latest_get(latest, "flowObsRx.nearWy"),
        "obs_valid": latest_get(latest, "flowObsRx.obsValid"),
        "obs_hits": latest_get(latest, "flowObsRx.obsHits"),
        "obs_count": latest_get(latest, "flowObsRx.obsCount"),
        "obs_range": latest_get(latest, "flowObsRx.obsRange"),
        "obs_bearing": latest_get(latest, "flowObsRx.obsBear"),
        "obs_bx": latest_get(latest, "flowObsRx.obsBx"),
        "obs_by": latest_get(latest, "flowObsRx.obsBy"),
        "obs_wx": latest_get(latest, "flowObsRx.obsWx"),
        "obs_wy": latest_get(latest, "flowObsRx.obsWy"),
        "cyl_valid": latest_get(latest, "flowObsRx.cylValid"),
        "cyl_conf": latest_get(latest, "flowObsRx.cylConf"),
        "cyl_age": latest_get(latest, "flowObsRx.cylAge"),
        "cyl_reject": latest_get(latest, "flowObsRx.cylReject"),
        "cyl_innov": latest_get(latest, "flowObsRx.cylInnov"),
        "cyl_bx": latest_get(latest, "flowObsRx.cylBx"),
        "cyl_by": latest_get(latest, "flowObsRx.cylBy"),
        "cyl_wx": latest_get(latest, "flowObsRx.cylWx"),
        "cyl_wy": latest_get(latest, "flowObsRx.cylWy"),
        "cyl_var_x": latest_get(latest, "flowObsRx.cylVarX"),
        "cyl_var_y": latest_get(latest, "flowObsRx.cylVarY"),
        "cyl_cov_xy": latest_get(latest, "flowObsRx.cylCovXY"),
        "map_peak": latest_get(latest, "flowObsRx.mapPeak"),
        "map_active": latest_get(latest, "flowObsRx.mapActive"),
        "map_best": latest_get(latest, "flowObsRx.mapBest"),
        "admm_obs_source": latest_get(latest, "obs.source"),
        "admm_obs_active": latest_get(latest, "obs.active"),
        "admm_obs_apply": latest_get(latest, "obs.apply"),
        "admm_obs_count": latest_get(latest, "obs.count"),
        "admm_obs_eff_cx": latest_get(latest, "obs.effCx"),
        "admm_obs_eff_cy": latest_get(latest, "obs.effCy"),
        "admm_obs_frz_valid": latest_get(latest, "obs.frzValid"),
        "admm_obs_frz_cx": latest_get(latest, "obs.frzCx"),
        "admm_obs_frz_cy": latest_get(latest, "obs.frzCy"),
        "state_x": latest_get(latest, "stateEstimate.x"),
        "state_y": latest_get(latest, "stateEstimate.y"),
        "state_yaw": latest_get(latest, "stateEstimate.yaw"),
    }
    for i in SECTORS:
        row[f"flow_x{i}"] = latest_get(latest, f"flowObsRx.flowX{i}")
        row[f"res_x{i}"] = latest_get(latest, f"flowObsRx.resX{i}")
        row[f"veff{i}"] = latest_get(latest, f"flowObsRx.vEff{i}")
        row[f"range{i}"] = latest_get(latest, f"flowObsRx.range{i}")
        row[f"valid{i}"] = latest_get(latest, f"flowObsRx.valid{i}")
        row[f"bx{i}"] = latest_get(latest, f"flowObsRx.bx{i}")
        row[f"by{i}"] = latest_get(latest, f"flowObsRx.by{i}")
    return row


def as_float(row, key):
    try:
        val = row.get(key, "")
        return float(val) if val != "" else math.nan
    except (TypeError, ValueError):
        return math.nan


def plot_csv(csv_path, png_path):
    import matplotlib.pyplot as plt

    with csv_path.open(newline="") as fp:
        rows = list(csv.DictReader(fp))
    if not rows:
        return

    t = [as_float(r, "t_s") for r in rows]
    fig, axes = plt.subplots(4, 1, figsize=(11, 12), sharex=False)

    axes[0].plot(t, [as_float(r, "rx_ok") for r in rows], label="rx_ok")
    axes[0].plot(t, [as_float(r, "crc_err") for r in rows], label="crc_err")
    axes[0].plot(t, [as_float(r, "bad_rx") for r in rows], label="bad_rx")
    axes[0].set_ylabel("counts")
    axes[0].legend(loc="best")
    axes[0].grid(True, alpha=0.3)

    for i in SECTORS:
        axes[1].plot(t, [as_float(r, f"flow_x{i}") for r in rows], linewidth=1.0, label=f"flow_x{i}")
    axes[1].set_ylabel("rad/s")
    axes[1].legend(ncol=3, fontsize=8, loc="best")
    axes[1].grid(True, alpha=0.3)

    for i in SECTORS:
        vals = []
        for r in rows:
            vals.append(as_float(r, f"range{i}") if as_float(r, f"valid{i}") > 0.5 else math.nan)
        axes[2].plot(t, vals, linewidth=1.0, label=f"range{i}")
    axes[2].plot(t, [as_float(r, "near_range") for r in rows], color="gray", linewidth=1.0, label="near_range")
    axes[2].plot(t, [as_float(r, "obs_range") for r in rows], color="black", linewidth=2.0, label="obs_range")
    axes[2].set_ylabel("range [m]")
    axes[2].legend(ncol=3, fontsize=8, loc="best")
    axes[2].grid(True, alpha=0.3)

    wx = [as_float(r, "obs_wx") for r in rows if as_float(r, "obs_valid") > 0.5]
    wy = [as_float(r, "obs_wy") for r in rows if as_float(r, "obs_valid") > 0.5]
    if wx and wy:
        axes[3].scatter(wx, wy, s=8, alpha=0.35, label="filtered world point")
    cwx = [as_float(r, "cyl_wx") for r in rows if as_float(r, "cyl_valid") > 0.5]
    cwy = [as_float(r, "cyl_wy") for r in rows if as_float(r, "cyl_valid") > 0.5]
    if cwx and cwy:
        axes[3].plot(cwx, cwy, color="black", linewidth=2.0, label="world cylinder tracker")
        axes[3].scatter([cwx[-1]], [cwy[-1]], s=60, color="tab:red", label="final world cylinder")
        axes[3].axvline(cwx[-1], color="tab:red", linewidth=0.8, alpha=0.35)
        axes[3].axhline(cwy[-1], color="tab:red", linewidth=0.8, alpha=0.35)
        last_cyl = next((r for r in reversed(rows) if as_float(r, "cyl_valid") > 0.5), None)
        if last_cyl is not None:
            var_x = max(0.0, as_float(last_cyl, "cyl_var_x"))
            var_y = max(0.0, as_float(last_cyl, "cyl_var_y"))
            cov_xy = as_float(last_cyl, "cyl_cov_xy")
            if math.isfinite(var_x) and math.isfinite(var_y) and math.isfinite(cov_xy):
                trace = var_x + var_y
                disc = max(0.0, (var_x - var_y) * (var_x - var_y) + 4.0 * cov_xy * cov_xy)
                lambda_1 = max(0.0, 0.5 * (trace + math.sqrt(disc)))
                lambda_2 = max(0.0, 0.5 * (trace - math.sqrt(disc)))
                angle_deg = 0.5 * math.degrees(math.atan2(2.0 * cov_xy, var_x - var_y))
                from matplotlib.patches import Ellipse
                ell = Ellipse(
                    (cwx[-1], cwy[-1]),
                    width=4.0 * math.sqrt(lambda_1),
                    height=4.0 * math.sqrt(lambda_2),
                    angle=angle_deg,
                    fill=False,
                    linestyle="--",
                    linewidth=2.0,
                    color="tab:red",
                    label="2-sigma covariance",
                )
                axes[3].add_patch(ell)
    state_x = [as_float(r, "state_x") for r in rows if math.isfinite(as_float(r, "state_x"))]
    state_y = [as_float(r, "state_y") for r in rows if math.isfinite(as_float(r, "state_y"))]
    if state_x and state_y:
        axes[3].plot(state_x, state_y, color="tab:green", linewidth=1.0, alpha=0.8, label="estimated position")
        axes[3].scatter([state_x[-1]], [state_y[-1]], marker="x", color="tab:green", label="final position")
    admm_x = [as_float(r, "admm_obs_eff_cx") for r in rows if as_float(r, "admm_obs_source") > 0.5]
    admm_y = [as_float(r, "admm_obs_eff_cy") for r in rows if as_float(r, "admm_obs_source") > 0.5]
    if admm_x and admm_y:
        axes[3].scatter([admm_x[-1]], [admm_y[-1]], marker="+", s=90, color="tab:purple", label="final ADMM center")
    frz_x = [as_float(r, "admm_obs_frz_cx") for r in rows if as_float(r, "admm_obs_frz_valid") > 0.5]
    frz_y = [as_float(r, "admm_obs_frz_cy") for r in rows if as_float(r, "admm_obs_frz_valid") > 0.5]
    if frz_x and frz_y:
        axes[3].scatter([frz_x[-1]], [frz_y[-1]], marker="s", s=55, facecolors="none", edgecolors="tab:orange", label="frozen center")
    axes[3].set_xlabel("world x [m]")
    axes[3].set_ylabel("world y [m]")
    axes[3].axis("equal")
    axes[3].legend(loc="best")
    axes[3].grid(True, alpha=0.3)

    fig.tight_layout()
    fig.savefig(png_path, dpi=180)


def main():
    args = parse_args()
    out = Path(args.out)
    latest = {}
    configs = []

    Path(args.cache_dir).mkdir(parents=True, exist_ok=True)
    cflib.crtp.init_drivers()

    with SyncCrazyflie(args.uri, cf=Crazyflie(rw_cache=args.cache_dir)) as scf:
        cf = scf.cf

        def on_log(_timestamp, data, _logconf):
            latest.update(data)

        for name, variables in LOG_BLOCKS:
            lc = LogConfig(name=name, period_in_ms=args.period_ms)
            for var, typ in variables:
                lc.add_variable(var, typ)
            cf.log.add_config(lc)
            lc.data_received_cb.add_callback(on_log)
            lc.start()
            configs.append(lc)

        touched_obs_params = args.admm_log_only or args.admm_apply or args.freeze_after_s is not None
        if args.obs_safety is not None:
            set_param(cf, "obs.safety", args.obs_safety, delay=0.05)
        if args.obs_delay_ms is not None:
            set_param(cf, "obs.delayMs", args.obs_delay_ms, delay=0.05)
        if args.freeze_after_s is not None:
            print(f"arming flow obstacle freeze after {args.freeze_after_s:.1f}s")
            set_param(cf, "obs.frzClear", 1, delay=0.05)
            set_param(cf, "obs.frzAfter", int(args.freeze_after_s * 1000.0), delay=0.05)
            set_param(cf, "obs.freeze", 1, delay=0.05)

        if args.admm_log_only:
            print("enabling flow-derived ADMM obstacle in log-only mode")
            set_param(cf, "obs.enable", 1, delay=0.05)
            set_param(cf, "obs.logOnly", 1, delay=0.05)
            set_param(cf, "obs.useFlow", 1, delay=0.05)
        elif args.admm_apply:
            print("enabling flow-derived ADMM obstacle APPLY mode")
            set_param(cf, "obs.enable", 1, delay=0.05)
            set_param(cf, "obs.logOnly", 0, delay=0.05)
            set_param(cf, "obs.useFlow", 1, delay=0.05)

        if not args.no_controller_switch:
            print("switching stabilizer.controller to OOT/6")
            set_param(cf, "stabilizer.controller", 6, delay=0.5)

        row_count = 0
        with out.open("w", newline="") as fp:
            writer = csv.DictWriter(fp, fieldnames=CSV_FIELDS)
            writer.writeheader()
            fp.flush()
            os.fsync(fp.fileno())

            print(f"connected {args.uri}")
            print(f"recording {args.duration:.1f}s to {out}")
            start = time.time()
            next_sample = start
            next_status = start + 1.0
            try:
                while time.time() - start < args.duration:
                    now = time.time()
                    if now >= next_sample:
                        writer.writerow(make_row(latest, start))
                        row_count += 1
                        fp.flush()
                        next_sample += 1.0 / args.sample_hz
                    if now >= next_status:
                        rx_ok = latest.get("flowObsRx.rxOk")
                        obs_valid = latest.get("flowObsRx.obsValid")
                        obs_range = latest.get("flowObsRx.obsRange")
                        cyl_conf = latest.get("flowObsRx.cylConf")
                        map_peak = latest.get("flowObsRx.mapPeak")
                        admm_src = latest.get("obs.source")
                        admm_active = latest.get("obs.active")
                        admm_apply = latest.get("obs.apply")
                        frz_valid = latest.get("obs.frzValid")
                        print(f"rows={row_count} rxOk={rx_ok} obsValid={obs_valid} obsRange={obs_range} cylConf={cyl_conf} mapPeak={map_peak} freeze={frz_valid} admm={admm_src}/{admm_active}/{admm_apply}")
                        next_status += 1.0
                    time.sleep(0.005)
            except KeyboardInterrupt:
                print("stopping early")
            finally:
                fp.flush()
                os.fsync(fp.fileno())
                for lc in configs:
                    try:
                        lc.stop()
                    except Exception:
                        pass
                if touched_obs_params:
                    try:
                        print("restoring obs params to safe defaults")
                        set_param(cf, "obs.enable", 0, delay=0.05)
                        set_param(cf, "obs.logOnly", 1, delay=0.05)
                        set_param(cf, "obs.useFlow", 0, delay=0.05)
                        set_param(cf, "obs.freeze", 0, delay=0.05)
                        set_param(cf, "obs.frzAfter", 0, delay=0.05)
                        set_param(cf, "obs.frzClear", 1, delay=0.05)
                    except Exception:
                        pass
                if not args.no_controller_switch and args.restore_controller >= 0:
                    try:
                        print(f"restoring stabilizer.controller to {args.restore_controller}")
                        set_param(cf, "stabilizer.controller", args.restore_controller, delay=0.1)
                    except Exception:
                        pass

    print(f"wrote {out} ({row_count} rows)")

    if not args.no_plot:
        png = out.with_suffix(".png")
        try:
            plot_csv(out, png)
            print(f"wrote {png}")
        except ImportError:
            print("matplotlib is not installed; CSV was written, skipping plot")


if __name__ == "__main__":
    main()
