#!/usr/bin/env python3
"""Fly a cautious flow-obstacle TinyMPC test.

Sequence:
  1. Reset estimator and take off with PID position setpoints.
  2. Switch to OOT/TinyMPC controller.
  3. Move side-to-side for optical-flow peering while the evidence map builds.
  4. Freeze the flow-derived obstacle after the peer window.
  5. Fly a slow commanded line while logging obstacle/ADMM telemetry.

Ctrl-C attempts a controlled PID landing.
"""

import argparse
import csv
import math
import time
from pathlib import Path

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.utils import uri_helper


DEFAULT_URI = "radio://0/80/2M/E7E7E7E7E8"

LOG_BLOCKS = [
    ("state", 50, [
        ("stateEstimate.x", "float"),
        ("stateEstimate.y", "float"),
        ("stateEstimate.z", "float"),
        ("stateEstimate.yaw", "float"),
    ]),
    ("pm", 100, [("pm.vbat", "float")]),
    ("obs_flags", 50, [
        ("obs.source", "uint8_t"),
        ("obs.active", "uint8_t"),
        ("obs.apply", "uint8_t"),
        ("obs.count", "uint8_t"),
        ("obs.frzValid", "uint8_t"),
    ]),
    ("obs_center", 50, [
        ("obs.effCx", "float"),
        ("obs.effCy", "float"),
        ("obs.effRad", "float"),
        ("obs.flowCf", "float"),
        ("obs.clear", "float"),
    ]),
    ("obs_freeze", 50, [
        ("obs.frzCx", "float"),
        ("obs.frzCy", "float"),
        ("obs.frzCf", "float"),
    ]),
    ("flow_map", 100, [
        ("flowObsRx.rxOk", "uint32_t"),
        ("flowObsRx.mapPeak", "float"),
        ("flowObsRx.mapActive", "uint8_t"),
        ("flowObsRx.cylValid", "float"),
        ("flowObsRx.cylConf", "float"),
    ]),
    ("flow_cyl", 100, [
        ("flowObsRx.cylWx", "float"),
        ("flowObsRx.cylWy", "float"),
        ("flowObsRx.cylVarX", "float"),
        ("flowObsRx.cylVarY", "float"),
    ]),
]


CSV_FIELDS = [
    "host_time",
    "phase",
    "cmd_x",
    "cmd_y",
    "cmd_z",
    "state_x",
    "state_y",
    "state_z",
    "state_yaw",
    "vbat",
    "rx_ok",
    "map_peak",
    "map_active",
    "flow_cyl_valid",
    "flow_cyl_conf",
    "flow_cyl_wx",
    "flow_cyl_wy",
    "flow_cyl_var_x",
    "flow_cyl_var_y",
    "obs_source",
    "obs_active",
    "obs_apply",
    "obs_count",
    "obs_eff_cx",
    "obs_eff_cy",
    "obs_eff_rad",
    "obs_flow_conf",
    "obs_clearance",
    "obs_frz_valid",
    "obs_frz_cx",
    "obs_frz_cy",
    "obs_frz_conf",
]


def parse_args():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--uri", default=uri_helper.uri_from_env(default=DEFAULT_URI))
    p.add_argument("--out", default="flow_obstacle_flight.csv")
    p.add_argument("--height", type=float, default=0.5)
    p.add_argument("--takeoff-s", type=float, default=2.0)
    p.add_argument("--settle-s", type=float, default=1.0)
    p.add_argument("--peer-s", type=float, default=10.0)
    p.add_argument("--freeze-after-s", type=float, default=None,
                   help="Latch obstacle this many seconds after switching to TinyMPC. Default: 80%% of peer-s.")
    p.add_argument("--peer-amp", type=float, default=0.25, help="Side-to-side peering amplitude [m].")
    p.add_argument("--peer-period-s", type=float, default=3.0)
    p.add_argument("--flow-start-timeout-s", type=float, default=5.0,
                   help="After switching to TinyMPC, wait this long for flow rxOk to increment before peering.")
    p.add_argument("--skip-flow-warmup", action="store_true",
                   help="Skip the pre-arm OOT switch used to verify AI-deck flow packets on the ground.")
    p.add_argument("--run-s", type=float, default=5.0)
    p.add_argument("--land-s", type=float, default=2.0)
    p.add_argument("--start-x", type=float, default=0.0)
    p.add_argument("--start-y", type=float, default=0.0)
    p.add_argument("--goal-x", type=float, default=1.0)
    p.add_argument("--goal-y", type=float, default=0.0)
    p.add_argument("--yaw-deg", type=float, default=0.0)
    p.add_argument("--obs-safety", type=float, default=0.25)
    p.add_argument("--obs-radius", type=float, default=0.25)
    p.add_argument("--obs-height", type=float, default=1.0)
    p.add_argument("--obs-act-margin", type=float, default=1000.0)
    p.add_argument("--obs-k-start", type=int, default=1)
    p.add_argument("--min-vbat", type=float, default=3.55)
    p.add_argument("--log-only", action="store_true", help="Peer/freeze/log without applying ADMM constraints.")
    p.add_argument("--run-without-freeze", action="store_true",
                   help="If freeze fails, fly the run leg with obs disabled instead of aborting.")
    p.add_argument("--no-reset-estimator", action="store_true")
    p.add_argument("--cache-dir", default=".cf_cache")
    return p.parse_args()


def set_param(cf, name, value, delay=0.03):
    cf.param.set_value(name, str(value))
    time.sleep(delay)


def latest_get(latest, key, default=None):
    return latest.get(key, default)


def make_row(phase, latest, cmd_x, cmd_y, cmd_z):
    return {
        "host_time": time.time(),
        "phase": phase,
        "cmd_x": cmd_x,
        "cmd_y": cmd_y,
        "cmd_z": cmd_z,
        "state_x": latest_get(latest, "stateEstimate.x"),
        "state_y": latest_get(latest, "stateEstimate.y"),
        "state_z": latest_get(latest, "stateEstimate.z"),
        "state_yaw": latest_get(latest, "stateEstimate.yaw"),
        "vbat": latest_get(latest, "pm.vbat"),
        "rx_ok": latest_get(latest, "flowObsRx.rxOk"),
        "map_peak": latest_get(latest, "flowObsRx.mapPeak"),
        "map_active": latest_get(latest, "flowObsRx.mapActive"),
        "flow_cyl_valid": latest_get(latest, "flowObsRx.cylValid"),
        "flow_cyl_conf": latest_get(latest, "flowObsRx.cylConf"),
        "flow_cyl_wx": latest_get(latest, "flowObsRx.cylWx"),
        "flow_cyl_wy": latest_get(latest, "flowObsRx.cylWy"),
        "flow_cyl_var_x": latest_get(latest, "flowObsRx.cylVarX"),
        "flow_cyl_var_y": latest_get(latest, "flowObsRx.cylVarY"),
        "obs_source": latest_get(latest, "obs.source"),
        "obs_active": latest_get(latest, "obs.active"),
        "obs_apply": latest_get(latest, "obs.apply"),
        "obs_count": latest_get(latest, "obs.count"),
        "obs_eff_cx": latest_get(latest, "obs.effCx"),
        "obs_eff_cy": latest_get(latest, "obs.effCy"),
        "obs_eff_rad": latest_get(latest, "obs.effRad"),
        "obs_flow_conf": latest_get(latest, "obs.flowCf"),
        "obs_clearance": latest_get(latest, "obs.clear"),
        "obs_frz_valid": latest_get(latest, "obs.frzValid"),
        "obs_frz_cx": latest_get(latest, "obs.frzCx"),
        "obs_frz_cy": latest_get(latest, "obs.frzCy"),
        "obs_frz_conf": latest_get(latest, "obs.frzCf"),
    }


def stream_position(cf, rows, latest, seconds, x, y, z, yaw_deg, phase):
    steps = max(1, int(seconds * 50.0))
    for _ in range(steps):
        cf.commander.send_position_setpoint(x, y, z, yaw_deg)
        rows.append(make_row(phase, latest, x, y, z))
        time.sleep(0.02)


def stream_line(cf, rows, latest, seconds, x0, y0, z0, x1, y1, z1, yaw_deg, phase):
    steps = max(1, int(seconds * 50.0))
    denom = max(1, steps - 1)
    for step in range(steps):
        a = step / denom
        x = x0 + a * (x1 - x0)
        y = y0 + a * (y1 - y0)
        z = z0 + a * (z1 - z0)
        cf.commander.send_position_setpoint(x, y, z, yaw_deg)
        rows.append(make_row(phase, latest, x, y, z))
        time.sleep(0.02)


def stream_peer(cf, rows, latest, seconds, x, y0, z, yaw_deg, amp, period_s):
    steps = max(1, int(seconds * 50.0))
    for step in range(steps):
        t = step * 0.02
        y = y0 + amp * math.sin(2.0 * math.pi * t / period_s)
        cf.commander.send_position_setpoint(x, y, z, yaw_deg)
        rows.append(make_row("peer", latest, x, y, z))
        time.sleep(0.02)


def wait_for_flow(cf, rows, latest, seconds, x, y, z, yaw_deg):
    start_rx = latest_get(latest, "flowObsRx.rxOk", 0) or 0
    steps = max(1, int(seconds * 50.0))
    for _ in range(steps):
        cf.commander.send_position_setpoint(x, y, z, yaw_deg)
        rows.append(make_row("wait_flow", latest, x, y, z))
        rx_now = latest_get(latest, "flowObsRx.rxOk", 0) or 0
        if rx_now > start_rx:
            return True
        time.sleep(0.02)
    return False


def land_pid(cf, rows, latest, seconds, z_final=0.05):
    set_param(cf, "stabilizer.controller", 1, delay=0.1)
    x = latest_get(latest, "stateEstimate.x", 0.0) or 0.0
    y = latest_get(latest, "stateEstimate.y", 0.0) or 0.0
    z = latest_get(latest, "stateEstimate.z", 0.4) or 0.4
    yaw = latest_get(latest, "stateEstimate.yaw", 0.0) or 0.0
    stream_line(cf, rows, latest, seconds, x, y, z, x, y, z_final, yaw, "land")


def main():
    args = parse_args()
    rows = []
    latest = {}
    configs = []
    out = Path(args.out)
    abort_after_cleanup = False

    Path(args.cache_dir).mkdir(parents=True, exist_ok=True)
    cflib.crtp.init_drivers()

    with SyncCrazyflie(args.uri, cf=Crazyflie(rw_cache=args.cache_dir)) as scf:
        cf = scf.cf

        def on_log(_timestamp, data, _logconf):
            latest.update(data)

        for name, period_ms, variables in LOG_BLOCKS:
            lc = LogConfig(name=name, period_in_ms=period_ms)
            for var, typ in variables:
                lc.add_variable(var, typ)
            cf.log.add_config(lc)
            lc.data_received_cb.add_callback(on_log)
            lc.start()
            configs.append(lc)

        try:
            time.sleep(1.0)
            vbat = latest_get(latest, "pm.vbat")
            if vbat is not None and vbat < args.min_vbat:
                raise SystemExit(f"ABORT: battery {vbat:.2f} V < {args.min_vbat:.2f} V")

            print(f"connected {args.uri}")
            if not args.no_reset_estimator:
                set_param(cf, "kalman.resetEstimation", 1)
                time.sleep(0.1)
                set_param(cf, "kalman.resetEstimation", 0)
                time.sleep(2.0)

            set_param(cf, "commander.enHighLevel", 0)
            set_param(cf, "stabilizer.controller", 1)
            set_param(cf, "obs.enable", 0)
            set_param(cf, "obs.logOnly", 1)
            set_param(cf, "obs.useFlow", 0)
            set_param(cf, "obs.freeze", 0)
            set_param(cf, "obs.frzClear", 1)
            set_param(cf, "obs.radius", args.obs_radius)
            set_param(cf, "obs.height", args.obs_height)
            set_param(cf, "obs.safety", args.obs_safety)
            set_param(cf, "obs.actMarg", args.obs_act_margin)
            set_param(cf, "obs.kStart", args.obs_k_start)
            freeze_after_s = args.freeze_after_s
            if freeze_after_s is None:
                freeze_after_s = args.peer_s
            set_param(cf, "obs.frzAfter", int(freeze_after_s * 1000.0))

            if not args.skip_flow_warmup:
                print("preflight AI-deck flow warmup on ground")
                set_param(cf, "stabilizer.controller", 6, delay=0.2)
                if not wait_for_flow(cf, rows, latest, args.flow_start_timeout_s,
                                     args.start_x, args.start_y, 0.05,
                                     args.yaw_deg):
                    print("ABORT: no AI-deck flow packets during preflight warmup")
                    return
                set_param(cf, "stabilizer.controller", 1, delay=0.2)

            cf.platform.send_arming_request(True)
            time.sleep(0.5)

            print("takeoff with PID")
            stream_line(cf, rows, latest, args.takeoff_s,
                        args.start_x, args.start_y, 0.05,
                        args.start_x, args.start_y, args.height,
                        args.yaw_deg, "takeoff")
            stream_position(cf, rows, latest, args.settle_s,
                            args.start_x, args.start_y, args.height,
                            args.yaw_deg, "settle")

            print("switching to TinyMPC and peering side-to-side")
            print(f"peer command: {args.peer_s:.1f}s side-to-side, freeze after {freeze_after_s:.1f}s")
            set_param(cf, "obs.enable", 1)
            set_param(cf, "obs.logOnly", 1)
            set_param(cf, "obs.useFlow", 1)
            set_param(cf, "obs.freeze", 1)
            set_param(cf, "stabilizer.controller", 6, delay=0.2)
            print("waiting for AI-deck flow packets")
            if not wait_for_flow(cf, rows, latest, args.flow_start_timeout_s,
                                 args.start_x, args.start_y, args.height,
                                 args.yaw_deg):
                print("ABORT: no AI-deck flow packets after switching to TinyMPC")
                abort_after_cleanup = True
                land_pid(cf, rows, latest, args.land_s)

            if not abort_after_cleanup:
                stream_peer(cf, rows, latest, args.peer_s,
                            args.start_x, args.start_y, args.height,
                            args.yaw_deg, args.peer_amp, args.peer_period_s)

            frz_valid = latest_get(latest, "obs.frzValid", 0) or 0
            source = latest_get(latest, "obs.source", 0) or 0
            print(f"freeze status: frzValid={frz_valid} source={source} "
                  f"center=({latest_get(latest, 'obs.frzCx')}, {latest_get(latest, 'obs.frzCy')})")
            print(f"map status: rxOk={latest_get(latest, 'flowObsRx.rxOk')} "
                  f"mapPeak={latest_get(latest, 'flowObsRx.mapPeak')} "
                  f"cylValid={latest_get(latest, 'flowObsRx.cylValid')} "
                  f"cylConf={latest_get(latest, 'flowObsRx.cylConf')}")
            if abort_after_cleanup:
                pass
            elif not args.log_only and int(frz_valid) == 0 and not args.run_without_freeze:
                print("ABORT: obstacle did not freeze; refusing to fly run leg")
                abort_after_cleanup = True
                land_pid(cf, rows, latest, args.land_s)
            elif not args.log_only and int(frz_valid) == 0 and args.run_without_freeze:
                print("freeze failed: disabling obs and flying run leg without obstacle constraints")
                set_param(cf, "obs.enable", 0)
                set_param(cf, "obs.logOnly", 1)
                set_param(cf, "obs.useFlow", 0)
            elif not args.log_only:
                set_param(cf, "obs.logOnly", 0)
                stream_position(cf, rows, latest, 0.5,
                                args.start_x, args.start_y, args.height,
                                args.yaw_deg, "apply_settle")

            if not abort_after_cleanup:
                print("flying run leg")
                stream_line(cf, rows, latest, args.run_s,
                            args.start_x, args.start_y, args.height,
                            args.goal_x, args.goal_y, args.height,
                            args.yaw_deg, "run")

                print("landing")
                land_pid(cf, rows, latest, args.land_s)
        except KeyboardInterrupt:
            print("\nabort requested, landing")
            land_pid(cf, rows, latest, args.land_s)
        finally:
            for action in (
                lambda: cf.commander.send_stop_setpoint(),
                lambda: cf.platform.send_arming_request(False),
                lambda: set_param(cf, "obs.enable", 0, delay=0.01),
                lambda: set_param(cf, "obs.logOnly", 1, delay=0.01),
                lambda: set_param(cf, "obs.useFlow", 0, delay=0.01),
                lambda: set_param(cf, "obs.freeze", 0, delay=0.01),
                lambda: set_param(cf, "obs.frzClear", 1, delay=0.01),
            ):
                try:
                    action()
                except Exception:
                    pass
            for lc in configs:
                try:
                    lc.stop()
                except Exception:
                    pass

    with out.open("w", newline="") as fp:
        writer = csv.DictWriter(fp, fieldnames=CSV_FIELDS)
        writer.writeheader()
        writer.writerows(rows)
    print(f"wrote {out} ({len(rows)} rows)")


if __name__ == "__main__":
    main()
