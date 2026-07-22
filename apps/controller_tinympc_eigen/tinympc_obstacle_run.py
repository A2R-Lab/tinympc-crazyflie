#!/usr/bin/env python3
"""Run and log the modeled-cylinder TinyMPC obstacle avoidance test.

The firmware side assumes a static cylinder and exposes it through the `obs` param/log
group. This script sets those params, flies a straight goal across the cylinder region,
and writes state + obstacle telemetry to CSV.
"""

import argparse
import csv
import time
from pathlib import Path

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.utils import uri_helper


DEFAULT_URI = "radio://0/80/2M/E7E7E7E7E8"

LOG_BLOCKS = [
    (
        "state",
        20,
        [
            ("stateEstimate.x", "float"),
            ("stateEstimate.y", "float"),
            ("stateEstimate.z", "float"),
            ("stateEstimate.yaw", "float"),
        ],
    ),
    (
        "obs_flags",
        20,
        [
            ("obs.active", "uint8_t"),
            ("obs.apply", "uint8_t"),
            ("obs.count", "uint8_t"),
            ("obs.firstK", "uint8_t"),
        ],
    ),
    (
        "obs_plane",
        20,
        [
            ("obs.a0", "float"),
            ("obs.a1", "float"),
            ("obs.a2", "float"),
            ("obs.b", "float"),
        ],
    ),
    (
        "obs_metric",
        20,
        [
            ("obs.margin", "float"),
            ("obs.viol", "float"),
            ("obs.clear", "float"),
        ],
    ),
    (
        "obs_mpc",
        50,
        [
            ("obs.mpcUs", "uint32_t"),
            ("obs.iter", "uint8_t"),
        ],
    ),
    (
        "pm",
        100,
        [
            ("pm.vbat", "float"),
        ],
    ),
]


def parse_args():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--uri", default=uri_helper.uri_from_env(default=DEFAULT_URI))
    p.add_argument("--out", default="tinympc_obstacle_run.csv")
    p.add_argument("--height", type=float, default=0.5)
    p.add_argument("--takeoff-s", type=float, default=2.0)
    p.add_argument("--settle-s", type=float, default=1.5)
    p.add_argument("--run-s", type=float, default=8.0)
    p.add_argument("--land-s", type=float, default=2.0)
    p.add_argument("--min-vbat", type=float, default=3.55)
    p.add_argument("--start-x", type=float, default=0.0)
    p.add_argument("--start-y", type=float, default=0.0)
    p.add_argument("--goal-x", type=float, default=2.0)
    p.add_argument("--goal-y", type=float, default=0.0)
    p.add_argument("--obs-x", type=float, default=1.0)
    p.add_argument("--obs-y", type=float, default=0.0)
    p.add_argument("--obs-z", type=float, default=0.5)
    p.add_argument("--obs-radius", type=float, default=0.25)
    p.add_argument("--obs-height", type=float, default=1.0)
    p.add_argument("--obs-safety", type=float, default=0.10)
    p.add_argument("--obs-act-margin", type=float, default=0.30)
    p.add_argument("--obs-side", type=float, default=1.0)
    p.add_argument("--obs-delay-ms", type=int, default=1000)
    p.add_argument("--obs-k-start", type=int, default=1)
    p.add_argument("--log-only", action="store_true", help="Compute/log obstacle planes without applying them.")
    p.add_argument("--no-reset-estimator", action="store_true")
    p.add_argument("--cache-dir", default=".cf_cache")
    return p.parse_args()


def set_param(cf, name, value, delay=0.03):
    cf.param.set_value(name, str(value))
    time.sleep(delay)


def stream_position(cf, seconds, x, y, z, yaw_deg, rows, latest, leg):
    deadline = time.time() + seconds
    while time.time() < deadline:
        rows.append(make_row(leg, latest))
        cf.commander.send_position_setpoint(x, y, z, yaw_deg)
        time.sleep(0.02)


def stream_line(cf, seconds, x0, y0, z0, x1, y1, z1, yaw_deg, rows, latest, leg):
    start = time.time()
    while True:
        t = time.time() - start
        if t >= seconds:
            break
        a = min(1.0, t / seconds)
        x = x0 + a * (x1 - x0)
        y = y0 + a * (y1 - y0)
        z = z0 + a * (z1 - z0)
        rows.append(make_row(leg, latest))
        cf.commander.send_position_setpoint(x, y, z, yaw_deg)
        time.sleep(0.02)


def make_row(leg, latest):
    now = time.time()
    return [
        now,
        leg,
        latest.get("stateEstimate.x"),
        latest.get("stateEstimate.y"),
        latest.get("stateEstimate.z"),
        latest.get("stateEstimate.yaw"),
        latest.get("obs.active"),
        latest.get("obs.apply"),
        latest.get("obs.count"),
        latest.get("obs.firstK"),
        latest.get("obs.a0"),
        latest.get("obs.a1"),
        latest.get("obs.a2"),
        latest.get("obs.b"),
        latest.get("obs.margin"),
        latest.get("obs.viol"),
        latest.get("obs.clear"),
        latest.get("obs.mpcUs"),
        latest.get("obs.iter"),
        latest.get("pm.vbat"),
    ]


def main():
    args = parse_args()
    rows = []
    latest = {}
    configs = []
    console = []

    cache_dir = Path(args.cache_dir)
    cache_dir.mkdir(parents=True, exist_ok=True)

    cflib.crtp.init_drivers()
    with SyncCrazyflie(args.uri, cf=Crazyflie(rw_cache=str(cache_dir))) as scf:
        cf = scf.cf
        cf.console.receivedChar.add_callback(lambda text: console.append(text))

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
            time.sleep(0.8)
            vbat = latest.get("pm.vbat")
            if vbat is not None and vbat < args.min_vbat:
                raise SystemExit(f"ABORT: battery {vbat:.2f} V < {args.min_vbat:.2f} V")

            print(f"connected {args.uri}")
            if not args.no_reset_estimator:
                set_param(cf, "kalman.resetEstimation", 1)
                time.sleep(0.1)
                set_param(cf, "kalman.resetEstimation", 0)
                time.sleep(2.0)

            set_param(cf, "obs.enable", 1)
            set_param(cf, "obs.logOnly", 1 if args.log_only else 0)
            set_param(cf, "obs.cx", args.obs_x)
            set_param(cf, "obs.cy", args.obs_y)
            set_param(cf, "obs.cz", args.obs_z)
            set_param(cf, "obs.radius", args.obs_radius)
            set_param(cf, "obs.height", args.obs_height)
            set_param(cf, "obs.safety", args.obs_safety)
            set_param(cf, "obs.actMarg", args.obs_act_margin)
            set_param(cf, "obs.side", args.obs_side)
            set_param(cf, "obs.delayMs", args.obs_delay_ms)
            set_param(cf, "obs.kStart", args.obs_k_start)

            set_param(cf, "commander.enHighLevel", 0)
            set_param(cf, "stabilizer.controller", 1)
            cf.platform.send_arming_request(True)
            time.sleep(1.0)

            print("takeoff with PID")
            stream_line(
                cf,
                args.takeoff_s,
                args.start_x,
                args.start_y,
                0.05,
                args.start_x,
                args.start_y,
                args.height,
                0.0,
                rows,
                latest,
                "takeoff",
            )
            stream_position(cf, args.settle_s, args.start_x, args.start_y, args.height, 0.0, rows, latest, "settle")

            print("switching to controller 6")
            stream_position(cf, 0.5, args.start_x, args.start_y, args.height, 0.0, rows, latest, "pre_switch")
            set_param(cf, "stabilizer.controller", 6)
            stream_position(cf, 1.0, args.start_x, args.start_y, args.height, 0.0, rows, latest, "mpc_settle")

            print("running straight obstacle test")
            stream_line(
                cf,
                args.run_s,
                args.start_x,
                args.start_y,
                args.height,
                args.goal_x,
                args.goal_y,
                args.height,
                0.0,
                rows,
                latest,
                "run",
            )

            print("landing")
            set_param(cf, "stabilizer.controller", 1)
            stream_line(
                cf,
                args.land_s,
                latest.get("stateEstimate.x", args.goal_x),
                latest.get("stateEstimate.y", args.goal_y),
                latest.get("stateEstimate.z", args.height),
                latest.get("stateEstimate.x", args.goal_x),
                latest.get("stateEstimate.y", args.goal_y),
                0.05,
                latest.get("stateEstimate.yaw", 0.0),
                rows,
                latest,
                "land",
            )
        except KeyboardInterrupt:
            print("\nabort requested, landing")
            set_param(cf, "stabilizer.controller", 1)
            stream_line(
                cf,
                args.land_s,
                latest.get("stateEstimate.x", args.start_x),
                latest.get("stateEstimate.y", args.start_y),
                latest.get("stateEstimate.z", args.height),
                latest.get("stateEstimate.x", args.start_x),
                latest.get("stateEstimate.y", args.start_y),
                0.05,
                latest.get("stateEstimate.yaw", 0.0),
                rows,
                latest,
                "abort_land",
            )
        finally:
            for action in (
                lambda: cf.commander.send_stop_setpoint(),
                lambda: cf.platform.send_arming_request(False),
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

    header = [
        "host_time",
        "leg",
        "x",
        "y",
        "z",
        "yaw_deg",
        "obs_active",
        "obs_apply",
        "obs_count",
        "obs_first_k",
        "obs_a0",
        "obs_a1",
        "obs_a2",
        "obs_b",
        "obs_margin",
        "obs_violation",
        "obs_clearance",
        "obs_mpc_us",
        "obs_iter",
        "vbat",
    ]
    out = Path(args.out)
    with out.open("w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(header)
        writer.writerows(rows)
    print(f"wrote {out} ({len(rows)} rows)")
    if console:
        console_path = out.with_suffix(".console.txt")
        console_path.write_text("".join(console))
        print(f"wrote {console_path}")


if __name__ == "__main__":
    main()
