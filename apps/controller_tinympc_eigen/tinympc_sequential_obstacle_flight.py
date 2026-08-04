#!/usr/bin/env python3
"""Fly a cautious straight-line test using GAP8 four-direction clearances.

The script verifies live CRC-valid UART packets before arming, takes off through
controller 6's PID passthrough, then enables TinyMPC output for the straight leg.
Ctrl-C or any vision/telemetry safety fault attempts a PID-passthrough landing.
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
FIELDS = [
    "host_time", "phase", "cmd_x", "cmd_y", "cmd_z",
    "stateEstimate.x", "stateEstimate.y", "stateEstimate.z",
    "stateEstimate.vx", "stateEstimate.vy", "pm.vbat",
    "seqRx.rxOk", "seqRx.crcErr", "seqRx.badRx", "seqRx.invalid",
    "seqRx.duplicate", "seqRx.seqGap", "gate8.raw", "gate8.qDrop",
    "seqRx.d0", "seqRx.d1", "seqRx.d2", "seqRx.d3",
    "seqRx.c0", "seqRx.c1", "seqRx.c2", "seqRx.c3",
    "seqAvoid.valid", "seqAvoid.stop", "seqAvoid.mask",
    "seqAvoid.chosen", "seqAvoid.side", "seqAvoid.sideVotes",
    "seqAvoid.nCstr", "seqAvoid.ageMs",
    "seqAvoid.routePhase", "seqAvoid.clearVotes", "seqAvoid.replans",
    "seqAvoid.grace", "seqAvoid.graceAge",
    "seqAvoid.waypointX", "seqAvoid.waypointY", "seqAvoid.waypointZ",
    "seqAvoid.pathOffX", "seqAvoid.pathOffY",
    "seqAvoid.pressure", "seqAvoid.maxSlack",
    "seqAvoid.eff0", "seqAvoid.eff1", "seqAvoid.eff2", "seqAvoid.eff3",
    "obs.mpcUs", "obs.iter", "obs.status", "obs.priRes", "obs.duaRes",
    "obs.healthHold", "obs.healthFaults",
]
LOG_BLOCKS = [
    ("seq_state", 50, [("stateEstimate.x", "float"),
                       ("stateEstimate.y", "float"),
                       ("stateEstimate.z", "float"),
                       ("pm.vbat", "float")]),
    ("seq_motion", 50, [("stateEstimate.vx", "float"),
                        ("stateEstimate.vy", "float")]),
    ("seq_link", 100, [("seqRx.rxOk", "uint32_t"),
                       ("seqRx.crcErr", "uint32_t"),
                       ("seqRx.badRx", "uint32_t"),
                       ("seqRx.invalid", "uint32_t")]),
    ("seq_link_detail", 100, [("seqRx.duplicate", "uint32_t"),
                               ("seqRx.seqGap", "uint32_t"),
                               ("gate8.raw", "uint32_t"),
                               ("gate8.qDrop", "uint32_t")]),
    ("seq_raw_distance", 50, [("seqRx.d0", "float"),
                               ("seqRx.d1", "float"),
                               ("seqRx.d2", "float"),
                               ("seqRx.d3", "float")]),
    ("seq_raw_conf", 50, [("seqRx.c0", "float"),
                           ("seqRx.c1", "float"),
                           ("seqRx.c2", "float"),
                           ("seqRx.c3", "float")]),
    ("seq_flags", 50, [("seqAvoid.valid", "uint8_t"),
                       ("seqAvoid.stop", "uint8_t"),
                       ("seqAvoid.mask", "uint8_t"),
                       ("seqAvoid.chosen", "int8_t"),
                       ("seqAvoid.side", "int8_t"),
                       ("seqAvoid.sideVotes", "uint8_t"),
                       ("seqAvoid.nCstr", "uint8_t"),
                       ("seqAvoid.ageMs", "uint32_t")]),
    ("seq_metric", 50, [("seqAvoid.pressure", "float"),
                        ("seqAvoid.maxSlack", "float")]),
    ("seq_route", 50, [("seqAvoid.routePhase", "uint8_t"),
                        ("seqAvoid.clearVotes", "uint8_t"),
                        ("seqAvoid.replans", "uint32_t"),
                        ("seqAvoid.grace", "uint8_t"),
                        ("seqAvoid.graceAge", "uint32_t")]),
    ("seq_waypoint", 50, [("seqAvoid.waypointX", "float"),
                           ("seqAvoid.waypointY", "float"),
                           ("seqAvoid.waypointZ", "float"),
                           ("seqAvoid.pathOffX", "float"),
                           ("seqAvoid.pathOffY", "float")]),
    ("seq_eff", 100, [("seqAvoid.eff0", "float"),
                      ("seqAvoid.eff1", "float"),
                      ("seqAvoid.eff2", "float"),
                      ("seqAvoid.eff3", "float")]),
    ("seq_solver", 100, [("obs.mpcUs", "uint32_t"),
                           ("obs.iter", "uint8_t"),
                           ("obs.status", "int8_t"),
                           ("obs.priRes", "float"),
                           ("obs.duaRes", "float"),
                           ("obs.healthHold", "uint8_t"),
                           ("obs.healthFaults", "uint32_t")]),
]


def arguments():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--uri", default=uri_helper.uri_from_env(default=DEFAULT_URI))
    parser.add_argument("--out", default="sequential_obstacle_flight.csv")
    parser.add_argument("--cache-dir", default=".cf_cache")
    parser.add_argument("--height", type=float, default=0.5)
    parser.add_argument("--takeoff-s", type=float, default=2.5)
    parser.add_argument("--settle-s", type=float, default=1.5)
    parser.add_argument("--run-s", type=float, default=8.0)
    parser.add_argument("--land-s", type=float, default=2.5)
    parser.add_argument("--start-x", type=float, default=0.0)
    parser.add_argument("--start-y", type=float, default=0.0)
    parser.add_argument("--goal-x", type=float, default=1.5)
    parser.add_argument("--goal-y", type=float, default=0.0)
    parser.add_argument("--yaw-deg", type=float, default=0.0)
    parser.add_argument("--min-vbat", type=float, default=3.55)
    parser.add_argument("--confidence-min", type=float, default=0.0)
    parser.add_argument("--safe-min", type=float, default=0.22,
                        help="Raw directional clearance threshold for open versus blocked.")
    parser.add_argument("--max-age-ms", type=int, default=250)
    parser.add_argument("--vision-grace-ms", type=int, default=300,
                        help="Legacy compatibility option; classifier mode does not use grace.")
    parser.add_argument("--ref-shift", type=float, default=0.35)
    parser.add_argument("--trigger", type=float, default=1.2)
    parser.add_argument("--side-min", type=float, default=0.35,
                        help="Legacy compatibility option; classifier waypoint mode ignores it.")
    parser.add_argument("--side-votes", type=int, default=3,
                        help="Legacy compatibility option; side selection is immediate.")
    parser.add_argument("--forward-min", type=float, default=0.35,
                        help="Legacy compatibility option; use --safe-min instead.")
    parser.add_argument("--forward-step", type=float, default=0.40,
                        help="Legacy compatibility option; no forward avoidance stage is used.")
    parser.add_argument("--pass-distance", type=float, default=0.80,
                        help="Legacy compatibility option; release uses three open slices.")
    parser.add_argument("--waypoint-tolerance", type=float, default=0.10,
                        help="Distance at which the next route stage is planned.")
    parser.add_argument("--clear-votes", type=int, default=3,
                        help="Legacy compatibility option; release uses one fresh classification.")
    parser.add_argument("--max-route-lateral", type=float, default=0.75,
                        help="Maximum lateral route displacement before holding.")
    parser.add_argument("--probe-progress", type=float, default=0.08,
                        help="Legacy compatibility option; forward probing is disabled.")
    parser.add_argument("--probe-votes", type=int, default=8,
                        help="Legacy compatibility option; forward probing is disabled.")
    parser.add_argument("--max-pri-res", type=float, default=0.50,
                        help="Hold when the ADMM primal residual exceeds this value (0 disables).")
    parser.add_argument("--max-dua-res", type=float, default=300.0,
                        help="Hold when the ADMM dual residual exceeds this value (0 disables).")
    parser.add_argument("--max-solve-us", type=int, default=20000,
                        help="Hold when one MPC solve exceeds this budget in microseconds (0 disables).")
    parser.add_argument("--zero-margins", action="store_true",
                        help="Legacy compatibility option; classifier mode applies no metric margins.")
    parser.add_argument("--max-lateral-error", type=float, default=1.0)
    parser.add_argument("--log-only", action="store_true",
                        help="Legacy compatibility option; sequential vision never applies ADMM planes.")
    parser.add_argument("--no-reset-estimator", action="store_true")
    return parser.parse_args()


def set_param(cf, name, value, delay=0.04):
    cf.param.set_value(name, str(value))
    time.sleep(delay)


def record(rows, latest, phase, command):
    row = {field: latest.get(field) for field in FIELDS}
    row.update(host_time=time.time(), phase=phase,
               cmd_x=command[0], cmd_y=command[1], cmd_z=command[2])
    rows.append(row)


def link_diagnostics(latest):
    """Return the counters that identify why the sequential preflight failed."""
    names = ("seqRx.rxOk", "seqRx.crcErr", "seqRx.badRx", "seqRx.invalid",
             "seqRx.duplicate", "seqRx.seqGap", "gate8.raw", "gate8.qDrop")
    return ", ".join(f"{name}={latest.get(name, 'n/a')}" for name in names)


def safety_check(args, latest, last_log_time, require_vision, phase):
    if time.monotonic() - last_log_time[0] > 0.6:
        raise RuntimeError("Crazyflie log link is stale")
    battery = latest.get("pm.vbat")
    if battery is not None and battery < args.min_vbat:
        raise RuntimeError(f"battery {battery:.2f} V is below {args.min_vbat:.2f} V")
    z = latest.get("stateEstimate.z")
    if phase in ("settle", "run") and z is not None and (
            z < 0.20 or z > max(1.0, 1.7 * args.height)):
        raise RuntimeError(f"unsafe height estimate {z:.2f} m")
    y = latest.get("stateEstimate.y")
    if y is not None and abs(y - args.start_y) > args.max_lateral_error:
        raise RuntimeError(f"lateral excursion {y - args.start_y:.2f} m")
    if require_vision:
        if latest.get("obs.healthHold", 0):
            raise RuntimeError("MPC solver health guard entered position hold")
        age = latest.get("seqAvoid.ageMs", args.max_age_ms + 1)
        if age > args.max_age_ms:
            raise RuntimeError(f"vision packet is stale ({age} ms)")
        if latest.get("seqAvoid.stop", 1):
            raise RuntimeError("vision has no open route direction; firmware entered hold")
        if latest.get("seqAvoid.routePhase", 0) == 3:
            raise RuntimeError("safe route reached its lateral limit and entered hold")


def stream(cf, args, rows, latest, last_log_time, phase, duration,
           start, target, require_vision=False):
    begun = time.monotonic()
    while True:
        elapsed = time.monotonic() - begun
        if elapsed >= duration:
            return
        alpha = min(1.0, elapsed / max(duration, 1e-3))
        command = tuple(a + alpha * (b - a) for a, b in zip(start, target))
        safety_check(args, latest, last_log_time, require_vision, phase)
        cf.commander.send_position_setpoint(*command, args.yaw_deg)
        record(rows, latest, phase, command)
        time.sleep(0.02)


def main():
    args = arguments()
    latest, rows, configs = {}, [], []
    last_log_time = [time.monotonic()]
    cache = Path(args.cache_dir)
    cache.mkdir(parents=True, exist_ok=True)
    cflib.crtp.init_drivers()

    with SyncCrazyflie(args.uri, cf=Crazyflie(rw_cache=str(cache))) as scf:
        cf = scf.cf

        def on_log(_timestamp, data, _config):
            latest.update(data)
            last_log_time[0] = time.monotonic()

        for name, period, variables in LOG_BLOCKS:
            config = LogConfig(name=name, period_in_ms=period)
            for variable, kind in variables:
                config.add_variable(variable, kind)
            cf.log.add_config(config)
            config.data_received_cb.add_callback(on_log)
            config.start()
            configs.append(config)

        armed = False
        try:
            # The GAP8 UART receiver is created by controller 6's init routine.
            # Select it while disarmed, and keep its output in PID passthrough,
            # before testing link health; otherwise seqRx is a valid log group
            # whose RX task has never been started.
            set_param(cf, "commander.enHighLevel", 0)
            set_param(cf, "obs.pidPass", 1)
            set_param(cf, "stabilizer.controller", 6, delay=0.2)
            time.sleep(1.0)
            before = latest.get("seqRx.rxOk", 0)
            print("checking GAP8 -> STM32 sequential UART...")
            deadline = time.monotonic() + 4.0
            while time.monotonic() < deadline and latest.get("seqRx.rxOk", 0) <= before:
                time.sleep(0.1)
            after = latest.get("seqRx.rxOk", 0)
            if after <= before:
                raise RuntimeError(
                    "no new CRC-valid .38 UART packets from GAP8 "
                    f"({link_diagnostics(latest)})")

            if not args.no_reset_estimator:
                set_param(cf, "kalman.resetEstimation", 1)
                set_param(cf, "kalman.resetEstimation", 0)
                time.sleep(2.0)

            set_param(cf, "percept.enable", 0)
            set_param(cf, "obs.enable", 0)
            set_param(cf, "obs.pidPass", 1)
            set_param(cf, "seqAvoid.logOnly", 1)
            set_param(cf, "seqAvoid.constrain", 1)
            set_param(cf, "seqAvoid.confMin", args.confidence_min)
            set_param(cf, "seqAvoid.safeMin", args.safe_min)
            set_param(cf, "seqAvoid.maxAge", args.max_age_ms)
            set_param(cf, "seqAvoid.graceMs", args.vision_grace_ms)
            set_param(cf, "seqAvoid.refShift", args.ref_shift)
            set_param(cf, "seqAvoid.trigger", args.trigger)
            set_param(cf, "seqAvoid.sideMin", args.side_min)
            set_param(cf, "seqAvoid.sideVotes", args.side_votes)
            set_param(cf, "seqAvoid.fwdMin", args.forward_min)
            set_param(cf, "seqAvoid.fwdStep", args.forward_step)
            set_param(cf, "seqAvoid.passDist", args.pass_distance)
            set_param(cf, "seqAvoid.wpTol", args.waypoint_tolerance)
            set_param(cf, "seqAvoid.clearVotes", args.clear_votes)
            set_param(cf, "seqAvoid.maxLat", args.max_route_lateral)
            set_param(cf, "seqAvoid.probeProg", args.probe_progress)
            set_param(cf, "seqAvoid.probeVotes", args.probe_votes)
            set_param(cf, "seqAvoid.maxPriRes", args.max_pri_res)
            set_param(cf, "seqAvoid.maxDuaRes", args.max_dua_res)
            set_param(cf, "seqAvoid.maxSolveUs", args.max_solve_us)
            margins = {
                "droneRad": 0.10, "trackMar": 0.08, "latency": 0.08,
                "percMar": 0.03, "confGain": 0.05,
            }
            for name, normal_value in margins.items():
                value = 0 if args.zero_margins else normal_value
                set_param(cf, f"seqAvoid.{name}", value)
            set_param(cf, "seqAvoid.enable", 1)
            set_param(cf, "commander.enHighLevel", 0)
            set_param(cf, "stabilizer.controller", 6)
            time.sleep(0.8)
            if latest.get("seqAvoid.ageMs", args.max_age_ms + 1) > args.max_age_ms:
                raise RuntimeError("controller sees stale sequential vision before arming")

            cf.platform.send_arming_request(True)
            armed = True
            time.sleep(0.5)
            ground = (args.start_x, args.start_y, 0.05)
            hover = (args.start_x, args.start_y, args.height)
            print("takeoff (controller 6, PID passthrough)")
            stream(cf, args, rows, latest, last_log_time, "takeoff",
                   args.takeoff_s, ground, hover)
            stream(cf, args, rows, latest, last_log_time, "settle",
                   args.settle_s, hover, hover)

            if latest.get("seqAvoid.stop", 1):
                raise RuntimeError("no open route direction at the start of the test")
            set_param(cf, "seqAvoid.logOnly", 1 if args.log_only else 0)
            set_param(cf, "obs.pidPass", 0)
            print("straight-line TinyMPC obstacle test")
            goal = (args.goal_x, args.goal_y, args.height)
            stream(cf, args, rows, latest, last_log_time, "run",
                   args.run_s, hover, goal, require_vision=True)
        except (KeyboardInterrupt, RuntimeError) as error:
            print(f"ABORT: {error}")
        finally:
            if armed:
                print("landing through PID passthrough")
                try:
                    set_param(cf, "obs.pidPass", 1)
                    x = latest.get("stateEstimate.x", args.start_x)
                    y = latest.get("stateEstimate.y", args.start_y)
                    z = max(0.05, latest.get("stateEstimate.z", args.height))
                    stream(cf, args, rows, latest, last_log_time, "land",
                           args.land_s, (x, y, z), (x, y, 0.05))
                except Exception as error:
                    print(f"landing stream interrupted: {error}")
                cf.commander.send_stop_setpoint()
                cf.platform.send_arming_request(False)
                time.sleep(0.3)
            for config in configs:
                config.stop()

    with Path(args.out).open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=FIELDS)
        writer.writeheader()
        writer.writerows(rows)
    print(f"wrote {len(rows)} samples to {args.out}")


if __name__ == "__main__":
    main()
