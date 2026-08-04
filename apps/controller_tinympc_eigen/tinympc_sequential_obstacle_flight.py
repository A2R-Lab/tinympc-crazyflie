#!/usr/bin/env python3
"""Fly or monitor a GAP8 four-direction obstacle-classification test.

The script verifies live CRC-valid UART packets before arming, takes off through
controller 6's PID passthrough, then enables TinyMPC output for the straight leg.
With --monitor-only it never arms or commands motion and prints each fresh
classification for a props-off, hand-carried camera test. Ctrl-C or any
vision/telemetry safety fault during flight attempts a PID-passthrough landing.
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
    "seqAvoid.valid", "seqAvoid.stop", "seqAvoid.mask",
    "seqAvoid.sample",
    "seqAvoid.side", "seqAvoid.ageMs",
    "seqAvoid.routePhase", "seqAvoid.replans",
    "seqAvoid.waypointX", "seqAvoid.waypointY",
    "seqAvoid.pathOffX", "seqAvoid.pathOffY",
    "seqAvoid.returnWait", "seqAvoid.scanAvg", "seqAvoid.scanCount",
    "seqAvoid.scanRetries", "seqAvoid.scanYaw", "seqAvoid.scanYawGoal",
    "seqAvoid.scanSettle",
    "seqAvoid.scanDecision", "seqAvoid.scanPidYaw",
    "seqAvoid.barrier",
    "seqAvoid.barrierA0", "seqAvoid.barrierA1", "seqAvoid.barrierB",
    "seqAvoid.maxSlack",
    "seqAvoid.dangerAvg", "seqAvoid.avgCount",
    "seqAvoid.fwdClear", "seqAvoid.speedCmd", "seqAvoid.goalDist",
    "seqAvoid.goalReached",
    "seqAvoid.openAvg0", "seqAvoid.openAvg1", "seqAvoid.openAvg2",
    "seqAvoid.openAvg3", "seqAvoid.sideScore0", "seqAvoid.sideScore3",
    "seqAvoid.scanClr0", "seqAvoid.scanClr1", "seqAvoid.scanClr2",
    "seqAvoid.scanClr3", "seqAvoid.scanSlope", "seqAvoid.scanRetryWait",
    "visGate.phase", "visGate.laps",
    "obs.mpcUs", "obs.iter", "obs.status", "obs.priRes", "obs.duaRes",
    "obs.healthHold", "obs.healthFaults", "obs.scanBypass",
    "obs.pidBypass",
    "obs.solveStart", "obs.solveDone", "obs.hbAge", "obs.stallHold",
    "obs.warmResets", "obs.resetWhy", "obs.solvePhase", "obs.stackFree",
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
    ("seq_classify", 50, [("seqAvoid.sample", "uint32_t"),
                           ("seqAvoid.mask", "uint8_t"),
                           ("seqAvoid.dangerAvg", "float"),
                           ("seqAvoid.avgCount", "uint8_t"),
                           ("seqAvoid.ageMs", "uint32_t"),
                           ("seqAvoid.valid", "uint8_t"),
                           ("seqAvoid.stop", "uint8_t"),
                           ("seqAvoid.side", "int8_t"),
                           ("seqAvoid.maxSlack", "float")]),
    ("seq_route", 50, [("seqAvoid.routePhase", "uint8_t"),
                        ("seqAvoid.replans", "uint32_t"),
                        ("obs.pidBypass", "uint32_t")]),
    ("seq_waypoint", 50, [("seqAvoid.waypointX", "float"),
                           ("seqAvoid.waypointY", "float"),
                           ("seqAvoid.pathOffX", "float"),
                           ("seqAvoid.pathOffY", "float"),
                           ("seqAvoid.returnWait", "float")]),
    ("seq_barrier", 50, [("seqAvoid.barrier", "uint8_t"),
                          ("seqAvoid.barrierA0", "float"),
                          ("seqAvoid.barrierA1", "float"),
                          ("seqAvoid.barrierB", "float")]),
    ("seq_scan", 50, [("seqAvoid.scanAvg", "float"),
                       ("seqAvoid.scanCount", "uint8_t"),
                       ("seqAvoid.scanRetries", "uint32_t"),
                       ("seqAvoid.scanYaw", "float"),
                       ("seqAvoid.scanYawGoal", "float"),
                       ("seqAvoid.scanSettle", "float"),
                       ("seqAvoid.scanDecision", "int8_t"),
                       ("seqAvoid.scanPidYaw", "uint8_t")]),
    ("seq_speed", 50, [("seqAvoid.fwdClear", "float"),
                        ("seqAvoid.speedCmd", "float"),
                        ("seqAvoid.goalDist", "float"),
                        ("seqAvoid.goalReached", "uint8_t"),
                        ("seqAvoid.scanSlope", "float"),
                        ("seqAvoid.scanRetryWait", "float")]),
    ("seq_direction_forward", 100, [("seqAvoid.openAvg0", "float"),
                                     ("seqAvoid.openAvg1", "float"),
                                     ("seqAvoid.openAvg2", "float"),
                                     ("seqAvoid.openAvg3", "float"),
                                     ("seqAvoid.sideScore0", "float"),
                                     ("seqAvoid.sideScore3", "float")]),
    ("seq_direction_scan", 100, [("seqAvoid.scanClr0", "float"),
                                  ("seqAvoid.scanClr1", "float"),
                                  ("seqAvoid.scanClr2", "float"),
                                  ("seqAvoid.scanClr3", "float")]),
    ("seq_circle", 100, [("visGate.phase", "float"),
                           ("visGate.laps", "uint32_t")]),
    ("seq_solver", 100, [("obs.mpcUs", "uint32_t"),
                           ("obs.iter", "uint8_t"),
                           ("obs.status", "int8_t"),
                           ("obs.priRes", "float"),
                           ("obs.duaRes", "float"),
                           ("obs.healthHold", "uint8_t"),
                           ("obs.healthFaults", "uint32_t"),
                           ("obs.scanBypass", "uint32_t")]),
    ("seq_solver_diag", 100, [("obs.solveStart", "uint32_t"),
                               ("obs.solveDone", "uint32_t"),
                               ("obs.hbAge", "uint32_t"),
                               ("obs.stallHold", "uint8_t"),
                               ("obs.warmResets", "uint32_t"),
                               ("obs.resetWhy", "uint8_t"),
                               ("obs.solvePhase", "uint8_t"),
                               ("obs.stackFree", "uint32_t")]),
]


def arguments():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--uri", default=uri_helper.uri_from_env(default=DEFAULT_URI))
    parser.add_argument("--out", default="sequential_obstacle_flight.csv")
    parser.add_argument("--cache-dir", default=".cf_cache")
    parser.add_argument("--monitor-only", action="store_true",
                        help="Never arm; print fresh camera classifications for hand testing.")
    parser.add_argument("--monitor-duration", type=float, default=0.0,
                        help="Monitor seconds; zero runs until Ctrl-C.")
    parser.add_argument("--height", type=float, default=0.5)
    parser.add_argument("--takeoff-s", type=float, default=2.5)
    parser.add_argument("--settle-s", type=float, default=1.5)
    parser.add_argument("--timeout-s", type=float, default=45.0,
                        help="Maximum wall-clock time allowed to reach the goal.")
    parser.add_argument("--land-s", type=float, default=2.5)
    parser.add_argument("--start-x", type=float, default=0.0)
    parser.add_argument("--start-y", type=float, default=0.0)
    parser.add_argument("--goal-x", type=float, default=1.5)
    parser.add_argument("--goal-y", type=float, default=0.0)
    parser.add_argument("--circle-radius", type=float, default=0.0,
                        help="Enable a tangent-start circular centerline of this radius (m).")
    parser.add_argument("--circle-speed", type=float, default=0.40,
                        help="Maximum tangential circle speed; visual clearance may reduce it (m/s).")
    parser.add_argument("--circle-laps", type=int, default=1,
                        help="Completed laps before landing in circle mode.")
    parser.add_argument("--circle-clockwise", action="store_true",
                        help="Fly clockwise; default is counterclockwise from a forward-facing start.")
    parser.add_argument("--yaw-deg", type=float, default=0.0)
    parser.add_argument("--min-vbat", type=float, default=3.55)
    parser.add_argument("--confidence-min", type=float, default=0.0)
    parser.add_argument("--safe-min", type=float, default=0.30,
                        help="Raw directional clearance threshold for open versus blocked.")
    parser.add_argument("--average-window", type=int, default=5,
                        help="Fresh classifications in the dangerous-slice moving average.")
    parser.add_argument("--trigger-average", type=float, default=3.5,
                        help="Start evasion when average dangerous slices exceeds this value.")
    parser.add_argument("--clear-average", type=float, default=1.0,
                        help="Release evasion when average dangerous slices falls below this value.")
    parser.add_argument("--max-age-ms", type=int, default=250)
    parser.add_argument("--vision-grace-ms", type=int, default=300,
                        help="Legacy compatibility option; classifier mode does not use grace.")
    parser.add_argument("--ref-shift", type=float, default=0.35)
    parser.add_argument("--return-delay", type=float, default=1.0,
                        help="Seconds to hold the displaced path after avoidance clears.")
    parser.add_argument("--return-rate", type=float, default=0.20,
                        help="Metres/second to blend back to the original path; zero disables return.")
    parser.add_argument("--no-return-scan", action="store_true",
                        help="Disable the side-looking clearance check before centerline return.")
    parser.add_argument("--return-scan-yaw", type=float, default=50.0,
                        help="Absolute yaw offset toward the obstacle during a return scan (deg).")
    parser.add_argument("--return-scan-yaw-rate", type=float, default=45.0,
                        help="Maximum scan and restoration yaw slew rate (deg/s).")
    parser.add_argument("--return-scan-settle", type=float, default=0.50,
                        help="Yaw settling time before collecting return-scan samples (s).")
    parser.add_argument("--return-scan-window", type=int, default=5,
                        help="Fresh side-looking classifications in the return-scan average.")
    parser.add_argument("--return-scan-clear-average", type=float, default=1.0,
                        help="Permit centerline return when scan danger average is below this value.")
    parser.add_argument("--return-scan-retry", type=float, default=3.0,
                        help="Forward time in the displaced lane before retrying a blocked scan (s).")
    parser.add_argument("--side-bias", type=float, default=0.03,
                        help="Open-frequency score lead required to choose one evade side (0 to 1).")
    parser.add_argument("--return-scan-gradient", type=float, default=0.06,
                        help="Outer-slice clearance gradient that marks the obstacle as falling away (m).")
    parser.add_argument("--return-scan-improvement", type=float, default=0.04,
                        help="Mean scan-clearance improvement that permits a sooner retry (m).")
    parser.add_argument("--return-scan-retry-min", type=float, default=1.5,
                        help="Minimum adaptive wait after a blocked return scan (s).")
    parser.add_argument("--return-scan-retry-max", type=float, default=6.0,
                        help="Maximum adaptive wait after a blocked return scan (s).")
    parser.add_argument("--cruise-speed", type=float, default=0.40,
                        help="Firmware maximum forward reference speed (m/s).")
    parser.add_argument("--accel-limit", type=float, default=0.40,
                        help="Firmware forward-reference acceleration limit (m/s^2).")
    parser.add_argument("--braking-accel", type=float, default=0.60,
                        help="Firmware assumed and commanded braking acceleration (m/s^2).")
    parser.add_argument("--goal-tolerance", type=float, default=0.10,
                        help="Horizontal distance used to declare goal reached (m).")
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
                        help="Exercise waypoint avoidance without applying the post-evasion half-space.")
    parser.add_argument("--no-reset-estimator", action="store_true")
    args = parser.parse_args()
    if not 1 <= args.average_window <= 32:
        parser.error("--average-window must be between 1 and 32")
    if not 0.0 <= args.clear_average <= 4.0:
        parser.error("--clear-average must be between 0 and 4")
    if not 0.0 <= args.trigger_average <= 4.0:
        parser.error("--trigger-average must be between 0 and 4")
    if args.clear_average > args.trigger_average:
        parser.error("--clear-average must not exceed --trigger-average")
    if args.return_delay < 0.0:
        parser.error("--return-delay must be non-negative")
    if args.return_rate < 0.0:
        parser.error("--return-rate must be non-negative")
    if not 1 <= args.return_scan_window <= 32:
        parser.error("--return-scan-window must be between 1 and 32")
    if not 0.0 <= args.return_scan_clear_average <= 4.0:
        parser.error("--return-scan-clear-average must be between 0 and 4")
    if args.return_scan_yaw < 0.0 or args.return_scan_yaw > 90.0:
        parser.error("--return-scan-yaw must be between 0 and 90 degrees")
    if args.return_scan_yaw_rate <= 0.0:
        parser.error("--return-scan-yaw-rate must be positive")
    if args.return_scan_settle < 0.0:
        parser.error("--return-scan-settle must be non-negative")
    if args.return_scan_retry < 0.0:
        parser.error("--return-scan-retry must be non-negative")
    if args.side_bias < 0.0:
        parser.error("--side-bias must be non-negative")
    if args.side_bias > 1.0:
        parser.error("--side-bias must not exceed 1")
    if args.return_scan_gradient < 0.0:
        parser.error("--return-scan-gradient must be non-negative")
    if args.return_scan_improvement < 0.0:
        parser.error("--return-scan-improvement must be non-negative")
    if args.return_scan_retry_min < 0.0:
        parser.error("--return-scan-retry-min must be non-negative")
    if args.return_scan_retry_max < args.return_scan_retry_min:
        parser.error("--return-scan-retry-max must not be below --return-scan-retry-min")
    if args.timeout_s <= 0.0:
        parser.error("--timeout-s must be positive")
    if args.cruise_speed <= 0.0:
        parser.error("--cruise-speed must be positive")
    if args.accel_limit <= 0.0:
        parser.error("--accel-limit must be positive")
    if args.braking_accel <= 0.0:
        parser.error("--braking-accel must be positive")
    if args.goal_tolerance <= 0.0:
        parser.error("--goal-tolerance must be positive")
    if args.circle_radius < 0.0:
        parser.error("--circle-radius must be non-negative")
    if args.circle_radius and args.circle_radius < 0.5:
        parser.error("--circle-radius must be at least 0.5 m")
    if args.circle_speed <= 0.0:
        parser.error("--circle-speed must be positive")
    if args.circle_laps < 1:
        parser.error("--circle-laps must be at least 1")
    return args


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


def monitor_classifications(args, latest, rows, monitor_samples):
    """Print each fresh firmware classification without issuing flight commands."""
    print("monitor-only mode: motors remain disarmed; press Ctrl-C to stop")
    print("directions are [0,1,2,3] = [-40,-13.3,+13.3,+40] degrees")
    started = time.monotonic()
    last_sample = None
    try:
        while args.monitor_duration <= 0 or (
                time.monotonic() - started < args.monitor_duration):
            if not monitor_samples:
                time.sleep(0.01)
                continue
            data = monitor_samples.pop(0)
            sample = int(data["seqAvoid.sample"])
            if sample == last_sample:
                continue
            last_sample = sample
            mask = int(data["seqAvoid.mask"]) & 0x0F
            dangerous = 4 - mask.bit_count()
            average = float(data["seqAvoid.dangerAvg"])
            count = int(data["seqAvoid.avgCount"])
            if count < args.average_window:
                state = "WARMUP"
            elif average > args.trigger_average:
                state = "EVADE"
            elif average < args.clear_average:
                state = "CLEAR"
            else:
                state = "MID"
            bits = "".join("S" if mask & (1 << index) else "D"
                           for index in range(4))
            print(f"sample={sample:6d} slices={bits} dangerous={dangerous} "
                  f"average={average:.2f} ({count}/{args.average_window}) "
                  f"state={state} age={int(data['seqAvoid.ageMs'])}ms")
            record(rows, latest, "monitor", (None, None, None))
    except KeyboardInterrupt:
        print("monitor stopped")


def safety_check(args, latest, last_log_time, require_vision, phase):
    if time.monotonic() - last_log_time[0] > 0.6:
        raise RuntimeError("Crazyflie log link is stale")
    battery = latest.get("pm.vbat")
    if battery is not None and battery < args.min_vbat:
        raise RuntimeError(f"battery {battery:.2f} V is below {args.min_vbat:.2f} V")
    z = latest.get("stateEstimate.z")
    # A transient low state estimate is common around takeoff and should not
    # abort an otherwise controlled run. Keep the upper-height escape guard.
    if (phase in ("settle", "run") and z is not None and
            z > max(1.0, 1.7 * args.height)):
        raise RuntimeError(f"excessive height estimate {z:.2f} m")
    x, y = latest.get("stateEstimate.x"), latest.get("stateEstimate.y")
    if args.circle_radius > 0.0 and x is not None and y is not None:
        center_y = args.start_y + (-args.circle_radius if args.circle_clockwise
                                   else args.circle_radius)
        radial_error = abs(((x - args.start_x) ** 2 +
                            (y - center_y) ** 2) ** 0.5 - args.circle_radius)
        if radial_error > args.max_lateral_error:
            raise RuntimeError(f"circle radial error {radial_error:.2f} m")
    elif y is not None and abs(y - args.start_y) > args.max_lateral_error:
        raise RuntimeError(f"lateral excursion {y - args.start_y:.2f} m")
    if require_vision:
        if latest.get("obs.stallHold", 0):
            started = latest.get("obs.solveStart", "n/a")
            completed = latest.get("obs.solveDone", "n/a")
            phase_at_solve = latest.get("obs.solvePhase", "n/a")
            raise RuntimeError(
                "MPC task heartbeat stalled "
                f"(started={started}, completed={completed}, "
                f"routePhase={phase_at_solve})")
        if latest.get("obs.healthHold", 0):
            raise RuntimeError("MPC solver health guard entered position hold")
        age = latest.get("seqAvoid.ageMs", args.max_age_ms + 1)
        if age > args.max_age_ms:
            raise RuntimeError(f"vision packet is stale ({age} ms)")
        if latest.get("seqAvoid.stop", 1):
            raise RuntimeError("vision has no open route direction; firmware entered hold")
        if latest.get("seqAvoid.routePhase", 0) == 3:
            raise RuntimeError("safe route reached its lateral limit and entered hold")


def timed_stream(cf, args, rows, latest, last_log_time, phase, duration,
                 start, target):
    begun = time.monotonic()
    while True:
        elapsed = time.monotonic() - begun
        if elapsed >= duration:
            return
        alpha = min(1.0, elapsed / max(duration, 1e-3))
        command = tuple(a + alpha * (b - a) for a, b in zip(start, target))
        safety_check(args, latest, last_log_time, False, phase)
        cf.commander.send_position_setpoint(*command, args.yaw_deg)
        record(rows, latest, phase, command)
        time.sleep(0.02)


def run_to_goal(cf, args, rows, latest, last_log_time, goal):
    """Send a fixed goal while firmware governs speed and avoidance."""
    begun = time.monotonic()
    previous_route = int(latest.get("seqAvoid.routePhase", 0))
    while True:
        elapsed = time.monotonic() - begun
        if elapsed >= args.timeout_s:
            distance = latest.get("seqAvoid.goalDist", "n/a")
            raise RuntimeError(
                f"goal timeout after {args.timeout_s:.1f} s "
                f"(remaining={distance} m)")
        safety_check(args, latest, last_log_time, True, "run")
        cf.commander.send_position_setpoint(*goal, args.yaw_deg)
        record(rows, latest, "run", goal)

        route = int(latest.get("seqAvoid.routePhase", 0))
        if route != previous_route:
            print(f"avoidance route phase {previous_route} -> {route} "
                  f"at {elapsed:.2f} s")
            previous_route = route
        if elapsed > 0.5 and latest.get("seqAvoid.goalReached", 0):
            print(f"firmware reached goal in {elapsed:.2f} s")
            return
        time.sleep(0.02)


def run_circle(cf, args, rows, latest, last_log_time, command):
    """Keep a benign streamed setpoint while firmware advances the circular path."""
    begun = time.monotonic()
    previous_route = int(latest.get("seqAvoid.routePhase", 0))
    previous_laps = int(latest.get("visGate.laps", 0))
    while True:
        elapsed = time.monotonic() - begun
        if elapsed >= args.timeout_s:
            raise RuntimeError(
                f"circle timeout after {args.timeout_s:.1f} s "
                f"(completed={latest.get('visGate.laps', 'n/a')} laps)")
        safety_check(args, latest, last_log_time, True, "run")
        cf.commander.send_position_setpoint(*command, args.yaw_deg)
        record(rows, latest, "circle", command)
        route = int(latest.get("seqAvoid.routePhase", 0))
        if route != previous_route:
            print(f"avoidance route phase {previous_route} -> {route} "
                  f"at {elapsed:.2f} s")
            previous_route = route
        laps = int(latest.get("visGate.laps", 0))
        if laps != previous_laps:
            print(f"circle lap {laps} completed at {elapsed:.2f} s")
            previous_laps = laps
        if laps >= args.circle_laps:
            print(f"firmware completed {laps} circle lap(s) in {elapsed:.2f} s")
            return
        time.sleep(0.02)


def main():
    args = arguments()
    latest, rows, configs, monitor_samples = {}, [], [], []
    last_log_time = [time.monotonic()]
    cache = Path(args.cache_dir)
    cache.mkdir(parents=True, exist_ok=True)
    cflib.crtp.init_drivers()

    with SyncCrazyflie(args.uri, cf=Crazyflie(rw_cache=str(cache))) as scf:
        cf = scf.cf

        def on_log(_timestamp, data, _config):
            latest.update(data)
            last_log_time[0] = time.monotonic()
            if args.monitor_only and "seqAvoid.sample" in data:
                monitor_samples.append(dict(data))

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

            if not args.monitor_only and not args.no_reset_estimator:
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
            set_param(cf, "seqAvoid.avgN", args.average_window)
            set_param(cf, "seqAvoid.trigAvg", args.trigger_average)
            set_param(cf, "seqAvoid.clearAvg", args.clear_average)
            set_param(cf, "seqAvoid.maxAge", args.max_age_ms)
            set_param(cf, "seqAvoid.graceMs", args.vision_grace_ms)
            set_param(cf, "seqAvoid.refShift", args.ref_shift)
            set_param(cf, "seqAvoid.returnDelay", args.return_delay)
            set_param(cf, "seqAvoid.returnRate", args.return_rate)
            set_param(cf, "seqAvoid.scanEnable", 0 if args.no_return_scan else 1)
            set_param(cf, "seqAvoid.scanYaw", args.return_scan_yaw)
            set_param(cf, "seqAvoid.scanYawRate", args.return_scan_yaw_rate)
            set_param(cf, "seqAvoid.scanSettle", args.return_scan_settle)
            set_param(cf, "seqAvoid.scanAvgN", args.return_scan_window)
            set_param(cf, "seqAvoid.scanClear", args.return_scan_clear_average)
            set_param(cf, "seqAvoid.scanRetry", args.return_scan_retry)
            set_param(cf, "seqAvoid.sideBias", args.side_bias)
            set_param(cf, "seqAvoid.scanGrad", args.return_scan_gradient)
            set_param(cf, "seqAvoid.scanImprove", args.return_scan_improvement)
            set_param(cf, "seqAvoid.scanRetryMin", args.return_scan_retry_min)
            set_param(cf, "seqAvoid.scanRetryMax", args.return_scan_retry_max)
            set_param(cf, "seqAvoid.cruise", args.cruise_speed)
            set_param(cf, "seqAvoid.cruiseAcc", args.accel_limit)
            set_param(cf, "seqAvoid.brakeAcc", args.braking_accel)
            set_param(cf, "seqAvoid.goalTol", args.goal_tolerance)
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
            circle_enabled = args.circle_radius > 0.0
            set_param(cf, "gateNav.navEn", 0)
            if circle_enabled:
                # Start at the near point on the circumference with yaw along +x.
                # CCW puts the center to the left; clockwise puts it to the right.
                center_sign = -1.0 if args.circle_clockwise else 1.0
                direction = 1 if args.circle_clockwise else -1
                set_param(cf, "circuit.gAx", args.start_x)
                set_param(cf, "circuit.gAy", args.start_y)
                set_param(cf, "circuit.gAz", args.height)
                set_param(cf, "circuit.gBx", args.start_x)
                set_param(cf, "circuit.gBy",
                          args.start_y + 2.0 * center_sign * args.circle_radius)
                set_param(cf, "circuit.gBz", args.height)
                set_param(cf, "circuit.loopW", args.circle_radius)
                set_param(cf, "circuit.speed", args.circle_speed)
                set_param(cf, "circuit.dir", direction)
                set_param(cf, "circuit.en", 1)
            else:
                set_param(cf, "circuit.en", 0)
            set_param(cf, "seqAvoid.enable", 1)
            set_param(cf, "commander.enHighLevel", 0)
            set_param(cf, "stabilizer.controller", 6)
            time.sleep(0.8)
            if latest.get("seqAvoid.ageMs", args.max_age_ms + 1) > args.max_age_ms:
                raise RuntimeError("controller sees stale sequential vision before arming")

            if args.monitor_only:
                monitor_samples.clear()
                monitor_classifications(args, latest, rows, monitor_samples)
            else:
                cf.platform.send_arming_request(True)
                armed = True
                time.sleep(0.5)
                ground = (args.start_x, args.start_y, 0.05)
                hover = (args.start_x, args.start_y, args.height)
                print("takeoff (controller 6, PID passthrough)")
                timed_stream(cf, args, rows, latest, last_log_time, "takeoff",
                             args.takeoff_s, ground, hover)
                timed_stream(cf, args, rows, latest, last_log_time, "settle",
                             args.settle_s, hover, hover)

                if latest.get("seqAvoid.stop", 1):
                    raise RuntimeError("no open route direction at the start of the test")
                set_param(cf, "seqAvoid.logOnly", 1 if args.log_only else 0)
                set_param(cf, "obs.pidPass", 0)
                if circle_enabled:
                    print("firmware-governed TinyMPC obstacle circle test")
                    command = (args.start_x, args.start_y, args.height)
                    run_circle(cf, args, rows, latest, last_log_time, command)
                else:
                    print("firmware-governed TinyMPC obstacle test")
                    goal = (args.goal_x, args.goal_y, args.height)
                    run_to_goal(cf, args, rows, latest, last_log_time, goal)
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
                    timed_stream(cf, args, rows, latest, last_log_time, "land",
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
