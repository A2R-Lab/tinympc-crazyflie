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
    ("motion", 50, [
        ("stateEstimate.vx", "float"),
        ("stateEstimate.vy", "float"),
        ("stabilizer.roll", "float"),
        ("stabilizer.pitch", "float"),
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
        ("flowObsRx.trackRx", "uint32_t"),
        ("flowObsRx.mapPeak", "float"),
        ("flowObsRx.mapActive", "uint8_t"),
        ("flowObsRx.cylValid", "float"),
        ("flowObsRx.cylConf", "float"),
    ]),
    ("flow_tracks", 100, [
        ("flowObsRx.depthN", "uint8_t"),
        ("flowObsRx.fwdDepth", "uint8_t"),
        ("flowObsRx.trkSupport", "uint8_t"),
        ("flowObsRx.trkSigma", "float"),
        ("flowObsRx.baseline", "float"),
    ]),
    ("flow_cyl", 100, [
        ("flowObsRx.cylWx", "float"),
        ("flowObsRx.cylWy", "float"),
        ("flowObsRx.cylVarX", "float"),
        ("flowObsRx.cylVarY", "float"),
    ]),
    ("reject_values", 100, [
        ("flowObsRx.aggDisp", "float"),
        ("flowObsRx.yawRatio", "float"),
        ("flowObsRx.depDisagr", "float"),
        ("flowObsRx.grpDisp", "float"),
        ("flowObsRx.grpScore", "float"),
    ]),
    ("reject_counts", 100, [
        ("flowObsRx.rejMotion", "uint32_t"),
        ("flowObsRx.rejYaw", "uint32_t"),
        ("flowObsRx.rejDepth", "uint32_t"),
        ("flowObsRx.rejDisp", "uint32_t"),
        ("flowObsRx.rejGroup", "uint32_t"),
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
    "state_vx",
    "state_vy",
    "roll",
    "pitch",
    "vbat",
    "track_rx",
    "track_depth_count",
    "track_support",
    "track_sigma",
    "track_baseline",
    "map_peak",
    "map_active",
    "flow_cyl_valid",
    "flow_cyl_conf",
    "flow_cyl_wx",
    "flow_cyl_wy",
    "flow_cyl_var_x",
    "flow_cyl_var_y",
    "aggregate_displacement",
    "yaw_explained_ratio",
    "depth_disagreement",
    "group_dispersion",
    "group_score",
    "reject_low_motion",
    "reject_yaw",
    "reject_depth_disagreement",
    "reject_dispersion",
    "reject_no_group",
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
    p.add_argument("--approach-s", type=float, default=0.0,
                   help="Slowly approach for up to this many seconds until the flow obstacle freezes.")
    p.add_argument("--approach-x", type=float, default=0.0,
                   help="Maximum world-x setpoint during the detection approach.")
    p.add_argument("--approach-hold-s", type=float, default=0.0,
                   help="Additional peering time at approach-x if detection has not frozen.")
    p.add_argument("--approach-peer-amp", type=float, default=0.08,
                   help="Lateral peering amplitude superimposed on the slow approach [m].")
    p.add_argument("--max-horizontal-excursion", type=float, default=0.25,
                   help="Abort peering if estimated horizontal displacement from the start exceeds this [m].")
    p.add_argument("--min-height-fraction", type=float, default=0.55,
                   help="Abort if estimated altitude falls below this fraction of commanded height.")
    p.add_argument("--max-height-factor", type=float, default=1.60,
                   help="Abort if estimated altitude exceeds this multiple of commanded height.")
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
    p.add_argument(
        "--pid-detour", action="store_true",
        help=("Keep the stable PID-passthrough controller and route the run leg "
              "around the frozen flow obstacle. Required for an avoidance run "
              "while obs.pidPass=1."))
    p.add_argument("--detour-margin", type=float, default=0.05,
                   help="Additional lateral clearance outside radius+safety [m].")
    p.add_argument("--min-detour-confidence", type=float, default=0.25,
                   help="Minimum frozen flow confidence required before a PID detour.")
    p.add_argument("--min-track-baseline", type=float, default=0.04,
                   help="Minimum measured lateral camera baseline before accepting a frozen obstacle [m].")
    p.add_argument("--max-track-sigma", type=float, default=0.20,
                   help="Maximum per-frame clustered track range sigma [m].")
    p.add_argument("--max-cylinder-sigma", type=float, default=0.30,
                   help="Maximum sqrt of either cylinder covariance diagonal [m].")
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
        "state_vx": latest_get(latest, "stateEstimate.vx"),
        "state_vy": latest_get(latest, "stateEstimate.vy"),
        "roll": latest_get(latest, "stabilizer.roll"),
        "pitch": latest_get(latest, "stabilizer.pitch"),
        "vbat": latest_get(latest, "pm.vbat"),
        "track_rx": latest_get(latest, "flowObsRx.trackRx"),
        "track_depth_count": latest_get(latest, "flowObsRx.depthN"),
        "track_support": latest_get(latest, "flowObsRx.trkSupport"),
        "track_sigma": latest_get(latest, "flowObsRx.trkSigma"),
        "track_baseline": latest_get(latest, "flowObsRx.baseline"),
        "map_peak": latest_get(latest, "flowObsRx.mapPeak"),
        "map_active": latest_get(latest, "flowObsRx.mapActive"),
        "flow_cyl_valid": latest_get(latest, "flowObsRx.cylValid"),
        "flow_cyl_conf": latest_get(latest, "flowObsRx.cylConf"),
        "flow_cyl_wx": latest_get(latest, "flowObsRx.cylWx"),
        "flow_cyl_wy": latest_get(latest, "flowObsRx.cylWy"),
        "flow_cyl_var_x": latest_get(latest, "flowObsRx.cylVarX"),
        "flow_cyl_var_y": latest_get(latest, "flowObsRx.cylVarY"),
        "aggregate_displacement": latest_get(latest, "flowObsRx.aggDisp"),
        "yaw_explained_ratio": latest_get(latest, "flowObsRx.yawRatio"),
        "depth_disagreement": latest_get(latest, "flowObsRx.depDisagr"),
        "group_dispersion": latest_get(latest, "flowObsRx.grpDisp"),
        "group_score": latest_get(latest, "flowObsRx.grpScore"),
        "reject_low_motion": latest_get(latest, "flowObsRx.rejMotion"),
        "reject_yaw": latest_get(latest, "flowObsRx.rejYaw"),
        "reject_depth_disagreement": latest_get(latest, "flowObsRx.rejDepth"),
        "reject_dispersion": latest_get(latest, "flowObsRx.rejDisp"),
        "reject_no_group": latest_get(latest, "flowObsRx.rejGroup"),
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


def ensure_link_fresh(latest, maximum_age_s=0.5):
    last_rx = latest_get(latest, "_host_rx_time")
    if last_rx is not None and time.monotonic() - last_rx > maximum_age_s:
        raise RuntimeError(
            f"flight telemetry stale for more than {maximum_age_s:.1f}s")


def stream_position(cf, rows, latest, seconds, x, y, z, yaw_deg, phase):
    steps = max(1, int(seconds * 50.0))
    for _ in range(steps):
        ensure_link_fresh(latest)
        cf.commander.send_position_setpoint(x, y, z, yaw_deg)
        rows.append(make_row(phase, latest, x, y, z))
        time.sleep(0.02)


def stream_line(cf, rows, latest, seconds, x0, y0, z0, x1, y1, z1, yaw_deg, phase):
    steps = max(1, int(seconds * 50.0))
    denom = max(1, steps - 1)
    for step in range(steps):
        ensure_link_fresh(latest)
        a = step / denom
        x = x0 + a * (x1 - x0)
        y = y0 + a * (y1 - y0)
        z = z0 + a * (z1 - z0)
        cf.commander.send_position_setpoint(x, y, z, yaw_deg)
        rows.append(make_row(phase, latest, x, y, z))
        time.sleep(0.02)


def stream_polyline(cf, rows, latest, seconds, points, z, yaw_deg, phase):
    segments = []
    total_distance = 0.0
    for start, end in zip(points, points[1:]):
        distance = math.hypot(end[0] - start[0], end[1] - start[1])
        segments.append((start, end, distance))
        total_distance += distance
    for index, (start, end, distance) in enumerate(segments):
        segment_s = seconds * distance / max(total_distance, 1.0e-6)
        stream_line(cf, rows, latest, segment_s,
                    start[0], start[1], z, end[0], end[1], z,
                    yaw_deg, f"{phase}_{index}")


def obstacle_detour_points(start, goal, obstacle, clearance):
    dx = goal[0] - start[0]
    dy = goal[1] - start[1]
    length = math.hypot(dx, dy)
    if length < 1.0e-6:
        raise ValueError("start and goal must differ")
    if math.hypot(start[0] - obstacle[0],
                  start[1] - obstacle[1]) < clearance:
        raise ValueError("start pose is inside the obstacle clearance")
    if math.hypot(goal[0] - obstacle[0],
                  goal[1] - obstacle[1]) < clearance:
        raise ValueError("goal pose is inside the obstacle clearance")
    ux, uy = dx / length, dy / length
    px, py = -uy, ux
    ox, oy = obstacle[0] - start[0], obstacle[1] - start[1]
    signed_lateral = ox * px + oy * py
    side = -1.0 if signed_lateral > 0.0 else 1.0
    detour_lateral = signed_lateral + side * clearance
    shifted_start = (start[0] + px * detour_lateral,
                     start[1] + py * detour_lateral)
    shifted_goal = (goal[0] + px * detour_lateral,
                    goal[1] + py * detour_lateral)
    return [start, shifted_start, shifted_goal, goal]


def inside_flight_envelope(latest, x0, y0, height, max_horizontal,
                           min_height_fraction, max_height_factor):
    x = latest_get(latest, "stateEstimate.x")
    y = latest_get(latest, "stateEstimate.y")
    z = latest_get(latest, "stateEstimate.z")
    if x is None or y is None or z is None:
        return False
    horizontal = math.hypot(x - x0, y - y0)
    return (
        horizontal <= max_horizontal
        and z >= height * min_height_fraction
        and z <= height * max_height_factor
    )


def stream_peer(cf, rows, latest, seconds, x, y0, z, yaw_deg, amp, period_s,
                max_horizontal, min_height_fraction, max_height_factor):
    steps = max(1, int(seconds * 50.0))
    for step in range(steps):
        ensure_link_fresh(latest)
        t = step * 0.02
        y = y0 + amp * math.sin(2.0 * math.pi * t / period_s)
        cf.commander.send_position_setpoint(x, y, z, yaw_deg)
        rows.append(make_row("peer", latest, x, y, z))
        if step >= 25 and not inside_flight_envelope(
                latest, x, y0, z, max_horizontal,
                min_height_fraction, max_height_factor):
            print(
                "ABORT: flight envelope exceeded during peering: "
                f"state=({latest_get(latest, 'stateEstimate.x')}, "
                f"{latest_get(latest, 'stateEstimate.y')}, "
                f"{latest_get(latest, 'stateEstimate.z')})"
            )
            return False
        time.sleep(0.02)
    return True


def stream_approach_until_frozen(cf, rows, latest, seconds,
                                 x0, y0, x1, z, yaw_deg,
                                 peer_amp, peer_period_s,
                                 min_height_fraction, max_height_factor):
    steps = max(1, int(seconds * 50.0))
    denom = max(1, steps - 1)
    for step in range(steps):
        ensure_link_fresh(latest)
        a = step / denom
        x = x0 + a * (x1 - x0)
        t = step * 0.02
        y = y0 + peer_amp * math.sin(2.0 * math.pi * t / peer_period_s)
        cf.commander.send_position_setpoint(x, y, z, yaw_deg)
        rows.append(make_row("approach", latest, x, y, z))
        if int(latest_get(latest, "obs.frzValid", 0) or 0):
            state_x = float(latest_get(latest, "stateEstimate.x", x) or x)
            state_y = float(latest_get(latest, "stateEstimate.y", y) or y)
            return True, (state_x, state_y)
        state_z = latest_get(latest, "stateEstimate.z")
        if (state_z is not None and
                (state_z < z * min_height_fraction or
                 state_z > z * max_height_factor)):
            print(f"ABORT: altitude envelope exceeded during approach: "
                  f"z={state_z}")
            return False, (x, y)
        time.sleep(0.02)
    return False, (x1, y0)


def wait_for_flow(cf, rows, latest, seconds, x, y, z, yaw_deg):
    start_rx = latest_get(latest, "flowObsRx.trackRx", 0) or 0
    steps = max(1, int(seconds * 50.0))
    for _ in range(steps):
        cf.commander.send_position_setpoint(x, y, z, yaw_deg)
        rows.append(make_row("wait_flow", latest, x, y, z))
        rx_now = latest_get(latest, "flowObsRx.trackRx", 0) or 0
        if rx_now > start_rx:
            return True
        time.sleep(0.02)
    return False


def land_pid(cf, rows, latest, seconds, z_final=0.05, switch_controller=True):
    if switch_controller:
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
    run_start = (args.start_x, args.start_y)

    Path(args.cache_dir).mkdir(parents=True, exist_ok=True)
    cflib.crtp.init_drivers()

    with SyncCrazyflie(args.uri, cf=Crazyflie(rw_cache=args.cache_dir)) as scf:
        cf = scf.cf

        def on_log(_timestamp, data, _logconf):
            latest.update(data)
            latest["_host_rx_time"] = time.monotonic()

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
                time.sleep(5.0)

            set_param(cf, "commander.enHighLevel", 0)
            set_param(cf, "stabilizer.controller", 1)
            set_param(cf, "obs.enable", 0)
            set_param(cf, "obs.logOnly", 1)
            set_param(cf, "obs.pidPass", 1)
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
                freeze_after_s = 0.8 * args.peer_s
            set_param(cf, "obs.frzAfter", int(freeze_after_s * 1000.0))

            if not args.skip_flow_warmup:
                print("preflight AI-deck flow warmup in PID passthrough on ground")
                set_param(cf, "stabilizer.controller", 6, delay=0.2)
                if not wait_for_flow(cf, rows, latest, args.flow_start_timeout_s,
                                     args.start_x, args.start_y, 0.05,
                                     args.yaw_deg):
                    print("ABORT: no AI-deck flow packets during preflight warmup")
                    return
            else:
                # Enter the OOT wrapper while disarmed. obs.pidPass keeps the
                # commanded flight output on stock PID while its perception
                # task runs; never switch controller instances in midair.
                set_param(cf, "stabilizer.controller", 6, delay=0.2)

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

            if not inside_flight_envelope(
                    latest, args.start_x, args.start_y, args.height,
                    args.max_horizontal_excursion,
                    args.min_height_fraction, args.max_height_factor):
                print(
                    "ABORT: takeoff/settle did not enter the required flight envelope: "
                    f"state=({latest_get(latest, 'stateEstimate.x')}, "
                    f"{latest_get(latest, 'stateEstimate.y')}, "
                    f"{latest_get(latest, 'stateEstimate.z')})"
                )
                abort_after_cleanup = True
                current_z = latest_get(latest, "stateEstimate.z", 0.0) or 0.0
                if current_z >= 0.08:
                    land_pid(cf, rows, latest, args.land_s, switch_controller=False)

            if not abort_after_cleanup:
                print("enabling flow perception; PID passthrough remains active")
                print(f"peer command: {args.peer_s:.1f}s side-to-side, freeze after {freeze_after_s:.1f}s")
                set_param(cf, "obs.enable", 1)
                set_param(cf, "obs.logOnly", 1)
                set_param(cf, "obs.useFlow", 1)
                set_param(cf, "obs.freeze", 1)
                print("waiting for AI-deck flow packets")
                if not wait_for_flow(cf, rows, latest, args.flow_start_timeout_s,
                                     args.start_x, args.start_y, args.height,
                                     args.yaw_deg):
                    print("ABORT: no AI-deck flow packets after switching to TinyMPC")
                    abort_after_cleanup = True
                    land_pid(cf, rows, latest, args.land_s, switch_controller=False)

            if not abort_after_cleanup:
                peer_ok = stream_peer(
                    cf, rows, latest, args.peer_s,
                    args.start_x, args.start_y, args.height,
                    args.yaw_deg, args.peer_amp, args.peer_period_s,
                    args.max_horizontal_excursion,
                    args.min_height_fraction, args.max_height_factor)
                if not peer_ok:
                    abort_after_cleanup = True
                    land_pid(cf, rows, latest, args.land_s, switch_controller=False)

            if (not abort_after_cleanup and args.approach_s > 0.0 and
                    not int(latest_get(latest, "obs.frzValid", 0) or 0)):
                print(f"approaching toward x={args.approach_x:.2f} m until "
                      f"detection, bounded to {args.approach_s:.1f}s")
                approach_ok, run_start = stream_approach_until_frozen(
                    cf, rows, latest, args.approach_s,
                    args.start_x, args.start_y, args.approach_x,
                    args.height, args.yaw_deg,
                    args.approach_peer_amp, args.peer_period_s,
                    args.min_height_fraction, args.max_height_factor)
                if not approach_ok and args.approach_hold_s > 0.0:
                    print(f"holding at x={args.approach_x:.2f} m with "
                          f"peering for {args.approach_hold_s:.1f}s")
                    approach_ok, run_start = stream_approach_until_frozen(
                        cf, rows, latest, args.approach_hold_s,
                        args.approach_x, args.start_y, args.approach_x,
                        args.height, args.yaw_deg,
                        args.approach_peer_amp, args.peer_period_s,
                        args.min_height_fraction, args.max_height_factor)
                if not approach_ok:
                    print("ABORT: approach ended without a frozen obstacle")
                    abort_after_cleanup = True
                    land_pid(cf, rows, latest, args.land_s,
                             switch_controller=False)

            frz_valid = latest_get(latest, "obs.frzValid", 0) or 0
            source = latest_get(latest, "obs.source", 0) or 0
            print(f"freeze status: frzValid={frz_valid} source={source} "
                  f"center=({latest_get(latest, 'obs.frzCx')}, {latest_get(latest, 'obs.frzCy')})")
            print(f"map status: trackRx={latest_get(latest, 'flowObsRx.trackRx')} "
                  f"mapPeak={latest_get(latest, 'flowObsRx.mapPeak')} "
                  f"cylValid={latest_get(latest, 'flowObsRx.cylValid')} "
                  f"cylConf={latest_get(latest, 'flowObsRx.cylConf')}")
            track_baseline = float(
                latest_get(latest, "flowObsRx.baseline", 0.0) or 0.0)
            track_support = int(
                latest_get(latest, "flowObsRx.trkSupport", 0) or 0)
            track_sigma = float(
                latest_get(latest, "flowObsRx.trkSigma", float("inf"))
                or float("inf"))
            cylinder_sigma = math.sqrt(max(
                float(latest_get(latest, "flowObsRx.cylVarX",
                                 float("inf")) or float("inf")),
                float(latest_get(latest, "flowObsRx.cylVarY",
                                 float("inf")) or float("inf")),
            ))
            print(f"observability: baseline={track_baseline:.3f}m "
                  f"support={track_support} trackSigma={track_sigma:.3f}m "
                  f"cylinderSigma={cylinder_sigma:.3f}m")
            safety_gate_ok = (
                track_baseline >= args.min_track_baseline
                and track_support >= 3
                and track_sigma <= args.max_track_sigma
                and cylinder_sigma <= args.max_cylinder_sigma
            )
            if abort_after_cleanup:
                pass
            elif int(frz_valid) != 0 and not safety_gate_ok:
                print("ABORT: frozen obstacle failed baseline/support/"
                      "covariance safety gates")
                abort_after_cleanup = True
                land_pid(cf, rows, latest, args.land_s,
                         switch_controller=False)
            elif not args.log_only and int(frz_valid) == 0 and not args.run_without_freeze:
                print("ABORT: obstacle did not freeze; refusing to fly run leg")
                abort_after_cleanup = True
                land_pid(cf, rows, latest, args.land_s, switch_controller=False)
            elif not args.log_only and int(frz_valid) == 0 and args.run_without_freeze:
                print("freeze failed: disabling obs and flying run leg without obstacle constraints")
                set_param(cf, "obs.enable", 0)
                set_param(cf, "obs.logOnly", 1)
                set_param(cf, "obs.useFlow", 0)
            elif not args.log_only:
                set_param(cf, "obs.logOnly", 0)
                stream_position(cf, rows, latest, 0.5,
                                run_start[0], run_start[1], args.height,
                                args.yaw_deg, "apply_settle")

            if not abort_after_cleanup:
                if args.pid_detour:
                    if int(frz_valid) == 0:
                        print("ABORT: PID detour requires a frozen obstacle")
                        abort_after_cleanup = True
                        land_pid(cf, rows, latest, args.land_s,
                                 switch_controller=False)
                    else:
                        frozen = (
                            float(latest_get(latest, "obs.frzCx")),
                            float(latest_get(latest, "obs.frzCy")),
                        )
                        frozen_confidence = float(
                            latest_get(latest, "obs.frzCf", 0.0) or 0.0)
                        if frozen_confidence < args.min_detour_confidence:
                            print("ABORT: frozen obstacle confidence "
                                  f"{frozen_confidence:.3f} < required "
                                  f"{args.min_detour_confidence:.3f}")
                            abort_after_cleanup = True
                            land_pid(cf, rows, latest, args.land_s,
                                     switch_controller=False)
                            frozen = None
                        if abort_after_cleanup:
                            pass
                        else:
                            clearance = (args.obs_radius + args.obs_safety +
                                         args.detour_margin)
                            try:
                                points = obstacle_detour_points(
                                    run_start,
                                    (args.goal_x, args.goal_y),
                                    frozen, clearance)
                            except ValueError as exc:
                                print(f"ABORT: infeasible avoidance geometry: {exc}")
                                abort_after_cleanup = True
                                land_pid(cf, rows, latest, args.land_s,
                                         switch_controller=False)
                            else:
                                print(f"flying PID avoidance detour around {frozen}: "
                                      f"{points}")
                                stream_polyline(
                                    cf, rows, latest, args.run_s, points,
                                    args.height, args.yaw_deg, "avoid")
                elif not args.log_only:
                    print("ABORT: obstacle constraints do not affect PID "
                          "passthrough; use --pid-detour or --log-only")
                    abort_after_cleanup = True
                    land_pid(cf, rows, latest, args.land_s,
                             switch_controller=False)
                else:
                    print("flying log-only run leg")
                    stream_line(cf, rows, latest, args.run_s,
                                args.start_x, args.start_y, args.height,
                                args.goal_x, args.goal_y, args.height,
                                args.yaw_deg, "run")

                if not abort_after_cleanup:
                    print("landing")
                    land_pid(cf, rows, latest, args.land_s,
                             switch_controller=False)
        except KeyboardInterrupt:
            print("\nabort requested, landing")
            land_pid(cf, rows, latest, args.land_s, switch_controller=False)
        except RuntimeError as exc:
            print(f"ABORT: {exc}")
        finally:
            for action in (
                lambda: cf.commander.send_stop_setpoint(),
                lambda: cf.platform.send_arming_request(False),
                lambda: set_param(cf, "stabilizer.controller", 1, delay=0.01),
                lambda: set_param(cf, "obs.enable", 0, delay=0.01),
                lambda: set_param(cf, "obs.logOnly", 1, delay=0.01),
                lambda: set_param(cf, "obs.pidPass", 0, delay=0.01),
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
