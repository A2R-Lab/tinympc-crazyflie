#!/usr/bin/env python3
"""Straight-line NanoCockpit neural obstacle-avoidance flight.

This intentionally does *not* perform flow peering, use the flow-cylinder
estimator, or wait for a gate. It starts the straight commander position-
setpoint leg immediately after takeoff; neural constraints become active as
fresh neural maps arrive.
"""

from __future__ import annotations

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
    ("state", 50, [("stateEstimate.x", "float"), ("stateEstimate.y", "float"),
                   ("stateEstimate.z", "float"), ("stateEstimate.yaw", "float"),
                   ("stateEstimate.vx", "float"), ("stateEstimate.vy", "float")]),
    ("battery", 100, [("pm.vbat", "float"),
                       ("perceptPlane.avoidX", "float"),
                       ("perceptPlane.avoidY", "float"),
                       ("perceptPlane.avoidZ", "float")]),
    ("perception", 50, [("percept.valid", "uint8_t"), ("percept.ageMs", "uint32_t"),
                         ("percept.maxDanger", "float"), ("percept.ctrDanger", "float"),
                         ("percept.minTtc", "float"), ("percept.corridor", "uint8_t"),
                         ("percept.nCstr", "uint8_t"), ("percept.applied", "uint8_t"),
                         ("percept.side", "int8_t"), ("percept.hold", "uint8_t"),
                         ("percept.holdAge", "uint32_t")]),
    ("plane", 50, [("perceptPlane.ax", "float"), ("perceptPlane.ay", "float"),
                    ("perceptPlane.az", "float"), ("perceptPlane.b", "float"),
                    ("perceptPlane.safeU", "float"), ("perceptPlane.lineU", "float")]),
    ("mpc", 50, [("mpcDiag.outX", "float"), ("mpcDiag.outY", "float"),
                  ("mpcDiag.outZ", "float"), ("mpcDiag.yawCmd", "float"),
                  ("obs.mpcUs", "uint32_t"), ("obs.iter", "uint8_t"),
                  ("mpcDiag.safe", "uint8_t"), ("mpcDiag.outVio", "float")]),
    ("grid", 100, [("percept.sample", "uint32_t"),
                    *[(f"perceptGridA.r{row}", "uint16_t") for row in range(5)],
                    *[(f"perceptGridB.r{row}", "uint16_t") for row in range(5, 10)],
                    ("percept.side", "int8_t"), ("percept.nCstr", "uint8_t")]),
    ("vision_health", 100, [("perceptMap.rxOk", "uint32_t"),
                            ("perceptMap.crcErr", "uint32_t"),
                            ("perceptMap.invalid", "uint32_t"),
                            ("gate8.stackFree", "uint32_t"),
                            ("perceptMass.cells", "uint8_t"),
                            ("perceptMass.cluster", "uint8_t"),
                            ("perceptMass.detected", "uint8_t"),
                            # Raw GAP8 danger scores make the live threshold
                            # decision observable in the flight CSV.
                            ("perceptMass.centerQ", "uint8_t"),
                            ("perceptMass.maxQ", "uint8_t"),
                            ("perceptPlane.refOff", "float")]),
    ("path_hit", 50, [("perceptPath.hit", "uint8_t"),
                       ("perceptPath.knot", "uint8_t"),
                       ("perceptPath.q", "uint8_t"),
                       ("perceptPath.safeL", "uint8_t"),
                       ("perceptPath.safeR", "uint8_t"),
                       ("perceptPath.u", "float"),
                       ("perceptPath.v", "float"),
                       ("perceptVio.plan", "float"),
                       ("perceptVio.state", "float"),
                       ("percept.mode", "uint8_t"),
                       ("percept.persist", "uint8_t"),
                       ("percept.clearCnt", "uint8_t")]),
]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--uri", default=uri_helper.uri_from_env(default=DEFAULT_URI))
    parser.add_argument("--out", type=Path, default=Path("neural_straight_obstacle_test.csv"))
    parser.add_argument("--map-out", type=Path,
                        help="ASCII map report (default: CSV path with .maps.txt suffix)")
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--apply", dest="apply", action="store_true",
                      help="apply neural half-spaces (default)")
    mode.add_argument("--log-only", dest="apply", action="store_false",
                      help="compute and log avoidance without applying constraints")
    parser.set_defaults(apply=True)
    parser.add_argument("--height", type=float, default=0.50)
    parser.add_argument("--takeoff-s", type=float, default=2.0)
    parser.add_argument("--settle-s", type=float, default=1.0)
    parser.add_argument("--run-s", type=float, default=7.0)
    parser.add_argument("--hold-ms", type=int, default=None,
                        help=argparse.SUPPRESS)  # Deprecated alias for --hold-steps.
    parser.add_argument("--mass-cells", type=int, default=4,
                        help="connected danger cells required for console detection")
    parser.add_argument("--danger-q", type=int, default=51,
                        help="raw GAP8 danger threshold, 0--255 (default: 51 = laptop p=0.65)")
    parser.add_argument("--line-offset-px", type=float, default=16.0,
                        help="vertical constraint-line offset toward the nearer safe side")
    parser.add_argument("--hold-steps", type=int, default=6,
                        help="minimum 50 Hz MPC steps before rejoin is allowed")
    parser.add_argument("--persist-maps", type=int, default=2,
                        help="consecutive dangerous maps required before committing")
    parser.add_argument("--safe-band-cells", type=int, default=2,
                        help="contiguous safe cells required on an avoidance side")
    parser.add_argument("--constraint-steps", type=int, default=8,
                        help="number of MPC knots guarded by the image-boundary plane")
    parser.add_argument("--clear-maps", type=int, default=3,
                        help="consecutive clear nominal-path maps required to rejoin")
    parser.add_argument("--rejoin-steps", type=int, default=25,
                        help="50 Hz MPC steps over which the avoidance offset decays")
    parser.add_argument("--max-lateral-speed", type=float, default=0.60,
                        help="maximum speed encoded in deformed reference knots")
    parser.add_argument("--land-s", type=float, default=2.0)
    parser.add_argument("--start-x", type=float, default=0.0)
    parser.add_argument("--start-y", type=float, default=0.0)
    parser.add_argument("--goal-x", type=float, default=1.20)
    parser.add_argument("--goal-y", type=float, default=0.0)
    parser.add_argument("--yaw-deg", type=float, default=0.0,
                        help="fallback yaw when the tracking point is within 5 cm")
    parser.add_argument("--min-vbat", type=float, default=3.55)
    parser.add_argument("--no-reset-estimator", action="store_true")
    parser.add_argument("--cache-dir", default=".cf_cache")
    return parser.parse_args()


def numeric(row: dict[str, object], key: str, default: float = 0.0) -> float:
    try:
        return float(row.get(key, default) or default)
    except (TypeError, ValueError):
        return default


def write_map_report(rows: list[dict[str, object]], output: Path) -> int:
    """Write one human-readable 10x10 safe/unsafe image for each map sample."""
    snapshots: list[str] = []
    seen: set[int] = set()
    for row in rows:
        sample = int(numeric(row, "percept.sample"))
        if sample <= 0 or sample in seen:
            continue
        if not all(f"perceptGrid{'A' if y < 5 else 'B'}.r{y}" in row
                   for y in range(10)):
            continue
        seen.add(sample)
        snapshots.append(
            f"sample={sample} phase={row.get('phase', '')} "
            f"center_danger={numeric(row, 'percept.ctrDanger'):.3f} "
            f"side={int(numeric(row, 'percept.side'))} "
            f"constraints={int(numeric(row, 'percept.nCstr'))} "
            f"hold={int(numeric(row, 'percept.hold'))} "
            f"hold_age_ms={int(numeric(row, 'percept.holdAge'))} "
            f"obstacle_cells={int(numeric(row, 'perceptMass.cells'))} "
            f"largest_cluster={int(numeric(row, 'perceptMass.cluster'))} "
            f"mass_detected={int(numeric(row, 'perceptMass.detected'))} "
            f"center_q={int(numeric(row, 'perceptMass.centerQ'))} "
            f"max_q={int(numeric(row, 'perceptMass.maxQ'))} "
            f"path_hit={int(numeric(row, 'perceptPath.hit'))} "
            f"mode={int(numeric(row, 'percept.mode'))} "
            f"persist={int(numeric(row, 'percept.persist'))} "
            f"clear_count={int(numeric(row, 'percept.clearCnt'))} "
            f"knot={int(numeric(row, 'perceptPath.knot'))} "
            f"path_q={int(numeric(row, 'perceptPath.q'))} "
            f"path_uv=({numeric(row, 'perceptPath.u'):.1f},"
            f"{numeric(row, 'perceptPath.v'):.1f}) "
            f"safe_lr=({int(numeric(row, 'perceptPath.safeL'))},"
            f"{int(numeric(row, 'perceptPath.safeR'))}) "
            f"safe_u={numeric(row, 'perceptPlane.safeU'):.1f} "
            f"line_u={numeric(row, 'perceptPlane.lineU'):.1f} "
            f"avoid=({numeric(row, 'perceptPlane.avoidX'):.2f},"
            f"{numeric(row, 'perceptPlane.avoidY'):.2f},"
            f"{numeric(row, 'perceptPlane.avoidZ'):.2f}) "
            f"ref_offset={numeric(row, 'perceptPlane.refOff'):.3f} "
            f"plan_vio={numeric(row, 'perceptVio.plan'):.4f} "
            f"state_vio={numeric(row, 'perceptVio.state'):.4f}"
        )
        for y in range(10):
            group = "A" if y < 5 else "B"
            mask = int(numeric(row, f"perceptGrid{group}.r{y}"))
            snapshots.append("".join("#" if mask & (1 << x) else "." for x in range(10)))
        snapshots.append("")
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text("\n".join(snapshots))
    return len(seen)


def print_summary(rows: list[dict[str, object]], map_count: int, apply: bool) -> None:
    flight = [row for row in rows if row.get("phase") == "straight_run"]
    if not flight:
        return
    valid = any(numeric(row, "percept.valid") > 0 for row in flight)
    peak_max = max(numeric(row, "percept.maxDanger") for row in flight)
    peak_center = max(numeric(row, "percept.ctrDanger") for row in flight)
    plane_count = sum(numeric(row, "percept.corridor") > 0 for row in flight)
    applied_count = sum(numeric(row, "percept.applied") > 0 for row in flight)
    held_count = sum(numeric(row, "percept.hold") > 0 for row in flight)
    mass_count = sum(numeric(row, "perceptMass.detected") > 0 for row in flight)
    peak_cells = max(int(numeric(row, "perceptMass.cells")) for row in flight)
    peak_cluster = max(
        int(numeric(row, "perceptMass.cluster")) for row in flight)
    peak_center_q = max(
        int(numeric(row, "perceptMass.centerQ")) for row in flight)
    peak_max_q = max(
        int(numeric(row, "perceptMass.maxQ")) for row in flight)
    peak_constraints = max(int(numeric(row, "percept.nCstr")) for row in flight)
    avoid_samples = sum(int(numeric(row, "percept.mode")) == 1 for row in flight)
    rejoin_samples = sum(int(numeric(row, "percept.mode")) == 2 for row in flight)
    peak_reference_offset = max(
        numeric(row, "perceptPlane.refOff") for row in flight)
    peak_plan_violation = max(numeric(row, "perceptVio.plan") for row in flight)
    peak_state_violation = max(numeric(row, "perceptVio.state") for row in flight)
    peak_mpc_offset = max(
        abs(numeric(row, "mpcDiag.outY") - numeric(row, "cmd_y")) for row in flight)
    peak_vehicle_offset = max(
        abs(numeric(row, "stateEstimate.y") - numeric(row, "cmd_y")) for row in flight)
    yaw_span = (
        max(numeric(row, "stateEstimate.yaw") for row in flight)
        - min(numeric(row, "stateEstimate.yaw") for row in flight)
    )
    print(
        "summary:",
        f"mode={'APPLY' if apply else 'LOG-ONLY'}, maps={map_count}, valid={valid},",
        f"max_danger={peak_max:.3f}, center_danger={peak_center:.3f},",
        f"plane_samples={plane_count}, applied_samples={applied_count}, held_samples={held_count},",
        f"mass_samples={mass_count}, peak_cells={peak_cells},",
        f"peak_cluster={peak_cluster}, peak_center_q={peak_center_q}, peak_max_q={peak_max_q},",
        f"avoid_samples={avoid_samples}, rejoin_samples={rejoin_samples},",
        f"max_constraints={peak_constraints}, ref_offset={peak_reference_offset:.3f} m,",
        f"mpc_lateral_offset={peak_mpc_offset:.3f} m,",
        f"vehicle_lateral_offset={peak_vehicle_offset:.3f} m, yaw_span={yaw_span:.1f} deg,",
        f"plan_violation={peak_plan_violation:.4f} m,",
        f"state_violation={peak_state_violation:.4f} m",
    )


def set_param(cf: Crazyflie, name: str, value: object, delay: float = 0.03) -> None:
    cf.param.set_value(name, str(value))
    time.sleep(delay)


def stream_line(cf: Crazyflie, rows: list[dict[str, object]], latest: dict[str, object],
                seconds: float, start: tuple[float, float, float],
                end: tuple[float, float, float], yaw_deg: float, phase: str) -> None:
    steps = max(1, int(seconds * 50.0))
    for step in range(steps):
        alpha = step / max(1, steps - 1)
        point = tuple(a + alpha * (b - a) for a, b in zip(start, end))
        cf.commander.send_position_setpoint(*point, yaw_deg)
        rows.append({"host_time": time.time(), "phase": phase, "cmd_x": point[0],
                     "cmd_y": point[1], "cmd_z": point[2],
                     "cmd_yaw_deg": yaw_deg, **latest})
        time.sleep(0.02)


def main() -> int:
    args = parse_args()
    rows: list[dict[str, object]] = []
    latest: dict[str, object] = {}
    logs: list[LogConfig] = []
    result = 0
    args.out.parent.mkdir(parents=True, exist_ok=True)
    cflib.crtp.init_drivers()

    with SyncCrazyflie(args.uri, cf=Crazyflie(rw_cache=args.cache_dir)) as scf:
        cf = scf.cf

        def on_log(_timestamp: int, data: dict[str, object], _logconf: LogConfig) -> None:
            latest.update(data)

        for name, period_ms, variables in LOG_BLOCKS:
            config = LogConfig(name=name, period_in_ms=period_ms)
            for variable, kind in variables:
                config.add_variable(variable, kind)
            cf.log.add_config(config)
            config.data_received_cb.add_callback(on_log)
            config.start()
            logs.append(config)

        try:
            time.sleep(1.0)
            battery = float(latest.get("pm.vbat", 0.0) or 0.0)
            if battery and battery < args.min_vbat:
                raise RuntimeError(f"battery {battery:.2f} V < {args.min_vbat:.2f} V")

            set_param(cf, "commander.enHighLevel", 0)
            set_param(cf, "stabilizer.controller", 1)
            set_param(cf, "obs.enable", 0)  # Do not mix in the flow/modelled-cylinder slot.
            set_param(cf, "obs.pidPass", 0)
            set_param(cf, "gateNav.navEn", 0)
            set_param(cf, "circuit.en", 0)
            set_param(cf, "visYaw.useRef", 0)
            set_param(cf, "percept.enable", 1)
            set_param(cf, "percept.constrain", 1)
            set_param(cf, "percept.logOnly", 0 if args.apply else 1)
            set_param(cf, "percept.massCells", max(0, min(255, args.mass_cells)))
            set_param(cf, "percept.dangerQ", max(0, min(255, args.danger_q)))
            set_param(cf, "percept.lineOffset",
                      max(1.0, min(80.0, args.line_offset_px)))
            hold_steps = (
                max(1, (args.hold_ms + 19) // 20)
                if args.hold_ms is not None else args.hold_steps
            )
            set_param(cf, "percept.holdSteps",
                      max(1, min(255, hold_steps)))
            set_param(cf, "percept.persist",
                      max(1, min(255, args.persist_maps)))
            set_param(cf, "percept.safeBand",
                      max(1, min(10, args.safe_band_cells)))
            set_param(cf, "percept.cstrSteps",
                      max(1, min(24, args.constraint_steps)))
            set_param(cf, "percept.clearMaps",
                      max(1, min(255, args.clear_maps)))
            set_param(cf, "percept.rejoin",
                      max(1, min(255, args.rejoin_steps)))
            set_param(cf, "percept.maxLatVel",
                      max(0.05, min(2.0, args.max_lateral_speed)))
            if not args.no_reset_estimator:
                set_param(cf, "kalman.resetEstimation", 1)
                time.sleep(0.1)
                set_param(cf, "kalman.resetEstimation", 0)
                time.sleep(2.0)

            cf.platform.send_arming_request(True)
            time.sleep(0.5)
            start = (args.start_x, args.start_y, args.height)
            stream_line(cf, rows, latest, args.takeoff_s,
                        (args.start_x, args.start_y, 0.05), start, args.yaw_deg, "takeoff")
            stream_line(cf, rows, latest, args.settle_s, start, start, args.yaw_deg, "settle")

            print("switching to TinyMPC; beginning straight reference immediately")
            set_param(cf, "stabilizer.controller", 6, delay=0.15)
            print("running straight neural test in", "APPLY" if args.apply else "LOG-ONLY", "mode")
            stream_line(cf, rows, latest, args.run_s, start,
                        (args.goal_x, args.goal_y, args.height), args.yaw_deg, "straight_run")
            set_param(cf, "stabilizer.controller", 1, delay=0.1)
            landing_start = (float(latest.get("stateEstimate.x", args.goal_x) or args.goal_x),
                             float(latest.get("stateEstimate.y", args.goal_y) or args.goal_y),
                             float(latest.get("stateEstimate.z", args.height) or args.height))
            stream_line(cf, rows, latest, args.land_s, landing_start,
                        (landing_start[0], landing_start[1], 0.05), args.yaw_deg, "land")
        except (KeyboardInterrupt, RuntimeError) as error:
            result = 2
            print(f"ABORT: {error}; landing")
            set_param(cf, "stabilizer.controller", 1, delay=0.05)
            x = float(latest.get("stateEstimate.x", args.start_x) or args.start_x)
            y = float(latest.get("stateEstimate.y", args.start_y) or args.start_y)
            z = float(latest.get("stateEstimate.z", args.height) or args.height)
            stream_line(cf, rows, latest, args.land_s, (x, y, z), (x, y, 0.05), args.yaw_deg, "abort_land")
        finally:
            for action in (lambda: cf.commander.send_stop_setpoint(),
                           lambda: cf.platform.send_arming_request(False),
                           lambda: set_param(cf, "percept.logOnly", 1, delay=0.01),
                           lambda: set_param(cf, "stabilizer.controller", 1, delay=0.01)):
                try:
                    action()
                except Exception:  # Best-effort radio cleanup.
                    pass
            for config in logs:
                try:
                    config.stop()
                except Exception:
                    pass

    fields = sorted({key for row in rows for key in row})
    with args.out.open("w", newline="") as output:
        writer = csv.DictWriter(output, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)
    map_output = args.map_out or args.out.with_suffix(".maps.txt")
    map_count = write_map_report(rows, map_output)
    print_summary(rows, map_count, args.apply)
    print(f"wrote telemetry to {args.out} and {map_count} map snapshots to {map_output}")
    return result


if __name__ == "__main__":
    raise SystemExit(main())
