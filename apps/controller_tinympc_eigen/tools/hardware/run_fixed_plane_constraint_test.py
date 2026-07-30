#!/usr/bin/env python3
"""Fly a straight reference through one fixed world-frame MPC half-space.

The legacy default installs

    x - y <= 0.35 m

while commanding a straight line from (0, 0) to (0.70, 0). A compliant
controller must turn toward positive y or stop before crossing the plane.
Use ``--through-current-angle-deg 15`` for the synthetic image-plane test:
after settling, it anchors a 15-degree boundary at the measured vehicle
position. The centered forward reference is forbidden and its closest
feasible direction advances along the plane toward positive y.
Vision, gate navigation, and modeled-cylinder constraints are disabled.
"""

from __future__ import annotations

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
APP_DIR = Path(__file__).resolve().parents[2]
LOG_BLOCKS = [
    ("state", 50, [
        ("stateEstimate.x", "float"),
        ("stateEstimate.y", "float"),
        ("stateEstimate.z", "float"),
        ("stateEstimate.yaw", "float"),
        ("stateEstimate.vx", "float"),
        ("stateEstimate.vy", "float"),
    ]),
    ("test_plane", 50, [
        ("testPlane.valid", "uint8_t"),
        ("testPlane.applied", "uint8_t"),
        ("testPlane.nCstr", "uint8_t"),
        ("testPlane.worstK", "uint8_t"),
        ("testPlane.ax", "float"),
        ("testPlane.ay", "float"),
        ("testPlane.az", "float"),
        ("testPlane.b", "float"),
    ]),
    ("violations", 50, [
        ("testPlaneVio.plan", "float"),
        ("testPlaneVio.output", "float"),
        ("testPlaneVio.state", "float"),
        ("obs.mpcUs", "uint32_t"),
        ("obs.iter", "uint8_t"),
    ]),
    ("solver_residuals", 50, [
        ("obs.priRes", "float"),
        ("obs.duaRes", "float"),
        ("obs.rho", "float"),
    ]),
    ("task_health", 50, [
        ("obs.taskUs", "uint32_t"),
        ("obs.stackFree", "uint32_t"),
        ("obs.staleWake", "uint32_t"),
        ("obs.dropWake", "uint32_t"),
        ("obs.ageMs", "uint32_t"),
    ]),
    ("mpc_output", 50, [
        ("mpcDiag.outX", "float"),
        ("mpcDiag.outY", "float"),
        ("mpcDiag.outZ", "float"),
        ("mpcDiag.yawCmd", "float"),
        ("mpcDiag.safe", "uint8_t"),
        ("mpcDiag.outVio", "float"),
    ]),
    ("battery", 100, [("pm.vbat", "float")]),
]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--uri", default=uri_helper.uri_from_env(default=DEFAULT_URI))
    parser.add_argument(
        "--out",
        type=Path,
        default=APP_DIR / "sim_runs" / "fixed_plane_01.csv",
    )
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument(
        "--apply", dest="apply", action="store_true",
        help="apply the fixed half-space (default)",
    )
    mode.add_argument(
        "--log-only", dest="apply", action="store_false",
        help="log nominal violations without applying the half-space",
    )
    parser.set_defaults(apply=True)
    parser.add_argument("--ax", type=float, default=1.0)
    parser.add_argument("--ay", type=float, default=-1.0)
    parser.add_argument("--az", type=float, default=0.0)
    parser.add_argument("--b", type=float, default=0.35)
    parser.add_argument(
        "--through-current-angle-deg",
        type=float,
        metavar="DEG",
        help=(
            "ignore --ax/--ay/--az/--b and latch a vertical plane through "
            "the measured settle position; positive angles force +y avoidance"
        ),
    )
    parser.add_argument("--k-start", type=int, default=1)
    parser.add_argument(
        "--iterations", type=int, default=5,
        help="ADMM iterations per 50 Hz solve; firmware clamps this to 1--20",
    )
    parser.add_argument("--height", type=float, default=0.50)
    parser.add_argument("--start-x", type=float, default=0.0)
    parser.add_argument("--start-y", type=float, default=0.0)
    parser.add_argument("--goal-x", type=float, default=0.70)
    parser.add_argument("--goal-y", type=float, default=0.0)
    parser.add_argument("--takeoff-s", type=float, default=2.0)
    parser.add_argument("--settle-s", type=float, default=1.0)
    parser.add_argument("--run-s", type=float, default=7.0)
    parser.add_argument("--land-s", type=float, default=2.0)
    parser.add_argument("--yaw-deg", type=float, default=0.0)
    parser.add_argument("--min-vbat", type=float, default=3.55)
    parser.add_argument(
        "--abort-violation", type=float, default=0.08,
        help="land if measured signed plane violation exceeds this many metres",
    )
    parser.add_argument(
        "--max-lateral", type=float, default=0.65,
        help="land if |y-start_y| exceeds this many metres",
    )
    parser.add_argument("--plan-tol", type=float, default=0.01)
    parser.add_argument("--state-tol", type=float, default=0.05)
    parser.add_argument(
        "--telemetry-timeout-s",
        type=float,
        default=0.50,
        help="abort if no log packet arrives for this long during the test",
    )
    parser.add_argument(
        "--min-forward-progress",
        type=float,
        default=0.35,
        help="minimum +x travel required by the through-current test",
    )
    parser.add_argument(
        "--min-lateral-progress",
        type=float,
        default=0.06,
        help="minimum travel toward the selected lateral side",
    )
    parser.add_argument("--no-reset-estimator", action="store_true")
    parser.add_argument("--cache-dir", default=".cf_cache")
    args = parser.parse_args()

    if args.through_current_angle_deg is not None:
        angle = math.radians(args.through_current_angle_deg)
        if (
            not math.isfinite(angle)
            or abs(args.through_current_angle_deg) < 2.0
            or abs(args.through_current_angle_deg) > 70.0
        ):
            parser.error(
                "--through-current-angle-deg must have magnitude from 2 to 70 degrees"
            )
        # Boundary tangent t=(cos(angle), sin(angle)). The corresponding
        # inequality sin(angle)*dx-cos(angle)*dy <= 0 rejects centered +x
        # motion and selects the signed lateral direction.
        args.ax = math.sin(angle)
        args.ay = -math.cos(angle)
        args.az = 0.0
        args.b = args.ax * args.start_x + args.ay * args.start_y

    norm = math.sqrt(args.ax * args.ax + args.ay * args.ay + args.az * args.az)
    if not math.isfinite(norm) or norm < 1e-4:
        parser.error("the plane normal must be finite and nonzero")
    values = (
        args.b, args.start_x, args.start_y, args.goal_x, args.goal_y,
        args.height, args.abort_violation, args.max_lateral,
        args.min_forward_progress, args.min_lateral_progress,
        args.telemetry_timeout_s,
    )
    if not all(math.isfinite(value) for value in values):
        parser.error("plane, trajectory, and safety arguments must be finite")
    if args.min_forward_progress < 0 or args.min_lateral_progress < 0:
        parser.error("minimum progress thresholds must be nonnegative")
    if args.telemetry_timeout_s <= 0:
        parser.error("--telemetry-timeout-s must be positive")
    start_value = args.ax * args.start_x + args.ay * args.start_y + args.az * args.height
    goal_value = args.ax * args.goal_x + args.ay * args.goal_y + args.az * args.height
    if start_value > args.b + 1e-6:
        parser.error("the takeoff point is already outside the requested half-space")
    if goal_value <= args.b:
        parser.error("the goal does not cross the plane, so it cannot test enforcement")
    if not 1 <= args.k_start <= 24:
        parser.error("--k-start must be between 1 and 24")
    if not 1 <= args.iterations <= 20:
        parser.error("--iterations must be between 1 and 20")
    return args


def numeric(row: dict[str, object], key: str, default: float = 0.0) -> float:
    try:
        return float(row.get(key, default) or default)
    except (TypeError, ValueError):
        return default


def set_param(cf: Crazyflie, name: str, value: object, delay: float = 0.03) -> None:
    cf.param.set_value(name, str(value))
    time.sleep(delay)


def stream_line(
    cf: Crazyflie,
    rows: list[dict[str, object]],
    latest: dict[str, object],
    seconds: float,
    start: tuple[float, float, float],
    end: tuple[float, float, float],
    yaw_deg: float,
    phase: str,
    args: argparse.Namespace,
) -> None:
    steps = max(1, int(seconds * 50.0))
    for step in range(steps):
        alpha = step / max(1, steps - 1)
        point = tuple(a + alpha * (b - a) for a, b in zip(start, end))
        cf.commander.send_position_setpoint(*point, yaw_deg)
        telemetry = {
            key: value for key, value in latest.items()
            if not key.startswith("_")
        }
        row = {
            "host_time": time.time(),
            "phase": phase,
            "cmd_x": point[0],
            "cmd_y": point[1],
            "cmd_z": point[2],
            "cmd_yaw_deg": yaw_deg,
            **telemetry,
        }
        rows.append(row)

        if phase == "plane_test":
            last_log_monotonic = numeric(
                latest, "_last_log_monotonic", time.monotonic())
            telemetry_age = time.monotonic() - last_log_monotonic
            if telemetry_age > args.telemetry_timeout_s:
                raise RuntimeError(
                    f"telemetry stale for {telemetry_age:.2f} s "
                    f"> {args.telemetry_timeout_s:.2f} s"
                )
            battery = numeric(row, "pm.vbat")
            if battery and battery < args.min_vbat:
                raise RuntimeError(
                    f"battery {battery:.2f} V < {args.min_vbat:.2f} V"
                )
            x = numeric(row, "stateEstimate.x", args.start_x)
            y = numeric(row, "stateEstimate.y", args.start_y)
            z = numeric(row, "stateEstimate.z", args.height)
            state_violation = numeric(row, "testPlaneVio.state")
            if numeric(row, "testPlane.applied") and state_violation > args.abort_violation:
                raise RuntimeError(
                    f"measured plane violation {state_violation:.3f} m "
                    f"> {args.abort_violation:.3f} m"
                )
            if abs(y - args.start_y) > args.max_lateral:
                raise RuntimeError(
                    f"lateral excursion {abs(y - args.start_y):.3f} m "
                    f"> {args.max_lateral:.3f} m"
                )
            if z < 0.20 or z > args.height + 0.35:
                raise RuntimeError(f"unsafe test altitude z={z:.3f} m")
            if x > max(args.start_x, args.goal_x) + 0.25:
                raise RuntimeError(f"forward excursion x={x:.3f} m")
        time.sleep(0.02)


def write_csv(rows: list[dict[str, object]], output: Path) -> None:
    output.parent.mkdir(parents=True, exist_ok=True)
    fields = sorted({key for row in rows for key in row})
    with output.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def print_summary(
    rows: list[dict[str, object]],
    args: argparse.Namespace,
) -> bool:
    flight = [row for row in rows if row.get("phase") == "plane_test"]
    applied = [row for row in flight if numeric(row, "testPlane.applied") > 0]
    if not applied:
        print("result: FAIL — the fixed plane was never applied")
        return False

    peak_plan = max(numeric(row, "testPlaneVio.plan") for row in applied)
    peak_output = max(numeric(row, "testPlaneVio.output") for row in applied)
    peak_state = max(numeric(row, "testPlaneVio.state") for row in applied)
    max_iter = max(int(numeric(row, "obs.iter")) for row in applied)
    max_solve_us = max(int(numeric(row, "obs.mpcUs")) for row in applied)
    max_constraints = max(int(numeric(row, "testPlane.nCstr")) for row in applied)
    max_mpc_y = max(numeric(row, "mpcDiag.outY") for row in applied)
    min_mpc_y = min(numeric(row, "mpcDiag.outY") for row in applied)
    projected_fallbacks = sum(
        numeric(row, "mpcDiag.safe") == 0 for row in applied)
    peak_output_knot_violation = max(
        numeric(row, "mpcDiag.outVio") for row in applied)
    peak_primal_residual = max(numeric(row, "obs.priRes") for row in applied)
    peak_dual_residual = max(numeric(row, "obs.duaRes") for row in applied)
    final_primal_residual = numeric(applied[-1], "obs.priRes")
    final_dual_residual = numeric(applied[-1], "obs.duaRes")
    rho_values = [numeric(row, "obs.rho") for row in applied]
    peak_task_us = max(int(numeric(row, "obs.taskUs")) for row in applied)
    stack_free_values = [
        int(numeric(row, "obs.stackFree")) for row in applied
        if numeric(row, "obs.stackFree") > 0
    ]
    minimum_stack_free = min(stack_free_values) if stack_free_values else 0
    stale_wakes = max(int(numeric(row, "obs.staleWake")) for row in applied)
    dropped_wakes = max(int(numeric(row, "obs.dropWake")) for row in applied)
    peak_plan_age_ms = max(int(numeric(row, "obs.ageMs")) for row in applied)
    max_state_y = max(numeric(row, "stateEstimate.y") for row in applied)
    min_state_y = min(numeric(row, "stateEstimate.y") for row in applied)

    solver_pass = peak_plan <= args.plan_tol and peak_output <= args.plan_tol
    tracking_pass = peak_state <= args.state_tol
    avoidance_pass = True
    avoidance_summary = ""
    if args.through_current_angle_deg is not None:
        anchor_x = getattr(args, "anchor_x", args.start_x)
        anchor_y = getattr(args, "anchor_y", args.start_y)
        lateral_sign = 1.0 if args.through_current_angle_deg > 0 else -1.0
        forward_progress = max(
            numeric(row, "stateEstimate.x") - anchor_x for row in applied
        )
        lateral_progress = max(
            lateral_sign * (numeric(row, "stateEstimate.y") - anchor_y)
            for row in applied
        )
        avoidance_pass = (
            forward_progress >= args.min_forward_progress
            and lateral_progress >= args.min_lateral_progress
        )
        avoidance_summary = (
            f" avoidance={'PASS' if avoidance_pass else 'FAIL'} "
            f"(forward={forward_progress:.3f}/{args.min_forward_progress:.3f} m, "
            f"lateral={lateral_progress:.3f}/{args.min_lateral_progress:.3f} m)"
        )
    print(
        "summary:",
        f"constraints={max_constraints}, iterations={max_iter}, "
        f"max_solve={max_solve_us} us,",
        f"plan_violation={peak_plan:.4f} m, "
        f"output_violation={peak_output:.4f} m, "
        f"state_violation={peak_state:.4f} m,",
        f"raw_output_knot_violation={peak_output_knot_violation:.4f} m, "
        f"projected_fallback_samples={projected_fallbacks},",
        f"rho=[{min(rho_values):.1f}, {max(rho_values):.1f}], "
        f"residual_final=({final_primal_residual:.4f}, "
        f"{final_dual_residual:.4f}), "
        f"residual_peak=({peak_primal_residual:.4f}, "
        f"{peak_dual_residual:.4f}),",
        f"task_max={peak_task_us} us, stack_min={minimum_stack_free} words, "
        f"stale_wakes={stale_wakes}, dropped_wakes={dropped_wakes}, "
        f"plan_age_max={peak_plan_age_ms} ms,",
        f"mpc_y=[{min_mpc_y:.3f}, {max_mpc_y:.3f}] m, "
        f"state_y=[{min_state_y:.3f}, {max_state_y:.3f}] m",
    )
    print(
        "result:",
        f"solver={'PASS' if solver_pass else 'FAIL'} "
        f"(tol={args.plan_tol:.3f} m),",
        f"tracking={'PASS' if tracking_pass else 'FAIL'} "
        f"(tol={args.state_tol:.3f} m),"
        f"{avoidance_summary}",
    )
    return solver_pass and tracking_pass and avoidance_pass


def main() -> int:
    args = parse_args()
    rows: list[dict[str, object]] = []
    latest: dict[str, object] = {}
    logs: list[LogConfig] = []
    result = 0
    cflib.crtp.init_drivers()

    norm = math.sqrt(args.ax * args.ax + args.ay * args.ay + args.az * args.az)
    print(
        "requested plane:",
        f"{args.ax / norm:.4f} x + {args.ay / norm:.4f} y + "
        f"{args.az / norm:.4f} z <= {args.b / norm:.4f}",
    )

    with SyncCrazyflie(args.uri, cf=Crazyflie(rw_cache=args.cache_dir)) as scf:
        cf = scf.cf

        def on_log(
            _timestamp: int,
            data: dict[str, object],
            _logconf: LogConfig,
        ) -> None:
            latest.update(data)
            latest["_last_log_monotonic"] = time.monotonic()

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
            battery = numeric(latest, "pm.vbat")
            if battery and battery < args.min_vbat:
                raise RuntimeError(
                    f"battery {battery:.2f} V < {args.min_vbat:.2f} V"
                )

            set_param(cf, "commander.enHighLevel", 0)
            set_param(cf, "stabilizer.controller", 1)
            set_param(cf, "obs.enable", 0)
            set_param(cf, "obs.pidPass", 0)
            set_param(cf, "percept.enable", 0)
            set_param(cf, "percept.constrain", 0)
            set_param(cf, "gateNav.navEn", 0)
            set_param(cf, "circuit.en", 0)
            set_param(cf, "visYaw.useRef", 0)
            set_param(cf, "testPlane.enable", 0)
            set_param(cf, "testPlane.logOnly", 0 if args.apply else 1)
            set_param(cf, "testPlane.ax", args.ax)
            set_param(cf, "testPlane.ay", args.ay)
            set_param(cf, "testPlane.az", args.az)
            set_param(cf, "testPlane.b", args.b)
            set_param(cf, "testPlane.kStart", args.k_start)
            set_param(cf, "testPlane.maxIter", args.iterations)

            if not args.no_reset_estimator:
                set_param(cf, "kalman.resetEstimation", 1)
                time.sleep(0.1)
                set_param(cf, "kalman.resetEstimation", 0)
                time.sleep(2.0)

            cf.platform.send_arming_request(True)
            time.sleep(0.5)
            start = (args.start_x, args.start_y, args.height)
            stream_line(
                cf, rows, latest, args.takeoff_s,
                (args.start_x, args.start_y, 0.05), start,
                args.yaw_deg, "takeoff", args,
            )
            stream_line(
                cf, rows, latest, args.settle_s, start, start,
                args.yaw_deg, "settle", args,
            )

            if args.through_current_angle_deg is not None:
                anchor_x = numeric(latest, "stateEstimate.x", args.start_x)
                anchor_y = numeric(latest, "stateEstimate.y", args.start_y)
                anchor_z = numeric(latest, "stateEstimate.z", args.height)
                args.anchor_x = anchor_x
                args.anchor_y = anchor_y
                args.b = (
                    args.ax * anchor_x
                    + args.ay * anchor_y
                    + args.az * anchor_z
                )
                set_param(cf, "testPlane.ax", args.ax)
                set_param(cf, "testPlane.ay", args.ay)
                set_param(cf, "testPlane.az", args.az)
                set_param(cf, "testPlane.b", args.b)
                print(
                    "latched plane through measured position:",
                    f"p=({anchor_x:.3f}, {anchor_y:.3f}, {anchor_z:.3f}),",
                    f"{args.ax:.4f} x + {args.ay:.4f} y <= {args.b:.4f}",
                )

            set_param(cf, "testPlane.enable", 1)
            print(
                "switching to TinyMPC; fixed plane is",
                "APPLIED" if args.apply else "LOG-ONLY",
            )
            set_param(cf, "stabilizer.controller", 6, delay=0.15)
            stream_line(
                cf, rows, latest, args.run_s, start,
                (args.goal_x, args.goal_y, args.height),
                args.yaw_deg, "plane_test", args,
            )

            set_param(cf, "stabilizer.controller", 1, delay=0.1)
            set_param(cf, "testPlane.enable", 0)
            landing_start = (
                numeric(latest, "stateEstimate.x", args.goal_x),
                numeric(latest, "stateEstimate.y", args.goal_y),
                numeric(latest, "stateEstimate.z", args.height),
            )
            stream_line(
                cf, rows, latest, args.land_s, landing_start,
                (landing_start[0], landing_start[1], 0.05),
                args.yaw_deg, "land", args,
            )
        except (KeyboardInterrupt, RuntimeError) as error:
            result = 2
            print(f"ABORT: {error}; landing")
            set_param(cf, "stabilizer.controller", 1, delay=0.05)
            set_param(cf, "testPlane.enable", 0, delay=0.02)
            position = (
                numeric(latest, "stateEstimate.x", args.start_x),
                numeric(latest, "stateEstimate.y", args.start_y),
                numeric(latest, "stateEstimate.z", args.height),
            )
            stream_line(
                cf, rows, latest, args.land_s, position,
                (position[0], position[1], 0.05),
                args.yaw_deg, "abort_land", args,
            )
        finally:
            for action in (
                lambda: cf.commander.send_stop_setpoint(),
                lambda: cf.platform.send_arming_request(False),
                lambda: set_param(cf, "testPlane.enable", 0, delay=0.01),
                lambda: set_param(cf, "stabilizer.controller", 1, delay=0.01),
            ):
                try:
                    action()
                except Exception:
                    pass
            for config in logs:
                try:
                    config.stop()
                except Exception:
                    pass

    write_csv(rows, args.out)
    if result == 0 and args.apply and not print_summary(rows, args):
        result = 1
    elif result == 0:
        print_summary(rows, args)
    print(f"wrote telemetry to {args.out}")
    return result


if __name__ == "__main__":
    raise SystemExit(main())
