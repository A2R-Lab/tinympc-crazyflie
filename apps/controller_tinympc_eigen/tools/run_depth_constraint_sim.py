#!/usr/bin/env python3
"""Closed-loop PyBullet obstacle-avoidance experiment using depth-sector constraints.

This runner keeps the first closed-loop experiment deliberately small:

* PyBullet provides the plant and obstacle geometry.
* Simulated sector-depth rays provide the "vision" input.
* The controller turns the selected sector into a TinyMPC-shaped half-space
  ``a^T p <= b``.
* ``--control-mode admm`` compiles a host TinyMPC-ADMM shared library, writes
  ``a_hs/b_hs/en_hs`` into ``tiny_AdmmData``, and executes consecutive planned
  state references between slower MPC replans.
* ``--control-mode projection`` keeps the earlier velocity-lookahead projection
  experiment for comparison.

This is a firmware-shadow experiment for the ADMM half-space path, not a full
Crazyflie motor/controller replica.
"""

from __future__ import annotations

import argparse
import ctypes
import csv
import json
import math
from dataclasses import dataclass
from pathlib import Path
import subprocess
import sys
import tempfile
from typing import Any

import numpy as np


VISION_ROOT = Path(__file__).resolve().parents[4] / "tinympc-vision"
APP_ROOT = Path(__file__).resolve().parents[1]
if str(VISION_ROOT) not in sys.path:
    sys.path.insert(0, str(VISION_ROOT))

from closed_loop.gym_pybullet_gate_env import GymPybulletGateEnv, GymPybulletGateEnvConfig  # noqa: E402
from closed_loop.packets import GateGeometry  # noqa: E402


R_CB = np.asarray(
    [
        [0.0, -1.0, 0.0],
        [0.0, 0.0, -1.0],
        [1.0, 0.0, 0.0],
    ],
    dtype=np.float64,
)
R_BC = R_CB.T


@dataclass(frozen=True)
class BoxObstacle:
    name: str
    center: np.ndarray
    half_extents: np.ndarray


@dataclass(frozen=True)
class SectorHit:
    sector: int
    azimuth_rad: float
    depth_m: float
    confidence: float
    obstacle_name: str
    obstacle_center: np.ndarray
    ray_world: np.ndarray


@dataclass(frozen=True)
class HalfspaceConstraint:
    active: bool
    a: np.ndarray
    b: float
    depth_m: float
    margin_m: float
    sector: int
    azimuth_rad: float
    confidence: float
    obstacle_name: str
    obstacle_center: np.ndarray
    current_slack_m: float
    status: str
    age_frames: int = 0


@dataclass(frozen=True)
class ReferenceTrajectory:
    t: np.ndarray
    pos: np.ndarray
    vel: np.ndarray

    def sample(self, query_t: float) -> tuple[np.ndarray, np.ndarray]:
        tt = np.asarray(self.t, dtype=np.float64)
        q = float(np.clip(float(query_t), float(tt[0]), float(tt[-1])))
        pos = np.asarray([np.interp(q, tt, self.pos[:, axis]) for axis in range(3)], dtype=np.float64)
        vel = np.asarray([np.interp(q, tt, self.vel[:, axis]) for axis in range(3)], dtype=np.float64)
        return pos, vel

    @property
    def final_position(self) -> np.ndarray:
        return np.asarray(self.pos[-1], dtype=np.float64)


class AdmmHostSolver:
    NSTATES = 12
    NINPUTS = 4
    MAX_HS = 3

    def __init__(self, max_iter: int, rho: float, horizon: int, model_dt_s: float,
                 force_rebuild: bool = False) -> None:
        self._lib = ctypes.CDLL(str(_build_admm_host_library(horizon, force=force_rebuild)))
        self._configure_abi()
        if not self._lib.tinympc_admm_host_init(
            int(max_iter), ctypes.c_float(float(rho)), True, ctypes.c_float(float(model_dt_s))
        ):
            raise RuntimeError("tinympc_admm_host_init failed")
        self.horizon = int(self._lib.tinympc_admm_host_horizon())
        if self.horizon != int(horizon):
            raise RuntimeError(f"host solver horizon {self.horizon} != requested {horizon}")

    def _configure_abi(self) -> None:
        f32p = ctypes.POINTER(ctypes.c_float)
        i32p = ctypes.POINTER(ctypes.c_int)
        self._lib.tinympc_admm_host_init.argtypes = [ctypes.c_int, ctypes.c_float, ctypes.c_bool, ctypes.c_float]
        self._lib.tinympc_admm_host_init.restype = ctypes.c_bool
        self._lib.tinympc_admm_host_reset_duals.argtypes = []
        self._lib.tinympc_admm_host_reset_duals.restype = None
        self._lib.tinympc_admm_host_solve.argtypes = [
            f32p,
            f32p,
            f32p,
            f32p,
            f32p,
            i32p,
            ctypes.c_int,
            f32p,
            f32p,
            i32p,
            i32p,
            f32p,
            f32p,
        ]
        self._lib.tinympc_admm_host_solve.restype = ctypes.c_bool

    def solve(
        self,
        x0: np.ndarray,
        x_ref: np.ndarray,
        constraints: list[HalfspaceConstraint],
        start_k: int,
        end_k: int,
        enable_constraints: bool,
    ) -> dict[str, Any]:
        x0_f = np.ascontiguousarray(np.asarray(x0, dtype=np.float32).reshape(self.NSTATES))
        xref_f = np.ascontiguousarray(np.asarray(x_ref, dtype=np.float32).reshape(self.horizon, self.NSTATES))
        uref_f = np.zeros((self.horizon - 1, self.NINPUTS), dtype=np.float32)
        a_hs = np.zeros((self.horizon, self.MAX_HS, 3), dtype=np.float32)
        b_hs = np.zeros((self.horizon, self.MAX_HS), dtype=np.float32)
        en_hs = np.zeros((self.horizon, self.MAX_HS), dtype=np.int32)
        active_constraints = [constraint for constraint in constraints[: self.MAX_HS] if constraint.active]
        for h, constraint in enumerate(active_constraints):
            for k in range(max(0, int(start_k)), min(int(end_k), self.horizon)):
                a_hs[k, h, :] = np.asarray(constraint.a, dtype=np.float32)
                b_hs[k, h] = np.float32(constraint.b)
                en_hs[k, h] = 1
        x_out = np.zeros((self.horizon, self.NSTATES), dtype=np.float32)
        u_out = np.zeros((self.horizon - 1, self.NINPUTS), dtype=np.float32)
        status = ctypes.c_int(0)
        iters = ctypes.c_int(0)
        pri = ctypes.c_float(0.0)
        dua = ctypes.c_float(0.0)
        ok = self._lib.tinympc_admm_host_solve(
            x0_f.ctypes.data_as(ctypes.POINTER(ctypes.c_float)),
            xref_f.ctypes.data_as(ctypes.POINTER(ctypes.c_float)),
            uref_f.ctypes.data_as(ctypes.POINTER(ctypes.c_float)),
            a_hs.ctypes.data_as(ctypes.POINTER(ctypes.c_float)),
            b_hs.ctypes.data_as(ctypes.POINTER(ctypes.c_float)),
            en_hs.ctypes.data_as(ctypes.POINTER(ctypes.c_int)),
            int(bool(enable_constraints and active_constraints)),
            x_out.ctypes.data_as(ctypes.POINTER(ctypes.c_float)),
            u_out.ctypes.data_as(ctypes.POINTER(ctypes.c_float)),
            ctypes.byref(status),
            ctypes.byref(iters),
            ctypes.byref(pri),
            ctypes.byref(dua),
        )
        if not ok:
            raise RuntimeError("tinympc_admm_host_solve failed")
        return {
            "states": np.asarray(x_out, dtype=np.float64),
            "controls": np.asarray(u_out, dtype=np.float64),
            "status": int(status.value),
            "iterations": int(iters.value),
            "pri_res": float(pri.value),
            "dua_res": float(dua.value),
        }


def parse_args() -> argparse.Namespace:
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", type=Path, default=Path("flow_sim_dataset/depth_constraint_closed_loop"))
    ap.add_argument("--duration", type=float, default=7.0)
    ap.add_argument("--plant-dt", "--dt", dest="plant_dt", type=float, default=0.002,
                    help="PyBullet integration and acceleration-shim step [s]")
    ap.add_argument("--model-dt", type=float, default=0.04,
                    help="MPC discrete-model knot interval [s]; regenerated from the continuous hover model")
    ap.add_argument("--mpc-rate-hz", type=float, default=5.0,
                    help="receding-horizon solve rate; five 0.04 s plan knots are executed at the 5 Hz default")
    ap.add_argument("--gui", action="store_true")
    ap.add_argument("--image-size", type=int, default=160)
    ap.add_argument("--fov-deg", type=float, default=70.0)
    ap.add_argument("--sectors", type=int, default=7)
    ap.add_argument("--control-mode", choices=["admm", "projection"], default="admm")
    ap.add_argument("--max-active-halfspaces", type=int, default=1,
                    help="number of sector half-spaces to pass to ADMM; capped by host MAX_HS=3")
    ap.add_argument("--tinympc-max-iter", type=int, default=2,
                    help="ADMM iterations per MPC tick; default matches src/controller_tinympc.cpp")
    ap.add_argument("--tinympc-rho", type=float, default=250.0)
    ap.add_argument("--target-speed", type=float, default=0.18)
    ap.add_argument("--lookahead-s", type=float, default=0.75)
    ap.add_argument("--velocity-tau-s", type=float, default=0.35)
    ap.add_argument("--lateral-kp", type=float, default=0.9)
    ap.add_argument("--vertical-kp", type=float, default=2.0)
    ap.add_argument("--margin-min", type=float, default=0.85)
    ap.add_argument("--margin-slack", type=float, default=0.25)
    ap.add_argument("--sidestep-gain", type=float, default=0.35,
                    help="lateral velocity bias added when the lookahead violates a constraint")
    ap.add_argument("--admm-reference-sidestep", type=float, default=0.0,
                    help="lateral goal offset away from the active obstacle in ADMM mode [m]; <=0 disables")
    ap.add_argument("--drone-clearance-radius", type=float, default=0.10,
                    help="clearance radius subtracted from point-to-box distance in logs/termination")
    ap.add_argument("--min-depth", type=float, default=0.20)
    ap.add_argument("--max-depth", type=float, default=4.0)
    ap.add_argument("--min-confidence", type=float, default=0.10)
    ap.add_argument("--camera-rate-hz", type=float, default=20.0,
                    help="sector-depth update rate; MPC ticks hold the latest constraint between camera frames")
    ap.add_argument("--constraint-max-age-s", type=float, default=0.20,
                    help="disable held sector constraints older than this many seconds")
    ap.add_argument("--depth-noise-std", type=float, default=0.0,
                    help="Gaussian depth noise applied to simulated sector depth [m]")
    ap.add_argument("--depth-bias", type=float, default=0.0,
                    help="constant depth bias applied to simulated sector depth [m]")
    ap.add_argument("--sector-dropout-prob", type=float, default=0.0,
                    help="probability that a camera frame returns no sector hit")
    ap.add_argument("--false-hit-prob", type=float, default=0.0,
                    help="probability that a camera frame returns a false short-depth sector")
    ap.add_argument("--false-hit-depth", type=float, default=0.7,
                    help="nominal false-hit depth [m]")
    ap.add_argument("--sector-switch-ratio", type=float, default=1.0,
                    help="while a sector is active, switch sectors only if new depth is below ratio*held_depth; >=1 disables")
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--start-k", type=int, default=1,
                    help="first horizon knot with an enabled obstacle half-space")
    ap.add_argument("--constraint-end-k", type=int, default=11,
                    help="exclusive final constrained knot; default constrains ten future knots of a 20-knot horizon")
    ap.add_argument("--horizon", type=int, default=20)
    ap.add_argument("--max-accel", type=float, default=0.8)
    ap.add_argument("--max-forward-speed", type=float, default=0.6)
    ap.add_argument("--max-lateral-speed", type=float, default=0.25)
    ap.add_argument("--max-command-delta", type=float, default=0.0,
                    help="limit per-tick change in commanded velocity norm [m/s]; <=0 disables")
    ap.add_argument("--admm-forward-slack-scale", type=float, default=0.0,
                    help="in ADMM mode, cap forward velocity to scale*constraint_slack/lookahead; <=0 disables")
    ap.add_argument("--admm-safety-filter", action=argparse.BooleanOptionalAction, default=False,
                    help="after ADMM, project the command lookahead back into the active half-space")
    ap.add_argument("--trajectory-file", type=Path, default=None,
                    help="CSV trajectory with columns t,x,y,z and optional vx,vy,vz; defaults to the gate target")
    ap.add_argument("--goal-tolerance", type=float, default=0.18,
                    help="position tolerance for considering a trajectory final waypoint reached [m]")
    ap.add_argument("--gate-x", type=float, default=3.0)
    ap.add_argument("--gate-y", type=float, default=0.0)
    ap.add_argument("--gate-z", type=float, default=1.1)
    ap.add_argument("--admm-min-target-z", type=float, default=-math.inf,
                    help="clamp ADMM terminal target z before converting to the sim velocity command")
    ap.add_argument("--admm-max-target-z", type=float, default=math.inf,
                    help="clamp ADMM terminal target z before converting to the sim velocity command")
    ap.add_argument("--initial-x", type=float, default=0.0)
    ap.add_argument("--initial-y", type=float, default=0.0)
    ap.add_argument("--initial-z", type=float, default=1.1)
    ap.add_argument("--obstacle", action="append", default=None,
                    help="box as name,x,y,z,hx,hy,hz; may be repeated")
    ap.add_argument("--no-avoidance", action="store_true",
                    help="log constraints but do not alter the nominal command")
    ap.add_argument("--force-rebuild-solver", action="store_true")
    ap.add_argument("--overwrite", action="store_true")
    return ap.parse_args()


def main() -> int:
    args = parse_args()
    _validate_timing(args)
    if args.out.exists() and not args.overwrite:
        raise SystemExit(f"{args.out} exists; use --overwrite or choose --out")
    args.out.mkdir(parents=True, exist_ok=True)

    geometry = GateGeometry(
        gate_x=float(args.gate_x),
        gate_y=float(args.gate_y),
        gate_z=float(args.gate_z),
        half_width=0.55,
        half_height=0.50,
        drone_radius=0.03,
    )
    trajectory = _load_reference_trajectory(args, geometry)
    env_config = GymPybulletGateEnvConfig(
        dt=float(args.plant_dt),
        image_size=int(args.image_size),
        fov_deg=float(args.fov_deg),
        max_accel=float(args.max_accel),
        initial_x=float(args.initial_x),
        initial_y=float(args.initial_y),
        initial_z=float(args.initial_z),
        initial_vx=0.0,
        initial_vy=0.0,
        initial_vz=0.0,
        ctrl_freq=int(round(1.0 / float(args.plant_dt))),
        gui=bool(args.gui),
        max_forward_speed=float(args.max_forward_speed),
        max_lateral_speed=float(args.max_lateral_speed),
        max_vertical_speed=0.7,
        command_lookahead_s=0.55,
        target_y_limit=1.4,
        target_z_min=0.45,
        target_z_max=2.0,
        gate_collision=False,
    )
    env = GymPybulletGateEnv(geometry=geometry, config=env_config)
    obstacles = _parse_obstacles(args.obstacle)
    admm_solver = (
        AdmmHostSolver(
            args.tinympc_max_iter,
            args.tinympc_rho,
            horizon=int(args.horizon),
            model_dt_s=float(args.model_dt),
            force_rebuild=bool(args.force_rebuild_solver),
        )
        if str(args.control_mode) == "admm"
        else None
    )
    rows: list[list[object]] = []
    horizon_rows: list[list[object]] = []
    plan_rows: list[list[object]] = []
    summary: dict[str, Any] = {}
    rng = np.random.default_rng(int(args.seed))

    try:
        env.reset()
        _add_obstacles(env, obstacles)
        steps = int(round(float(args.duration) / float(args.plant_dt)))
        camera_period_steps = _period_steps(1.0 / max(1e-6, float(args.camera_rate_hz)), args.plant_dt, "camera")
        mpc_period_steps = _period_steps(1.0 / float(args.mpc_rate_hz), args.plant_dt, "MPC")
        model_period_steps = _period_steps(float(args.model_dt), args.plant_dt, "model")
        max_constraint_age_steps = max(0, int(round(float(args.constraint_max_age_s) / float(args.plant_dt))))
        held_constraints = [_inactive_constraint("no_depth")]
        held_age_steps = max_constraint_age_steps + 1
        min_clearance = math.inf
        active_count = 0
        collision = False
        obstacle_collision = False
        env_collision = False
        reached_goal = False
        min_z = math.inf
        max_command_jump = 0.0
        max_terminal_jump = 0.0
        prev_command_velocity: np.ndarray | None = None
        prev_terminal: np.ndarray | None = None
        dropout_count = 0
        false_hit_count = 0
        stale_disabled_count = 0
        switch_suppressed_count = 0
        mpc_solve_count = 0
        held_plan: np.ndarray | None = None
        held_solver_info: dict[str, Any] = {}
        plan_k = 0
        for step in range(steps):
            t = step * float(args.plant_dt)
            state = env.get_state()
            quat = _env_quat(env)
            rot_wb = _quat_xyzw_to_rot(quat)
            if step % camera_period_steps == 0:
                hits, hit_source = _sample_sector_hits(state[:3], rot_wb, obstacles, args, rng)
                if hit_source == "dropout":
                    dropout_count += 1
                elif hit_source == "false_hit":
                    false_hit_count += 1
                if hits:
                    should_switch = _should_switch_sector(held_constraints[0], hits[0], args)
                    if should_switch or int(args.max_active_halfspaces) > 1:
                        held_constraints = [_constraint_from_hit(state[:3], rot_wb, hit, args) for hit in hits[: max(1, int(args.max_active_halfspaces))]]
                        held_age_steps = 0
                    else:
                        switch_suppressed_count += 1
            constraints = [_aged_constraint(item, held_age_steps, state[:3]) for item in held_constraints]
            if held_age_steps > max_constraint_age_steps:
                constraints = [_inactive_constraint("stale_depth")]
                stale_disabled_count += 1
            constraint = constraints[0] if constraints else _inactive_constraint("no_depth")
            nominal_velocity = _nominal_velocity(state, t, trajectory, args)
            solver_info: dict[str, Any] = {}
            mpc_solved = False
            if admm_solver is not None:
                if step % mpc_period_steps == 0:
                    x0_solver = _solver_state(env, state)
                    x_ref = _reference_trajectory(x0_solver, t, trajectory, constraint, args, admm_solver.horizon)
                    held_solver_info = admm_solver.solve(
                        x0=x0_solver,
                        x_ref=x_ref,
                        constraints=constraints,
                        start_k=int(args.start_k),
                        end_k=int(args.constraint_end_k),
                        enable_constraints=not bool(args.no_avoidance),
                    )
                    held_plan = np.asarray(held_solver_info["states"], dtype=np.float64)
                    plan_k = 0
                    mpc_solved = True
                    mpc_solve_count += 1
                elif held_plan is not None:
                    plan_k = min(plan_k + int(step % model_period_steps == 0), admm_solver.horizon - 1)
                if held_plan is None:
                    raise RuntimeError("ADMM plan was not initialized")
                solver_info = held_solver_info
                command_velocity, violation = _velocity_from_admm_plan(
                    state, held_plan, constraint, args, plan_k=plan_k
                )
                if bool(args.admm_safety_filter):
                    command_velocity, violation = _project_velocity(command_velocity, state[:3], constraint, args)
            else:
                command_velocity, violation = _project_velocity(nominal_velocity, state[:3], constraint, args)
            if args.no_avoidance:
                command_velocity = nominal_velocity
                violation = 0.0
            if prev_command_velocity is not None and float(args.max_command_delta) > 0.0:
                command_velocity = _limit_command_delta(command_velocity, prev_command_velocity, float(args.max_command_delta))
            if prev_command_velocity is not None:
                max_command_jump = max(max_command_jump, float(np.linalg.norm(command_velocity - prev_command_velocity)))
            prev_command_velocity = command_velocity.copy()
            if mpc_solved and solver_info.get("states") is not None:
                terminal = np.asarray(solver_info["states"], dtype=np.float64)[-1, 0:3]
                if prev_terminal is not None and np.all(np.isfinite(terminal)) and np.all(np.isfinite(prev_terminal)):
                    max_terminal_jump = max(max_terminal_jump, float(np.linalg.norm(terminal - prev_terminal)))
                prev_terminal = terminal.copy()
            accel = (command_velocity - state[3:6]) / max(1e-3, float(args.velocity_tau_s))
            accel = np.clip(accel, -float(args.max_accel), float(args.max_accel))
            _, _, done, info = env.step(accel)
            next_state = env.get_state()
            clearance = _clearance_to_obstacles(next_state[:3], obstacles, float(args.drone_clearance_radius))
            min_clearance = min(min_clearance, clearance)
            min_z = min(min_z, float(next_state[2]))
            obstacle_collision = bool(obstacle_collision or clearance < 0.0)
            env_collision = bool(env_collision or info.get("collision", False))
            collision = bool(obstacle_collision or env_collision)
            goal_error = float(np.linalg.norm(next_state[:3] - trajectory.final_position))
            reached_goal = bool(reached_goal or (t >= float(trajectory.t[-1]) and goal_error <= float(args.goal_tolerance)))
            if constraint.active:
                active_count += 1
            rows.append(_log_row(
                step, t, state, next_state, nominal_velocity, command_velocity, accel,
                constraint, violation, clearance, info, solver_info, mpc_solved, plan_k,
            ))
            if mpc_solved:
                _append_horizon_rows(horizon_rows, step, t, constraint, args)
                _append_plan_rows(plan_rows, step, t, solver_info)
            if collision or reached_goal:
                break
            held_age_steps += 1

        final_state = env.get_state()
        summary = {
            "steps": len(rows),
            "sim_time_s": len(rows) * float(args.plant_dt),
            "plant_dt_s": float(args.plant_dt),
            "model_dt_s": float(args.model_dt),
            "mpc_rate_hz": float(args.mpc_rate_hz),
            "horizon": int(args.horizon),
            "constraint_start_k": int(args.start_k),
            "constraint_end_k": int(args.constraint_end_k),
            "mpc_solve_count": int(mpc_solve_count),
            "control_mode": str(args.control_mode),
            "avoidance_enabled": not bool(args.no_avoidance),
            "constraint_active_steps": active_count,
            "constraint_active_fraction": active_count / max(1, len(rows)),
            "max_active_halfspaces": int(args.max_active_halfspaces),
            "camera_rate_hz": float(args.camera_rate_hz),
            "depth_noise_std": float(args.depth_noise_std),
            "depth_bias": float(args.depth_bias),
            "sector_dropout_prob": float(args.sector_dropout_prob),
            "false_hit_prob": float(args.false_hit_prob),
            "dropout_count": int(dropout_count),
            "false_hit_count": int(false_hit_count),
            "stale_disabled_count": int(stale_disabled_count),
            "switch_suppressed_count": int(switch_suppressed_count),
            "max_command_jump_mps": float(max_command_jump),
            "max_terminal_jump_m": float(max_terminal_jump),
            "collision": bool(collision),
            "obstacle_collision": bool(obstacle_collision),
            "env_collision": bool(env_collision),
            "reached_goal": bool(reached_goal),
            "reached_goal_x": bool(reached_goal),
            "goal": {
                "x": float(trajectory.final_position[0]),
                "y": float(trajectory.final_position[1]),
                "z": float(trajectory.final_position[2]),
            },
            "min_obstacle_clearance_m": float(min_clearance),
            "min_z_m": float(min_z),
            "final_state": final_state.astype(float).tolist(),
            "obstacles": [
                {
                    "name": obstacle.name,
                    "center": obstacle.center.astype(float).tolist(),
                    "half_extents": obstacle.half_extents.astype(float).tolist(),
                }
                for obstacle in obstacles
            ],
        }
    finally:
        env.close()

    _write_log(args.out / "closed_loop.csv", rows)
    _write_horizon(args.out / "constraints.csv", horizon_rows)
    _write_plan(args.out / "planned_horizon.csv", plan_rows)
    (args.out / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    print(json.dumps(summary, indent=2))
    print(f"wrote logs to {args.out}")
    return 0


def _parse_obstacles(raw_obstacles: list[str] | None) -> list[BoxObstacle]:
    if not raw_obstacles:
        raw_obstacles = [
            "center_box,1.55,0.18,1.10,0.16,0.22,0.28",
        ]
    out = []
    for raw in raw_obstacles:
        parts = [part.strip() for part in raw.split(",")]
        if len(parts) != 7:
            raise SystemExit(f"bad --obstacle {raw!r}; expected name,x,y,z,hx,hy,hz")
        name = parts[0]
        values = [float(v) for v in parts[1:]]
        out.append(
            BoxObstacle(
                name=name,
                center=np.asarray(values[:3], dtype=np.float64),
                half_extents=np.asarray(values[3:], dtype=np.float64),
            )
        )
    return out


def _load_reference_trajectory(args: argparse.Namespace, geometry: GateGeometry) -> ReferenceTrajectory:
    if args.trajectory_file is None:
        duration = max(float(args.duration), float(args.plant_dt))
        return ReferenceTrajectory(
            t=np.asarray([0.0, duration], dtype=np.float64),
            pos=np.asarray(
                [
                    [float(args.initial_x), float(args.initial_y), float(args.initial_z)],
                    [float(args.gate_x), float(geometry.gate_y), float(geometry.gate_z)],
                ],
                dtype=np.float64,
            ),
            vel=np.asarray(
                [
                    [float(args.target_speed), 0.0, 0.0],
                    [float(args.target_speed), 0.0, 0.0],
                ],
                dtype=np.float64,
            ),
        )

    with args.trajectory_file.open(newline="") as f:
        rows = list(csv.DictReader(f))
    if not rows:
        raise SystemExit(f"empty trajectory file: {args.trajectory_file}")
    missing = {"t", "x", "y", "z"} - set(rows[0])
    if missing:
        raise SystemExit(f"trajectory file {args.trajectory_file} missing columns: {sorted(missing)}")

    t = np.asarray([float(row["t"]) for row in rows], dtype=np.float64)
    pos = np.asarray([[float(row["x"]), float(row["y"]), float(row["z"])] for row in rows], dtype=np.float64)
    if np.any(~np.isfinite(t)) or np.any(~np.isfinite(pos)):
        raise SystemExit(f"trajectory file {args.trajectory_file} contains non-finite t/x/y/z")
    if len(t) < 2 or np.any(np.diff(t) <= 0.0):
        raise SystemExit(f"trajectory file {args.trajectory_file} must contain at least two rows with strictly increasing t")

    if all(col in rows[0] for col in ("vx", "vy", "vz")):
        vel = np.asarray([[float(row["vx"]), float(row["vy"]), float(row["vz"])] for row in rows], dtype=np.float64)
    else:
        vel = np.zeros_like(pos)
        for axis in range(3):
            vel[:, axis] = np.gradient(pos[:, axis], t)
    if np.any(~np.isfinite(vel)):
        raise SystemExit(f"trajectory file {args.trajectory_file} contains non-finite velocity")
    return ReferenceTrajectory(t=t, pos=pos, vel=vel)


def _build_admm_host_library(horizon: int, force: bool = False) -> Path:
    game_root = APP_ROOT.parents[2] / "game-on-the-flat"
    build_dir = Path(tempfile.gettempdir()) / "tinympc_admm_host"
    output = build_dir / f"libtinympc_admm_host_n{int(horizon)}.so"
    sources = [
        APP_ROOT / "tools" / "tinympc_admm_host.cpp",
        APP_ROOT / "TinyMPC-ADMM" / "src" / "tinympc" / "model.cpp",
        APP_ROOT / "TinyMPC-ADMM" / "src" / "tinympc" / "auxil.cpp",
        APP_ROOT / "TinyMPC-ADMM" / "src" / "tinympc" / "cost_lqr.cpp",
        APP_ROOT / "TinyMPC-ADMM" / "src" / "tinympc" / "lqr.cpp",
        APP_ROOT / "TinyMPC-ADMM" / "src" / "tinympc" / "constraint_linear.cpp",
        APP_ROOT / "TinyMPC-ADMM" / "src" / "tinympc" / "admm.cpp",
        APP_ROOT / "TinyMPC-ADMM" / "src" / "tinympc" / "rho_benchmark.cpp",
        APP_ROOT / "TinyMPC-ADMM" / "src" / "tinympc" / "utils.cpp",
        game_root / "src" / "tinympc_model.c",
        game_root / "src" / "dare.c",
        game_root / "src" / "quat.c",
        game_root / "src" / "linalg.c",
    ]
    headers = [
        APP_ROOT / "src" / "params_100hz.h",
        APP_ROOT / "TinyMPC-ADMM" / "src" / "tinympc" / "types.h",
        APP_ROOT / "TinyMPC-ADMM" / "src" / "tinympc" / "admm.h",
        game_root / "include" / "gotf" / "tinympc_model.h",
    ]
    newest_input = max(path.stat().st_mtime for path in [*sources, *headers])
    if output.exists() and not force and output.stat().st_mtime >= newest_input:
        return output
    build_dir.mkdir(parents=True, exist_ok=True)
    command = [
        "g++",
        "-std=c++14",
        "-O2",
        "-shared",
        "-fPIC",
        f"-DNHORIZON={int(horizon)}",
        f"-I{APP_ROOT / 'TinyMPC-ADMM' / 'src'}",
        f"-I{APP_ROOT / 'TinyMPC-ADMM' / 'ext' / 'Eigen'}",
        f"-I{game_root / 'include'}",
        *[str(path) for path in sources],
        "-o",
        str(output),
    ]
    subprocess.run(command, cwd=APP_ROOT, check=True)
    return output


def _period_steps(period_s: float, plant_dt_s: float, label: str) -> int:
    ratio = float(period_s) / float(plant_dt_s)
    rounded = int(round(ratio))
    if rounded < 1 or not math.isclose(ratio, rounded, rel_tol=0.0, abs_tol=1e-8):
        raise SystemExit(f"{label} period {period_s:g}s is not an integer multiple of plant dt {plant_dt_s:g}s")
    return rounded


def _validate_timing(args: argparse.Namespace) -> None:
    if float(args.plant_dt) <= 0.0 or float(args.model_dt) <= 0.0 or float(args.mpc_rate_hz) <= 0.0:
        raise SystemExit("--plant-dt, --model-dt, and --mpc-rate-hz must be positive")
    _period_steps(float(args.model_dt), float(args.plant_dt), "model")
    mpc_period_s = 1.0 / float(args.mpc_rate_hz)
    _period_steps(mpc_period_s, float(args.plant_dt), "MPC")
    model_per_mpc = mpc_period_s / float(args.model_dt)
    if not math.isclose(model_per_mpc, round(model_per_mpc), rel_tol=0.0, abs_tol=1e-8):
        raise SystemExit("MPC period must contain an integer number of model knots")
    if int(args.horizon) < 2:
        raise SystemExit("--horizon must be at least 2")
    if not 0 <= int(args.start_k) < int(args.constraint_end_k) <= int(args.horizon):
        raise SystemExit("need 0 <= --start-k < --constraint-end-k <= --horizon")


def _add_obstacles(env: GymPybulletGateEnv, obstacles: list[BoxObstacle]) -> None:
    if env.env is None or env._p is None:
        raise RuntimeError("environment must be reset before adding obstacles")
    p = env._p
    client = env.env.getPyBulletClient()
    for obstacle in obstacles:
        collision = p.createCollisionShape(
            p.GEOM_BOX,
            halfExtents=obstacle.half_extents.tolist(),
            physicsClientId=client,
        )
        visual = p.createVisualShape(
            p.GEOM_BOX,
            halfExtents=obstacle.half_extents.tolist(),
            rgbaColor=[0.1, 0.45, 0.95, 1.0],
            physicsClientId=client,
        )
        p.createMultiBody(
            baseMass=0.0,
            baseCollisionShapeIndex=collision,
            baseVisualShapeIndex=visual,
            basePosition=obstacle.center.tolist(),
            physicsClientId=client,
        )


def _select_sector_hit(
    position: np.ndarray,
    rot_wb: np.ndarray,
    obstacles: list[BoxObstacle],
    args: argparse.Namespace,
) -> SectorHit | None:
    edges = np.linspace(-0.5 * math.radians(float(args.fov_deg)), 0.5 * math.radians(float(args.fov_deg)), int(args.sectors) + 1)
    hits: list[SectorHit] = []
    for sector in range(int(args.sectors)):
        az = float(0.5 * (edges[sector] + edges[sector + 1]))
        d_world = _normalized(rot_wb @ (R_BC @ _ray_from_azimuth(az)))
        best_depth = math.inf
        best_name = ""
        best_center = np.zeros(3, dtype=np.float64)
        for obstacle in obstacles:
            depth = _ray_aabb(position, d_world, obstacle.center - obstacle.half_extents, obstacle.center + obstacle.half_extents)
            if depth is not None and depth < best_depth:
                best_depth = float(depth)
                best_name = obstacle.name
                best_center = obstacle.center.copy()
        if float(args.min_depth) <= best_depth <= float(args.max_depth):
            confidence = max(0.0, min(1.0, 1.0 - best_depth / float(args.max_depth)))
            if confidence >= float(args.min_confidence):
                hits.append(SectorHit(sector, az, best_depth, confidence, best_name, best_center, d_world))
    if not hits:
        return None
    hits.sort(key=lambda item: item.depth_m / max(0.05, item.confidence))
    return hits[0]


def _sample_sector_hits(
    position: np.ndarray,
    rot_wb: np.ndarray,
    obstacles: list[BoxObstacle],
    args: argparse.Namespace,
    rng: np.random.Generator,
) -> tuple[list[SectorHit], str]:
    if rng.random() < float(args.sector_dropout_prob):
        return [], "dropout"
    if rng.random() < float(args.false_hit_prob):
        az = float(rng.uniform(-0.45 * math.radians(float(args.fov_deg)), 0.45 * math.radians(float(args.fov_deg))))
        depth = max(float(args.min_depth), float(args.false_hit_depth) + float(rng.normal(0.0, max(0.0, float(args.depth_noise_std)))))
        d_world = _normalized(rot_wb @ (R_BC @ _ray_from_azimuth(az)))
        sector_width = math.radians(float(args.fov_deg)) / max(1, int(args.sectors))
        sector = int(np.clip(math.floor((az + 0.5 * math.radians(float(args.fov_deg))) / sector_width), 0, int(args.sectors) - 1))
        return [SectorHit(sector, az, depth, 0.25, "false_hit", position + depth * d_world, d_world)], "false_hit"

    hits = _select_sector_hits(position, rot_wb, obstacles, args)
    if not hits:
        return [], "no_hit"
    if int(args.max_active_halfspaces) <= 1:
        hits = hits[:1]
    noisy_hits = []
    for hit in hits:
        depth = float(hit.depth_m) + float(args.depth_bias)
        if float(args.depth_noise_std) > 0.0:
            depth += float(rng.normal(0.0, float(args.depth_noise_std)))
        depth = float(np.clip(depth, float(args.min_depth), float(args.max_depth)))
        confidence = max(0.0, min(1.0, 1.0 - depth / float(args.max_depth)))
        noisy_center = np.asarray(position, dtype=np.float64) + depth * hit.ray_world
        noisy_hits.append(
            SectorHit(
                sector=hit.sector,
                azimuth_rad=hit.azimuth_rad,
                depth_m=depth,
                confidence=confidence,
                obstacle_name=hit.obstacle_name,
                obstacle_center=noisy_center,
                ray_world=hit.ray_world.copy(),
            )
        )
    noisy_hits.sort(key=lambda item: item.depth_m / max(0.05, item.confidence))
    return noisy_hits, "hit"


def _sample_sector_hit(
    position: np.ndarray,
    rot_wb: np.ndarray,
    obstacles: list[BoxObstacle],
    args: argparse.Namespace,
    rng: np.random.Generator,
) -> tuple[SectorHit | None, str]:
    hits, source = _sample_sector_hits(position, rot_wb, obstacles, args, rng)
    return (hits[0] if hits else None), source


def _select_sector_hits(
    position: np.ndarray,
    rot_wb: np.ndarray,
    obstacles: list[BoxObstacle],
    args: argparse.Namespace,
) -> list[SectorHit]:
    edges = np.linspace(-0.5 * math.radians(float(args.fov_deg)), 0.5 * math.radians(float(args.fov_deg)), int(args.sectors) + 1)
    hits: list[SectorHit] = []
    for sector in range(int(args.sectors)):
        az = float(0.5 * (edges[sector] + edges[sector + 1]))
        d_world = _normalized(rot_wb @ (R_BC @ _ray_from_azimuth(az)))
        best_depth = math.inf
        best_name = ""
        best_center = np.zeros(3, dtype=np.float64)
        for obstacle in obstacles:
            depth = _ray_aabb(position, d_world, obstacle.center - obstacle.half_extents, obstacle.center + obstacle.half_extents)
            if depth is not None and depth < best_depth:
                best_depth = float(depth)
                best_name = obstacle.name
                best_center = obstacle.center.copy()
        if float(args.min_depth) <= best_depth <= float(args.max_depth):
            confidence = max(0.0, min(1.0, 1.0 - best_depth / float(args.max_depth)))
            if confidence >= float(args.min_confidence):
                hits.append(SectorHit(sector, az, best_depth, confidence, best_name, best_center, d_world))
    hits.sort(key=lambda item: item.depth_m / max(0.05, item.confidence))
    return hits


def _inactive_constraint(status: str) -> HalfspaceConstraint:
    return HalfspaceConstraint(False, np.zeros(3), math.nan, math.nan, math.nan, -1, math.nan, 0.0, "", np.zeros(3), math.nan, status, 0)


def _should_switch_sector(held: HalfspaceConstraint, hit: SectorHit, args: argparse.Namespace) -> bool:
    if not held.active or not math.isfinite(held.depth_m):
        return True
    if int(hit.sector) == int(held.sector) and hit.obstacle_name == held.obstacle_name:
        return True
    ratio = float(args.sector_switch_ratio)
    if ratio >= 1.0:
        return True
    return float(hit.depth_m) < ratio * float(held.depth_m)


def _aged_constraint(constraint: HalfspaceConstraint, age_frames: int, position: np.ndarray) -> HalfspaceConstraint:
    if age_frames == constraint.age_frames and not constraint.active:
        return constraint
    slack = float(constraint.current_slack_m)
    active = bool(constraint.active)
    if constraint.active and math.isfinite(constraint.b):
        slack = float(constraint.b - np.dot(constraint.a, np.asarray(position, dtype=np.float64)))
        active = bool(slack > 0.02)
    return HalfspaceConstraint(
        active=active,
        a=constraint.a.copy(),
        b=float(constraint.b),
        depth_m=float(constraint.depth_m),
        margin_m=float(constraint.margin_m),
        sector=int(constraint.sector),
        azimuth_rad=float(constraint.azimuth_rad),
        confidence=float(constraint.confidence),
        obstacle_name=constraint.obstacle_name,
        obstacle_center=constraint.obstacle_center.copy(),
        current_slack_m=slack,
        status=("active" if active else "violates_current") if age_frames == 0 else f"held_{'active' if active else 'violates_current'}",
        age_frames=int(age_frames),
    )


def _constraint_from_hit(
    position: np.ndarray,
    rot_wb: np.ndarray,
    hit: SectorHit | None,
    args: argparse.Namespace,
) -> HalfspaceConstraint:
    if hit is None:
        return _inactive_constraint("no_depth")
    margin = float(args.margin_min) + (1.0 - hit.confidence) * float(args.margin_slack)
    margin = min(margin, 0.8 * hit.depth_m)
    a = _normalized(hit.ray_world)
    p_obst = np.asarray(position, dtype=np.float64) + hit.depth_m * a
    b = float(np.dot(a, p_obst) - margin)
    slack = float(b - np.dot(a, position))
    active = bool(slack > 0.02)
    return HalfspaceConstraint(
        active=active,
        a=a,
        b=b,
        depth_m=float(hit.depth_m),
        margin_m=float(margin),
        sector=int(hit.sector),
        azimuth_rad=float(hit.azimuth_rad),
        confidence=float(hit.confidence),
        obstacle_name=hit.obstacle_name,
        obstacle_center=hit.obstacle_center.copy(),
        current_slack_m=slack,
        status="active" if active else "violates_current",
        age_frames=0,
    )


def _nominal_velocity(
    state: np.ndarray,
    t: float,
    trajectory: ReferenceTrajectory,
    args: argparse.Namespace,
) -> np.ndarray:
    pos = np.asarray(state[:3], dtype=np.float64)
    ref_pos, ref_vel = trajectory.sample(float(t) + float(args.lookahead_s))
    velocity = np.asarray(
        [
            ref_vel[0] + float(args.lateral_kp) * (ref_pos[0] - pos[0]),
            ref_vel[1] + float(args.lateral_kp) * (ref_pos[1] - pos[1]),
            ref_vel[2] + float(args.vertical_kp) * (ref_pos[2] - pos[2]),
        ],
        dtype=np.float64,
    )
    velocity[0] = float(np.clip(velocity[0], 0.05, float(args.max_forward_speed)))
    velocity[1] = float(np.clip(velocity[1], -float(args.max_lateral_speed), float(args.max_lateral_speed)))
    velocity[2] = float(np.clip(velocity[2], -0.7, 0.7))
    return velocity


def _solver_state(env: GymPybulletGateEnv, state: np.ndarray) -> np.ndarray:
    try:
        return np.asarray(env.get_controller_state(), dtype=np.float64).reshape(12)
    except Exception:
        x = np.zeros(12, dtype=np.float64)
        x[0:3] = np.asarray(state[:3], dtype=np.float64)
        x[6:9] = np.asarray(state[3:6], dtype=np.float64)
        return x


def _reference_trajectory(
    x0: np.ndarray,
    t: float,
    trajectory: ReferenceTrajectory,
    constraint: HalfspaceConstraint,
    args: argparse.Namespace,
    horizon: int,
) -> np.ndarray:
    x_ref = np.zeros((int(horizon), AdmmHostSolver.NSTATES), dtype=np.float64)
    for k in range(int(horizon)):
        ref_pos, ref_vel = trajectory.sample(float(t) + k * float(args.model_dt))
        if constraint.active and float(args.admm_reference_sidestep) > 0.0:
            obstacle_side = float(constraint.obstacle_center[1] - x0[1])
            side = -math.copysign(1.0, obstacle_side) if abs(obstacle_side) > 1e-3 else -1.0
            ref_pos = ref_pos.copy()
            ref_pos[1] += side * float(args.admm_reference_sidestep)
        x_ref[k, 0:3] = ref_pos
        x_ref[k, 6:9] = ref_vel
    return x_ref


def _velocity_from_admm_plan(
    state: np.ndarray,
    planned_states: np.ndarray,
    constraint: HalfspaceConstraint,
    args: argparse.Namespace,
    plan_k: int,
) -> tuple[np.ndarray, float]:
    plan = np.asarray(planned_states, dtype=np.float64).reshape(-1, AdmmHostSolver.NSTATES)
    # The simulation's acceleration/PID shim cannot accept motor duty directly.
    # Execute the next consecutive state target instead of repeatedly steering
    # toward the terminal point, which faithfully represents 25 Hz plan-knot
    # execution between 5 Hz replans.
    target = plan[min(max(1, int(plan_k) + 1), len(plan) - 1), 0:3].copy()
    target[2] = float(np.clip(target[2], float(args.admm_min_target_z), float(args.admm_max_target_z)))
    pos = np.asarray(state[:3], dtype=np.float64)
    velocity = (target - pos) / max(1e-3, float(args.model_dt))
    velocity[0] = float(np.clip(velocity[0], 0.05, float(args.max_forward_speed)))
    if constraint.active and float(args.admm_forward_slack_scale) > 0.0:
        forward_cap = max(0.05, float(args.admm_forward_slack_scale) * max(0.0, constraint.current_slack_m) / max(1e-3, float(args.lookahead_s)))
        velocity[0] = min(float(velocity[0]), forward_cap)
    velocity[1] = float(np.clip(velocity[1], -float(args.max_lateral_speed), float(args.max_lateral_speed)))
    velocity[2] = float(np.clip(velocity[2], -0.7, 0.7))
    violation = 0.0
    return velocity, violation


def _project_velocity(
    velocity: np.ndarray,
    position: np.ndarray,
    constraint: HalfspaceConstraint,
    args: argparse.Namespace,
) -> tuple[np.ndarray, float]:
    if not constraint.active:
        return velocity.copy(), 0.0
    lookahead = float(args.lookahead_s)
    predicted = np.asarray(position, dtype=np.float64) + lookahead * np.asarray(velocity, dtype=np.float64)
    violation = float(np.dot(constraint.a, predicted) - constraint.b)
    if violation <= 0.0:
        return velocity.copy(), 0.0
    adjusted = np.asarray(velocity, dtype=np.float64) - (violation / max(1e-3, lookahead)) * constraint.a
    obstacle_side = float(constraint.obstacle_center[1] - position[1])
    side = -math.copysign(1.0, obstacle_side) if abs(obstacle_side) > 1e-3 else -1.0
    adjusted[1] += side * float(args.sidestep_gain) * min(1.0, violation / max(0.05, constraint.current_slack_m))
    forward_cap = max(0.05, 0.75 * max(0.0, constraint.current_slack_m) / max(1e-3, lookahead))
    adjusted[0] = min(float(adjusted[0]), forward_cap)
    adjusted[0] = float(np.clip(adjusted[0], 0.05, float(args.max_forward_speed)))
    adjusted[1] = float(np.clip(adjusted[1], -float(args.max_lateral_speed), float(args.max_lateral_speed)))
    adjusted[2] = float(np.clip(adjusted[2], -0.7, 0.7))
    return adjusted, violation


def _limit_command_delta(command: np.ndarray, previous: np.ndarray, max_delta: float) -> np.ndarray:
    delta = np.asarray(command, dtype=np.float64) - np.asarray(previous, dtype=np.float64)
    norm = float(np.linalg.norm(delta))
    if norm <= max(0.0, float(max_delta)) or norm <= 1e-12:
        return np.asarray(command, dtype=np.float64)
    return np.asarray(previous, dtype=np.float64) + delta * (float(max_delta) / norm)


def _append_horizon_rows(
    rows: list[list[object]],
    step: int,
    t: float,
    constraint: HalfspaceConstraint,
    args: argparse.Namespace,
) -> None:
    for k in range(int(args.horizon)):
        enabled = bool(constraint.active and int(args.start_k) <= k < int(args.constraint_end_k))
        rows.append(
            [
                step,
                f"{t:.9f}",
                k,
                int(enabled),
                f"{constraint.a[0]:.9f}" if enabled else "",
                f"{constraint.a[1]:.9f}" if enabled else "",
                f"{constraint.a[2]:.9f}" if enabled else "",
                f"{constraint.b:.9f}" if enabled else "",
                "" if not math.isfinite(constraint.depth_m) else f"{constraint.depth_m:.9f}",
                "" if not math.isfinite(constraint.margin_m) else f"{constraint.margin_m:.9f}",
                constraint.sector if enabled else "",
                "" if not math.isfinite(constraint.azimuth_rad) else f"{constraint.azimuth_rad:.9f}",
                f"{constraint.confidence:.6f}",
                constraint.obstacle_name,
                constraint.status,
            ]
        )


def _append_plan_rows(
    rows: list[list[object]],
    step: int,
    t: float,
    solver_info: dict[str, Any],
) -> None:
    if solver_info.get("states") is None:
        return
    states = np.asarray(solver_info["states"], dtype=np.float64).reshape(-1, AdmmHostSolver.NSTATES)
    for k, state in enumerate(states):
        rows.append(
            [
                step,
                f"{t:.9f}",
                k,
                f"{float(state[0]):.9f}",
                f"{float(state[1]):.9f}",
                f"{float(state[2]):.9f}",
                f"{float(state[6]):.9f}",
                f"{float(state[7]):.9f}",
                f"{float(state[8]):.9f}",
            ]
        )


def _log_row(
    step: int,
    t: float,
    state: np.ndarray,
    next_state: np.ndarray,
    nominal_velocity: np.ndarray,
    command_velocity: np.ndarray,
    accel: np.ndarray,
    constraint: HalfspaceConstraint,
    violation: float,
    clearance: float,
    info: dict[str, Any],
    solver_info: dict[str, Any],
    mpc_solved: bool,
    plan_k: int,
) -> list[object]:
    terminal = np.full(3, math.nan, dtype=np.float64)
    if solver_info.get("states") is not None:
        terminal = np.asarray(solver_info["states"], dtype=np.float64)[-1, 0:3]
    return [
        step,
        f"{t:.9f}",
        *[f"{float(v):.9f}" for v in state[:6]],
        *[f"{float(v):.9f}" for v in next_state[:6]],
        *[f"{float(v):.9f}" for v in nominal_velocity],
        *[f"{float(v):.9f}" for v in command_velocity],
        *[f"{float(v):.9f}" for v in accel],
        int(constraint.active),
        constraint.sector,
        "" if not math.isfinite(constraint.azimuth_rad) else f"{constraint.azimuth_rad:.9f}",
        "" if not math.isfinite(constraint.depth_m) else f"{constraint.depth_m:.9f}",
        "" if not math.isfinite(constraint.margin_m) else f"{constraint.margin_m:.9f}",
        f"{constraint.a[0]:.9f}",
        f"{constraint.a[1]:.9f}",
        f"{constraint.a[2]:.9f}",
        "" if not math.isfinite(constraint.b) else f"{constraint.b:.9f}",
        "" if not math.isfinite(constraint.current_slack_m) else f"{constraint.current_slack_m:.9f}",
        f"{violation:.9f}",
        f"{clearance:.9f}",
        constraint.obstacle_name,
        constraint.status,
        constraint.age_frames,
        bool(info.get("collision", False)),
        int(mpc_solved),
        int(plan_k),
        solver_info.get("status", ""),
        solver_info.get("iterations", ""),
        "" if not solver_info else f"{float(solver_info.get('pri_res', math.nan)):.9f}",
        "" if not solver_info else f"{float(solver_info.get('dua_res', math.nan)):.9f}",
        "" if not math.isfinite(float(terminal[0])) else f"{float(terminal[0]):.9f}",
        "" if not math.isfinite(float(terminal[1])) else f"{float(terminal[1]):.9f}",
        "" if not math.isfinite(float(terminal[2])) else f"{float(terminal[2]):.9f}",
    ]


def _write_log(path: Path, rows: list[list[object]]) -> None:
    with path.open("w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(
            [
                "step",
                "t",
                "x",
                "y",
                "z",
                "vx",
                "vy",
                "vz",
                "next_x",
                "next_y",
                "next_z",
                "next_vx",
                "next_vy",
                "next_vz",
                "nominal_vx",
                "nominal_vy",
                "nominal_vz",
                "command_vx",
                "command_vy",
                "command_vz",
                "accel_x",
                "accel_y",
                "accel_z",
                "constraint_active",
                "sector",
                "azimuth_rad",
                "depth_m",
                "margin_m",
                "a_x",
                "a_y",
                "a_z",
                "b",
                "current_slack_m",
                "lookahead_violation_m",
                "obstacle_clearance_m",
                "obstacle_name",
                "status",
                "constraint_age_frames",
                "gate_collision",
                "mpc_solved",
                "executed_plan_k",
                "admm_status",
                "admm_iterations",
                "admm_pri_res",
                "admm_dua_res",
                "admm_terminal_x",
                "admm_terminal_y",
                "admm_terminal_z",
            ]
        )
        writer.writerows(rows)


def _write_horizon(path: Path, rows: list[list[object]]) -> None:
    with path.open("w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(
            [
                "step",
                "t",
                "k",
                "en_hs",
                "a_x",
                "a_y",
                "a_z",
                "b",
                "depth_m",
                "margin_m",
                "sector",
                "azimuth_center_rad",
                "confidence",
                "obstacle_name",
                "status",
            ]
        )
        writer.writerows(rows)


def _write_plan(path: Path, rows: list[list[object]]) -> None:
    with path.open("w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["step", "t", "k", "x", "y", "z", "vx", "vy", "vz"])
        writer.writerows(rows)


def _ray_from_azimuth(azimuth_rad: float) -> np.ndarray:
    return _normalized(np.asarray([math.tan(float(azimuth_rad)), 0.0, 1.0], dtype=np.float64))


def _ray_aabb(origin: np.ndarray, direction: np.ndarray, box_min: np.ndarray, box_max: np.ndarray) -> float | None:
    t_min = -math.inf
    t_max = math.inf
    for axis in range(3):
        o = float(origin[axis])
        d = float(direction[axis])
        lo = float(box_min[axis])
        hi = float(box_max[axis])
        if abs(d) <= 1e-9:
            if o < lo or o > hi:
                return None
            continue
        t1 = (lo - o) / d
        t2 = (hi - o) / d
        t_axis_min = min(t1, t2)
        t_axis_max = max(t1, t2)
        t_min = max(t_min, t_axis_min)
        t_max = min(t_max, t_axis_max)
        if t_min > t_max:
            return None
    if t_max < 0.0:
        return None
    return max(0.0, t_min)


def _clearance_to_obstacles(position: np.ndarray, obstacles: list[BoxObstacle], drone_radius: float) -> float:
    clearances = []
    for obstacle in obstacles:
        delta = np.abs(np.asarray(position, dtype=np.float64) - obstacle.center) - obstacle.half_extents
        outside = np.maximum(delta, 0.0)
        outside_dist = float(np.linalg.norm(outside))
        inside_depth = float(np.max(delta))
        clearances.append(outside_dist if inside_depth > 0.0 else inside_depth)
    point_clearance = min(clearances) if clearances else math.inf
    return point_clearance - float(drone_radius)


def _env_quat(env: GymPybulletGateEnv) -> np.ndarray:
    if env.env is None:
        return np.asarray([0.0, 0.0, 0.0, 1.0], dtype=np.float64)
    return np.asarray(env.env.quat[0], dtype=np.float64)


def _quat_xyzw_to_rot(q: np.ndarray) -> np.ndarray:
    q = _normalized(np.asarray(q, dtype=np.float64).reshape(4))
    x, y, z, w = q
    return np.asarray(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def _normalized(v: np.ndarray) -> np.ndarray:
    norm = float(np.linalg.norm(v))
    if norm <= 1e-12 or not math.isfinite(norm):
        return np.zeros_like(v, dtype=np.float64)
    return np.asarray(v, dtype=np.float64) / norm


if __name__ == "__main__":
    raise SystemExit(main())
