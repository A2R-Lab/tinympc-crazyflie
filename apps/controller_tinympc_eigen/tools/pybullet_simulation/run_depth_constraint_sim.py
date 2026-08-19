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
* ``--control-mode geometric`` uses a quaternion/SO(3) attitude error and is
  the simulation-only Tier-3 path for references that cross 180 degrees.

This is a firmware-shadow experiment for the ADMM half-space path, not a full
Crazyflie motor/controller replica.
"""

from __future__ import annotations

import argparse
import ctypes
import csv
import hashlib
import json
import math
import os
from collections import deque
from dataclasses import dataclass
from pathlib import Path
import subprocess
import sys
import tempfile
import time
from typing import Any

import numpy as np


APP_ROOT = Path(__file__).resolve().parents[2]
from firmware_pybullet_env import GateGeometry, GymPybulletGateEnv, GymPybulletGateEnvConfig  # noqa: E402


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
    quat_wxyz: np.ndarray
    omega_body: np.ndarray
    input_delta_n: np.ndarray

    def sample(self, query_t: float) -> tuple[np.ndarray, np.ndarray]:
        tt = np.asarray(self.t, dtype=np.float64)
        q = float(np.clip(float(query_t), float(tt[0]), float(tt[-1])))
        pos = np.asarray([np.interp(q, tt, self.pos[:, axis]) for axis in range(3)], dtype=np.float64)
        vel = np.asarray([np.interp(q, tt, self.vel[:, axis]) for axis in range(3)], dtype=np.float64)
        return pos, vel

    def sample_full(self, query_t: float) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        tt = np.asarray(self.t, dtype=np.float64)
        qtime = float(np.clip(float(query_t), float(tt[0]), float(tt[-1])))
        pos, vel = self.sample(qtime)
        upper = int(np.searchsorted(tt, qtime, side="right"))
        upper = min(max(1, upper), len(tt) - 1)
        lower = upper - 1
        alpha = (qtime - tt[lower]) / max(1e-12, tt[upper] - tt[lower])
        # Match firmware sampleTrajectoryReference(): interpolate quaternion
        # components linearly, then normalize.  Do not use SLERP here—the
        # purpose of this runner is firmware data-path parity.
        quat = (1.0 - alpha) * self.quat_wxyz[lower] + alpha * self.quat_wxyz[upper]
        quat /= np.linalg.norm(quat)
        omega = (1.0 - alpha) * self.omega_body[lower] + alpha * self.omega_body[upper]
        input_delta = (1.0 - alpha) * self.input_delta_n[lower] + alpha * self.input_delta_n[upper]
        return pos, vel, quat, omega, input_delta

    @property
    def final_position(self) -> np.ndarray:
        return np.asarray(self.pos[-1], dtype=np.float64)


class FlowDeckAltitudeEstimator:
    """Model downward range validity and bridge invalid attitudes inertially."""

    def __init__(self, mode: str, max_tilt_deg: float, reacquire_tau_s: float) -> None:
        self.mode = str(mode)
        self.minimum_cosine = math.cos(math.radians(float(max_tilt_deg)))
        self.reacquire_tau_s = max(0.0, float(reacquire_tau_s))
        self.estimated_z: float | None = None
        self.last_t: float | None = None
        self.records: list[list[object]] = []
        self.invalid_samples = 0
        self.maximum_error_m = 0.0

    def apply(
        self, world: np.ndarray, quaternion_xyzw: np.ndarray, timestamp: float
    ) -> None:
        rotation = _quat_xyzw_to_rot(quaternion_xyzw)
        cosine = float(rotation[2, 2])
        valid = bool(cosine >= self.minimum_cosine)
        delayed_z = float(world[2])
        delayed_vz = float(world[8])
        dt = 0.0 if self.last_t is None else max(0.0, float(timestamp) - self.last_t)
        raw_range = delayed_z / cosine if cosine > 1.0e-3 else 4.0
        raw_range = float(np.clip(raw_range, 0.0, 4.0))
        if self.estimated_z is None:
            self.estimated_z = delayed_z
        if self.mode == "raw-range":
            self.estimated_z = raw_range
        elif self.mode == "freeze":
            if valid:
                self.estimated_z = delayed_z
        elif self.mode == "inertial":
            predicted = self.estimated_z + delayed_vz * dt
            if valid:
                alpha = 1.0 if self.reacquire_tau_s <= 0.0 else 1.0 - math.exp(
                    -dt / self.reacquire_tau_s
                )
                self.estimated_z = predicted + alpha * (delayed_z - predicted)
            else:
                self.estimated_z = predicted
        else:
            self.estimated_z = delayed_z
        if not valid:
            self.invalid_samples += 1
        world[2] = self.estimated_z
        self.maximum_error_m = max(
            self.maximum_error_m, abs(float(self.estimated_z) - delayed_z)
        )
        self.records.append([
            f"{float(timestamp):.9f}", f"{delayed_z:.9f}",
            f"{float(self.estimated_z):.9f}", f"{raw_range:.9f}",
            f"{cosine:.9f}", int(valid), self.mode,
        ])
        self.last_t = float(timestamp)

class NeuralSectorPerception:
    """Adapter from the deployed GAP8 ONNX output to firmware-style sector hits."""

    ANGLES_DEG = (-40.0, -13.333333, 13.333333, 40.0)

    def __init__(self, model_path: Path | None) -> None:
        if model_path is None:
            raise SystemExit("--onnx-model is required with --perception-mode neural")
        try:
            import onnxruntime as ort
        except ImportError as exc:
            raise SystemExit("neural perception requires onnxruntime") from exc
        self.model_path = model_path.resolve()
        if not self.model_path.is_file():
            raise SystemExit(f"ONNX model not found: {self.model_path}")
        self.session = ort.InferenceSession(str(self.model_path), providers=["CPUExecutionProvider"])
        self.input_name = self.session.get_inputs()[0].name
        manifest_path = self.model_path.with_name("quantization_manifest.json")
        if not manifest_path.is_file():
            raise SystemExit(f"neural model requires adjacent {manifest_path.name}")
        manifest = json.loads(manifest_path.read_text())
        self.scale = float(manifest["scale"])
        self.last_clearance_m = np.full(4, math.nan, dtype=np.float64)
        self.last_confidence_score = np.full(4, math.nan, dtype=np.float64)
        self.last_dangerous = np.zeros(4, dtype=bool)

    def sample(self, frame: np.ndarray, position: np.ndarray, rot_wb: np.ndarray,
               args: argparse.Namespace) -> tuple[list[SectorHit], str]:
        if frame.shape != (160, 160):
            raise RuntimeError(f"neural camera contract requires 160x160, got {frame.shape}")
        tensor = frame[20:140, :].astype(np.float32)[None, None]
        raw = np.asarray(self.session.run(None, {self.input_name: tensor})[0])
        if raw.shape != (1, 12, 15, 20):
            raise RuntimeError(f"unexpected ONNX output shape {raw.shape}")
        logical = raw.astype(np.float32)[0] * self.scale - 6.0
        clearance = (np.clip(logical[4:8].mean(axis=(1, 2)), -6.0, 6.0) + 6.0) * 0.5
        confidence = logical[8:12].mean(axis=(1, 2))
        dangerous = clearance < float(args.neural_danger_clearance)
        self.last_clearance_m = np.asarray(clearance, dtype=np.float64)
        self.last_confidence_score = np.asarray(confidence, dtype=np.float64)
        self.last_dangerous = np.asarray(dangerous, dtype=bool)
        if int(np.count_nonzero(dangerous)) < 2:
            return [], "neural_fewer_than_two_dangerous"

        # Match the requested simple policy: a single plane normal to the
        # vehicle's forward axis. Use the closest region conservatively.
        depth = float(np.min(clearance))
        forward_world = _normalized(rot_wb @ np.asarray([1.0, 0.0, 0.0]))
        mean_score = float(np.mean(confidence))
        normalized_confidence = min(1.0, max(0.0, (mean_score + 6.0) / 12.0))
        return [SectorHit(
            -2, 0.0, depth, normalized_confidence, "neural_two_or_more_dangerous",
            np.asarray(position, dtype=np.float64) + depth * forward_world,
            forward_world,
        )], "neural_two_or_more_dangerous"


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
        self._lib.tinympc_admm_host_select_model.argtypes = [ctypes.c_int]
        self._lib.tinympc_admm_host_select_model.restype = ctypes.c_bool
        self._lib.tinympc_admm_host_selected_model.argtypes = []
        self._lib.tinympc_admm_host_selected_model.restype = ctypes.c_int
        self._lib.tinympc_admm_host_set_input_baseline.argtypes = [f32p]
        self._lib.tinympc_admm_host_set_input_baseline.restype = ctypes.c_bool
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
        u_ref: np.ndarray | None = None,
        input_baseline_n: np.ndarray | None = None,
    ) -> dict[str, Any]:
        x0_f = np.ascontiguousarray(np.asarray(x0, dtype=np.float32).reshape(self.NSTATES))
        xref_f = np.ascontiguousarray(np.asarray(x_ref, dtype=np.float32).reshape(self.horizon, self.NSTATES))
        uref_f = np.ascontiguousarray(
            np.zeros((self.horizon - 1, self.NINPUTS), dtype=np.float32)
            if u_ref is None else np.asarray(u_ref, dtype=np.float32).reshape(self.horizon - 1, self.NINPUTS)
        )
        baseline_f = np.ascontiguousarray(
            GymPybulletGateEnv.FIRMWARE_HOVER_THRUST_N
            if input_baseline_n is None
            else np.asarray(input_baseline_n, dtype=np.float32).reshape(self.NINPUTS),
            dtype=np.float32,
        )
        if not self._lib.tinympc_admm_host_set_input_baseline(
            baseline_f.ctypes.data_as(ctypes.POINTER(ctypes.c_float))
        ):
            raise RuntimeError(f"invalid TinyMPC motor-thrust baseline: {baseline_f.tolist()}")
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
            "model_id": int(self._lib.tinympc_admm_host_selected_model()),
        }

    def select_model(self, model_id: int) -> None:
        if int(self._lib.tinympc_admm_host_selected_model()) == int(model_id):
            return
        if not self._lib.tinympc_admm_host_select_model(int(model_id)):
            raise RuntimeError(f"failed to select stored TinyMPC model {model_id}")


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", type=Path, default=Path("flow_sim_dataset/depth_constraint_closed_loop"))
    ap.add_argument("--duration", type=float, default=7.0)
    ap.add_argument("--plant-dt", "--dt", dest="plant_dt", type=float, default=0.002,
                    help="PyBullet integration and acceleration-shim step [s]")
    ap.add_argument("--model-dt", type=float, default=0.02,
                    help="MPC discrete-model knot interval [s]; regenerated from the continuous hover model")
    ap.add_argument("--mpc-rate-hz", type=float, default=50.0,
                    help="receding-horizon solve rate; default matches generated firmware")
    ap.add_argument("--gui", action="store_true")
    ap.add_argument("--realtime", action=argparse.BooleanOptionalAction, default=None,
                    help="pace simulation to wall time (defaults on with --gui, off headlessly)")
    ap.add_argument("--progress-interval-s", type=float, default=1.0,
                    help="print progress this often in simulated seconds; <=0 disables")
    ap.add_argument("--plot", action="store_true",
                    help="write trajectory_topdown.png after the run")
    ap.add_argument("--image-size", type=int, default=160)
    ap.add_argument("--fov-deg", type=float, default=70.0)
    ap.add_argument("--sectors", type=int, default=7)
    ap.add_argument("--perception-mode", choices=["none", "geometry", "neural"], default="none",
                    help="none for maneuver simulation; geometry/neural are isolated legacy obstacle experiments")
    ap.add_argument("--onnx-model", type=Path, default=None,
                    help="sequential_int.onnx; required for --perception-mode neural")
    ap.add_argument("--neural-danger-clearance", type=float, default=0.25,
                    help="at least two neural clearances must be below this to place the plane [m]")
    ap.add_argument("--neural-plane-margin", type=float, default=0.10,
                    help="stand-off subtracted from the closest neural clearance [m]")
    ap.add_argument(
        "--control-mode", choices=["admm", "tinympc-acro", "geometric", "projection"], default="admm",
        help="tinympc-acro uses reference-centered quaternion-error TinyMPC; geometric is a control experiment",
    )
    ap.add_argument("--max-active-halfspaces", type=int, default=1,
                    help="number of sector half-spaces to pass to ADMM; capped by host MAX_HS=3")
    ap.add_argument("--tinympc-max-iter", type=int, default=5,
                    help="ADMM iterations per MPC tick; default matches src/controller_tinympc.cpp")
    ap.add_argument("--tinympc-rho", type=float, default=250.0,
                    help="ADMM rho; 250 matches src/tinympc_generated_params.h and firmware")
    ap.add_argument("--model-schedule", choices=["level", "bank15", "bank30", "bank60"], default="level",
                    help="level matches current firmware; bank schedules are host-only experimental bundles")
    ap.add_argument("--controller-frame", choices=["firmware-local", "world"], default="firmware-local",
                    help="firmware-local resets origin/removes yaw per solve; world reproduces the old global-state controller")
    ap.add_argument("--motor-time-constant-ms", type=float, default=0.0,
                    help="first-order motor-thrust time constant [ms]; 0 applies thrust instantaneously")
    ap.add_argument("--motor-command-delay-ms", type=float, default=0.0,
                    help="pure motor-command delay [ms]; must be an integer multiple of plant dt")
    ap.add_argument("--controller-compute-delay-ms", type=float, default=0.0,
                    help="state-sample to solved-command latency [ms]; multiple of plant dt")
    ap.add_argument("--trajectory-handoff-hold-s", type=float, default=0.0,
                    help="hold the first reference state before advancing, as firmware does on activation")
    ap.add_argument("--plant-mass-scale", type=float, default=1.0,
                    help="actual plant mass divided by the firmware-model mass")
    ap.add_argument("--plant-inertia-scale", type=float, default=1.0,
                    help="actual diagonal inertia divided by the firmware-model inertia")
    ap.add_argument("--plant-thrust-scale", type=float, default=1.0,
                    help="actual thrust effectiveness divided by commanded-model effectiveness")
    ap.add_argument("--plant-motor-thrust-scales", default="1,1,1,1",
                    help="four comma-separated per-motor thrust effectiveness scales")
    ap.add_argument("--rotor-drag-scale", type=float, default=0.0,
                    help="scale for gym-pybullet-drones' identified rotor drag; 0 disables drag")
    ap.add_argument("--state-estimate-delay-ms", type=float, default=0.0,
                    help="delay applied to the state seen by TinyMPC [ms]; multiple of plant dt")
    ap.add_argument("--position-noise-std-m", type=float, default=0.0)
    ap.add_argument("--velocity-noise-std-mps", type=float, default=0.0)
    ap.add_argument("--attitude-noise-std-deg", type=float, default=0.0)
    ap.add_argument("--gyro-noise-std-deg-s", type=float, default=0.0)
    ap.add_argument(
        "--flow-z-mode", choices=("ideal", "raw-range", "freeze", "inertial"),
        default="ideal",
        help="altitude handling when the downward rangefinder is tilted away from the floor",
    )
    ap.add_argument("--flow-z-max-tilt-deg", type=float, default=35.0,
                    help="largest tilt that accepts a downward range update")
    ap.add_argument("--flow-z-reacquire-tau-ms", type=float, default=120.0,
                    help="range re-acquisition blend time for inertial altitude mode")
    ap.add_argument("--geometric-position-kp", default="8,8,12",
                    help="three comma-separated SE(3) position gains [1/s^2]")
    ap.add_argument("--geometric-velocity-kd", default="5,5,7",
                    help="three comma-separated SE(3) velocity gains [1/s]")
    ap.add_argument("--geometric-attitude-kp", type=float, default=0.008,
                    help="SO(3) attitude-error moment gain [N m]")
    ap.add_argument("--geometric-rate-kd", type=float, default=0.0008,
                    help="body-rate-error moment gain [N m s/rad]")
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
    ap.add_argument("--start-k", type=int, default=0,
                    help="first horizon knot with an enabled obstacle half-space")
    ap.add_argument("--constraint-end-k", type=int, default=20,
                    help="exclusive final constrained knot; default matches all 20 firmware horizon knots")
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
    return ap.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    _validate_timing(args)
    motor_thrust_scales = _parse_motor_thrust_scales(args.plant_motor_thrust_scales)
    geometric_position_kp = _parse_positive_vector3(
        args.geometric_position_kp, "--geometric-position-kp"
    )
    geometric_velocity_kd = _parse_positive_vector3(
        args.geometric_velocity_kd, "--geometric-velocity-kd"
    )
    if str(args.model_schedule) != "level":
        print(
            "WARNING: stored bank-model scheduling is an experimental host path; "
            "the current onboard controller still installs only the level generated model.",
            file=sys.stderr,
            flush=True,
        )
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
        motor_time_constant_s=1.0e-3 * float(args.motor_time_constant_ms),
        motor_command_delay_s=1.0e-3 * float(args.motor_command_delay_ms),
        mass_scale=float(args.plant_mass_scale),
        inertia_scale=float(args.plant_inertia_scale),
        thrust_scale=float(args.plant_thrust_scale),
        motor_thrust_scales=motor_thrust_scales,
        rotor_drag_scale=float(args.rotor_drag_scale),
    )
    env = GymPybulletGateEnv(geometry=geometry, config=env_config)
    obstacles = [] if args.perception_mode == "none" and not args.obstacle else _parse_obstacles(args.obstacle)
    admm_solver = (
        AdmmHostSolver(
            args.tinympc_max_iter,
            args.tinympc_rho,
            horizon=int(args.horizon),
            model_dt_s=float(args.model_dt),
            force_rebuild=bool(args.force_rebuild_solver),
        )
        if str(args.control_mode) in ("admm", "tinympc-acro")
        else None
    )
    rows: list[list[object]] = []
    horizon_rows: list[list[object]] = []
    plan_rows: list[list[object]] = []
    neural_rows: list[list[object]] = []
    summary: dict[str, Any] = {}
    rng = np.random.default_rng(int(args.seed))
    flow_altitude = FlowDeckAltitudeEstimator(
        str(args.flow_z_mode), float(args.flow_z_max_tilt_deg),
        1.0e-3 * float(args.flow_z_reacquire_tau_ms),
    )
    neural = NeuralSectorPerception(args.onnx_model) if args.perception_mode == "neural" else None

    try:
        env.reset()
        _add_obstacles(env, obstacles)
        run_duration_s = float(args.duration) + float(args.trajectory_handoff_hold_s)
        steps = int(round(run_duration_s / float(args.plant_dt)))
        camera_period_steps = (
            _period_steps(1.0 / max(1e-6, float(args.camera_rate_hz)), args.plant_dt, "camera")
            if args.perception_mode != "none" else steps + 1
        )
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
        controller_update_count = 0
        geometric_clip_steps = 0
        geometric_max_attitude_error_rad = 0.0
        geometric_max_rate_error_rad_s = 0.0
        model_switch_count = 0
        scheduled_model_id = 0
        held_plan: np.ndarray | None = None
        held_solver_info: dict[str, Any] = {}
        plan_k = 0
        realtime = bool(args.gui) if args.realtime is None else bool(args.realtime)
        wall_start = time.monotonic()
        next_progress_t = 0.0
        estimate_delay_steps = _nonnegative_delay_steps(
            1.0e-3 * float(args.state_estimate_delay_ms), float(args.plant_dt), "state estimate"
        )
        controller_delay_steps = _nonnegative_delay_steps(
            1.0e-3 * float(args.controller_compute_delay_ms), float(args.plant_dt), "controller compute"
        )
        pending_motor_controls: deque[tuple[int, np.ndarray]] = deque()
        active_motor_control = np.zeros(AdmmHostSolver.NINPUTS, dtype=np.float64)
        initial_state = env.get_state()
        initial_quat = _env_quat(env)
        initial_controller_state = _solver_state(env, initial_state)
        state_history: deque[tuple[np.ndarray, np.ndarray]] = deque(
            [(initial_controller_state.copy(), initial_quat.copy()) for _ in range(estimate_delay_steps + 1)],
            maxlen=estimate_delay_steps + 1,
        )
        for step in range(steps):
            t = step * float(args.plant_dt)
            handoff_active = t < float(args.trajectory_handoff_hold_s) - 1.0e-12
            trajectory_t = max(0.0, t - float(args.trajectory_handoff_hold_s))
            if float(args.progress_interval_s) > 0.0 and t + 1.0e-12 >= next_progress_t:
                print(f"sim {t:6.2f}/{run_duration_s:.2f} s  ({100.0 * step / max(1, steps):5.1f}%)", flush=True)
                next_progress_t += float(args.progress_interval_s)
            state = env.get_state()
            quat = _env_quat(env)
            rot_wb = _quat_xyzw_to_rot(quat)
            state_history.append((_solver_state(env, state), quat.copy()))
            if args.perception_mode != "none" and step % camera_period_steps == 0:
                if neural is None:
                    hits, hit_source = _sample_sector_hits(state[:3], rot_wb, obstacles, args, rng)
                else:
                    hits, hit_source = neural.sample(env.render_camera(), state[:3], rot_wb, args)
                    neural_rows.append([
                        step, f"{t:.9f}",
                        *[f"{float(value):.9f}" for value in neural.last_clearance_m],
                        *[f"{float(value):.9f}" for value in neural.last_confidence_score],
                        *[int(value) for value in neural.last_dangerous],
                        int(len(hits) > 0), hit_source,
                    ])
                if hit_source == "dropout":
                    dropout_count += 1
                elif hit_source == "false_hit":
                    false_hit_count += 1
                if neural is not None:
                    if hits:
                        # A neural obstacle plane is a world-fixed observation.
                        # Refresh its age while danger persists, but do not move
                        # it forward with every new camera frame.
                        if held_constraints[0].obstacle_name != "neural_two_or_more_dangerous":
                            held_constraints = [_constraint_from_hit(state[:3], rot_wb, hits[0], args)]
                        held_age_steps = 0
                    else:
                        # This simple policy releases immediately when fewer
                        # than two regions remain below threshold.
                        held_constraints = [_inactive_constraint(hit_source)]
                        held_age_steps = max_constraint_age_steps + 1
                elif hits:
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
            nominal_velocity = _nominal_velocity(state, trajectory_t, trajectory, args)
            solver_info: dict[str, Any] = {}
            mpc_solved = False
            if admm_solver is not None:
                if step % mpc_period_steps == 0:
                    estimate_world, estimate_quat = _noisy_state_estimate(
                        state_history[0][0], state_history[0][1], args, rng,
                        flow_altitude, t,
                    )
                    reference_time = 0.0 if handoff_active else trajectory_t
                    if str(args.control_mode) == "tinympc-acro":
                        x0_solver, feedforward_input_delta = _tinympc_reference_error_state(
                            estimate_world, estimate_quat, trajectory, reference_time
                        )
                        frame_origin, frame_yaw = None, 0.0
                        x_ref = np.zeros(
                            (admm_solver.horizon, AdmmHostSolver.NSTATES), dtype=np.float64
                        )
                        u_ref = np.zeros(
                            (admm_solver.horizon - 1, AdmmHostSolver.NINPUTS), dtype=np.float64
                        )
                    elif str(args.perception_mode) == "none" and str(args.controller_frame) == "firmware-local":
                        x0_solver, frame_origin, frame_yaw = _firmware_local_state_from_world(
                            estimate_world, estimate_quat
                        )
                    else:
                        x0_solver = estimate_world
                        frame_origin, frame_yaw = None, 0.0
                    if str(args.control_mode) != "tinympc-acro":
                        x_ref, u_ref = _reference_trajectory(
                            x0_solver, trajectory_t, trajectory, constraint, args, admm_solver.horizon,
                            frame_origin=frame_origin, frame_yaw=frame_yaw,
                            hold_reference=handoff_active,
                        )
                    next_model_id = 0 if str(args.control_mode) == "tinympc-acro" else _scheduled_model_id(
                        trajectory, trajectory_t, args, scheduled_model_id,
                        hold_reference=handoff_active,
                    )
                    if next_model_id != scheduled_model_id:
                        admm_solver.select_model(next_model_id)
                        scheduled_model_id = next_model_id
                        model_switch_count += 1
                    held_solver_info = admm_solver.solve(
                        x0=x0_solver,
                        x_ref=x_ref,
                        constraints=constraints,
                        start_k=int(args.start_k),
                        end_k=int(args.constraint_end_k),
                        enable_constraints=(
                            str(args.control_mode) != "tinympc-acro" and not bool(args.no_avoidance)
                        ),
                        u_ref=u_ref,
                        input_baseline_n=(
                            GymPybulletGateEnv.FIRMWARE_HOVER_THRUST_N
                            + feedforward_input_delta
                            if str(args.control_mode) == "tinympc-acro" else None
                        ),
                    )
                    held_plan = np.asarray(held_solver_info["states"], dtype=np.float64)
                    plan_k = 0
                    solved_controls = np.asarray(held_solver_info["controls"], dtype=np.float64)
                    motor_control = solved_controls[0].copy()
                    if str(args.control_mode) == "tinympc-acro":
                        physical_baseline = (
                            GymPybulletGateEnv.FIRMWARE_HOVER_THRUST_N
                            + feedforward_input_delta
                        )
                        held_solver_info["input_baseline_n"] = physical_baseline.copy()
                        held_solver_info["first_control_correction_n"] = motor_control.copy()
                        motor_control += feedforward_input_delta
                        held_solver_info["commanded_physical_motor_thrust_n"] = (
                            GymPybulletGateEnv.FIRMWARE_HOVER_THRUST_N + motor_control
                        )
                    pending_motor_controls.append((step + controller_delay_steps, motor_control))
                    mpc_solved = True
                    mpc_solve_count += 1
                elif held_plan is not None:
                    plan_k = min(plan_k + int(step % model_period_steps == 0), admm_solver.horizon - 1)
                if held_plan is None:
                    raise RuntimeError("ADMM plan was not initialized")
                solver_info = held_solver_info
                if str(args.control_mode) == "tinympc-acro":
                    _, command_velocity = trajectory.sample(
                        0.0 if handoff_active else trajectory_t
                    )
                    violation = 0.0
                else:
                    command_velocity, violation = _velocity_from_admm_plan(
                        state, held_plan, constraint, args, plan_k=plan_k
                    )
                    if bool(args.admm_safety_filter):
                        command_velocity, violation = _project_velocity(command_velocity, state[:3], constraint, args)
            elif str(args.control_mode) == "geometric":
                estimate_world, estimate_quat = _noisy_state_estimate(
                    state_history[0][0], state_history[0][1], args, rng,
                    flow_altitude, t,
                )
                reference_time = 0.0 if handoff_active else trajectory_t
                motor_control, geometric_info = _geometric_motor_control(
                    position=estimate_world[0:3],
                    velocity=estimate_world[6:9],
                    quaternion_xyzw=estimate_quat,
                    omega_body=estimate_world[9:12],
                    trajectory=trajectory,
                    query_t=reference_time,
                    args=args,
                    position_kp=geometric_position_kp,
                    velocity_kd=geometric_velocity_kd,
                )
                pending_motor_controls.append(
                    (step + controller_delay_steps, motor_control.copy())
                )
                controller_update_count += 1
                geometric_clip_steps += int(geometric_info["motor_clipped"] > 0.5)
                geometric_max_attitude_error_rad = max(
                    geometric_max_attitude_error_rad,
                    float(geometric_info["attitude_error_rad"]),
                )
                geometric_max_rate_error_rad_s = max(
                    geometric_max_rate_error_rad_s,
                    float(geometric_info["rate_error_rad_s"]),
                )
                _, command_velocity = trajectory.sample(reference_time)
                violation = 0.0
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
            if str(args.control_mode) in ("admm", "tinympc-acro", "geometric"):
                while pending_motor_controls and pending_motor_controls[0][0] <= step:
                    _, active_motor_control = pending_motor_controls.popleft()
                _, _, done, info = env.step_motor_thrust(active_motor_control)
            else:
                _, _, done, info = env.step_velocity(command_velocity)
            next_state = env.get_state()
            next_controller_state = _solver_state(env, next_state)
            clearance = _clearance_to_obstacles(next_state[:3], obstacles, float(args.drone_clearance_radius))
            min_clearance = min(min_clearance, clearance)
            min_z = min(min_z, float(next_state[2]))
            obstacle_collision = bool(obstacle_collision or clearance < 0.0)
            env_collision = bool(env_collision or info.get("collision", False))
            collision = bool(obstacle_collision or env_collision)
            goal_error = float(np.linalg.norm(next_state[:3] - trajectory.final_position))
            reached_goal = bool(reached_goal or (
                trajectory_t >= float(trajectory.t[-1])
                and goal_error <= float(args.goal_tolerance)
            ))
            if constraint.active:
                active_count += 1
            rows.append(_log_row(
                step, t, state, next_state, nominal_velocity, command_velocity, accel,
                constraint, violation, clearance, info, solver_info, mpc_solved, plan_k,
                next_controller_state, quat,
            ))
            if mpc_solved:
                _append_horizon_rows(horizon_rows, step, t, constraint, args)
                _append_plan_rows(
                    plan_rows, step, t, solver_info,
                    frame_origin=frame_origin, frame_yaw=frame_yaw,
                )
            if collision or reached_goal:
                break
            held_age_steps += 1
            if realtime:
                delay = wall_start + (step + 1) * float(args.plant_dt) - time.monotonic()
                if delay > 0.0:
                    time.sleep(delay)

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
            "controller_update_count": int(controller_update_count),
            "model_schedule": str(args.model_schedule),
            "current_firmware_model_parity": (
                str(args.control_mode) == "admm" and str(args.model_schedule) == "level"
            ),
            "controller_frame": str(args.controller_frame),
            "motor_time_constant_ms": float(args.motor_time_constant_ms),
            "motor_command_delay_ms": float(args.motor_command_delay_ms),
            "controller_compute_delay_ms": float(args.controller_compute_delay_ms),
            "trajectory_handoff_hold_s": float(args.trajectory_handoff_hold_s),
            "plant_mass_scale": float(args.plant_mass_scale),
            "plant_inertia_scale": float(args.plant_inertia_scale),
            "plant_thrust_scale": float(args.plant_thrust_scale),
            "plant_motor_thrust_scales": list(motor_thrust_scales),
            "rotor_drag_scale": float(args.rotor_drag_scale),
            "state_estimate_delay_ms": float(args.state_estimate_delay_ms),
            "position_noise_std_m": float(args.position_noise_std_m),
            "velocity_noise_std_mps": float(args.velocity_noise_std_mps),
            "attitude_noise_std_deg": float(args.attitude_noise_std_deg),
            "gyro_noise_std_deg_s": float(args.gyro_noise_std_deg_s),
            "flow_z_mode": str(args.flow_z_mode),
            "flow_z_max_tilt_deg": float(args.flow_z_max_tilt_deg),
            "flow_z_reacquire_tau_ms": float(args.flow_z_reacquire_tau_ms),
            "flow_z_invalid_fraction": (
                flow_altitude.invalid_samples / max(1, len(flow_altitude.records))
            ),
            "flow_z_maximum_estimation_error_m": float(flow_altitude.maximum_error_m),
            "model_switch_count": int(model_switch_count),
            "final_model_id": int(scheduled_model_id),
            "control_mode": str(args.control_mode),
            "trajectory_file": str(args.trajectory_file) if args.trajectory_file else None,
            "geometric_controller": {
                "position_kp": geometric_position_kp.astype(float).tolist(),
                "velocity_kd": geometric_velocity_kd.astype(float).tolist(),
                "attitude_kp_nm": float(args.geometric_attitude_kp),
                "rate_kd_nms_per_rad": float(args.geometric_rate_kd),
                "motor_clip_fraction": geometric_clip_steps / max(1, controller_update_count),
                "maximum_attitude_error_deg": math.degrees(geometric_max_attitude_error_rad),
                "maximum_rate_error_deg_s": math.degrees(geometric_max_rate_error_rad_s),
            },
            "plant_backend": "gym-pybullet-drones/CF2X-four-motor",
            "avoidance_enabled": not bool(args.no_avoidance),
            "constraint_active_steps": active_count,
            "constraint_active_fraction": active_count / max(1, len(rows)),
            "max_active_halfspaces": int(args.max_active_halfspaces),
            "camera_rate_hz": float(args.camera_rate_hz),
            "perception_mode": str(args.perception_mode),
            "onnx_model": str(args.onnx_model) if args.onnx_model else None,
            "neural_danger_clearance_m": float(args.neural_danger_clearance),
            "neural_plane_margin_m": float(args.neural_plane_margin),
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
            "reference_trajectory": [
                {"t": float(tt), "x": float(pos[0]), "y": float(pos[1]), "z": float(pos[2])}
                for tt, pos in zip(trajectory.t, trajectory.pos)
            ],
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
    with (args.out / "flow_altitude.csv").open("w", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow([
            "t", "delayed_true_z_m", "estimated_z_m", "raw_slant_range_m",
            "body_z_world_z_cosine", "range_valid", "mode",
        ])
        writer.writerows(flow_altitude.records)
    if neural is not None:
        _write_neural_perception(args.out / "neural_perception.csv", neural_rows)
    (args.out / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    print(json.dumps(summary, indent=2))
    print(f"wrote logs to {args.out}")
    if bool(args.plot):
        plotter = Path(__file__).with_name("plot_depth_constraint_run.py")
        subprocess.run([sys.executable, str(plotter), str(args.out)], check=True)
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
            quat_wxyz=np.asarray([[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]]),
            omega_body=np.zeros((2, 3), dtype=np.float64),
            input_delta_n=np.zeros((2, 4), dtype=np.float64),
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
    if all(col in rows[0] for col in ("qw", "qx", "qy", "qz")):
        quat = np.asarray([[float(row[key]) for key in ("qw", "qx", "qy", "qz")] for row in rows], dtype=np.float64)
        norms = np.linalg.norm(quat, axis=1)
        if np.any(norms < 1e-9):
            raise SystemExit(f"trajectory file {args.trajectory_file} contains a zero quaternion")
        quat /= norms[:, None]
        for index in range(1, len(quat)):
            if float(np.dot(quat[index - 1], quat[index])) < 0.0:
                quat[index] *= -1.0
    else:
        quat = np.tile(np.asarray([1.0, 0.0, 0.0, 0.0]), (len(t), 1))
    if all(col in rows[0] for col in ("wx", "wy", "wz")):
        omega = np.asarray([[float(row[key]) for key in ("wx", "wy", "wz")] for row in rows], dtype=np.float64)
    else:
        omega = np.zeros((len(t), 3), dtype=np.float64)
    thrust_columns = tuple(f"motor_{motor}_thrust_n" for motor in range(4))
    hover = GymPybulletGateEnv.FIRMWARE_HOVER_THRUST_N
    if all(col in rows[0] for col in thrust_columns):
        physical = np.asarray([[float(row[key]) for key in thrust_columns] for row in rows], dtype=np.float64)
        input_delta = physical - hover[None, :]
    else:
        input_delta = np.zeros((len(t), 4), dtype=np.float64)
    return ReferenceTrajectory(t=t, pos=pos, vel=vel, quat_wxyz=quat,
                               omega_body=omega, input_delta_n=input_delta)


def _build_admm_host_library(horizon: int, force: bool = False) -> Path:
    build_dir = Path(tempfile.gettempdir()) / "tinympc_admm_host"
    sources = [
        APP_ROOT / "tools" / "pybullet_simulation" / "tinympc_admm_host.cpp",
        APP_ROOT / "TinyMPC-ADMM" / "src" / "tinympc" / "model.cpp",
        APP_ROOT / "TinyMPC-ADMM" / "src" / "tinympc" / "auxil.cpp",
        APP_ROOT / "TinyMPC-ADMM" / "src" / "tinympc" / "cost_lqr.cpp",
        APP_ROOT / "TinyMPC-ADMM" / "src" / "tinympc" / "lqr.cpp",
        APP_ROOT / "TinyMPC-ADMM" / "src" / "tinympc" / "constraint_linear.cpp",
        APP_ROOT / "TinyMPC-ADMM" / "src" / "tinympc" / "admm.cpp",
        APP_ROOT / "TinyMPC-ADMM" / "src" / "tinympc" / "rho_benchmark.cpp",
        APP_ROOT / "TinyMPC-ADMM" / "src" / "tinympc" / "utils.cpp",
    ]
    headers = [
        APP_ROOT / "src" / "tinympc_generated_params.h",
        APP_ROOT / "src" / "tinympc_banked_model_bank.h",
        APP_ROOT / "TinyMPC-ADMM" / "src" / "tinympc" / "types.h",
        APP_ROOT / "TinyMPC-ADMM" / "src" / "tinympc" / "admm.h",
    ]
    fingerprint = hashlib.sha256(f"horizon={int(horizon)};cxx14;o2".encode())
    for path in [*sources, *headers]:
        fingerprint.update(str(path.relative_to(APP_ROOT)).encode())
        fingerprint.update(path.read_bytes())
    build_id = fingerprint.hexdigest()[:16]
    if force:
        build_id += f"-{os.getpid()}-{time.time_ns()}"
    output = build_dir / f"libtinympc_admm_host_n{int(horizon)}-{build_id}.so"
    if output.exists():
        return output
    build_dir.mkdir(parents=True, exist_ok=True)
    staging = build_dir / f".{output.name}.{os.getpid()}.{time.time_ns()}.tmp"
    command = [
        "g++",
        "-std=c++14",
        "-O2",
        "-shared",
        "-fPIC",
        "-DEIGEN_DONT_VECTORIZE",
        f"-DNHORIZON={int(horizon)}",
        f"-I{APP_ROOT / 'TinyMPC-ADMM' / 'src'}",
        f"-I{APP_ROOT / 'TinyMPC-ADMM' / 'ext' / 'Eigen'}",
        *[str(path) for path in sources],
        "-o",
        str(staging),
    ]
    subprocess.run(command, cwd=APP_ROOT, check=True)
    staging.replace(output)
    return output


def _period_steps(period_s: float, plant_dt_s: float, label: str) -> int:
    ratio = float(period_s) / float(plant_dt_s)
    rounded = int(round(ratio))
    if rounded < 1 or not math.isclose(ratio, rounded, rel_tol=0.0, abs_tol=1e-8):
        raise SystemExit(f"{label} period {period_s:g}s is not an integer multiple of plant dt {plant_dt_s:g}s")
    return rounded


def _nonnegative_delay_steps(delay_s: float, plant_dt_s: float, label: str) -> int:
    if float(delay_s) < 0.0:
        raise SystemExit(f"{label} delay must be nonnegative")
    ratio = float(delay_s) / float(plant_dt_s)
    rounded = int(round(ratio))
    if not math.isclose(ratio, rounded, rel_tol=0.0, abs_tol=1.0e-8):
        raise SystemExit(f"{label} delay {delay_s:g}s is not an integer multiple of plant dt {plant_dt_s:g}s")
    return rounded


def _parse_motor_thrust_scales(value: str) -> tuple[float, float, float, float]:
    try:
        scales = tuple(float(item.strip()) for item in str(value).split(","))
    except ValueError as exc:
        raise SystemExit("--plant-motor-thrust-scales must contain four numbers") from exc
    if len(scales) != 4 or min(scales) <= 0.0 or not all(math.isfinite(item) for item in scales):
        raise SystemExit("--plant-motor-thrust-scales must contain four positive finite numbers")
    return scales  # type: ignore[return-value]


def _parse_positive_vector3(value: str, label: str) -> np.ndarray:
    try:
        values = np.asarray([float(item.strip()) for item in str(value).split(",")])
    except ValueError as exc:
        raise SystemExit(f"{label} must contain three numbers") from exc
    if values.shape != (3,) or np.any(values <= 0.0) or not np.all(np.isfinite(values)):
        raise SystemExit(f"{label} must contain three positive finite numbers")
    return values


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
    _nonnegative_delay_steps(
        1.0e-3 * float(args.state_estimate_delay_ms), float(args.plant_dt), "state estimate"
    )
    _nonnegative_delay_steps(
        1.0e-3 * float(args.controller_compute_delay_ms), float(args.plant_dt), "controller compute"
    )
    if float(args.trajectory_handoff_hold_s) < 0.0:
        raise SystemExit("--trajectory-handoff-hold-s must be nonnegative")
    for name in (
        "position_noise_std_m", "velocity_noise_std_mps",
        "attitude_noise_std_deg", "gyro_noise_std_deg_s", "rotor_drag_scale",
    ):
        if float(getattr(args, name)) < 0.0:
            raise SystemExit(f"--{name.replace('_', '-')} must be nonnegative")
    if not 0 <= int(args.start_k) < int(args.constraint_end_k) <= int(args.horizon):
        raise SystemExit("need 0 <= --start-k < --constraint-end-k <= --horizon")
    if str(args.model_schedule) in ("bank15", "bank30", "bank60") and (
        not math.isclose(float(args.model_dt), 0.02, abs_tol=1e-9)
        or not math.isclose(float(args.tinympc_rho), 250.0, abs_tol=1e-6)
    ):
        raise SystemExit("bank model schedules require offline-cache settings --model-dt 0.02 --tinympc-rho 250")


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
    if hit.obstacle_name == "neural_two_or_more_dangerous":
        margin = float(args.neural_plane_margin)
    else:
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


def _hat(vector: np.ndarray) -> np.ndarray:
    x, y, z = np.asarray(vector, dtype=np.float64)
    return np.asarray([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]])


def _vee(skew: np.ndarray) -> np.ndarray:
    matrix = np.asarray(skew, dtype=np.float64)
    return np.asarray([matrix[2, 1], matrix[0, 2], matrix[1, 0]])


def _reference_derivatives(
    trajectory: ReferenceTrajectory, query_t: float, derivative_dt_s: float,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    position, velocity, quaternion, omega, input_delta = trajectory.sample_full(query_t)
    dt = max(1.0e-5, float(derivative_dt_s))
    before_t = max(float(trajectory.t[0]), float(query_t) - dt)
    after_t = min(float(trajectory.t[-1]), float(query_t) + dt)
    denominator = max(1.0e-9, after_t - before_t)
    _, velocity_before, _, omega_before, _ = trajectory.sample_full(before_t)
    _, velocity_after, _, omega_after, _ = trajectory.sample_full(after_t)
    acceleration = (velocity_after - velocity_before) / denominator
    omega_dot = (omega_after - omega_before) / denominator
    return position, velocity, quaternion, omega, acceleration, omega_dot


def _quaternion_product_wxyz(left: np.ndarray, right: np.ndarray) -> np.ndarray:
    aw, ax, ay, az = np.asarray(left, dtype=np.float64)
    bw, bx, by, bz = np.asarray(right, dtype=np.float64)
    return np.asarray([
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    ])


def _tinympc_reference_error_state(
    estimated_world_state: np.ndarray,
    estimated_quaternion_xyzw: np.ndarray,
    trajectory: ReferenceTrajectory,
    query_t: float,
) -> tuple[np.ndarray, np.ndarray]:
    """Put the physical state in a nonsingular chart centered on the reference."""
    ref_position, ref_velocity, ref_quaternion, ref_omega, ref_input_delta = (
        trajectory.sample_full(query_t)
    )
    ref_rotation = _quat_xyzw_to_rot(
        np.asarray([ref_quaternion[1], ref_quaternion[2], ref_quaternion[3], ref_quaternion[0]])
    )
    actual_quaternion = np.asarray([
        estimated_quaternion_xyzw[3], estimated_quaternion_xyzw[0],
        estimated_quaternion_xyzw[1], estimated_quaternion_xyzw[2],
    ])
    actual_quaternion /= np.linalg.norm(actual_quaternion)
    ref_conjugate = ref_quaternion * np.asarray([1.0, -1.0, -1.0, -1.0])
    error_quaternion = _quaternion_product_wxyz(ref_conjugate, actual_quaternion)
    if error_quaternion[0] < 0.0:
        error_quaternion *= -1.0
    error_quaternion /= np.linalg.norm(error_quaternion)
    denominator = max(1.0e-8, float(error_quaternion[0]))
    actual_rotation = _quat_xyzw_to_rot(estimated_quaternion_xyzw)
    reference_omega_in_actual_body = actual_rotation.T @ ref_rotation @ ref_omega
    error_state = np.zeros(AdmmHostSolver.NSTATES, dtype=np.float64)
    error_state[0:3] = ref_rotation.T @ (
        np.asarray(estimated_world_state[0:3]) - ref_position
    )
    error_state[3:6] = error_quaternion[1:4] / denominator
    error_state[6:9] = ref_rotation.T @ (
        np.asarray(estimated_world_state[6:9]) - ref_velocity
    )
    error_state[9:12] = np.asarray(estimated_world_state[9:12]) - reference_omega_in_actual_body
    return error_state, np.asarray(ref_input_delta, dtype=np.float64)


def _geometric_motor_control(
    position: np.ndarray,
    velocity: np.ndarray,
    quaternion_xyzw: np.ndarray,
    omega_body: np.ndarray,
    trajectory: ReferenceTrajectory,
    query_t: float,
    args: argparse.Namespace,
    position_kp: np.ndarray,
    velocity_kd: np.ndarray,
) -> tuple[np.ndarray, dict[str, float]]:
    """Chart-safe SE(3) tracking control returning firmware motor-thrust deltas."""
    ref_position, ref_velocity, ref_quaternion, ref_omega, ref_acceleration, ref_omega_dot = (
        _reference_derivatives(trajectory, query_t, float(args.model_dt))
    )
    rotation = _quat_xyzw_to_rot(np.asarray(quaternion_xyzw, dtype=np.float64))
    ref_rotation = _quat_xyzw_to_rot(
        np.asarray([ref_quaternion[1], ref_quaternion[2], ref_quaternion[3], ref_quaternion[0]])
    )
    mass = float(np.sum(GymPybulletGateEnv.FIRMWARE_HOVER_THRUST_N)) / GymPybulletGateEnv.GRAVITY_MPS2
    inertia = np.diag(GymPybulletGateEnv.FIRMWARE_INERTIA_KGM2)
    desired_force_world = mass * (
        ref_acceleration
        + position_kp * (ref_position - np.asarray(position, dtype=np.float64))
        + velocity_kd * (ref_velocity - np.asarray(velocity, dtype=np.float64))
        + np.asarray([0.0, 0.0, GymPybulletGateEnv.GRAVITY_MPS2])
    )
    # During the chart-crossing portion the explicit quaternion is authoritative.
    # Once the reference is upright and no longer rotating, recover ordinary
    # SE(3) position authority by tilting body-z into the requested force.
    if float(np.linalg.norm(ref_omega)) < 0.05 and float(ref_rotation[2, 2]) > 0.95:
        force_norm = float(np.linalg.norm(desired_force_world))
        if force_norm > 1.0e-9:
            desired_z = desired_force_world / force_norm
            heading = ref_rotation[:, 0]
            desired_y = np.cross(desired_z, heading)
            desired_y_norm = float(np.linalg.norm(desired_y))
            if desired_y_norm > 1.0e-9:
                desired_y /= desired_y_norm
                desired_x = np.cross(desired_y, desired_z)
                ref_rotation = np.column_stack((desired_x, desired_y, desired_z))
                ref_omega = np.zeros(3, dtype=np.float64)
                ref_omega_dot = np.zeros(3, dtype=np.float64)
    collective = float(np.dot(desired_force_world, rotation[:, 2]))

    rotation_error = 0.5 * _vee(ref_rotation.T @ rotation - rotation.T @ ref_rotation)
    desired_omega_in_body = rotation.T @ ref_rotation @ ref_omega
    omega_error = np.asarray(omega_body, dtype=np.float64) - desired_omega_in_body
    transport = (
        _hat(np.asarray(omega_body, dtype=np.float64)) @ desired_omega_in_body
        - rotation.T @ ref_rotation @ ref_omega_dot
    )
    moment = (
        -float(args.geometric_attitude_kp) * rotation_error
        -float(args.geometric_rate_kd) * omega_error
        + np.cross(omega_body, inertia @ omega_body)
        - inertia @ transport
    )
    arm = GymPybulletGateEnv.FIRMWARE_ARM_OFFSET_M
    yaw_ratio = GymPybulletGateEnv.FIRMWARE_THRUST_TO_YAW_TORQUE_M
    allocation = np.asarray([
        [1.0, 1.0, 1.0, 1.0],
        [-arm, -arm, arm, arm],
        [-arm, arm, arm, -arm],
        [-yaw_ratio, yaw_ratio, -yaw_ratio, yaw_ratio],
    ])
    physical_motor_thrust = np.linalg.solve(allocation, np.r_[collective, moment])
    maximum = GymPybulletGateEnv.FIRMWARE_MAX_MOTOR_THRUST_N
    clipped = np.clip(physical_motor_thrust, 0.0, maximum)
    return clipped - GymPybulletGateEnv.FIRMWARE_HOVER_THRUST_N, {
        "collective_thrust_n": collective,
        "attitude_error_rad": float(np.linalg.norm(rotation_error)),
        "rate_error_rad_s": float(np.linalg.norm(omega_error)),
        "unclipped_min_motor_thrust_n": float(np.min(physical_motor_thrust)),
        "unclipped_max_motor_thrust_n": float(np.max(physical_motor_thrust)),
        "motor_clipped": float(np.any(np.abs(clipped - physical_motor_thrust) > 1.0e-10)),
    }


def _rodrigues_to_rpy(rodrigues: np.ndarray) -> np.ndarray:
    quat = np.concatenate(([1.0], np.asarray(rodrigues, dtype=np.float64)))
    quat /= np.linalg.norm(quat)
    qw, qx, qy, qz = quat
    return np.asarray([
        math.atan2(2.0 * (qw * qx + qy * qz), 1.0 - 2.0 * (qx * qx + qy * qy)),
        math.asin(float(np.clip(2.0 * (qw * qy - qz * qx), -1.0, 1.0))),
        math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz)),
    ])


def _noisy_state_estimate(
    world_state: np.ndarray, quat_xyzw: np.ndarray, args: argparse.Namespace,
    rng: np.random.Generator,
    flow_altitude: FlowDeckAltitudeEstimator | None = None,
    timestamp: float = 0.0,
) -> tuple[np.ndarray, np.ndarray]:
    """Apply estimator-like white noise to a delayed ground-truth snapshot."""
    world = np.asarray(world_state, dtype=np.float64).copy()
    quat_true = np.asarray(
        [quat_xyzw[3], quat_xyzw[0], quat_xyzw[1], quat_xyzw[2]], dtype=np.float64
    )
    quat_true /= np.linalg.norm(quat_true)
    world[0:3] += rng.normal(0.0, float(args.position_noise_std_m), 3)
    world[6:9] += rng.normal(0.0, float(args.velocity_noise_std_mps), 3)
    world[9:12] += rng.normal(0.0, math.radians(float(args.gyro_noise_std_deg_s)), 3)
    rotation_error = rng.normal(0.0, math.radians(float(args.attitude_noise_std_deg)), 3)
    angle = float(np.linalg.norm(rotation_error))
    if angle > 1.0e-12:
        error_quat = np.r_[math.cos(0.5 * angle), math.sin(0.5 * angle) * rotation_error / angle]
        aw, ax, ay, az = quat_true
        bw, bx, by, bz = error_quat
        quat_estimate = np.asarray([
            aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
        ])
    else:
        quat_estimate = quat_true
    quat_estimate /= np.linalg.norm(quat_estimate)
    quat_estimate_xyzw = np.asarray(
        [quat_estimate[1], quat_estimate[2], quat_estimate[3], quat_estimate[0]],
        dtype=np.float64,
    )
    if flow_altitude is not None:
        flow_altitude.apply(world, quat_estimate_xyzw, float(timestamp))
    world[3:6] = quat_estimate[1:4] / math.copysign(
        max(1.0e-9, abs(float(quat_estimate[0]))), float(quat_estimate[0])
    )
    return world, quat_estimate_xyzw


def _firmware_local_state_from_world(
    world_state: np.ndarray, quat_xyzw: np.ndarray
) -> tuple[np.ndarray, np.ndarray, float]:
    """Mirror firmware updateInitialState() from a possibly delayed/noisy estimate."""
    world = np.asarray(world_state, dtype=np.float64).copy()
    quat = np.asarray([quat_xyzw[3], quat_xyzw[0], quat_xyzw[1], quat_xyzw[2]], dtype=np.float64)
    quat /= np.linalg.norm(quat)
    qw, qx, qy, qz = quat
    yaw = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
    half = 0.5 * yaw
    yaw_inverse = np.asarray([math.cos(half), 0.0, 0.0, -math.sin(half)])
    aw, ax, ay, az = yaw_inverse
    bw, bx, by, bz = quat
    local_quat = np.asarray([
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    ])
    c, s = math.cos(yaw), math.sin(yaw)
    local = world.copy()
    local[0:3] = 0.0
    local[3:6] = local_quat[1:4] / math.copysign(max(1e-8, abs(local_quat[0])), local_quat[0])
    local[6:9] = np.asarray([c * world[6] + s * world[7], -s * world[6] + c * world[7], world[8]])
    return local, world[0:3].copy(), yaw


def _firmware_local_state(env: GymPybulletGateEnv, state: np.ndarray) -> tuple[np.ndarray, np.ndarray, float]:
    """Backward-compatible exact-state firmware-local conversion."""
    return _firmware_local_state_from_world(_solver_state(env, state), _env_quat(env))


def _scheduled_model_id(
    trajectory: ReferenceTrajectory, t: float, args: argparse.Namespace, current: int,
    hold_reference: bool = False,
) -> int:
    schedule = str(args.model_schedule)
    if schedule not in ("bank15", "bank30", "bank60"):
        return 0
    query_t = 0.0 if hold_reference else (
        float(t) + 0.5 * (int(args.horizon) - 1) * float(args.model_dt)
    )
    _, _, quat, _, _ = trajectory.sample_full(query_t)
    qw, qx, qy, qz = quat
    roll = math.atan2(2.0 * (qw * qx + qy * qz), 1.0 - 2.0 * (qx * qx + qy * qy))
    if schedule == "bank60":
        # Do not jump directly from hover to the 60-degree operating point.
        # Select the closest stored chart as the reference traverses the bank;
        # all matrices and Riccati caches are still generated offline.
        magnitude = abs(math.degrees(roll))
        if magnitude >= 45.0:
            return 5 if roll < 0.0 else 6
        if magnitude >= 22.5:
            return 3 if roll < 0.0 else 4
        if magnitude >= 7.5:
            return 1 if roll < 0.0 else 2
        return 0
    if schedule == "bank15":
        bank_angle, left_id, right_id = 15.0, 1, 2
    else:
        bank_angle, left_id, right_id = 30.0, 3, 4
    enter = math.radians(0.5 * bank_angle)
    exit_angle = math.radians(bank_angle / 3.0)
    if current == left_id and roll < -exit_angle:
        return left_id
    if current == right_id and roll > exit_angle:
        return right_id
    if roll <= -enter:
        return left_id
    if roll >= enter:
        return right_id
    return 0


def _reference_trajectory(
    x0: np.ndarray,
    t: float,
    trajectory: ReferenceTrajectory,
    constraint: HalfspaceConstraint,
    args: argparse.Namespace,
    horizon: int,
    frame_origin: np.ndarray | None = None,
    frame_yaw: float = 0.0,
    hold_reference: bool = False,
) -> tuple[np.ndarray, np.ndarray]:
    x_ref = np.zeros((int(horizon), AdmmHostSolver.NSTATES), dtype=np.float64)
    u_ref = np.zeros((int(horizon) - 1, AdmmHostSolver.NINPUTS), dtype=np.float64)
    for k in range(int(horizon)):
        query_t = 0.0 if hold_reference else float(t) + k * float(args.model_dt)
        ref_pos, ref_vel, ref_quat, ref_omega, ref_input = trajectory.sample_full(
            query_t
        )
        if constraint.active and float(args.admm_reference_sidestep) > 0.0:
            obstacle_side = float(constraint.obstacle_center[1] - x0[1])
            side = -math.copysign(1.0, obstacle_side) if abs(obstacle_side) > 1e-3 else -1.0
            ref_pos = ref_pos.copy()
            ref_pos[1] += side * float(args.admm_reference_sidestep)
        # Mirror firmware applyRaceIntent(): project every reference knot onto
        # the feasible side before installing the same position half-space.
        if constraint.active:
            violation = float(np.dot(constraint.a, ref_pos) - constraint.b)
            if violation > 0.0:
                ref_pos = ref_pos.copy() - violation * constraint.a
        if frame_origin is not None:
            c, s = math.cos(frame_yaw), math.sin(frame_yaw)
            delta = ref_pos - frame_origin
            ref_pos = np.asarray([c * delta[0] + s * delta[1], -s * delta[0] + c * delta[1], delta[2]])
            ref_vel = np.asarray([c * ref_vel[0] + s * ref_vel[1], -s * ref_vel[0] + c * ref_vel[1], ref_vel[2]])
            half = -0.5 * frame_yaw
            aw, ax, ay, az = math.cos(half), 0.0, 0.0, math.sin(half)
            bw, bx, by, bz = ref_quat
            ref_quat = np.asarray([
                aw * bw - ax * bx - ay * by - az * bz,
                aw * bx + ax * bw + ay * bz - az * by,
                aw * by - ax * bz + ay * bw + az * bx,
                aw * bz + ax * by - ay * bx + az * bw,
            ])
        x_ref[k, 0:3] = ref_pos
        denominator = math.copysign(max(1e-8, abs(float(ref_quat[0]))), float(ref_quat[0]))
        x_ref[k, 3:6] = ref_quat[1:4] / denominator
        x_ref[k, 6:9] = ref_vel
        x_ref[k, 9:12] = ref_omega
        if k < int(horizon) - 1:
            # Match firmware updateHorizonReference(): infer collective
            # specific force from adjacent velocity samples and give every
            # motor the same hover-scaled reference.  Any per-motor thrust
            # columns in a simulation CSV are intentionally ignored.
            _, next_vel, _, _, _ = trajectory.sample_full(
                query_t + float(args.model_dt)
            )
            specific_force = (next_vel - ref_vel) / float(args.model_dt)
            specific_force[2] += GymPybulletGateEnv.GRAVITY_MPS2
            thrust_scale = np.linalg.norm(specific_force) / GymPybulletGateEnv.GRAVITY_MPS2 - 1.0
            collective_delta = float(GymPybulletGateEnv.FIRMWARE_HOVER_THRUST_N[0]) * thrust_scale
            lower = -float(GymPybulletGateEnv.FIRMWARE_HOVER_THRUST_N[0])
            upper = float(GymPybulletGateEnv.FIRMWARE_MAX_MOTOR_THRUST_N) - float(
                GymPybulletGateEnv.FIRMWARE_HOVER_THRUST_N[0]
            )
            u_ref[k].fill(float(np.clip(collective_delta, lower, upper)))
    return x_ref, u_ref


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
    minimum_forward = 0.0 if constraint.obstacle_name == "neural_two_or_more_dangerous" else 0.05
    velocity[0] = float(np.clip(velocity[0], minimum_forward, float(args.max_forward_speed)))
    if constraint.active and float(args.admm_forward_slack_scale) > 0.0:
        forward_cap = max(minimum_forward, float(args.admm_forward_slack_scale) * max(0.0, constraint.current_slack_m) / max(1e-3, float(args.lookahead_s)))
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
    minimum_forward = 0.0 if constraint.obstacle_name == "neural_two_or_more_dangerous" else 0.05
    forward_cap = max(minimum_forward, 0.75 * max(0.0, constraint.current_slack_m) / max(1e-3, lookahead))
    adjusted[0] = min(float(adjusted[0]), forward_cap)
    adjusted[0] = float(np.clip(adjusted[0], minimum_forward, float(args.max_forward_speed)))
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
    frame_origin: np.ndarray | None = None,
    frame_yaw: float = 0.0,
) -> None:
    if solver_info.get("states") is None:
        return
    states = np.asarray(solver_info["states"], dtype=np.float64).reshape(-1, AdmmHostSolver.NSTATES)
    for k, state in enumerate(states):
        position = state[0:3].copy()
        velocity = state[6:9].copy()
        if frame_origin is not None:
            c, s = math.cos(frame_yaw), math.sin(frame_yaw)
            position = frame_origin + np.asarray([
                c * position[0] - s * position[1],
                s * position[0] + c * position[1],
                position[2],
            ])
            velocity = np.asarray([
                c * velocity[0] - s * velocity[1],
                s * velocity[0] + c * velocity[1],
                velocity[2],
            ])
        rows.append(
            [
                step,
                f"{t:.9f}",
                k,
                f"{float(position[0]):.9f}",
                f"{float(position[1]):.9f}",
                f"{float(position[2]):.9f}",
                f"{float(velocity[0]):.9f}",
                f"{float(velocity[1]):.9f}",
                f"{float(velocity[2]):.9f}",
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
    controller_state: np.ndarray,
    quaternion_xyzw: np.ndarray,
) -> list[object]:
    terminal = np.full(3, math.nan, dtype=np.float64)
    if solver_info.get("states") is not None:
        terminal = np.asarray(solver_info["states"], dtype=np.float64)[-1, 0:3]
    motor_thrust = list(info.get("motor_thrust_n", [math.nan] * 4))
    motor_rpm = list(info.get("motor_rpm", [math.nan] * 4))
    input_baseline = list(solver_info.get("input_baseline_n", [math.nan] * 4))
    control_correction = list(
        solver_info.get("first_control_correction_n", [math.nan] * 4)
    )
    commanded_motor_thrust = list(
        solver_info.get("commanded_physical_motor_thrust_n", [math.nan] * 4)
    )
    controller_state = np.asarray(controller_state, dtype=np.float64).reshape(12)
    quaternion_xyzw = np.asarray(quaternion_xyzw, dtype=np.float64).reshape(4)
    rpy = _rodrigues_to_rpy(controller_state[3:6])
    return [
        step,
        f"{t:.9f}",
        *[f"{float(v):.9f}" for v in state[:6]],
        *[f"{float(v):.9f}" for v in next_state[:6]],
        *[f"{float(v):.9f}" for v in controller_state[3:6]],
        f"{float(quaternion_xyzw[3]):.9f}",
        f"{float(quaternion_xyzw[0]):.9f}",
        f"{float(quaternion_xyzw[1]):.9f}",
        f"{float(quaternion_xyzw[2]):.9f}",
        *[f"{float(v):.9f}" for v in rpy],
        *[f"{float(v):.9f}" for v in controller_state[9:12]],
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
        solver_info.get("model_id", ""),
        "" if not solver_info else f"{float(solver_info.get('pri_res', math.nan)):.9f}",
        "" if not solver_info else f"{float(solver_info.get('dua_res', math.nan)):.9f}",
        "" if not math.isfinite(float(terminal[0])) else f"{float(terminal[0]):.9f}",
        "" if not math.isfinite(float(terminal[1])) else f"{float(terminal[1]):.9f}",
        "" if not math.isfinite(float(terminal[2])) else f"{float(terminal[2]):.9f}",
        *["" if not math.isfinite(float(v)) else f"{float(v):.9f}" for v in motor_thrust],
        *["" if not math.isfinite(float(v)) else f"{float(v):.3f}" for v in motor_rpm],
        *["" if not math.isfinite(float(v)) else f"{float(v):.9f}" for v in input_baseline],
        *["" if not math.isfinite(float(v)) else f"{float(v):.9f}" for v in control_correction],
        *["" if not math.isfinite(float(v)) else f"{float(v):.9f}" for v in commanded_motor_thrust],
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
                "rod_x",
                "rod_y",
                "rod_z",
                "quat_w",
                "quat_x",
                "quat_y",
                "quat_z",
                "roll_rad",
                "pitch_rad",
                "yaw_rad",
                "body_wx_rad_s",
                "body_wy_rad_s",
                "body_wz_rad_s",
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
                "tinympc_model_id",
                "admm_pri_res",
                "admm_dua_res",
                "admm_terminal_x",
                "admm_terminal_y",
                "admm_terminal_z",
                "motor_0_thrust_n",
                "motor_1_thrust_n",
                "motor_2_thrust_n",
                "motor_3_thrust_n",
                "motor_0_rpm",
                "motor_1_rpm",
                "motor_2_rpm",
                "motor_3_rpm",
                "tinympc_baseline_motor_0_thrust_n",
                "tinympc_baseline_motor_1_thrust_n",
                "tinympc_baseline_motor_2_thrust_n",
                "tinympc_baseline_motor_3_thrust_n",
                "tinympc_correction_motor_0_thrust_n",
                "tinympc_correction_motor_1_thrust_n",
                "tinympc_correction_motor_2_thrust_n",
                "tinympc_correction_motor_3_thrust_n",
                "commanded_motor_0_thrust_n",
                "commanded_motor_1_thrust_n",
                "commanded_motor_2_thrust_n",
                "commanded_motor_3_thrust_n",
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


def _write_neural_perception(path: Path, rows: list[list[object]]) -> None:
    """Write the unfiltered ONNX values at the camera rate, not plant rate."""
    with path.open("w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow([
            "step", "t",
            "clearance_0_m", "clearance_1_m", "clearance_2_m", "clearance_3_m",
            "confidence_0", "confidence_1", "confidence_2", "confidence_3",
            "dangerous_0", "dangerous_1", "dangerous_2", "dangerous_3",
            "plane_activated", "decision",
        ])
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
