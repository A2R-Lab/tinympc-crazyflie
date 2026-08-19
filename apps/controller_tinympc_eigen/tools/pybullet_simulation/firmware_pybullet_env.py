"""gym-pybullet-drones CF2X plant adapter for the TinyMPC firmware shadow."""

from __future__ import annotations

from dataclasses import dataclass
import math
from collections import deque

import numpy as np
import pybullet as p

try:
    from gym_pybullet_drones.control.DSLPIDControl import DSLPIDControl
    from gym_pybullet_drones.envs.CtrlAviary import CtrlAviary
    from gym_pybullet_drones.utils.enums import DroneModel, Physics
except ImportError as exc:  # pragma: no cover - gives a useful CLI failure
    raise ImportError(
        "gym-pybullet-drones is required; install tools/pybullet_simulation/requirements.txt"
    ) from exc


@dataclass(frozen=True)
class GateGeometry:
    gate_x: float
    gate_y: float
    gate_z: float
    half_width: float
    half_height: float
    drone_radius: float


@dataclass(frozen=True)
class GymPybulletGateEnvConfig:
    dt: float = 0.002
    image_size: int = 160
    fov_deg: float = 70.0
    max_accel: float = 0.8
    initial_x: float = 0.0
    initial_y: float = 0.0
    initial_z: float = 1.1
    initial_vx: float = 0.0
    initial_vy: float = 0.0
    initial_vz: float = 0.0
    ctrl_freq: int = 500
    gui: bool = False
    max_forward_speed: float = 0.6
    max_lateral_speed: float = 0.25
    max_vertical_speed: float = 0.7
    command_lookahead_s: float = 0.55
    target_y_limit: float = 1.4
    target_z_min: float = 0.45
    target_z_max: float = 2.0
    gate_collision: bool = False
    motor_time_constant_s: float = 0.0
    motor_command_delay_s: float = 0.0
    mass_scale: float = 1.0
    inertia_scale: float = 1.0
    thrust_scale: float = 1.0
    motor_thrust_scales: tuple[float, float, float, float] = (1.0, 1.0, 1.0, 1.0)
    rotor_drag_scale: float = 0.0


class _ClientView:
    def __init__(self, client: int) -> None:
        self._client = client
        self.quat = np.asarray([[0.0, 0.0, 0.0, 1.0]], dtype=np.float64)

    def getPyBulletClient(self) -> int:
        return self._client


class GymPybulletGateEnv:
    """Four-motor Crazyflie plant backed by gym-pybullet-drones.

    TinyMPC outputs physical motor thrust deviations. ``step_motor_thrust``
    adds the firmware hover thrust and converts each motor to the RPM expected
    by ``CtrlAviary``. Projection-mode experiments use the library's geometric
    PID only as a compatibility path.
    """

    FIRMWARE_HOVER_THRUST_N = np.full(4, 0.1103625, dtype=np.float64)
    # Values used to generate src/tinympc_generated_params.h (brushless,
    # AI-deck + Flow deck). The generated model's arm offset is the x/y
    # coordinate of each motor, not the center-to-motor diagonal distance.
    FIRMWARE_INERTIA_KGM2 = np.asarray([2.3951e-5, 2.3951e-5, 3.2347e-5], dtype=np.float64)
    FIRMWARE_ARM_OFFSET_M = 0.03535
    FIRMWARE_THRUST_COEFF_NS2_PER_RAD2 = 3.72e-8
    FIRMWARE_THRUST_TO_YAW_TORQUE_M = 7.73e-11 / 3.72e-8
    FIRMWARE_MAX_MOTOR_THRUST_N = FIRMWARE_THRUST_COEFF_NS2_PER_RAD2 * 2900.0**2
    GRAVITY_MPS2 = 9.81

    def __init__(self, geometry: GateGeometry, config: GymPybulletGateEnvConfig) -> None:
        self.geometry = geometry
        self.config = config
        if float(config.motor_time_constant_s) < 0.0 or float(config.motor_command_delay_s) < 0.0:
            raise ValueError("motor lag and command delay must be nonnegative")
        if min(float(config.mass_scale), float(config.inertia_scale), float(config.thrust_scale)) <= 0.0:
            raise ValueError("mass, inertia, and thrust scales must be positive")
        if len(config.motor_thrust_scales) != 4 or min(float(v) for v in config.motor_thrust_scales) <= 0.0:
            raise ValueError("motor thrust scales must contain four positive values")
        if float(config.rotor_drag_scale) < 0.0:
            raise ValueError("rotor drag scale must be nonnegative")
        frequency = int(round(1.0 / float(config.dt)))
        if frequency != int(config.ctrl_freq):
            raise ValueError("plant dt and ctrl_freq must describe the same frequency")
        self._aviary = CtrlAviary(
            drone_model=DroneModel.CF2X,
            num_drones=1,
            initial_xyzs=np.asarray([[config.initial_x, config.initial_y, config.initial_z]], dtype=float),
            initial_rpys=np.zeros((1, 3)),
            physics=Physics.PYB,
            pyb_freq=frequency,
            ctrl_freq=frequency,
            gui=bool(config.gui),
            user_debug_gui=False,
            obstacles=False,
        )
        self._p = p
        self._client = int(self._aviary.getPyBulletClient())
        self.env = _ClientView(self._client)
        self._pid = DSLPIDControl(drone_model=DroneModel.CF2X)
        self._obs = np.zeros((1, 20), dtype=np.float64)
        self._actual_motor_thrust_n = self.FIRMWARE_HOVER_THRUST_N.copy()
        self._command_delay_steps = int(round(float(config.motor_command_delay_s) / float(config.dt)))
        if not math.isclose(
            float(config.motor_command_delay_s), self._command_delay_steps * float(config.dt),
            rel_tol=0.0, abs_tol=1.0e-9,
        ):
            raise ValueError("motor command delay must be an integer multiple of plant dt")
        self._motor_command_queue: deque[np.ndarray] = deque(
            [self.FIRMWARE_HOVER_THRUST_N.copy() for _ in range(self._command_delay_steps)]
        )
        self._set_firmware_dynamics()
        # Retain CtrlAviary's stepping/lifecycle but replace its stock CF2X
        # actuator model with the generated firmware wrench exactly.
        self._aviary._physics = self._firmware_physics

    def _set_firmware_dynamics(self) -> None:
        # Generated firmware hover thrust includes the configured AI/Flow decks
        # and guards, so match that mass instead of the bare 27 g URDF.
        mass = float(np.sum(self.FIRMWARE_HOVER_THRUST_N) / self.GRAVITY_MPS2) * float(self.config.mass_scale)
        inertia = self.FIRMWARE_INERTIA_KGM2 * float(self.config.inertia_scale)
        p.changeDynamics(
            int(self._aviary.DRONE_IDS[0]), -1,
            mass=mass,
            localInertiaDiagonal=inertia.tolist(),
            linearDamping=0.0,
            angularDamping=0.0,
            physicsClientId=self._client,
        )
        self._aviary.M = mass
        self._aviary.J = np.diag(inertia)
        self._aviary.J_INV = np.diag(1.0 / inertia)
        self._aviary.GRAVITY = mass * self.GRAVITY_MPS2
        self._aviary.HOVER_RPM = np.sqrt(self._aviary.GRAVITY / (4.0 * self._aviary.KF))
        p.setGravity(0.0, 0.0, -self.GRAVITY_MPS2, physicsClientId=self._client)

        self._aviary.MAX_RPM = np.sqrt(self.FIRMWARE_MAX_MOTOR_THRUST_N / float(self._aviary.KF))

    def _firmware_physics(self, rpm: np.ndarray, nth_drone: int) -> None:
        """Apply the exact generated motor wrench inside Gym's PyBullet step."""
        thrust = (
            np.asarray(rpm, dtype=np.float64) ** 2
            * float(self._aviary.KF)
            * float(self.config.thrust_scale)
            * np.asarray(self.config.motor_thrust_scales, dtype=np.float64)
        )
        arm = self.FIRMWARE_ARM_OFFSET_M
        torque = np.asarray([
            arm * (-thrust[0] - thrust[1] + thrust[2] + thrust[3]),
            arm * (-thrust[0] + thrust[1] + thrust[2] - thrust[3]),
            self.FIRMWARE_THRUST_TO_YAW_TORQUE_M
            * (-thrust[0] + thrust[1] - thrust[2] + thrust[3]),
        ])
        body = int(self._aviary.DRONE_IDS[nth_drone])
        p.applyExternalForce(
            body, -1, [0.0, 0.0, float(np.sum(thrust))], [0.0, 0.0, 0.0],
            p.LINK_FRAME, physicsClientId=self._client,
        )
        p.applyExternalTorque(
            body, -1, torque.tolist(), p.LINK_FRAME, physicsClientId=self._client,
        )
        if float(self.config.rotor_drag_scale) > 0.0:
            linear_velocity_world, _ = p.getBaseVelocity(body, physicsClientId=self._client)
            # CtrlAviary calls this argument ``rpm``, but its numerical value
            # is only an internal action representation selected to reproduce
            # thrust with the stock URDF KF. Convert it back to the physical
            # propeller angular speed expected by Gym's generic Crazyflie drag model
            # before evaluating the rotor-speed-dependent drag law.
            physical_omega_rad_s = np.sqrt(
                np.maximum(0.0, np.asarray(rpm, dtype=np.float64) ** 2 * float(self._aviary.KF))
                / self.FIRMWARE_THRUST_COEFF_NS2_PER_RAD2
            )
            drag_factor = (
                -float(self.config.rotor_drag_scale)
                * np.asarray(self._aviary.DRAG_COEFF, dtype=np.float64)
                * float(np.sum(physical_omega_rad_s))
            )
            drag_world = drag_factor * np.asarray(linear_velocity_world, dtype=np.float64)
            orientation_xyzw = p.getBasePositionAndOrientation(body, physicsClientId=self._client)[1]
            rotation_wb = np.asarray(
                p.getMatrixFromQuaternion(orientation_xyzw), dtype=np.float64
            ).reshape(3, 3)
            drag_body = rotation_wb.T @ drag_world
            p.applyExternalForce(
                body, -1, drag_body.tolist(), [0.0, 0.0, 0.0],
                p.LINK_FRAME, physicsClientId=self._client,
            )

    def reset(self) -> np.ndarray:
        self._obs, _ = self._aviary.reset()
        self._set_firmware_dynamics()
        self._actual_motor_thrust_n = self.FIRMWARE_HOVER_THRUST_N.copy()
        self._motor_command_queue = deque(
            [self.FIRMWARE_HOVER_THRUST_N.copy() for _ in range(self._command_delay_steps)]
        )
        p.resetBaseVelocity(
            int(self._aviary.DRONE_IDS[0]),
            [self.config.initial_vx, self.config.initial_vy, self.config.initial_vz],
            [0.0, 0.0, 0.0], physicsClientId=self._client,
        )
        self._refresh_obs()
        return self.get_state()

    def _refresh_obs(self) -> None:
        # Keep direct PyBullet resets (used by deterministic plant/perception
        # validation) synchronized with Gym's kinematic cache.
        self._aviary._updateAndStoreKinematicInformation()
        self._obs[0] = self._aviary._getDroneStateVector(0)
        self.env.quat[0] = self._obs[0, 3:7]

    def get_state(self) -> np.ndarray:
        self._refresh_obs()
        return np.r_[self._obs[0, 0:3], self._obs[0, 10:13]].astype(np.float64)

    def get_controller_state(self) -> np.ndarray:
        self._refresh_obs()
        quat = self._obs[0, 3:7]
        vector = np.asarray(quat[:3], dtype=np.float64)
        # The generated model reconstructs q = normalize([1, r]), therefore
        # its Rodrigues state is r = q_xyz/q_w (tan(theta/2) axis), not the
        # modified Rodrigues parameter q_xyz/(1+q_w).
        qw = float(quat[3])
        denominator = math.copysign(max(1.0e-9, abs(qw)), qw)
        rodrigues = vector / denominator
        rotation_wb = np.asarray(p.getMatrixFromQuaternion(quat), dtype=np.float64).reshape(3, 3)
        angular_velocity_body = rotation_wb.T @ self._obs[0, 13:16]
        return np.r_[
            self._obs[0, 0:3], rodrigues, self._obs[0, 10:13], angular_velocity_body
        ].astype(np.float64)

    def step_motor_thrust(self, thrust_delta_n: np.ndarray):
        delta = np.asarray(thrust_delta_n, dtype=np.float64).reshape(4)
        commanded_thrust = np.clip(self.FIRMWARE_HOVER_THRUST_N + delta, 0.0, None)
        commanded_thrust = np.minimum(commanded_thrust, self.FIRMWARE_MAX_MOTOR_THRUST_N)
        if self._command_delay_steps > 0:
            self._motor_command_queue.append(commanded_thrust.copy())
            delayed_thrust = self._motor_command_queue.popleft()
        else:
            delayed_thrust = commanded_thrust
        tau = float(self.config.motor_time_constant_s)
        if tau > 0.0:
            alpha = 1.0 - math.exp(-float(self.config.dt) / tau)
            self._actual_motor_thrust_n += alpha * (delayed_thrust - self._actual_motor_thrust_n)
        else:
            self._actual_motor_thrust_n = delayed_thrust.copy()
        rpm = np.sqrt(self._actual_motor_thrust_n / float(self._aviary.KF))
        rpm = np.clip(rpm, 0.0, float(self._aviary.MAX_RPM))
        obs, reward, terminated, truncated, info = self._aviary.step(rpm.reshape(1, 4))
        self._obs = np.asarray(obs, dtype=np.float64)
        self._refresh_obs()
        collision = self._collision()
        info = dict(info)
        info.update({
            "collision": collision,
            "motor_rpm": rpm.tolist(),
            "motor_thrust_n": (
                self._actual_motor_thrust_n
                * float(self.config.thrust_scale)
                * np.asarray(self.config.motor_thrust_scales, dtype=np.float64)
            ).tolist(),
            "motor_command_thrust_n": commanded_thrust.tolist(),
        })
        return self.get_state(), reward, bool(terminated or truncated or collision), info

    def step_velocity(self, velocity: np.ndarray):
        self._refresh_obs()
        velocity = np.asarray(velocity, dtype=np.float64).reshape(3)
        target = self._obs[0, 0:3] + velocity * float(self.config.command_lookahead_s)
        rpm, _, _ = self._pid.computeControlFromState(
            control_timestep=float(self.config.dt), state=self._obs[0],
            target_pos=target, target_vel=velocity,
        )
        obs, reward, terminated, truncated, info = self._aviary.step(np.asarray(rpm).reshape(1, 4))
        self._obs = np.asarray(obs, dtype=np.float64)
        self._refresh_obs()
        collision = self._collision()
        info = dict(info)
        info.update({"collision": collision, "motor_rpm": np.asarray(rpm).astype(float).tolist()})
        return self.get_state(), reward, bool(terminated or truncated or collision), info

    def _collision(self) -> bool:
        body = int(self._aviary.DRONE_IDS[0])
        return bool(p.getContactPoints(bodyA=body, physicsClientId=self._client))

    def render_camera(self) -> np.ndarray:
        self._refresh_obs()
        eye = self._obs[0, 0:3]
        rotation = np.asarray(p.getMatrixFromQuaternion(self._obs[0, 3:7]), dtype=float).reshape(3, 3)
        forward = rotation @ np.array([1.0, 0.0, 0.0])
        up = rotation @ np.array([0.0, 0.0, 1.0])
        view = p.computeViewMatrix(eye.tolist(), (eye + forward).tolist(), up.tolist())
        projection = p.computeProjectionMatrixFOV(float(self.config.fov_deg), 1.0, 0.05, 8.0)
        _, _, rgba, _, _ = p.getCameraImage(
            self.config.image_size, self.config.image_size, view, projection,
            renderer=p.ER_TINY_RENDERER, physicsClientId=self._client,
        )
        rgb = np.asarray(rgba, dtype=np.uint8).reshape(
            self.config.image_size, self.config.image_size, 4
        )[..., :3]
        return np.clip(np.rint(rgb @ np.array([0.299, 0.587, 0.114])), 0, 255).astype(np.uint8)

    def close(self) -> None:
        self._aviary.close()
