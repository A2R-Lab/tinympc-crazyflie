#!/usr/bin/env python3
"""Capture a synchronized optical-flow dataset from tinympc-vision PyBullet.

The output layout is intentionally the same shape as a future hardware capture:

  dataset/
    calibration.json
    frames.csv
    telemetry.csv
    frames/frame_000000.png ...
    depth/frame_000000.npy ...

`offline_flow_harness.py` consumes this directly.  The default motion is a
translation-dominated forward approach toward the simulated gate, which gives a
known front-range truth signal before any GAP8 or hardware capture work exists.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
import sys

import numpy as np


DEFAULT_VISION_ROOT = Path(__file__).resolve().parents[4] / "tinympc-vision"


def parse_args() -> argparse.Namespace:
    ap = argparse.ArgumentParser()
    ap.add_argument("--vision-root", type=Path, default=DEFAULT_VISION_ROOT)
    ap.add_argument("--out", type=Path, default=Path("flow_sim_dataset"))
    ap.add_argument("--duration", type=float, default=4.0)
    ap.add_argument("--dt", type=float, default=1.0 / 30.0)
    ap.add_argument("--image-size", type=int, default=160)
    ap.add_argument("--fov-deg", type=float, default=70.0)
    ap.add_argument("--gate-x", type=float, default=2.2)
    ap.add_argument("--gate-y", type=float, default=0.0)
    ap.add_argument("--gate-z", type=float, default=1.0)
    ap.add_argument("--gate-half-width", type=float, default=0.45)
    ap.add_argument("--gate-half-height", type=float, default=0.45)
    ap.add_argument("--initial-x", type=float, default=0.0)
    ap.add_argument("--initial-y", type=float, default=0.32)
    ap.add_argument("--initial-z", type=float, default=1.15)
    ap.add_argument("--speed", type=float, default=0.35, help="constant +world-x target speed")
    ap.add_argument("--lateral-speed", type=float, default=-0.02)
    ap.add_argument("--vertical-speed", type=float, default=0.0)
    ap.add_argument("--clutter", type=int, default=120,
                    help="number of visual-only feature boxes/squares to add")
    ap.add_argument("--clutter-seed", type=int, default=7)
    ap.add_argument("--wall-x", type=float, default=3.0,
                    help="x location of the textured background wall")
    ap.add_argument("--dynamic", action="store_true",
                    help="step PyBullet dynamics instead of the default fixed-attitude kinematic path")
    ap.add_argument("--noise-std", type=float, default=0.0,
                    help="Gaussian image noise standard deviation in uint8 gray levels")
    ap.add_argument("--blur-ksize", type=int, default=0,
                    help="odd Gaussian blur kernel size; 0 disables")
    ap.add_argument("--jpeg-quality", type=int, default=100,
                    help="JPEG round-trip quality before saving PNG; 100 disables")
    ap.add_argument("--gui", action="store_true")
    ap.add_argument("--overwrite", action="store_true")
    return ap.parse_args()


def main() -> int:
    args = parse_args()
    if not args.vision_root.exists():
        raise SystemExit(f"tinympc-vision checkout not found: {args.vision_root}")
    sys.path.insert(0, str(args.vision_root))

    try:
        import cv2
        from closed_loop.gym_pybullet_gate_env import GymPybulletGateEnv, GymPybulletGateEnvConfig
        from closed_loop.packets import GateGeometry
    except ImportError as exc:
        raise SystemExit(
            "sim capture requires opencv-python plus tinympc-vision's PyBullet dependencies"
        ) from exc

    if args.out.exists() and any(args.out.iterdir()) and not args.overwrite:
        raise SystemExit(f"{args.out} exists and is not empty; pass --overwrite")
    frames_dir = args.out / "frames"
    depth_dir = args.out / "depth"
    frames_dir.mkdir(parents=True, exist_ok=True)
    depth_dir.mkdir(parents=True, exist_ok=True)

    geom = GateGeometry(
        gate_x=float(args.gate_x),
        gate_y=float(args.gate_y),
        gate_z=float(args.gate_z),
        half_width=float(args.gate_half_width),
        half_height=float(args.gate_half_height),
    )
    cfg = GymPybulletGateEnvConfig(
        dt=float(args.dt),
        image_size=int(args.image_size),
        fov_deg=float(args.fov_deg),
        initial_x=float(args.initial_x),
        initial_y=float(args.initial_y),
        initial_z=float(args.initial_z),
        initial_vx=float(args.speed),
        initial_vy=float(args.lateral_speed),
        initial_vz=float(args.vertical_speed),
        ctrl_freq=max(1, int(round(1.0 / float(args.dt)))),
        gui=bool(args.gui),
        gate_collision=False,
        min_forward_speed=0.0,
        max_forward_speed=max(0.1, abs(float(args.speed)) + 0.5),
    )
    env = GymPybulletGateEnv(geometry=geom, config=cfg)
    steps = int(round(float(args.duration) / float(args.dt)))
    velocity = np.asarray([args.speed, args.lateral_speed, args.vertical_speed], dtype=np.float64)

    width = height = int(args.image_size)
    focal = 0.5 * float(height) / math.tan(0.5 * math.radians(float(args.fov_deg)))
    calibration = {
        "model": "pybullet_pinhole",
        "width": width,
        "height": height,
        "fov_deg": float(args.fov_deg),
        "fx": focal,
        "fy": focal,
        "cx": 0.5 * float(width - 1),
        "cy": 0.5 * float(height - 1),
        "distortion": [0.0, 0.0, 0.0, 0.0, 0.0],
        "camera_frame": "x_right_y_down_z_forward",
        "world_frame": "x_forward_y_left_z_up",
    }
    (args.out / "calibration.json").write_text(json.dumps(calibration, indent=2) + "\n")

    frame_rows: list[list[object]] = []
    telemetry_rows: list[list[object]] = []
    try:
        env.reset()
        if int(args.clutter) > 0:
            _add_feature_clutter(env, args)
        for i in range(steps):
            t = i * float(args.dt)
            if not args.dynamic:
                _set_kinematic_pose(env, args, t, velocity)
            image, depth_m = _render_camera_gray_depth(env, width, height)
            image = _degrade_image(cv2, image, args, frame_index=i)
            filename = f"frame_{i:06d}.png"
            depth_filename = f"frame_{i:06d}.npy"
            cv2.imwrite(str(frames_dir / filename), image)
            np.save(depth_dir / depth_filename, depth_m.astype(np.float32))
            projection = env.project_gate_outline((width, height))
            frame_rows.append(
                [
                    i,
                    f"{t:.9f}",
                    str(Path("frames") / filename),
                    str(Path("depth") / depth_filename),
                    f"{projection.get('distance_to_gate_m', math.nan):.9f}",
                ]
            )

            full_state = env.get_controller_state()
            quat = np.asarray(env.env.quat[0], dtype=np.float64)
            telemetry_rows.append(
                [
                    i,
                    f"{t:.9f}",
                    *[f"{v:.9f}" for v in full_state[0:3]],
                    *[f"{v:.9f}" for v in full_state[6:9]],
                    *[f"{v:.9f}" for v in full_state[9:12]],
                    *[f"{v:.9f}" for v in quat],
                    f"{projection.get('distance_to_gate_m', math.nan):.9f}",
                ]
            )
            if args.dynamic:
                env.step(velocity * 0.0)
                env._p.resetBaseVelocity(
                    env._drone_id,
                    linearVelocity=velocity.tolist(),
                    angularVelocity=[0.0, 0.0, 0.0],
                    physicsClientId=env.env.getPyBulletClient(),
                )
    finally:
        env.close()

    with (args.out / "frames.csv").open("w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["frame", "t", "filename", "depth_filename", "range_front_truth_m"])
        w.writerows(frame_rows)
    with (args.out / "telemetry.csv").open("w", newline="") as f:
        w = csv.writer(f)
        w.writerow(
            [
                "frame",
                "t",
                "x",
                "y",
                "z",
                "vx",
                "vy",
                "vz",
                "gyro_x",
                "gyro_y",
                "gyro_z",
                "qx",
                "qy",
                "qz",
                "qw",
                "range_front",
            ]
        )
        w.writerows(telemetry_rows)

    print(f"wrote {len(frame_rows)} frames to {args.out}")
    return 0


def _render_camera_gray_depth(env, width: int, height: int) -> tuple[np.ndarray, np.ndarray]:
    if env.env is None or env._p is None:
        raise RuntimeError("Call reset() before rendering.")
    p = env._p
    pos = np.asarray(env.env.pos[0], dtype=np.float64)
    quat = np.asarray(env.env.quat[0], dtype=np.float64)
    rot = np.asarray(p.getMatrixFromQuaternion(quat), dtype=np.float64).reshape(3, 3)
    forward = _normalized(rot @ np.asarray([1.0, 0.0, 0.0]), np.asarray([1.0, 0.0, 0.0]))
    up = _normalized(rot @ np.asarray([0.0, 0.0, 1.0]), np.asarray([0.0, 0.0, 1.0]))
    view = p.computeViewMatrix(
        cameraEyePosition=pos.tolist(),
        cameraTargetPosition=(pos + forward).tolist(),
        cameraUpVector=up.tolist(),
    )
    near = float(env.config.camera_near_m)
    far = float(env.config.camera_far_m)
    projection = p.computeProjectionMatrixFOV(
        fov=float(env.config.fov_deg),
        aspect=float(width) / float(height),
        nearVal=near,
        farVal=far,
    )
    _, _, rgba, depth_buffer, _ = p.getCameraImage(
        width=width,
        height=height,
        viewMatrix=view,
        projectionMatrix=projection,
        renderer=p.ER_TINY_RENDERER,
        lightDirection=[-0.4, 0.2, 1.0],
        lightColor=[1.0, 1.0, 1.0],
        physicsClientId=env.env.getPyBulletClient(),
    )
    rgba = np.asarray(rgba, dtype=np.uint8).reshape(height, width, 4)
    rgb = rgba[..., :3].astype(np.float32)
    gray = np.clip(0.299 * rgb[..., 0] + 0.587 * rgb[..., 1] + 0.114 * rgb[..., 2], 0, 255).astype(np.uint8)
    depth_buffer = np.asarray(depth_buffer, dtype=np.float32).reshape(height, width)
    depth_m = far * near / (far - (far - near) * depth_buffer)
    return gray, depth_m


def _degrade_image(cv2, image: np.ndarray, args: argparse.Namespace, frame_index: int) -> np.ndarray:
    out = image.copy()
    ksize = int(args.blur_ksize)
    if ksize > 0:
        if ksize % 2 == 0:
            ksize += 1
        out = cv2.GaussianBlur(out, (ksize, ksize), 0)
    if float(args.noise_std) > 0.0:
        rng = np.random.default_rng(int(args.clutter_seed) + 1000003 * int(frame_index))
        noise = rng.normal(0.0, float(args.noise_std), size=out.shape)
        out = np.clip(out.astype(np.float32) + noise, 0, 255).astype(np.uint8)
    quality = int(args.jpeg_quality)
    if quality < 100:
        quality = max(1, min(100, quality))
        ok, encoded = cv2.imencode(".jpg", out, [int(cv2.IMWRITE_JPEG_QUALITY), quality])
        if ok:
            out = cv2.imdecode(encoded, cv2.IMREAD_GRAYSCALE)
    return out


def _normalized(vector: np.ndarray, fallback: np.ndarray) -> np.ndarray:
    norm = float(np.linalg.norm(vector))
    if norm <= 1e-9 or not math.isfinite(norm):
        return fallback.copy()
    return np.asarray(vector, dtype=np.float64) / norm


def _set_kinematic_pose(env, args: argparse.Namespace, t: float, velocity: np.ndarray) -> None:
    if env.env is None or env._p is None or env._drone_id is None:
        raise RuntimeError("Call reset before setting pose.")
    pos = np.asarray(
        [
            float(args.initial_x) + float(velocity[0]) * t,
            float(args.initial_y) + float(velocity[1]) * t,
            float(args.initial_z) + float(velocity[2]) * t,
        ],
        dtype=np.float64,
    )
    client = env.env.getPyBulletClient()
    env._p.resetBasePositionAndOrientation(
        env._drone_id,
        pos.tolist(),
        [0.0, 0.0, 0.0, 1.0],
        physicsClientId=client,
    )
    env._p.resetBaseVelocity(
        env._drone_id,
        linearVelocity=velocity.astype(float).tolist(),
        angularVelocity=[0.0, 0.0, 0.0],
        physicsClientId=client,
    )
    env.env._updateAndStoreKinematicInformation()
    env._sync_state()


def _add_feature_clutter(env, args: argparse.Namespace) -> None:
    """Add visual-only high-contrast objects for sparse optical flow.

    The gate environment is visually sparse. These boxes are intentionally
    collision-free so they enrich the camera stream without changing dynamics.
    """
    if env.env is None or env._p is None:
        raise RuntimeError("Call reset before adding clutter.")
    p = env._p
    client = env.env.getPyBulletClient()
    rng = np.random.default_rng(int(args.clutter_seed))
    n = int(args.clutter)
    colors = [
        [0.02, 0.02, 0.02, 1.0],
        [0.95, 0.95, 0.95, 1.0],
        [0.05, 0.35, 0.95, 1.0],
        [0.95, 0.8, 0.05, 1.0],
        [0.05, 0.75, 0.25, 1.0],
    ]

    # A dotted wall behind the gate gives many long-lived corners and a known
    # far-depth layer.
    wall_x = float(args.wall_x)
    for _ in range(n // 2):
        size = float(rng.uniform(0.025, 0.075))
        y = float(rng.uniform(-1.35, 1.35))
        z = float(rng.uniform(0.15, 1.9))
        color = colors[int(rng.integers(0, len(colors)))]
        visual = p.createVisualShape(
            p.GEOM_BOX,
            halfExtents=[0.008, size, size],
            rgbaColor=color,
            physicsClientId=client,
        )
        p.createMultiBody(
            baseMass=0.0,
            baseCollisionShapeIndex=-1,
            baseVisualShapeIndex=visual,
            basePosition=[wall_x, y, z],
            physicsClientId=client,
        )

    # Free-floating visual boxes create nearer depth variation. Keep a loose
    # center corridor, but allow some objects in view like real obstacles.
    for _ in range(n - n // 2):
        sx = float(rng.uniform(0.025, 0.07))
        sy = float(rng.uniform(0.025, 0.09))
        sz = float(rng.uniform(0.025, 0.09))
        x = float(rng.uniform(0.75, max(0.8, float(args.gate_x) + 0.15)))
        y = float(rng.uniform(-1.15, 1.15))
        z = float(rng.uniform(0.25, 1.75))
        color = colors[int(rng.integers(0, len(colors)))]
        visual = p.createVisualShape(
            p.GEOM_BOX,
            halfExtents=[sx, sy, sz],
            rgbaColor=color,
            physicsClientId=client,
        )
        p.createMultiBody(
            baseMass=0.0,
            baseCollisionShapeIndex=-1,
            baseVisualShapeIndex=visual,
            basePosition=[x, y, z],
            physicsClientId=client,
        )


if __name__ == "__main__":
    raise SystemExit(main())
