#!/usr/bin/env python3
"""Generate deterministic flight trajectory manifests for saved course layouts.

This script intentionally has no Isaac Sim dependency.  It consumes a layout's
``scene_geometry.json`` and writes motion manifests that the renderer can replay.
"""

import argparse
import json
import math
from pathlib import Path

import numpy as np

from gap8_perception.scene_distribution_v2 import load_scene_distribution


DISTRIBUTION_PATH = Path(__file__).resolve().parents[1] / "gap8_perception/configs/scene_distribution_v2.json"
SCENE_DISTRIBUTION, SCENE_DISTRIBUTION_SHA256 = load_scene_distribution(DISTRIBUTION_PATH)
TRAJECTORY_CONTRACT = SCENE_DISTRIBUTION["mission_and_trajectory"]
TRAJECTORY_TYPES = tuple(TRAJECTORY_CONTRACT["trajectory_type_cycle"])
WAYPOINT_REACHED_RADIUS_M = float(TRAJECTORY_CONTRACT["waypoint_reached_radius_m"])
DRONE_COLLISION_RADIUS_M = 0.10


def vector(values):
    return [float(value) for value in values]


def distance_to_route_segments(point_xy, route_xy):
    point = np.asarray(point_xy, dtype=float)
    route = np.asarray(route_xy, dtype=float)
    distances = []
    for start, finish in zip(route[:-1], route[1:]):
        segment = finish - start
        denominator = float(segment @ segment)
        fraction = 0.0 if denominator <= 1.0e-12 else float(
            np.clip((point - start) @ segment / denominator, 0.0, 1.0)
        )
        distances.append(float(np.linalg.norm(point - (start + fraction * segment))))
    return min(distances, default=float(np.linalg.norm(point - route[0])))


def point_box_clearance(point, obstacle):
    """Signed distance to the axis-aligned obstacle bounds."""
    offset = (
        np.abs(np.asarray(point, dtype=float) - np.asarray(obstacle["center_m"], dtype=float))
        - np.asarray(obstacle["size_m"], dtype=float) / 2.0
    )
    return float(np.linalg.norm(np.maximum(offset, 0.0)) + min(float(offset.max()), 0.0))


def path_clears_other_obstacles(path, obstacles, ignored_index, margin_m=0.15):
    """Conservatively reject candidates with an earlier/intervening contact."""
    samples = []
    for start, finish in zip(path[:-1], path[1:]):
        distance = float(np.linalg.norm(np.asarray(finish) - np.asarray(start)))
        count = max(2, int(math.ceil(distance / 0.025)) + 1)
        samples.extend(
            np.asarray(start) * (1.0 - fraction) + np.asarray(finish) * fraction
            for fraction in np.linspace(0.0, 1.0, count)
        )
    required = DRONE_COLLISION_RADIUS_M + margin_m
    return all(
        point_box_clearance(point, obstacle) > required
        for index, obstacle in enumerate(obstacles)
        if index != ignored_index
        for point in samples
    )


def trajectory_seed(master_seed, layout_index, trajectory_index, generation_attempt=0):
    sequence = np.random.SeedSequence([
        master_seed, layout_index, trajectory_index, generation_attempt,
    ])
    return int(sequence.generate_state(1, dtype=np.uint64)[0])


def segment_duration(start, end, nominal_speed, max_acceleration=None):
    if max_acceleration is None:
        max_acceleration = float(TRAJECTORY_CONTRACT["maximum_reference_acceleration_mps2"])
    distance = float(np.linalg.norm(end - start))
    return max(distance / nominal_speed, math.sqrt(6.0 * distance / max_acceleration), 0.25)


def sample_path(
    waypoints,
    rng,
    frame_count,
    nominal_speed,
    frame_rate_hz=float(TRAJECTORY_CONTRACT["nominal_capture_rate_hz"]),
    frame_jitter_fraction=float(TRAJECTORY_CONTRACT["capture_interval_jitter_fraction"]),
    frame_skip_probability=float(TRAJECTORY_CONTRACT["single_frame_skip_probability"]),
    event_waypoint_index=None,
    event_time_s=None,
):
    """Sample an event-aligned C1 path at a nearly fixed capture rate."""
    points = [np.asarray(point, dtype=float) for point in waypoints]
    durations = [
        segment_duration(start, end, nominal_speed)
        for start, end in zip(points[:-1], points[1:])
    ]
    total_time = float(sum(durations))
    nominal_dt = 1.0 / frame_rate_hz
    dts = []
    elapsed = 0.0
    for _ in range(frame_count - 1):
        dt = nominal_dt * (1.0 + rng.uniform(-frame_jitter_fraction, frame_jitter_fraction))
        if rng.random() < frame_skip_probability:
            dt += nominal_dt
        if elapsed + dt > total_time:
            break
        dts.append(dt)
        elapsed += dt
    dts = np.asarray(dts, dtype=float)
    if event_time_s is not None:
        event_time = float(event_time_s)
        window_start = float(np.clip(event_time - 0.70 * elapsed, 0.0, total_time - elapsed))
    elif event_waypoint_index is None:
        window_start = total_time - elapsed
    else:
        event_time = float(sum(durations[:event_waypoint_index]))
        # Preserve both approach and aftermath while putting most frames before
        # the closest-approach event, where looming and TTC are most useful.
        window_start = float(np.clip(event_time - 0.70 * elapsed, 0.0, total_time - elapsed))
    times = window_start + np.concatenate(([0.0], np.cumsum(dts)))
    positions = []
    segment_index = 0
    segment_start_time = 0.0
    for time_s in times:
        while (
            segment_index < len(durations) - 1
            and time_s > segment_start_time + durations[segment_index]
        ):
            segment_start_time += durations[segment_index]
            segment_index += 1
        duration = durations[segment_index]
        u = float(np.clip((time_s - segment_start_time) / duration, 0.0, 1.0))
        smooth_u = u * u * (3.0 - 2.0 * u)
        positions.append(points[segment_index] * (1.0 - smooth_u) + points[segment_index + 1] * smooth_u)
    positions = np.asarray(positions)
    velocities = np.gradient(positions, times, axis=0, edge_order=1)
    accelerations = np.gradient(velocities, times, axis=0, edge_order=1)
    return times, dts, positions, velocities, accelerations


def collision_contact_time(waypoints, obstacle, nominal_speed):
    """Time of first geometric drone-envelope contact on the target segment."""
    start, target = np.asarray(waypoints[0]), np.asarray(waypoints[1])
    duration = segment_duration(start, target, nominal_speed)
    for u in np.linspace(0.0, 1.0, 10001):
        smooth_u = u * u * (3.0 - 2.0 * u)
        point = start * (1.0 - smooth_u) + target * smooth_u
        if point_box_clearance(point, obstacle) <= DRONE_COLLISION_RADIUS_M:
            return float(u * duration)
    raise RuntimeError("collision reference does not intersect its target obstacle")


def path_for_type(layout, trajectory_type, rng):
    reference = [np.asarray(point, dtype=float) for point in layout["route_waypoints_m"]]
    start, finish = reference[0].copy(), reference[-1].copy()
    obstacles = layout["obstacles"]
    if trajectory_type == "safe" or not obstacles:
        path = [point.copy() for point in reference]
        if len(path) > 2:
            for point in path[1:-1]:
                point[1] += float(rng.uniform(*TRAJECTORY_CONTRACT["safe_route_lateral_jitter_m"]))
        return path, None

    obstacle_distances = np.asarray([
        np.linalg.norm(np.asarray(obstacle["center_m"], dtype=float)[:2] - start[:2])
        for obstacle in obstacles
    ])
    altitude_min, altitude_max = TRAJECTORY_CONTRACT["flight_altitude_m"]
    flight_z = float(np.clip(start[2] + rng.uniform(-0.08, 0.08), altitude_min, altitude_max))
    eligible = list(np.flatnonzero(obstacle_distances >= 1.50))
    rng.shuffle(eligible)
    local_near_miss_fallback = None
    for obstacle_index in eligible:
        obstacle_index = int(obstacle_index)
        obstacle = obstacles[obstacle_index]
        center = np.asarray(obstacle["center_m"], dtype=float)
        size = np.asarray(obstacle["size_m"], dtype=float)
        if trajectory_type == "collision":
            target = np.asarray((
                center[0], center[1],
                np.clip(flight_z, center[2] - size[2] / 3, center[2] + size[2] / 3),
            ))
            approach = target - start
            approach /= max(float(np.linalg.norm(approach)), 1.0e-9)
            beyond = target + approach * (0.5 * float(np.linalg.norm(size)) + 0.35)
            path = [start, target, beyond]
            if path_clears_other_obstacles(path[:2], obstacles, obstacle_index):
                return path, obstacle_index
            continue
        for side in rng.permutation((-1.0, 1.0)):
            clearance = float(rng.uniform(*TRAJECTORY_CONTRACT["near_miss_clearance_m"]))
            pass_point = np.asarray((
                center[0],
                center[1] + float(side) * (size[1] / 2 + DRONE_COLLISION_RADIUS_M + clearance),
                flight_z,
            ))
            mission_path = [start, pass_point, finish]
            if path_clears_other_obstacles(mission_path, obstacles, obstacle_index):
                return mission_path, obstacle_index
            # A near-miss example may also represent one local candidate horizon
            # around closest approach. Preserve the full mission path whenever
            # feasible, but retain the first clear local candidate as a fallback
            # for dense rooms where every post-event route to the distant final
            # waypoint crosses another obstacle.
            local_path = [start, pass_point]
            if (
                local_near_miss_fallback is None
                and path_clears_other_obstacles(local_path, obstacles, obstacle_index)
            ):
                local_near_miss_fallback = (local_path, obstacle_index)
    if local_near_miss_fallback is not None:
        return local_near_miss_fallback
    raise RuntimeError(
        f"no {trajectory_type} target has a mechanically clear approach in layout "
        f"{layout.get('layout_id', '<unknown>')}"
    )


def mission_waypoints(layout):
    """Return ordered navigation goals, marking gate centers as visual goals."""
    route = [np.asarray(point, dtype=float) for point in layout["route_waypoints_m"]]
    gates = [
        np.asarray(gate.get("center_m", gate.get("center")), dtype=float)
        for gate in layout.get("gates", [])
    ]
    waypoints = []
    for route_index, point in enumerate(route[1:]):
        gate_index = next(
            (index for index, center in enumerate(gates) if np.linalg.norm(point - center) <= 1.0e-4),
            None,
        )
        waypoints.append({
            "position_m": vector(point),
            "kind": "gate" if gate_index is not None else "mission",
            "source": "visual_gate" if gate_index is not None else "provided_waypoint",
            "gate_index": gate_index,
            "route_index": route_index + 1,
        })
    return waypoints


def tangent_yaws(velocities, minimum_horizontal_speed=0.02):
    """Continuous path-tangent yaw, with stationary samples holding heading."""
    velocities = np.asarray(velocities, dtype=float)
    speed = np.linalg.norm(velocities[:, :2], axis=1)
    valid = speed > minimum_horizontal_speed
    if not valid.any():
        return np.zeros(len(velocities), dtype=float)
    raw = np.full(len(velocities), np.nan, dtype=float)
    valid_indices = np.flatnonzero(valid)
    raw[valid] = np.unwrap(np.arctan2(velocities[valid, 1], velocities[valid, 0]))
    raw[:valid_indices[0]] = raw[valid_indices[0]]
    raw[valid_indices[-1] + 1:] = raw[valid_indices[-1]]
    for start, end in zip(valid_indices[:-1], valid_indices[1:]):
        raw[start:end + 1] = np.linspace(raw[start], raw[end], end - start + 1)
    return raw


def active_waypoint(position, waypoints):
    for waypoint in waypoints:
        if np.linalg.norm(np.asarray(waypoint["position_m"]) - position) > WAYPOINT_REACHED_RADIUS_M:
            return waypoint
    return waypoints[-1]


def generate_trajectory_manifest(
    layout,
    layout_index,
    trajectory_index,
    master_seed,
    frame_count,
    trajectory_type=None,
    frame_rate_hz=30.0,
    frame_jitter_fraction=0.05,
    frame_skip_probability=0.02,
    generation_attempt=0,
):
    seed = trajectory_seed(master_seed, layout_index, trajectory_index, generation_attempt)
    rng = np.random.default_rng(seed)
    kind = trajectory_type or TRAJECTORY_TYPES[trajectory_index % len(TRAJECTORY_TYPES)]
    nominal_speed = float(rng.uniform(*TRAJECTORY_CONTRACT["nominal_speed_mps"]))
    waypoints, target_obstacle = path_for_type(layout, kind, rng)
    event_waypoint_index = 1 if kind in {"collision", "near_miss"} else None
    event_time_s = None
    if kind == "collision" and target_obstacle is not None:
        event_time_s = collision_contact_time(
            waypoints, layout["obstacles"][target_obstacle], nominal_speed
        )
    times, dts, positions, velocities, accelerations = sample_path(
        waypoints,
        rng,
        frame_count,
        nominal_speed,
        frame_rate_hz,
        frame_jitter_fraction,
        frame_skip_probability,
        event_waypoint_index,
        event_time_s,
    )
    navigation_waypoints = mission_waypoints(layout)
    yaws = tangent_yaws(velocities)
    states = []
    previous_yaw = None
    for index, (time_s, position, velocity, acceleration, yaw) in enumerate(
        zip(times, positions, velocities, accelerations, yaws)
    ):
        dt_s = 0.0 if index == 0 else float(dts[index - 1])
        yaw_rate = 0.0 if previous_yaw is None else float(math.atan2(math.sin(yaw - previous_yaw), math.cos(yaw - previous_yaw)) / dt_s)
        previous_yaw = yaw
        waypoint = active_waypoint(position, navigation_waypoints)
        states.append({
            "frame_index": index,
            "time_s": float(time_s),
            "delta_t_s": dt_s,
            "position_m": vector(position),
            "velocity_mps": vector(velocity),
            "acceleration_mps2": vector(acceleration),
            "attitude_rpy_rad": [0.0, 0.0, float(yaw)],
            "angular_velocity_rps": [0.0, 0.0, yaw_rate],
            "active_waypoint_m": waypoint["position_m"],
            "active_waypoint_kind": waypoint["kind"],
            "active_waypoint_source": waypoint["source"],
        })
    return {
        "layout_id": layout["layout_id"],
        "split": layout["split"],
        "trajectory_id": f"trajectory_{trajectory_index:05d}",
        "trajectory_index": trajectory_index,
        "trajectory_type": kind,
        "deterministic_seed": seed,
        **({"generation_attempt": int(generation_attempt)} if generation_attempt else {}),
        "nominal_speed_mps": nominal_speed,
        "capture_timing": {
            "nominal_frame_rate_hz": frame_rate_hz,
            "jitter_fraction": frame_jitter_fraction,
            "frame_skip_probability": frame_skip_probability,
        },
        "target_obstacle_index": target_obstacle,
        "mission_goal_m": navigation_waypoints[-1]["position_m"],
        "navigation_waypoints": navigation_waypoints,
        "planning_context": {
            "mode": "receding_horizon_candidate",
            "horizon_frame": "current_drone_yaw_local",
            "camera_heading_policy": "horizontal_velocity_tangent",
            "waypoint_reached_radius_m": WAYPOINT_REACHED_RADIUS_M,
            "gate_policy": "a confidently detected traversable gate may replace the active waypoint",
            "note": "collision and near-miss paths are candidate horizons retained to supervise risk, not globally safe plans",
        },
        "waypoints_m": [vector(point) for point in waypoints],
        "states": states,
    }


def write_trajectories(
    layout_path,
    count,
    master_seed,
    frame_count,
    frame_rate_hz=30.0,
    frame_jitter_fraction=0.05,
    frame_skip_probability=0.02,
):
    layout = json.loads(layout_path.read_text())
    trajectory_root = layout_path.parent / "trajectories"
    trajectory_root.mkdir(parents=True, exist_ok=True)
    manifests = []
    for trajectory_index in range(count):
        manifest = generate_trajectory_manifest(
            layout,
            int(layout["layout_index"]),
            trajectory_index,
            master_seed,
            frame_count,
            frame_rate_hz=frame_rate_hz,
            frame_jitter_fraction=frame_jitter_fraction,
            frame_skip_probability=frame_skip_probability,
        )
        output_dir = trajectory_root / manifest["trajectory_id"]
        output_dir.mkdir(parents=True, exist_ok=True)
        output_path = output_dir / "trajectory_manifest.json"
        output_path.write_text(json.dumps(manifest, indent=2) + "\n")
        manifests.append(output_path)
    return manifests


def parse_args():
    parser = argparse.ArgumentParser(description="Generate flight manifests for one saved layout.")
    parser.add_argument("--layout", type=Path, required=True, help="path to scene_geometry.json")
    parser.add_argument("--trajectories", type=int, default=6)
    parser.add_argument("--frames", type=int, default=48, help="maximum captured frames per trajectory")
    parser.add_argument("--frame-rate-hz", type=float, default=30.0)
    parser.add_argument("--frame-jitter-fraction", type=float, default=0.05)
    parser.add_argument("--frame-skip-probability", type=float, default=0.02)
    parser.add_argument("--seed", type=int, default=42, help="independent trajectory master seed")
    return parser.parse_args()


def main():
    args = parse_args()
    if args.trajectories < 1 or args.frames < 2:
        raise ValueError("--trajectories must be positive and --frames must be at least two")
    if args.frame_rate_hz <= 0 or not 0 <= args.frame_jitter_fraction < 1:
        raise ValueError("frame rate must be positive and jitter must be in [0, 1)")
    if not 0 <= args.frame_skip_probability <= 1:
        raise ValueError("frame skip probability must be in [0, 1]")
    for path in write_trajectories(
        args.layout.resolve(),
        args.trajectories,
        args.seed,
        args.frames,
        args.frame_rate_hz,
        args.frame_jitter_fraction,
        args.frame_skip_probability,
    ):
        print(path)


if __name__ == "__main__":
    main()
