#!/usr/bin/env python3
"""Generate reproducible, calibrated Isaac Sim flight-sequence episodes."""

import argparse
import json
import math
import os
import sys
import traceback
from pathlib import Path

import cv2
import numpy as np

from generate_flight_trajectories import distance_to_route_segments, generate_trajectory_manifest
from crazyflie_rollout import execute_trajectory
from gap8_perception.scene_distribution_v2 import (
    load_scene_distribution, sample_room as sample_room_from_distribution,
)


DISTRIBUTION_PATH = Path(__file__).resolve().parents[1] / "gap8_perception/configs/scene_distribution_v2.json"
SCENE_DISTRIBUTION, SCENE_DISTRIBUTION_SHA256 = load_scene_distribution(DISTRIBUTION_PATH)
TRAJECTORY_CONTRACT = SCENE_DISTRIBUTION["mission_and_trajectory"]
SCENARIOS = tuple(SCENE_DISTRIBUTION["scenario_schedule"]["order"])
CLASS_NAMES = ["background", "track", "boundary", "obstacle", "gate"]
OVERSCAN_RESOLUTION = 512
TEXTURE_DIR = Path(__file__).resolve().parents[1] / "assets" / "course"
TEXTURE_EXTENSIONS = {".png", ".jpg", ".jpeg", ".tif", ".tiff"}
TEXTURE_PROBABILITY = float(SCENE_DISTRIBUTION["appearance"]["texture_probability_per_object"])
DRONE_COLLISION_RADIUS_M = 0.10


def parse_args():
    parser = argparse.ArgumentParser(
        description="Generate procedurally varied, temporally consistent flight episodes."
    )
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--layouts", "--episodes", dest="layouts", type=int, default=6)
    parser.add_argument("--layout-start-index", type=int, default=0)
    parser.add_argument("--trajectories-per-layout", type=int, default=6)
    parser.add_argument("--scene-only", action="store_true", help="write randomized layouts without rendering reference trajectories")
    parser.add_argument("--trajectory-start-index", type=int, default=0)
    parser.add_argument("--frames", type=int, default=48, help="maximum captured frames per trajectory")
    parser.add_argument("--frame-rate-hz", type=float, default=30.0)
    parser.add_argument("--frame-jitter-fraction", type=float, default=0.05)
    parser.add_argument("--frame-skip-probability", type=float, default=0.02)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--trajectory-seed", type=int, default=4242)
    parser.add_argument("--train-fraction", type=float, default=0.8)
    parser.add_argument("--validation-fraction", type=float, default=0.1)
    parser.add_argument("--scenario", choices=("mixed",) + SCENARIOS, default="mixed")
    parser.add_argument("--width", type=int, default=160)
    parser.add_argument("--height", type=int, default=160)
    parser.add_argument("--rt-subframes", type=int, default=1)
    parser.add_argument("--near-clip-m", type=float, default=0.02)
    parser.add_argument(
        "--camera-calibration",
        type=Path,
        default=Path(__file__).resolve().parents[1]
        / "gap8_perception/configs/hm01b0_calibration.json",
    )
    parser.add_argument(
        "--save-stage",
        type=Path,
        help="directory in which to export one editable USD stage per episode",
    )
    return parser.parse_args()


args = parse_args()
args.output_dir = args.output_dir.expanduser().resolve()
args.camera_calibration = args.camera_calibration.expanduser().resolve()
if args.save_stage is not None:
    args.save_stage = args.save_stage.expanduser().resolve()
if args.layouts < 1 or args.trajectories_per_layout < 0 or (not args.scene_only and args.trajectories_per_layout < 1) or args.frames < 2:
    raise ValueError("layouts must be positive, trajectories nonnegative (positive unless --scene-only), and --frames at least two")
if args.frame_rate_hz <= 0 or not 0 <= args.frame_jitter_fraction < 1:
    raise ValueError("frame rate must be positive and jitter must be in [0, 1)")
if not 0 <= args.frame_skip_probability <= 1:
    raise ValueError("frame skip probability must be in [0, 1]")
if args.train_fraction < 0 or args.validation_fraction < 0 or args.train_fraction + args.validation_fraction > 1:
    raise ValueError("split fractions must be nonnegative and sum to at most one")
camera_calibration = json.loads(args.camera_calibration.read_text())
sys.argv = [sys.argv[0]]
os.environ.setdefault("OMNI_KIT_ACCEPT_EULA", "YES")

from isaacsim import SimulationApp

app = SimulationApp(
    launch_config={
        "headless": True,
        "width": args.width,
        "height": args.height,
        "renderer": "RayTracedLighting",
        # Isaac otherwise sizes its worker pools from the whole host rather
        # than the CPUs granted by Slurm. This overloaded tiger and prevented
        # Replicator from scheduling its first rendered frame.
        "limit_cpu_threads": int(os.environ.get("SLURM_CPUS_PER_TASK", "8")),
    }
)

import omni.replicator.core as rep
import omni.usd
from pxr import Gf, Sdf, UsdGeom, UsdLux, Vt


def layout_seeds(master_seed, layout_index):
    """Create independent, replayable streams for layout geometry and appearance."""
    names = ("scene_geometry", "appearance")
    children = np.random.SeedSequence([master_seed, layout_index]).spawn(len(names))
    return {
        name: int(child.generate_state(1, dtype=np.uint64)[0])
        for name, child in zip(names, children)
    }


def rngs_for_layout(seeds):
    return {name: np.random.default_rng(value) for name, value in seeds.items()}


def split_for_layout(master_seed, layout_index):
    """Assign a split once per layout so trajectories cannot leak across splits."""
    value = float(np.random.default_rng(np.random.SeedSequence([master_seed, layout_index, 991])).random())
    if value < args.train_fraction:
        return "train"
    if value < args.train_fraction + args.validation_fraction:
        return "validation"
    return "test"


def stratified_split_for_scenario_occurrence(occurrence):
    """Guarantee every repeated scenario reaches train, validation, and test."""
    if not (
        np.isclose(args.train_fraction, 0.70)
        and np.isclose(args.validation_fraction, 0.15)
    ):
        raise ValueError(
            "balanced mixed-scenario generation currently requires 0.70/0.15/0.15 splits"
        )
    # Five train, one validation, and one test layout per seven occurrences.
    # Validation/test appear early so six-occurrence scenario families still
    # have both held-out partitions in an 80-layout dataset.
    pattern = ("train", "train", "validation", "train", "test", "train", "train")
    return pattern[int(occurrence) % len(pattern)]


def vector(values):
    return [float(value) for value in values]


def create_material(
    name, color, roughness, specular_level=0.18, mdl="OmniPBR.mdl", texture=None
):
    material_args = {
        "mdl": mdl,
        "diffuse_color_constant": tuple(float(value) for value in color),
        "reflection_roughness_constant": float(roughness),
        "specular_level": float(specular_level),
        "name": name,
        "parent": "/World",
    }
    if texture is not None:
        material_args["diffuse_texture"] = str(texture)
    return rep.functional.create.material(**material_args)


def create_primitive(
    name, center, size, semantic_class, material, primitive="box", parent="/World/Track"
):
    """Create one labeled primitive with a common obstacle interface.

    ``size`` is always a full XYZ extent. For spheres, cylinders, and cones,
    X/Y define the diameter and Z defines the height. This keeps collision and
    metadata generation independent of the visual primitive implementation.
    """
    creators = {
        "box": rep.functional.create.cube,
        "sphere": rep.functional.create.sphere,
        "cylinder": rep.functional.create.cylinder,
        "cone": rep.functional.create.cone,
    }
    if primitive not in creators:
        raise ValueError(f"unsupported primitive: {primitive}")
    prim = creators[primitive](
        name=name,
        position=tuple(float(value) for value in center),
        scale=tuple(float(value) for value in size),
        parent=parent,
        material=material,
    )
    rep.functional.modify.semantics(prim, {"class": semantic_class}, mode="add")
    return prim


def create_wall_plane(name, center, size, material, rotation):
    """Create a vertical boundary plane sharing the obstacle material."""
    prim = rep.functional.create.plane(
        name=name,
        position=tuple(float(value) for value in center),
        scale=tuple(float(value) for value in size),
        rotation=tuple(float(value) for value in rotation),
        parent="/World/Track",
        material=material,
    )
    rep.functional.modify.semantics(prim, {"class": "boundary"}, mode="add")
    return prim


def validate_mechanical_feasibility(obstacles):
    """Reject floating obstacles; elevated parts must rest on another part."""
    tolerance = 1.0e-5
    for index, obstacle in enumerate(obstacles):
        center = np.asarray(obstacle["center_m"], dtype=float)
        size = np.asarray(obstacle["size_m"], dtype=float)
        bottom = center[2] - size[2] / 2.0
        if bottom <= tolerance:
            continue
        supported = False
        for support_index, support in enumerate(obstacles):
            if index == support_index:
                continue
            support_center = np.asarray(support["center_m"], dtype=float)
            support_size = np.asarray(support["size_m"], dtype=float)
            support_top = support_center[2] + support_size[2] / 2.0
            horizontal_overlap = (
                abs(center[0] - support_center[0]) <= (size[0] + support_size[0]) / 2.0 + tolerance
                and abs(center[1] - support_center[1]) <= (size[1] + support_size[1]) / 2.0 + tolerance
            )
            if horizontal_overlap and support_top >= bottom - tolerance:
                supported = True
                break
        if not supported:
            raise ValueError(
                f"obstacle {index} floats: bottom={bottom:.4f} m; "
                "obstacles must touch the floor or rest on another obstacle"
            )


def create_obstacle(name, spec, material, parent="/World/Track"):
    """Instantiate an obstacle spec; arrangements are groups of these specs."""
    return create_primitive(
        name,
        spec["center_m"],
        spec["size_m"],
        "obstacle",
        material,
        primitive=spec.get("primitive", "box"),
        parent=parent,
    )


def gate_geometry(center, yaw, width=None, height=None, depth=None):
    """Return the exact four collision bars used to render a gate."""
    contract = SCENE_DISTRIBUTION["gate"]
    width = float(contract["outer_width_m"] if width is None else width)
    height = float(contract["outer_height_m"] if height is None else height)
    depth = float(contract["depth_m"] if depth is None else depth)
    angle = float(yaw)
    lateral = np.asarray((-math.sin(angle), math.cos(angle), 0.0))
    center = np.asarray(center, dtype=float)
    rail = float(contract["rail_m"])
    parts = []
    definitions = (
        ("left", center - lateral * width / 2, (depth, rail, height)),
        ("right", center + lateral * width / 2, (depth, rail, height)),
        ("top", center + np.asarray((0.0, 0.0, height / 2)), (depth, width + rail, rail)),
        ("bottom", center - np.asarray((0.0, 0.0, height / 2)), (depth, width + rail, rail)),
    )
    for part_name, position, size in definitions:
        parts.append({
            "object_id": f"gate_{part_name}",
            "center_m": vector(position),
            "size_m": vector(size),
            "yaw_rad": angle,
        })
    return {
        "center_m": vector(center),
        "yaw_rad": angle,
        "opening_m": [width - rail, height - rail],
        "parts": parts,
    }


def load_obj_mesh(path):
    """Read the small indexed OBJ contract used by the NewBee gate rails."""
    vertices, texcoords, faces = [], [], []
    for raw_line in Path(path).read_text().splitlines():
        fields = raw_line.split()
        if not fields or fields[0].startswith("#"):
            continue
        if fields[0] == "v":
            vertices.append(tuple(float(value) for value in fields[1:4]))
        elif fields[0] == "vt":
            texcoords.append(tuple(float(value) for value in fields[1:3]))
        elif fields[0] == "f":
            face = []
            for token in fields[1:]:
                indices = token.split("/")
                if len(indices) < 2 or not indices[1]:
                    raise ValueError(f"OBJ face has no texture coordinate: {path}")
                face.append((int(indices[0]) - 1, int(indices[1]) - 1))
            faces.append(face)
    if not vertices or not texcoords or not faces:
        raise ValueError(f"empty gate OBJ: {path}")
    return np.asarray(vertices, dtype=float), np.asarray(texcoords, dtype=float), faces


def create_textured_obj_gate(name, spec, material):
    """Render the authoritative CrazySim NewBee mesh, including its atlas UVs."""
    asset = spec["visual_asset"]
    scale = np.asarray(asset["mesh_scale"], dtype=float)
    center = np.asarray(spec["center_m"], dtype=float)
    yaw = float(spec["yaw_rad"])
    cosine, sine = math.cos(yaw), math.sin(yaw)
    rotation = np.asarray(((cosine, -sine, 0.0), (sine, cosine, 0.0), (0.0, 0.0, 1.0)))
    stage = omni.usd.get_context().get_stage()
    for index, source in enumerate(asset["mesh_parts"]):
        source = Path(source)
        vertices, texcoords, faces = load_obj_mesh(source)
        points = (rotation @ (vertices * scale).T).T + center
        counts, vertex_indices, face_uvs = [], [], []
        for face in faces:
            counts.append(len(face))
            vertex_indices.extend(vertex_index for vertex_index, _ in face)
            face_uvs.extend(tuple(texcoords[texture_index]) for _, texture_index in face)
        mesh = UsdGeom.Mesh.Define(stage, f"/World/Track/{name}_{index}_{source.stem}")
        mesh.CreatePointsAttr(Vt.Vec3fArray([Gf.Vec3f(*point) for point in points]))
        mesh.CreateFaceVertexCountsAttr(Vt.IntArray(counts))
        mesh.CreateFaceVertexIndicesAttr(Vt.IntArray(vertex_indices))
        mesh.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
        st = UsdGeom.PrimvarsAPI(mesh).CreatePrimvar(
            "st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.faceVarying
        )
        st.Set(Vt.Vec2fArray([Gf.Vec2f(*uv) for uv in face_uvs]))
        rep.functional.modify.material(mesh.GetPrim(), material)
        rep.functional.modify.semantics(mesh.GetPrim(), {"class": "gate"}, mode="add")


def create_gate(name, spec, material):
    """Render either an authoritative visual asset or manifest collision bars."""
    if "visual_asset" in spec:
        create_textured_obj_gate(name, spec, material)
        return
    parent = "/World/Track"
    rep.functional.create.xform(name=name, parent=parent)
    for part in spec["parts"]:
        suffix = str(part["object_id"]).rsplit("_", 1)[-1]
        prim = create_primitive(
            f"{name}_{suffix}", part["center_m"], part["size_m"], "gate", material,
            primitive="box", parent=parent,
        )
        UsdGeom.Xformable(prim).AddRotateZOp().Set(math.degrees(float(part["yaw_rad"])))


def choose_texture(rng):
    """Choose one course texture or deliberately leave the material untextured."""
    if rng.random() >= TEXTURE_PROBABILITY:
        return None
    candidates = sorted(
        path for path in TEXTURE_DIR.iterdir()
        if path.is_file() and path.suffix.lower() in TEXTURE_EXTENSIONS
    )
    if not candidates:
        return None
    return str(candidates[int(rng.integers(len(candidates)))])


def appearance_profile(rng):
    contract = SCENE_DISTRIBUTION["appearance"]
    condition = str(rng.choice(contract["condition_order"], p=contract["condition_probability"]))
    palette_options = np.asarray([
        ((0.14, 0.18, 0.24), (0.75, 0.25, 0.12), (0.17, 0.57, 0.42)),
        ((0.20, 0.18, 0.13), (0.18, 0.35, 0.70), (0.72, 0.64, 0.18)),
        ((0.24, 0.24, 0.24), (0.42, 0.42, 0.42), (0.62, 0.62, 0.62)),
    ], dtype=float)
    palette = palette_options[int(rng.integers(len(palette_options)))]
    profile = {
        "condition": condition,
        "floor_color": vector(palette[0]),
        "obstacle_color": vector(palette[1]),
        "gate_color": vector(palette[2]),
        "specular_level": float(rng.uniform(*contract["specular_level"])),
        "ambient_intensity": float(contract["baseline_ambient_intensity"]),
        "key_intensity": float(contract["baseline_key_intensity"]),
        "key_temperature_k": float(contract["baseline_key_temperature_k"]),
        "motion_blur_sigma_px": 0.0,
    }
    if condition == "low_contrast":
        profile.update(zip(("ambient_intensity", "key_intensity"), contract["low_contrast_ambient_key"]))
    elif condition == "strong_shadows":
        profile.update(zip(("ambient_intensity", "key_intensity"), contract["strong_shadow_ambient_key"]))
    elif condition == "backlit":
        profile.update(zip(
            ("ambient_intensity", "key_intensity", "key_temperature_k"),
            contract["backlit_ambient_key_temperature"],
        ))
    elif condition == "low_texture":
        profile.update(floor_color=[0.34, 0.34, 0.34], obstacle_color=[0.39, 0.39, 0.39])
    elif condition == "motion_blur":
        profile["motion_blur_sigma_px"] = float(rng.uniform(*contract["motion_blur_sigma_px"]))
    return profile


def add_obstacle(specs, center, size, primitive="box", arrangement="isolated"):
    size = tuple(float(value) for value in size)
    if primitive in {"cylinder", "cone"}:
        size = (size[0], size[0], size[2])
    specs.append({
        "object_id": f"obstacle_{len(specs):05d}",
        "primitive": str(primitive),
        "arrangement": str(arrangement),
        "center_m": vector(center),
        "size_m": vector(size),
        "yaw_rad": 0.0,
    })


def add_arch(specs, center, span, height, depth):
    """Compose an arch from two posts and a top beam."""
    center = np.asarray(center, dtype=float)
    post_size = (depth, 0.18, height)
    add_obstacle(specs, center + np.asarray((0.0, -span / 2.0, height / 2.0)), post_size, arrangement="arch")
    add_obstacle(specs, center + np.asarray((0.0, span / 2.0, height / 2.0)), post_size, arrangement="arch")
    add_obstacle(specs, center + np.asarray((0.0, 0.0, height)), (depth, span + 0.18, 0.18), arrangement="arch")


def add_mixed_obstacle_field(rng, specs, extent, protected_points, count=6):
    """Add varied floor-supported objects while keeping the reference route open."""
    half_x, half_y = extent[0] / 2.0, extent[1] / 2.0
    contract = SCENE_DISTRIBUTION["obstacles"]
    primitive_cycle = tuple(contract["primitive_cycle"])
    protected = np.asarray([point[:2] for point in protected_points], dtype=float)
    maximum_attempts = int(SCENE_DISTRIBUTION["obstacles"]["maximum_placement_attempts_per_object"])
    # Sequential random packing can paint itself into a corner even when the
    # room has ample usable area. Restart the mixed portion as a whole instead
    # of silently returning fewer obstacles or failing on one unlucky ordering.
    core_obstacle_count = len(specs)
    for field_attempt in range(20):
        del specs[core_obstacle_count:]
        field_complete = True
        for index in range(count):
            primitive = primitive_cycle[index % len(primitive_cycle)]
            placed = False
            for _ in range(maximum_attempts):
                x_inset, y_inset = contract["wall_inset_x_m"], contract["wall_inset_y_m"]
                x = float(rng.uniform(-half_x + x_inset, half_x - x_inset))
                y = float(rng.uniform(-half_y + y_inset, half_y - y_inset))
                if primitive == "sphere":
                    diameter = float(rng.uniform(*contract["sphere_diameter_m"]))
                    size = (diameter, diameter, diameter)
                elif primitive in {"cylinder", "cone"}:
                    diameter = float(rng.uniform(*contract["circular_diameter_m"]))
                    height = float(rng.uniform(*contract["height_m"]))
                    size = (diameter, diameter, height)
                else:
                    width = float(rng.uniform(*contract["box_xy_m"]))
                    depth = float(rng.uniform(*contract["box_xy_m"]))
                    height = float(rng.uniform(*contract["height_m"]))
                    size = (width, depth, height)
                center = np.asarray((x, y, size[2] / 2.0))
                route_min, route_max = contract["route_clearance_m"]
                route_clearance = min(route_max, max(route_min, half_y - 0.95))
                obstacle_radius = 0.5 * float(np.linalg.norm(size[:2]))
                required_route_clearance = max(
                    route_clearance, obstacle_radius + DRONE_COLLISION_RADIUS_M + 0.08
                )
                route_clear = distance_to_route_segments(
                    center[:2], protected
                ) > required_route_clearance
                existing_clear = all(
                    np.linalg.norm(center[:2] - np.asarray(spec["center_m"][:2]))
                    > contract["minimum_existing_center_clearance_base_m"] + max(size[0], size[1]) / 2.0
                    for spec in specs
                )
                if route_clear and existing_clear:
                    add_obstacle(specs, center, size, primitive=primitive, arrangement="mixed_field")
                    placed = True
                    break
            if not placed:
                field_complete = False
                break
        if field_complete:
            return
    del specs[core_obstacle_count:]
    raise RuntimeError(
        f"failed to place all {count} non-overlapping mixed-field obstacles after "
        f"{field_attempt + 1} complete field attempts"
    )


def scenario_layout(rng, scenario, extent, room_type):
    """Return obstacle geometry, gate geometry, and a feasible reference route."""
    contract = SCENE_DISTRIBUTION["scenario_core"]
    half_x, half_y = extent[0] / 2, extent[1] / 2
    inset = float(contract["start_finish_wall_inset_x_m"])
    altitude = float(contract["route_altitude_m"])
    start = np.asarray((-half_x + inset, float(rng.uniform(*contract["start_finish_lateral_m"])), altitude))
    finish = np.asarray((half_x - inset, float(rng.uniform(*contract["start_finish_lateral_m"])), altitude))
    obstacles, gates = [], []
    mid_x = float(rng.uniform(*np.asarray(contract["midpoint_x_extent_fraction"]) * extent[0]))
    if scenario == "curved_corridor":
        # A large, floor-supported central cylinder forces a genuine arc. The
        # route is a sampled semicircle rather than a single lateral waypoint,
        # so tangent yaw changes continuously throughout the maneuver.
        side = float(rng.choice((-1.0, 1.0)))
        radius = float(
            min(extent) * rng.uniform(*contract["curved_course_radius_fraction_of_min_extent"])
        )
        obstacle_clearance = float(rng.uniform(
            *contract["curved_course_clearance_from_center_obstacle_m"]
        ))
        center_radius = max(0.48, radius - obstacle_clearance)
        center = np.asarray((mid_x, 0.0, 0.70))
        add_obstacle(
            obstacles, center, (2.0 * center_radius, 2.0 * center_radius, 1.40),
            primitive="cylinder", arrangement="curved_course_center",
        )
        angles = np.linspace(
            np.pi, 0.0, int(contract["curved_course_route_samples"])
        )
        arc = [
            np.asarray((
                mid_x + radius * np.cos(angle),
                side * radius * np.sin(angle), altitude,
            ))
            for angle in angles
        ]
        route = [start, *arc, finish]
    elif scenario == "s_turn":
        # Alternating wall-attached barriers create a chicane. Both barriers
        # are floor-supported and leave a physically traversable room-scale gap.
        side = float(rng.choice((-1.0, 1.0)))
        gap = float(
            extent[1] * rng.uniform(*contract["s_turn_gap_fraction_of_room_width"])
        )
        thickness = float(rng.uniform(*contract["s_turn_wall_thickness_m"]))
        x_offset = float(contract["s_turn_wall_x_fraction"]) * extent[0]
        barrier_span = extent[1] - gap
        barrier_height = 1.45
        add_obstacle(
            obstacles, (-x_offset, -side * gap / 2.0, barrier_height / 2.0),
            (thickness, barrier_span, barrier_height), primitive="box",
            arrangement="s_turn_wall",
        )
        add_obstacle(
            obstacles, (x_offset, side * gap / 2.0, barrier_height / 2.0),
            (thickness, barrier_span, barrier_height), primitive="box",
            arrangement="s_turn_wall",
        )
        first_gap = side * (half_y - gap / 2.0)
        second_gap = -side * (half_y - gap / 2.0)
        route = [
            start,
            np.asarray((-x_offset, first_gap, altitude)),
            np.asarray((x_offset, second_gap, altitude)),
            finish,
        ]
    elif scenario == "corner_turn":
        # A room-scale central block forces a broad left or right dogleg. This
        # produces sustained yaw without relying on an airborne obstacle.
        side = float(rng.choice((-1.0, 1.0)))
        fraction = float(rng.uniform(*contract["corner_block_fraction_xy"]))
        block_x = fraction * extent[0]
        block_y = fraction * extent[1]
        block_height = 1.45
        add_obstacle(
            obstacles, (mid_x, 0.0, block_height / 2.0),
            (block_x, block_y, block_height), primitive="box",
            arrangement="corner_turn_block",
        )
        clearance = 0.68
        bypass_y = side * (block_y / 2.0 + clearance)
        route = [
            start,
            np.asarray((mid_x - block_x / 2.0 - clearance, bypass_y, altitude)),
            np.asarray((mid_x + block_x / 2.0 + clearance, bypass_y, altitude)),
            finish,
        ]
    elif scenario == "collision":
        center = np.asarray((mid_x, 0.0, 0.60))
        add_obstacle(obstacles, center, contract["collision_obstacle_size_m"])
        side = float(rng.choice((-1.0, 1.0)))
        route = [start, np.asarray((center[0], side * contract["collision_bypass_lateral_m"], altitude)), finish]
    elif scenario == "near_miss":
        center = np.asarray((mid_x, 0.0, 0.55))
        add_obstacle(obstacles, center, contract["near_miss_obstacle_size_m"])
        side = float(rng.choice((-1.0, 1.0)))
        route = [start, np.asarray((mid_x, side * contract["near_miss_bypass_lateral_m"], altitude)), finish]
    elif scenario == "corridor":
        gap = float(rng.uniform(*contract["corridor_gap_m"]))
        for side in (-1.0, 1.0):
            size = contract["corridor_obstacle_size_m"]
            add_obstacle(obstacles, (mid_x, side * (gap / 2 + size[1] / 2), size[2] / 2), size, primitive="box", arrangement="corridor")
        route = [start, np.asarray((mid_x, 0.0, altitude)), finish]
    elif scenario == "competing_obstacles":
        x_fraction = contract["competing_x_extent_fraction"]
        xs = np.linspace(x_fraction[0] * extent[0], x_fraction[1] * extent[0], int(contract["competing_obstacle_count"]))
        signs = rng.choice((-1.0, 1.0), size=len(xs))
        route = [start]
        for index, x in enumerate(xs):
            y = float(signs[index] * rng.uniform(*contract["competing_lateral_m"]))
            add_obstacle(obstacles, (x, y, 0.60), (0.62, 0.68, 1.20), primitive=str(rng.choice(("box", "cylinder", "cone", "sphere"))), arrangement="slalom")
            route.append(np.asarray((x, -signs[index] * contract["competing_route_opposite_lateral_m"], altitude)))
        add_arch(
            obstacles, (float(rng.uniform(-0.15, 0.15) * extent[0]), 0.0, 0.0),
            span=contract["arch_span_m"], height=contract["arch_height_m"], depth=contract["arch_depth_m"],
        )
        route.append(finish)
    elif scenario == "vertical_over":
        barrier_height = float(contract["vertical_over_barrier_height_m"])
        barrier_width = extent[1] - 2.0 * float(contract["vertical_over_barrier_wall_inset_m"])
        add_obstacle(
            obstacles,
            (mid_x, 0.0, barrier_height / 2.0),
            (float(contract["vertical_over_barrier_depth_m"]), barrier_width, barrier_height),
            primitive="box",
            arrangement="vertical_over_floor_supported_barrier",
        )
        route = [
            start,
            np.asarray((mid_x, 0.0, float(contract["vertical_over_route_height_m"]))),
            finish,
        ]
    elif scenario == "vertical_under":
        beam_height = float(contract["vertical_under_beam_center_height_m"])
        beam_span = extent[1] - 2.0 * float(contract["vertical_under_beam_wall_inset_m"])
        add_arch(
            obstacles,
            (mid_x, 0.0, 0.0),
            span=beam_span,
            height=beam_height,
            depth=float(contract["vertical_under_beam_depth_m"]),
        )
        route = [
            start,
            np.asarray((mid_x, 0.0, float(contract["vertical_under_route_height_m"]))),
            finish,
        ]
    else:
        gate_center = np.asarray((mid_x, 0.0, SCENE_DISTRIBUTION["gate"]["center_height_m"]))
        gates.append(gate_geometry(gate_center, 0.0))
        gate_route_point = gate_center.copy()
        gate_route_point[2] = altitude
        route = [start, gate_route_point, finish]
        if scenario == "obstacle_gate":
            add_obstacle(obstacles, (mid_x + 0.70, -0.95, 0.60), (0.70, 0.62, 1.20), primitive="cylinder", arrangement="obstacle_gate")
            add_obstacle(obstacles, (mid_x + 1.20, 0.82, 0.55), (0.56, 0.54, 1.10), primitive="cone", arrangement="obstacle_gate")
    count_low, count_high = SCENE_DISTRIBUTION["obstacles"][
        "mixed_field_count_by_room_inclusive"
    ][room_type]
    add_mixed_obstacle_field(
        rng, obstacles, extent, route,
        count=int(rng.integers(int(count_low), int(count_high) + 1)),
    )
    return obstacles, gates, route


def sample_room(geometry_rng):
    """Sample room class and dimensions for the episode's bounded environment."""
    return sample_room_from_distribution(geometry_rng, SCENE_DISTRIBUTION)


def generate_track(geometry_rng, appearance_rng, scenario):
    room_type, extent = sample_room(geometry_rng)
    obstacles, gates, route = scenario_layout(geometry_rng, scenario, extent, room_type)
    appearance = appearance_profile(appearance_rng)
    for obstacle in obstacles:
        # Texture choice is per obstacle, not per episode. This gives a mixed
        # material field while preserving deterministic appearance replay.
        obstacle["texture"] = choose_texture(appearance_rng)
    validate_mechanical_feasibility(obstacles)
    return {
        "extent_m": list(extent),
        "room_type": room_type,
        "scenario": scenario,
        "obstacles": obstacles,
        "gates": gates,
        "route_waypoints_m": [vector(point) for point in route],
        "appearance": {
            **appearance,
            "track_texture": choose_texture(appearance_rng),
            "wall_texture": choose_texture(appearance_rng),
            "gate_texture": choose_texture(appearance_rng),
        },
    }


def build_scene(track):
    """Build only the randomized track, its collision geometry, and calibrated camera."""
    omni.usd.get_context().new_stage()
    stage = omni.usd.get_context().get_stage()
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    rep.orchestrator.set_capture_on_play(False)
    rep.functional.create.xform(name="World")
    rep.functional.create.xform(parent="/World", name="Track")

    appearance = track["appearance"]
    specular_level = appearance["specular_level"]
    floor_material = create_material(
        "TrackFloor", appearance["floor_color"], 0.72, specular_level,
        texture=appearance.get("track_texture"),
    )
    gate_material = create_material(
        "Gate", appearance["gate_color"], 0.62, specular_level,
        texture=appearance.get("gate_texture"),
    )
    extent_x, extent_y = track["extent_m"]
    create_primitive("Floor", (0.0, 0.0, -0.08), (extent_x + 1.0, extent_y + 1.0, 0.12), "track", floor_material, primitive="box")
    # Four room-scale planes provide a bounded, realistic visual background.
    # They intentionally share the randomized obstacle material/appearance.
    wall_height = float(SCENE_DISTRIBUTION["room"]["wall_height_m"])
    wall_material = create_material(
        "Walls", appearance["obstacle_color"], 0.46, specular_level,
        texture=appearance.get("wall_texture"),
    )
    create_wall_plane("WallNorth", (0.0, extent_y / 2.0, wall_height / 2.0), (extent_x + 1.0, wall_height, 1.0), wall_material, (90.0, 0.0, 0.0))
    create_wall_plane("WallSouth", (0.0, -extent_y / 2.0, wall_height / 2.0), (extent_x + 1.0, wall_height, 1.0), wall_material, (90.0, 0.0, 0.0))
    create_wall_plane("WallEast", (extent_x / 2.0, 0.0, wall_height / 2.0), (extent_y + 1.0, wall_height, 1.0), wall_material, (90.0, 0.0, 90.0))
    create_wall_plane("WallWest", (-extent_x / 2.0, 0.0, wall_height / 2.0), (extent_y + 1.0, wall_height, 1.0), wall_material, (90.0, 0.0, 90.0))
    for index, spec in enumerate(track["obstacles"]):
        obstacle_material = create_material(
            f"ObstacleMaterial_{index}", appearance["obstacle_color"], 0.46,
            specular_level, texture=spec.get("texture"),
        )
        create_obstacle(f"Obstacle_{index}", spec, obstacle_material)
    for index, gate in enumerate(track["gates"]):
        create_gate(f"Gate_{index}", gate, gate_material)

    dome = UsdLux.DomeLight.Define(stage, "/World/Ambient")
    dome.CreateIntensityAttr(float(appearance["ambient_intensity"]))
    dome.CreateColorAttr(Gf.Vec3f(0.78, 0.84, 1.0))
    key = UsdLux.RectLight.Define(stage, "/World/KeyLight")
    key.CreateWidthAttr(3.0)
    key.CreateHeightAttr(3.0)
    key.CreateIntensityAttr(float(appearance["key_intensity"]))
    key.CreateEnableColorTemperatureAttr(True)
    key.CreateColorTemperatureAttr(float(appearance["key_temperature_k"]))
    key_xform = UsdGeom.Xformable(key.GetPrim())
    key_xform.AddTranslateOp().Set(Gf.Vec3d(-extent_x * 0.20, -extent_y * 0.32, 4.0))
    # RectLight emits along local -Z; above the course, the default orientation
    # points it downward into the scene.
    key_xform.AddRotateXOp().Set(0.0)

    camera = rep.functional.create.camera(position=(0, 0, 1), look_at=(1, 0, 1), parent="/World", name="Camera")
    usd_camera = UsdGeom.Camera(camera)
    usd_camera.CreateClippingRangeAttr(Gf.Vec2f(args.near_clip_m, 100.0))
    intrinsic = camera_calibration["camera_matrix"]
    aperture = 20.955
    focal_length = float(intrinsic[0][0] * aperture / OVERSCAN_RESOLUTION)
    usd_camera.CreateHorizontalApertureAttr(aperture)
    usd_camera.CreateVerticalApertureAttr(float(focal_length * OVERSCAN_RESOLUTION / intrinsic[1][1]))
    usd_camera.CreateFocalLengthAttr(focal_length)
    camera_xform = UsdGeom.Xformable(camera)
    camera_xform.ClearXformOpOrder()
    camera_matrix = camera_xform.MakeMatrixXform()
    render_product = rep.create.render_product(camera, (OVERSCAN_RESOLUTION, OVERSCAN_RESOLUTION), name="HM01B0Render")
    return camera_matrix, render_product


def look_at_matrix(eye, target, up=None):
    eye = Gf.Vec3d(*eye)
    target = Gf.Vec3d(*target)
    up = Gf.Vec3d(*(up if up is not None else (0, 0, 1)))
    forward = (target - eye).GetNormalized()
    if abs(forward * up) > 0.99:
        up = Gf.Vec3d(0, 1, 0)
    right = (forward ^ up).GetNormalized()
    camera_up = (right ^ forward).GetNormalized()
    matrix = Gf.Matrix4d()
    matrix[0] = [right[0], right[1], right[2], 0]
    matrix[1] = [camera_up[0], camera_up[1], camera_up[2], 0]
    matrix[2] = [-forward[0], -forward[1], -forward[2], 0]
    matrix[3] = [eye[0], eye[1], eye[2], 1]
    return matrix


def point_box_distance(point, center, size):
    point, center, size = np.asarray(point), np.asarray(center), np.asarray(size)
    outside = np.maximum(np.abs(point - center) - size / 2.0, 0.0)
    return float(np.linalg.norm(outside))


def ray_box_distance(origin, direction, center, size):
    """Distance to the first forward ray/AABB intersection, if one exists."""
    low = np.asarray(center, dtype=float) - np.asarray(size, dtype=float) / 2.0
    high = np.asarray(center, dtype=float) + np.asarray(size, dtype=float) / 2.0
    origin, direction = np.asarray(origin, dtype=float), np.asarray(direction, dtype=float)
    near, far = -math.inf, math.inf
    for axis in range(3):
        if abs(direction[axis]) < 1.0e-8:
            if origin[axis] < low[axis] or origin[axis] > high[axis]:
                return math.inf
            continue
        entry = (low[axis] - origin[axis]) / direction[axis]
        exit = (high[axis] - origin[axis]) / direction[axis]
        near, far = max(near, min(entry, exit)), min(far, max(entry, exit))
        if far < max(near, 0.0):
            return math.inf
    return max(near, 0.0)


def privileged_state(position, velocity, obstacles):
    speed = float(np.linalg.norm(velocity))
    forward = velocity / speed if speed > 0.05 else np.asarray((1.0, 0.0, 0.0))
    distances = [point_box_distance(position, spec["center_m"], spec["size_m"]) for spec in obstacles]
    ray_distances = [ray_box_distance(position, forward, spec["center_m"], spec["size_m"]) for spec in obstacles]
    ray_distance = min(ray_distances, default=math.inf)
    nearest_distance = float(min(distances, default=math.inf))
    clearance = nearest_distance - DRONE_COLLISION_RADIUS_M
    return {
        "nearest_obstacle_distance_m": nearest_distance,
        "drone_clearance_m": clearance,
        "drone_collision_radius_m": DRONE_COLLISION_RADIUS_M,
        "forward_obstacle_distance_m": None if math.isinf(ray_distance) else float(ray_distance),
        "ttc_s": None if math.isinf(ray_distance) or speed <= 0.05 else float(ray_distance / speed),
        "collision": bool(clearance <= 0.0),
    }


def build_distortion_remap():
    intrinsic = np.asarray(camera_calibration["camera_matrix"], np.float64)
    distortion = np.asarray(camera_calibration.get("simulation_distortion_coefficients", camera_calibration["distortion_coefficients"]), np.float64)
    u, v = np.meshgrid(np.arange(args.width, dtype=np.float64), np.arange(args.height, dtype=np.float64))
    pixels = np.stack((u, v), axis=-1).reshape(-1, 1, 2)
    ideal = cv2.undistortPointsIter(
        pixels, intrinsic, distortion, None, None,
        (cv2.TERM_CRITERIA_COUNT | cv2.TERM_CRITERIA_EPS, 50, 1e-12),
    ).reshape(args.height, args.width, 2)
    return (
        (ideal[..., 0] * intrinsic[0][0] + OVERSCAN_RESOLUTION / 2).astype(np.float32),
        (ideal[..., 1] * intrinsic[1][1] + OVERSCAN_RESOLUTION / 2).astype(np.float32),
    )


def apply_sensor_model(output_dir, sensor_rng, appearance):
    """Apply calibrated lens mapping and lightweight HM01B0-like sensor noise."""
    map_x, map_y = build_distortion_remap()
    blur_sigma = float(appearance["motion_blur_sigma_px"])
    for index, source in enumerate(sorted(output_dir.glob("rgb_*.png"))):
        image = cv2.imread(str(source), cv2.IMREAD_UNCHANGED)
        if image is None:
            raise RuntimeError(f"missing RGB output: {source}")
        warped = cv2.remap(image, map_x, map_y, cv2.INTER_LINEAR)
        if blur_sigma:
            warped = cv2.GaussianBlur(warped, (0, 0), blur_sigma)
        gray = cv2.cvtColor(warped, cv2.COLOR_BGR2GRAY).astype(np.float32) / 255.0
        gain = float(sensor_rng.normal(1.0, 0.06))
        response = np.clip(0.05 + 0.95 * np.power(np.clip(gray * gain, 0.0, 1.0), 1.15), 0.0, 1.0)
        noise = sensor_rng.normal(0.0, np.sqrt(14.0 + 55.0 * response), response.shape)
        sampled = np.clip(np.rint(response * 255.0 + noise), 0, 255).astype(np.uint8)
        if not cv2.imwrite(str(source), cv2.cvtColor(sampled, cv2.COLOR_GRAY2BGR)):
            raise RuntimeError(f"failed to write RGB output: {source}")
    for source in sorted(output_dir.glob("distance_to_image_plane_*.npy")):
        depth = np.load(source, allow_pickle=False)
        np.save(source, cv2.remap(depth, map_x, map_y, cv2.INTER_NEAREST), allow_pickle=False)
    for source in sorted(output_dir.glob("semantic_segmentation_*.png")):
        if source.name.endswith("labels.json"):
            continue
        semantic = cv2.imread(str(source), cv2.IMREAD_UNCHANGED)
        if semantic is None:
            raise RuntimeError(f"missing semantic output: {source}")
        if not cv2.imwrite(str(source), cv2.remap(semantic, map_x, map_y, cv2.INTER_NEAREST)):
            raise RuntimeError(f"failed to write semantic output: {source}")


def sensor_seed_for_trajectory(master_seed, layout_index, trajectory_index):
    sequence = np.random.SeedSequence([master_seed, layout_index, trajectory_index, 1771])
    return int(sequence.generate_state(1, dtype=np.uint64)[0])


def render_trajectory(
    layout_dir, track, trajectory_manifest, export_stage=False, execution=None,
):
    output_dir = layout_dir / "trajectories" / trajectory_manifest["trajectory_id"]
    output_dir.mkdir(parents=True, exist_ok=True)
    for pattern in ("rgb_*.png", "distance_to_image_plane_*.npy", "semantic_segmentation_*.png", "semantic_segmentation_labels_*.json"):
        for stale in output_dir.glob(pattern):
            stale.unlink()

    execution = execute_trajectory(trajectory_manifest) if execution is None else execution
    physical_rollout_times = execution.pop("_physical_rollout_time_s")
    physical_rollout_states = execution.pop("_physical_rollout_state_f32")
    executed_states = execution["states"]
    camera_matrix, render_product = build_scene(track)
    if export_stage:
        stage_path = layout_dir / "scene.usda"
        if not omni.usd.get_context().get_stage().GetRootLayer().Export(str(stage_path)):
            raise RuntimeError(f"failed to export stage: {stage_path}")
    if args.save_stage is not None and export_stage:
        args.save_stage.mkdir(parents=True, exist_ok=True)
        stage_path = args.save_stage / f"{track['layout_id']}.usda"
        if not omni.usd.get_context().get_stage().GetRootLayer().Export(str(stage_path)):
            raise RuntimeError(f"failed to export stage: {stage_path}")

    backend = rep.backends.get("DiskBackend")
    backend.initialize(output_dir=str(output_dir))
    writer = rep.writers.get("BasicWriter")
    writer.initialize(
        backend=backend,
        rgb=True,
        distance_to_image_plane=True,
        semantic_segmentation=True,
        colorize_semantic_segmentation=False,
        semantic_types=["class"],
    )
    writer.attach(render_product)
    records = []
    collision_frame_index = None
    for state in executed_states:
        position = np.asarray(state["position_m"], dtype=float)
        velocity = np.asarray(state["velocity_mps"], dtype=float)
        acceleration = np.asarray(state["acceleration_mps2"], dtype=float)
        frame_privileged = privileged_state(position, velocity, track["obstacles"])
        if state["frame_index"] > 0 and frame_privileged["collision"]:
            collision_frame_index = int(state["frame_index"])
            break

        roll, pitch, yaw = (float(value) for value in state["attitude_rpy_rad"])
        cosine_roll, sine_roll = math.cos(roll), math.sin(roll)
        cosine_pitch, sine_pitch = math.cos(pitch), math.sin(pitch)
        cosine_yaw, sine_yaw = math.cos(yaw), math.sin(yaw)
        rotation = np.asarray((
            (cosine_yaw * cosine_pitch, cosine_yaw * sine_pitch * sine_roll - sine_yaw * cosine_roll, cosine_yaw * sine_pitch * cosine_roll + sine_yaw * sine_roll),
            (sine_yaw * cosine_pitch, sine_yaw * sine_pitch * sine_roll + cosine_yaw * cosine_roll, sine_yaw * sine_pitch * cosine_roll - cosine_yaw * sine_roll),
            (-sine_pitch, cosine_pitch * sine_roll, cosine_pitch * cosine_roll),
        ))
        forward, lateral, up = rotation[:, 0], rotation[:, 1], rotation[:, 2]
        # TinyMPC uses body +Y as its positive lateral axis, while an OpenUSD
        # camera uses +X to the image right.  For a forward-looking camera,
        # image right is therefore body -Y.
        camera_right = -lateral
        # The TinyMPC deck=both vehicle model puts the AI deck 1 cm above its
        # body origin; render from that physical camera-deck location.
        camera_position = position + rotation @ np.asarray((0.0, 0.0, 0.010))
        target = camera_position + forward
        camera_matrix.Set(look_at_matrix(camera_position, target, up))
        rep.orchestrator.step(rt_subframes=args.rt_subframes, delta_time=float(state["delta_t_s"]))
        records.append({
            "frame_index": int(state["frame_index"]),
            "time_s": float(state["time_s"]),
            "delta_t_s": float(state["delta_t_s"]),
            "vehicle_state": {
                "position_m": vector(position),
                "velocity_mps": vector(velocity),
                "acceleration_mps2": vector(acceleration),
                "attitude_rpy_rad": state["attitude_rpy_rad"],
                "angular_velocity_rps": state["angular_velocity_rps"],
            },
            "camera_state": {
                "position_m": vector(camera_position),
                "forward_world": vector(forward),
                "right_world": vector(camera_right),
                "up_world": vector(up),
                "attitude_rpy_rad": state["attitude_rpy_rad"],
                "calibration": "hm01b0",
                "axis_convention": {
                    "tinympc_local": "+x forward, +y positive lateral, +z up",
                    "image": "+u right = -local y, +v down = -local z",
                },
            },
            "privileged": frame_privileged,
        })
    rep.orchestrator.wait_until_complete()
    writer.detach()
    sensor_seed = sensor_seed_for_trajectory(
        args.trajectory_seed, track["layout_index"], trajectory_manifest["trajectory_index"]
    )
    apply_sensor_model(output_dir, np.random.default_rng(sensor_seed), track["appearance"])
    np.savez_compressed(
        output_dir / "physical_rollout_high_rate.npz",
        time_s_f64=physical_rollout_times,
        physical_state_f32=physical_rollout_states,
    )
    (output_dir / "frames.jsonl").write_text("\n".join(json.dumps(record) for record in records) + "\n")
    minimum_clearance = min(
        (record["privileged"]["drone_clearance_m"] for record in records),
        default=math.inf,
    )
    actual_outcome_type = (
        "collision" if collision_frame_index is not None
        else "near_miss" if minimum_clearance <= float(TRAJECTORY_CONTRACT["near_miss_clearance_m"][1])
        else "safe"
    )
    rendered_manifest = {
        **{key: value for key, value in trajectory_manifest.items() if key != "states"},
        "reference_trajectory_type": trajectory_manifest["trajectory_type"],
        "trajectory_type": actual_outcome_type,
        "minimum_executed_clearance_m": minimum_clearance,
        "camera_geometry_convention_version": 2,
        "reference_states": trajectory_manifest["states"],
        "states": executed_states,
        "execution": {key: value for key, value in execution.items() if key != "states"},
        "sensor_noise_seed": sensor_seed,
        "rendered_frame_count": len(records),
        "terminated_on_collision": collision_frame_index is not None,
        "collision_frame_index": collision_frame_index,
        "modalities": [
            "rgb", "distance_to_image_plane", "semantic_segmentation",
            "physical_rollout_high_rate",
        ],
    }
    (output_dir / "trajectory_manifest.json").write_text(json.dumps(rendered_manifest, indent=2) + "\n")
    (output_dir / "_SUCCESS").write_text(json.dumps({"frames": len(records)}) + "\n")
    return rendered_manifest


def layout_document(layout_index, scenario, split=None):
    seeds = layout_seeds(args.seed, layout_index)
    streams = rngs_for_layout(seeds)
    track = generate_track(streams["scene_geometry"], streams["appearance"], scenario)
    extent_x, extent_y = track["extent_m"]
    track.update({
        "layout_id": f"layout_{layout_index:05d}",
        "layout_index": layout_index,
        "split": split_for_layout(args.seed, layout_index) if split is None else split,
        "deterministic_seeds": {"layout_master": args.seed, **seeds},
        "coordinate_system": {"up_axis": "z", "units": "meters"},
        "flight_volume_surface_altitude_m": SCENE_DISTRIBUTION["room"]["flight_volume_surface_altitude_m"],
        "scene_distribution": {
            "path": str(DISTRIBUTION_PATH),
            "sha256": SCENE_DISTRIBUTION_SHA256,
            "schema_version": SCENE_DISTRIBUTION["schema_version"],
        },
        "floor": {"center_m": [0.0, 0.0, -0.08], "size_m": [extent_x + 1.0, extent_y + 1.0, 0.12]},
        "boundaries": [
            {"name": "north", "plane": [0.0, -1.0, 0.0, extent_y / 2.0], "height_m": 3.0},
            {"name": "south", "plane": [0.0, 1.0, 0.0, extent_y / 2.0], "height_m": 3.0},
            {"name": "east", "plane": [-1.0, 0.0, 0.0, extent_x / 2.0], "height_m": 3.0},
            {"name": "west", "plane": [1.0, 0.0, 0.0, extent_x / 2.0], "height_m": 3.0},
        ],
    })
    for gate_index, gate in enumerate(track["gates"]):
        for part in gate["parts"]:
            part["object_id"] = f"gate_{gate_index:05d}_{part['object_id'].rsplit('_', 1)[-1]}"
    return track


def main():
    args.output_dir.mkdir(parents=True, exist_ok=True)
    dataset_layouts = []
    scenario_occurrences = {}
    for layout_index in range(args.layout_start_index, args.layout_start_index + args.layouts):
        scenario = SCENARIOS[layout_index % len(SCENARIOS)] if args.scenario == "mixed" else args.scenario
        split = None
        if args.scenario == "mixed":
            occurrence = scenario_occurrences.get(scenario, 0)
            scenario_occurrences[scenario] = occurrence + 1
            split = stratified_split_for_scenario_occurrence(occurrence)
        track = layout_document(layout_index, scenario, split)
        layout_dir = args.output_dir / track["layout_id"]
        layout_dir.mkdir(parents=True, exist_ok=True)
        geometry_path = layout_dir / "scene_geometry.json"
        geometry_path.write_text(json.dumps(track, indent=2) + "\n")
        trajectory_summaries = []
        for trajectory_index in range(
            args.trajectory_start_index,
            args.trajectory_start_index + (0 if args.scene_only else args.trajectories_per_layout),
        ):
            manifest = execution = None
            for generation_attempt in range(32):
                candidate_manifest = generate_trajectory_manifest(
                    track,
                    layout_index,
                    trajectory_index,
                    args.trajectory_seed,
                    args.frames,
                    frame_rate_hz=args.frame_rate_hz,
                    frame_jitter_fraction=args.frame_jitter_fraction,
                    frame_skip_probability=args.frame_skip_probability,
                    generation_attempt=generation_attempt,
                )
                candidate_execution = execute_trajectory(candidate_manifest)
                first_collision = next((
                    int(state["frame_index"])
                    for state in candidate_execution["states"]
                    if privileged_state(
                        np.asarray(state["position_m"], dtype=float),
                        np.asarray(state["velocity_mps"], dtype=float),
                        track["obstacles"],
                    )["collision"]
                ), None)
                if first_collision is None or first_collision >= 10:
                    manifest, execution = candidate_manifest, candidate_execution
                    break
            if manifest is None:
                raise RuntimeError(
                    f"failed to generate trajectory {trajectory_index} with at least "
                    "10 physical pre-contact frames after 32 deterministic attempts"
                )
            rendered = render_trajectory(
                layout_dir, track, manifest, export_stage=(trajectory_index == 0),
                execution=execution,
            )
            trajectory_summaries.append({
                "trajectory_id": rendered["trajectory_id"],
                "trajectory_type": rendered["trajectory_type"],
                "nominal_speed_mps": rendered["nominal_speed_mps"],
                "rendered_frame_count": rendered["rendered_frame_count"],
            })
        layout_manifest = {
            "layout_id": track["layout_id"],
            "layout_path": track["layout_id"],
            "layout_index": layout_index,
            "split": track["split"],
            "scene_geometry": "scene_geometry.json",
            "trajectory_seed_master": args.trajectory_seed,
            "trajectories": trajectory_summaries,
        }
        (layout_dir / "layout_manifest.json").write_text(json.dumps(layout_manifest, indent=2) + "\n")
        dataset_layouts.append(layout_manifest)

    dataset_manifest = {
        "format_version": 1,
        "layout_count": args.layouts,
        "trajectories_per_layout": args.trajectories_per_layout,
        "camera_calibration": camera_calibration,
        "capture_timing": {
            "nominal_frame_rate_hz": args.frame_rate_hz,
            "jitter_fraction": args.frame_jitter_fraction,
            "frame_skip_probability": args.frame_skip_probability,
            "window_alignment": "trajectory_event_end",
        },
        "split_policy": {
            "unit": "layout",
            "method": (
                "scenario_stratified_5_train_1_validation_1_test_per_7_occurrences"
                if args.scenario == "mixed" else "seeded_hash"
            ),
            "train_fraction": args.train_fraction,
            "validation_fraction": args.validation_fraction,
            "test_fraction": 1.0 - args.train_fraction - args.validation_fraction,
        },
        "layouts": dataset_layouts,
    }
    (args.output_dir / "dataset_manifest.json").write_text(json.dumps(dataset_manifest, indent=2) + "\n")


if __name__ == "__main__":
    try:
        main()
    except BaseException:
        # Isaac's fast shutdown can terminate the interpreter before Python emits
        # an exception raised inside main(), making failed renders look successful.
        traceback.print_exc()
        raise
    finally:
        app.close()
