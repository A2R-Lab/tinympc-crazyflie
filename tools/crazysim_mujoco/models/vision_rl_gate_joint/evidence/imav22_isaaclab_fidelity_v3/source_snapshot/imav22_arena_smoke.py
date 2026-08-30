"""Render and validate one seeded IMAV22 arena reconstruction in Isaac Lab."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import sys


PROJECT_ROOT = Path(__file__).resolve().parents[2]
ISAAC_PROJECT_ROOT = Path(__file__).resolve().parents[1]

from isaaclab.app import AppLauncher


parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--output-dir", type=Path, required=True)
parser.add_argument("--seed", type=int, default=2022)
parser.add_argument(
    "--complexity-mode",
    choices=("gate_only", "static_full", "relocated_full"),
    default="static_full",
)
parser.add_argument(
    "--relocation-event-index",
    type=int,
    default=0,
    help="Apply this many consecutive 30-second relocation events before rendering.",
)
parser.add_argument("--drone-x", type=float, default=-3.35)
parser.add_argument("--drone-y", type=float, default=0.0)
parser.add_argument("--drone-yaw-deg", type=float, default=0.0)
parser.add_argument(
    "--reference-layout",
    action="store_true",
    help="Render the deterministic published-inventory reconstruction instead of a random seed.",
)
parser.add_argument("--steps", type=int, default=28)
parser.add_argument(
    "--nanoflow-model",
    type=Path,
    default=None,
    help="Optional NanoFlowNet LiteRT graph; enables matched gate-flow captures.",
)
parser.add_argument(
    "--flow-distances-m",
    type=float,
    nargs="+",
    default=(3.0, 2.2, 1.6, 1.1, 0.75, 0.50),
)
parser.add_argument(
    "--flow-lateral-offsets-m",
    type=float,
    nargs="+",
    default=(-0.25, 0.0, 0.25),
)
parser.add_argument("--flow-pair-travel-m", type=float, default=0.08)
parser.add_argument("--flow-pair-interval-s", type=float, default=0.10)
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()
args.enable_cameras = True

launcher = AppLauncher(args)
simulation_app = launcher.app

# Do not expose the project's ``isaaclab`` directory until AppLauncher has
# loaded the installed Isaac Lab package; it would otherwise shadow it.
for path in (PROJECT_ROOT, ISAAC_PROJECT_ROOT):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))


import numpy as np
import torch
import trimesh
import matplotlib.colors as mpl_colors
import matplotlib.pyplot as plt
from PIL import Image
from isaaclab_physx.renderers import IsaacRtxRendererCfg
from pxr import Gf, Sdf, UsdGeom, UsdShade

import isaaclab.sim as sim_utils
from isaaclab.sensors.camera import Camera, CameraCfg

from envs.imav22_arena import (
    DERIVED_ASSET_ROOT,
    DERIVED_FLAG_TEXTURE,
    GATE_BORDER_M,
    GATE_CLEAR_OPENING_M,
    GATE_DAE_PATH,
    MODE_OBJECT_COUNTS,
    RELOCATED_FULL,
    STATIC_FULL,
    ArenaLayout,
    ArenaObjectSpec,
    competition_reference_layout,
    relocate_one_obstacle,
    sample_randomized_layout,
    texture_path,
    verify_imav22_assets,
)
from envs.hm01b0_camera_contract import (
    CAMERA_DISTORTION_RAW_RTX,
    CAMERA_MATRIX_RAW_RTX,
    CAMERA_RESOLUTION_WH,
    CAMERA_SENSOR_BLACK_LEVEL,
    CAMERA_SENSOR_GAIN_MEAN,
    CAMERA_SENSOR_GAIN_STD,
    CAMERA_SENSOR_GAMMA,
    CAMERA_SENSOR_NOISE_VARIANCE_FLOOR_U8,
    CAMERA_SENSOR_NOISE_VARIANCE_SIGNAL_U8,
    CAMERA_SENSOR_RESPONSE_SCALE,
)
from envs.nanoflow_litert import LiteRTNanoFlowEstimator
from envs.simulator_flow_oracle import camera_pose_to_transform, rigid_flow_distorted_torch
from policies.camera_preprocessing import canonicalize_hm01b0_grayscale


def _yaw_quaternion(yaw: float) -> tuple[float, float, float, float]:
    return (math.cos(yaw / 2.0), 0.0, 0.0, math.sin(yaw / 2.0))


def _textured_material(
    color: tuple[float, float, float], *, roughness: float = 0.72
) -> sim_utils.PbrMdlCfg:
    return sim_utils.PbrMdlCfg(
        diffuse_color_constant=color,
        reflection_roughness_constant=roughness,
    )


def _author_texture(
    prim_path: str,
    relative: str,
    *,
    scale: float = 2.0,
    absolute_path: Path | None = None,
) -> None:
    shader = UsdShade.Shader.Get(
        sim_utils.get_current_stage(), f"{prim_path}/geometry/material/Shader"
    )
    if not shader:
        raise RuntimeError(f"missing OmniPBR shader below {prim_path}")
    shader.CreateInput("diffuse_texture", Sdf.ValueTypeNames.Asset).Set(
        Sdf.AssetPath(str(absolute_path if absolute_path is not None else texture_path(relative)))
    )
    shader.CreateInput("project_uvw", Sdf.ValueTypeNames.Bool).Set(True)
    shader.CreateInput("texture_scale", Sdf.ValueTypeNames.Float2).Set(
        Gf.Vec2f(float(scale), float(scale))
    )
    shader.CreateInput("specular_level", Sdf.ValueTypeNames.Float).Set(0.10)


def _uv_material(
    material_path: str,
    image_path: Path,
    *,
    alpha: bool = False,
) -> UsdShade.Material:
    """Return a shared USD PreviewSurface material driven by explicit ``st`` UVs."""

    stage = sim_utils.get_current_stage()
    material = UsdShade.Material.Get(stage, material_path)
    if material:
        return material
    material = UsdShade.Material.Define(stage, material_path)
    shader = UsdShade.Shader.Define(stage, f"{material_path}/PreviewSurface")
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.82)
    shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.0)
    reader = UsdShade.Shader.Define(stage, f"{material_path}/PrimvarReader")
    reader.CreateIdAttr("UsdPrimvarReader_float2")
    reader.CreateInput("varname", Sdf.ValueTypeNames.Token).Set("st")
    reader.CreateOutput("result", Sdf.ValueTypeNames.Float2)
    texture = UsdShade.Shader.Define(stage, f"{material_path}/Texture")
    texture.CreateIdAttr("UsdUVTexture")
    texture.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(Sdf.AssetPath(str(image_path)))
    texture.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(
        reader.ConnectableAPI(), "result"
    )
    texture.CreateInput("wrapS", Sdf.ValueTypeNames.Token).Set("repeat")
    texture.CreateInput("wrapT", Sdf.ValueTypeNames.Token).Set("repeat")
    texture.CreateOutput("rgb", Sdf.ValueTypeNames.Float3)
    texture.CreateOutput("a", Sdf.ValueTypeNames.Float)
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).ConnectToSource(
        texture.ConnectableAPI(), "rgb"
    )
    if alpha:
        shader.CreateInput("opacity", Sdf.ValueTypeNames.Float).ConnectToSource(
            texture.ConnectableAPI(), "a"
        )
        shader.CreateInput("opacityThreshold", Sdf.ValueTypeNames.Float).Set(0.05)
    material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
    return material


def _spawn_uv_mesh(
    prim_path: str,
    *,
    points: list[tuple[float, float, float]],
    face_vertex_counts: list[int],
    face_vertex_indices: list[int],
    uvs: list[tuple[float, float]],
    image_path: Path,
    material_path: str,
    alpha: bool = False,
    double_sided: bool = True,
) -> None:
    """Spawn a world-space mesh whose texture mapping is explicit and reproducible."""

    stage = sim_utils.get_current_stage()
    mesh = UsdGeom.Mesh.Define(stage, prim_path)
    mesh.CreatePointsAttr([Gf.Vec3f(*point) for point in points])
    mesh.CreateFaceVertexCountsAttr(face_vertex_counts)
    mesh.CreateFaceVertexIndicesAttr(face_vertex_indices)
    mesh.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
    mesh.CreateDoubleSidedAttr(double_sided)
    minimum = tuple(min(point[axis] for point in points) for axis in range(3))
    maximum = tuple(max(point[axis] for point in points) for axis in range(3))
    mesh.CreateExtentAttr([Gf.Vec3f(*minimum), Gf.Vec3f(*maximum)])
    st = UsdGeom.PrimvarsAPI(mesh).CreatePrimvar(
        "st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.vertex
    )
    st.Set([Gf.Vec2f(*uv) for uv in uvs])
    material = _uv_material(material_path, image_path, alpha=alpha)
    UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim())
    UsdShade.MaterialBindingAPI(mesh).Bind(material)


def _spawn_cuboid(
    prim_path: str,
    *,
    center: tuple[float, float, float],
    size: tuple[float, float, float],
    yaw: float,
    color: tuple[float, float, float],
    texture: str | None,
    collision: bool = True,
    texture_scale: float = 2.0,
) -> None:
    cfg = sim_utils.CuboidCfg(
        size=size,
        collision_props=sim_utils.CollisionPropertiesCfg() if collision else None,
        visual_material=_textured_material(color),
    )
    cfg.func(
        prim_path,
        cfg,
        translation=center,
        orientation=_yaw_quaternion(yaw),
    )
    if texture is not None:
        _author_texture(prim_path, texture, scale=texture_scale)


def _spawn_floor() -> None:
    # Eight-metre green scoring turf plus the one-metre perimeter inside the
    # published ten-metre arena.  The strip colors/textures are the Cyberzoo
    # surfaces from the official simulator branch.
    _spawn_cuboid(
        "/World/IMAV22/Floor/ScoringTurf",
        center=(0.0, 0.0, -0.035),
        size=(8.0, 8.0, 0.07),
        yaw=0.0,
        color=(0.17, 0.42, 0.12),
        texture="cyberzoo_floor/grass_green.png",
        texture_scale=6.0,
    )
    strips = (
        ("North", (0.0, 4.5, -0.035), (10.0, 1.0, 0.07), "cyberzoo_floor/grass_black.png"),
        ("South", (0.0, -4.5, -0.035), (10.0, 1.0, 0.07), "cyberzoo_floor/grass_black.png"),
        ("East", (4.5, 0.0, -0.035), (1.0, 8.0, 0.07), "cyberzoo_floor/grass_blue.png"),
        ("West", (-4.5, 0.0, -0.035), (1.0, 8.0, 0.07), "cyberzoo_floor/grass_blue.png"),
    )
    for name, center, size, texture in strips:
        _spawn_cuboid(
            f"/World/IMAV22/Floor/{name}",
            center=center,
            size=size,
            yaw=0.0,
            color=(0.08, 0.10, 0.13),
            texture=texture,
            texture_scale=4.0,
        )
    # The exact traffic-pattern rug is visible in final-arena imagery.  It is
    # a visual distractor rather than one of the ten scored obstacles.
    rug_center = (-1.05, 2.65, 0.006)
    rug_yaw = 0.12
    _spawn_cuboid(
        "/World/IMAV22/Venue/TrafficRug",
        center=rug_center,
        size=(2.0, 2.0, 0.012),
        yaw=rug_yaw,
        color=(0.16, 0.16, 0.17),
        texture=None,
        collision=False,
        texture_scale=1.0,
    )
    sim_utils.set_prim_visibility(
        sim_utils.get_prim_at_path(
            "/World/IMAV22/Venue/TrafficRug/geometry/mesh"
        ),
        False,
    )
    cosine, sine = math.cos(rug_yaw), math.sin(rug_yaw)
    rug_points = []
    for local_x, local_y in ((-1.0, -1.0), (1.0, -1.0), (1.0, 1.0), (-1.0, 1.0)):
        rug_points.append(
            (
                rug_center[0] + cosine * local_x - sine * local_y,
                rug_center[1] + sine * local_x + cosine * local_y,
                0.013,
            )
        )
    rug_points.append(
        (
            sum(point[0] for point in rug_points) / 4.0,
            sum(point[1] for point in rug_points) / 4.0,
            0.025,
        )
    )
    rug_points[:4] = [(point[0], point[1], 0.025) for point in rug_points[:4]]
    rug_uvs = [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0), (0.5, 0.5)]
    for index in range(4):
        next_index = (index + 1) % 4
        _spawn_uv_mesh(
            f"/World/IMAV22/Venue/TrafficRugTop/Triangle{index}",
            points=[rug_points[4], rug_points[index], rug_points[next_index]],
            face_vertex_counts=[3],
            face_vertex_indices=[0, 1, 2],
            uvs=[rug_uvs[4], rug_uvs[index], rug_uvs[next_index]],
            image_path=texture_path("traffic_mat/mat_traffic_1.png"),
            material_path="/World/Looks/IMAV22TrafficRug",
        )


def _spawn_venue_background() -> None:
    """Add the curtain/net perimeter and restrained non-scoring venue clutter."""

    for name, center, size in (
        ("NorthCurtain", (0.0, 4.96, 1.30), (9.92, 0.035, 2.60)),
        ("SouthCurtain", (0.0, -4.96, 1.30), (9.92, 0.035, 2.60)),
        ("EastCurtain", (4.96, 0.0, 1.30), (0.035, 9.92, 2.60)),
        ("WestCurtain", (-4.96, 0.0, 1.30), (0.035, 9.92, 2.60)),
    ):
        _spawn_cuboid(
            f"/World/IMAV22/Venue/{name}",
            center=center,
            size=size,
            yaw=0.0,
            color=(0.025, 0.027, 0.030),
            texture="cyberzoo_walls/wall_curtains.png",
            collision=False,
            texture_scale=3.0,
        )
    # The official Cyberzoo model hangs netting above part of the black curtain.
    # Keep it below 3.4 m and leave the ceiling completely open.
    for name, points in (
        (
            "NorthNet",
            [(-4.9, 4.93, 2.60), (4.9, 4.93, 2.60), (4.9, 4.93, 3.40), (-4.9, 4.93, 3.40)],
        ),
        (
            "EastNet",
            [(4.93, -4.9, 2.60), (4.93, 4.9, 2.60), (4.93, 4.9, 3.40), (4.93, -4.9, 3.40)],
        ),
    ):
        _spawn_uv_mesh(
            f"/World/IMAV22/Venue/{name}",
            points=points,
            face_vertex_counts=[4],
            face_vertex_indices=[0, 1, 2, 3],
            uvs=[(0.0, 0.0), (12.0, 0.0), (12.0, 1.0), (0.0, 1.0)],
            image_path=texture_path("cyberzoo_net/net.png"),
            material_path="/World/Looks/IMAV22CyberzooNet",
            alpha=True,
        )
    # Padded venue posts and one equipment cabinet sit outside the 8 x 8 m
    # scoring area.  They improve the background without changing obstacle ABI.
    for index, (x, y) in enumerate(((-4.65, -4.65), (4.65, -4.65), (4.65, 4.65), (-4.65, 4.65))):
        _spawn_cuboid(
            f"/World/IMAV22/Venue/PaddedPost{index}",
            center=(x, y, 1.45),
            size=(0.18, 0.18, 2.90),
            yaw=0.0,
            color=(0.24, 0.25, 0.27),
            texture="cyberzoo_poles/pole.png",
            collision=False,
            texture_scale=1.0,
        )
    _spawn_cuboid(
        "/World/IMAV22/Venue/EquipmentCabinet",
        center=(-4.56, 2.75, 0.65),
        size=(0.62, 1.45, 1.30),
        yaw=0.0,
        color=(0.30, 0.32, 0.34),
        texture="cyberzoo_surroundings/cabinets.png",
        collision=False,
        texture_scale=1.0,
    )


def _spawn_gate(obj: ArenaObjectSpec) -> None:
    x, y, z = obj.center_m
    depth = obj.size_m[0]
    clear_y, clear_z = GATE_CLEAR_OPENING_M
    border = GATE_BORDER_M
    outer_y = clear_y + 2.0 * border
    rail_offset = clear_y / 2.0 + border / 2.0
    direction_y = (-math.sin(obj.yaw_rad), math.cos(obj.yaw_rad))
    rails = (
        ("Left", -rail_offset, 0.0, (depth, border, clear_z)),
        ("Right", rail_offset, 0.0, (depth, border, clear_z)),
        ("Bottom", 0.0, -rail_offset, (depth, outer_y, border)),
        ("Top", 0.0, rail_offset, (depth, outer_y, border)),
    )
    for name, lateral, vertical, size in rails:
        center = (
            x + direction_y[0] * lateral,
            y + direction_y[1] * lateral,
            z + vertical,
        )
        _spawn_cuboid(
            f"/World/IMAV22/Objects/{obj.object_id}/{name}",
            center=center,
            size=size,
            yaw=obj.yaw_rad,
            color=obj.color_rgb,
            texture=None,
            texture_scale=1.0,
        )
        # The public DAE is authoritative for appearance.  Keep these exact
        # opening boxes as invisible collision geometry only.
        sim_utils.set_prim_visibility(
            sim_utils.get_prim_at_path(
                f"/World/IMAV22/Objects/{obj.object_id}/{name}/geometry/mesh"
            ),
            False,
        )
    _spawn_official_gate_visual(obj)


def _spawn_official_gate_visual(obj: ArenaObjectSpec) -> None:
    """Create the exact official DAE mesh with its original UV atlas."""

    scene = trimesh.load(GATE_DAE_PATH, force="scene", process=False)
    if len(scene.geometry) != 1:
        raise RuntimeError("official IMAV22 gate DAE must contain exactly one mesh")
    source = next(iter(scene.geometry.values()))
    if source.visual.uv is None or len(source.visual.uv) != len(source.vertices):
        raise RuntimeError("official IMAV22 gate DAE lost its per-vertex UV map")

    # The DAE uses local X as gate width and local Y as depth.  Arena yaw uses
    # local X as the gate normal, so rotate the DAE by yaw - 90 degrees.
    mesh_yaw = obj.yaw_rad - math.pi / 2.0
    cosine = math.cos(mesh_yaw)
    sine = math.sin(mesh_yaw)
    x, y, _opening_center_z = obj.center_m
    points = [
        Gf.Vec3f(
            x + cosine * float(vertex[0]) - sine * float(vertex[1]),
            y + sine * float(vertex[0]) + cosine * float(vertex[1]),
            float(vertex[2]),
        )
        for vertex in source.vertices
    ]
    normals = [
        Gf.Vec3f(
            cosine * float(normal[0]) - sine * float(normal[1]),
            sine * float(normal[0]) + cosine * float(normal[1]),
            float(normal[2]),
        )
        for normal in source.vertex_normals
    ]
    stage = sim_utils.get_current_stage()
    mesh = UsdGeom.Mesh.Define(
        stage, f"/World/IMAV22/Objects/{obj.object_id}/OfficialGateVisual"
    )
    mesh.CreatePointsAttr(points)
    mesh.CreateFaceVertexCountsAttr([3] * len(source.faces))
    mesh.CreateFaceVertexIndicesAttr(source.faces.reshape(-1).astype(int).tolist())
    mesh.CreateNormalsAttr(normals)
    mesh.SetNormalsInterpolation(UsdGeom.Tokens.vertex)
    mesh.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
    mesh.CreateDoubleSidedAttr(True)
    minimum = tuple(min(point[axis] for point in points) for axis in range(3))
    maximum = tuple(max(point[axis] for point in points) for axis in range(3))
    mesh.CreateExtentAttr([Gf.Vec3f(*minimum), Gf.Vec3f(*maximum)])
    st = UsdGeom.PrimvarsAPI(mesh).CreatePrimvar(
        "st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.vertex
    )
    st.Set([Gf.Vec2f(float(uv[0]), float(uv[1])) for uv in source.visual.uv])

    material_path = "/World/Looks/IMAV22OfficialGate"
    material = UsdShade.Material.Get(stage, material_path)
    if not material:
        material = UsdShade.Material.Define(stage, material_path)
        shader = UsdShade.Shader.Define(stage, f"{material_path}/PreviewSurface")
        shader.CreateIdAttr("UsdPreviewSurface")
        shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.78)
        shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.0)
        reader = UsdShade.Shader.Define(stage, f"{material_path}/PrimvarReader")
        reader.CreateIdAttr("UsdPrimvarReader_float2")
        reader.CreateInput("varname", Sdf.ValueTypeNames.Token).Set("st")
        reader.CreateOutput("result", Sdf.ValueTypeNames.Float2)
        texture = UsdShade.Shader.Define(stage, f"{material_path}/Texture")
        texture.CreateIdAttr("UsdUVTexture")
        texture.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(
            Sdf.AssetPath(str(texture_path(obj.texture_relative_path)))
        )
        texture.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(
            reader.ConnectableAPI(), "result"
        )
        texture.CreateOutput("rgb", Sdf.ValueTypeNames.Float3)
        shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).ConnectToSource(
            texture.ConnectableAPI(), "rgb"
        )
        material.CreateSurfaceOutput().ConnectToSource(
            shader.ConnectableAPI(), "surface"
        )
    UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim())
    UsdShade.MaterialBindingAPI(mesh).Bind(material)


def _spawn_feather_flag(obj: ArenaObjectSpec) -> None:
    """Spawn a smooth, double-sided feather-banner proxy from arena imagery."""

    width_axis = (-math.sin(obj.yaw_rad), math.cos(obj.yaw_rad))
    staff_xy = (
        obj.center_m[0] - 0.40 * width_axis[0],
        obj.center_m[1] - 0.40 * width_axis[1],
    )
    # Convex perimeter in staff-relative (lateral, height) coordinates.  A
    # center fan avoids the broken, segmented silhouette of the old proxy.
    perimeter = (
        (0.02, 0.18),
        (0.26, 0.23),
        (0.56, 0.48),
        (0.74, 0.88),
        (0.76, 1.30),
        (0.62, 1.58),
        (0.36, 1.72),
        (0.10, 1.68),
        (0.02, 1.50),
    )
    center_local = (
        sum(point[0] for point in perimeter) / len(perimeter),
        sum(point[1] for point in perimeter) / len(perimeter),
    )
    local_points = (*perimeter, center_local)
    points = [
        (
            staff_xy[0] + width_axis[0] * lateral,
            staff_xy[1] + width_axis[1] * lateral,
            height,
        )
        for lateral, height in local_points
    ]
    center_index = len(perimeter)
    faces = []
    for index in range(len(perimeter)):
        faces.extend((center_index, index, (index + 1) % len(perimeter)))
    _spawn_uv_mesh(
        f"/World/IMAV22/Objects/{obj.object_id}/FeatherBanner",
        points=points,
        face_vertex_counts=[3] * len(perimeter),
        face_vertex_indices=faces,
        uvs=[(lateral / 0.76, (height - 0.18) / 1.54) for lateral, height in local_points],
        image_path=DERIVED_ASSET_ROOT / DERIVED_FLAG_TEXTURE,
        material_path="/World/Looks/IMAV22FeatherBanner",
    )
    # A single hidden collision slab preserves the obstacle footprint without
    # imposing the old four-piece visual geometry.
    _spawn_cuboid(
        f"/World/IMAV22/Objects/{obj.object_id}/Collision",
        center=(obj.center_m[0], obj.center_m[1], 0.95),
        size=(obj.size_m[0], obj.size_m[1], obj.size_m[2]),
        yaw=obj.yaw_rad,
        color=(0.1, 0.1, 0.1),
        texture=None,
    )
    sim_utils.set_prim_visibility(
        sim_utils.get_prim_at_path(
            f"/World/IMAV22/Objects/{obj.object_id}/Collision/geometry/mesh"
        ),
        False,
    )
    for suffix, radius, height, z, color in (
        ("Staff", 0.018, 1.90, 0.95, (0.72, 0.74, 0.75)),
        ("Base", 0.115, 0.055, 0.0275, (0.09, 0.10, 0.11)),
    ):
        staff_cfg = sim_utils.CylinderCfg(
            radius=radius,
            height=height,
            axis="Z",
            collision_props=sim_utils.CollisionPropertiesCfg(),
            visual_material=_textured_material(color, roughness=0.72),
        )
        staff_cfg.func(
            f"/World/IMAV22/Objects/{obj.object_id}_{suffix}",
            staff_cfg,
            translation=(*staff_xy, z),
        )


def _spawn_panel(obj: ArenaObjectSpec) -> None:
    """Spawn the official one-metre black panel face on a twin-foot frame."""

    root = f"/World/IMAV22/Objects/{obj.object_id}"
    _spawn_cuboid(
        root,
        center=obj.center_m,
        size=obj.size_m,
        yaw=obj.yaw_rad,
        color=(0.035, 0.038, 0.042),
        texture=None,
    )
    sim_utils.set_prim_visibility(
        sim_utils.get_prim_at_path(f"{root}/geometry/mesh"),
        False,
    )
    normal = (math.cos(obj.yaw_rad), math.sin(obj.yaw_rad))
    width_axis = (-normal[1], normal[0])
    half_width = obj.size_m[1] / 2.0
    bottom_z = obj.center_m[2] - obj.size_m[2] / 2.0
    top_z = obj.center_m[2] + obj.size_m[2] / 2.0
    for side, sign in (("Front", 1.0), ("Back", -1.0)):
        x = obj.center_m[0] + sign * normal[0] * (obj.size_m[0] / 2.0 + 0.002)
        y = obj.center_m[1] + sign * normal[1] * (obj.size_m[0] / 2.0 + 0.002)
        left = (x - sign * width_axis[0] * half_width, y - sign * width_axis[1] * half_width)
        right = (x + sign * width_axis[0] * half_width, y + sign * width_axis[1] * half_width)
        _spawn_uv_mesh(
            f"{root}_{side}Face",
            points=[
                (*left, bottom_z),
                (*right, bottom_z),
                (*right, top_z),
                (*left, top_z),
            ],
            face_vertex_counts=[3, 3],
            face_vertex_indices=[0, 1, 2, 0, 2, 3],
            uvs=[(0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0)],
            image_path=texture_path(obj.texture_relative_path),
            material_path=f"/World/Looks/IMAV22Panel_{obj.texture_relative_path[-5]}",
            alpha=False,
            double_sided=True,
        )
    for index, lateral in enumerate((-0.32 * obj.size_m[1], 0.32 * obj.size_m[1])):
        center = (
            obj.center_m[0] + width_axis[0] * lateral,
            obj.center_m[1] + width_axis[1] * lateral,
            0.035,
        )
        _spawn_cuboid(
            f"{root}_Foot{index}",
            center=center,
            size=(0.43, 0.10, 0.07),
            yaw=obj.yaw_rad,
            color=(0.045, 0.047, 0.050),
            texture=None,
            texture_scale=1.0,
        )
        _spawn_cuboid(
            f"{root}_Support{index}",
            center=(
                obj.center_m[0] + width_axis[0] * lateral,
                obj.center_m[1] + width_axis[1] * lateral,
                0.13,
            ),
            size=(0.08, 0.08, 0.26),
            yaw=obj.yaw_rad,
            color=(0.045, 0.047, 0.050),
            texture=None,
            texture_scale=1.0,
        )


def _spawn_object(obj: ArenaObjectSpec) -> None:
    if obj.kind == "gate":
        _spawn_gate(obj)
        return
    if obj.kind == "pole":
        cfg = sim_utils.CylinderCfg(
            radius=obj.size_m[0] / 2.0,
            height=obj.size_m[2],
            axis="Z",
            collision_props=sim_utils.CollisionPropertiesCfg(),
            visual_material=_textured_material(obj.color_rgb, roughness=0.64),
        )
        prim_path = f"/World/IMAV22/Objects/{obj.object_id}"
        cfg.func(prim_path, cfg, translation=obj.center_m)
        _author_texture(prim_path, obj.texture_relative_path, scale=1.0)
        return
    if obj.kind == "flag":
        _spawn_feather_flag(obj)
        return
    if obj.kind == "panel":
        _spawn_panel(obj)
        return
    _spawn_cuboid(
        f"/World/IMAV22/Objects/{obj.object_id}",
        center=obj.center_m,
        size=obj.size_m,
        yaw=obj.yaw_rad,
        color=obj.color_rgb,
        texture=obj.texture_relative_path,
        texture_scale=2.0,
    )


def _camera(
    prim_path: str,
    width: int,
    height: int,
    *,
    aperture: float,
    with_depth: bool = False,
) -> Camera:
    sim_utils.create_prim(str(Path(prim_path).parent).replace("\\", "/"), "Xform")
    focal_length = (
        CAMERA_MATRIX_RAW_RTX[0][0] * aperture / width if with_depth else 24.0
    )
    vertical_aperture = (
        focal_length * height / CAMERA_MATRIX_RAW_RTX[1][1] if with_depth else None
    )
    distortion = (
        sim_utils.OpenCvPinholeDistortionCfg(
            fx=CAMERA_MATRIX_RAW_RTX[0][0],
            fy=CAMERA_MATRIX_RAW_RTX[1][1],
            cx=CAMERA_MATRIX_RAW_RTX[0][2],
            cy=CAMERA_MATRIX_RAW_RTX[1][2],
            image_size=CAMERA_RESOLUTION_WH,
            apply_lens_distortion=True,
            k1=CAMERA_DISTORTION_RAW_RTX[0],
            k2=CAMERA_DISTORTION_RAW_RTX[1],
            p1=CAMERA_DISTORTION_RAW_RTX[2],
            p2=CAMERA_DISTORTION_RAW_RTX[3],
            k3=CAMERA_DISTORTION_RAW_RTX[4],
        )
        if with_depth
        else None
    )
    return Camera(
        cfg=CameraCfg(
            prim_path=prim_path,
            update_period=0.0,
            update_latest_camera_pose=True,
            height=height,
            width=width,
            data_types=["rgb", "distance_to_image_plane"] if with_depth else ["rgb"],
            renderer_cfg=IsaacRtxRendererCfg(),
            spawn=sim_utils.PinholeCameraCfg(
                focal_length=focal_length,
                focus_distance=8.0,
                horizontal_aperture=aperture,
                vertical_aperture=vertical_aperture,
                clipping_range=(0.02, 30.0),
                distortion=distortion,
            ),
        )
    )


def build_scene(layout: ArenaLayout) -> tuple[Camera, Camera]:
    sim_utils.create_prim("/World/IMAV22", "Xform")
    _spawn_floor()
    _spawn_venue_background()
    for obj in layout.objects:
        _spawn_object(obj)

    # No ceiling: use Isaac Lab's neutral dome/distant-light pattern so the
    # grayscale HM01B0 proxy retains useful contrast.
    dome = sim_utils.DomeLightCfg(intensity=650.0, color=(0.86, 0.90, 1.0))
    dome.func("/World/DomeLight", dome)
    distant = sim_utils.DistantLightCfg(
        intensity=2200.0,
        color=(1.0, 0.94, 0.84),
        angle=0.45,
    )
    distant.func("/World/DistantLight", distant, orientation=(0.93, 0.18, -0.28, 0.14))

    overview = _camera("/World/OverviewRig/Camera", 720, 600, aperture=25.0)
    hm01b0 = _camera(
        "/World/HM01B0Rig/Camera", 160, 160, aperture=20.955, with_depth=True
    )
    return overview, hm01b0


def _rgb(camera: Camera) -> np.ndarray:
    return camera.data.output["rgb"].torch[0, ..., :3].detach().cpu().numpy().astype(np.uint8)


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _render_sensor_sample(
    sim: sim_utils.SimulationContext,
    camera: Camera,
    eye: tuple[float, float, float],
    target: tuple[float, float, float],
    generator: torch.Generator,
) -> dict[str, torch.Tensor]:
    camera.set_world_poses_from_view(
        torch.tensor([eye], device=sim.device),
        torch.tensor([target], device=sim.device),
    )
    for _ in range(3):
        sim.step(render=True)
        camera.update(dt=sim.get_physics_dt())
    rgb = camera.data.output["rgb"].torch[0, ..., :3]
    if rgb.dtype == torch.uint8:
        rgb_float = rgb.to(torch.float32).mul(1.0 / 255.0)
    else:
        rgb_float = rgb.to(torch.float32)
        if bool((rgb_float.detach().amax() > 1.0).item()):
            rgb_float = rgb_float.mul(1.0 / 255.0)
    gray = canonicalize_hm01b0_grayscale(
        (
            0.299 * rgb_float[..., 0]
            + 0.587 * rgb_float[..., 1]
            + 0.114 * rgb_float[..., 2]
        ).clamp(0.0, 1.0)
    )
    gain = (
        torch.randn((1, 1), device=gray.device, generator=generator)
        * CAMERA_SENSOR_GAIN_STD
        + CAMERA_SENSOR_GAIN_MEAN
    )
    response = (
        CAMERA_SENSOR_BLACK_LEVEL
        + CAMERA_SENSOR_RESPONSE_SCALE
        * (gray * gain).clamp(0.0, 1.0).pow(CAMERA_SENSOR_GAMMA)
    ).clamp(0.0, 1.0)
    noise_std = torch.sqrt(
        CAMERA_SENSOR_NOISE_VARIANCE_FLOOR_U8
        + CAMERA_SENSOR_NOISE_VARIANCE_SIGNAL_U8 * response
    ).mul(1.0 / 255.0)
    sensor_gray = (
        response
        + torch.randn(response.shape, device=gray.device, generator=generator) * noise_std
    ).clamp(0.0, 1.0)
    depth = camera.data.output["distance_to_image_plane"].torch
    if depth.ndim == 4 and depth.shape[-1] == 1:
        depth = depth[..., 0]
    depth = depth[0, 20:140].to(torch.float32).clone()
    position = camera.data.pos_w.torch.to(dtype=depth.dtype).clone()
    quaternion_ros = camera.data.quat_w_ros.torch.to(dtype=depth.dtype).clone()
    transform = camera_pose_to_transform(position, quaternion_ros)[0]
    intrinsic = camera.data.intrinsic_matrices.torch[0].to(dtype=depth.dtype).clone()
    intrinsic[1, 2] -= 20.0
    return {
        "rgb": rgb_float[20:140].clone(),
        "sensor_gray": sensor_gray[20:140].clone(),
        "depth": depth,
        "transform": transform,
        "intrinsic": intrinsic,
    }


def _flow_metrics(
    predicted: torch.Tensor, oracle: torch.Tensor, valid: torch.Tensor
) -> dict[str, float]:
    error = torch.linalg.vector_norm(predicted - oracle, dim=0)
    pred_mag = torch.linalg.vector_norm(predicted, dim=0)
    oracle_mag = torch.linalg.vector_norm(oracle, dim=0)
    return {
        "valid_fraction": float(valid.float().mean().item()),
        "mean_epe_px": float(error[valid].mean().item()) if bool(valid.any()) else 0.0,
        "p95_epe_px": (
            float(torch.quantile(error[valid], 0.95).item()) if bool(valid.any()) else 0.0
        ),
        "prediction_mean_magnitude_px": float(pred_mag.mean().item()),
        "oracle_mean_magnitude_px": (
            float(oracle_mag[valid].mean().item()) if bool(valid.any()) else 0.0
        ),
    }


def _flow_rgb(flow: np.ndarray, scale: float, valid: np.ndarray | None = None) -> np.ndarray:
    u, v = flow
    angle = (np.arctan2(v, u) + np.pi) / (2.0 * np.pi)
    magnitude = np.linalg.norm(flow, axis=0)
    hsv = np.stack(
        (
            angle,
            np.clip(magnitude / max(scale, 1.0e-6), 0.0, 1.0),
            np.ones_like(angle),
        ),
        axis=-1,
    )
    rgb = mpl_colors.hsv_to_rgb(hsv)
    if valid is not None:
        rgb = np.where(valid[..., None], rgb, 0.18)
    return rgb


def _write_flow_gallery(
    records: list[dict[str, object]], output_dir: Path, pair_interval_s: float
) -> list[str]:
    magnitudes = []
    for record in records:
        predicted = record["predicted_flow"].numpy()
        oracle = record["oracle_flow"].numpy()
        valid = record["oracle_valid"].numpy().astype(bool)
        magnitudes.append(np.linalg.norm(predicted, axis=0).reshape(-1))
        magnitudes.append(np.linalg.norm(oracle, axis=0)[valid].reshape(-1))
    shared_scale = float(np.percentile(np.concatenate(magnitudes), 95))
    gallery_dir = output_dir / "flow_gallery"
    gallery_dir.mkdir(parents=True, exist_ok=True)
    files = []
    for index, record in enumerate(records):
        rgb = record["rgb"].numpy()
        predicted = record["predicted_flow"].numpy()
        oracle = record["oracle_flow"].numpy()
        valid = record["oracle_valid"].numpy().astype(bool)
        figure, axes = plt.subplots(1, 3, figsize=(9.4, 2.8), constrained_layout=True)
        axes[0].imshow(rgb)
        axes[0].set_title("True camera RGB")
        axes[1].imshow(_flow_rgb(predicted, shared_scale))
        axes[1].set_title("NanoFlowNet")
        axes[2].imshow(_flow_rgb(oracle, shared_scale, valid))
        axes[2].set_title("Simulator ground truth")
        for axis in axes:
            axis.set_xticks([])
            axis.set_yticks([])
        metrics = record["metrics"]
        figure.suptitle(
            f"{record['gate_id']}  range={record['distance_m']:.2f} m  "
            f"lateral={record['lateral_offset_m']:+.2f} m  "
            f"EPE={metrics['mean_epe_px']:.2f} px  scale={shared_scale:.1f} px",
            fontsize=10,
        )
        filename = f"{index:03d}_{record['gate_id']}_d{record['distance_m']:.2f}_lat{record['lateral_offset_m']:+.2f}.png"
        figure.savefig(gallery_dir / filename, dpi=160)
        plt.close(figure)
        files.append(f"flow_gallery/{filename}")

    gate_ids = sorted({str(record["gate_id"]) for record in records})
    contact_files = []
    for gate_id in gate_ids:
        gate_records = [record for record in records if record["gate_id"] == gate_id]
        offsets = sorted({float(record["lateral_offset_m"]) for record in gate_records})
        distances = sorted(
            {float(record["distance_m"]) for record in gate_records}, reverse=True
        )
        figure, axes = plt.subplots(
            len(distances), 3 * len(offsets),
            figsize=(3.1 * 3 * len(offsets), 2.35 * len(distances)),
            squeeze=False,
            constrained_layout=True,
        )
        for row, distance in enumerate(distances):
            for group, offset in enumerate(offsets):
                record = next(
                    item
                    for item in gate_records
                    if float(item["distance_m"]) == distance
                    and float(item["lateral_offset_m"]) == offset
                )
                rgb = record["rgb"].numpy()
                predicted = record["predicted_flow"].numpy()
                oracle = record["oracle_flow"].numpy()
                valid = record["oracle_valid"].numpy().astype(bool)
                views = (
                    (rgb, "RGB"),
                    (_flow_rgb(predicted, shared_scale), "NanoFlow"),
                    (_flow_rgb(oracle, shared_scale, valid), "Ground truth"),
                )
                for subcolumn, (image, label) in enumerate(views):
                    axis = axes[row, 3 * group + subcolumn]
                    axis.imshow(image)
                    axis.set_xticks([])
                    axis.set_yticks([])
                    if row == 0:
                        axis.set_title(f"lat {offset:+.2f} m\n{label}", fontsize=9)
                    if 3 * group + subcolumn == 0:
                        axis.set_ylabel(f"range {distance:.2f} m", fontsize=9)
        figure.suptitle(
            f"IMAV22 {gate_id}: camera vs NanoFlowNet vs true flow\n"
            f"shared color scale {shared_scale:.1f} px per {pair_interval_s:.2f} s pair",
            fontsize=14,
            fontweight="bold",
        )
        filename = f"{gate_id}_nanoflow_contact_sheet.png"
        figure.savefig(output_dir / filename, dpi=150)
        plt.close(figure)
        contact_files.append(filename)

    index_lines = [
        "# IMAV22 gate NanoFlowNet gallery",
        "",
        f"All flow panels use one shared 95th-percentile scale: {shared_scale:.3f} pixels per {pair_interval_s:.3f} s frame pair.",
        "",
    ]
    for contact in contact_files:
        index_lines.append(f"![{contact}]({contact})")
        index_lines.append("")
    index_lines.extend(["## Individual captures", ""])
    for filename, record in zip(files, records, strict=True):
        index_lines.append(
            f"- [{record['gate_id']}, {record['distance_m']:.2f} m, lateral {record['lateral_offset_m']:+.2f} m]({filename})"
        )
    (output_dir / "GALLERY.md").write_text("\n".join(index_lines) + "\n")
    return contact_files


def _capture_gate_flow(
    sim: sim_utils.SimulationContext,
    hm01b0: Camera,
    layout: ArenaLayout,
    output_dir: Path,
) -> dict[str, object]:
    model_path = args.nanoflow_model.expanduser().resolve()
    if not model_path.is_file():
        raise FileNotFoundError(model_path)
    if args.flow_pair_travel_m <= 0.0 or args.flow_pair_interval_s <= 0.0:
        raise ValueError("flow pair travel and interval must be positive")
    if any(value <= args.flow_pair_travel_m for value in args.flow_distances_m):
        raise ValueError("every flow distance must exceed flow-pair travel")
    estimator = LiteRTNanoFlowEstimator(model_path)
    generator = torch.Generator(device=sim.device)
    generator.manual_seed(int(args.seed) + 91_337)
    records: list[dict[str, object]] = []
    gates = [obj for obj in layout.objects if obj.kind == "gate"]
    with torch.inference_mode():
        for gate in gates:
            gate_x, gate_y, _ = gate.center_m
            normal = (math.cos(gate.yaw_rad), math.sin(gate.yaw_rad))
            width_axis = (-normal[1], normal[0])
            for distance in args.flow_distances_m:
                for lateral in args.flow_lateral_offsets_m:
                    current_distance = float(distance)
                    previous_distance = current_distance + float(args.flow_pair_travel_m)
                    eye_previous = (
                        gate_x - previous_distance * normal[0] + lateral * width_axis[0],
                        gate_y - previous_distance * normal[1] + lateral * width_axis[1],
                        0.85,
                    )
                    eye_current = (
                        gate_x - current_distance * normal[0] + lateral * width_axis[0],
                        gate_y - current_distance * normal[1] + lateral * width_axis[1],
                        0.85,
                    )
                    target_previous = (
                        eye_previous[0] + 4.0 * normal[0],
                        eye_previous[1] + 4.0 * normal[1],
                        eye_previous[2],
                    )
                    target_current = (
                        eye_current[0] + 4.0 * normal[0],
                        eye_current[1] + 4.0 * normal[1],
                        eye_current[2],
                    )
                    previous = _render_sensor_sample(
                        sim, hm01b0, eye_previous, target_previous, generator
                    )
                    current = _render_sensor_sample(
                        sim, hm01b0, eye_current, target_current, generator
                    )
                    pair = torch.stack(
                        (previous["sensor_gray"], current["sensor_gray"]), dim=0
                    ).unsqueeze(0)
                    predicted = estimator(pair)[0]
                    oracle, valid = rigid_flow_distorted_torch(
                        previous["depth"].unsqueeze(0),
                        current["depth"].unsqueeze(0),
                        previous["transform"].unsqueeze(0),
                        current["transform"].unsqueeze(0),
                        current["intrinsic"],
                        current["depth"].new_tensor(CAMERA_DISTORTION_RAW_RTX),
                    )
                    predicted_cpu = predicted.detach().cpu()
                    oracle_cpu = oracle[0].detach().cpu()
                    valid_cpu = valid[0].detach().cpu()
                    records.append(
                        {
                            "gate_id": gate.object_id,
                            "distance_m": current_distance,
                            "lateral_offset_m": float(lateral),
                            "previous_eye_xyz_m": eye_previous,
                            "current_eye_xyz_m": eye_current,
                            "rgb": current["rgb"].detach().cpu(),
                            "sensor_gray_previous": previous["sensor_gray"].detach().cpu(),
                            "sensor_gray_current": current["sensor_gray"].detach().cpu(),
                            "predicted_flow": predicted_cpu,
                            "oracle_flow": oracle_cpu,
                            "oracle_valid": valid_cpu,
                            "metrics": _flow_metrics(predicted_cpu, oracle_cpu, valid_cpu),
                        }
                    )
    archive = output_dir / "imav22_gate_nanoflow_captures.pt"
    torch.save(records, archive)
    contact_files = _write_flow_gallery(records, output_dir, args.flow_pair_interval_s)
    metric_keys = (
        "valid_fraction",
        "mean_epe_px",
        "p95_epe_px",
        "prediction_mean_magnitude_px",
        "oracle_mean_magnitude_px",
    )

    def summarize(selected: list[dict[str, object]]) -> dict[str, float]:
        return {
            key: float(np.mean([float(record["metrics"][key]) for record in selected]))
            for key in metric_keys
        }

    metrics_by_distance = {
        f"{float(distance):.2f}": summarize(
            [record for record in records if float(record["distance_m"]) == float(distance)]
        )
        for distance in args.flow_distances_m
    }
    return {
        "format": "imav22-gate-nanoflow-oracle-gallery-v1",
        "archive": str(archive),
        "gallery_index": str(output_dir / "GALLERY.md"),
        "contact_sheets": contact_files,
        "nanoflow_model": str(model_path),
        "nanoflow_model_sha256": _sha256(model_path),
        "capture_count": len(records),
        "gate_count": len(gates),
        "aggregate_metrics": summarize(records),
        "metrics_by_distance_m": metrics_by_distance,
        "distances_m": [float(value) for value in args.flow_distances_m],
        "lateral_offsets_m": [float(value) for value in args.flow_lateral_offsets_m],
        "pair_travel_m": float(args.flow_pair_travel_m),
        "pair_interval_s": float(args.flow_pair_interval_s),
        "equivalent_forward_speed_mps": float(
            args.flow_pair_travel_m / args.flow_pair_interval_s
        ),
        "flow_convention": "previous_to_current_sensor_pixels_per_frame_interval",
        "oracle": "distortion_aware_depth_pose_reprojection_with_occlusion",
        "actor_used": False,
        "tinympc_used": False,
        "trajectory_execution": "predetermined_kinematic_camera_translation_v1",
    }


def _hm01b0_gray(rgb_square: np.ndarray) -> np.ndarray:
    rgb = rgb_square[20:140]
    return np.rint(
        0.299 * rgb[..., 0] + 0.587 * rgb[..., 1] + 0.114 * rgb[..., 2]
    ).clip(0, 255).astype(np.uint8)


def main() -> None:
    if args.steps < 8:
        raise ValueError("arena rendering requires at least eight warm-up steps")
    output_dir = args.output_dir.expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    verify_imav22_assets()
    if args.relocation_event_index < 0:
        raise ValueError("relocation event index must be non-negative")
    if args.reference_layout and args.complexity_mode != STATIC_FULL:
        raise ValueError("the deterministic reference layout is static_full")
    if args.relocation_event_index and args.complexity_mode != RELOCATED_FULL:
        raise ValueError("relocation events require relocated_full complexity mode")
    layout = (
        competition_reference_layout()
        if args.reference_layout
        else sample_randomized_layout(args.seed, complexity_mode=args.complexity_mode)
    )
    for event_index in range(1, args.relocation_event_index + 1):
        layout = relocate_one_obstacle(
            layout,
            event_index=event_index,
            drone_position_m=(args.drone_x, args.drone_y),
            drone_yaw_rad=math.radians(args.drone_yaw_deg),
        )

    sim = sim_utils.SimulationContext(
        sim_utils.SimulationCfg(dt=1.0 / 120.0, device=args.device, use_fabric=True)
    )
    overview, hm01b0 = build_scene(layout)
    sim.reset()
    overview.set_world_poses_from_view(
        torch.tensor([[-7.8, -8.6, 9.2]], device=sim.device),
        torch.tensor([[0.0, 0.0, 0.35]], device=sim.device),
    )
    hm01b0.set_world_poses_from_view(
        torch.tensor([[args.drone_x, args.drone_y, 0.85]], device=sim.device),
        torch.tensor(
            [[
                args.drone_x + 4.15 * math.cos(math.radians(args.drone_yaw_deg)),
                args.drone_y + 4.15 * math.sin(math.radians(args.drone_yaw_deg)),
                0.90,
            ]],
            device=sim.device,
        ),
    )
    for _ in range(args.steps):
        sim.step(render=True)
        overview.update(dt=sim.get_physics_dt())
        hm01b0.update(dt=sim.get_physics_dt())

    overview_rgb = _rgb(overview)
    sensor_rgb_square = _rgb(hm01b0)
    sensor_rgb = sensor_rgb_square[20:140]
    sensor_gray = _hm01b0_gray(sensor_rgb_square)
    Image.fromarray(overview_rgb, mode="RGB").save(output_dir / "arena_overview.png")
    Image.fromarray(sensor_rgb, mode="RGB").save(output_dir / "hm01b0_rgb_160x120.png")
    Image.fromarray(sensor_gray, mode="L").save(output_dir / "hm01b0_gray_160x120.png")
    closeup_files: list[str] = []
    for gate in (obj for obj in layout.objects if obj.kind == "gate"):
        gate_x, gate_y, _ = gate.center_m
        normal = (math.cos(gate.yaw_rad), math.sin(gate.yaw_rad))
        width_axis = (-normal[1], normal[0])
        views = {
            "front": (
                gate_x - 2.0 * normal[0],
                gate_y - 2.0 * normal[1],
                0.92,
            ),
            "oblique": (
                gate_x - 1.65 * normal[0] + 1.05 * width_axis[0],
                gate_y - 1.65 * normal[1] + 1.05 * width_axis[1],
                1.18,
            ),
            "rear": (
                gate_x + 2.0 * normal[0],
                gate_y + 2.0 * normal[1],
                0.92,
            ),
        }
        for view_name, eye in views.items():
            overview.set_world_poses_from_view(
                torch.tensor([eye], device=sim.device),
                torch.tensor([[gate_x, gate_y, 0.66]], device=sim.device),
            )
            for _ in range(8):
                sim.step(render=True)
                overview.update(dt=sim.get_physics_dt())
            filename = f"{gate.object_id}_{view_name}.png"
            Image.fromarray(_rgb(overview), mode="RGB").save(output_dir / filename)
            closeup_files.append(filename)
    panel_closeup_files: list[str] = []
    for panel in (obj for obj in layout.objects if obj.kind == "panel"):
        panel_x, panel_y, panel_z = panel.center_m
        normal = (math.cos(panel.yaw_rad), math.sin(panel.yaw_rad))
        for view_name, sign in (("front", -1.0), ("rear", 1.0)):
            eye = (
                panel_x + sign * 1.40 * normal[0],
                panel_y + sign * 1.40 * normal[1],
                1.10,
            )
            overview.set_world_poses_from_view(
                torch.tensor([eye], device=sim.device),
                torch.tensor([[panel_x, panel_y, panel_z]], device=sim.device),
            )
            for _ in range(8):
                sim.step(render=True)
                overview.update(dt=sim.get_physics_dt())
            filename = f"{panel.object_id}_{view_name}.png"
            Image.fromarray(_rgb(overview), mode="RGB").save(output_dir / filename)
            panel_closeup_files.append(filename)
        hm_eye = (
            panel_x - 1.40 * normal[0],
            panel_y - 1.40 * normal[1],
            0.85,
        )
        hm01b0.set_world_poses_from_view(
            torch.tensor([hm_eye], device=sim.device),
            torch.tensor([[panel_x, panel_y, 1.05]], device=sim.device),
        )
        for _ in range(8):
            sim.step(render=True)
            hm01b0.update(dt=sim.get_physics_dt())
        filename = f"{panel.object_id}_hm01b0_gray_160x120.png"
        Image.fromarray(_hm01b0_gray(_rgb(hm01b0)), mode="L").save(output_dir / filename)
        panel_closeup_files.append(filename)
    flow_capture = None
    if args.nanoflow_model is not None:
        flow_capture = _capture_gate_flow(sim, hm01b0, layout, output_dir)
        (output_dir / "flow_manifest.json").write_text(
            json.dumps(flow_capture, indent=2, sort_keys=True) + "\n"
        )
    (output_dir / "layout_manifest.json").write_text(
        json.dumps(layout.to_manifest(), indent=2) + "\n"
    )
    metrics = {
        "layout_id": layout.layout_id,
        "complexity_mode": layout.complexity_mode,
        "relocation_event_index": layout.relocation_event_index,
        "last_relocated_object_id": layout.last_relocated_object_id,
        "seed": layout.seed,
        "object_count": len(layout.objects),
        "object_counts": {
            kind: sum(obj.kind == kind for obj in layout.objects)
            for kind in ("pole", "panel", "flag", "gate")
        },
        "overview_shape": list(overview_rgb.shape),
        "sensor_shape": list(sensor_gray.shape),
        "sensor_mean": float(sensor_gray.mean()),
        "sensor_standard_deviation": float(sensor_gray.std()),
        "sensor_minimum": int(sensor_gray.min()),
        "sensor_maximum": int(sensor_gray.max()),
        "gate_closeups": closeup_files,
        "panel_closeups": panel_closeup_files,
        "flow_capture": flow_capture,
        "passed": bool(
            overview_rgb.shape == (600, 720, 3)
            and sensor_gray.shape == (120, 160)
            and sensor_gray.std() > 8.0
            and sensor_gray.max() > sensor_gray.min()
            and len(layout.objects) == sum(MODE_OBJECT_COUNTS[layout.complexity_mode].values())
            and (
                flow_capture is None
                or flow_capture["capture_count"]
                == 2 * len(args.flow_distances_m) * len(args.flow_lateral_offsets_m)
            )
        ),
    }
    (output_dir / "validation.json").write_text(json.dumps(metrics, indent=2) + "\n")
    print(json.dumps(metrics, sort_keys=True))
    if not metrics["passed"]:
        raise RuntimeError("IMAV22 arena render failed shape, contrast, or inventory validation")


if __name__ == "__main__":
    try:
        main()
    finally:
        simulation_app.close()
