#!/usr/bin/env python3
"""Generate the bounded TinyRacer canonical course manifests and MuJoCo scenes."""

from __future__ import annotations

import json
import math
from pathlib import Path
import re
import sys


HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
TRAJECTORIES = ROOT / "apps/controller_tinympc_eigen/src/trajectories/50hz"


def trajectory_points(name: str) -> list[list[float]]:
    path = TRAJECTORIES / f"traj_{name}_50hz.h"
    rows: list[list[float]] = []
    in_data = False
    for line in path.read_text().splitlines():
        if "trajectory_reference_data" in line:
            in_data = True
            continue
        if in_data and line.startswith("};"):
            break
        if in_data:
            values = re.findall(r"[-+]?\d*\.?\d+(?:e[-+]?\d+)?(?=f)", line, re.I)
            if len(values) >= 2:
                rows.append([float(values[0]), float(values[1])])
    if not rows:
        raise RuntimeError(f"no trajectory data in {path}")
    stride = max(1, len(rows) // 40)
    sampled = rows[::stride]
    if sampled[-1] != rows[-1]:
        sampled.append(rows[-1])
    return [[round(x, 4), round(y, 4)] for x, y in sampled]


COURSES = {
    "canonical_corridor": {
        "trajectory": "canonical_corridor",
        "description": "Straight corridor with an offset obstacle",
        "obstacles": [
            {"name": "offset_box", "shape": "box", "center": [1.80, -0.24],
             "half_size": [0.12, 0.27], "height": 0.72},
        ],
        "gates": [],
        "corridor_y": [-1.3, 1.3],
        "pass_point": [5.0, 0.0], "pass_radius_m": 0.45,
        "minimum_dodge_encounters": 1, "maximum_final_cross_track_m": 0.20,
        "required_heading_change_deg": 0.0,
        "duration_s": 32.0,
    },
    "canonical_circle": {
        "trajectory": "canonical_circle",
        "description": "Closed oval with a tangent-heading obstacle encounter",
        # Measured together on the rootless-Docker validation host with the
        # canonical-circle camera scene. Keeping the pair in the manifest
        # avoids silently running the 50 Hz firmware schedule about 15% fast.
        "sitl_timing_profile": {
            "realtime_factor": 0.2,
            "firmware_time_factor": 0.1600,
        },
        "obstacles": [
            {"name": "turn_box", "shape": "box", "center": [-0.50, 1.20],
             "half_size": [0.16, 0.25], "height": 0.76},
        ],
        "gates": [],
        "pass_point": [0.0, 0.0], "pass_radius_m": 0.35,
        "minimum_dodge_encounters": 1, "maximum_final_cross_track_m": 0.20,
        "required_heading_change_deg": 150.0, "duration_s": 32.0,
    },
    "canonical_figure8": {
        "trajectory": "canonical_figure8",
        "description": "Broad-lobed smooth figure-eight with an obstacle on the second lobe",
        "obstacles": [
            {"name": "lower_lobe_box", "shape": "box",
             "center": [0.5742, -0.8280],
             "half_size": [0.16, 0.10], "height": 0.72},
        ],
        "gates": [],
        "pass_point": [0.0, 0.0], "pass_radius_m": 0.35,
        "minimum_dodge_encounters": 1, "maximum_final_cross_track_m": 0.22,
        "required_heading_change_deg": 150.0, "duration_s": 80.0,
    },
    "canonical_chicane": {
        "trajectory": "canonical_chicane",
        "description": "S-chicane with an obstacle between opposing banked turns",
        "obstacles": [
            {"name": "chicane_cylinder", "shape": "circle", "center": [4.50, 4.15],
             "radius": 0.20, "height": 0.76},
        ],
        "gates": [],
        "pass_point": [6.90, 4.263], "pass_radius_m": 0.45,
        "minimum_dodge_encounters": 1, "maximum_final_cross_track_m": 0.25,
        "required_heading_change_deg": 60.0, "duration_s": 26.0,
    },
    "canonical_hairpin": {
        "trajectory": "canonical_hairpin",
        "description": "180-degree hairpin with obstacle avoidance",
        "obstacles": [
            {"name": "apex_cylinder", "shape": "circle", "center": [2.24, 0.718],
             "radius": 0.18, "height": 0.76},
        ],
        "gates": [],
        "pass_point": [0.08, 1.61], "pass_radius_m": 0.38,
        "minimum_dodge_encounters": 1, "maximum_final_cross_track_m": 0.22,
        "required_heading_change_deg": 150.0,
        "duration_s": 60.0,
    },
    "dronet_u": {
        "trajectory": "dronet_u",
        "description": (
            "PULP-DroNet v3 static U-course replica: published 5.0 m by "
            "4.7 m interior footprint and obstacle arrangement; turn "
            "centerline and longitudinal obstacle positions are "
            "diagram-digitized approximations"
        ),
        # Figure 5 gives one-metre obstacle intrusion and alternating walls,
        # but no longitudinal coordinates or thickness.  The x positions are
        # digitized from the vector figure; 0.30 m thickness is disclosed here.
        "obstacles": [
            {"name": "static_1", "shape": "box", "center": [1.50, -0.50],
             "half_size": [0.15, 0.50], "height": 1.20, "center_z": 0.60},
            {"name": "static_2", "shape": "box", "center": [2.70, 0.50],
             "half_size": [0.15, 0.50], "height": 1.20, "center_z": 0.60},
            {"name": "static_3", "shape": "box", "center": [2.90, 2.20],
             "half_size": [0.15, 0.50], "height": 1.20, "center_z": 0.60},
            {"name": "static_4", "shape": "box", "center": [1.30, 3.20],
             "half_size": [0.15, 0.50], "height": 1.20, "center_z": 0.60},
        ],
        "walls": [
            {"name": "s1_outer_wall", "shape": "box", "center": [2.50, -1.05],
             "half_size": [2.50, 0.05], "height": 2.00},
            {"name": "s1_inner_wall", "shape": "box", "center": [1.725, 1.05],
             "half_size": [1.725, 0.05], "height": 2.00},
            {"name": "s3_inner_wall", "shape": "box", "center": [1.725, 1.65],
             "half_size": [1.725, 0.05], "height": 2.00},
            {"name": "s3_outer_wall", "shape": "box", "center": [2.50, 3.75],
             "half_size": [2.50, 0.05], "height": 2.00},
            {"name": "turn_outer_wall", "shape": "box", "center": [5.05, 1.35],
             "half_size": [0.05, 2.40], "height": 2.00},
        ],
        "segments": [
            {"name": "S1", "pass_point": [3.30, 0.0], "pass_radius_m": 0.90},
            {"name": "S2", "pass_point": [3.30, 2.70], "pass_radius_m": 0.90},
            {"name": "S3", "pass_point": [0.0, 2.70], "pass_radius_m": 0.30},
        ],
        "gates": [],
        "pass_point": [0.0, 2.70], "pass_radius_m": 0.30,
        "minimum_dodge_encounters": 4,
        "maximum_final_cross_track_m": 0.30,
        "required_heading_change_deg": 180.0,
        "duration_s": 35.0,
        "published_geometry": {
            "interior_footprint_m": [5.0, 4.7],
            "straight_corridor_width_m": 2.0,
            "corridor_gap_m": 0.7,
            "obstacle_intrusion_m": 1.0,
            "target_altitude_m": 0.5,
            "target_speeds_mps": [0.5, 1.0, 1.5],
            "trials_per_speed": 5,
        },
    },
    "gate_obstacle_poc": {
        "trajectory": "straight_9m",
        "description": (
            "Isolated 12 m corridor proof of concept: a hardware-scale 0.45 m "
            "square gate followed by a floor-mounted side-intruding obstacle "
            "with the same camera-relative top elevation as the avoidance "
            "policy's training family. The gate must be "
            "crossed before the avoidance decision is evaluated."
        ),
        "obstacles": [
            {"name": "post_gate_box", "shape": "box",
             "center": [6.20, 0.50], "half_size": [0.15, 0.50],
             "height": 2.20, "center_z": 1.10},
        ],
        "gates": [
            {"name": "expected_gate", "center": [4.0, 0.30, 1.50],
             "normal": [1.0, 0.0], "opening": [0.45, 0.45]},
        ],
        "corridor_y": [-2.0, 2.0],
        "pass_point": [8.5, 0.0], "pass_radius_m": 0.45,
        "minimum_dodge_encounters": 1, "maximum_final_cross_track_m": 0.30,
        "required_heading_change_deg": 0.0, "duration_s": 24.0,
    },
    "gate_obstacle_poc_obstacle_only": {
        "trajectory": "straight_9m",
        "description": (
            "Matched obstacle-only control for the gate/obstacle proof of "
            "concept: identical corridor and tall side-intruding obstacle geometry, "
            "with the gate removed."
        ),
        "obstacles": [
            {"name": "post_gate_box", "shape": "box",
             "center": [6.20, 0.50], "half_size": [0.15, 0.50],
             "height": 2.20, "center_z": 1.10},
        ],
        "gates": [],
        "corridor_y": [-2.0, 2.0],
        "pass_point": [8.5, 0.0], "pass_radius_m": 0.45,
        "minimum_dodge_encounters": 1, "maximum_final_cross_track_m": 0.30,
        "required_heading_change_deg": 0.0, "duration_s": 24.0,
    },
}


ASSETS = """
    <texture type="skybox" builtin="gradient" rgb1="0.62 0.67 0.70" rgb2="0.24 0.28 0.31" width="512" height="3072"/>
    <texture type="2d" name="groundplane" builtin="checker" mark="edge" rgb1="0.22 0.30 0.38" rgb2="0.11 0.18 0.25" markrgb="0.8 0.8 0.8" width="512" height="512"/>
    <material name="groundplane" texture="groundplane" texuniform="true" texrepeat="3 3" reflectance="0.15"/>
"""


def gate_xml(gate: dict) -> str:
    x, y, z = gate["center"]
    nx, ny = gate["normal"]
    yaw = math.degrees(math.atan2(ny, nx))
    return f"""    <body name="{gate['name']}" pos="{x} {y} {z}" euler="0 0 {yaw:.5f}">
      <geom name="{gate['name']}_visual" type="mesh" mesh="gate_photo_ring" material="gate_photo_material" contype="0" conaffinity="0"/>
      <geom name="{gate['name']}_left" type="box" pos="0 -0.2775 0" size="0.0125 0.0525 0.33" rgba="0 0 0 0" contype="1" conaffinity="1"/>
      <geom name="{gate['name']}_right" type="box" pos="0 0.2775 0" size="0.0125 0.0525 0.33" rgba="0 0 0 0" contype="1" conaffinity="1"/>
      <geom name="{gate['name']}_top" type="box" pos="0 0 0.2775" size="0.0125 0.33 0.0525" rgba="0 0 0 0" contype="1" conaffinity="1"/>
      <geom name="{gate['name']}_bottom" type="box" pos="0 0 -0.2775" size="0.0125 0.33 0.0525" rgba="0 0 0 0" contype="1" conaffinity="1"/>
    </body>"""


def obstacle_xml(obstacle: dict) -> str:
    x, y = obstacle["center"]
    half_height = obstacle.get("height", 0.72) / 2.0
    z = obstacle.get("center_z", 1.5)
    if obstacle["shape"] == "circle":
        return (f'    <geom name="{obstacle["name"]}" type="cylinder" pos="{x} {y} {z}" '
                f'size="{obstacle["radius"]} {half_height}" rgba="0.75 0.23 0.13 1" contype="1" conaffinity="1"/>')
    hx, hy = obstacle["half_size"]
    return (f'    <geom name="{obstacle["name"]}" type="box" pos="{x} {y} {z}" '
            f'size="{hx} {hy} {half_height}" rgba="0.75 0.23 0.13 1" contype="1" conaffinity="1"/>')


def wall_xml(wall: dict) -> str:
    x, y = wall["center"]
    hx, hy = wall["half_size"]
    half_height = wall.get("height", 2.0) / 2.0
    return (f'    <geom name="{wall["name"]}" type="box" pos="{x} {y} {half_height}" '
            f'size="{hx} {hy} {half_height}" rgba="0.72 0.70 0.66 1" '
            'contype="1" conaffinity="1"/>')


def scene_xml(name: str, course: dict) -> str:
    if name in ("gate_obstacle_poc", "gate_obstacle_poc_obstacle_only"):
        return gate_obstacle_poc_scene_xml(course)
    geometry = [wall_xml(wall) for wall in course.get("walls", [])]
    geometry += [obstacle_xml(obstacle) for obstacle in course["obstacles"]]
    if "corridor_y" in course:
        geometry += [
            '    <geom name="wall_left" type="box" pos="2.0 1.35 1.5" size="3.5 0.05 1.5" rgba="0.48 0.47 0.44 1" contype="1" conaffinity="1"/>',
            '    <geom name="wall_right" type="box" pos="2.0 -1.35 1.5" size="3.5 0.05 1.5" rgba="0.48 0.47 0.44 1" contype="1" conaffinity="1"/>',
        ]
    return f"""<mujoco model="TinyRacer {name}">
  <option integrator="RK4" density="1.225" viscosity="1.8e-5" timestep="0.001"/>
  <compiler inertiafromgeom="false" autolimits="true" angle="degree"/>
  <statistic center="2.5 1.5 1.5" extent="7.5"/>
  <visual><headlight diffuse="0.58 0.58 0.58" ambient="0.38 0.38 0.38" specular="0 0 0"/><global azimuth="20" elevation="-25" ellipsoidinertia="true"/></visual>
  <asset>{ASSETS}  </asset>
  <worldbody>
    <light pos="1 0 5" dir="0 0 -1" directional="true"/>
    <geom name="floor" size="0 0 0.05" type="plane" material="groundplane"/>
{chr(10).join(geometry)}
  </worldbody>
</mujoco>
"""


def gate_obstacle_poc_scene_xml(course: dict) -> str:
    """Render the POC with the same 12 m textured corridor/gate assets.

    The gate's invisible colliders deliberately leave exactly the real gate's
    0.45 by 0.45 m opening; the photo mesh is visual-only so image labels and
    physical pass/fail geometry cannot silently disagree.
    """
    gate = course["gates"][0] if course["gates"] else None
    if gate is not None:
        gx, gy, gz = gate["center"]
        gate_body = f'''    <body name="expected_gate" pos="{gx} {gy} {gz}">
      <geom type="mesh" mesh="newbeedrone_gate_top" material="newbeedrone_gate" contype="0" conaffinity="0"/>
      <geom type="mesh" mesh="newbeedrone_gate_bottom" material="newbeedrone_gate" contype="0" conaffinity="0"/>
      <geom type="mesh" mesh="newbeedrone_gate_left" material="newbeedrone_gate" contype="0" conaffinity="0"/>
      <geom type="mesh" mesh="newbeedrone_gate_right" material="newbeedrone_gate" contype="0" conaffinity="0"/>
      <!-- Real NewBeeDrone dimensions: 0.45 m clear opening, 0.555 m rail-center span. -->
      <geom name="expected_gate_left" type="box" pos="0 -0.279 0" size="0.025 0.054 0.225" rgba="0 0 0 0" contype="1" conaffinity="1"/>
      <geom name="expected_gate_right" type="box" pos="0 0.279 0" size="0.025 0.054 0.225" rgba="0 0 0 0" contype="1" conaffinity="1"/>
      <geom name="expected_gate_top" type="box" pos="0 0 0.279" size="0.025 0.333 0.054" rgba="0 0 0 0" contype="1" conaffinity="1"/>
      <geom name="expected_gate_bottom" type="box" pos="0 0 -0.279" size="0.025 0.333 0.054" rgba="0 0 0 0" contype="1" conaffinity="1"/>
    </body>'''
    else:
        gate_body = ""
    obstacle = obstacle_xml(course["obstacles"][0])
    return f'''<mujoco model="TinyRacer gate-obstacle proof of concept">
  <option integrator="RK4" density="1.225" viscosity="1.8e-5" timestep="0.001"/>
  <compiler inertiafromgeom="false" autolimits="true" angle="degree"/>
  <statistic center="6 0 1.5" extent="7"/>
  <visual>
    <headlight diffuse="0.12 0.12 0.12" ambient="0.08 0.08 0.08" specular="0 0 0"/>
    <rgba haze="0.03 0.03 0.03 0" fog="1 1 1 0"/>
    <map fogstart="0" fogend="0"/>
    <global azimuth="135" elevation="-18" ellipsoidinertia="true"/>
  </visual>
  <asset>
    <texture name="corridor_floor_texture" type="2d" builtin="checker"
             rgb1="0.31 0.31 0.30" rgb2="0.23 0.23 0.22" width="512" height="512"/>
    <texture name="corridor_wall_texture" type="2d" file="textures/off_white_painted_plaster.png"/>
    <material name="corridor_floor" texture="corridor_floor_texture" texrepeat="12 4"
              texuniform="true" reflectance="0.12" specular="0.18" shininess="0.25"/>
    <material name="corridor_wall" texture="corridor_wall_texture" texrepeat="6 2"
              texuniform="true" rgba="1 1 1 1" specular="0.08" shininess="0.12"/>
    <material name="corridor_ceiling" rgba="0.82 0.82 0.79 1" specular="0.04" shininess="0.08"/>
    <material name="strip_light" rgba="1 0.98 0.86 1" emission="1" specular="0" shininess="0"/>
    <texture type="2d" name="newbeedrone_gate_texture" file="textures/newbeedrone_gate_front_rgba_v1.png"/>
    <material name="newbeedrone_gate" texture="newbeedrone_gate_texture" texuniform="false"
              rgba="1 1 1 1" specular="0.08" shininess="0.12"/>
    <mesh name="newbeedrone_gate_top" file="meshes/newbeedrone_gate_top.obj" scale="1 0.9 0.9"/>
    <mesh name="newbeedrone_gate_bottom" file="meshes/newbeedrone_gate_bottom.obj" scale="1 0.9 0.9"/>
    <mesh name="newbeedrone_gate_left" file="meshes/newbeedrone_gate_left.obj" scale="1 0.9 0.9"/>
    <mesh name="newbeedrone_gate_right" file="meshes/newbeedrone_gate_right.obj" scale="1 0.9 0.9"/>
  </asset>
  <worldbody>
    <camera name="crazyflie_corridor_preview" pos="0 0 1.5" xyaxes="0 -1 0 0 0 1" fovy="47.168554"/>
    <geom name="corridor_floor" type="box" pos="6 0 -0.05" size="6 2.05 0.05"
          material="corridor_floor" contype="1" conaffinity="1"/>
    <geom name="corridor_left_wall" type="box" pos="6 2.05 1.5" size="6 1.5 0.05"
          euler="90 0 0" material="corridor_wall" contype="1" conaffinity="1"/>
    <geom name="corridor_right_wall" type="box" pos="6 -2.05 1.5" size="6 1.5 0.05"
          euler="90 0 0" material="corridor_wall" contype="1" conaffinity="1"/>
    <geom name="corridor_end_wall" type="box" pos="12.05 0 1.5" size="1.5 2.05 0.05"
          euler="0 90 0" material="corridor_wall" contype="1" conaffinity="1"/>
    <geom name="corridor_ceiling" type="box" pos="6 0 3.05" size="6 2.05 0.05"
          material="corridor_ceiling" contype="1" conaffinity="1"/>
{gate_body}
{obstacle}
    <geom name="ceiling_strip_light" type="box" pos="6 0 2.985" size="5.5 0.055 0.015"
          material="strip_light" contype="0" conaffinity="0"/>
    <light name="strip_light_1" pos="1 0 2.94" dir="0 0 -1" diffuse="0.85 0.82 0.72" specular="0.12 0.12 0.10" cutoff="75" exponent="1" attenuation="0.25 0.08 0.02" castshadow="true"/>
    <light name="strip_light_2" pos="3.5 0 2.94" dir="0 0 -1" diffuse="0.85 0.82 0.72" specular="0.12 0.12 0.10" cutoff="75" exponent="1" attenuation="0.25 0.08 0.02" castshadow="true"/>
    <light name="strip_light_3" pos="6 0 2.94" dir="0 0 -1" diffuse="0.85 0.82 0.72" specular="0.12 0.12 0.10" cutoff="75" exponent="1" attenuation="0.25 0.08 0.02" castshadow="true"/>
    <light name="strip_light_4" pos="8.5 0 2.94" dir="0 0 -1" diffuse="0.85 0.82 0.72" specular="0.12 0.12 0.10" cutoff="75" exponent="1" attenuation="0.25 0.08 0.02" castshadow="true"/>
    <light name="strip_light_5" pos="11 0 2.94" dir="0 0 -1" diffuse="0.85 0.82 0.72" specular="0.12 0.12 0.10" cutoff="75" exponent="1" attenuation="0.25 0.08 0.02" castshadow="true"/>
  </worldbody>
</mujoco>
'''


def main() -> None:
    selected = sys.argv[1:]
    unknown = sorted(set(selected) - set(COURSES))
    if unknown:
        raise SystemExit(f"unknown course(s): {', '.join(unknown)}")
    names = selected or COURSES.keys()
    course_dir = HERE / "courses"
    scene_dir = HERE / "scenes"
    for name in names:
        fields = COURSES[name]
        course = {
            "format": "tinympc-crazysim-course-v2",
            "name": name,
            "scene": f"vision_{name}.xml",
            **fields,
            "centerline": trajectory_points(fields["trajectory"]),
            "gate_order_required": bool(fields["gates"]),
        }
        (course_dir / f"{name}.json").write_text(json.dumps(course, indent=2) + "\n")
        (scene_dir / f"vision_{name}.xml").write_text(scene_xml(name, course))
        print(f"generated {name}: {len(course['centerline'])} centerline points")


if __name__ == "__main__":
    main()
