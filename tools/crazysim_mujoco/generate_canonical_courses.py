#!/usr/bin/env python3
"""Generate the bounded TinyRacer canonical course manifests and MuJoCo scenes."""

from __future__ import annotations

import json
import math
from pathlib import Path
import re


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
        "description": "Straight corridor, offset obstacle, then a forward barrel roll",
        "obstacles": [
            {"name": "offset_box", "shape": "box", "center": [1.80, -0.24],
             "half_size": [0.12, 0.27], "height": 0.72},
        ],
        "gates": [],
        "corridor_y": [-1.3, 1.3],
        "pass_point": [5.0, 0.0], "pass_radius_m": 0.45,
        "minimum_dodge_encounters": 1, "maximum_final_cross_track_m": 0.20,
        "required_heading_change_deg": 0.0,
        "acro_maneuver": "barrel_roll_forward_360", "acro_axis": "roll",
        "acro_trigger_index": 680, "minimum_acro_rotation_deg": 315.0,
        "duration_s": 32.0,
    },
    "canonical_circle": {
        "trajectory": "canonical_circle",
        "description": "Closed oval with a tangent-heading obstacle encounter",
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
        "description": "Figure-eight with an obstacle on the second lobe",
        "obstacles": [
            {"name": "lower_lobe_box", "shape": "box",
             "center": [0.2599, -0.4491],
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
        "description": "180-degree hairpin with avoidance followed by a backflip",
        "obstacles": [
            {"name": "apex_cylinder", "shape": "circle", "center": [2.24, 0.718],
             "radius": 0.18, "height": 0.76},
        ],
        "gates": [],
        "pass_point": [0.08, 1.61], "pass_radius_m": 0.38,
        "minimum_dodge_encounters": 1, "maximum_final_cross_track_m": 0.22,
        "required_heading_change_deg": 150.0,
        "acro_maneuver": "backflip_360", "acro_axis": "pitch",
        "acro_trigger_index": 738, "minimum_acro_rotation_deg": 315.0,
        "duration_s": 60.0,
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
    z = 1.5
    half_height = obstacle.get("height", 0.72) / 2.0
    if obstacle["shape"] == "circle":
        return (f'    <geom name="{obstacle["name"]}" type="cylinder" pos="{x} {y} {z}" '
                f'size="{obstacle["radius"]} {half_height}" rgba="0.75 0.23 0.13 1" contype="1" conaffinity="1"/>')
    hx, hy = obstacle["half_size"]
    return (f'    <geom name="{obstacle["name"]}" type="box" pos="{x} {y} {z}" '
            f'size="{hx} {hy} {half_height}" rgba="0.75 0.23 0.13 1" contype="1" conaffinity="1"/>')


def scene_xml(name: str, course: dict) -> str:
    geometry = [obstacle_xml(obstacle) for obstacle in course["obstacles"]]
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


def main() -> None:
    course_dir = HERE / "courses"
    scene_dir = HERE / "scenes"
    for name, fields in COURSES.items():
        course = {
            "format": "tinympc-crazysim-course-v2",
            "name": name,
            "scene": f"vision_{name}.xml",
            **fields,
            "centerline": trajectory_points(fields["trajectory"]),
            "gate_order_required": False,
        }
        (course_dir / f"{name}.json").write_text(json.dumps(course, indent=2) + "\n")
        (scene_dir / f"vision_{name}.xml").write_text(scene_xml(name, course))
        print(f"generated {name}: {len(course['centerline'])} centerline points")


if __name__ == "__main__":
    main()
