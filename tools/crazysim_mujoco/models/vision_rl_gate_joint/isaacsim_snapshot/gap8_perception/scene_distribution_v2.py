"""Validation and deterministic sampling helpers for the frozen scene distribution."""
from __future__ import annotations

import hashlib
import json
from pathlib import Path

import numpy as np


def load_scene_distribution(path: str | Path) -> tuple[dict[str, object], str]:
    path = Path(path)
    record = json.loads(path.read_text())
    if record.get("schema_version") != "horizon-aware-scene-distribution-v2.3-planar":
        raise ValueError("scene distribution schema mismatch")
    if record.get("vertical_avoidance_enabled") is not False:
        raise ValueError("current flow-deck navigation distribution must be planar")
    probabilities = [entry["probability"] for entry in record["room"]["types"].values()]
    if not np.isclose(sum(probabilities), 1.0, atol=1.0e-12):
        raise ValueError("room probabilities must sum to one")
    appearance_probability = record["appearance"]["condition_probability"]
    if len(appearance_probability) != len(record["appearance"]["condition_order"]):
        raise ValueError("appearance conditions and probabilities differ in length")
    if not np.isclose(sum(appearance_probability), 1.0, atol=1.0e-12):
        raise ValueError("appearance probabilities must sum to one")
    if record["obstacles"]["failure_policy"] != (
        "sample_room_specific_count_then_fail_layout_without_silent_count_reduction"
    ):
        raise ValueError("placement failure must not silently alter the distribution")
    room_counts = record["obstacles"].get("mixed_field_count_by_room_inclusive", {})
    if set(room_counts) != set(record["room"]["types"]):
        raise ValueError("every room type must define its mixed-obstacle count range")
    gap_fraction = record["scenario_core"].get("s_turn_gap_fraction_of_room_width")
    if (
        not isinstance(gap_fraction, list) or len(gap_fraction) != 2
        or not 0.0 < float(gap_fraction[0]) < float(gap_fraction[1]) < 0.5
    ):
        raise ValueError("S-turn opening must be a valid room-width fraction below one half")
    turning = record["mission_and_trajectory"]["turning_scenarios"]
    schedule = record["scenario_schedule"]["order"]
    if {"vertical_over", "vertical_under"} & set(schedule):
        raise ValueError("planar schedule may not contain vertical avoidance scenarios")
    if not turning or any(name not in schedule for name in turning):
        raise ValueError("turning scenarios must be represented in the schedule")
    declared_fraction = float(record["mission_and_trajectory"]["turning_scenario_fraction"])
    measured_fraction = sum(name in turning for name in schedule) / len(schedule)
    if not np.isclose(declared_fraction, measured_fraction, atol=1.0e-12):
        raise ValueError("turning scenario fraction differs from the schedule")
    return record, hashlib.sha256(path.read_bytes()).hexdigest()


def sample_room(rng: np.random.Generator, distribution: dict[str, object]) -> tuple[str, tuple[float, float]]:
    room_types = distribution["room"]["types"]
    names = list(room_types)
    probabilities = [room_types[name]["probability"] for name in names]
    name = str(rng.choice(names, p=probabilities))
    parameters = room_types[name]
    return name, (
        float(rng.uniform(*parameters["length_m"])),
        float(rng.uniform(*parameters["width_m"])),
    )


def sample_mixed_obstacle_sizes(
    rng: np.random.Generator, distribution: dict[str, object], count: int,
) -> list[tuple[str, tuple[float, float, float]]]:
    obstacle = distribution["obstacles"]
    ranges = obstacle["mixed_field_count_by_room_inclusive"].values()
    low = min(int(bounds[0]) for bounds in ranges)
    high = max(int(bounds[1]) for bounds in ranges)
    if not int(low) <= count <= int(high):
        raise ValueError("requested obstacle count is outside the frozen distribution")
    result = []
    cycle = obstacle["primitive_cycle"]
    for index in range(count):
        primitive = cycle[index % len(cycle)]
        if primitive == "sphere":
            diameter = float(rng.uniform(*obstacle["sphere_diameter_m"]))
            size = (diameter, diameter, diameter)
        elif primitive in {"cylinder", "cone"}:
            diameter = float(rng.uniform(*obstacle["circular_diameter_m"]))
            size = (diameter, diameter, float(rng.uniform(*obstacle["height_m"])))
        else:
            size = (
                float(rng.uniform(*obstacle["box_xy_m"])),
                float(rng.uniform(*obstacle["box_xy_m"])),
                float(rng.uniform(*obstacle["height_m"])),
            )
        result.append((primitive, size))
    return result


def scenario_for_layout(layout_index: int, distribution: dict[str, object]) -> str:
    order = distribution["scenario_schedule"]["order"]
    return str(order[layout_index % len(order)])
