#!/usr/bin/env python3
"""Create the balanced physical obstacle-validation manifest.

The positive matrix covers every distance/orientation/lateral-offset triple.
The remaining scene factors are assigned cyclically so each factor is exercised
without requiring their prohibitively large full Cartesian product.
"""

import argparse
import json
from pathlib import Path


DISTANCES = (0.5, 0.75, 1.0, 1.5, 2.0, 2.5)
ORIENTATIONS = (-45, -20, 0, 20, 45)
OFFSETS = (-0.3, 0.0, 0.3)
SHAPES = ("planar", "cylindrical")
WIDTHS = ("narrow", "broad")
DEPTHS = ("single", "foreground_background")
TEXTURES = ("high", "medium", "repeated", "low")
LIGHTING = ("nominal", "dim", "bright")
MOTIONS = ("translation", "translation_yaw")


def positive_cases(configuration):
    result = []
    index = 0
    for distance in DISTANCES:
        for orientation in ORIENTATIONS:
            for offset in OFFSETS:
                shape = SHAPES[index % len(SHAPES)]
                width = WIDTHS[(index // 2) % len(WIDTHS)]
                depth = DEPTHS[(index // 3) % len(DEPTHS)]
                texture = TEXTURES[(index // 5) % len(TEXTURES)]
                lighting = LIGHTING[(index // 7) % len(LIGHTING)]
                motion = MOTIONS[(index // 11) % len(MOTIONS)]
                result.append({
                    "id": f"f{configuration}-p-{index + 1:03d}",
                    "configuration_features": configuration,
                    "label": "positive",
                    "distance_m": distance,
                    "orientation_deg": orientation,
                    "lateral_offset_m": offset,
                    "obstacle_shape": shape,
                    "obstacle_width": width,
                    "depth_configuration": depth,
                    "texture": texture,
                    "lighting": lighting,
                    "motion": motion,
                    "repetitions": 1,
                })
                index += 1
    return result


def negative_cases(configuration):
    result = []
    index = 0
    for motion in ("stationary", "translation", "translation_yaw", "pure_yaw"):
        for texture in TEXTURES:
            for lighting in LIGHTING:
                result.append({
                    "id": f"f{configuration}-n-{index + 1:03d}",
                    "configuration_features": configuration,
                    "label": "negative",
                    "distance_m": None,
                    "orientation_deg": None,
                    "lateral_offset_m": None,
                    "obstacle_shape": "none",
                    "obstacle_width": "none",
                    "depth_configuration": "stationary_scene",
                    "texture": texture,
                    "lighting": lighting,
                    "motion": motion,
                    "repetitions": 1,
                })
                index += 1
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--configurations", type=int, nargs="+", choices=(27, 36),
                        default=(27, 36))
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()
    cases = []
    for configuration in args.configurations:
        cases.extend(positive_cases(configuration))
        cases.extend(negative_cases(configuration))
    manifest = {
        "schema_version": 1,
        "design": "full distance x orientation x offset; balanced cyclic assignment of other factors",
        "acceptance": {
            "minimum_positive_detection_rate": 0.95,
            "maximum_world_position_error_m": 0.35,
            "negative_false_cylinders_allowed": 0,
        },
        "cases": cases,
    }
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(manifest, indent=2) + "\n")
    positives = sum(case["label"] == "positive" for case in cases)
    negatives = len(cases) - positives
    print(f"wrote {args.out}: {positives} positive, {negatives} negative")


if __name__ == "__main__":
    main()
