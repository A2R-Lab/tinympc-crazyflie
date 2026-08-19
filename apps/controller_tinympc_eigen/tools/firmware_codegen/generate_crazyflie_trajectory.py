#!/usr/bin/env python3
"""Generates canonical Crazyflie state-reference trajectories."""

from __future__ import annotations

import math
import re
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, Sequence


REFERENCE_DIM = 13
GRAVITY_M_S2 = 9.81
Vector3 = tuple[float, float, float]
PathFunction = Callable[[float, float], tuple[Vector3, Vector3, Vector3]]


@dataclass(frozen=True)
class TrajectorySettings:
    height_m: float = 0.4
    yaw_deg: float = 0.0
    hover_duration_s: float = 0.5
    straight_length_m: float = 2.0
    straight_duration_s: float = 8.0
    circle_radius_m: float = 0.75
    circle_period_s: float = 15.0
    circle_laps: int = 1
    figure8_x_amplitude_m: float = 0.5
    figure8_y_amplitude_m: float = 1.0
    figure8_period_s: float = 15.0
    figure8_laps: int = 1


@dataclass(frozen=True)
class Trajectory:
    name: str
    sample_rate_hz: int
    duration_s: float
    loops: bool
    tangent_heading: bool
    references: tuple[tuple[float, ...], ...]


def prompt_number(name: str, default: float | int, value_type: type):
    while True:
        try:
            supplied = input(f"{name} [{default}]: ").strip()
        except EOFError:
            supplied = ""
        if not supplied:
            return default
        try:
            return value_type(supplied)
        except ValueError:
            print(f"Please enter a valid {value_type.__name__} or press Enter for {default}.")


def prompt_settings(defaults: TrajectorySettings) -> TrajectorySettings:
    print("Canonical Crazyflie trajectory parameters (press Enter for each default)")
    return TrajectorySettings(
        height_m=prompt_number("flight_height_m", defaults.height_m, float),
        yaw_deg=prompt_number("yaw_deg", defaults.yaw_deg, float),
        hover_duration_s=prompt_number(
            "hover_duration_s", defaults.hover_duration_s, float
        ),
        straight_length_m=prompt_number(
            "straight_length_m", defaults.straight_length_m, float
        ),
        straight_duration_s=prompt_number(
            "straight_duration_s", defaults.straight_duration_s, float
        ),
        circle_radius_m=prompt_number(
            "circle_radius_m", defaults.circle_radius_m, float
        ),
        circle_period_s=prompt_number(
            "circle_period_s", defaults.circle_period_s, float
        ),
        circle_laps=prompt_number("circle_laps", defaults.circle_laps, int),
        figure8_x_amplitude_m=prompt_number(
            "figure8_x_amplitude_m", defaults.figure8_x_amplitude_m, float
        ),
        figure8_y_amplitude_m=prompt_number(
            "figure8_y_amplitude_m", defaults.figure8_y_amplitude_m, float
        ),
        figure8_period_s=prompt_number(
            "figure8_period_s", defaults.figure8_period_s, float
        ),
        figure8_laps=prompt_number("figure8_laps", defaults.figure8_laps, int),
    )


def read_solve_rate(generated_header: Path) -> int:
    if not generated_header.is_file():
        raise FileNotFoundError(f"generated firmware header not found: {generated_header}")
    match = re.search(
        r"^\s*#define\s+TINYMPC_GENERATED_SOLVE_RATE_HZ\s+(\d+)\s*$",
        generated_header.read_text(),
        flags=re.MULTILINE,
    )
    if match is None:
        raise ValueError(
            "TINYMPC_GENERATED_SOLVE_RATE_HZ is missing from "
            f"{generated_header}"
        )
    solve_rate_hz = int(match.group(1))
    if solve_rate_hz <= 0:
        raise ValueError("TINYMPC_GENERATED_SOLVE_RATE_HZ must be positive")
    return solve_rate_hz


def validate_settings(settings: TrajectorySettings) -> None:
    positive_values = {
        "flight_height_m": settings.height_m,
        "hover_duration_s": settings.hover_duration_s,
        "straight_length_m": settings.straight_length_m,
        "straight_duration_s": settings.straight_duration_s,
        "circle_radius_m": settings.circle_radius_m,
        "circle_period_s": settings.circle_period_s,
        "circle_laps": settings.circle_laps,
        "figure8_x_amplitude_m": settings.figure8_x_amplitude_m,
        "figure8_y_amplitude_m": settings.figure8_y_amplitude_m,
        "figure8_period_s": settings.figure8_period_s,
        "figure8_laps": settings.figure8_laps,
    }
    invalid = [name for name, value in positive_values.items() if value <= 0]
    if invalid:
        raise ValueError("these values must be positive: " + ", ".join(invalid))
    if not math.isfinite(settings.yaw_deg):
        raise ValueError("yaw_deg must be finite")


def add(a: Vector3, b: Vector3) -> Vector3:
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def scale(value: float, vector: Vector3) -> Vector3:
    return (value * vector[0], value * vector[1], value * vector[2])


def dot(a: Vector3, b: Vector3) -> float:
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def cross(a: Vector3, b: Vector3) -> Vector3:
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def normalize(vector: Vector3) -> Vector3:
    magnitude = math.sqrt(dot(vector, vector))
    if magnitude <= 1e-9:
        raise ValueError("cannot normalize a zero vector")
    return scale(1.0 / magnitude, vector)


def desired_rotation(acceleration: Vector3, yaw_rad: float) -> tuple[Vector3, ...]:
    body_z = normalize(add(acceleration, (0.0, 0.0, GRAVITY_M_S2)))
    heading = (math.cos(yaw_rad), math.sin(yaw_rad), 0.0)
    body_y = normalize(cross(body_z, heading))
    body_x = cross(body_y, body_z)
    return (
        (body_x[0], body_y[0], body_z[0]),
        (body_x[1], body_y[1], body_z[1]),
        (body_x[2], body_y[2], body_z[2]),
    )


def rotation_to_quaternion(rotation: tuple[Vector3, ...]) -> tuple[float, float, float, float]:
    trace = rotation[0][0] + rotation[1][1] + rotation[2][2]
    if trace > 0.0:
        root = math.sqrt(trace + 1.0) * 2.0
        qw = 0.25 * root
        qx = (rotation[2][1] - rotation[1][2]) / root
        qy = (rotation[0][2] - rotation[2][0]) / root
        qz = (rotation[1][0] - rotation[0][1]) / root
    elif rotation[0][0] > rotation[1][1] and rotation[0][0] > rotation[2][2]:
        root = math.sqrt(1.0 + rotation[0][0] - rotation[1][1] - rotation[2][2]) * 2.0
        qw = (rotation[2][1] - rotation[1][2]) / root
        qx = 0.25 * root
        qy = (rotation[0][1] + rotation[1][0]) / root
        qz = (rotation[0][2] + rotation[2][0]) / root
    elif rotation[1][1] > rotation[2][2]:
        root = math.sqrt(1.0 + rotation[1][1] - rotation[0][0] - rotation[2][2]) * 2.0
        qw = (rotation[0][2] - rotation[2][0]) / root
        qx = (rotation[0][1] + rotation[1][0]) / root
        qy = 0.25 * root
        qz = (rotation[1][2] + rotation[2][1]) / root
    else:
        root = math.sqrt(1.0 + rotation[2][2] - rotation[0][0] - rotation[1][1]) * 2.0
        qw = (rotation[1][0] - rotation[0][1]) / root
        qx = (rotation[0][2] + rotation[2][0]) / root
        qy = (rotation[1][2] + rotation[2][1]) / root
        qz = 0.25 * root
    return (qw, qx, qy, qz)


def transpose_multiply(
    left: tuple[Vector3, ...], right: tuple[Vector3, ...]
) -> tuple[Vector3, ...]:
    return tuple(
        tuple(sum(left[k][row] * right[k][column] for k in range(3)) for column in range(3))
        for row in range(3)
    )


def rotation_derivative(
    rotations: Sequence[tuple[Vector3, ...]], index: int, dt: float
) -> tuple[Vector3, ...]:
    if index == 0:
        before, after, denominator = rotations[0], rotations[1], dt
    elif index == len(rotations) - 1:
        before, after, denominator = rotations[-2], rotations[-1], dt
    else:
        before, after, denominator = rotations[index - 1], rotations[index + 1], 2.0 * dt
    return tuple(
        tuple((after[row][column] - before[row][column]) / denominator for column in range(3))
        for row in range(3)
    )


def body_rate(
    rotation: tuple[Vector3, ...], rotation_dot: tuple[Vector3, ...]
) -> Vector3:
    omega_cross = transpose_multiply(rotation, rotation_dot)
    return (
        0.5 * (omega_cross[2][1] - omega_cross[1][2]),
        0.5 * (omega_cross[0][2] - omega_cross[2][0]),
        0.5 * (omega_cross[1][0] - omega_cross[0][1]),
    )


def hover_path(_: float, __: float) -> tuple[Vector3, Vector3, Vector3]:
    return (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)


def straight_path(length_m: float) -> PathFunction:
    def path(time_s: float, duration_s: float):
        tau = min(max(time_s / duration_s, 0.0), 1.0)
        blend = 10.0 * tau**3 - 15.0 * tau**4 + 6.0 * tau**5
        blend_dot = (30.0 * tau**2 - 60.0 * tau**3 + 30.0 * tau**4) / duration_s
        blend_ddot = (60.0 * tau - 180.0 * tau**2 + 120.0 * tau**3) / duration_s**2
        return (
            (length_m * blend, 0.0, 0.0),
            (length_m * blend_dot, 0.0, 0.0),
            (length_m * blend_ddot, 0.0, 0.0),
        )

    return path


def circle_path(radius_m: float, laps: int) -> PathFunction:
    def path(time_s: float, duration_s: float):
        tau = min(max(time_s / duration_s, 0.0), 1.0)
        blend = 10.0 * tau**3 - 15.0 * tau**4 + 6.0 * tau**5
        blend_dot = (30.0 * tau**2 - 60.0 * tau**3 + 30.0 * tau**4) / duration_s
        blend_ddot = (60.0 * tau - 180.0 * tau**2 + 120.0 * tau**3) / duration_s**2
        angle = 2.0 * math.pi * laps * blend
        angular_rate = 2.0 * math.pi * laps * blend_dot
        angular_acceleration = 2.0 * math.pi * laps * blend_ddot
        return (
            (radius_m * math.sin(angle), radius_m * (1.0 - math.cos(angle)), 0.0),
            (radius_m * angular_rate * math.cos(angle), radius_m * angular_rate * math.sin(angle), 0.0),
            (radius_m * (angular_acceleration * math.cos(angle) - angular_rate**2 * math.sin(angle)),
             radius_m * (angular_acceleration * math.sin(angle) + angular_rate**2 * math.cos(angle)), 0.0),
        )

    return path


def figure8_path(x_amplitude_m: float, y_amplitude_m: float, laps: int) -> PathFunction:
    def path(time_s: float, duration_s: float):
        angular_rate = 2.0 * math.pi * laps / duration_s
        angle = angular_rate * time_s
        return (
            (x_amplitude_m * math.sin(angle), y_amplitude_m * math.sin(2.0 * angle), 0.0),
            (
                x_amplitude_m * angular_rate * math.cos(angle),
                2.0 * y_amplitude_m * angular_rate * math.cos(2.0 * angle),
                0.0,
            ),
            (
                -x_amplitude_m * angular_rate**2 * math.sin(angle),
                -4.0 * y_amplitude_m * angular_rate**2 * math.sin(2.0 * angle),
                0.0,
            ),
        )

    return path


def generate_trajectory(
    name: str,
    sample_rate_hz: int,
    requested_duration_s: float,
    height_m: float,
    yaw_rad: float,
    loops: bool,
    tangent_heading: bool,
    path: PathFunction,
) -> Trajectory:
    motion_duration_s = max(1, round(requested_duration_s * sample_rate_hz)) / sample_rate_hz
    duration_s = motion_duration_s
    interval_count = round(duration_s * sample_rate_hz)
    dt = 1.0 / sample_rate_hz
    samples = []
    for index in range(interval_count + 1):
        time_s = index * dt
        samples.append(path(time_s, motion_duration_s))
    rotations = [
        desired_rotation(
            acceleration,
            math.atan2(velocity[1], velocity[0])
            if tangent_heading and math.hypot(velocity[0], velocity[1]) > 1e-6
            else yaw_rad,
        )
        for _, velocity, acceleration in samples
    ]
    references = []
    for index, ((position, velocity, _), rotation) in enumerate(zip(samples, rotations)):
        position = (position[0], position[1], position[2] + height_m)
        quaternion = rotation_to_quaternion(rotation)
        if references and sum(
            quaternion[axis] * references[-1][axis + 3] for axis in range(4)
        ) < 0.0:
            quaternion = tuple(-value for value in quaternion)
        angular_velocity = body_rate(
            rotation, rotation_derivative(rotations, index, dt)
        )
        references.append(position + quaternion + velocity + angular_velocity)
    return Trajectory(
        name, sample_rate_hz, duration_s, loops, tangent_heading,
        tuple(references)
    )


def validate_trajectory(trajectory: Trajectory) -> None:
    if len(trajectory.references) < 2:
        raise ValueError(f"{trajectory.name}: trajectory needs at least two samples")
    for index, reference in enumerate(trajectory.references):
        if len(reference) != REFERENCE_DIM:
            raise ValueError(
                f"{trajectory.name}: sample {index} does not have "
                f"{REFERENCE_DIM} reference values"
            )
        if not all(math.isfinite(value) for value in reference):
            raise ValueError(f"{trajectory.name}: sample {index} contains a non-finite value")
        quaternion_norm = math.sqrt(sum(reference[axis] ** 2 for axis in range(3, 7)))
        if abs(quaternion_norm - 1.0) > 1e-6:
            raise ValueError(
                f"{trajectory.name}: sample {index} quaternion is not normalized"
            )
    expected_samples = round(trajectory.duration_s * trajectory.sample_rate_hz) + 1
    if len(trajectory.references) != expected_samples:
        raise ValueError(f"{trajectory.name}: duration and sample count disagree")

    dt = 1.0 / trajectory.sample_rate_hz
    maximum_velocity = max(
        math.sqrt(sum(reference[axis] ** 2 for axis in range(7, 10)))
        for reference in trajectory.references
    )
    maximum_error = 0.0
    for index in range(1, len(trajectory.references) - 1):
        for axis in range(3):
            numerical_velocity = (
                trajectory.references[index + 1][axis]
                - trajectory.references[index - 1][axis]
            ) / (2.0 * dt)
            maximum_error = max(
                maximum_error,
                abs(numerical_velocity - trajectory.references[index][axis + 7]),
            )
    tolerance = max(0.05, 0.25 * maximum_velocity)
    if maximum_error > tolerance:
        raise ValueError(
            f"{trajectory.name}: position/velocity mismatch {maximum_error:.3f} m/s"
        )


def float_literal(value: float) -> str:
    if abs(value) < 5e-9:
        value = 0.0
    literal = f"{value:.8g}"
    if "." not in literal and "e" not in literal:
        literal += ".0"
    return literal + "f"


def render_header(trajectory: Trajectory) -> str:
    frequency_name = f"{trajectory.sample_rate_hz}HZ"
    trajectory_name = trajectory.name.upper().replace("-", "_")
    guard = f"TRAJ_{trajectory_name}_{frequency_name}_H"
    rows = [
        "  {" + ", ".join(float_literal(value) for value in state) + "},"
        for state in trajectory.references
    ]
    return "\n".join(
        [
            "/* Autogenerated by generate_crazyflie_trajectory.py. */",
            f"#ifndef {guard}",
            f"#define {guard}",
            "",
            f'#define TRAJECTORY_NAME "{trajectory.name}"',
            f"#define TRAJECTORY_SAMPLE_RATE_HZ {trajectory.sample_rate_hz}",
            f"#define TRAJECTORY_SAMPLE_DT_S ({float_literal(1.0 / trajectory.sample_rate_hz)})",
            f"#define TRAJECTORY_DURATION_S ({float_literal(trajectory.duration_s)})",
            f"#define TRAJECTORY_SAMPLE_COUNT {len(trajectory.references)}",
            f"#define TRAJECTORY_REFERENCE_DIM {REFERENCE_DIM}",
            f"#define TRAJECTORY_LOOPS {1 if trajectory.loops else 0}",
            f"#define TRAJECTORY_TANGENT_HEADING {1 if trajectory.tangent_heading else 0}",
            "",
            "// [p_W(3), q_WB(wxyz), v_W(3), omega_B(3)]",
            "static const float trajectory_reference_data[TRAJECTORY_SAMPLE_COUNT][TRAJECTORY_REFERENCE_DIM] = {",
            *rows,
            "};",
            "",
            f"#endif  // {guard}",
            "",
        ]
    )


def main() -> int:
    app_directory = Path(__file__).resolve().parents[2]
    generated_header = app_directory / "src" / "tinympc_generated_params.h"
    solve_rate_hz = read_solve_rate(generated_header)
    settings = prompt_settings(TrajectorySettings())
    validate_settings(settings)
    yaw_rad = math.radians(settings.yaw_deg)

    specifications = (
        ("hover", settings.hover_duration_s, False, False, hover_path),
        (
            "straight",
            settings.straight_duration_s,
            False,
            False,
            straight_path(settings.straight_length_m),
        ),
        (
            "circle",
            settings.circle_period_s * settings.circle_laps,
            True,
            True,
            circle_path(settings.circle_radius_m, settings.circle_laps),
        ),
        (
            "figure8",
            settings.figure8_period_s * settings.figure8_laps,
            True,
            False,
            figure8_path(
                settings.figure8_x_amplitude_m,
                settings.figure8_y_amplitude_m,
                settings.figure8_laps,
            ),
        ),
    )

    output_directory = app_directory / "src" / "trajectories" / f"{solve_rate_hz}hz"
    output_directory.mkdir(parents=True, exist_ok=True)
    print(f"Using firmware solve rate: {solve_rate_hz} Hz")
    for name, duration_s, loops, tangent_heading, path in specifications:
        trajectory = generate_trajectory(
            name, solve_rate_hz, duration_s, settings.height_m, yaw_rad,
            loops, tangent_heading, path,
        )
        validate_trajectory(trajectory)
        output_path = output_directory / f"traj_{name}_{solve_rate_hz}hz.h"
        output_path.write_text(render_header(trajectory))
        print(
            f"wrote {output_path.relative_to(app_directory)} "
            f"({len(trajectory.references)} samples, {trajectory.duration_s:g} s)"
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
