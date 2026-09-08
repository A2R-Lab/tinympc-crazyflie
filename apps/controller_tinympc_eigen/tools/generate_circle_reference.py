#!/usr/bin/env python3
"""Generate a periodic circle for the legacy 12-state hover-MPC interface.

Attitude, body-rate and per-motor feedforward references solve the current
linearized hover dynamics. This is a steady periodic reference, not a
takeoff/entry/stop maneuver or a nonlinear flight-validation result.
"""
import argparse
import hashlib
import json
from pathlib import Path
import re

import numpy as np
from scipy.linalg import expm, logm

APP = Path(__file__).resolve().parents[1]


def array(source, name):
    match = re.search(r"\b" + name + r"(?:\[[^]]+\])+\s*=\s*\{(.*?)\};", source, re.S)
    if not match:
        raise ValueError(f"Missing array {name}")
    values = match[1].replace("{", "").replace("}", "")
    return np.array([float(x.strip().removesuffix("f"))
                     for x in values.split(",") if x.strip()])


def generate(radius=0.5, altitude=0.5, duration=5.0):
    if not all(np.isfinite(x) and x > 0 for x in (radius, altitude, duration)):
        raise ValueError("Radius, altitude and duration must be finite and positive")
    header = APP / "src/tinympc_generated_params.h"
    source = header.read_text()
    dt = float(re.search(r"MODEL_DT_S \(\(float\)([^)]+)\)", source)[1])
    rate = int(re.search(r"SOLVE_RATE_HZ (\d+)", source)[1])
    intervals = round(duration * rate)
    if intervals < 4 or abs(intervals / rate - duration) > 1e-9:
        raise ValueError("Duration must contain an integral number of controller samples")
    a = array(source, "tinympc_generated_A").reshape(12, 12)
    b = array(source, "tinympc_generated_B").reshape(12, 4)
    f = array(source, "tinympc_generated_f")
    if np.max(np.abs(f)) > 1e-12:
        raise ValueError("This generator requires zero affine drift about hover")
    # Recover the continuous model from its existing zero-order-hold matrices.
    # This uses the checked-in model, without rebuilding or retuning its caches.
    augmented = np.block([[a, b], [np.zeros((4, 12)), np.eye(4)]])
    continuous = logm(augmented) / dt
    if np.max(np.abs(continuous.imag)) > 1e-8:
        raise ValueError("Model has no supported real continuous reconstruction")
    continuous = continuous.real
    ac, bc = continuous[:12, :12], continuous[:12, 12:]
    reconstruction_error = float(np.max(np.abs(expm(continuous * dt) - augmented)))
    omega = 2 * np.pi / duration
    # x=R*cos(wt), y=R*sin(wt); world yaw (Rodrigues z) stays zero.
    selector = np.eye(12)[[0, 1, 2, 5]]
    system = np.block([[1j * omega * np.eye(12) - ac, -bc],
                       [selector, np.zeros((4, 4))]])
    solution = np.linalg.solve(system, np.r_[np.zeros(12), radius, -1j * radius, 0, 0])
    condition = float(np.linalg.cond(system))
    if condition > 1e10:
        raise ValueError("Harmonic reference equations are poorly conditioned")
    xamp, uamp = solution[:12], solution[12:]
    time = np.arange(intervals + 1) / rate
    phase = np.exp(1j * omega * time)
    states = np.real(phase[:, None] * xamp)
    states[:, 2] += altitude
    thrust_delta = np.real(phase[:-1, None] * uamp)
    derivative = np.real(phase[:, None] * (1j * omega * xamp))
    full_input = np.real(phase[:, None] * uamp)
    dynamics_error = float(np.max(np.abs(derivative - states @ ac.T - full_input @ bc.T)))
    velocity_error = float(np.max(np.abs(derivative[:, :3] - states[:, 6:9])))
    if dynamics_error > 1e-7 or velocity_error > 1e-7 or reconstruction_error > 1e-8:
        raise ValueError("Circle is inconsistent with the current model's kinematics")
    # Current controller converts legacy command offsets into Newton deviations.
    # Invert that exact convention; U_ref_data must not contain raw Newtons.
    controller = (APP / "src/controller_tinympc.cpp").read_text()
    legacy_hover = array(controller, "legacy_hover_command")
    if "3.72e-8f" not in source or "2900.0f" not in source:
        raise ValueError("Unsupported thrust mapping: expected original quadratic model")
    coefficient = 3.72e-8 * 2900.0**2
    equivalent_force = coefficient * legacy_hover**2 + thrust_delta
    lower = array(source, "tinympc_generated_input_lower").reshape(4, -1)[:, 0]
    upper = array(source, "tinympc_generated_input_upper").reshape(4, -1)[:, 0]
    if np.any(thrust_delta < lower) or np.any(thrust_delta > upper):
        raise ValueError("Motor feedforward exceeds generated thrust bounds")
    if np.any(equivalent_force < 0) or np.any(equivalent_force > coefficient):
        raise ValueError("Legacy input conversion would clip the feedforward")
    inputs = np.sqrt(equivalent_force / coefficient) - legacy_hover
    # Sinusoidal inputs vary continuously; the discrete model holds one input
    # over 20 ms. Quantify this approximation instead of claiming exact ZOH fit.
    shifted = np.real(np.exp(1j * omega * (time + dt))[:, None] * xamp)
    shifted[:, 2] += altitude
    zoh_residual = shifted - states @ a.T - full_input @ b.T
    report = {
        "radius_m": radius, "altitude_m": altitude, "duration_s": duration,
        "sample_rate_hz": rate, "state_samples": intervals + 1,
        "input_samples": intervals, "speed_mps": radius * omega,
        "centripetal_acceleration_mps2": radius * omega**2,
        "direction": "counterclockwise viewed from above", "yaw_rad": 0,
        "center_world_xy_m": [0, 0], "model_dt_s": dt,
        "model_header_sha256": hashlib.sha256(header.read_bytes()).hexdigest(),
        "legacy_hover_commands": legacy_hover.tolist(),
        "continuous_dynamics_max_residual": dynamics_error,
        "position_derivative_velocity_max_error": velocity_error,
        "discrete_model_reconstruction_max_error": reconstruction_error,
        "harmonic_system_condition_number": condition,
        "model_zoh_residual_max_by_state": np.max(np.abs(zoh_residual), axis=0).tolist(),
        "max_abs_motor_thrust_delta_n": float(np.max(np.abs(thrust_delta))),
        "reference_model": "linearized hover, fixed world yaw",
        "endpoint": "duplicate periodic start; nonzero velocity; no entry or stop ramp",
    }
    return states, inputs, report


def c_array(name, values):
    def scalar(value):
        text = format(0.0 if abs(value) < 1e-12 else value, ".9g")
        if "." not in text and "e" not in text:
            text += ".0"
        return text + "f"
    rows = ["  {" + ", ".join(map(scalar, row)) + "}," for row in values]
    return f"static const float {name}[{len(values)}][{values.shape[1]}] = {{\n" + "\n".join(rows) + "\n};\n"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--radius", type=float, default=0.5)
    parser.add_argument("--altitude", type=float, default=0.5)
    parser.add_argument("--duration", type=float, default=5.0)
    parser.add_argument("--out", type=Path, default=APP / "src/traj_circle_small.h")
    args = parser.parse_args()
    states, inputs, report = generate(args.radius, args.altitude, args.duration)
    text = (
        "// Autogenerated by tools/generate_circle_reference.py.\n"
        f"// Circle: radius {args.radius:g} m, z={args.altitude:g} m, period {args.duration:g} s.\n"
        f"// {report['sample_rate_hz']} Hz; endpoint duplicates start; fixed world yaw=0.\n"
        "// State: [position_W(3), Rodrigues_WB(3), velocity_W(3), body_rate(3)].\n"
        "// Input: legacy normalized-command offsets, converted to thrust deltas by controller.\n"
        "// Linearized-hover feedforward; no takeoff, entry, stop or repeat logic.\n"
        "#pragma once\n\n"
        f"#define CIRCLE_REFERENCE_SAMPLE_RATE_HZ {report['sample_rate_hz']}\n"
        f"#define CIRCLE_REFERENCE_DURATION_S {args.duration:.9f}f\n\n"
        + c_array("X_ref_data", states) + "\n" + c_array("U_ref_data", inputs)
    )
    args.out.write_text(text)
    args.out.with_suffix(".provenance.json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
