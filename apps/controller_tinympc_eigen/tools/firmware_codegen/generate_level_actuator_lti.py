#!/usr/bin/env python3
"""Generate the fixed level-flight TinyMPC model with four rotor states.

All nonlinear differentiation and Riccati work happens here.  Firmware only
loads the emitted float arrays and runs the same fixed-horizon ADMM recursion.
"""

from __future__ import annotations

import math
import argparse
from pathlib import Path
import sys

import numpy as np


HERE = Path(__file__).resolve().parent
APP = HERE.parents[1]
PYBULLET_TOOLS = APP / "tools" / "pybullet_simulation"
sys.path.insert(0, str(PYBULLET_TOOLS))
sys.path.insert(0, str(HERE))

from quadrotor_dynamics import (  # noqa: E402
    MOTOR_TIME_CONSTANT_S,
    ROTOR_STATE_SCALE_RPM,
    RPM_TO_THRUST,
    StoredReference,
    _linearize_interval,
)
from tinympc_to_crazyflie_adapter import (  # noqa: E402
    CompileTimeProblem,
    POSITION_STATE_WEIGHT,
    build_upstream_cache,
    vehicle_mass,
)


RIGID_STATE_DIM = 12
MOTOR_STATE_DIM = 4
STATE_DIM = RIGID_STATE_DIM + MOTOR_STATE_DIM
INPUT_DIM = 4
DT_S = 0.02
RHO = 250.0
MAX_MOTOR_THRUST_N = 0.20

# Preserve the ordinary controller's rigid-body cost.  Rotor-state cost makes
# the optimizer account for stored actuator energy without making it chase RPM
# estimation noise more aggressively than attitude/rate error.
BASE_Q_DIAGONAL = np.asarray([
    POSITION_STATE_WEIGHT, POSITION_STATE_WEIGHT, POSITION_STATE_WEIGHT,
    4.0, 4.0, 400.0,
    4.0, 4.0, 4.0,
    20.0, 20.0, 40.0,
    300.0, 300.0, 300.0, 300.0,
])
_HOVER_THRUST_N = vehicle_mass(CompileTimeProblem()) * 9.81 / INPUT_DIM
_HOVER_NORMALIZED_COMMAND = math.sqrt(
    _HOVER_THRUST_N / MAX_MOTOR_THRUST_N)
_THRUST_SLOPE_N_PER_COMMAND = (
    2.0 * MAX_MOTOR_THRUST_N * _HOVER_NORMALIZED_COMMAND)
BASE_R_EIGENVALUE = 100.0 / (_THRUST_SLOPE_N_PER_COMMAND ** 2)
YAW_MOTOR_MODE = np.asarray([-1.0, 1.0, -1.0, 1.0]) / 2.0


def _cost_modes() -> dict[str, tuple[np.ndarray, np.ndarray]]:
    modes: dict[str, tuple[np.ndarray, np.ndarray]] = {}
    for name in ("baseline", "yaw_angle_4x", "yaw_rate_4x", "yaw_diff_r_quarter"):
        q_diagonal = BASE_Q_DIAGONAL.copy()
        if name == "yaw_angle_4x":
            q_diagonal[5] *= 4.0
        if name == "yaw_rate_4x":
            q_diagonal[11] *= 4.0
        r = np.eye(INPUT_DIM) * BASE_R_EIGENVALUE
        if name == "yaw_diff_r_quarter":
            yaw_eigenvalue = BASE_R_EIGENVALUE / 4.0
            r += (yaw_eigenvalue - BASE_R_EIGENVALUE) * np.outer(
                YAW_MOTOR_MODE, YAW_MOTOR_MODE)
        modes[name] = (q_diagonal, r)
    return modes


def _hover_reference() -> StoredReference:
    hover = np.full(INPUT_DIM, vehicle_mass(CompileTimeProblem()) * 9.81 / INPUT_DIM)
    return StoredReference(
        time_s=np.asarray([0.0, DT_S]),
        position_w=np.zeros((2, 3)),
        quaternion_wb=np.tile(np.asarray([1.0, 0.0, 0.0, 0.0]), (2, 1)),
        velocity_w=np.zeros((2, 3)),
        omega_b=np.zeros((2, 3)),
        motor_thrust_n=np.tile(hover, (2, 1)),
    )


def _literal(value: float) -> str:
    rendered = f"{float(value):.9g}"
    if "." not in rendered and "e" not in rendered.lower():
        rendered += ".0"
    return rendered + "f"


def _array(name: str, values: np.ndarray) -> str:
    flat = np.asarray(values, dtype=np.float32).reshape(-1)
    lines = []
    for start in range(0, len(flat), 8):
        lines.append("    " + ", ".join(_literal(v) for v in flat[start:start + 8]) + ",")
    return f"static const float {name}[{len(flat)}] = {{\n" + "\n".join(lines) + "\n};"


def _mode_block(
    name: str,
    a: np.ndarray,
    b: np.ndarray,
    affine: np.ndarray,
    q_diagonal: np.ndarray,
    r: np.ndarray,
) -> tuple[str, float]:
    q = np.diag(q_diagonal)
    problem = CompileTimeProblem(state_dim=STATE_DIM, input_dim=INPUT_DIM)
    cache = build_upstream_cache(problem, a, b, affine, q, r)
    coeff_d2p = cache["Kinf"].T @ r - cache["AmBKt"] @ cache["Pinf"] @ b
    spectral_radius = float(np.max(np.abs(np.linalg.eigvals(a - b @ cache["Kinf"]))))
    if spectral_radius >= 1.0:
        raise RuntimeError(f"generated {name} feedback is unstable: {spectral_radius}")
    yaw_r_eigenvalue = float(YAW_MOTOR_MODE @ r @ YAW_MOTOR_MODE)
    block = "\n\n".join([
        f'#define TINYMPC_LEVEL_COST_MODE_NAME "{name}"',
        f"#define TINYMPC_LEVEL_Q_YAW ({_literal(q_diagonal[5])})",
        f"#define TINYMPC_LEVEL_Q_YAW_RATE ({_literal(q_diagonal[11])})",
        f"#define TINYMPC_LEVEL_R_YAW_EIGENVALUE ({_literal(yaw_r_eigenvalue)})",
        f"#define TINYMPC_LEVEL_CLOSED_LOOP_RADIUS ({_literal(spectral_radius)})",
        _array("tinympc_level_actuator_Q_diagonal", q_diagonal),
        _array("tinympc_level_actuator_R", r),
        _array("tinympc_level_actuator_K", cache["Kinf"]),
        _array("tinympc_level_actuator_P", cache["Pinf"]),
        _array("tinympc_level_actuator_Hinv", cache["Quu_inv"]),
        _array("tinympc_level_actuator_AmBKt", cache["AmBKt"]),
        _array("tinympc_level_actuator_coeff_d2p", coeff_d2p),
        _array("tinympc_level_actuator_APf", cache["APf"]),
        _array("tinympc_level_actuator_BPf", cache["BPf"]),
    ])
    return block, spectral_radius


def generate(output: Path) -> None:
    reference = _hover_reference()
    a, b, affine = _linearize_interval(reference, 0)
    hover_thrust = float(reference.motor_thrust_n[0, 0])
    hover_rotor_state = float(reference.motor_state[0, 0])
    if float(np.max(np.abs(affine))) > 1.0e-8:
        raise RuntimeError("hover linearization has a nonzero affine defect")
    if not 0.0 < hover_thrust < MAX_MOTOR_THRUST_N:
        raise RuntimeError("hover thrust lies outside the physical motor limit")

    body = "\n\n".join([
        "/* Autogenerated by generate_level_actuator_lti.py. */\n"
        "#ifndef TINYMPC_LEVEL_ACTUATOR_LTI_H\n"
        "#define TINYMPC_LEVEL_ACTUATOR_LTI_H\n\n"
        f"#define TINYMPC_LEVEL_ACTUATOR_STATE_DIM {STATE_DIM}\n"
        f"#define TINYMPC_LEVEL_ACTUATOR_DT_S ({_literal(DT_S)})\n"
        f"#define TINYMPC_LEVEL_ACTUATOR_RHO ({_literal(RHO)})\n"
        f"#define TINYMPC_LEVEL_MOTOR_TIME_CONSTANT_S ({_literal(MOTOR_TIME_CONSTANT_S)})\n"
        f"#define TINYMPC_LEVEL_ROTOR_STATE_SCALE_RPM ({_literal(ROTOR_STATE_SCALE_RPM)})\n"
        f"#define TINYMPC_LEVEL_RPM_TO_THRUST_LINEAR ({_literal(RPM_TO_THRUST[1])})\n"
        f"#define TINYMPC_LEVEL_RPM_TO_THRUST_QUADRATIC ({_literal(RPM_TO_THRUST[2])})\n"
        f"#define TINYMPC_LEVEL_HOVER_THRUST_N ({_literal(hover_thrust)})\n"
        f"#define TINYMPC_LEVEL_HOVER_ROTOR_STATE ({_literal(hover_rotor_state)})\n"
        f"#define TINYMPC_LEVEL_MAX_MOTOR_THRUST_N ({_literal(MAX_MOTOR_THRUST_N)})",
        _array("tinympc_level_actuator_A", a),
        _array("tinympc_level_actuator_B", b),
        _array("tinympc_level_actuator_affine", affine),
    ])
    modes = _cost_modes()
    mode_conditions = {
        "yaw_angle_4x": "defined(TINYMPC_LEVEL_COST_YAW_ANGLE_4X)",
        "yaw_rate_4x": "defined(TINYMPC_LEVEL_COST_YAW_RATE_4X)",
        "yaw_diff_r_quarter": "defined(TINYMPC_LEVEL_COST_YAW_DIFF_R_QUARTER)",
    }
    selection = [
        "#if (defined(TINYMPC_LEVEL_COST_YAW_ANGLE_4X) + \\",
        "     defined(TINYMPC_LEVEL_COST_YAW_RATE_4X) + \\",
        "     defined(TINYMPC_LEVEL_COST_YAW_DIFF_R_QUARTER)) > 1",
        '#error "Select at most one TinyMPC level cost variant"',
        "#endif",
    ]
    radii: dict[str, float] = {}
    for index, name in enumerate(("yaw_angle_4x", "yaw_rate_4x", "yaw_diff_r_quarter", "baseline")):
        if name == "baseline":
            selection.append("#else")
        else:
            selection.append(("#if " if index == 0 else "#elif ") + mode_conditions[name])
        q_diagonal, r = modes[name]
        block, radii[name] = _mode_block(name, a, b, affine, q_diagonal, r)
        selection.append(block)
    selection.extend(["#endif", "#endif"])
    body += "\n\n" + "\n".join(selection) + "\n"
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(body)
    print(f"generated {output}")
    for name in modes:
        print(f"{name} closed-loop spectral radius: {radii[name]:.9f}")
    print(f"hover thrust: {hover_thrust:.7f} N/motor")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, default=APP / "src" / "tinympc_level_actuator_lti.h")
    args = parser.parse_args()
    output = args.output
    generate(output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
