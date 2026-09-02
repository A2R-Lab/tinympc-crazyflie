"""Canonical offline plant profile for CrazySim ``cf21B_500`` parity.

This profile is a simulation authority only.  It reproduces the parameters
that CrazySim loads from drone-models ``params.toml`` and must not be treated
as identified truth for any physical Crazyflie assembly.
"""

from __future__ import annotations


PROFILE_NAME = "cf21B_500_runtime"
SOURCE_CLASS = "CrazySim runtime simulation authority; not hardware truth"

MASS_KG = 0.04338
INERTIA_DIAGONAL_KGM2 = (25.0e-6, 28.0e-6, 49.0e-6)
ARM_OFFSET_M = 0.035355
BODY_LINEAR_DRAG_DIAGONAL_N_PER_MPS = (
    -0.02149163,
    -0.02149163,
    -0.02359736,
)
RPM_TO_THRUST = (
    0.0,
    -3.133427287299859e-7,
    4.407354891648379e-10,
)
RPM_TO_TORQUE = (
    0.0,
    1.65886356219615e-9,
    2.4693477924534137e-12,
)
ROTOR_DYNAMICS_COEFFICIENT_PER_S = 15.416891997523813
MOTOR_TIME_CONSTANT_S = 1.0 / ROTOR_DYNAMICS_COEFFICIENT_PER_S
MAX_MOTOR_THRUST_N = 0.20
# CrazySim converts the transmitted normalized motor command to requested
# thrust as command^2 * thrust_max before inverting the RPM polynomial.
NORMALIZED_COMMAND_FULL_THRUST_N = MAX_MOTOR_THRUST_N
ROTOR_STATE_SCALE_RPM = 10000.0
PROPELLER_INERTIA_KGM2 = 38.93e-9

# Immutable source identity retained in generated provenance.  The file hashes
# identify the exact local sources audited when this profile was introduced.
SOURCE_REVISIONS = {
    "crazysim_commit": "3ec8b55da4bff887da542a9f314da825460e65be",
    "crazyflie_firmware_commit": "aa6571dc465f06f7d1f9aaf7b0b861fbcd1b3d67",
    "drone_models_commit": "89d8cf79fb722bf4bd7e363ade7d90d6c45d6d5d",
}
SOURCE_SHA256 = {
    "drone_models_params_toml":
        "5f9613e0f4fbf949e351e696ad5d840632405c2224313f1f12c7e1a57dd1e48e",
    "crazysim_py":
        "f5ab66d1a3179a29e4038acf4565f6e3f2809b12d76083c865b023007a5542d5",
}


def is_active_configuration(crazyflie: str, deck: str, propeller_guards: bool) -> bool:
    return crazyflie == "brushless" and deck == "both" and propeller_guards


def provenance() -> dict[str, object]:
    return {
        "profile_name": PROFILE_NAME,
        "source_class": SOURCE_CLASS,
        "mass_kg": MASS_KG,
        "inertia_diagonal_kgm2": list(INERTIA_DIAGONAL_KGM2),
        "arm_offset_m": ARM_OFFSET_M,
        "body_linear_drag_diagonal_n_per_mps":
            list(BODY_LINEAR_DRAG_DIAGONAL_N_PER_MPS),
        "rpm_to_thrust": list(RPM_TO_THRUST),
        "rpm_to_torque": list(RPM_TO_TORQUE),
        "rotor_dynamics_coefficient_per_s": ROTOR_DYNAMICS_COEFFICIENT_PER_S,
        "propeller_inertia_kgm2": PROPELLER_INERTIA_KGM2,
        "motor_time_constant_s": MOTOR_TIME_CONSTANT_S,
        "maximum_motor_thrust_n": MAX_MOTOR_THRUST_N,
        "normalized_command_full_thrust_n":
            NORMALIZED_COMMAND_FULL_THRUST_N,
        "source_revisions": dict(SOURCE_REVISIONS),
        "source_sha256": dict(SOURCE_SHA256),
    }
