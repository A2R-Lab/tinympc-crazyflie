#!/usr/bin/env python3
"""Identity and dynamics checks for the canonical CrazySim runtime profile."""

import importlib
from pathlib import Path
import sys
import unittest

import numpy as np


APP = Path(__file__).resolve().parents[1]
TOOLS = APP / "tools"
sys.path.insert(0, str(TOOLS))
sys.path.insert(0, str(TOOLS / "firmware_codegen"))
sys.path.insert(0, str(TOOLS / "pybullet_simulation"))

profile = importlib.import_module("crazysim_runtime_profile")
adapter = importlib.import_module("tinympc_to_crazyflie_adapter")
dynamics = importlib.import_module("quadrotor_dynamics")


class CrazySimRuntimeProfileTest(unittest.TestCase):
    def test_active_consumers_share_the_canonical_profile(self):
        problem = adapter.CompileTimeProblem()
        self.assertEqual(adapter.vehicle_mass(problem), profile.MASS_KG)
        self.assertEqual(dynamics.MASS_KG, profile.MASS_KG)
        np.testing.assert_array_equal(
            np.diag(dynamics.INERTIA_KGM2), profile.INERTIA_DIAGONAL_KGM2)
        self.assertEqual(dynamics.ARM_OFFSET_M, profile.ARM_OFFSET_M)
        np.testing.assert_array_equal(
            np.diag(dynamics.BODY_LINEAR_DRAG_N_PER_MPS),
            profile.BODY_LINEAR_DRAG_DIAGONAL_N_PER_MPS)
        np.testing.assert_array_equal(dynamics.RPM_TO_THRUST, profile.RPM_TO_THRUST)
        np.testing.assert_array_equal(dynamics.RPM_TO_TORQUE, profile.RPM_TO_TORQUE)
        self.assertEqual(dynamics.MOTOR_TIME_CONSTANT_S, profile.MOTOR_TIME_CONSTANT_S)
        self.assertEqual(
            profile.NORMALIZED_COMMAND_FULL_THRUST_N,
            profile.MAX_MOTOR_THRUST_N)
        _, _, hover = adapter.build_input_bounds_and_reference(problem)
        _, input_cost = adapter.build_cost_matrices(problem)
        thrust_slope = 2.0 * np.sqrt(
            profile.NORMALIZED_COMMAND_FULL_THRUST_N * hover[0])
        np.testing.assert_allclose(
            np.diag(input_cost), 100.0 / thrust_slope**2,
            rtol=0.0, atol=1.0e-10)
        generated_header = (
            APP / "src" / "tinympc_generated_params.h").read_text()
        self.assertIn(
            "sqrtf(thrust_newtons / 0.2f)", generated_header)
        self.assertIn(
            "return 0.2f * command * command", generated_header)
        self.assertIn("not hardware truth", profile.SOURCE_CLASS)

    def test_both_nonlinear_models_apply_identical_level_body_drag(self):
        velocity = 1.0
        expected_ax = profile.BODY_LINEAR_DRAG_DIAGONAL_N_PER_MPS[0] / profile.MASS_KG
        rigid_state = np.zeros(12)
        rigid_state[6] = velocity
        hover = np.full(4, profile.MASS_KG * 9.81 / 4.0)
        adapter_dx = np.asarray(
            adapter.build_continuous_model(adapter.CompileTimeProblem())(
                rigid_state, hover), dtype=float)
        self.assertAlmostEqual(adapter_dx[6], expected_ax, places=12)
        self.assertAlmostEqual(adapter_dx[8], 0.0, places=12)

        absolute = np.zeros(17)
        absolute[3] = 1.0
        absolute[7] = velocity
        absolute[13:17] = dynamics._thrust_to_rpm(hover) / dynamics.ROTOR_STATE_SCALE_RPM
        dynamics_dx = dynamics._absolute_derivative(absolute, hover)
        self.assertAlmostEqual(dynamics_dx[7], expected_ax, places=12)
        self.assertAlmostEqual(dynamics_dx[9], 0.0, places=12)


if __name__ == "__main__":
    unittest.main()
