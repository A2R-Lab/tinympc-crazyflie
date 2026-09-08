#!/usr/bin/env python3
"""Check the emitted circle's geometry, timing, kinematics and input encoding."""
from pathlib import Path
import sys
import unittest
import subprocess

import numpy as np

APP = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(APP / "tools"))
from generate_circle_reference import array, generate  # noqa: E402


class CircleReferenceTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        source = (APP / "src/traj_circle_small.h").read_text()
        cls.x = array(source, "X_ref_data").reshape(-1, 12)
        cls.u = array(source, "U_ref_data").reshape(-1, 4)

    def test_geometry_and_analytic_velocity(self):
        self.assertEqual(self.x.shape, (501, 12))
        self.assertEqual(self.u.shape, (500, 4))
        omega = 2 * np.pi / 5
        theta = omega * np.arange(501) / 100
        np.testing.assert_allclose(self.x[:, 0], 0.5 * np.cos(theta), atol=1e-8)
        np.testing.assert_allclose(self.x[:, 1], 0.5 * np.sin(theta), atol=1e-8)
        np.testing.assert_allclose(self.x[:, 2], 0.5, atol=1e-8)
        np.testing.assert_allclose(self.x[:, 6], -0.5 * omega * np.sin(theta), atol=1e-8)
        np.testing.assert_allclose(self.x[:, 7], 0.5 * omega * np.cos(theta), atol=1e-8)
        np.testing.assert_allclose(self.x[0], self.x[-1], atol=1e-8)
        # Independent numerical derivative catches the old factor-of-two bug.
        position = self.x[:-1, :3]
        velocity = (np.roll(position, -1, axis=0) - np.roll(position, 1, axis=0)) / 0.02
        np.testing.assert_allclose(velocity, self.x[:-1, 6:9], atol=2e-5)

    def test_attitude_rates_and_feedforward(self):
        # Continuous hover equations: ax=2g*phi_y, ay=-2g*phi_x;
        # omega_body=2*phi_dot at this linearization.
        omega = 2 * np.pi / 5
        acceleration = -omega**2 * self.x[:, :2]
        np.testing.assert_allclose(2 * 9.81 * self.x[:, 4], acceleration[:, 0], atol=1e-7)
        np.testing.assert_allclose(-2 * 9.81 * self.x[:, 3], acceleration[:, 1], atol=1e-7)
        phi = self.x[:-1, 3:6]
        rates = (np.roll(phi, -1, axis=0) - np.roll(phi, 1, axis=0)) / 0.01
        np.testing.assert_allclose(rates, self.x[:-1, 9:12], atol=3e-6)
        expected_x, expected_u, report = generate()
        np.testing.assert_allclose(self.x, expected_x, atol=1e-8)
        np.testing.assert_allclose(self.u, expected_u, atol=1e-10)
        base = np.array(report["legacy_hover_commands"])
        delta = 3.72e-8 * 2900**2 * ((self.u + base)**2 - base**2)
        self.assertLess(np.max(np.abs(delta)), 0.00004)
        np.testing.assert_allclose(delta.sum(axis=1), 0, atol=1e-10)
        self.assertLess(max(report["model_zoh_residual_max_by_state"]), 4e-5)

    def test_header_compiles_at_current_rate(self):
        subprocess.run([
            "c++", "-std=c++17", "-Wall", "-Wextra", "-Werror",
            "-I", str(APP / "src"), "-x", "c++", "-fsyntax-only", "-",
        ], input='''#include "tinympc_generated_params.h"
#include "traj_circle_small.h"
static_assert(CIRCLE_REFERENCE_SAMPLE_RATE_HZ == TINYMPC_GENERATED_SOLVE_RATE_HZ);
static_assert(sizeof(X_ref_data)/sizeof(X_ref_data[0]) == 501);
static_assert(sizeof(U_ref_data)/sizeof(U_ref_data[0]) == 500);
''', text=True, check=True)


if __name__ == "__main__":
    unittest.main()
