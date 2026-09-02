#!/usr/bin/env python3
"""Regression tests for the banked affine-model firmware recursion."""

import importlib.util
import math
from pathlib import Path
import sys
import unittest

import numpy as np


GENERATOR = (Path(__file__).resolve().parents[1] / "tools" /
             "firmware_codegen" / "generate_banked_model_bank.py")
SPEC = importlib.util.spec_from_file_location(
    "generate_banked_model_bank", GENERATOR)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)

HORIZON_KNOTS = 20


def affine_reference(bundle, initial_state=None):
    """Construct a dynamically exact reference in the bundle coordinates."""
    states = np.zeros((HORIZON_KNOTS, MODULE.STATE_DIM))
    inputs = np.tile(
        np.asarray(bundle["nominal_input"]), (HORIZON_KNOTS - 1, 1))
    states[0] = (np.asarray(bundle["nominal_state"])
                 if initial_state is None else np.asarray(initial_state))
    for knot in range(HORIZON_KNOTS - 1):
        states[knot + 1] = (
            np.asarray(bundle["affine"])
            + np.asarray(bundle["A"]) @ states[knot]
            + np.asarray(bundle["B"]) @ inputs[knot]
        )
    return states, inputs


def firmware_primal_update(
    bundle, state_reference, input_reference, coefficient,
    *, add_terminal_admm_term,
):
    """Run one firmware backward/forward pass at an exact ADMM fixed point."""
    a = np.asarray(bundle["A"])
    b = np.asarray(bundle["B"])
    affine = np.asarray(bundle["affine"])
    q_diagonal = np.asarray(bundle["Q_diagonal"])
    r = np.asarray(bundle["R"])
    cache = bundle["cache"]
    gain = np.asarray(cache["Kinf"])
    terminal_cost = np.asarray(cache["Pinf"])
    quu_inverse = np.asarray(cache["Quu_inv"])
    closed_loop_transpose = np.asarray(cache["AmBKt"])
    apf = np.asarray(cache["APf"])
    bpf = np.asarray(cache["BPf"])
    rho = float(MODULE.level.RHO)

    costate = np.zeros_like(state_reference)
    feedforward = np.zeros_like(input_reference)
    predicted_state = np.zeros_like(state_reference)
    predicted_input = np.zeros_like(input_reference)

    costate[-1] = -terminal_cost @ state_reference[-1]
    if add_terminal_admm_term:
        costate[-1] -= rho * state_reference[-1]

    for knot in range(HORIZON_KNOTS - 2, -1, -1):
        # At a fixed point Z == reference and Y == 0.
        input_linear_cost = -(r + rho * np.eye(MODULE.INPUT_DIM)) @ input_reference[knot]
        rhs = input_linear_cost + bpf + b.T @ costate[knot + 1]
        feedforward[knot] = quu_inverse @ rhs
        costate[knot] = (
            -(q_diagonal + rho) * state_reference[knot]
            + apf
            + closed_loop_transpose @ costate[knot + 1]
            - gain.T @ input_linear_cost
            + coefficient @ feedforward[knot]
        )

    predicted_state[0] = state_reference[0]
    for knot in range(HORIZON_KNOTS - 1):
        predicted_input[knot] = (
            -feedforward[knot] - gain @ predicted_state[knot])
        predicted_state[knot + 1] = (
            affine + a @ predicted_state[knot] + b @ predicted_input[knot])
    return predicted_state, predicted_input


class BankedModelFixedPointTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.bundles = MODULE.build_bundles()
        cls.turn_bundles = [
            bundle for bundle in cls.bundles
            if bundle["metadata"]["maneuver_kind"] == "turn"]
        cls.braking_bundles = [
            bundle for bundle in cls.bundles
            if bundle["metadata"]["maneuver_kind"] == "braking"]
        cls.bundle = cls.bundles[1]
        initial_error = np.zeros(MODULE.STATE_DIM)
        initial_error[[0, 1, 3, 6, 7, 11]] = [
            0.04, -0.03, 0.01, 0.08, -0.05, 0.04]
        cls.state_reference, cls.input_reference = affine_reference(
            cls.bundle, initial_error)

    def test_all_signed_tiers_are_nonlinear_zero_error_fixed_points(self):
        self.assertEqual(
            [float(bundle["metadata"]["speed_mps"])
             for bundle in self.turn_bundles[::2]],
            list(MODULE.SPEEDS_MPS))
        self.assertEqual(len(self.bundles), 16)
        for bundle in self.bundles[1:11]:
            with self.subTest(bundle=bundle["name"]):
                np.testing.assert_array_equal(
                    bundle["nominal_state"], np.zeros(MODULE.STATE_DIM))
                np.testing.assert_array_equal(
                    bundle["nominal_input"], np.zeros(MODULE.INPUT_DIM))
                self.assertEqual(bundle["Q_diagonal"][6], 5000.0)
                self.assertEqual(bundle["Q_diagonal"][7], 5000.0)
                self.assertLess(
                    bundle["verification"][
                        "nonlinear_fixed_point_20_knot_max_abs"],
                    MODULE.NONLINEAR_FIXED_POINT_TOLERANCE)

    def test_speed_indexed_braking_bundles_are_exact_phase_fixed_points(self):
        braking = self.braking_bundles
        expected_pitch_deg = [
            -29.29759070111179,
            -28.185406534783958,
            -27.04959629846418,
            -25.890320194538365,
            -24.707821915176453,
        ]
        self.assertEqual(
            [float(bundle["metadata"]["speed_mps"]) for bundle in braking],
            list(MODULE.SPEEDS_MPS))
        for bundle, expected_pitch in zip(braking, expected_pitch_deg):
            metadata = bundle["metadata"]
            with self.subTest(bundle=bundle["name"]):
                self.assertEqual(metadata["maneuver_kind"], "braking")
                self.assertEqual(
                    float(metadata["braking_deceleration_mps2"]), 6.0)
                self.assertLess(float(metadata["pitch_rad"]), 0.0)
                self.assertAlmostEqual(
                    math.degrees(float(metadata["pitch_rad"])),
                    expected_pitch, places=9)
                self.assertEqual(float(metadata["roll_rad"]), 0.0)
                self.assertLess(
                    bundle["verification"][
                        "nonlinear_frozen_phase_fixed_point_max_abs"],
                    1.0e-12)

    def test_drag_aware_operating_points_are_signed_and_physical(self):
        hover_total = MODULE.MASS_KG * 9.81
        for bundle in self.bundles[1:11]:
            metadata = bundle["metadata"]
            speed = float(metadata["speed_mps"])
            side = int(metadata["side_sign"])
            with self.subTest(bundle=bundle["name"]):
                self.assertGreater(float(metadata["pitch_rad"]), 0.0)
                # Positive yaw-rate/left-turn convention requires negative
                # roll in the quaternion chart used by the stored model.
                self.assertEqual(np.sign(float(metadata["roll_rad"])), -side)
                self.assertGreater(float(metadata["nominal_total_thrust_n"]), hover_total)
                self.assertAlmostEqual(
                    float(metadata["nominal_tangential_drag_accel_mps2"]),
                    -float(MODULE.BODY_LINEAR_DRAG_N_PER_MPS[0, 0]) * speed
                    / MODULE.MASS_KG,
                    places=12,
                )
                self.assertLess(
                    float(metadata[
                        "translational_equilibrium_residual_max_abs"]),
                    1.0e-12)
                physical_input = np.asarray(bundle["physical_input"])
                self.assertTrue(np.all(np.isfinite(physical_input)))
                self.assertTrue(np.all(physical_input > 0.0))
                self.assertTrue(np.all(
                    physical_input < MODULE.plant_profile.MAX_MOTOR_THRUST_N))

    def test_coordinate_round_trip_for_every_signed_tier(self):
        error = np.asarray([
            0.07, -0.04, 0.03, 0.02, -0.015, 0.025,
            0.11, -0.08, 0.04, 0.07, -0.05, 0.09,
            0.01, -0.008, 0.006, -0.004,
        ])
        for bundle in self.bundles[1:]:
            metadata = bundle["metadata"]
            time_s = 0.17
            if metadata["maneuver_kind"] == "turn":
                reference = MODULE._turn_reference_absolute(
                    metadata, bundle["physical_input"], time_s)
                yaw = float(metadata["yaw_rate_rad_s"]) * time_s
            else:
                reference = MODULE._braking_reference_absolute(
                    metadata, bundle["physical_input"])
                yaw = 0.0
            absolute = MODULE._bank_error_to_absolute(error, reference, yaw)
            recovered = MODULE._bank_error_from_absolute(
                absolute, reference, yaw)
            with self.subTest(bundle=bundle["name"]):
                np.testing.assert_allclose(
                    recovered, error, rtol=0.0, atol=2.0e-14)

    def test_level_bundle_retains_accepted_hover_parity(self):
        self.assertLess(
            self.bundles[0]["verification"][
                "accepted_hover_parity_max_abs"], 1.0e-8)

    def test_corrected_recursion_preserves_exact_banked_reference(self):
        cache = self.bundle["cache"]
        rho = float(MODULE.level.RHO)
        augmented_coefficient = (
            np.asarray(cache["Kinf"]).T
            @ (np.asarray(self.bundle["R"]) + rho * np.eye(MODULE.INPUT_DIM))
            - np.asarray(cache["AmBKt"])
            @ np.asarray(cache["Pinf"])
            @ np.asarray(self.bundle["B"])
        )
        np.testing.assert_allclose(
            np.asarray(cache["coeff_d2p"]), augmented_coefficient,
            rtol=1.0e-12, atol=1.0e-12)
        states, inputs = firmware_primal_update(
            self.bundle,
            self.state_reference,
            self.input_reference,
            np.asarray(cache["coeff_d2p"]),
            add_terminal_admm_term=False,
        )
        self.assertLess(
            float(np.max(np.abs(inputs - self.input_reference))), 1.0e-4)
        self.assertLess(
            float(np.max(np.abs(states - self.state_reference))), 1.5e-3)

    def test_terminal_admm_term_breaks_the_exact_fixed_point(self):
        cache = self.bundle["cache"]
        _, inputs = firmware_primal_update(
            self.bundle,
            self.state_reference,
            self.input_reference,
            np.asarray(cache["coeff_d2p"]),
            add_terminal_admm_term=True,
        )
        self.assertGreater(
            float(np.max(np.abs(inputs - self.input_reference))), 5.0e-4)

    def test_unaugmented_input_cost_coefficient_breaks_fixed_point(self):
        cache = self.bundle["cache"]
        unaugmented_coefficient = (
            np.asarray(cache["Kinf"]).T @ np.asarray(self.bundle["R"])
            - np.asarray(cache["AmBKt"])
            @ np.asarray(cache["Pinf"])
            @ np.asarray(self.bundle["B"])
        )
        _, inputs = firmware_primal_update(
            self.bundle,
            self.state_reference,
            self.input_reference,
            unaugmented_coefficient,
            add_terminal_admm_term=False,
        )
        self.assertGreater(
            float(np.max(np.abs(inputs - self.input_reference))), 1.0e-3)


if __name__ == "__main__":
    unittest.main()
