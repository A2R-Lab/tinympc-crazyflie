from __future__ import annotations

import unittest

import numpy as np

from tools.crazysim_mujoco.rl.data import RunTransitions
from tools.crazysim_mujoco.rl.generate_closed_loop_corrections import (
    correction_focus_mask, validate_source_configs)


def dataset() -> RunTransitions:
    action = np.asarray([1, 1, 1, 2, 2, 0, 0, 0], dtype=np.int64)
    behavior = np.asarray([1, 1, 1, 1, 1, 2, 0, 0], dtype=np.int64)
    count = len(action)
    scores = np.zeros((count, 3), dtype=np.float32)
    scores[np.arange(count), action] = 1.0
    latent = np.zeros((count, 7), dtype=np.float32)
    return RunTransitions(
        np.zeros((count, 2, 160, 160), dtype=np.uint8), latent, latent.copy(),
        behavior, action, scores, action != 0,
        np.asarray([False, False, False, False, False, True, True, False]),
        np.zeros(count, dtype=np.float32), np.zeros(count, dtype=np.float32),
        np.zeros(count, dtype=np.int64))


def config(seed: int, policy_hash: str = "abc") -> dict:
    return {
        "random_seed": seed,
        "vision_adapter": "hybrid_rl",
        "vision_control_enabled": True,
        "stop_on_contact": True,
        "vision_model": {"sha256": policy_hash},
    }


class ClosedLoopCorrectionsTest(unittest.TestCase):
    def test_source_contract_and_reserved_seeds(self):
        validate_source_configs([config(404), config(505), config(606)], "abc")
        with self.assertRaisesRegex(ValueError, "reserved evaluation"):
            validate_source_configs([config(404), config(505), config(707)], "abc")
        with self.assertRaisesRegex(ValueError, "not closed-loop"):
            bad = config(606)
            bad["vision_control_enabled"] = False
            validate_source_configs([config(404), config(505), bad], "abc")
        with self.assertRaisesRegex(ValueError, "policy hash"):
            validate_source_configs(
                [config(404), config(505), config(606, "wrong")], "abc")

    def test_focus_includes_disagreement_recovery_transition_and_terminal(self):
        focus = correction_focus_mask(
            dataset(), transition_radius=0, terminal_frames=1)
        np.testing.assert_array_equal(
            focus, [False, False, False, True, True, True, True, True])


if __name__ == "__main__":
    unittest.main()
