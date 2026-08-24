from __future__ import annotations

import unittest
import tempfile
from pathlib import Path

import numpy as np

from tools.crazysim_mujoco.rl.data import RunTransitions
from tools.crazysim_mujoco.rl.generate_oracle_expert import (
    mirror_dataset, oracle_labels, photometric_dataset, save_npz_deterministic,
    sha256, validate_source_seeds)


def source_dataset() -> RunTransitions:
    count = 3
    actions = np.asarray([0, 1, 2], dtype=np.int64)
    scores = np.eye(3, dtype=np.float32)
    latent = np.asarray([
        [0.0, 0.2, 0.3, -0.4, 0.5, 1.0, 1.0],
        [0.1, -0.1, 0.3, 0.2, -0.3, 1.0, 1.0],
        [0.2, 0.0, 0.3, 0.0, 0.0, 1.0, 1.0]], dtype=np.float32)
    return RunTransitions(
        np.arange(count * 2 * 160 * 160, dtype=np.uint32).reshape(
            count, 2, 160, 160).astype(np.uint8),
        latent, latent.copy(), actions.copy(), actions.copy(), scores,
        actions != 0, np.asarray([True, False, False]),
        np.zeros(count, dtype=np.float32), np.asarray([0.0, 0.0, 1.0]),
        np.full(count, 11, dtype=np.int64))


class OracleExpertTest(unittest.TestCase):
    def test_passes_opposite_signed_obstacle_cross_track(self):
        geometry = [
            {"name": "left_block", "near_progress_m": 1.0,
             "signed_cross_track_m": 0.5},
            {"name": "right_block", "near_progress_m": 4.0,
             "signed_cross_track_m": -0.5},
        ]
        action, scores, decision, _, selected = oracle_labels(
            np.asarray([0.0, 3.0, 6.0]), geometry)
        np.testing.assert_array_equal(action, [2, 1, 0])
        np.testing.assert_array_equal(decision, [True, True, False])
        np.testing.assert_array_equal(selected, [0, 1, -1])
        self.assertTrue(np.all(np.argmax(scores, axis=1) == action))

    def test_mirror_flips_image_lateral_latents_and_actions(self):
        source = source_dataset()
        mirrored = mirror_dataset(source, 11)
        np.testing.assert_array_equal(mirrored.frames, source.frames[:, :, :, ::-1])
        for index in (1, 3, 4):
            np.testing.assert_allclose(mirrored.latent[:, index], -source.latent[:, index])
        np.testing.assert_array_equal(mirrored.expert_action, [0, 2, 1])
        np.testing.assert_array_equal(mirrored.expert_scores, source.expert_scores[:, [0, 2, 1]])
        np.testing.assert_array_equal(mirrored.episode_id, source.episode_id)

    def test_noise_and_metadata_are_deterministic(self):
        source = source_dataset()
        first = photometric_dataset(source, 11, 1102, "noise")
        second = photometric_dataset(source, 11, 1102, "noise")
        np.testing.assert_array_equal(first.frames, second.frames)
        np.testing.assert_array_equal(first.episode_id, [11, 11, 11])
        with tempfile.TemporaryDirectory() as directory:
            one = Path(directory) / "one.npz"
            two = Path(directory) / "two.npz"
            arrays = {"frames": first.frames, "episode_id": first.episode_id}
            save_npz_deterministic(one, arrays)
            save_npz_deterministic(two, arrays)
            self.assertEqual(sha256(one), sha256(two))

    def test_heldout_and_incomplete_training_seed_sets_are_rejected(self):
        validate_source_seeds([11, 22, 33])
        with self.assertRaisesRegex(ValueError, "held-out"):
            validate_source_seeds([11, 22, 101])
        with self.assertRaisesRegex(ValueError, "exactly"):
            validate_source_seeds([11, 22])


if __name__ == "__main__":
    unittest.main()
