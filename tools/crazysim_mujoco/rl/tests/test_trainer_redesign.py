from __future__ import annotations

import tempfile
import unittest
from pathlib import Path

import numpy as np
import torch
import torch.nn.functional as functional

from tools.crazysim_mujoco.rl.data import (
    RunTransitions, load_expert_npz, split_by_episode)
from tools.crazysim_mujoco.rl.model import TemporalPolicy
from tools.crazysim_mujoco.rl.train_mbpo import balanced_expert_indices


def dataset(count=12):
    action = np.resize(np.asarray([0, 1, 2]), count)
    scores = np.zeros((count, 3), dtype=np.float32)
    scores[np.arange(count), action] = 1.0
    latent = np.zeros((count, 7), dtype=np.float32)
    next_latent = latent.copy()
    next_latent[:, 0] = 0.01
    return RunTransitions(
        np.zeros((count, 2, 160, 160), dtype=np.uint8), latent, next_latent,
        action.copy(), action.copy(), scores, action != 0,
        (action == 0) & (np.arange(count) % 2 == 0),
        np.zeros(count, dtype=np.float32), np.zeros(count, dtype=np.float32),
        np.repeat(np.arange(3), int(np.ceil(count / 3)))[:count])


class TrainerRedesignTest(unittest.TestCase):
    def test_npz_schema_and_derived_reward(self):
        source = dataset()
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "expert.npz"
            np.savez(path, **{
                field: getattr(source, field) for field in (
                    "frames", "latent", "next_latent", "behavior_action",
                    "expert_action", "expert_scores", "decision_mask",
                    "hard_track_mask", "done", "episode_id")})
            loaded = load_expert_npz(path)
        self.assertEqual(loaded.frames.shape, (12, 2, 160, 160))
        self.assertTrue(np.all(np.isfinite(loaded.reward)))
        np.testing.assert_array_equal(loaded.expert_action, source.expert_action)

    def test_episode_split_has_no_overlap(self):
        training, validation = split_by_episode(dataset(), 1.0 / 3.0, 9)
        self.assertFalse(set(training.episode_id) & set(validation.episode_id))
        self.assertEqual(len(training.action) + len(validation.action), 12)

    def test_balanced_sampler_includes_hard_track(self):
        source = dataset(18)
        index = balanced_expert_indices(
            source, 12, torch.Generator().manual_seed(5)).numpy()
        selected = source.expert_action[index]
        np.testing.assert_array_equal(np.bincount(selected, minlength=3), [4, 4, 4])
        self.assertTrue(np.any(source.hard_track_mask[index]))

    def test_actor_supervision_reaches_encoder(self):
        torch.manual_seed(3)
        policy = TemporalPolicy()
        frames = torch.rand(3, 2, 160, 160)
        labels = torch.tensor([0, 1, 2])
        functional.cross_entropy(policy(frames), labels).backward()
        gradient = policy.encoder[0].weight.grad
        self.assertIsNotNone(gradient)
        self.assertGreater(float(torch.linalg.vector_norm(gradient)), 0.0)


if __name__ == "__main__":
    unittest.main()

