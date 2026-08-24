import hashlib
import json
import os
from pathlib import Path
import tempfile
import unittest

import numpy as np

os.environ.setdefault("MUJOCO_GL", "egl")

from tools.crazysim_mujoco.rl.generate_procedural_avoidance_dataset import (  # noqa: E402
    ACTION_COUNT,
    DatasetConfig,
    generate_dataset,
)


def _digest(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _without_hashes(value):
    if isinstance(value, dict):
        return {key: _without_hashes(item) for key, item in value.items()
                if "sha256" not in key}
    if isinstance(value, list):
        return [_without_hashes(item) for item in value]
    return value


class ProceduralAvoidanceDatasetTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        try:
            import mujoco  # noqa: F401
        except ImportError as error:
            raise unittest.SkipTest(f"MuJoCo is unavailable: {error}")

        cls._temporary = tempfile.TemporaryDirectory()
        root = Path(cls._temporary.name)
        cls.first = root / "first"
        cls.second = root / "second"
        common = dict(
            seed=9137,
            episodes=12,
            samples_per_episode=48,
            validation_fraction=0.25,
            layout_variants=12,
            appearance_variants=5,
            noise_variants=3,
        )
        cls.first_manifest = generate_dataset(DatasetConfig(output=str(cls.first), **common))
        cls.second_manifest = generate_dataset(DatasetConfig(output=str(cls.second), **common))

    @classmethod
    def tearDownClass(cls):
        cls._temporary.cleanup()

    def _combined(self, root: Path):
        with np.load(root / "train.npz") as train, np.load(root / "validation.npz") as validation:
            keys = train.files
            return {key: np.concatenate((train[key], validation[key]), axis=0) for key in keys}

    def test_semantic_determinism(self):
        for split in ("train", "validation"):
            with np.load(self.first / f"{split}.npz") as first, \
                    np.load(self.second / f"{split}.npz") as second:
                self.assertEqual(set(first.files), set(second.files))
                for name in first.files:
                    if name != "frames":
                        np.testing.assert_array_equal(first[name], second[name])
                        continue
                    delta = np.abs(first[name].astype(np.int16) -
                                   second[name].astype(np.int16))
                    self.assertLessEqual(int(delta.max()), 8)
                    self.assertLess(float(np.count_nonzero(delta) / delta.size), 1.0e-5)
        for filename in ("train_manifest.json", "validation_manifest.json", "manifest.json"):
            first = _without_hashes(json.loads((self.first / filename).read_text()))
            second = _without_hashes(json.loads((self.second / filename).read_text()))
            self.assertEqual(first, second)

        # Every recorded digest must still verify the concrete artifact it names.
        manifest = json.loads((self.first / "manifest.json").read_text())
        for filename, detail in manifest["files"].items():
            self.assertEqual(detail["sha256"], _digest(self.first / filename))

    def test_split_is_by_episode(self):
        train = json.loads((self.first / "train_manifest.json").read_text())
        validation = json.loads((self.first / "validation_manifest.json").read_text())
        train_ids = set(train["episode_ids"])
        validation_ids = set(validation["episode_ids"])
        self.assertFalse(train_ids & validation_ids)
        self.assertEqual(train_ids | validation_ids, set(range(12)))

    def test_balanced_actions_and_counterfactual_preference(self):
        arrays = self._combined(self.first)
        counts = np.bincount(arrays["expert_action"], minlength=ACTION_COUNT)
        self.assertTrue(np.all(counts > 0), counts)
        fractions = counts / counts.sum()
        self.assertGreaterEqual(float(fractions.min()), 0.10, fractions)
        supervised = (arrays["decision_mask"] != 0) | (arrays["hard_track_mask"] != 0)
        preferred = np.argmax(arrays["expert_scores"][supervised], axis=1)
        np.testing.assert_array_equal(preferred, arrays["expert_action"][supervised])

    def test_clearance_and_terminal_semantics(self):
        arrays = self._combined(self.first)
        terminal_rows = arrays["terminal"].astype(bool)
        self.assertEqual(int(terminal_rows.sum()), 12)
        np.testing.assert_array_equal(arrays["done"], arrays["terminal"])

        contact_rows = arrays["contact"].astype(bool)
        success_rows = arrays["success"].astype(bool)
        timeout_rows = arrays["timeout"].astype(bool)
        self.assertTrue(np.all(terminal_rows[contact_rows | success_rows | timeout_rows]))
        self.assertTrue(np.all(arrays["reward"][contact_rows] == -100.0))
        self.assertTrue(np.all(arrays["reward"][success_rows] == 20.0))
        self.assertTrue(np.all(arrays["reward"][timeout_rows] == -5.0))
        self.assertFalse(np.any(contact_rows & success_rows))
        self.assertFalse(np.any(contact_rows & timeout_rows))
        self.assertFalse(np.any(success_rows & timeout_rows))

        contact_episode_ids = set(arrays["episode_id"][contact_rows].tolist())
        for episode_id in np.unique(arrays["episode_id"]):
            episode = arrays["episode_id"] == episode_id
            minimum = float(arrays["latent"][episode, 5].min())
            if int(episode_id) in contact_episode_ids:
                self.assertLessEqual(minimum, 0.0)
            else:
                self.assertGreater(minimum, 0.0)

    def test_schema_and_rendered_frames(self):
        arrays = self._combined(self.first)
        self.assertEqual(arrays["frames"].shape[1:], (2, 160, 160))
        self.assertEqual(arrays["frames"].dtype, np.uint8)
        self.assertEqual(arrays["latent"].shape[1], 7)
        self.assertEqual(arrays["expert_scores"].shape[1], ACTION_COUNT)
        self.assertGreater(float(arrays["frames"].std()), 2.0)
        self.assertTrue(np.array_equal(arrays["frames"][0, 0], arrays["frames"][0, 1]))

    def test_training_loader_accepts_generated_schema(self):
        from tools.crazysim_mujoco.rl.data import load_expert_npz

        loaded = load_expert_npz(self.first / "train.npz")
        self.assertGreater(len(loaded.expert_action), 0)
        self.assertEqual(loaded.expert_scores.shape[1], ACTION_COUNT)
        self.assertTrue(np.all(loaded.expert_action[loaded.hard_track_mask] == 0))


if __name__ == "__main__":
    unittest.main()
