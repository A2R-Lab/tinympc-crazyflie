from __future__ import annotations

import hashlib
import json
import tempfile
import unittest
from pathlib import Path

import numpy as np

from tools.crazysim_mujoco.rl.evaluate_procedural_policy import (
    classification_metrics, evaluate_policy,
)
from tools.crazysim_mujoco.rl.model import TemporalPolicy
from tools.crazysim_mujoco.rl.reward import transition_reward
from tools.crazysim_mujoco.rl.validate_procedural_dataset import (
    DatasetValidationError, Thresholds, validate_dataset,
)


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def write_shard(root: Path, name: str = "train", *, bad_terminal: bool = False,
                scenario: int = 10) -> Path:
    count = 6
    frames = np.arange(count * 2 * 160 * 160, dtype=np.uint32)
    frames = np.mod(frames, 251).astype(np.uint8).reshape(count, 2, 160, 160)
    latent = np.zeros((count, 7), dtype=np.float32)
    latent[:, 0] = np.linspace(0.0, 1.0, count)
    next_latent = latent.copy()
    next_latent[:, 0] += 0.1
    expert = np.asarray([0, 0, 1, 1, 2, 2], dtype=np.int64)
    behavior = expert.copy()
    scores = np.full((count, 3), -1.0, dtype=np.float32)
    scores[np.arange(count), expert] = 1.0
    done = np.asarray([0, 1, 0, 1, 0, 1], dtype=np.uint8)
    contact = np.asarray([0, 1, 0, 0, 0, 0], dtype=np.uint8)
    complete = np.asarray([0, 0, 0, 1, 0, 0], dtype=np.uint8)
    timeout = np.asarray([0, 0, 0, 0, 0, 1], dtype=np.uint8)
    if bad_terminal:
        contact[1] = 0
    episode = np.asarray([scenario * 10 + value for value in (0, 0, 1, 1, 2, 2)])
    scenario_ids = np.full(count, scenario)
    clearance = np.asarray([0.8, 0.1, 0.3, 0.7, 0.15, 0.5], dtype=np.float32)
    previous = np.asarray([0, 0, 1, 1, 2, 2])
    reward = transition_reward(
        next_latent[:, 0] - latent[:, 0], next_latent[:, 1], next_latent[:, 2],
        clearance, behavior, previous, contact, complete)
    reward[contact.astype(bool)] = -100.0
    reward[complete.astype(bool)] = 20.0
    reward[timeout.astype(bool)] = -5.0
    npz = root / f"{name}.npz"
    np.savez_compressed(
        npz, frames=frames, latent=latent, next_latent=next_latent,
        behavior_action=behavior, expert_action=expert, expert_scores=scores,
        decision_mask=np.asarray([0, 0, 1, 1, 1, 1], dtype=np.uint8),
        hard_track_mask=np.asarray([1, 1, 0, 0, 0, 0], dtype=np.uint8),
        done=done, contact=contact, complete=complete, timeout=timeout,
        episode_id=episode, scenario_id=scenario_ids, clearance=clearance,
        reward=reward)
    manifest = root / f"{name}.json"
    manifest.write_text(json.dumps({
        "format": "test-procedural-v1", "schema_version": 1,
        "path": npz.name, "sha256": sha256(npz), "split": name,
        "seed": scenario, "source_hash": "generator-source-hash",
        "layout_id": f"layout-{scenario}", "appearance_id": f"look-{scenario}",
        "noise_id": f"noise-{scenario}",
    }))
    return manifest


class ProceduralDatasetValidationTest(unittest.TestCase):
    def test_valid_smoke_dataset_and_terminal_rewards(self):
        with tempfile.TemporaryDirectory() as directory:
            manifest = write_shard(Path(directory))
            report = validate_dataset([manifest], Thresholds.smoke())
            self.assertTrue(report["accepted"], report["failed_checks"])
            self.assertEqual(report["summary"]["episode_outcomes"], {
                "success": 1, "contact": 1, "near_miss": 0, "timeout": 1})

    def test_terminal_without_outcome_is_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            manifest = write_shard(Path(directory), bad_terminal=True)
            report = validate_dataset([manifest], Thresholds.smoke())
            self.assertFalse(report["accepted"])
            self.assertIn("train.npz:terminal_equivalence", report["failed_checks"])

    def test_hash_mismatch_fails_closed(self):
        with tempfile.TemporaryDirectory() as directory:
            manifest = write_shard(Path(directory))
            document = json.loads(manifest.read_text())
            document["sha256"] = "0" * 64
            manifest.write_text(json.dumps(document))
            with self.assertRaisesRegex(DatasetValidationError, "SHA-256 mismatch"):
                validate_dataset([manifest], Thresholds.smoke())

    def test_cross_split_scenario_overlap_is_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            train = write_shard(root, "train", scenario=7)
            validation = write_shard(root, "validation", scenario=7)
            report = validate_dataset([train, validation], Thresholds.smoke())
            self.assertFalse(report["accepted"])
            self.assertIn("split_isolation:train:validation:scenario",
                          report["failed_checks"])


class ProceduralPolicyEvaluationTest(unittest.TestCase):
    def test_confusion_metrics_are_macro_not_majority_accuracy(self):
        expected = np.asarray([0, 0, 0, 1, 2])
        predicted = np.asarray([0, 0, 0, 0, 0])
        metrics = classification_metrics(expected, predicted)
        self.assertAlmostEqual(metrics["accuracy"], 0.6)
        self.assertAlmostEqual(metrics["balanced_accuracy"], 1.0 / 3.0)
        self.assertLess(metrics["macro_f1"], metrics["accuracy"])

    def test_onnx_contract_and_checkpoint_parity_are_reported(self):
        try:
            import onnxruntime  # noqa: F401
            import torch
        except ImportError:
            self.skipTest("torch and onnxruntime are required")
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            manifest = write_shard(root, "validation", scenario=44)
            torch.manual_seed(17)
            policy = TemporalPolicy().eval()
            checkpoint = root / "checkpoint.pt"
            torch.save({"policy": policy.state_dict(), "latent_dim": 7}, checkpoint)
            model = root / "policy.onnx"
            torch.onnx.export(
                policy, torch.zeros(1, 2, 160, 160), model,
                input_names=["frames"], output_names=["action_logits"],
                opset_version=17, dynamo=False)
            report = evaluate_policy(
                [manifest], model, splits={"validation"},
                checkpoint_path=checkpoint)
            by_name = {item["name"]: item for item in report["checks"]}
            self.assertTrue(by_name["onnx_contract"]["passed"])
            self.assertTrue(by_name["pytorch_onnx_parity"]["passed"])
            self.assertLessEqual(report["pytorch_onnx_parity"][
                "maximum_absolute_logit_error"], 1.0e-5)


if __name__ == "__main__":
    unittest.main()
