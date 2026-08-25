from __future__ import annotations

import hashlib
import json
import tempfile
import unittest
from pathlib import Path

from .parse_gvsoc_harness import deployment_flash_accounting
from .validate_candidate_bundle import validate


def sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


class DeploymentAccountingTest(unittest.TestCase):
    def test_gvsoc_job_uses_power_of_two_gap8_cluster(self):
        script = (Path(__file__).parent /
                  "run_joint_dory_gvsoc.sbatch").read_text()
        self.assertIn("platform=gvsoc CORE=8", script)
        self.assertNotIn("platform=gvsoc CORE=7", script)

    def test_flash_bound_subtracts_only_ordered_evaluation_clips(self):
        with tempfile.TemporaryDirectory() as tmp:
            app = Path(tmp)
            build = app / "BUILD/GAP8_V2/GCC_RISCV"
            build.mkdir(parents=True)
            (build / "target.board.devices.flash.img").write_bytes(b"x" * 500)
            hex_dir = app / "hex"
            hex_dir.mkdir()
            for index in range(200):
                (hex_dir / f"joint_gvsoc_clip_{index:03d}.hex").write_bytes(b"c")
            report = deployment_flash_accounting(app)
            self.assertEqual(report["complete_instrumented_flash_image_bytes"], 500)
            self.assertEqual(report["gvsoc_evaluation_corpus_bytes"], 200)
            self.assertEqual(report["deployment_flash_upper_bound_bytes"], 300)

    def test_flash_bound_rejects_incomplete_corpus(self):
        with tempfile.TemporaryDirectory() as tmp:
            app = Path(tmp)
            build = app / "BUILD/x"
            build.mkdir(parents=True)
            (build / "target.board.devices.flash.img").write_bytes(b"x" * 500)
            (app / "hex").mkdir()
            with self.assertRaisesRegex(RuntimeError, "exactly clips"):
                deployment_flash_accounting(app)

    def test_bundle_binds_checkpoint_and_runtime_policy(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            checkpoint = root / "checkpoint.pt"
            policy = root / "policy.onnx"
            checkpoint.write_bytes(b"checkpoint")
            policy.write_bytes(b"policy")
            bundle = root / "bundle.json"
            bundle.write_text(json.dumps({
                "format": "tinympc-joint-gate-obstacle-student-v1",
                "runtime_adapter": "joint_gate_rl",
                "artifacts": {
                    "checkpoint": {"path": checkpoint.name, "sha256": sha(checkpoint)},
                    "policy_onnx": {"path": policy.name, "sha256": sha(policy)},
                },
            }))
            report = validate(bundle, checkpoint, sha(checkpoint))
            self.assertEqual(report["checkpoint_sha256"], sha(checkpoint))
            self.assertEqual(report["policy_onnx_sha256"], sha(policy))
            other = root / "other.pt"
            other.write_bytes(checkpoint.read_bytes())
            with self.assertRaisesRegex(ValueError, "not the bundle checkpoint"):
                validate(bundle, other, sha(other))


if __name__ == "__main__":
    unittest.main()
