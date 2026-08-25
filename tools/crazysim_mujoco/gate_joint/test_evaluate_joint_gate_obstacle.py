import csv
import hashlib
import importlib.util
import json
import tempfile
import unittest
from pathlib import Path
from unittest import mock

MODULE = Path(__file__).with_name("evaluate_joint_gate_obstacle.py")
SPEC = importlib.util.spec_from_file_location("joint_evaluation", MODULE)
assert SPEC and SPEC.loader
evaluation = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(evaluation)

COMMON = {"acceptance_contract.controller_sha256": "c" * 64,
          "camera_calibration.sha256": "d" * 64,
          "acceptance_contract.vision_bridge_sha256": "1" * 64,
          "acceptance_contract.analyzer_sha256": "2" * 64,
          **evaluation.FROZEN_INVARIANTS}
MATRIX_TOOLS = {
    "matrix_runner": {"name": "run_joint_gate_obstacle_matrix.sh",
                      "sha256": "3" * 64},
    "evaluator": {"name": "evaluate_joint_gate_obstacle.py",
                  "sha256": "4" * 64},
}
CANDIDATE_HASH = hashlib.sha256(b"candidate policy").hexdigest()
IDENTITY = {"bundle_path": "/synthetic/bundle.json",
            "bundle_manifest_sha256": "e" * 64, "policy_sha256": CANDIDATE_HASH}


def make_run(root: Path, group: str, seed: int, *, generic_contact=False,
             gate_pass=True, classified=True, missing_contact=False) -> Path:
    directory = root / f"{group}_{seed}"
    directory.mkdir()
    candidate = group != "baseline_obstacle_only"
    course_manifest = evaluation.COURSE_DIR / f"{evaluation.COURSES[group]}.json"
    course_data = json.loads(course_manifest.read_text())
    course_scene = evaluation.SCENE_DIR / course_data["scene"]
    config = {
        "random_seed": seed, "course": evaluation.COURSES[group],
        "vision_adapter": evaluation.CANDIDATE_ADAPTER if candidate else evaluation.BASELINE_ADAPTER,
        "vision_model": ({"bundle_manifest_sha256": IDENTITY["bundle_manifest_sha256"]}
                         if candidate else {"sha256": evaluation.BASELINE_POLICY_SHA256}),
        "launch_time_s": 1.0, "stop_on_contact": True,
        "course_manifest_sha256": evaluation.sha256(course_manifest),
        "course_scene_sha256": evaluation.sha256(course_scene),
        "acceptance_contract": {"controller_sha256": "c" * 64,
                                "vision_bridge_sha256": "1" * 64,
                                "analyzer_sha256": "2" * 64,
                                "gate_corner_span_m": 1.2333333333333334,
                                "generated_model_sha256": evaluation.FROZEN_INVARIANTS["acceptance_contract.generated_model_sha256"],
                                "bank_header_sha256": evaluation.FROZEN_INVARIANTS["acceptance_contract.bank_header_sha256"],
                                "bank_provenance_sha256": evaluation.FROZEN_INVARIANTS["acceptance_contract.bank_provenance_sha256"]},
        "camera_calibration": {
            "sha256": "d" * 64, "model": "Himax HM01B0",
            "bridge_calibration_transform_enabled": True,
            "poc_overscan_render_resolution": [192, 192],
            "poc_transform": "principal_point+plumb_bob+isaac_gray_response+seeded_noise",
            "poc_sensor_seed": seed,
        },
    }
    summary = {
        "course": evaluation.COURSES[group], "launch_time_s": 1.0, "duration_s": 2.0,
        "course_success": not generic_contact, "course_pass_point_reached": not generic_contact,
        "crashed": generic_contact, "contact_count_max": 1 if generic_contact else 0,
        "course_gate_results": [{"crossed": gate_pass}],
        "course_gates_passed_in_order": gate_pass,
        "course_obstacle_clearance_min_m": .2,
        "course_mean_horizontal_speed_active_mps": .4,
        "vision_inference_latency_ms": {"p95": 2.0},
    }
    if group != "candidate_gate_obstacle":
        summary["course_gate_results"] = []
    if classified:
        summary["course_gate_frame_contact_count"] = 0
        summary["course_obstacle_contact_count"] = 1 if generic_contact else 0
    if missing_contact:
        summary.pop("crashed")
        summary.pop("contact_count_max")
    (directory / "run_config.json").write_text(json.dumps(config))
    (directory / "summary.json").write_text(json.dumps(summary))
    (directory / "state.csv").write_text("time_s\n2.0\n")
    (directory / "firmware.log").write_text("ok\n")
    with (directory / "vision.csv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=["inference_ms"])
        writer.writeheader()
        writer.writerow({"inference_ms": 2})
    return directory


def make_groups(root: Path):
    groups = {group: [make_run(root, group, seed) for seed in range(10)]
              for group in evaluation.GROUPS}
    (root / "matrix_status.tsv").write_text("\n".join(
        f"{group}\t{seed}\t0" for group in evaluation.GROUPS for seed in range(10)) + "\n")
    return groups


class JointEvaluationTest(unittest.TestCase):
    def test_exact_scene_geometry_rejects_collider_drift(self):
        scene = evaluation.SCENE_DIR / "vision_gate_obstacle_easy_transition.xml"
        gate = {"center": [4.0, 0.3, 1.5], "opening": [1.0, 1.0]}
        errors = []
        evaluation.validate_newbee_scene_geometry(scene, gate, errors)
        self.assertEqual(errors, [])
        with tempfile.TemporaryDirectory() as temporary:
            changed = Path(temporary) / "scene.xml"
            changed.write_text(scene.read_text().replace(
                'pos="0 -0.62 0"', 'pos="0 -0.500 0"', 1))
            errors = []
            evaluation.validate_newbee_scene_geometry(changed, gate, errors)
            self.assertIn(
                "course scene collision rail mismatch: expected_gate_left", errors)

    def test_main_returns_nonzero_when_acceptance_fails(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            groups = make_groups(root)
            # Preserve the run artifacts while forcing a matrix-contract
            # failure, so this checks the CLI acceptance exit contract.
            status_path = root / "matrix_status.tsv"
            status = status_path.read_text().replace(
                "candidate_gate_obstacle\t0\t0",
                "candidate_gate_obstacle\t0\t1", 1)
            status_path.write_text(status)
            (root / "policy.onnx").write_bytes(b"candidate policy")
            bundle = root / "bundle.json"
            bundle.write_text(json.dumps({
                "format": "tinympc-joint-gate-obstacle-student-v1",
                "runtime_adapter": "joint_gate_rl",
                "artifacts": {"policy_onnx": {
                    "path": "policy.onnx", "sha256": CANDIDATE_HASH}},
            }))
            common = root / "common.json"
            common.write_text(json.dumps(COMMON))
            tool_dir = root / "matrix_tools"
            tool_dir.mkdir()
            matrix_runner = tool_dir / "run_joint_gate_obstacle_matrix.sh"
            matrix_runner.write_text("#!/bin/sh\n")
            evaluator_snapshot = tool_dir / "evaluate_joint_gate_obstacle.py"
            evaluator_snapshot.write_bytes(MODULE.read_bytes())
            argv = ["evaluate_joint_gate_obstacle.py"]
            for group in evaluation.GROUPS:
                option = "--" + group.replace("_", "-") + "-run"
                for run in groups[group]:
                    argv.extend([option, str(run)])
            argv.extend(["--candidate-bundle", str(bundle),
                         "--common-provenance", str(common),
                         "--out-dir", str(root / "evaluation"),
                         "--seed-start", "0",
                         "--matrix-runner-snapshot", str(matrix_runner),
                         "--evaluator-snapshot", str(evaluator_snapshot)])
            with mock.patch("sys.argv", argv), mock.patch.object(
                    evaluation, "load_candidate_bundle",
                    return_value=(IDENTITY, [])):
                self.assertEqual(evaluation.main(), 1)
            self.assertTrue((root / "evaluation" / "evaluation.json").is_file())
            self.assertTrue((root / "evaluation" / "evaluation.md").is_file())

    def test_acceptance_and_separate_model_provenance(self):
        with tempfile.TemporaryDirectory() as temporary:
            groups = make_groups(Path(temporary))
            report = evaluation.evaluate(
                groups, COMMON, IDENTITY,
                matrix_tool_provenance=MATRIX_TOOLS)
            self.assertTrue(report["acceptance"]["accepted"])
            self.assertEqual(report["matrix_tool_provenance"], MATRIX_TOOLS)
            baseline = report["groups"]["baseline_obstacle_only"]["runs"][0]
            self.assertEqual(baseline["expected_policy_sha256"], evaluation.BASELINE_POLICY_SHA256)
            config_path = groups["candidate_obstacle_only"][0] / "run_config.json"
            config = json.loads(config_path.read_text())
            config["vision_model"]["bundle_manifest_sha256"] = "0" * 64
            config_path.write_text(json.dumps(config))
            rejected = evaluation.evaluate(
                groups, COMMON, IDENTITY,
                matrix_tool_provenance=MATRIX_TOOLS)
            self.assertFalse(rejected["acceptance"]["checks"]["matrix_valid"])
            self.assertIn("candidate bundle manifest provenance mismatch",
                          rejected["groups"]["candidate_obstacle_only"]["runs"][0]["errors"])
            bad_common = dict(COMMON); bad_common["acceptance_contract.generated_model_sha256"] = "0" * 64
            rejected = evaluation.evaluate(
                groups, bad_common, IDENTITY,
                matrix_tool_provenance=MATRIX_TOOLS)
            self.assertFalse(rejected["acceptance"]["checks"]["matrix_valid"])

            missing_runtime_hash = dict(COMMON)
            del missing_runtime_hash["acceptance_contract.analyzer_sha256"]
            rejected = evaluation.evaluate(
                groups, missing_runtime_hash, IDENTITY,
                matrix_tool_provenance=MATRIX_TOOLS)
            self.assertIn(
                "missing or invalid common runtime tool hash "
                "acceptance_contract.analyzer_sha256",
                rejected["matrix_errors"])

            rejected = evaluation.evaluate(
                groups, COMMON, IDENTITY,
                matrix_tool_provenance={"matrix_runner": MATRIX_TOOLS["matrix_runner"]})
            self.assertIn("missing or invalid matrix tool provenance",
                          rejected["matrix_errors"])

    def test_retained_matrix_tools_are_hashed_and_evaluator_is_exact(self):
        with tempfile.TemporaryDirectory() as temporary:
            tool_dir = Path(temporary)
            runner = tool_dir / "run_joint_gate_obstacle_matrix.sh"
            runner.write_text("#!/bin/sh\n")
            evaluator_snapshot = tool_dir / "evaluate_joint_gate_obstacle.py"
            evaluator_snapshot.write_bytes(MODULE.read_bytes())
            errors = []
            tools = evaluation.load_matrix_tool_provenance(
                runner, evaluator_snapshot, errors)
            self.assertEqual(errors, [])
            self.assertEqual(tools["matrix_runner"]["sha256"],
                             hashlib.sha256(runner.read_bytes()).hexdigest())
            self.assertEqual(tools["evaluator"]["sha256"],
                             hashlib.sha256(MODULE.read_bytes()).hexdigest())

    def test_candidate_bundle_manifest_and_artifact_hash(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            (root / "policy.onnx").write_bytes(b"candidate policy")
            bundle = {"format": "tinympc-joint-gate-obstacle-student-v1",
                      "runtime_adapter": "joint_gate_rl",
                      "artifacts": {"policy_onnx": {"path": "policy.onnx",
                                                     "sha256": CANDIDATE_HASH}}}
            path = root / "bundle.json"
            path.write_text(json.dumps(bundle))
            identity, errors = evaluation.load_candidate_bundle(path)
            self.assertEqual(errors, [])
            self.assertEqual(identity["policy_sha256"], CANDIDATE_HASH)
            self.assertEqual(identity["bundle_manifest_sha256"], hashlib.sha256(path.read_bytes()).hexdigest())
            bundle["artifacts"]["policy_onnx"]["sha256"] = "0" * 64
            path.write_text(json.dumps(bundle))
            _, errors = evaluation.load_candidate_bundle(path)
            self.assertIn("candidate bundle policy hash does not match policy artifact", errors)
            bundle["artifacts"]["policy_onnx"]["path"] = "../policy.onnx"
            path.write_text(json.dumps(bundle))
            _, errors = evaluation.load_candidate_bundle(path)
            self.assertIn("candidate bundle policy must be a direct child artifact", errors)

    def test_expected_seed_block_is_enforced(self):
        with tempfile.TemporaryDirectory() as temporary:
            groups = make_groups(Path(temporary))
            report = evaluation.evaluate(
                groups, COMMON, IDENTITY, expected_seed_start=100,
                matrix_tool_provenance=MATRIX_TOOLS)
            self.assertIn("matrix seeds differ from declared fresh seed block",
                          report["matrix_errors"])

    def test_conservative_stop_first_contact_classification(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            pre = make_run(root, "candidate_gate_obstacle", 10, generic_contact=True,
                           gate_pass=False, classified=False)
            post = make_run(root, "candidate_gate_obstacle", 11, generic_contact=True,
                            gate_pass=True, classified=False)
            obstacle = make_run(root, "candidate_obstacle_only", 12, generic_contact=True,
                                classified=False)
            pre_row = evaluation.make_row(pre, "candidate_gate_obstacle", COMMON, CANDIDATE_HASH)
            post_row = evaluation.make_row(post, "candidate_gate_obstacle", COMMON, CANDIDATE_HASH)
            obstacle_row = evaluation.make_row(obstacle, "candidate_obstacle_only", COMMON, CANDIDATE_HASH)
            self.assertEqual(pre_row["gate_frame_contacts"], 1)
            self.assertEqual(pre_row["source"], "conservative_stop_first_contact_pre_gate")
            self.assertEqual(post_row["obstacle_contacts"], 1)
            self.assertEqual(post_row["source"], "conservative_stop_first_contact_post_gate")
            self.assertEqual(obstacle_row["obstacle_contacts"], 1)
            self.assertEqual(obstacle_row["clearance_scope"], "truncated_at_first_contact")

    def test_missing_contact_telemetry_is_unknown_and_fails_acceptance(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            groups = make_groups(root)
            summary_path = groups["candidate_obstacle_only"][0] / "summary.json"
            summary = json.loads(summary_path.read_text())
            for key in ("crashed", "contact_count_max", "course_gate_frame_contact_count",
                        "course_obstacle_contact_count"):
                summary.pop(key, None)
            summary_path.write_text(json.dumps(summary))
            report = evaluation.evaluate(
                groups, COMMON, IDENTITY,
                matrix_tool_provenance=MATRIX_TOOLS)
            aggregate = report["groups"]["candidate_obstacle_only"]["aggregate"]
            self.assertIsNone(aggregate["obstacle_contact_rate"])
            self.assertEqual(aggregate["unknown_obstacle_contact_runs"], 1)
            self.assertFalse(report["acceptance"]["checks"]["obstacle_only_collision_not_worse"])


if __name__ == "__main__":
    unittest.main()
