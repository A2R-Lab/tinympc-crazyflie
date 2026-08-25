import importlib.util
import json
import math
import tempfile
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[3]
EVAL = Path(__file__).with_name("evaluate_heldout_rooms.py")
SPEC = importlib.util.spec_from_file_location("heldout_eval", EVAL)
evaluation = importlib.util.module_from_spec(SPEC); SPEC.loader.exec_module(evaluation)
ANALYZE = ROOT / "tools/crazysim_mujoco/analyze_run.py"
ANALYZE_SPEC = importlib.util.spec_from_file_location("heldout_analyze", ANALYZE)
analyze = importlib.util.module_from_spec(ANALYZE_SPEC); ANALYZE_SPEC.loader.exec_module(analyze)
GENERATOR_PATH = ROOT / "tools/crazysim_mujoco/generate_canonical_courses.py"
GENERATOR_SPEC = importlib.util.spec_from_file_location("course_generator", GENERATOR_PATH)
generator = importlib.util.module_from_spec(GENERATOR_SPEC); GENERATOR_SPEC.loader.exec_module(generator)


def synthetic_gate_data(cross_gate=True):
    t = np.asarray([0., 1., 2., 3.])
    x = np.asarray([0., .5, 1., 0.]) if cross_gate else np.asarray([0., .25, .25, 0.])
    zeros = np.zeros_like(t)
    return {
        "time_s": t, "x_m": x, "y_m": zeros, "z_m": np.full_like(t, 1.5),
        "vx_mps": zeros, "vy_mps": zeros, "qw": np.ones_like(t),
        "qx": zeros, "qy": zeros, "qz": zeros, "wx_radps": zeros,
        "wy_radps": zeros, "wz_radps": zeros, "contacts": zeros,
        "rpm_1": np.ones_like(t), "rpm_2": np.ones_like(t),
        "rpm_3": np.ones_like(t), "rpm_4": np.ones_like(t),
    }


def synthetic_gate_course():
    return {
        "name": "synthetic_heldout", "description": "test",
        "centerline": [[0., 0.], [1., 0.], [0., 0.]],
        # This deliberately equals the launch point.  Completion must wait
        # for the physical forward gate crossing and subsequent return.
        "pass_point": [0., 0.], "pass_radius_m": .05,
        "obstacles": [{"name": "box", "shape": "box", "center": [.75, .35],
                       "half_size": [.12, .18], "height": 2.2, "center_z": 1.1}],
        "gates": [{"name": "gate", "center": [.5, 0., 1.5],
                    "normal": [1., 0.], "opening": [.45, .45]}],
        "room_bounds_xy": [[-.5, 1.5], [-.5, .5]],
        "maximum_final_cross_track_m": .35, "required_heading_change_deg": 0.,
    }

class HeldoutGeometryTest(unittest.TestCase):
    def test_acceptance_requires_some_success_on_every_course(self):
        rates = {
            course: {"contact_free_completion_rate": rate}
            for course, rate in zip(evaluation.COURSES, (1.0, 1.0, 1.0, 0.0))
        }
        result = evaluation.acceptance(rates, .75, True)
        self.assertFalse(result["accepted"])
        self.assertFalse(result[
            "each_course_contact_free_completion_at_least_40pct"])
        for value in rates.values():
            value["contact_free_completion_rate"] = .8
        self.assertTrue(evaluation.acceptance(rates, .8, True)["accepted"])

    def test_easy_transition_artifacts_match_canonical_generator(self):
        for name in ("gate_obstacle_easy_transition",
                     "gate_obstacle_easy_transition_obstacle_only"):
            fields = generator.COURSES[name]
            course = {
                "format": "tinympc-crazysim-course-v2",
                "name": name,
                "scene": f"vision_{name}.xml",
                **fields,
                "centerline": generator.trajectory_points(fields["trajectory"]),
                "gate_order_required": bool(fields["gates"]),
            }
            manifest = ROOT / f"tools/crazysim_mujoco/courses/{name}.json"
            scene = ROOT / f"tools/crazysim_mujoco/scenes/vision_{name}.xml"
            self.assertEqual(json.loads(manifest.read_text()), course)
            self.assertEqual(scene.read_text(), generator.scene_xml(name, course))

    def test_launch_validity_is_stricter_than_outer_runner_guard(self):
        self.assertTrue(evaluation.calibrated_launch({"launch_time_s": 0.91}))
        self.assertTrue(evaluation.calibrated_launch({"launch_time_s": 1.10}))
        self.assertFalse(evaluation.calibrated_launch({"launch_time_s": 0.89}))
        self.assertFalse(evaluation.calibrated_launch({"launch_time_s": None}))

    def test_slurm_matrix_is_pinned_to_calibrated_host(self):
        script = (ROOT / "tools/crazysim_mujoco/heldout_rooms/heldout_rooms_cpu.sbatch").read_text()
        self.assertIn("#SBATCH --nodelist=a2r-tiger", script)
        self.assertIn('hostname -s)" == a2r-tiger', script)
        self.assertIn('--seed-start "$seed_start"', script)
        self.assertIn('${3:?pass an explicit fresh seed start}', script)

    def test_matrix_supports_fresh_configurable_seed_ranges(self):
        script = (ROOT / "tools/crazysim_mujoco/heldout_rooms/run_heldout_rooms_matrix.sh").read_text()
        self.assertIn('--seed-start) seed_start="$2"', script)
        self.assertIn('exit "$rc"', script)
        self.assertIn('provenance/matrix_tools', script)
        self.assertIn('--matrix-runner-snapshot "$matrix_runner_snapshot"', script)
        self.assertIn('--evaluator-snapshot "$evaluator_snapshot"', script)
        self.assertIn('cmp -s "$matrix_runner" "$matrix_runner_snapshot"', script)
        self.assertIn('cmp -s "$evaluator" "$evaluator_snapshot"', script)

    def test_runner_routes_rooms_through_joint_hm01b0_path(self):
        runner = (ROOT / "tools/crazysim_mujoco/run.sh").read_text()
        for name in ("heldout_room_straight", "heldout_room_circle",
                     "heldout_room_oval", "heldout_room_figure8"):
            self.assertGreaterEqual(runner.count(name), 4)
        self.assertIn('"vision_bridge_sha256": sha256(', runner)
        self.assertIn('"analyzer_sha256": sha256(', runner)

    def test_matrix_tool_snapshots_are_hashed_and_exact(self):
        with tempfile.TemporaryDirectory() as temporary:
            tool_dir = Path(temporary)
            runner = tool_dir / "run_heldout_rooms_matrix.sh"
            runner.write_text("#!/bin/sh\n")
            evaluator_snapshot = tool_dir / "evaluate_heldout_rooms.py"
            evaluator_snapshot.write_bytes(EVAL.read_bytes())
            errors = []
            tools = evaluation.load_matrix_tool_provenance(
                runner, evaluator_snapshot, errors)
            self.assertEqual(errors, [])
            self.assertTrue(evaluation.valid_digest(
                tools["matrix_runner"]["sha256"]))
            self.assertEqual(tools["evaluator"]["sha256"],
                             evaluation.digest(EVAL))
            self.assertFalse(evaluation.valid_digest(None))

    def test_run_without_runtime_tool_hashes_is_incomplete(self):
        with tempfile.TemporaryDirectory() as temporary:
            run = Path(temporary)
            (run / "run_config.json").write_text(json.dumps({
                "course": "heldout_room_straight",
                "acceptance_contract": {},
            }))
            (run / "summary.json").write_text("{}")
            result = evaluation.row(
                run, ROOT / "tools/crazysim_mujoco/courses", None)
            self.assertFalse(result["complete"])
            self.assertIn("missing or invalid runtime tool hash vision_bridge_sha256",
                          result["errors"])
            self.assertIn("missing or invalid runtime tool hash analyzer_sha256",
                          result["errors"])

    def test_gates_are_header_anchored_and_tangent_aligned(self):
        for manifest in sorted((ROOT / "tools/crazysim_mujoco/courses").glob("heldout_room_*.json")):
            data = json.loads(manifest.read_text()); scene = ROOT / "tools/crazysim_mujoco/scenes" / data["scene"]
            self.assertEqual(evaluation.geometry_errors(data, scene.read_text()), [])
            self.assertEqual(data["gates"][0]["scale"], 2.0)
            self.assertEqual(data["gates"][0]["opening"], [0.90, 0.90])
            normal = data["gates"][0]["normal"]
            self.assertAlmostEqual(math.hypot(*normal), 1.0, places=6)

    def test_oval_gate_is_disclosed_offset_on_low_curvature_segment(self):
        manifest = ROOT / "tools/crazysim_mujoco/courses/heldout_room_oval.json"
        data = json.loads(manifest.read_text())
        source = data["gate_trajectory_source"]
        self.assertEqual(source["index"], 375)
        self.assertEqual(source["lateral_side"], "right")
        self.assertAlmostEqual(source["lateral_offset_m"], 0.20)
        header = ROOT / data["source_trajectory"]
        rows = []
        import re
        for line in header.read_text().splitlines():
            values = re.findall(
                r"[-+]?\d*\.?\d+(?:e[-+]?\d+)?(?=f)", line, re.I)
            if len(values) >= 13:
                rows.append([float(value) for value in values])
        xs = [row[0] for row in rows]
        ys = [row[1] for row in rows]
        major_radius = 0.5 * (max(xs) - min(xs))
        minor_radius = 0.5 * (max(ys) - min(ys))
        # The sampled header does not necessarily land on the analytic extrema
        # exactly, so four decimal places is the appropriate discretization
        # tolerance for the regenerated 50 Hz path.
        self.assertAlmostEqual(major_radius, 1.0, places=4)
        self.assertAlmostEqual(minor_radius, 0.75, places=4)
        # Ellipse peak curvature a/b^2 falls from the former 4.0 1/m to
        # 1.778 1/m, while the midpoint gate uses the minimum b/a^2.
        self.assertLessEqual(major_radius / minor_radius**2, 1.78)
        gate_curvature = minor_radius / major_radius**2
        self.assertAlmostEqual(gate_curvature, 0.75, places=4)
        self.assertAlmostEqual(math.dist(
            rows[source["index"]][:2], data["gates"][0]["center"][:2]),
            0.20, places=6)
        self.assertEqual(evaluation.geometry_errors(
            data, (ROOT / "tools/crazysim_mujoco/scenes" /
                   data["scene"]).read_text()), [])

    def test_gate_header_provenance_fails_closed(self):
        manifest = ROOT / "tools/crazysim_mujoco/courses/heldout_room_straight.json"
        data = json.loads(manifest.read_text())
        scene = ROOT / "tools/crazysim_mujoco/scenes" / data["scene"]
        data["gates"][0]["center"][1] += .30
        self.assertIn("gate center deviates from derived trajectory offset",
                      evaluation.geometry_errors(data, scene.read_text()))

    def test_transition_scenes_use_the_visual_newbee_room_contract(self):
        courses = ROOT / "tools/crazysim_mujoco/courses"
        scenes = ROOT / "tools/crazysim_mujoco/scenes"
        for name in ("gate_obstacle_easy_transition",
                     "gate_obstacle_easy_transition_obstacle_only"):
            data = json.loads((courses / f"{name}.json").read_text())
            scene = scenes / data["scene"]
            tree = ET.parse(scene)
            text = scene.read_text()
            self.assertIn("newbeedrone_gate_front_rgba_v1.png", text)
            self.assertIn("off_white_painted_plaster.png", text)
            self.assertGreaterEqual(text.count('name="strip_light_'), 5)
            obstacle = tree.find(".//geom[@name='post_gate_box']")
            self.assertIsNotNone(obstacle)
            self.assertEqual([float(value) for value in obstacle.get("pos").split()],
                             [*data["obstacles"][0]["center"], 1.1])
            gate_body = tree.find(".//body[@name='expected_gate']")
            if name.endswith("obstacle_only"):
                self.assertIsNone(gate_body)
            else:
                self.assertIsNotNone(gate_body)
                self.assertEqual(data["gates"][0]["opening"], [1.0, 1.0])
                visual_meshes = gate_body.findall("geom[@type='mesh']")
                self.assertEqual(len(visual_meshes), 4)
                self.assertEqual(len(gate_body.findall("geom[@type='box']")), 4)

    def test_completion_waits_for_ordered_gate_and_reports_interval_clearance(self):
        summary = analyze.build_summary(synthetic_gate_data(), 0., course=synthetic_gate_course())
        self.assertTrue(summary["course_gates_passed_in_order"])
        self.assertTrue(summary["course_completion_after_ordered_gates"])
        self.assertAlmostEqual(summary["course_completion_time_s"], 3.0)
        self.assertGreaterEqual(summary["course_validation_interval_start_time_s"], 1.0)
        self.assertIsNotNone(summary["course_gate_clearance_min_m"])
        self.assertIsNotNone(summary["course_wall_clearance_min_m"])

    def test_initial_finish_sphere_cannot_complete_without_gate(self):
        summary = analyze.build_summary(synthetic_gate_data(cross_gate=False), 0., course=synthetic_gate_course())
        self.assertFalse(summary["course_gates_passed_in_order"])
        self.assertFalse(summary["course_pass_point_reached"])
        self.assertFalse(summary["course_success"])

if __name__ == "__main__": unittest.main()
