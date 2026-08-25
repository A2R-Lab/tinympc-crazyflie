#!/usr/bin/env python3
"""Evaluate the fixed 4 x 5 fake-room held-out matrix without launching SITL."""
from __future__ import annotations

import argparse, csv, hashlib, json, math, re
import xml.etree.ElementTree as ET
from pathlib import Path

COURSES = ("heldout_room_straight", "heldout_room_circle", "heldout_room_oval", "heldout_room_figure8")
REQUIRED = ("run_config.json", "summary.json", "state.csv", "vision.csv", "firmware.log")
LAUNCH_TOLERANCE_S = 0.10
FROZEN = {"generated_model_sha256":"2a92dadbad53e56a857947903fa02b11970a8236360a78997392d5d35e23dba6", "bank_header_sha256":"5d5db163f21cbb9fc2be066d7d67a31de1447e4753dcda0d6b2d19845292bdd7", "bank_provenance_sha256":"125ba361148397cdcc7f00d3389c7adabe13db477858b58095d6ec3fc03bba14"}
RUNTIME_TOOL_HASHES = ("vision_bridge_sha256", "analyzer_sha256")

def digest(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as f:
        for b in iter(lambda: f.read(1 << 20), b""):
            h.update(b)
    return h.hexdigest()

def valid_digest(value) -> bool:
    return (isinstance(value, str) and len(value) == 64
            and all(character in "0123456789abcdef" for character in value))

def load_matrix_tool_provenance(runner_snapshot: Path, evaluator_snapshot: Path,
                                errors: list[str]) -> dict:
    """Hash the exact held-out orchestration and evaluator scripts."""
    result = {}
    for key, path, expected_name in (
            ("matrix_runner", runner_snapshot, "run_heldout_rooms_matrix.sh"),
            ("evaluator", evaluator_snapshot, "evaluate_heldout_rooms.py")):
        if path.name != expected_name or not path.is_file():
            errors.append("invalid " + key + " provenance snapshot")
            continue
        result[key] = {"name": path.name, "sha256": digest(path)}
    if ("evaluator" in result and
            result["evaluator"]["sha256"] != digest(Path(__file__).resolve())):
        errors.append("evaluator differs from retained provenance snapshot")
    return result

def number(value):
    try:
        value = float(value)
        return value if math.isfinite(value) else None
    except (TypeError, ValueError): return None

def calibrated_launch(summary, requested_s=1.0):
    actual = number(summary.get("launch_time_s"))
    return actual is not None and abs(actual - requested_s) <= LAUNCH_TOLERANCE_S + 1e-12

def p95(values):
    values = sorted(values)
    if not values: return None
    n = .95 * (len(values) - 1); lo, hi = int(n), math.ceil(n)
    return values[lo] + (values[hi] - values[lo]) * (n - lo)

def geometry_errors(data, scene_text):
    """Reject a swapped room or an undisclosed mismatch in easier-course geometry."""
    errors = []
    gates, obstacles, bounds = data.get("gates"), data.get("obstacles"), data.get("room_bounds_xy")
    if (not isinstance(gates, list) or len(gates) != 1
            or gates[0].get("scale") != 2.0
            or gates[0].get("opening") != [.90, .90]):
        errors.append("expected one disclosed 2x NewBee gate with 0.90m opening")
    else:
        normal = gates[0].get("normal")
        if not isinstance(normal, list) or len(normal) != 2 or abs(math.hypot(*normal) - 1.0) > .001: errors.append("gate normal is not unit tangent")
    if (not isinstance(obstacles, list) or len(obstacles) != 1
            or obstacles[0].get("half_size") != [.12, .18]
            or number(obstacles[0].get("height")) != 2.2
            or number(obstacles[0].get("center_z")) != 1.1):
        errors.append("unexpected tall blocking obstacle geometry")
    if number(data.get("pass_radius_m")) != .30:
        errors.append("expected 0.30m completion radius")
    if "minimum_dodge_encounters" in data:
        errors.append("unenforced minimum_dodge_encounters contract")
    if not (isinstance(bounds, list) and len(bounds) == 2 and all(len(x) == 2 and x[0] < x[1] for x in bounds)): errors.append("invalid room bounds")
    for name in ("tangent_gate", "tangent_gate_visual_top", "tangent_gate_visual_bottom",
                 "tangent_gate_visual_left", "tangent_gate_visual_right",
                 "tangent_gate_top", "tangent_gate_bottom", "tangent_gate_left",
                 "tangent_gate_right", "easy_obstacle", "room_wall_west",
                 "room_wall_east", "room_wall_north", "room_wall_south",
                 "generic_ceiling", "generic_ceiling_strip"):
        if f'name="{name}"' not in scene_text: errors.append("scene lacks " + name)
    for asset in ("newbeedrone_gate_front_rgba_v1.png", "newbeedrone_gate_top.obj",
                  "newbeedrone_gate_bottom.obj", "newbeedrone_gate_left.obj",
                  "newbeedrone_gate_right.obj", "off_white_painted_plaster.png"):
        if asset not in scene_text: errors.append("scene lacks visual asset " + asset)
    if scene_text.count('name="generic_strip_light_') < 4:
        errors.append("scene lacks four ceiling strip lights")
    try:
        gate_body = ET.fromstring(scene_text).find(".//body[@name='tangent_gate']")
        scene_position = ([float(value) for value in gate_body.get("pos").split()]
                          if gate_body is not None else [])
        scene_euler = ([float(value) for value in gate_body.get("euler").split()]
                       if gate_body is not None else [])
        gate = gates[0] if isinstance(gates, list) and gates else {}
        expected_position = gate.get("center", [])
        normal = gate.get("normal", [])
        expected_yaw = math.degrees(math.atan2(normal[1], normal[0]))
        if (len(scene_position) != 3 or len(expected_position) != 3
                or math.dist(scene_position, expected_position) > 1e-6
                or len(scene_euler) != 3
                or abs(math.remainder(scene_euler[2] - expected_yaw, 360.0)) > 1e-4):
            errors.append("scene gate pose disagrees with manifest")
        if gate_body is not None:
            visual_meshes = {geom.get("mesh") for geom in gate_body.findall("geom[@type='mesh']")}
            if visual_meshes != {
                    "newbeedrone_gate_top_easy", "newbeedrone_gate_bottom_easy",
                    "newbeedrone_gate_left_easy", "newbeedrone_gate_right_easy"}:
                errors.append("scene does not use the disclosed 2x NewBee visual meshes")
            expected_rails = {
                "tangent_gate_left": ([0.0, -0.558, 0.0], [0.025, 0.108, 0.45]),
                "tangent_gate_right": ([0.0, 0.558, 0.0], [0.025, 0.108, 0.45]),
                "tangent_gate_top": ([0.0, 0.0, 0.558], [0.025, 0.666, 0.108]),
                "tangent_gate_bottom": ([0.0, 0.0, -0.558], [0.025, 0.666, 0.108]),
            }
            rails = {geom.get("name"): geom for geom in gate_body.findall("geom[@type='box']")}
            for name, (position, size) in expected_rails.items():
                geom = rails.get(name)
                try:
                    actual_position = [float(value) for value in geom.get("pos").split()]
                    actual_size = [float(value) for value in geom.get("size").split()]
                except (AttributeError, TypeError, ValueError):
                    errors.append("scene lacks readable scaled collision rail " + name)
                    continue
                if math.dist(actual_position, position) > 1e-9 or math.dist(actual_size, size) > 1e-9:
                    errors.append("scene collision rail disagrees with disclosed 2x scale: " + name)
    except (ET.ParseError, TypeError, ValueError, IndexError, AttributeError):
        errors.append("scene gate pose is unreadable")
    source = data.get("gate_trajectory_source", {}); header = Path(__file__).resolve().parents[3] / str(data.get("source_trajectory", ""))
    if not isinstance(source, dict) or not header.is_file() or digest(header) != source.get("header_sha256"): errors.append("gate trajectory header provenance mismatch")
    else:
        rows = []
        for line in header.read_text().splitlines():
            values = re.findall(r"[-+]?\d*\.?\d+(?:e[-+]?\d+)?(?=f)", line, re.I)
            if len(values) >= 13: rows.append([float(x) for x in values])
        index = source.get("index")
        if not isinstance(index, int) or not 0 <= index < len(rows): errors.append("invalid gate trajectory index")
        else:
            point, velocity, gate = rows[index], rows[index][7:9], gates[0]
            distance = math.dist(point[:2], gate["center"][:2]); speed = math.hypot(*velocity)
            dot = (velocity[0] * gate["normal"][0] + velocity[1] * gate["normal"][1]) / speed if speed else -1
            if distance > .01: errors.append("gate center deviates from trajectory header")
            if dot < .999: errors.append("gate normal is not header tangent")
        obstacle_source = data.get("obstacle_trajectory_source", {})
        completion_source = data.get("completion_trajectory_source", {})
        obstacle = obstacles[0] if isinstance(obstacles, list) and obstacles else {}
        if not isinstance(obstacle_source, dict) or not isinstance(completion_source, dict):
            errors.append("missing ordered obstacle/completion provenance")
        else:
            oi, ci = obstacle_source.get("index"), completion_source.get("index")
            same_header = (obstacle_source.get("header_sha256") == source.get("header_sha256")
                           and completion_source.get("header_sha256") == source.get("header_sha256"))
            if not (isinstance(index, int) and isinstance(oi, int) and isinstance(ci, int)
                    and 0 <= oi < len(rows) and 0 <= ci < len(rows)
                    and index < oi < ci and same_header):
                errors.append("gate/obstacle/completion provenance is not strictly ordered")
            else:
                side, offset = obstacle_source.get("lateral_side"), number(obstacle_source.get("lateral_offset_m"))
                if side not in ("left", "right") or offset is None or abs(offset - .25) > 1e-9:
                    errors.append("invalid obstacle lateral provenance")
                else:
                    velocity = rows[oi][7:9]; speed = math.hypot(*velocity)
                    if speed <= 0.0:
                        errors.append("zero obstacle-source tangent")
                    else:
                        tx, ty = velocity[0] / speed, velocity[1] / speed
                        nx, ny = -ty, tx
                        if side == "right": nx, ny = -nx, -ny
                        expected = (rows[oi][0] + offset * nx, rows[oi][1] + offset * ny)
                        actual = obstacle.get("center", [None, None])
                        if (not isinstance(actual, list) or len(actual) != 2
                                or any(number(v) is None for v in actual)
                                or math.dist(expected, actual) > 1e-6):
                            errors.append("obstacle center deviates from derived tangent offset")
                        else:
                            # The inflated 0.10m vehicle footprint must be blocked
                            # by the nominal reference; otherwise a contact-free run
                            # need not demonstrate a lateral transition at all.
                            dx = max(abs(rows[oi][0] - actual[0]) - .12, 0.0)
                            dy = max(abs(rows[oi][1] - actual[1]) - .18, 0.0)
                            if math.hypot(dx, dy) - .10 >= 0.0:
                                errors.append("nominal path does not intrude into inflated obstacle")
                if math.dist(rows[ci][:2], data.get("pass_point", [])) > 1e-6:
                    errors.append("completion point deviates from trajectory header")
    return errors

def row(run: Path, course_dir: Path, frozen_bundle_hash: str | None):
    errors = ["missing " + name for name in REQUIRED if not (run / name).is_file()]
    config = summary = {}
    try: config = json.loads((run / "run_config.json").read_text())
    except Exception as exc: errors.append("invalid run_config.json: " + str(exc))
    try: summary = json.loads((run / "summary.json").read_text())
    except Exception as exc: errors.append("invalid summary.json: " + str(exc))
    course = config.get("course")
    if course not in COURSES: errors.append("unexpected course")
    trajectory = {"heldout_room_straight":"straight_9m", "heldout_room_circle":"circle", "heldout_room_oval":"oval", "heldout_room_figure8":"figure8"}.get(course)
    expected = {"trajectory":trajectory, "vision_adapter":"joint_gate_rl", "reference_mode":"progress", "actuator_lti":True, "rate_cascade":False, "flowdeck_enabled":False, "stop_on_contact":True, "launch_time_s":1.0, "duration_s":30.0, "progress_speed_mps":.5, "progress_entry_acceleration_mps2":1.0, "progress_terminal_deceleration_mps2":1.5, "progress_reward_weight":.40, "realtime_factor":.2, "firmware_time_factor":.1047352488}
    for key, value in expected.items():
        actual = config.get(key)
        if isinstance(value, float):
            if number(actual) is None or abs(number(actual) - value) > 1e-6: errors.append("run config mismatch " + key)
        elif actual != value: errors.append("run config mismatch " + key)
    if not calibrated_launch(summary, expected["launch_time_s"]):
        errors.append("motor launch is outside calibrated +/-0.10s window")
    calibration = config.get("camera_calibration", {})
    if (calibration.get("model") != "Himax HM01B0" or
            calibration.get("bridge_calibration_transform_enabled") is not True or
            calibration.get("poc_overscan_render_resolution") != [192, 192] or
            calibration.get("poc_transform") !=
            "principal_point+plumb_bob+isaac_gray_response+seeded_noise" or
            calibration.get("poc_sensor_seed") != config.get("random_seed")):
        errors.append("HM01B0/Isaac camera transform is not active")
    if frozen_bundle_hash is None or config.get("vision_model", {}).get("bundle_manifest_sha256") != frozen_bundle_hash: errors.append("frozen bundle manifest mismatch")
    contract = config.get("acceptance_contract", {})
    if number(contract.get("gate_corner_span_m")) != 1.11:
        errors.append("scaled gate metric span compile contract mismatch")
    runtime_tool_hashes = {}
    for key in RUNTIME_TOOL_HASHES:
        value = contract.get(key)
        if not valid_digest(value):
            errors.append("missing or invalid runtime tool hash " + key)
        runtime_tool_hashes[key] = value
    for key, value in FROZEN.items():
        if contract.get(key) != value: errors.append("frozen invariant mismatch " + key)
    manifest = course_dir / f"{course}.json"
    scene_hash = None
    if manifest.is_file():
        expected_manifest = digest(manifest)
        if config.get("course_manifest_sha256") != expected_manifest: errors.append("course manifest hash mismatch")
        manifest_data = json.loads(manifest.read_text()); scene_name = manifest_data.get("scene")
        scene = course_dir.parent / "scenes" / str(scene_name)
        if not scene.is_file(): errors.append("missing checked-in scene")
        else:
            scene_hash = digest(scene)
            if config.get("course_scene_sha256") != scene_hash: errors.append("course scene hash mismatch")
            errors.extend(geometry_errors(manifest_data, scene.read_text()))
    else: errors.append("missing checked-in manifest")
    samples = []
    try:
        with (run / "vision.csv").open() as f: samples = [number(x.get("inference_ms")) for x in csv.DictReader(f)]
        samples = [x for x in samples if x is not None]
        if not samples: errors.append("no finite inference samples")
    except Exception as exc: errors.append("invalid vision.csv: " + str(exc))
    gate_results = summary.get("course_gate_results")
    gate = summary.get("course_gates_passed_in_order") is True and isinstance(gate_results, list) and bool(gate_results) and all(
        isinstance(x, dict) and x.get("crossed") is True for x in gate_results)
    if not isinstance(gate_results, list) or not gate_results or not all(isinstance(x, dict) and isinstance(x.get("crossed"), bool) for x in gate_results): errors.append("malformed physical gate results")
    gate_times = [number(x.get("time_s")) for x in gate_results if isinstance(x, dict)] if isinstance(gate_results, list) else []
    interval_start = number(summary.get("course_validation_interval_start_time_s"))
    interval_end = number(summary.get("course_validation_interval_end_time_s"))
    if (summary.get("course_completion_after_ordered_gates") is not True
            or interval_start is None or interval_end is None
            or interval_start > interval_end
            or not gate_times or any(value is None for value in gate_times)
            or interval_start + .021 < max(gate_times)):
        errors.append("invalid ordered gate-to-completion validation interval")
    contacts = number(summary.get("contact_count_max"))
    contact_free = contacts == 0 and summary.get("crashed") is False
    clearance = {"obstacle_m": number(summary.get("course_obstacle_clearance_min_m")),
                 "wall_m": number(summary.get("course_wall_clearance_min_m")),
                 "gate_m": number(summary.get("course_gate_clearance_min_m"))}
    if any(v is None for v in clearance.values()): errors.append("missing classified clearance")
    speed = number(summary.get("course_mean_horizontal_speed_active_mps"))
    if speed is None: errors.append("missing finite speed")
    try:
        with (run / "state.csv").open() as stream: state = list(csv.DictReader(stream))
        final = number(state[-1].get("time_s")) if state else None
        if final is None or number(summary.get("duration_s")) is None or abs(final - number(summary.get("duration_s"))) > .02: errors.append("state/summary duration mismatch")
    except (OSError, csv.Error): errors.append("invalid state.csv")
    if number(summary.get("launch_time_s")) is None or abs(number(summary.get("launch_time_s")) - 1.0) > .10: errors.append("actual launch timing invalid")
    camera_common = dict(calibration) if isinstance(calibration, dict) else {}
    # Seeded sensor noise is expected to differ between held-out trials; every
    # other camera/calibration field must remain common across the matrix.
    camera_common.pop("poc_sensor_seed", None)
    return {"run": str(run), "course": course, "seed": config.get("random_seed"), "complete": not errors,
            "errors": errors, "gate_pass": gate, "contact_free_completion": contact_free and summary.get("course_success") is True and not errors,
            "clearance": clearance, "speed_mps": speed,
            "inference_p95_ms": p95(samples), "scene_sha256": scene_hash,
            "common_hashes": {
                "controller_source_sha256": config.get("controller_source_sha256"),
                "runner_sha256": config.get("runner_sha256"),
                "runtime_tool_hashes": runtime_tool_hashes,
                "camera_calibration": camera_common,
            },
            "firmware_binary_sha256": contract.get("firmware_binary_sha256")}

def acceptance(by_course, aggregate, valid):
    per_course_floor = all(
        value.get("contact_free_completion_rate") is not None and
        value["contact_free_completion_rate"] >= .40
        for value in by_course.values())
    return {
        "no_invalid_or_missing_trials": valid,
        "each_course_contact_free_completion_at_least_40pct": per_course_floor,
        "aggregate_contact_free_completion_at_least_70pct": aggregate >= .70,
        "aggregate_contact_free_completion_rate": aggregate,
        "accepted": valid and per_course_floor and aggregate >= .70,
    }

def main() -> int:
    p = argparse.ArgumentParser(description=__doc__); p.add_argument("--matrix-root", type=Path, required=True); p.add_argument("--out-dir", type=Path, required=True); p.add_argument("--seed-start", type=int, default=4801); p.add_argument("--matrix-runner-snapshot", type=Path, required=True); p.add_argument("--evaluator-snapshot", type=Path, required=True); args = p.parse_args()
    if args.seed_start < 0:
        p.error("seed start must be nonnegative")
    course_dir = Path(__file__).resolve().parents[1] / "courses"; rows = []
    try: frozen_bundle_hash = digest(args.matrix_root / "provenance/candidate_bundle/bundle.json")
    except OSError: frozen_bundle_hash = None
    for course in COURSES:
        runs = sorted(args.matrix_root.glob(course + "_seed*"))
        if len(runs) != 5: rows += [{"course": course, "complete": False, "errors": ["expected exactly five runs"]}]
        rows += [row(run, course_dir, frozen_bundle_hash) for run in runs]
    by = {}
    for course in COURSES:
        items = [x for x in rows if x.get("course") == course and "seed" in x]
        by[course] = {"trials": len(items), "gate_pass_rate": sum(x["gate_pass"] for x in items) / len(items) if items else None,
          "contact_free_completion_rate": sum(x["contact_free_completion"] for x in items) / len(items) if items else None,
          "minimum_clearance_m": {k: min((x["clearance"][k] for x in items if x["clearance"].get(k) is not None), default=None) for k in ("obstacle_m", "wall_m", "gate_m")},
          "mean_speed_mps": sum(x["speed_mps"] for x in items if x["speed_mps"] is not None) / max(1, sum(x["speed_mps"] is not None for x in items)),
          "max_inference_p95_ms": max((x["inference_p95_ms"] for x in items if x["inference_p95_ms"] is not None), default=None)}
    expected_seeds = set(range(args.seed_start, args.seed_start + 5))
    status_errors = []
    try:
        records = [line.split("\t") for line in (args.matrix_root / "matrix_status.tsv").read_text().splitlines()]
        expected = {(course, str(seed)) for course in COURSES for seed in expected_seeds}
        observed = {(x[0], x[1]) for x in records if len(x) == 3}
        if len(records) != 20 or any(len(x) != 3 or x[2] != "0" for x in records) or observed != expected: status_errors.append("invalid matrix_status.tsv")
    except OSError: status_errors.append("missing matrix_status.tsv")
    provenance_errors = []
    matrix_tools = load_matrix_tool_provenance(
        args.matrix_runner_snapshot, args.evaluator_snapshot,
        provenance_errors)
    if (set(matrix_tools) != {"matrix_runner", "evaluator"} or
            any(not valid_digest(value.get("sha256"))
                for value in matrix_tools.values())):
        provenance_errors.append("missing or invalid matrix tool provenance")
    try:
        bundle = args.matrix_root / "provenance/candidate_bundle/bundle.json"; policy = json.loads(bundle.read_text())["artifacts"]["policy_onnx"]
        policy_path = bundle.parent / policy["path"]
        if json.loads(bundle.read_text()).get("runtime_adapter") != "joint_gate_rl" or digest(policy_path) != policy["sha256"]: provenance_errors.append("invalid frozen model provenance")
    except (OSError, KeyError, TypeError, json.JSONDecodeError): provenance_errors.append("missing frozen model provenance")
    valid = len(rows) == 20 and not status_errors and not provenance_errors and all(x.get("complete") for x in rows) and all(len([x for x in rows if x.get("course") == c and "seed" in x]) == 5 and {x.get("seed") for x in rows if x.get("course") == c and "seed" in x} == expected_seeds for c in COURSES)
    aggregate = sum(x.get("contact_free_completion") is True for x in rows if "seed" in x) / 20
    hashes = [x.get("common_hashes") for x in rows if x.get("common_hashes")]
    common = hashes[0] if hashes and all(x == hashes[0] for x in hashes) else None
    if common is None:
        status_errors.append("non-common controller/runner/runtime-tool/camera provenance")
        valid = False
    firmware_by_course = {}
    for course in COURSES:
        values = {x.get("firmware_binary_sha256") for x in rows
                  if x.get("course") == course and "seed" in x}
        if len(values) != 1 or None in values:
            status_errors.append("non-common firmware binary within " + course)
            valid = False
        else:
            firmware_by_course[course] = next(iter(values))
    report = {"format":"heldout-fake-room-evaluation-v1", "courses":by, "runs":rows, "common_hashes":common, "matrix_tool_provenance":matrix_tools, "firmware_binary_sha256_by_course":firmware_by_course, "matrix_errors":status_errors + provenance_errors, "acceptance":acceptance(by, aggregate, valid)}
    args.out_dir.mkdir(parents=True, exist_ok=True); (args.out_dir / "evaluation.json").write_text(json.dumps(report, indent=2) + "\n")
    lines=["# Held-out fake-room evaluation", "", f"Accepted: **{'yes' if report['acceptance']['accepted'] else 'no'}**.", "", "## Matrix tool provenance", "", f"- runner SHA-256: {matrix_tools.get('matrix_runner', {}).get('sha256')}", f"- evaluator SHA-256: {matrix_tools.get('evaluator', {}).get('sha256')}", "", "| course | gate pass | contact-free completion | obstacle clearance | wall clearance | gate clearance | speed | p95 inference |", "| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |"]
    for c, x in by.items(): lines.append(f"| {c} | {x['gate_pass_rate']} | {x['contact_free_completion_rate']} | {x['minimum_clearance_m']['obstacle_m']} | {x['minimum_clearance_m']['wall_m']} | {x['minimum_clearance_m']['gate_m']} | {x['mean_speed_mps']} | {x['max_inference_p95_ms']} |")
    (args.out_dir / "evaluation.md").write_text("\n".join(lines) + "\n")
    print(json.dumps(report["acceptance"], indent=2))
    return 0 if report["acceptance"]["accepted"] else 1

if __name__ == "__main__": raise SystemExit(main())
