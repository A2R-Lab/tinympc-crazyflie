#!/usr/bin/env python3
"""Strict evidence reader for joint gate/obstacle MuJoCo matrices.

It does not launch simulations. Gate success comes only from analyzer-produced
physical course-plane results; detector output is never promoted to a pass.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any


GROUPS = (
    "candidate_gate_obstacle",
    "candidate_obstacle_only",
    "baseline_obstacle_only",
)
COURSES = {
    "candidate_gate_obstacle": "gate_obstacle_easy_transition",
    "candidate_obstacle_only": "gate_obstacle_easy_transition_obstacle_only",
    "baseline_obstacle_only": "gate_obstacle_easy_transition_obstacle_only",
}
CANDIDATE_ADAPTER = "joint_gate_rl"
BASELINE_ADAPTER = "hybrid_rl"
BASELINE_POLICY_SHA256 = (
    "291d1de3a7152f09cc2e96f4a6973d322c95bade4e5c8249c7bc14f7b656187e"
)
FROZEN_INVARIANTS = {
    "acceptance_contract.generated_model_sha256": "2a92dadbad53e56a857947903fa02b11970a8236360a78997392d5d35e23dba6",
    "acceptance_contract.bank_header_sha256": "5d5db163f21cbb9fc2be066d7d67a31de1447e4753dcda0d6b2d19845292bdd7",
    "acceptance_contract.bank_provenance_sha256": "125ba361148397cdcc7f00d3389c7adabe13db477858b58095d6ec3fc03bba14",
}
RUNTIME_TOOL_HASH_KEYS = (
    "acceptance_contract.vision_bridge_sha256",
    "acceptance_contract.analyzer_sha256",
)
REQUIRED = ("run_config.json", "summary.json", "state.csv", "vision.csv", "firmware.log")
COURSE_DIR = Path(__file__).resolve().parents[1] / "courses"
SCENE_DIR = Path(__file__).resolve().parents[1] / "scenes"


def finite(value: Any) -> float | None:
    try:
        result = float(value)
    except (TypeError, ValueError):
        return None
    return result if math.isfinite(result) else None


def boolean(value: Any) -> bool | None:
    return value if isinstance(value, bool) else None


def read_json(path: Path, errors: list[str]) -> dict[str, Any] | None:
    try:
        value = json.loads(path.read_text())
        if not isinstance(value, dict):
            raise ValueError("expected a JSON object")
        return value
    except (OSError, json.JSONDecodeError, ValueError) as exc:
        errors.append(f"invalid or missing {path.name}: {exc}")
        return None


def dotted(value: Any, key: str) -> Any:
    for part in key.split("."):
        if not isinstance(value, dict):
            return None
        value = value.get(part)
    return value


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def valid_sha256(value: Any) -> bool:
    return (isinstance(value, str) and len(value) == 64 and
            all(character in "0123456789abcdef" for character in value))


def load_matrix_tool_provenance(
        runner_snapshot: Path, evaluator_snapshot: Path,
        errors: list[str]) -> dict[str, dict[str, str]]:
    """Hash the exact matrix scripts retained beside terminal evidence."""
    result: dict[str, dict[str, str]] = {}
    for key, path, expected_name in (
            ("matrix_runner", runner_snapshot, "run_joint_gate_obstacle_matrix.sh"),
            ("evaluator", evaluator_snapshot, "evaluate_joint_gate_obstacle.py")):
        if path.name != expected_name or not path.is_file():
            errors.append(f"invalid {key} provenance snapshot")
            continue
        result[key] = {"name": path.name, "sha256": sha256(path)}
    current_evaluator = Path(__file__).resolve()
    if ("evaluator" in result and
            result["evaluator"]["sha256"] != sha256(current_evaluator)):
        errors.append("evaluator differs from retained provenance snapshot")
    return result


def validate_newbee_scene_geometry(
        scene_path: Path, expected_gate: dict[str, Any] | None,
        errors: list[str]) -> None:
    """Bind manifest opening and pose to the rendered mesh and colliders."""
    try:
        root = ET.parse(scene_path).getroot()
    except (OSError, ET.ParseError) as exc:
        errors.append(f"unreadable course scene XML: {exc}")
        return
    body = root.find(".//body[@name='expected_gate']")
    if expected_gate is None:
        if body is not None:
            errors.append("obstacle-only scene unexpectedly contains gate body")
        return
    if body is None:
        errors.append("course scene lacks expected gate body")
        return
    try:
        position = [float(value) for value in body.get("pos", "").split()]
    except ValueError:
        position = []
    if len(position) != 3 or math.dist(position, expected_gate["center"]) > 1e-9:
        errors.append("course scene gate pose disagrees with manifest")
    opening = float(expected_gate["opening"][0])
    scale = opening / 0.45
    expected_mesh_scale = [1.0, 0.9 * scale, 0.9 * scale]
    meshes = {element.get("name"): element for element in root.findall(".//asset/mesh")}
    for suffix in ("top", "bottom", "left", "right"):
        mesh = meshes.get("newbeedrone_gate_" + suffix)
        try:
            actual = [float(value) for value in mesh.get("scale", "").split()]
        except (AttributeError, ValueError):
            actual = []
        if len(actual) != 3 or math.dist(actual, expected_mesh_scale) > 1e-9:
            errors.append("course scene visual mesh scale mismatch: " + suffix)
    rail_center = 0.279 * scale
    rail_half = 0.054 * scale
    opening_half = 0.225 * scale
    full_half = 0.333 * scale
    expected_rails = {
        "expected_gate_left": ([0.0, -rail_center, 0.0], [0.025, rail_half, opening_half]),
        "expected_gate_right": ([0.0, rail_center, 0.0], [0.025, rail_half, opening_half]),
        "expected_gate_top": ([0.0, 0.0, rail_center], [0.025, full_half, rail_half]),
        "expected_gate_bottom": ([0.0, 0.0, -rail_center], [0.025, full_half, rail_half]),
    }
    rails = {element.get("name"): element for element in body.findall("geom[@type='box']")}
    for name, (expected_position, expected_size) in expected_rails.items():
        rail = rails.get(name)
        try:
            actual_position = [float(value) for value in rail.get("pos", "").split()]
            actual_size = [float(value) for value in rail.get("size", "").split()]
        except (AttributeError, ValueError):
            actual_position, actual_size = [], []
        if (len(actual_position) != 3 or len(actual_size) != 3 or
                math.dist(actual_position, expected_position) > 1e-9 or
                math.dist(actual_size, expected_size) > 1e-9):
            errors.append("course scene collision rail mismatch: " + name)


def validate_course_contract(group: str, config: dict[str, Any],
                             errors: list[str]) -> None:
    """Bind each row to the documented physical easier-course revision."""
    name = COURSES[group]
    calibration = config.get("camera_calibration", {})
    if (not isinstance(calibration, dict) or
            calibration.get("model") != "Himax HM01B0" or
            calibration.get("bridge_calibration_transform_enabled") is not True or
            calibration.get("poc_overscan_render_resolution") != [192, 192] or
            calibration.get("poc_transform") !=
            "principal_point+plumb_bob+isaac_gray_response+seeded_noise" or
            calibration.get("poc_sensor_seed") != config.get("random_seed")):
        errors.append("HM01B0/Isaac camera transform is not active")
    manifest_path = COURSE_DIR / f"{name}.json"
    manifest = read_json(manifest_path, errors)
    if manifest is None:
        return
    if config.get("course_manifest_sha256") != sha256(manifest_path):
        errors.append("course manifest provenance mismatch")
    obstacles = manifest.get("obstacles")
    expected_obstacle = {
        "name": "post_gate_box", "shape": "box", "center": [6.2, 0.6],
        "half_size": [0.15, 0.4], "height": 2.2, "center_z": 1.1,
    }
    if obstacles != [expected_obstacle]:
        errors.append("unexpected easier-course obstacle geometry")
    expected_gates = ([] if name.endswith("obstacle_only") else [{
        "name": "expected_gate", "center": [4.0, 0.3, 1.5],
        "normal": [1.0, 0.0], "opening": [1.0, 1.0],
    }])
    if manifest.get("gates") != expected_gates:
        errors.append("unexpected physical NewBee gate geometry")
    span = finite(dotted(config, "acceptance_contract.gate_corner_span_m"))
    if span is None or abs(span - 1.2333333333333334) > 1e-9:
        errors.append("scaled gate metric span compile contract mismatch")
    if (manifest.get("pass_point") != [8.75, 0.0] or
            manifest.get("pass_radius_m") != 0.40 or
            manifest.get("maximum_final_cross_track_m") != 0.35):
        errors.append("unexpected easier-course completion contract")
    scene_path = SCENE_DIR / str(manifest.get("scene", ""))
    if not scene_path.is_file():
        errors.append("missing checked-in course scene")
        return
    if config.get("course_scene_sha256") != sha256(scene_path):
        errors.append("course scene provenance mismatch")
    validate_newbee_scene_geometry(
        scene_path, expected_gates[0] if expected_gates else None, errors)
    scene = scene_path.read_text()
    for asset in ("off_white_painted_plaster.png", "post_gate_box",
                  "ceiling_strip_light"):
        if asset not in scene:
            errors.append("course scene lacks " + asset)
    has_gate = 'name="expected_gate"' in scene
    if has_gate != bool(expected_gates):
        errors.append("course scene gate presence mismatch")
    if expected_gates and ("newbeedrone_gate_front_rgba_v1.png" not in scene or
                           scene.count('name="expected_gate_') < 4):
        errors.append("course scene lacks physical NewBee gate assets")


def load_candidate_bundle(path: Path) -> tuple[dict[str, str] | None, list[str]]:
    """Validate the joint manifest and its policy artifact, returning identity."""
    errors: list[str] = []
    bundle = read_json(path, errors)
    if bundle is None:
        return None, errors
    if bundle.get("format") != "tinympc-joint-gate-obstacle-student-v1":
        errors.append("candidate bundle has unexpected format")
    if bundle.get("runtime_adapter") != CANDIDATE_ADAPTER:
        errors.append("candidate bundle has unexpected runtime_adapter")
    policy = dotted(bundle, "artifacts.policy_onnx")
    if not isinstance(policy, dict):
        errors.append("candidate bundle lacks artifacts.policy_onnx")
        return None, errors
    expected_hash = policy.get("sha256")
    relative_path = policy.get("path")
    if not isinstance(expected_hash, str) or len(expected_hash) != 64:
        errors.append("candidate bundle policy hash is invalid")
    if not isinstance(relative_path, str):
        errors.append("candidate bundle policy path is invalid")
    else:
        policy_path = (path.parent / relative_path).resolve()
        if Path(relative_path).name != relative_path or policy_path.parent != path.parent.resolve():
            errors.append("candidate bundle policy must be a direct child artifact")
            return None, errors
        try:
            actual_hash = sha256(policy_path)
        except OSError as exc:
            errors.append(f"candidate policy is unreadable: {exc}")
        else:
            if actual_hash != expected_hash:
                errors.append("candidate bundle policy hash does not match policy artifact")
    if errors:
        return None, errors
    return {"bundle_path": str(path.resolve()), "bundle_manifest_sha256": sha256(path),
            "policy_sha256": expected_hash}, []


def percentile_95(values: list[float]) -> float | None:
    values.sort()
    if not values:
        return None
    position = .95 * (len(values) - 1)
    lower, upper = math.floor(position), math.ceil(position)
    return values[lower] + (values[upper] - values[lower]) * (position - lower)


def vision_latency(path: Path, errors: list[str]) -> float | None:
    try:
        with path.open(newline="") as stream:
            rows = list(csv.DictReader(stream))
        values = [finite(row.get("inference_ms")) for row in rows]
        samples = [value for value in values if value is not None]
        if not rows or not samples:
            raise ValueError("no finite inference_ms samples")
        return percentile_95(samples)
    except (OSError, csv.Error, ValueError) as exc:
        errors.append(f"invalid vision.csv: {exc}")
        return None


def state_final_time(path: Path, errors: list[str]) -> float | None:
    try:
        with path.open(newline="") as stream:
            rows = list(csv.DictReader(stream))
        value = finite(rows[-1].get("time_s")) if rows else None
        if value is None:
            raise ValueError("no final finite time_s")
        return value
    except (OSError, csv.Error, ValueError) as exc:
        errors.append(f"invalid state.csv: {exc}")
        return None


def physical_gate_pass(summary: dict[str, Any], errors: list[str]) -> bool | None:
    results = summary.get("course_gate_results")
    ordered = boolean(summary.get("course_gates_passed_in_order"))
    if not isinstance(results, list) or not results:
        errors.append("missing physical course_gate_results")
        return None
    if ordered is None:
        errors.append("missing course_gates_passed_in_order")
        return None
    crossed = [boolean(item.get("crossed")) if isinstance(item, dict) else None
               for item in results]
    if any(value is None for value in crossed):
        errors.append("malformed physical course_gate_results")
        return None
    return ordered and all(crossed)


def generic_contact(summary: dict[str, Any]) -> bool | None:
    crashed = boolean(summary.get("crashed"))
    count = finite(summary.get("contact_count_max"))
    if crashed is True or (count is not None and count > 0):
        return True
    if crashed is False and count == 0:
        return False
    return None


def classify_contacts(summary: dict[str, Any], group: str,
                      gate_passed: bool | None) -> dict[str, Any]:
    """Use classified telemetry, otherwise infer only from stop-first-contact."""
    frame = finite(summary.get("course_gate_frame_contact_count"))
    obstacle = finite(summary.get("course_obstacle_contact_count"))
    if frame is not None and obstacle is not None:
        return {"gate_frame_contacts": frame, "obstacle_contacts": obstacle,
                "source": "analyzer_classified", "conservative_inference": False}
    contact = generic_contact(summary)
    stop_first = boolean(summary.get("stop_on_contact"))
    if stop_first is None:
        # Current run_config owns this field; row() fills it before calling us.
        stop_first = boolean(summary.get("_run_config_stop_on_contact"))
    if contact is False:
        return {"gate_frame_contacts": 0.0, "obstacle_contacts": 0.0,
                "source": "generic_no_contact", "conservative_inference": True}
    if contact is not True or stop_first is not True:
        return {"gate_frame_contacts": None, "obstacle_contacts": None,
                "source": "unknown_missing_classified_contact_telemetry",
                "conservative_inference": False}
    if group == "candidate_gate_obstacle":
        if gate_passed is True:
            return {"gate_frame_contacts": 0.0, "obstacle_contacts": 1.0,
                    "source": "conservative_stop_first_contact_post_gate",
                    "conservative_inference": True}
        return {"gate_frame_contacts": 1.0, "obstacle_contacts": 0.0,
                "source": "conservative_stop_first_contact_pre_gate",
                "conservative_inference": True}
    return {"gate_frame_contacts": 0.0, "obstacle_contacts": 1.0,
            "source": "conservative_stop_first_contact_obstacle_only",
            "conservative_inference": True}


def make_row(run: Path, group: str, common_provenance: dict[str, Any],
             candidate_policy_sha256: str) -> dict[str, Any]:
    run = run.resolve()
    errors: list[str] = []
    for name in REQUIRED:
        if not (run / name).is_file():
            errors.append(f"missing {name}")
    config = read_json(run / "run_config.json", errors)
    summary = read_json(run / "summary.json", errors)
    csv_latency = vision_latency(run / "vision.csv", errors)
    final_time = state_final_time(run / "state.csv", errors)

    expected_adapter = (BASELINE_ADAPTER if group == "baseline_obstacle_only"
                        else CANDIDATE_ADAPTER)
    expected_policy = (BASELINE_POLICY_SHA256 if group == "baseline_obstacle_only"
                       else candidate_policy_sha256)
    if config is not None:
        if config.get("course") != COURSES[group]:
            errors.append("unexpected run_config course")
        if config.get("vision_adapter") != expected_adapter:
            errors.append("unexpected vision_adapter")
        validate_course_contract(group, config, errors)
        if group == "baseline_obstacle_only":
            if dotted(config, "vision_model.sha256") != expected_policy:
                errors.append("baseline vision policy provenance mismatch")
        else:
            # Joint runs mount a bundle directory. The run config records the
            # manifest hash; load_candidate_bundle independently proves that
            # manifest's declared policy hash matches the actual policy file.
            manifest_hash = common_provenance.get("_candidate_bundle_manifest_sha256")
            if dotted(config, "vision_model.bundle_manifest_sha256") != manifest_hash:
                errors.append("candidate bundle manifest provenance mismatch")
        for key, expected in common_provenance.items():
            if key.startswith("_"):
                continue
            if dotted(config, key) != expected:
                errors.append(f"common provenance mismatch {key}")
    if summary is not None:
        if summary.get("course") != COURSES[group]:
            errors.append("unexpected summary course")
        requested = finite(config.get("launch_time_s")) if config else None
        actual = finite(summary.get("launch_time_s"))
        if requested is None or actual is None or abs(requested - actual) > .10:
            errors.append("launch timing invalid")
        duration = finite(summary.get("duration_s"))
        if duration is None or final_time is None or abs(duration - final_time) > .02:
            errors.append("state/summary duration mismatch")

    gate_passed = (physical_gate_pass(summary, errors)
                   if summary is not None and group == "candidate_gate_obstacle" else None)
    if summary is not None:
        summary["_run_config_stop_on_contact"] = (
            boolean(config.get("stop_on_contact")) if config else None)
        contact = classify_contacts(summary, group, gate_passed)
    else:
        contact = {"gate_frame_contacts": None, "obstacle_contacts": None,
                   "source": "unknown_missing_summary", "conservative_inference": False}
    clearance = finite(summary.get("course_obstacle_clearance_min_m")) if summary else None
    contact_seen = generic_contact(summary) if summary else None
    truncated = contact_seen is True and boolean(summary.get("course_success")) is not True
    summary_latency = (finite(summary.get("vision_inference_latency_ms", {}).get("p95"))
                       if summary and isinstance(summary.get("vision_inference_latency_ms"), dict)
                       else None)
    return {
        "run_directory": str(run), "group": group,
        "seed": config.get("random_seed") if config else None,
        "complete": not errors, "errors": errors,
        "course_completion": boolean(summary.get("course_success")) if summary else None,
        "passpoint": boolean(summary.get("course_pass_point_reached")) if summary else None,
        "ordered_gate_plane_pass": gate_passed,
        **contact,
        "minimum_obstacle_clearance_m": clearance,
        "clearance_scope": "truncated_at_first_contact" if truncated else "full_observed_window",
        "mean_speed_mps": finite(summary.get("course_mean_horizontal_speed_active_mps")) if summary else None,
        "inference_p95_ms": summary_latency if summary_latency is not None else csv_latency,
        "expected_adapter": expected_adapter,
        "expected_policy_sha256": expected_policy,
    }


def aggregate(rows: list[dict[str, Any]]) -> dict[str, Any]:
    count = len(rows)
    completion_rate = sum(row["course_completion"] is True for row in rows) / count if count else None
    gate_rate = sum(row["ordered_gate_plane_pass"] is True for row in rows) / count if count else None
    frame = [row["gate_frame_contacts"] for row in rows]
    obstacle = [row["obstacle_contacts"] for row in rows]
    clearances = [row["minimum_obstacle_clearance_m"] for row in rows
                  if row["minimum_obstacle_clearance_m"] is not None]
    latencies = [row["inference_p95_ms"] for row in rows
                 if row["inference_p95_ms"] is not None]
    speeds = [row["mean_speed_mps"] for row in rows if row["mean_speed_mps"] is not None]
    frame_known = all(value is not None for value in frame)
    obstacle_known = all(value is not None for value in obstacle)
    return {
        "trials": count,
        "complete_trials": sum(row["complete"] for row in rows),
        "completion_rate": completion_rate,
        "ordered_gate_pass_rate": gate_rate,
        "gate_frame_contacts": sum(frame) if frame_known else None,
        "unknown_gate_frame_contact_runs": sum(value is None for value in frame),
        "obstacle_contact_rate": (
            sum(value > 0 for value in obstacle) / count if count and obstacle_known else None),
        "unknown_obstacle_contact_runs": sum(value is None for value in obstacle),
        "minimum_obstacle_clearance_m": min(clearances) if clearances else None,
        "unknown_clearance_runs": count - len(clearances),
        "mean_speed_mps": sum(speeds) / len(speeds) if speeds else None,
        "maximum_run_inference_p95_ms": max(latencies) if latencies else None,
        "unknown_inference_latency_runs": count - len(latencies),
    }


def matrix_status_errors(groups: dict[str, list[Path]],
                         rows: dict[str, list[dict[str, Any]]]) -> list[str]:
    parents = {path.resolve().parent for values in groups.values() for path in values}
    if len(parents) != 1:
        return ["runs do not share one matrix root"]
    path = next(iter(parents)) / "matrix_status.tsv"
    if not path.is_file():
        return ["missing matrix_status.tsv"]
    errors: list[str] = []
    observed: list[tuple[str, str, str]] = []
    for line in path.read_text().splitlines():
        fields = line.split("\t")
        if len(fields) != 3:
            errors.append("malformed matrix_status.tsv row")
        else:
            observed.append((fields[0], fields[1], fields[2]))
    expected = {(group, str(row["seed"])) for group in GROUPS for row in rows[group]}
    keys = {(group, seed) for group, seed, _ in observed}
    if keys != expected:
        errors.append("matrix_status group/seed rows differ from supplied runs")
    if len(keys) != len(observed):
        errors.append("matrix_status duplicate group/seed rows")
    if any(code != "0" for _, _, code in observed):
        errors.append("matrix_status has nonzero exit code")
    return errors


def evaluate(groups: dict[str, list[Path]], common_provenance: dict[str, Any],
             candidate_identity: dict[str, str] | None,
             candidate_identity_errors: list[str] | None = None,
             expected_seed_start: int | None = None,
             matrix_tool_provenance: dict[str, dict[str, str]] | None = None,
             matrix_tool_errors: list[str] | None = None) -> dict[str, Any]:
    identity_errors = list(candidate_identity_errors or [])
    candidate_hash = candidate_identity.get("policy_sha256") if candidate_identity else ""
    run_provenance = dict(common_provenance)
    run_provenance["_candidate_bundle_manifest_sha256"] = (
        candidate_identity.get("bundle_manifest_sha256") if candidate_identity else None)
    rows = {group: [make_row(path, group, run_provenance, candidate_hash)
                    for path in groups[group]] for group in GROUPS}
    matrix_errors = [f"candidate bundle: {error}" for error in identity_errors]
    matrix_errors.extend(matrix_tool_errors or [])
    expected_matrix_tools = {"matrix_runner", "evaluator"}
    if (not isinstance(matrix_tool_provenance, dict) or
            set(matrix_tool_provenance) != expected_matrix_tools or
            any(not isinstance(matrix_tool_provenance.get(name), dict) or
                not valid_sha256(matrix_tool_provenance[name].get("sha256"))
                for name in expected_matrix_tools)):
        matrix_errors.append("missing or invalid matrix tool provenance")
    for key, value in FROZEN_INVARIANTS.items():
        if common_provenance.get(key) != value:
            matrix_errors.append(f"frozen invariant mismatch {key}")
    for key in RUNTIME_TOOL_HASH_KEYS:
        if not valid_sha256(common_provenance.get(key)):
            matrix_errors.append(f"missing or invalid common runtime tool hash {key}")
    directories = [row["run_directory"] for values in rows.values() for row in values]
    if len(set(directories)) != len(directories):
        matrix_errors.append("run directory is reused across groups")
    for group in GROUPS:
        seeds = [row["seed"] for row in rows[group]]
        if len(rows[group]) != 10:
            matrix_errors.append(f"{group}: expected exactly ten trials")
        if None in seeds or len(set(seeds)) != len(seeds):
            matrix_errors.append(f"{group}: missing or duplicate seeds")
    gate_seeds = {row["seed"] for row in rows["candidate_gate_obstacle"]}
    candidate_seeds = {row["seed"] for row in rows["candidate_obstacle_only"]}
    baseline_seeds = {row["seed"] for row in rows["baseline_obstacle_only"]}
    if not (gate_seeds == candidate_seeds == baseline_seeds):
        matrix_errors.append("candidate gate, candidate obstacle, and baseline seed sets differ")
    if expected_seed_start is not None:
        expected_seeds = set(range(expected_seed_start, expected_seed_start + 10))
        if gate_seeds != expected_seeds:
            matrix_errors.append("matrix seeds differ from declared fresh seed block")
    if any(not row["complete"] for values in rows.values() for row in values):
        matrix_errors.append("incomplete run artifacts")
    matrix_errors.extend(matrix_status_errors(groups, rows))

    baseline_by_seed = {row["seed"]: row for row in rows["baseline_obstacle_only"]}
    pairs = []
    for candidate in rows["candidate_obstacle_only"]:
        baseline = baseline_by_seed.get(candidate["seed"])
        if baseline is None:
            continue
        candidate_clearance = candidate["minimum_obstacle_clearance_m"]
        baseline_clearance = baseline["minimum_obstacle_clearance_m"]
        pairs.append({
            "seed": candidate["seed"],
            "clearance_delta_m_candidate_minus_baseline": (
                candidate_clearance - baseline_clearance
                if candidate_clearance is not None and baseline_clearance is not None else None),
            "candidate_obstacle_contact": (
                candidate["obstacle_contacts"] > 0
                if candidate["obstacle_contacts"] is not None else None),
            "baseline_obstacle_contact": (
                baseline["obstacle_contacts"] > 0
                if baseline["obstacle_contacts"] is not None else None),
            "completion_delta": (int(candidate["course_completion"] is True)
                                 - int(baseline["course_completion"] is True)),
        })

    aggregates = {group: aggregate(rows[group]) for group in GROUPS}
    gate = aggregates["candidate_gate_obstacle"]
    candidate = aggregates["candidate_obstacle_only"]
    baseline = aggregates["baseline_obstacle_only"]
    candidate_rows = rows["candidate_gate_obstacle"] + rows["candidate_obstacle_only"]
    checks = {
        "matrix_valid": not matrix_errors,
        "eight_of_ten_contact_free_ordered_gate_and_completion": (
            sum(row["ordered_gate_plane_pass"] is True
                and row["course_completion"] is True
                and row["gate_frame_contacts"] == 0
                and row["obstacle_contacts"] == 0
                for row in rows["candidate_gate_obstacle"]) >= 8),
        "zero_gate_frame_contacts": gate["gate_frame_contacts"] == 0,
        "positive_course_completion": (
            gate["completion_rate"] is not None and gate["completion_rate"] > 0),
        "obstacle_only_collision_not_worse": (
            candidate["obstacle_contact_rate"] is not None
            and baseline["obstacle_contact_rate"] is not None
            and candidate["obstacle_contact_rate"] <= baseline["obstacle_contact_rate"]),
        "inference_p95_under_33ms": all(
            row["inference_p95_ms"] is not None and row["inference_p95_ms"] < 33
            for row in candidate_rows),
    }
    return {
        "format": "joint-gate-obstacle-evaluation-v2",
        "candidate_bundle": candidate_identity,
        "baseline_policy_sha256": BASELINE_POLICY_SHA256,
        "common_provenance": common_provenance,
        "matrix_tool_provenance": matrix_tool_provenance,
        "matrix_errors": matrix_errors,
        "groups": {group: {"runs": rows[group], "aggregate": aggregates[group]}
                   for group in GROUPS},
        "paired_obstacle_only_deltas": pairs,
        "acceptance": {"checks": checks, "accepted": all(checks.values())},
    }


def markdown(report: dict[str, Any]) -> str:
    acceptance = report["acceptance"]
    tools = report.get("matrix_tool_provenance") or {}
    lines = [
        "# Joint gate + obstacle evaluation", "",
        f"Accepted: **{'yes' if acceptance['accepted'] else 'no'}**.", "",
        "Physical gate passage comes only from analyzer course-plane results, never detector telemetry. "
        "Contact rows explicitly identify analyzer classification, conservative stop-first-contact "
        "inference, or unknown telemetry.", "", "## Matrix tool provenance", "",
        f"- runner SHA-256: {tools.get('matrix_runner', {}).get('sha256')}",
        f"- evaluator SHA-256: {tools.get('evaluator', {}).get('sha256')}",
        "", "## Acceptance checks", "",
    ]
    lines.extend(f"- {name}: {'pass' if passed else 'FAIL'}"
                 for name, passed in acceptance["checks"].items())
    if report["matrix_errors"]:
        lines.extend(["", "## Matrix errors", ""])
        lines.extend(f"- {error}" for error in report["matrix_errors"])
    lines.extend(["", "## Groups", "",
                  "| group | trials | complete | completion | gate pass | frame contacts | obstacle collision rate | min clearance m | mean speed m/s | max run p95 ms |",
                  "| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |"])
    for group, result in report["groups"].items():
        value = result["aggregate"]
        lines.append(
            f"| {group} | {value['trials']} | {value['complete_trials']} | "
            f"{value['completion_rate']} | {value['ordered_gate_pass_rate']} | "
            f"{value['gate_frame_contacts']} | {value['obstacle_contact_rate']} | "
            f"{value['minimum_obstacle_clearance_m']} | {value['mean_speed_mps']} | "
            f"{value['maximum_run_inference_p95_ms']} |")
    lines.extend(["", "## Contact classification", "",
                  "| group | seed | frame | obstacle | source | clearance scope |",
                  "| --- | ---: | ---: | ---: | --- | --- |"])
    for group in GROUPS:
        for row in report["groups"][group]["runs"]:
            lines.append(f"| {group} | {row['seed']} | {row['gate_frame_contacts']} | "
                         f"{row['obstacle_contacts']} | {row['source']} | {row['clearance_scope']} |")
    return "\n".join(lines) + "\n"


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    for group in GROUPS:
        parser.add_argument("--" + group.replace("_", "-") + "-run", action="append",
                            type=Path, dest=group, required=True)
    parser.add_argument("--candidate-bundle", type=Path, required=True)
    parser.add_argument("--common-provenance", type=Path, required=True,
                        help="nonempty JSON object of run_config dotted-key: expected-value")
    parser.add_argument("--out-dir", type=Path, required=True)
    parser.add_argument("--seed-start", type=int, required=True)
    parser.add_argument("--matrix-runner-snapshot", type=Path, required=True)
    parser.add_argument("--evaluator-snapshot", type=Path, required=True)
    args = parser.parse_args()
    if args.seed_start < 0:
        parser.error("seed start must be nonnegative")
    common = json.loads(args.common_provenance.read_text())
    if not isinstance(common, dict) or not common:
        parser.error("common provenance must be a nonempty JSON object")
    identity, identity_errors = load_candidate_bundle(args.candidate_bundle)
    matrix_tool_errors: list[str] = []
    matrix_tools = load_matrix_tool_provenance(
        args.matrix_runner_snapshot, args.evaluator_snapshot,
        matrix_tool_errors)
    report = evaluate({group: getattr(args, group) for group in GROUPS}, common,
                      identity, identity_errors,
                      expected_seed_start=args.seed_start,
                      matrix_tool_provenance=matrix_tools,
                      matrix_tool_errors=matrix_tool_errors)
    args.out_dir.mkdir(parents=True, exist_ok=True)
    (args.out_dir / "evaluation.json").write_text(json.dumps(report, indent=2) + "\n")
    (args.out_dir / "evaluation.md").write_text(markdown(report))
    print(json.dumps(report["acceptance"], indent=2))
    return 0 if report["acceptance"]["accepted"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
