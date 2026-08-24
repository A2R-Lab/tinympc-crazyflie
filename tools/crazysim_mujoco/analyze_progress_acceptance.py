#!/usr/bin/env python3
"""Produce focused acceptance evidence for progress-circle speed matrices."""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
import re
from typing import Any

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

import analyze_progress_speed_ladder as ladder


DEFAULTS = {
    "entry_fraction": 0.05,
    "max_cross_track_rmse_m": 0.15,
    "max_cross_track_p95_m": 0.25,
    "max_altitude_rmse_m": 0.15,
    "min_speed_fraction": 0.85,
    "flyaway_distance_m": 2.0,
}

EXPECTED_SPEEDS_MPS = (1.0, 1.5, 2.0)
EXPECTED_SEEDS = (1, 2, 3)
EXPECTED_TRIALS = frozenset(
    (speed, seed) for speed in EXPECTED_SPEEDS_MPS for seed in EXPECTED_SEEDS)
SHA256_RE = re.compile(r"^[0-9a-f]{64}$")
TOP_LEVEL_HASH_FIELDS = ("runner_sha256", "trajectory_header_sha256")
GLOBAL_CONTRACT_HASH_FIELDS = (
    "controller_sha256",
    "bank_header_sha256",
    "bank_provenance_sha256",
    "progress_path_sha256",
    "frenet_error_sha256",
    "generated_model_sha256",
    "plant_model_sha256",
)
FIRMWARE_BINARY_HASH_FIELD = "firmware_binary_sha256"
CONTRACT_HASH_FIELDS = GLOBAL_CONTRACT_HASH_FIELDS + (
    FIRMWARE_BINARY_HASH_FIELD,)
HOMOGENEOUS_CONFIG_FIELDS = (
    "runner_sha256",
    "trajectory_header_sha256",
    "trajectory",
    "reference_mode",
    "progress_reference_limits",
    "progress_sample_limit",
    "level_cost_mode",
    "duration_s",
    "launch_time_s",
    "spawn_z_m",
    "model",
    "mass",
    "pwm_thrust_full_n",
    "launch_prespin",
    "inertia_scale",
    "motor_tau_scale",
    "thrust_scale",
    "realtime_factor",
    "firmware_time_factor",
    "actuator_lti",
    "stop_on_contact",
    "camera_capture_enabled",
    "extra_simulator_arguments",
)


def _canonical(value: Any) -> str:
    """Return a stable representation suitable for exact identity checks."""
    return json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False)


def _expected_speed(value: Any) -> float | None:
    try:
        speed = float(value)
    except (TypeError, ValueError):
        return None
    if not math.isfinite(speed):
        return None
    for expected in EXPECTED_SPEEDS_MPS:
        if math.isclose(speed, expected, rel_tol=0.0, abs_tol=1.0e-9):
            return expected
    return None


def _valid_seed(value: Any) -> int | None:
    if isinstance(value, bool):
        return None
    try:
        seed = int(value)
    except (TypeError, ValueError):
        return None
    if seed not in EXPECTED_SEEDS or value != seed:
        return None
    return seed


def _valid_resolved_plant_profile(profile: Any) -> tuple[bool, str | None]:
    if not isinstance(profile, dict):
        return False, "must be an object"
    required = ("name", "mass_kg", "diagonal_inertia_kg_m2",
                "drag_matrix_n_s_per_m")
    missing = [key for key in required if key not in profile]
    if missing:
        return False, f"missing {', '.join(missing)}"
    if not isinstance(profile["name"], str) or not profile["name"].strip():
        return False, "name must be a nonempty string"
    try:
        mass = float(profile["mass_kg"])
        inertia = [float(value) for value in profile["diagonal_inertia_kg_m2"]]
        drag = [[float(value) for value in row]
                for row in profile["drag_matrix_n_s_per_m"]]
    except (TypeError, ValueError):
        return False, "mass, inertia, and drag must be numeric"
    if not math.isfinite(mass) or mass <= 0.0:
        return False, "mass_kg must be positive and finite"
    if len(inertia) != 3 or any(not math.isfinite(value) or value <= 0.0
                                for value in inertia):
        return False, "diagonal_inertia_kg_m2 must contain three positive finite values"
    if len(drag) != 3 or any(len(row) != 3 for row in drag) or any(
            not math.isfinite(value) for row in drag for value in row):
        return False, "drag_matrix_n_s_per_m must be a finite 3x3 matrix"
    return True, None


def validate_matrix_contract(results: list[dict[str, Any]]) -> dict[str, Any]:
    """Validate the complete nine-trial configuration and evidence identity.

    Each result must contain the full run metadata under ``run_config``. This
    function is deliberately independent of the runner so fabricated, stale,
    incomplete, or duplicated matrices fail before behavioral gates can pass.
    """
    errors: list[dict[str, Any]] = []

    def reject(code: str, message: str, run: str | None = None,
               field: str | None = None) -> None:
        item: dict[str, Any] = {"code": code, "message": message}
        if run is not None:
            item["run_directory"] = run
        if field is not None:
            item["field"] = field
        errors.append(item)

    observed: dict[tuple[float, int], list[str]] = {}
    unexpected_trials: list[dict[str, Any]] = []
    identities: dict[str, list[tuple[str, Any]]] = {
        field: [] for field in HOMOGENEOUS_CONFIG_FIELDS
    }
    identities.update({
        f"acceptance_contract.{field}": []
        for field in GLOBAL_CONTRACT_HASH_FIELDS
    })
    identities["acceptance_contract.resolved_plant_profile"] = []
    firmware_identities: dict[float, list[tuple[str, int | None, str]]] = {
        speed: [] for speed in EXPECTED_SPEEDS_MPS
    }

    if len(results) != len(EXPECTED_TRIALS):
        reject("trial_count", f"expected exactly 9 trials, found {len(results)}")

    for index, result in enumerate(results):
        run = str(result.get("run_directory", f"<run-{index + 1}>"))
        config = result.get("run_config")
        if not isinstance(config, dict):
            reject("missing_run_config", "full run_config metadata is required", run)
            continue

        speed = _expected_speed(result.get("requested_speed_mps"))
        seed = _valid_seed(result.get("random_seed"))
        if speed is None:
            reject("unexpected_speed", "speed must be one of 1.0, 1.5, or 2.0 m/s", run)
        if seed is None:
            reject("unexpected_seed", "seed must be one of 1, 2, or 3", run)
        if speed is not None and seed is not None:
            observed.setdefault((speed, seed), []).append(run)
        elif seed is not None:
            try:
                raw_speed = float(result.get("requested_speed_mps"))
            except (TypeError, ValueError):
                raw_speed = None
            if raw_speed is not None and not math.isfinite(raw_speed):
                raw_speed = None
            unexpected_trials.append({
                "requested_speed_mps": raw_speed,
                "random_seed": seed,
                "run_directory": run,
            })
            reject("extra_trial",
                   f"unexpected speed={raw_speed!r}, seed={seed}", run)

        configured_speed = _expected_speed(config.get("progress_speed_mps"))
        if configured_speed != speed:
            reject("speed_config_mismatch",
                   "analyzed speed and run_config progress_speed_mps must match", run,
                   "progress_speed_mps")
        configured_seed = _valid_seed(config.get("random_seed"))
        if configured_seed != seed:
            reject("seed_config_mismatch",
                   "analyzed seed and run_config random_seed must match", run,
                   "random_seed")

        required_values = {
            "trajectory": "circle",
            "reference_mode": "progress",
            "progress_reference_limits": "uncapped",
            "progress_sample_limit": 0,
            "actuator_lti": True,
            "stop_on_contact": True,
            "camera_capture_enabled": True,
        }
        for field, expected in required_values.items():
            if field not in config:
                reject("missing_config_field", "required run_config field is absent",
                       run, field)
            elif config[field] != expected:
                reject("invalid_config_value",
                       f"required value is {expected!r}, found {config[field]!r}",
                       run, field)

        contract = config.get("acceptance_contract")
        if not isinstance(contract, dict):
            reject("missing_acceptance_contract",
                   "run_config.acceptance_contract object is required", run,
                   "acceptance_contract")
            contract = {}
        contract_values = {
            "circle_radius_m": 0.75,
            "flowdeck_enabled": True,
            "passive_camera_capture_enabled": True,
            "camera_inference_enabled": False,
            "vision_control_enabled": False,
        }
        for field, expected in contract_values.items():
            if field not in contract:
                reject("missing_contract_field", "required acceptance contract field is absent",
                       run, field)
            elif contract[field] != expected:
                reject("invalid_contract_value",
                       f"required value is {expected!r}, found {contract[field]!r}",
                       run, field)

        for field in CONTRACT_HASH_FIELDS:
            value = contract.get(field)
            if not isinstance(value, str) or SHA256_RE.fullmatch(value) is None:
                reject("invalid_contract_hash",
                       "required lowercase SHA-256 identity is absent or malformed",
                       run, field)
            elif field == FIRMWARE_BINARY_HASH_FIELD:
                if speed is not None:
                    firmware_identities[speed].append((run, seed, value))
            else:
                identities[f"acceptance_contract.{field}"].append((run, value))

        profile = contract.get("resolved_plant_profile")
        profile_valid, profile_error = _valid_resolved_plant_profile(profile)
        if not profile_valid:
            reject("invalid_resolved_plant_profile", profile_error or "invalid profile",
                   run, "resolved_plant_profile")
        else:
            identities["acceptance_contract.resolved_plant_profile"].append(
                (run, profile))

        for field in HOMOGENEOUS_CONFIG_FIELDS:
            if field not in config:
                reject("missing_config_field", "required homogeneous field is absent",
                       run, field)
            else:
                identities[field].append((run, config[field]))
        for field in TOP_LEVEL_HASH_FIELDS:
            value = config.get(field)
            if not isinstance(value, str) or SHA256_RE.fullmatch(value) is None:
                reject("invalid_config_hash",
                       "required lowercase SHA-256 identity is absent or malformed",
                       run, field)

    for pair, runs in sorted(observed.items()):
        if len(runs) > 1:
            reject("duplicate_trial",
                   f"trial speed={pair[0]:g}, seed={pair[1]} appears {len(runs)} times")
    observed_pairs = frozenset(observed)
    for speed, seed in sorted(EXPECTED_TRIALS - observed_pairs):
        reject("missing_trial", f"missing speed={speed:g}, seed={seed}")
    for speed, seed in sorted(observed_pairs - EXPECTED_TRIALS):
        reject("extra_trial", f"unexpected speed={speed:g}, seed={seed}")

    homogeneous_identity: dict[str, Any] = {}
    for field, values in identities.items():
        if not values:
            continue
        try:
            distinct = {_canonical(value) for _, value in values}
        except (TypeError, ValueError):
            reject("nonserializable_identity",
                   "identity value must be finite and JSON serializable", field=field)
            continue
        if len(distinct) != 1:
            reject("heterogeneous_matrix_identity",
                   "all nine trials must use the same value", field=field)
        else:
            homogeneous_identity[field] = values[0][1]

    firmware_identity_by_speed: list[dict[str, Any]] = []
    for speed in EXPECTED_SPEEDS_MPS:
        values = firmware_identities[speed]
        hashes = sorted({value for _, _, value in values})
        seeds = sorted(seed for _, seed, _ in values if seed is not None)
        if len(values) != len(EXPECTED_SEEDS) or seeds != list(EXPECTED_SEEDS):
            reject(
                "firmware_binary_group_incomplete",
                f"speed={speed:g} must have one valid firmware identity for seeds 1, 2, and 3",
                field=FIRMWARE_BINARY_HASH_FIELD,
            )
        if len(hashes) > 1:
            reject(
                "firmware_binary_seed_drift",
                f"speed={speed:g} uses {len(hashes)} firmware binaries across its seeds",
                field=FIRMWARE_BINARY_HASH_FIELD,
            )
        firmware_identity_by_speed.append({
            "requested_speed_mps": speed,
            "firmware_binary_sha256": hashes[0] if len(hashes) == 1 else None,
            "random_seeds": seeds,
            "run_directories": [run for run, _, _ in values],
        })

    return {
        "valid": not errors,
        "expected_trials": [
            {"requested_speed_mps": speed, "random_seed": seed}
            for speed, seed in sorted(EXPECTED_TRIALS)
        ],
        "observed_trials": [
            {"requested_speed_mps": speed, "random_seed": seed,
             "run_directories": runs}
            for (speed, seed), runs in sorted(observed.items())
        ] + unexpected_trials,
        "homogeneous_identity": homogeneous_identity,
        "firmware_identity_by_speed": firmware_identity_by_speed,
        "errors": errors,
    }


def assess_run(result: dict[str, Any], plot: dict[str, np.ndarray],
               thresholds: dict[str, float]) -> dict[str, Any]:
    """Reduce the full diagnostic result to explicit acceptance gates."""
    total = max(len(plot["route"]) - 1, 1)
    entry_sample = thresholds["entry_fraction"] * total
    entered = np.asarray(plot["progress"]) >= entry_sample
    post_entry = np.asarray(plot["tangent_velocity"])[entered]
    post_entry = post_entry[np.isfinite(post_entry)]
    median_speed = float(np.median(post_entry)) if post_entry.size else None
    requested_speed = float(result["requested_speed_mps"])
    required_speed = thresholds["min_speed_fraction"] * requested_speed

    route_distance = np.linalg.norm(
        np.asarray(plot["position"]) - np.asarray(plot["projected"]), axis=1)
    max_route_distance = (float(np.max(route_distance))
                          if route_distance.size else math.inf)
    flyaway = bool(max_route_distance > thresholds["flyaway_distance_m"])
    bounded = result["progress"]["bounded_diagnostics"]
    invariant_counts = {
        key: bounded.get(key) for key in (
            "projection_violation_count_max", "command_violation_count_max",
            "lead_violation_count_max")
    }
    invariants_valid = bool(
        bounded.get("present") and bounded.get("parseable") and bounded.get("valid")
        and all(value == 0 for value in invariant_counts.values()))

    cross = result["tracking"]["cross_track_m"]
    altitude = result["tracking"]["altitude_error_m"]
    any_contact_or_crash = bool(
        result["safety"]["contact_or_crash_active_route"]
        or result["safety"]["contact_or_crash_post_completion"])
    gates = {
        "timing_calibrated": bool(result["launch"]["calibrated"]),
        "full_geometric_completion": bool(
            result["progress"]["firmware_complete_750_of_750"]),
        "no_contact_or_crash": not any_contact_or_crash,
        "no_flyaway": not flyaway,
        "progress_invariants_valid": invariants_valid,
        "cross_track_rmse_within_limit": bool(
            cross["rmse"] is not None
            and cross["rmse"] <= thresholds["max_cross_track_rmse_m"]),
        "cross_track_p95_within_limit": bool(
            cross["p95_abs"] is not None
            and cross["p95_abs"] <= thresholds["max_cross_track_p95_m"]),
        "altitude_rmse_within_limit": bool(
            altitude["rmse"] is not None
            and altitude["rmse"] <= thresholds["max_altitude_rmse_m"]),
        "post_entry_speed_available": median_speed is not None,
        "post_entry_speed_at_least_fraction_of_request": bool(
            median_speed is not None and median_speed >= required_speed),
    }
    return {
        "requested_speed_mps": result["requested_speed_mps"],
        "random_seed": result.get("random_seed"),
        "run_directory": result["run_directory"],
        "accepted": all(gates.values()),
        "gates": gates,
        "launch_timing_error_s": result["launch"]["timing_error_s"],
        "launch_timing_validation": {
            "source": "analyze_progress_speed_ladder, matching run.sh post-analysis validator",
            "rule": "airborne evidence present and abs(actual-requested) <= 0.5 s",
            "calibrated": bool(result["launch"]["calibrated"]),
        },
        "firmware_complete_750_of_750": result["progress"]["firmware_complete_750_of_750"],
        "contact_or_crash": any_contact_or_crash,
        "contact_or_crash_active_route": result["safety"]["contact_or_crash_active_route"],
        "contact_or_crash_post_completion": result["safety"]["contact_or_crash_post_completion"],
        "flyaway": flyaway,
        "maximum_route_relative_distance_m": max_route_distance,
        "progress_invariant_counts": invariant_counts,
        "progress_invariant_record_count": bounded.get("record_count", 0),
        "cross_track_rmse_m": cross["rmse"],
        "cross_track_p95_abs_m": cross["p95_abs"],
        "altitude_rmse_m": altitude["rmse"],
        "median_post_entry_tangential_speed_mps": median_speed,
        "minimum_post_entry_tangential_speed_mps": required_speed,
        "post_entry_speed_fraction_of_request": (
            median_speed / requested_speed if median_speed is not None and requested_speed > 0 else None),
        "post_entry_samples": int(post_entry.size),
        "post_entry_definition": {
            "source": "offline monotonic geometric projection",
            "minimum_route_fraction": thresholds["entry_fraction"],
            "minimum_progress_sample": entry_sample,
        },
    }


def plot_summary(rows: list[dict[str, Any]], plots: dict[str, dict[str, np.ndarray]],
                 thresholds: dict[str, float], path: Path) -> None:
    rows = sorted(rows, key=lambda x: (x["requested_speed_mps"], x["random_seed"] or -1))
    labels = [f'{r["requested_speed_mps"]:g}/s{r["random_seed"]}' for r in rows]
    x = np.arange(len(rows))
    fig, axes = plt.subplots(2, 3, figsize=(16, 9))
    colors = ["tab:green" if r["accepted"] else "tab:red" for r in rows]
    axes[0, 0].bar(x, [abs(r["launch_timing_error_s"]) for r in rows], color=colors)
    axes[0, 0].axhline(0.5, ls="--", color="black", label="0.5 s limit")
    axes[0, 0].set_title("Absolute launch timing error (s)"); axes[0, 0].legend()
    axes[0, 1].plot(x, [r["cross_track_rmse_m"] for r in rows], "o-", label="RMSE")
    axes[0, 1].plot(x, [r["cross_track_p95_abs_m"] for r in rows], "s-", label="P95 |error|")
    axes[0, 1].axhline(thresholds["max_cross_track_rmse_m"], ls="--", color="C0")
    axes[0, 1].axhline(thresholds["max_cross_track_p95_m"], ls="--", color="C1")
    axes[0, 1].set_title("Cross-track error (m)"); axes[0, 1].legend()
    axes[0, 2].plot(x, [r["altitude_rmse_m"] for r in rows], "o-")
    axes[0, 2].axhline(thresholds["max_altitude_rmse_m"], ls="--", color="black")
    axes[0, 2].set_title("Altitude RMSE (m)")
    axes[1, 0].plot(x, [r["requested_speed_mps"] for r in rows], "--", label="command")
    axes[1, 0].plot(x, [r["minimum_post_entry_tangential_speed_mps"] for r in rows],
                    ":", label=f'{thresholds["min_speed_fraction"]:.0%} minimum')
    axes[1, 0].plot(x, [r["median_post_entry_tangential_speed_mps"] for r in rows], "o", label="median measured")
    axes[1, 0].set_title("Post-entry tangential speed (m/s)"); axes[1, 0].legend()
    for row in rows:
        data = plots[row["run_directory"]]
        axes[1, 1].plot(data["position"][:, 0], data["position"][:, 1],
                        label=f'{row["requested_speed_mps"]:g}/s{row["random_seed"]}')
    first = plots[rows[0]["run_directory"]]
    axes[1, 1].plot(first["route"][:, 0], first["route"][:, 1], "k--", alpha=.5)
    axes[1, 1].axis("equal"); axes[1, 1].set_title("Active-route trajectories")
    gate_names = list(rows[0]["gates"])
    gate_matrix = np.asarray([[r["gates"][g] for g in gate_names] for r in rows]).T
    axes[1, 2].imshow(gate_matrix, aspect="auto", cmap="RdYlGn", vmin=0, vmax=1)
    axes[1, 2].set_yticks(np.arange(len(gate_names)), [g.replace("_", " ") for g in gate_names], fontsize=7)
    axes[1, 2].set_xticks(x, labels, rotation=45, ha="right")
    axes[1, 2].set_title("Acceptance gates")
    for axis in axes.flat[:4]:
        axis.grid(True, alpha=.25); axis.set_xticks(x, labels, rotation=45, ha="right")
    axes[1, 1].grid(True, alpha=.25)
    fig.suptitle("Progress-circle acceptance summary (speed/seed)")
    fig.tight_layout(); fig.savefig(path, dpi=170); plt.close(fig)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--run", action="append", required=True, metavar="SPEED=DIR")
    parser.add_argument("--circle-header", required=True, type=Path)
    parser.add_argument("--out", required=True, type=Path)
    for name, value in DEFAULTS.items():
        parser.add_argument("--" + name.replace("_", "-"), type=float, default=value)
    args = parser.parse_args()
    thresholds = {name: getattr(args, name) for name in DEFAULTS}
    route = ladder.load_header(args.circle_header)
    rows, plots, matrix_results = [], {}, []
    for spec in args.run:
        speed_text, directory = spec.split("=", 1)
        run_directory = Path(directory)
        result, plot = ladder.analyze_run(run_directory, float(speed_text), route)
        matrix_result = dict(result)
        matrix_result["run_config"] = json.loads(
            (run_directory / "run_config.json").read_text())
        matrix_results.append(matrix_result)
        row = assess_run(result, plot, thresholds)
        rows.append(row); plots[row["run_directory"]] = plot
    matrix_contract = validate_matrix_contract(matrix_results)
    args.out.mkdir(parents=True, exist_ok=True)
    report = {
        "thresholds": thresholds,
        "matrix_contract": matrix_contract,
        "all_accepted": bool(
            matrix_contract["valid"] and all(r["accepted"] for r in rows)),
        "runs": rows,
    }
    (args.out / "acceptance_report.json").write_text(json.dumps(report, indent=2) + "\n")
    columns = ["requested_speed_mps", "random_seed", "accepted", "launch_timing_error_s",
               "firmware_complete_750_of_750", "contact_or_crash", "flyaway",
               "maximum_route_relative_distance_m", "cross_track_rmse_m",
               "cross_track_p95_abs_m", "altitude_rmse_m",
               "median_post_entry_tangential_speed_mps",
               "minimum_post_entry_tangential_speed_mps",
               "post_entry_speed_fraction_of_request", "post_entry_samples"]
    with (args.out / "acceptance_summary.csv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=columns, extrasaction="ignore")
        writer.writeheader(); writer.writerows(rows)
    plot_summary(rows, plots, thresholds, args.out / "acceptance_summary.png")
    print(json.dumps({"all_accepted": report["all_accepted"], "runs": len(rows), "out": str(args.out)}))


if __name__ == "__main__":
    main()
