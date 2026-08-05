#!/usr/bin/env python3
"""Verify saved obstacle-validation evidence and report incomplete gates.

The default exit status is nonzero until every acceptance gate is supported.
Use --allow-incomplete when refreshing the machine-readable progress report.
"""

import argparse
import hashlib
import json
import subprocess
from pathlib import Path

import analyze_obstacle_corpus


SCRIPT = Path(__file__).resolve()
APP_DIR = SCRIPT.parents[3]
CF_REPO = SCRIPT.parents[5]
WORKSPACE = CF_REPO.parent
DEFAULT_GAP_REPO = WORKSPACE / "tinympc-nanocockpit"
RESULTS = APP_DIR / "tools/results/flow_obstacle_validation_2026-07-23.json"
EQUIVALENCE = APP_DIR / "tools/results/flow_15hz_equivalence.json"
CORPUS_PROGRESS = SCRIPT.with_name("logs") / "corpus-progress.json"


def sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def git_head(path):
    return subprocess.check_output(
        ["git", "-C", str(path), "rev-parse", "HEAD"], text=True
    ).strip()


def git_dirty(path):
    return bool(subprocess.check_output(
        ["git", "-C", str(path), "status", "--porcelain"], text=True
    ).strip())


def gate(name, status, evidence):
    return {"name": name, "status": status, "evidence": evidence}


def artifact_checks(results, gap_repo):
    checks = []
    stm = results["stm32_build"]["artifacts"]
    for name, artifact in stm.items():
        path = APP_DIR / artifact["path"]
        actual = sha256(path) if path.is_file() else None
        checks.append({
            "name": f"stm32_{name}",
            "path": str(path),
            "expected_sha256": artifact["sha256"],
            "actual_sha256": actual,
            "passed": actual == artifact["sha256"],
        })
    gap_root = gap_repo / "src/gap"
    for features in ("27", "36"):
        build = results["gap8_builds"][features]
        for kind, hash_key in (
            ("build_log", "build_log_sha256"),
            ("elf", "elf_sha256"),
        ):
            path = gap_root / build[kind]
            actual = sha256(path) if path.is_file() else None
            checks.append({
                "name": f"gap8_f{features}_{kind}",
                "path": str(path),
                "expected_sha256": build[hash_key],
                "actual_sha256": actual,
                "passed": actual == build[hash_key],
            })
    return checks


def build_report(gap_repo):
    results = json.loads(RESULTS.read_text())
    equivalence = json.loads(EQUIVALENCE.read_text())
    corpus = json.loads(CORPUS_PROGRESS.read_text())
    gates = []

    revisions = {
        "stm32_outer": {
            "expected": results["source"]["stm32_outer_commit"],
            "actual": git_head(CF_REPO),
            "expected_dirty": results["source"]["dirty"],
            "actual_dirty": git_dirty(CF_REPO),
        },
        "crazyflie_firmware": {
            "expected": results["source"]["crazyflie_firmware_commit"],
            "actual": git_head(CF_REPO / "crazyflie-firmware"),
            "expected_dirty": results["source"]["dirty"],
            "actual_dirty": git_dirty(CF_REPO / "crazyflie-firmware"),
        },
        "gap8": {
            "expected": results["source"]["gap8_commit"],
            "actual": git_head(gap_repo),
            "expected_dirty": results["source"]["dirty"],
            "actual_dirty": git_dirty(gap_repo),
        },
    }
    gates.append(gate(
        "source_revisions",
        "passed" if all(
            item["expected"] == item["actual"]
            and item["expected_dirty"] == item["actual_dirty"]
            for item in revisions.values()
        ) else "failed",
        revisions,
    ))

    artifacts = artifact_checks(results, gap_repo)
    gates.append(gate(
        "build_artifact_integrity",
        "passed" if all(item["passed"] for item in artifacts) else "failed",
        artifacts,
    ))

    equivalence_passed = (
        equivalence.get("all_passed") is True
        and equivalence.get("frames") == 204
        and equivalence.get("cases") == 17
    )
    gates.append(gate("compiled_python_equivalence",
                      "passed" if equivalence_passed else "failed", {
                          "all_passed": equivalence.get("all_passed"),
                          "frames": equivalence.get("frames"),
                          "cases": equivalence.get("cases"),
                          "max_abs_error": equivalence.get("max_abs_error"),
                      }))

    for features in ("27", "36"):
        timing = results["updated_hardware_timing"][features]
        timing_passed = (
            timing["flow_hz_mean"] >= 14.8
            and timing["cnn_hz_mean"] >= 14.8
            and timing["flow_snapshot_drops"] == 0
            and timing["flow_tx_drops"] == 0
            and timing["uart_errors"] == 0
            and timing["camera_recoveries"] == 0
        )
        gates.append(gate(
            f"nominal_15_plus_15_hz_f{features}",
            "passed" if timing_passed else "failed",
            timing,
        ))

    corpus_matches_results = (
        corpus["expected_cases"]
        == results["acceptance"]["physical_corpus"]["expected_cases"]
        and corpus["completed_cases"]
        == results["acceptance"]["physical_corpus"][
            "exact_manifest_cases_completed"
        ]
    )
    corpus_complete = (
        corpus_matches_results
        and corpus["completed_cases"] == corpus["expected_cases"]
        and not corpus["duplicate_case_ids"]
        and not corpus["unreadable_sidecars"]
        and not corpus["invalid_metadata"]
        and not corpus["invalid_motion"]
        and not corpus["unverified_motion"]
        and not corpus["invalid_synchronization"]
        and not corpus["invalid_setup"]
    )
    gates.append(gate(
        "physical_corpus_coverage",
        "passed" if corpus_complete else (
            "incomplete" if corpus_matches_results else "failed"
        ),
        {
            "expected_cases": corpus["expected_cases"],
            "completed_cases": corpus["completed_cases"],
            "missing_cases": corpus["missing_cases"],
            "result_count_matches_audit": corpus_matches_results,
            "duplicates": len(corpus["duplicate_case_ids"]),
            "unreadable": len(corpus["unreadable_sidecars"]),
            "invalid_metadata": len(corpus["invalid_metadata"]),
            "invalid_motion": len(corpus["invalid_motion"]),
            "unverified_motion": len(corpus["unverified_motion"]),
            "invalid_synchronization": len(
                corpus["invalid_synchronization"]
            ),
            "invalid_setup": len(corpus["invalid_setup"]),
        },
    ))

    log_dir = SCRIPT.with_name("logs")
    qualified_runs = [
        analyze_obstacle_corpus.summarize_run(
            log_dir / f"{case_id}.csv.json"
        )
        for case_id in corpus["completed"]
    ]
    positives = [run for run in qualified_runs if run["label"] == "positive"]
    negatives = [run for run in qualified_runs if run["label"] == "negative"]
    expected_positive = results["acceptance"]["physical_corpus"][
        "expected_positive_cases"
    ]
    expected_negative = results["acceptance"]["physical_corpus"][
        "expected_negative_cases"
    ]

    false_positive_ids = [
        run["case_id"] for run in negatives if run["detected"]
    ]
    negative_status = (
        "failed" if false_positive_ids else (
            "passed" if len(negatives) == expected_negative else "incomplete"
        )
    )
    gates.append(gate("no_false_cylinders_in_negative_corpus",
                      negative_status, {
                          "required_cases": expected_negative,
                          "qualified_cases": len(negatives),
                          "false_positive_case_ids": false_positive_ids,
                      }))

    detected_positive = [run for run in positives if run["detected"]]
    detection_rate = (
        len(detected_positive) / len(positives) if positives else None
    )
    detection_status = "incomplete"
    if len(positives) == expected_positive:
        detection_status = (
            "passed" if detection_rate >= 0.95 else "failed"
        )
    gates.append(gate("positive_detection_at_least_95_percent",
                      detection_status, {
                          "required_cases": expected_positive,
                          "qualified_cases": len(positives),
                          "detected_cases": len(detected_positive),
                          "detection_rate": detection_rate,
                          "minimum_rate": 0.95,
                      }))

    localization_missing = [
        run["case_id"] for run in detected_positive
        if run["world_position_error_m"] is None
    ]
    localization_failures = [
        {
            "case_id": run["case_id"],
            "world_position_error_m": run["world_position_error_m"],
        }
        for run in detected_positive
        if run["world_position_error_m"] is not None
        and run["world_position_error_m"] > 0.35
    ]
    localization_status = (
        "failed" if localization_failures or localization_missing else (
            "passed" if len(positives) == expected_positive else "incomplete"
        )
    )
    gates.append(gate("localization_error_at_most_0p35m",
                      localization_status, {
                          "threshold_m": 0.35,
                          "qualified_positive_cases": len(positives),
                          "detected_positive_cases": len(detected_positive),
                          "missing_error_case_ids": localization_missing,
                          "failures": localization_failures,
                      }))

    bias = analyze_obstacle_corpus.bias_audit(positives)
    bias_status = (
        "passed" if bias["passed"] is True else (
            "failed" if bias["passed"] is False else "incomplete"
        )
    )
    gates.append(gate("no_systematic_orientation_or_left_right_bias",
                      bias_status, bias))

    fault_fields = (
        "crc_errors", "bad_frames", "duplicates", "invalid_frames",
        "sequence_gaps", "sequence_resets", "uart_queue_drops",
        "map_vote_without_new_sample_violations",
    )
    transport_totals = {
        field: sum(run["transport"][field] or 0
                   for run in qualified_runs)
        for field in fault_fields
    }
    transport_faults = {
        field: value for field, value in transport_totals.items() if value
    }
    gates.append(gate(
        "corpus_transport_and_new_sample_voting",
        "failed" if transport_faults else (
            "passed" if corpus_complete else "incomplete"
        ),
        {
            "qualified_cases": len(qualified_runs),
            "totals": transport_totals,
            "nonzero_faults": transport_faults,
        },
    ))

    overall = "passed" if all(
        item["status"] == "passed" for item in gates
    ) else (
        "failed" if any(item["status"] == "failed" for item in gates)
        else "incomplete"
    )
    return {
        "schema_version": 1,
        "overall_status": overall,
        "free_flight_authorized": False,
        "gates": gates,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--gap-repo", type=Path, default=DEFAULT_GAP_REPO)
    parser.add_argument("--out", type=Path)
    parser.add_argument("--allow-incomplete", action="store_true")
    args = parser.parse_args()
    report = build_report(args.gap_repo.resolve())
    rendered = json.dumps(report, indent=2) + "\n"
    if args.out:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        args.out.write_text(rendered)
        print(args.out)
    else:
        print(rendered, end="")
    if report["overall_status"] != "passed" and not args.allow_incomplete:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
