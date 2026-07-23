#!/usr/bin/env python3
"""Audit physical-corpus coverage without treating ad-hoc logs as manifest cases."""

import argparse
import csv
import json
from collections import Counter
from pathlib import Path

import analyze_obstacle_corpus


def audit(manifest, sidecars):
    expected = {case["id"]: case for case in manifest["cases"]}
    observed = []
    unreadable = []
    for path in sidecars:
        try:
            metadata = json.loads(path.read_text())
            observed.append({
                "case_id": metadata["case"]["id"],
                "path": str(path),
                "metadata": metadata,
                "sidecar": path,
            })
        except (OSError, KeyError, TypeError, json.JSONDecodeError) as error:
            unreadable.append({"path": str(path), "error": str(error)})
    counts = Counter(item["case_id"] for item in observed)
    completed = []
    invalid_metadata = {}
    invalid_motion = {}
    unverified_motion = {}
    invalid_synchronization = {}
    invalid_transport = {}
    invalid_setup = {}
    for case_id, expected_case in expected.items():
        matching = [item for item in observed if item["case_id"] == case_id]
        if len(matching) != 1:
            continue
        item = matching[0]
        actual_case = item["metadata"]["case"]
        mismatches = {
            key: {"expected": value, "actual": actual_case.get(key)}
            for key, value in expected_case.items()
            if key != "repetitions"
            if actual_case.get(key) != value
        }
        if mismatches:
            invalid_metadata[case_id] = {
                "path": item["path"],
                "mismatches": mismatches,
            }
            continue
        try:
            summary = analyze_obstacle_corpus.summarize_run(item["sidecar"])
            evidence = summary["motion_evidence"]
        except (OSError, KeyError, TypeError, ValueError,
                csv.Error, json.JSONDecodeError) as error:
            unverified_motion[case_id] = {
                "path": item["path"],
                "error": str(error),
            }
            continue
        if evidence["label_supported"] is True:
            setup = summary["setup_evidence"]
            synchronization = summary["synchronization"]
            if not setup["label_supported"]:
                invalid_setup[case_id] = {
                    "path": item["path"],
                    "evidence": setup,
                }
            elif summary["transport"].get("delivery_stable") is not True:
                invalid_transport[case_id] = {
                    "path": item["path"],
                    "evidence": summary["transport"],
                }
            elif synchronization["within_one_flow_period"]:
                completed.append(case_id)
            else:
                invalid_synchronization[case_id] = {
                    "path": item["path"],
                    "evidence": synchronization,
                }
        elif evidence["label_supported"] is False:
            invalid_motion[case_id] = {
                "path": item["path"],
                "evidence": evidence,
            }
        else:
            unverified_motion[case_id] = {
                "path": item["path"],
                "evidence": evidence,
            }
    completed = sorted(completed)
    absent = sorted(case_id for case_id in expected if counts[case_id] == 0)
    missing = sorted(set(expected).difference(completed))
    duplicates = {
        case_id: [
            item["path"] for item in observed if item["case_id"] == case_id
        ]
        for case_id in sorted(counts)
        if counts[case_id] > 1
    }
    outside_manifest = {
        case_id: [
            item["path"] for item in observed if item["case_id"] == case_id
        ]
        for case_id in sorted(counts)
        if case_id not in expected
    }
    by_configuration = {}
    for configuration in sorted({
        case["configuration_features"] for case in expected.values()
    }):
        ids = {
            case_id for case_id, case in expected.items()
            if case["configuration_features"] == configuration
        }
        by_configuration[str(configuration)] = {
            "expected": len(ids),
            "completed": len(ids.intersection(completed)),
            "missing": len(ids.intersection(missing)),
        }
    return {
        "schema_version": 2,
        "expected_cases": len(expected),
        "completed_cases": len(completed),
        "missing_cases": len(missing),
        "absent_cases": absent,
        "duplicate_case_ids": duplicates,
        "outside_manifest": outside_manifest,
        "unreadable_sidecars": unreadable,
        "invalid_metadata": invalid_metadata,
        "invalid_motion": invalid_motion,
        "unverified_motion": unverified_motion,
        "invalid_synchronization": invalid_synchronization,
        "invalid_transport": invalid_transport,
        "invalid_setup": invalid_setup,
        "by_configuration": by_configuration,
        "completed": completed,
        "missing": missing,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", type=Path,
                        default=Path(__file__).with_name("obstacle_corpus.json"))
    parser.add_argument("--logs-dir", type=Path,
                        default=Path(__file__).with_name("logs"))
    parser.add_argument("--out", type=Path)
    args = parser.parse_args()
    report = audit(
        json.loads(args.manifest.read_text()),
        sorted(args.logs_dir.glob("*.csv.json")),
    )
    rendered = json.dumps(report, indent=2) + "\n"
    if args.out:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        args.out.write_text(rendered)
        print(args.out)
    else:
        print(rendered, end="")


if __name__ == "__main__":
    main()
