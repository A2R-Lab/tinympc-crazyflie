#!/usr/bin/env python3
"""Evaluate the isolated combined gate + obstacle-avoidance POC.

This is deliberately an evidence *reader*: it does not rerun, repair, or
silently omit trials.  Gate success is taken only from ``analyze_run.py``'s
ground-truth course result (which computes a physical plane crossing from
MuJoCo state and declared course geometry), never from neural detections.
Vision data is reported separately as association/latency evidence.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
from collections import Counter
from pathlib import Path
from typing import Any, Iterable


GROUPS = (
    "candidate_gate_obstacle",
    "candidate_obstacle_only",
    "baseline_obstacle_only",
)
EXPECTED = {
    "candidate_gate_obstacle": ("gate_obstacle_poc", "combined_gate_rl"),
    "candidate_obstacle_only": ("gate_obstacle_poc_obstacle_only", "combined_gate_rl"),
    "baseline_obstacle_only": ("gate_obstacle_poc_obstacle_only", "hybrid_rl"),
}


def read_json(path: Path, errors: list[str]) -> dict[str, Any] | None:
    if not path.is_file():
        errors.append(f"missing {path.name}")
        return None
    try:
        value = json.loads(path.read_text())
    except (OSError, json.JSONDecodeError) as exc:
        errors.append(f"invalid {path.name}: {exc}")
        return None
    if not isinstance(value, dict):
        errors.append(f"invalid {path.name}: expected object")
        return None
    return value


def finite(value: Any) -> float | None:
    try:
        result = float(value)
    except (TypeError, ValueError):
        return None
    return result if math.isfinite(result) else None


def bool_or_none(value: Any) -> bool | None:
    return value if isinstance(value, bool) else None


def load_vision(path: Path, errors: list[str]) -> dict[str, Any] | None:
    """Summarize only perception telemetry; it is not success ground truth."""
    if not path.is_file():
        errors.append("missing vision.csv")
        return None
    try:
        with path.open(newline="") as stream:
            rows = list(csv.DictReader(stream))
    except (OSError, csv.Error) as exc:
        errors.append(f"invalid vision.csv: {exc}")
        return None
    if not rows:
        errors.append("empty vision.csv")
        return {"samples": 0}

    def values(name: str) -> list[float]:
        return [value for row in rows if (value := finite(row.get(name))) is not None]

    gate_valid = [row.get("gate_valid", "").strip().lower() in ("1", "true")
                  for row in rows]
    gate_confidence = values("gate_confidence")
    inference = values("inference_ms")
    actions = Counter()
    for row in rows:
        action = finite(row.get("rl_action"))
        if action is not None and action.is_integer() and 0 <= action <= 2:
            actions[("track", "left", "right")[int(action)]] += 1
    reasons = Counter(row.get("gate_reason", "").strip() or "missing"
                      for row in rows)
    valid_indices = [index for index, valid in enumerate(gate_valid) if valid]
    result: dict[str, Any] = {
        "samples": len(rows),
        "gate_valid_samples": sum(gate_valid),
        "gate_valid_fraction": sum(gate_valid) / len(rows),
        "gate_confidence_mean": (sum(gate_confidence) / len(gate_confidence)
                                  if gate_confidence else None),
        "gate_confidence_max": max(gate_confidence, default=None),
        "gate_reason_counts": dict(sorted(reasons.items())),
        "rl_action_counts": {name: int(actions[name])
                             for name in ("track", "left", "right")},
        "inference_latency_ms": percentile_summary(inference),
    }
    if valid_indices and "time_s" in rows[0]:
        timestamps = values("time_s")
        # The CSV should have one timestamp per row.  If malformed, do not
        # manufacture a time range from unaligned values.
        if len(timestamps) == len(rows):
            result["first_gate_valid_time_s"] = timestamps[valid_indices[0]]
            result["last_gate_valid_time_s"] = timestamps[valid_indices[-1]]
    return result


def percentile_summary(values: Iterable[float]) -> dict[str, float | int] | None:
    ordered = sorted(values)
    if not ordered:
        return None

    def percentile(percent: float) -> float:
        position = (len(ordered) - 1) * percent
        lower, upper = math.floor(position), math.ceil(position)
        if lower == upper:
            return ordered[lower]
        return ordered[lower] + (ordered[upper] - ordered[lower]) * (position - lower)

    return {"sample_count": len(ordered), "mean": sum(ordered) / len(ordered),
            "p50": percentile(.50), "p95": percentile(.95), "maximum": ordered[-1]}


def firmware_gate_lines(path: Path, errors: list[str]) -> int | None:
    if not path.is_file():
        errors.append("missing firmware.log")
        return None


def state_end_time(path: Path, errors: list[str]) -> float | None:
    if not path.is_file():
        errors.append("missing state.csv")
        return None
    try:
        with path.open(newline="") as stream:
            reader = csv.DictReader(stream)
            last = None
            for last in reader:
                pass
    except (OSError, csv.Error) as exc:
        errors.append(f"invalid state.csv: {exc}")
        return None
    value = finite(last.get("time_s")) if last else None
    if value is None:
        errors.append("state.csv has no finite final time_s")
    return value
    try:
        return sum("gate" in line.lower() for line in path.read_text(errors="replace").splitlines())
    except OSError as exc:
        errors.append(f"unreadable firmware.log: {exc}")
        return None


def ground_truth_gate(summary: dict[str, Any] | None) -> dict[str, Any] | None:
    if summary is None:
        return None
    results = summary.get("course_gate_results")
    ordered = bool_or_none(summary.get("course_gates_passed_in_order"))
    if not isinstance(results, list):
        return {"declared_gate_count": None, "passed_in_order": ordered,
                "results": None, "source": "analyzer_summary"}
    normalized = []
    for item in results:
        if not isinstance(item, dict):
            continue
        normalized.append({
            "name": item.get("name"), "crossed": bool_or_none(item.get("crossed")),
            "time_s": finite(item.get("time_s")),
            "lateral_error_m": finite(item.get("lateral_error_m")),
            "vertical_error_m": finite(item.get("vertical_error_m")),
            "clearance_margin_m": finite(item.get("clearance_margin_m")),
        })
    return {"declared_gate_count": len(normalized), "passed_in_order": ordered,
            "results": normalized, "source": "analyzer_summary"}


def make_row(path: Path, group: str) -> dict[str, Any]:
    path = path.resolve()
    errors: list[str] = []
    summary = read_json(path / "summary.json", errors)
    config = read_json(path / "run_config.json", errors)
    vision = load_vision(path / "vision.csv", errors)
    firmware_lines = firmware_gate_lines(path / "firmware.log", errors)
    state_end = state_end_time(path / "state.csv", errors)
    expected_course, expected_adapter = EXPECTED[group]
    if config is not None:
        if config.get("course") != expected_course:
            errors.append(
                f"unexpected config course {config.get('course')!r}; expected {expected_course!r}")
        if config.get("vision_adapter") != expected_adapter:
            errors.append(
                f"unexpected adapter {config.get('vision_adapter')!r}; expected {expected_adapter!r}")
    if summary is not None and summary.get("course") != expected_course:
        errors.append(
            f"unexpected summary course {summary.get('course')!r}; expected {expected_course!r}")
    summary_duration = finite(summary.get("duration_s")) if summary else None
    if state_end is not None and summary_duration is not None and abs(
            state_end - summary_duration) > 0.02:
        errors.append(
            f"state/summary duration mismatch {state_end:.6f}/{summary_duration:.6f}")
    if config is not None and summary is not None:
        requested_launch = finite(config.get("launch_time_s"))
        actual_launch = finite(summary.get("launch_time_s"))
        if (requested_launch is not None and actual_launch is not None and
                abs(requested_launch - actual_launch) > 0.10):
            errors.append(
                f"launch timing error {actual_launch - requested_launch:+.6f}s")
    latency = summary.get("vision_inference_latency_ms") if summary else None
    if not isinstance(latency, dict):
        latency = vision.get("inference_latency_ms") if vision else None
    contact_count = finite(summary.get("contact_count_max")) if summary else None
    crashed = bool_or_none(summary.get("crashed")) if summary else None
    contact_or_crash = ((crashed is True) or (contact_count is not None and contact_count > 0.0)
                        if crashed is not None or contact_count is not None else None)
    gate = ground_truth_gate(summary)
    gate_passed = (gate.get("passed_in_order") is True
                   if gate is not None and (gate.get("declared_gate_count") or 0) > 0
                   else None)
    row = {
        "group": group,
        "run_directory": str(path),
        "status": "complete" if not errors else "incomplete",
        "random_seed": config.get("random_seed") if config else None,
        "course": (summary.get("course") if summary else None)
                  or (config.get("course") if config else None),
        "vision_adapter": config.get("vision_adapter") if config else None,
        "course_completion": bool_or_none(summary.get("course_success")) if summary else None,
        "pass_point_reached": bool_or_none(summary.get("course_pass_point_reached")) if summary else None,
        "crashed": crashed,
        "contact_or_crash": contact_or_crash,
        "contact_before_completion": (bool_or_none(summary.get("course_contact_before_completion"))
                                      if summary else None),
        "contact_count_max": contact_count,
        "first_crash_time_s": (finite(summary.get("first_crash_time_s"))
                               if summary else None),
        "first_crash_after_launch_s": (
            finite(summary.get("first_crash_after_launch_s")) if summary else None),
        "minimum_obstacle_clearance_m": (finite(summary.get("course_obstacle_clearance_min_m"))
                                           if summary else None),
        "ground_truth_gate": gate,
        "obstacle_clearance_scope": (
            "truncated_before_obstacle" if group == "candidate_gate_obstacle" and
            gate_passed is not True and contact_or_crash is True
            else "observed_course_window"),
        "vision_gate_association": vision,
        "rl_action_counts": (vision.get("rl_action_counts") if vision else None),
        "inference_latency_ms": latency,
        "firmware_gate_log_line_count": firmware_lines,
        "errors": errors,
    }
    return row


def aggregate(rows: list[dict[str, Any]]) -> dict[str, Any]:
    total = len(rows)
    completed = sum(row["course_completion"] is True for row in rows)
    contacted = sum(row["contact_or_crash"] is True for row in rows)
    incomplete = sum(row["status"] != "complete" for row in rows)
    clearances = [row["minimum_obstacle_clearance_m"] for row in rows
                  if row["minimum_obstacle_clearance_m"] is not None]
    contact_counts = [row["contact_count_max"] for row in rows
                      if row["contact_count_max"] is not None]
    gates = [row["ground_truth_gate"] for row in rows
             if row["ground_truth_gate"] is not None
             and (gate_count := row["ground_truth_gate"].get("declared_gate_count"))
             is not None and gate_count > 0]
    ordered_pass = sum(gate.get("passed_in_order") is True for gate in gates)
    return {
        "trials": total,
        "incomplete_trials": incomplete,
        "course_completions": completed,
        "course_completion_rate_all_trials": completed / total if total else None,
        "contacts_or_crashes": contacted,
        "contact_or_crash_rate_all_trials": contacted / total if total else None,
        "sum_contact_count_max": sum(contact_counts) if contact_counts else None,
        "maximum_contact_count_max": max(contact_counts) if contact_counts else None,
        "runs_with_contact_count": len(contact_counts),
        "minimum_obstacle_clearance_m": min(clearances) if clearances else None,
        "runs_with_clearance": len(clearances),
        "runs_with_declared_gates": len(gates),
        "ground_truth_gate_ordered_passes": ordered_pass,
        "ground_truth_gate_ordered_pass_rate_declared_gate_runs": (
            ordered_pass / len(gates) if gates else None),
    }


def obstacle_regression(candidate: list[dict[str, Any]], baseline: list[dict[str, Any]]) -> dict[str, Any]:
    """Pair on recorded seed where possible, preserving unmatched trials."""
    by_seed = lambda rows: {row["random_seed"]: row for row in rows
                            if row["random_seed"] is not None}
    candidate_by_seed, baseline_by_seed = by_seed(candidate), by_seed(baseline)
    shared = sorted(set(candidate_by_seed) & set(baseline_by_seed), key=str)
    pairs = []
    for seed in shared:
        left, right = candidate_by_seed[seed], baseline_by_seed[seed]
        candidate_clearance = left["minimum_obstacle_clearance_m"]
        baseline_clearance = right["minimum_obstacle_clearance_m"]
        pairs.append({
            "random_seed": seed,
            "candidate_completion": left["course_completion"],
            "baseline_completion": right["course_completion"],
            "candidate_contact_or_crash": left["contact_or_crash"],
            "baseline_contact_or_crash": right["contact_or_crash"],
            "clearance_delta_m_candidate_minus_baseline": (
                candidate_clearance - baseline_clearance
                if candidate_clearance is not None and baseline_clearance is not None else None),
        })
    return {
        "comparison": "candidate_obstacle_only_minus_baseline_obstacle_only",
        "candidate_aggregate": aggregate(candidate),
        "baseline_aggregate": aggregate(baseline),
        "paired_seeds": shared,
        "candidate_only_seeds": sorted(set(candidate_by_seed) - set(baseline_by_seed), key=str),
        "baseline_only_seeds": sorted(set(baseline_by_seed) - set(candidate_by_seed), key=str),
        "paired_runs": pairs,
        "note": "All supplied trials remain in aggregates; paired deltas are supplemental.",
    }


def matrix_status_evidence(groups: dict[str, list[Path]]) -> dict[str, Any]:
    paths = [path.resolve() for values in groups.values() for path in values]
    parents = {path.parent for path in paths}
    if len(parents) != 1:
        return {"verified": False, "errors": ["run directories do not share one matrix root"]}
    status_path = next(iter(parents)) / "matrix_status.tsv"
    if not status_path.is_file():
        return {"verified": False, "path": str(status_path),
                "errors": ["missing matrix_status.tsv"]}
    expected = {(group, str(seed)) for group, rows in groups.items()
                for seed in [json.loads((path / "run_config.json").read_text()).get("random_seed")
                             for path in rows]}
    observed: list[tuple[str, str, str]] = []
    errors: list[str] = []
    try:
        for line in status_path.read_text().splitlines():
            fields = line.split("\t")
            if len(fields) != 3:
                errors.append(f"malformed matrix status line: {line!r}")
                continue
            observed.append((fields[0], fields[1], fields[2]))
    except OSError as exc:
        errors.append(f"unreadable matrix_status.tsv: {exc}")
    observed_keys = {(group, seed) for group, seed, _ in observed}
    if observed_keys != expected:
        errors.append("matrix_status group/seed rows do not match supplied runs")
    if len(observed) != len(observed_keys):
        errors.append("matrix_status contains duplicate group/seed rows")
    if any(code != "0" for _, _, code in observed):
        errors.append("matrix_status contains a nonzero run exit code")
    return {"verified": not errors, "path": str(status_path),
            "rows": len(observed), "errors": errors}


def format_value(value: Any) -> str:
    if value is None:
        return "—"
    if isinstance(value, bool):
        return "yes" if value else "no"
    if isinstance(value, float):
        return f"{value:.3f}"
    return str(value)


def gate_order_value(gate: dict[str, Any]) -> bool | None:
    """Avoid calling the vacuous ``all([])`` result a gate pass."""
    return (gate.get("passed_in_order")
            if (gate.get("declared_gate_count") or 0) > 0 else None)


def markdown(report: dict[str, Any]) -> str:
    lines = ["# Gate + obstacle POC evaluation", "",
             "Gate pass/order is MuJoCo/course ground truth from `summary.json`; "
             "neural detections are association evidence only.",
             f"Matrix status verified: {format_value(report['matrix_status']['verified'])}.", ""]
    for group in GROUPS:
        aggregate_row = report["groups"][group]["aggregate"]
        lines += [f"## {group.replace('_', ' ')}", "",
                  f"{aggregate_row['course_completions']}/{aggregate_row['trials']} course completions; "
                  f"{aggregate_row['contacts_or_crashes']}/{aggregate_row['trials']} contacts/crashes "
                  f"(sum/max contact count {format_value(aggregate_row['sum_contact_count_max'])}/"
                  f"{format_value(aggregate_row['maximum_contact_count_max'])}); "
                  f"minimum clearance {format_value(aggregate_row['minimum_obstacle_clearance_m'])} m; "
                  f"ordered gate passes {aggregate_row['ground_truth_gate_ordered_passes']}/"
                  f"{aggregate_row['runs_with_declared_gates']} declared-gate runs.", "",
                  "| seed | complete | passpoint | contact/crash | contact max | crash after launch (s) | gate order | min clearance (m) | actions T/L/R | p95 inference (ms) | evidence |",
                  "| --- | --- | --- | --- | ---: | ---: | --- | ---: | --- | ---: | --- |"]
        for row in report["groups"][group]["runs"]:
            gate = row["ground_truth_gate"] or {}
            action = row["rl_action_counts"] or {}
            latency = row["inference_latency_ms"] or {}
            evidence = "complete" if not row["errors"] else "; ".join(row["errors"])
            lines.append(
                "| {seed} | {complete} | {passpoint} | {contact_or_crash} | {contacts} | {crash_time} | {gate} | {clearance} | {actions} | {latency} | {evidence} |".format(
                    seed=format_value(row["random_seed"]),
                    complete=format_value(row["course_completion"]),
                    passpoint=format_value(row["pass_point_reached"]),
                    contact_or_crash=format_value(row["contact_or_crash"]),
                    contacts=format_value(row["contact_count_max"]),
                    crash_time=format_value(row["first_crash_after_launch_s"]),
                    gate=format_value(gate_order_value(gate)),
                    clearance=format_value(row["minimum_obstacle_clearance_m"]),
                    actions="/".join(str(action.get(name, 0)) for name in ("track", "left", "right")),
                    latency=format_value(finite(latency.get("p95"))), evidence=evidence))
        lines.append("")
        if any(row["obstacle_clearance_scope"] == "truncated_before_obstacle"
               for row in report["groups"][group]["runs"]):
            lines += ["Obstacle clearance for gate-contact runs is truncated before the "
                      "obstacle encounter and is not an avoidance-success result.", ""]
    regression = report["obstacle_only_regression"]
    lines += ["## Obstacle-only regression", "",
              "Candidate and baseline aggregates retain every supplied run. "
              "Paired rows below are only same-seed deltas.", "",
              "| seed | candidate complete | baseline complete | clearance delta (m) |", "| --- | --- | --- | ---: |"]
    for pair in regression["paired_runs"]:
        lines.append("| {random_seed} | {candidate_completion} | {baseline_completion} | {clearance_delta_m_candidate_minus_baseline} |".format(
            **{key: format_value(value) for key, value in pair.items()}))
    if not regression["paired_runs"]:
        lines.append("| — | — | — | no same-seed pairs |")
    if regression["candidate_only_seeds"] or regression["baseline_only_seeds"]:
        lines += ["", "Unpaired seeds: candidate-only `{};` baseline-only `{}`.".format(
            regression["candidate_only_seeds"], regression["baseline_only_seeds"])]
    return "\n".join(lines) + "\n"


def evaluate(groups: dict[str, list[Path]]) -> dict[str, Any]:
    evaluated = {name: [make_row(path, name) for path in paths]
                 for name, paths in groups.items()}
    for name, rows in evaluated.items():
        seen_paths: set[str] = set()
        seen_seeds: set[Any] = set()
        for row in rows:
            duplicate = []
            if row["run_directory"] in seen_paths:
                duplicate.append("duplicate run directory within group")
            if row["random_seed"] in seen_seeds:
                duplicate.append("duplicate random seed within group")
            seen_paths.add(row["run_directory"])
            seen_seeds.add(row["random_seed"])
            if duplicate:
                row["errors"].extend(duplicate)
                row["status"] = "incomplete"
    return {
        "format": "tinympc-gate-obstacle-poc-evaluation-v2",
        "ground_truth_gate_definition": "analyze_run summary.course_gate_results and "
                                        "summary.course_gates_passed_in_order",
        "groups": {name: {"runs": evaluated[name], "aggregate": aggregate(evaluated[name])}
                   for name in GROUPS},
        "matrix_status": matrix_status_evidence(groups),
        "obstacle_only_regression": obstacle_regression(
            evaluated["candidate_obstacle_only"], evaluated["baseline_obstacle_only"]),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--candidate-gate-obstacle-run", action="append", required=True,
                        type=Path, dest="candidate_gate_obstacle")
    parser.add_argument("--candidate-obstacle-only-run", action="append", required=True,
                        type=Path, dest="candidate_obstacle_only")
    parser.add_argument("--baseline-obstacle-only-run", action="append", required=True,
                        type=Path, dest="baseline_obstacle_only")
    parser.add_argument("--out-dir", required=True, type=Path)
    args = parser.parse_args()
    groups = {name: getattr(args, name) for name in GROUPS}
    report = evaluate(groups)
    args.out_dir.mkdir(parents=True, exist_ok=True)
    (args.out_dir / "evaluation.json").write_text(json.dumps(report, indent=2, allow_nan=False) + "\n")
    (args.out_dir / "evaluation.md").write_text(markdown(report))
    print(json.dumps({name: report["groups"][name]["aggregate"] for name in GROUPS}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
