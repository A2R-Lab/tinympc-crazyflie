#!/usr/bin/env python3
"""Aggregate bench CSV/JSON pairs into machine-readable acceptance metrics."""

import argparse
import csv
import json
import math
from collections import defaultdict
from pathlib import Path


LOG_TIMESTAMP_FIELDS = (
    "log_ts_health_flow",
    "log_ts_health_link",
    "log_ts_packet",
    "log_ts_motion_state",
    "log_ts_candidate",
    "log_ts_cylinder",
    "log_ts_reject_values",
    "log_ts_reject_counts",
    "log_ts_raw_0_1",
    "log_ts_raw_2_3",
    "log_ts_raw_4_5",
    "log_ts_raw_6_7",
    "log_ts_raw_8_8",
    "log_ts_range_0_2",
    "log_ts_range_3_5",
    "log_ts_range_6_8",
)
FLOW_PERIOD_MS = 1000.0 / 15.0
MIN_FLOW_DELIVERY_RATE_HZ = 12.0
MAX_FLOW_DELIVERY_STALL_S = 0.50
MAX_TRANSLATION_MEAN_SPEED_M_S = 0.75
MAX_TRANSLATION_P95_SPEED_M_S = 1.50
EXPECTED_POSITIVE_CASES = 180
MAX_SYMMETRIC_DETECTION_RATE_DELTA = 0.10
MAX_MEAN_SIGNED_BEARING_BIAS_DEG = 5.0
MAX_MEAN_BODY_LATERAL_BIAS_M = 0.10
MAX_SETUP_POSITION_ERROR_M = 0.075
MAX_SETUP_ORIENTATION_ERROR_DEG = 5.0


def number(value):
    try:
        return float(value)
    except (TypeError, ValueError):
        return math.nan


def circular_error_deg(a, b):
    return abs((a - b + 180.0) % 360.0 - 180.0)


def signed_circular_error_deg(estimate, truth):
    return (estimate - truth + 180.0) % 360.0 - 180.0


def finite_values(rows, field):
    return [value for row in rows if math.isfinite(value := number(row.get(field)))]


def counter_delta(rows, field):
    values = finite_values(rows, field)
    return max(0.0, values[-1] - values[0]) if values else None


def flow_delivery_evidence(rows):
    samples = []
    for row in rows:
        timestamp = number(row.get("t_s"))
        new_count = number(row.get("new_samples"))
        if math.isfinite(timestamp) and math.isfinite(new_count):
            samples.append((timestamp, new_count))
    if len(samples) < 2 or samples[-1][0] <= samples[0][0]:
        return {
            "observation_duration_s": None,
            "delivery_rate_hz": None,
            "startup_delivery_latency_s": None,
            "maximum_no_new_sample_interval_s": None,
            "minimum_delivery_rate_hz": MIN_FLOW_DELIVERY_RATE_HZ,
            "maximum_allowed_stall_s": MAX_FLOW_DELIVERY_STALL_S,
            "delivery_stable": None,
        }
    duration = samples[-1][0] - samples[0][0]
    delivered = max(0.0, samples[-1][1] - samples[0][1])
    last_change_time = samples[0][0]
    previous_count = samples[0][1]
    maximum_stall = 0.0
    startup_latency = None
    for timestamp, count in samples[1:]:
        if count > previous_count:
            if startup_latency is None:
                startup_latency = timestamp - samples[0][0]
            else:
                maximum_stall = max(
                    maximum_stall, timestamp - last_change_time
                )
            last_change_time = timestamp
            previous_count = count
    if startup_latency is None:
        maximum_stall = duration
    else:
        maximum_stall = max(
            maximum_stall, samples[-1][0] - last_change_time
        )
    delivery_rate = delivered / duration
    return {
        "observation_duration_s": duration,
        "delivery_rate_hz": delivery_rate,
        "startup_delivery_latency_s": startup_latency,
        "maximum_no_new_sample_interval_s": maximum_stall,
        "minimum_delivery_rate_hz": MIN_FLOW_DELIVERY_RATE_HZ,
        "maximum_allowed_stall_s": MAX_FLOW_DELIVERY_STALL_S,
        "delivery_stable": (
            delivery_rate >= MIN_FLOW_DELIVERY_RATE_HZ
            and maximum_stall <= MAX_FLOW_DELIVERY_STALL_S
        ),
    }


def percentile(values, fraction):
    if not values:
        return None
    ordered = sorted(values)
    return ordered[math.ceil(fraction * len(ordered)) - 1]


def motion_evidence(rows, label):
    """Return measured motion evidence and whether it supports the case label.

    These deliberately generous bench thresholds catch gross operator/metadata
    mismatches; they are not flight-dynamics acceptance limits.
    """
    yaw_rates = [abs(value) for value in finite_values(rows, "yaw_rate")]
    vx = finite_values(rows, "body_vx")
    vy = finite_values(rows, "body_vy")
    speeds = [math.hypot(x, y) for x, y in zip(vx, vy)]
    headings = finite_values(rows, "state_yaw")
    evidence = {
        "body_speed_mean_m_s": sum(speeds) / len(speeds) if speeds else None,
        "body_speed_p95_m_s": percentile(speeds, 0.95),
        "yaw_rate_abs_mean_rad_s": (
            sum(yaw_rates) / len(yaw_rates) if yaw_rates else None
        ),
        "yaw_rate_abs_p95_rad_s": percentile(yaw_rates, 0.95),
        "heading_span_deg": max(headings) - min(headings) if headings else None,
        "maximum_translation_mean_speed_m_s": (
            MAX_TRANSLATION_MEAN_SPEED_M_S
        ),
        "maximum_translation_p95_speed_m_s": (
            MAX_TRANSLATION_P95_SPEED_M_S
        ),
        "label_supported": None,
    }
    speed = evidence["body_speed_mean_m_s"]
    speed_p95 = evidence["body_speed_p95_m_s"]
    yaw = evidence["yaw_rate_abs_mean_rad_s"]
    if speed is None or speed_p95 is None or yaw is None:
        return evidence
    translation_speed_supported = (
        speed >= 0.05
        and speed <= MAX_TRANSLATION_MEAN_SPEED_M_S
        and speed_p95 <= MAX_TRANSLATION_P95_SPEED_M_S
    )
    evidence["label_supported"] = {
        "stationary": speed <= 0.02 and yaw <= 0.10,
        "translation": translation_speed_supported and yaw <= 0.35,
        "translation_yaw": translation_speed_supported and yaw >= 0.35,
        "pure_yaw": yaw >= 0.50,
    }.get(label)
    return evidence


def circular_u32_span(values):
    """Smallest arc containing uint32 timestamps, robust to wraparound."""
    modulus = 1 << 32
    ordered = sorted(int(value) % modulus for value in values)
    gaps = [
        ordered[index + 1] - ordered[index]
        for index in range(len(ordered) - 1)
    ]
    gaps.append(ordered[0] + modulus - ordered[-1])
    return modulus - max(gaps)


def synchronization_evidence(rows):
    spans = []
    for row in rows:
        timestamps = [number(row.get(field)) for field in LOG_TIMESTAMP_FIELDS]
        if all(math.isfinite(value) for value in timestamps):
            spans.append(circular_u32_span(timestamps))
    complete_fraction = len(spans) / len(rows) if rows else 0.0
    p99 = percentile(spans, 0.99)
    return {
        "timestamp_blocks": len(LOG_TIMESTAMP_FIELDS),
        "complete_rows": len(spans),
        "total_rows": len(rows),
        "complete_fraction": complete_fraction,
        "skew_mean_ms": sum(spans) / len(spans) if spans else None,
        "skew_p95_ms": percentile(spans, 0.95),
        "skew_p99_ms": p99,
        "skew_max_ms": max(spans) if spans else None,
        "flow_period_ms": FLOW_PERIOD_MS,
        "within_one_flow_period": (
            complete_fraction >= 0.95
            and p99 is not None
            and p99 <= FLOW_PERIOD_MS
        ),
    }


def resolve_truth_world(case, rows):
    truth_x = number(case.get("truth_world_x_m"))
    truth_y = number(case.get("truth_world_y_m"))
    if all(math.isfinite(value) for value in (truth_x, truth_y)):
        return truth_x, truth_y, "measured_world"
    start_range = number(case.get("truth_start_range_m"))
    start_bearing = number(case.get("truth_start_bearing_deg"))
    if not all(math.isfinite(value) for value in (start_range, start_bearing)):
        return math.nan, math.nan, None
    for row in rows:
        state_x = number(row.get("state_x"))
        state_y = number(row.get("state_y"))
        state_yaw = number(row.get("state_yaw"))
        if all(math.isfinite(value)
               for value in (state_x, state_y, state_yaw)):
            world_bearing = math.radians(state_yaw + start_bearing)
            return (
                state_x + start_range * math.cos(world_bearing),
                state_y + start_range * math.sin(world_bearing),
                "measured_start_polar",
            )
    return math.nan, math.nan, None


def first_finite_pose(rows):
    for row in rows:
        pose = (
            number(row.get("state_x")),
            number(row.get("state_y")),
            number(row.get("state_yaw")),
        )
        if all(math.isfinite(value) for value in pose):
            return pose
    return math.nan, math.nan, math.nan


def setup_evidence(case, rows, truth_x, truth_y):
    if case["label"] != "positive":
        return {"applicable": False, "label_supported": True}
    state_x, state_y, state_yaw = first_finite_pose(rows)
    actual_orientation = number(case.get("truth_orientation_deg"))
    width = number(case.get("truth_obstacle_width_m"))
    nominal_distance = number(case.get("distance_m"))
    nominal_lateral = number(case.get("lateral_offset_m"))
    values = (
        truth_x, truth_y, state_x, state_y, state_yaw,
        actual_orientation, width, nominal_distance, nominal_lateral,
    )
    if not all(math.isfinite(value) for value in values):
        return {
            "applicable": True,
            "label_supported": False,
            "reason": "missing measured pose, width, orientation, or initial state",
        }
    yaw = math.radians(state_yaw)
    dx, dy = truth_x - state_x, truth_y - state_y
    measured_forward = math.cos(yaw) * dx + math.sin(yaw) * dy
    measured_lateral = -math.sin(yaw) * dx + math.cos(yaw) * dy
    distance_error = measured_forward - nominal_distance
    lateral_error = measured_lateral - nominal_lateral
    orientation_error = signed_circular_error_deg(
        actual_orientation, number(case.get("orientation_deg"))
    )
    supported = (
        width > 0.0
        and abs(distance_error) <= MAX_SETUP_POSITION_ERROR_M
        and abs(lateral_error) <= MAX_SETUP_POSITION_ERROR_M
        and abs(orientation_error) <= MAX_SETUP_ORIENTATION_ERROR_DEG
    )
    return {
        "applicable": True,
        "label_supported": supported,
        "measured_forward_m": measured_forward,
        "nominal_forward_m": nominal_distance,
        "forward_error_m": distance_error,
        "measured_lateral_m": measured_lateral,
        "nominal_lateral_m": nominal_lateral,
        "lateral_error_m": lateral_error,
        "measured_orientation_deg": actual_orientation,
        "nominal_orientation_deg": number(case.get("orientation_deg")),
        "orientation_error_deg": orientation_error,
        "measured_width_or_diameter_m": width,
        "maximum_position_error_m": MAX_SETUP_POSITION_ERROR_M,
        "maximum_orientation_error_deg": MAX_SETUP_ORIENTATION_ERROR_DEG,
    }


def summarize_run(sidecar):
    meta = json.loads(sidecar.read_text())
    csv_path = Path(meta["csv"])
    with csv_path.open(newline="") as stream:
        rows = list(csv.DictReader(stream))
    case = meta["case"]
    detected_rows = [row for row in rows if number(row["cyl_valid"]) > 0.5]
    result = {
        "case_id": case["id"],
        "configuration_features": case["configuration_features"],
        "label": case["label"],
        "distance_m": case["distance_m"],
        "orientation_deg": case["orientation_deg"],
        "lateral_offset_m": case["lateral_offset_m"],
        "texture": case["texture"],
        "lighting": case["lighting"],
        "motion": case["motion"],
        "detected": bool(detected_rows),
        "first_detection_time_s": None,
        "first_detection_distance_m": None,
        "range_error_m": None,
        "bearing_error_deg": None,
        "signed_bearing_error_deg": None,
        "world_position_error_m": None,
        "signed_world_x_error_m": None,
        "signed_world_y_error_m": None,
        "signed_body_lateral_error_m": None,
        "truth_world_x_m": None,
        "truth_world_y_m": None,
        "truth_source": None,
        "truth_obstacle_width_m": case.get("truth_obstacle_width_m"),
        "truth_orientation_deg": case.get("truth_orientation_deg"),
        "maximum_dropout_s": None,
        "rows": len(rows),
        "motion_evidence": motion_evidence(rows, case["motion"]),
        "synchronization": synchronization_evidence(rows),
        "transport": {
            "received_samples": counter_delta(rows, "rx_ok"),
            "new_samples": counter_delta(rows, "new_samples"),
            "crc_errors": counter_delta(rows, "crc_err"),
            "bad_frames": counter_delta(rows, "bad_rx"),
            "duplicates": counter_delta(rows, "dup_rx"),
            "invalid_frames": counter_delta(rows, "invalid_rx"),
            "sequence_gaps": counter_delta(rows, "sequence_gaps"),
            "sequence_resets": counter_delta(rows, "sequence_resets"),
            "uart_queue_drops": counter_delta(rows, "uart_queue_drops"),
            "map_votes": counter_delta(rows, "map_votes"),
            "map_vote_without_new_sample_violations": 0,
            **flow_delivery_evidence(rows),
        },
    }
    truth_x, truth_y, truth_source = resolve_truth_world(case, rows)
    if math.isfinite(truth_x) and math.isfinite(truth_y):
        result["truth_world_x_m"] = truth_x
        result["truth_world_y_m"] = truth_y
        result["truth_source"] = truth_source
    result["setup_evidence"] = setup_evidence(
        case, rows, truth_x, truth_y
    )
    previous_new = None
    previous_votes = None
    for row in rows:
        new_count = number(row.get("new_samples"))
        votes = number(row.get("map_votes"))
        if not all(math.isfinite(v) for v in (new_count, votes)):
            continue
        if (previous_new is not None and votes > previous_votes and
                new_count <= previous_new):
            result["transport"]["map_vote_without_new_sample_violations"] += 1
        previous_new, previous_votes = new_count, votes
    if not detected_rows:
        return result
    first = detected_rows[0]
    result["first_detection_time_s"] = number(first["t_s"])
    first_state_x = number(first["state_x"])
    first_state_y = number(first["state_y"])
    if all(math.isfinite(v) for v in
           (truth_x, truth_y, first_state_x, first_state_y)):
        result["first_detection_distance_m"] = math.hypot(
            truth_x - first_state_x, truth_y - first_state_y
        )
    errors = []
    world_x_errors = []
    world_y_errors = []
    body_lateral_errors = []
    bearing_errors = []
    signed_bearing_errors = []
    range_errors = []
    for row in detected_rows:
        wx, wy = number(row["cyl_wx"]), number(row["cyl_wy"])
        sx, sy = number(row["state_x"]), number(row["state_y"])
        if all(math.isfinite(v) for v in (truth_x, truth_y, wx, wy)):
            errors.append(math.hypot(wx - truth_x, wy - truth_y))
            world_x_errors.append(wx - truth_x)
            world_y_errors.append(wy - truth_y)
        if all(math.isfinite(v)
               for v in (truth_x, truth_y, wx, wy, sx, sy)):
            truth_range = math.hypot(truth_x - sx, truth_y - sy)
            estimate_range = math.hypot(wx - sx, wy - sy)
            range_errors.append(abs(estimate_range - truth_range))
            truth_bearing = math.degrees(math.atan2(truth_y - sy, truth_x - sx))
            estimate_bearing = math.degrees(math.atan2(wy - sy, wx - sx))
            bearing_errors.append(circular_error_deg(estimate_bearing, truth_bearing))
            signed_bearing_errors.append(
                signed_circular_error_deg(estimate_bearing, truth_bearing)
            )
            yaw = number(row.get("state_yaw"))
            if math.isfinite(yaw):
                yaw_rad = math.radians(yaw)
                error_x = wx - truth_x
                error_y = wy - truth_y
                body_lateral_errors.append(
                    -math.sin(yaw_rad) * error_x
                    + math.cos(yaw_rad) * error_y
                )
    if errors:
        result["world_position_error_m"] = sum(errors) / len(errors)
    if range_errors:
        result["range_error_m"] = sum(range_errors) / len(range_errors)
    if bearing_errors:
        result["bearing_error_deg"] = sum(bearing_errors) / len(bearing_errors)
    if signed_bearing_errors:
        result["signed_bearing_error_deg"] = (
            sum(signed_bearing_errors) / len(signed_bearing_errors)
        )
    if world_x_errors:
        result["signed_world_x_error_m"] = (
            sum(world_x_errors) / len(world_x_errors)
        )
        result["signed_world_y_error_m"] = (
            sum(world_y_errors) / len(world_y_errors)
        )
    if body_lateral_errors:
        result["signed_body_lateral_error_m"] = (
            sum(body_lateral_errors) / len(body_lateral_errors)
        )
    first_detected_index = rows.index(first)
    dropout_start = None
    dropouts = []
    for row in rows[first_detected_index:]:
        timestamp = number(row["t_s"])
        if not math.isfinite(timestamp):
            continue
        if number(row["cyl_valid"]) > 0.5:
            if dropout_start is not None:
                dropouts.append(max(0.0, timestamp - dropout_start))
                dropout_start = None
        elif dropout_start is None:
            dropout_start = timestamp
    if dropout_start is not None:
        end_times = finite_values(rows, "t_s")
        if end_times:
            dropouts.append(max(0.0, end_times[-1] - dropout_start))
    result["maximum_dropout_s"] = max(dropouts, default=0.0)
    return result


def grouped(results, key, rate_name="detection_rate"):
    buckets = defaultdict(list)
    for result in results:
        buckets[str(result.get(key))].append(result)
    output = {}
    for name, items in sorted(buckets.items()):
        bucket = {
            "cases": len(items),
            rate_name: sum(item["detected"] for item in items) / len(items),
        }
        for field in (
            "world_position_error_m",
            "signed_bearing_error_deg",
            "signed_body_lateral_error_m",
        ):
            values = [
                item[field] for item in items if item.get(field) is not None
            ]
            bucket[f"mean_{field}"] = (
                sum(values) / len(values) if values else None
            )
        output[name] = bucket
    return output


def bias_audit(positives):
    orientations = grouped(positives, "orientation_deg")
    offsets = grouped(positives, "lateral_offset_m")
    orientation_pairs = (("-45", "45"), ("-20", "20"))
    offset_pairs = (("-0.3", "0.3"),)

    def symmetry_deltas(groups, pairs):
        return {
            f"{left}_vs_{right}": abs(
                groups[left]["detection_rate"]
                - groups[right]["detection_rate"]
            )
            for left, right in pairs
            if left in groups and right in groups
        }

    orientation_deltas = symmetry_deltas(orientations, orientation_pairs)
    offset_deltas = symmetry_deltas(offsets, offset_pairs)
    bearing_biases = [
        item["signed_bearing_error_deg"] for item in positives
        if item["signed_bearing_error_deg"] is not None
    ]
    lateral_biases = [
        item["signed_body_lateral_error_m"] for item in positives
        if item["signed_body_lateral_error_m"] is not None
    ]
    mean_bearing_bias = (
        sum(bearing_biases) / len(bearing_biases) if bearing_biases else None
    )
    mean_lateral_bias = (
        sum(lateral_biases) / len(lateral_biases) if lateral_biases else None
    )
    evaluable = (
        len(positives) == EXPECTED_POSITIVE_CASES
        and len(orientation_deltas) == len(orientation_pairs)
        and len(offset_deltas) == len(offset_pairs)
        and len(bearing_biases) >= int(0.95 * EXPECTED_POSITIVE_CASES)
        and len(lateral_biases) >= int(0.95 * EXPECTED_POSITIVE_CASES)
    )
    passed = None
    if evaluable:
        passed = (
            max((*orientation_deltas.values(),
                 *offset_deltas.values()), default=0.0)
            <= MAX_SYMMETRIC_DETECTION_RATE_DELTA
            and abs(mean_bearing_bias) <= MAX_MEAN_SIGNED_BEARING_BIAS_DEG
            and abs(mean_lateral_bias) <= MAX_MEAN_BODY_LATERAL_BIAS_M
        )
    return {
        "evaluable": evaluable,
        "passed": passed,
        "positive_cases_required": EXPECTED_POSITIVE_CASES,
        "positive_cases_present": len(positives),
        "orientation_detection_rate_deltas": orientation_deltas,
        "lateral_offset_detection_rate_deltas": offset_deltas,
        "maximum_allowed_symmetric_detection_rate_delta":
            MAX_SYMMETRIC_DETECTION_RATE_DELTA,
        "mean_signed_bearing_bias_deg": mean_bearing_bias,
        "maximum_allowed_mean_signed_bearing_bias_deg":
            MAX_MEAN_SIGNED_BEARING_BIAS_DEG,
        "mean_signed_body_lateral_bias_m": mean_lateral_bias,
        "maximum_allowed_mean_signed_body_lateral_bias_m":
            MAX_MEAN_BODY_LATERAL_BIAS_M,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("sidecars", nargs="+", type=Path)
    parser.add_argument("--out", required=True, type=Path)
    args = parser.parse_args()
    results = [summarize_run(path) for path in args.sidecars]
    positives = [item for item in results if item["label"] == "positive"]
    negatives = [item for item in results if item["label"] == "negative"]
    report = {
        "schema_version": 1,
        "runs": results,
        "summary": {
            "positive_cases": len(positives),
            "detection_rate": (sum(item["detected"] for item in positives) / len(positives)
                               if positives else None),
            "negative_cases": len(negatives),
            "false_positive_rate": (sum(item["detected"] for item in negatives) / len(negatives)
                                    if negatives else None),
            "localization_failures_over_0p35m": sum(
                item["world_position_error_m"] is not None and
                item["world_position_error_m"] > 0.35 for item in positives
            ),
            "bias_audit": bias_audit(positives),
            "by_distance": grouped(positives, "distance_m"),
            "by_orientation": grouped(positives, "orientation_deg"),
            "positive_by_texture": grouped(positives, "texture"),
            "positive_by_motion": grouped(positives, "motion"),
            "positive_by_configuration": grouped(positives, "configuration_features"),
            "negative_by_texture": grouped(
                negatives, "texture", "false_positive_rate"
            ),
            "negative_by_motion": grouped(
                negatives, "motion", "false_positive_rate"
            ),
            "negative_by_configuration": grouped(
                negatives, "configuration_features", "false_positive_rate"
            ),
            "motion_label_unsupported_runs": [
                item["case_id"] for item in results
                if item["motion_evidence"]["label_supported"] is False
            ],
            "motion_label_unverified_runs": [
                item["case_id"] for item in results
                if item["motion_evidence"]["label_supported"] is None
            ],
            "synchronization_outside_one_flow_period_runs": [
                item["case_id"] for item in results
                if not item["synchronization"]["within_one_flow_period"]
            ],
            "transport_totals": {
                field: sum(
                    run["transport"][field] or 0 for run in results
                )
                for field in (
                    "crc_errors", "bad_frames", "duplicates", "invalid_frames",
                    "sequence_gaps", "sequence_resets", "uart_queue_drops",
                    "map_vote_without_new_sample_violations",
                )
            },
        },
    }
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(report, indent=2) + "\n")
    print(f"wrote {args.out} from {len(results)} runs")


if __name__ == "__main__":
    main()
