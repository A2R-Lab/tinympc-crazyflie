#!/usr/bin/env python3
"""Event-align direct-motor TinyMPC failures without assuming one log schema.

``TINYMPC-DIAG`` records are deliberately parsed as extensible ``key=value``
records.  This makes a report useful for legacy runs (where it states what is
missing) and for diagnostic firmware builds without having to revise this
tool whenever one more field is added.
"""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import re
import struct
from pathlib import Path
from typing import Any

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

KV_RE = re.compile(r"([A-Za-z][A-Za-z0-9_]*)=([^\s,]+)")
NUMBER_RE = re.compile(r"^[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?$")
TIME_KEYS = ("t_s", "time_s", "sim_time_s", "elapsed_s", "t_after_launch_s")
ACTION_KEYS = (tuple(f"u{i}" for i in range(1, 5)) + tuple(f"first_u{i}" for i in range(1, 5))
               + tuple(f"action_{i}" for i in range(1, 5)) + tuple(f"motor_{i}" for i in range(1, 5)))
BINARY_MAGIC = 0x544D5043
BINARY_VERSION = 2
BINARY_STRUCT = struct.Struct("<IHH16I2i3I4x3Q44f")
BINARY_RECORD_SIZE = 296
LEGACY_BINARY_STRUCT = struct.Struct("<IHH15I2i3I3Q44f")
LEGACY_BINARY_RECORD_SIZE = 288
FALLBACK_REASONS = {0: "fresh", 1: "motors_disabled", 2: "no_completed_plan", 3: "stale_plan", 4: "non_finite_motor_command"}
SCALAR_TRIGGER_LABELS = {
    1: "residual_threshold",
    2: "body_rate_above_5_rad_s",
    4: "tilt_above_15_deg",
    8: "reference_demand_above_0_15_m",
}
UINT32_MAX = (1 << 32) - 1
UINT32_NONWRAPPING_MAX = UINT32_MAX // 2
PLAUSIBLE_LIFECYCLE_TICKS_MAX = 60_000
MMAP_MAGIC = 0x544D444D
MMAP_VERSION = 1
MMAP_HEADER_SIZE = 4096
MMAP_SLOT_SIZE = 320
MMAP_CAPACITY = 16384
MMAP_FILE_SIZE = MMAP_HEADER_SIZE + MMAP_CAPACITY * MMAP_SLOT_SIZE
MMAP_HEADER_STRUCT = struct.Struct("<IHHIIIIIHH32xQQQ")
MMAP_FNV_OFFSET_BASIS = 2166136261
MMAP_FNV_PRIME = 16777619
ABSOLUTE_RPM_MATCH_TOLERANCE = 1.0
CANONICAL_CF21B_500_PROFILE_NAME = "cf21B_500_runtime"
CANONICAL_CF21B_500_RPM_TO_THRUST = (
    0.0, -3.133427287299859e-7, 4.407354891648379e-10)
CANONICAL_CF21B_500_FULL_THRUST_N = 0.20


def number(value: str) -> float | str:
    return float(value) if NUMBER_RE.match(value) else value


def load_csv(path: Path) -> dict[str, np.ndarray]:
    with path.open(newline="") as stream:
        rows = list(csv.DictReader(stream))
    if not rows:
        return {}
    result: dict[str, np.ndarray] = {}
    for key in rows[0]:
        try:
            result[key] = np.asarray([float(row.get(key, "nan")) for row in rows])
        except ValueError:
            pass
    return result


def pick(record: dict[str, Any], names: tuple[str, ...]) -> Any:
    return next((record[key] for key in names if key in record), None)


def nonwrapping_elapsed(later: int, earlier: int) -> int | None:
    """Return a duration only when subtraction cannot be unsigned underflow."""
    if later <= 0 or earlier <= 0 or later < earlier:
        return None
    return later - earlier


def plan_age_seconds(plan_age_ticks: int) -> float | None:
    """Reject the sentinel and values characteristic of uint32 underflow."""
    if plan_age_ticks == UINT32_MAX or plan_age_ticks > UINT32_NONWRAPPING_MAX:
        return None
    return float(plan_age_ticks) / 1000.0


def infer_tick_layout(records: list[dict[str, Any]]) -> str:
    """Distinguish corrected FreeRTOS lifecycle ticks from legacy mixed v2."""
    solves = [record for record in records if record["event"] == 1]
    if not solves:
        return "unavailable"
    previous: tuple[int, int, int] | None = None
    for record in solves:
        current = (record["release_tick"], record["start_tick"], record["finish_tick"])
        release_tick, start_tick, finish_tick = current
        ordered = release_tick <= start_tick <= finish_tick
        plausible = (start_tick - release_tick <= PLAUSIBLE_LIFECYCLE_TICKS_MAX and
                     finish_tick - start_tick <= PLAUSIBLE_LIFECYCLE_TICKS_MAX)
        monotonic = previous is None or all(now >= before for now, before in zip(current, previous))
        if not ordered or not plausible or not monotonic:
            return "legacy_mixed"
        previous = current
    return "corrected_freertos"


def configured_firmware_tick_us(config: dict[str, Any] | None) -> int | None:
    if not isinstance(config, dict):
        return None
    diagnostic = config.get("mpc_diagnostic")
    value = diagnostic.get("firmware_tick_us") if isinstance(diagnostic, dict) else None
    if value is None:
        value = config.get("firmware_tick_us")
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    numeric = float(value)
    return int(numeric) if math.isfinite(numeric) and numeric > 0 and numeric.is_integer() else None


def parse_diag(path: Path, launch_time_s: float) -> list[dict[str, Any]]:
    if not path.exists():
        return []
    result = []
    for line_number, line in enumerate(path.read_text(errors="replace").splitlines(), 1):
        if "TINYMPC-DIAG" not in line:
            continue
        record: dict[str, Any] = {key.lower(): number(value) for key, value in KV_RE.findall(line)}
        raw_time = pick(record, TIME_KEYS)
        # Firmware diagnostics conventionally use sim time; no timestamp is
        # silently invented when the line has none.
        record.update({"line": line_number, "raw": line, "time_s": raw_time if isinstance(raw_time, float) else None})
        result.append(record)
    return result


def _parse_binary_payload(payload: bytes, *, present: bool = True) -> tuple[list[dict[str, Any]], dict[str, Any]]:
    """Decode compact v2 (and explicitly legacy v1) payload records."""
    validation: dict[str, Any] = {"present": present, "valid": False, "errors": []}
    validation["bytes"] = len(payload)
    if BINARY_STRUCT.size != BINARY_RECORD_SIZE or LEGACY_BINARY_STRUCT.size != LEGACY_BINARY_RECORD_SIZE:
        raise AssertionError("internal binary schema size mismatch")
    if len(payload) < 8:
        validation["errors"].append(f"truncated_header_bytes={len(payload)}")
        return [], validation
    _, stream_version, stream_record_size = struct.unpack_from("<IHH", payload)
    if stream_version == BINARY_VERSION and stream_record_size == BINARY_RECORD_SIZE:
        schema, record_size, schema_version, authoritative = BINARY_STRUCT, BINARY_RECORD_SIZE, 2, True
    elif stream_version == 1 and stream_record_size == LEGACY_BINARY_RECORD_SIZE:
        schema, record_size, schema_version, authoritative = LEGACY_BINARY_STRUCT, LEGACY_BINARY_RECORD_SIZE, 1, False
    else:
        # Size is bounded by the known schemas; do not trust an arbitrary file value.
        schema, record_size, schema_version, authoritative = BINARY_STRUCT, BINARY_RECORD_SIZE, 2, False
        validation["errors"].append(f"unsupported_schema=version:{stream_version},size:{stream_record_size}")
    validation.update({"schema_version": int(stream_version), "record_size": int(stream_record_size),
                       "authoritative": authoritative})
    if len(payload) % record_size:
        validation["errors"].append(f"trailing_bytes={len(payload) % record_size}")
    records: list[dict[str, Any]] = []
    previous_sequence: int | None = None
    previous_dropped = 0
    previous_queue_high_water = 0
    sequence_gaps = 0
    for offset in range(0, len(payload) - record_size + 1, record_size):
        values = schema.unpack_from(payload, offset)
        magic, version, declared_record_size = values[:3]
        if magic != BINARY_MAGIC:
            validation["errors"].append(f"record[{offset // record_size}].magic=0x{magic:08x}")
        if version != schema_version:
            validation["errors"].append(f"record[{offset // record_size}].version={version}")
        if declared_record_size != record_size:
            validation["errors"].append(f"record[{offset // record_size}].record_size={declared_record_size}")
        integer_end = 24 if schema_version == 2 else 23
        integers = values[3:integer_end]
        if schema_version == 2:
            queue_high_water_count = integers[13]
            integers_without_queue = integers[:13] + integers[14:]
        else:
            queue_high_water_count = None
            integers_without_queue = integers
        (event, event_sequence, release_sequence, solve_sequence, release_tick,
         start_tick, finish_tick, plan_tick, plan_age_ticks, release_due_count,
         release_mutex_miss_count, semaphore_coalesced_count, dropped_record_count,
         solve_us, solve_tick_gap, model_id, solver_iterations, clamp_mask,
         numeric_flags, fallback_reason) = integers_without_queue
        if event not in (0, 1, 2):
            validation["errors"].append(
                f"record[{offset // record_size}].event={event}")
        if offset > 0 and event == 0:
            validation["errors"].append(
                f"record[{offset // record_size}].unexpected_stream_header")
        release_us, start_us, finish_us = values[integer_end:integer_end + 3]
        floats = values[integer_end + 3:]
        primal, dual, optimizer_max = floats[:3]
        estimator = floats[3:16]; reference = floats[16:28]
        rotor = floats[28:32]; raw_action = floats[32:36]
        clamped_action = floats[36:40]; progress = floats[40:44]
        if previous_sequence is not None and event_sequence != previous_sequence + 1:
            gap = max(0, int(event_sequence - previous_sequence - 1)); sequence_gaps += gap
            validation["errors"].append(f"event_sequence_gap={previous_sequence}->{event_sequence}")
        if dropped_record_count < previous_dropped:
            validation["errors"].append("dropped_record_count_decreased")
        if queue_high_water_count is not None:
            if queue_high_water_count < previous_queue_high_water:
                validation["errors"].append("queue_high_water_count_decreased")
            if queue_high_water_count > 64:
                validation["errors"].append(f"queue_high_water_count_exceeds_capacity={queue_high_water_count}")
            previous_queue_high_water = int(queue_high_water_count)
        previous_sequence = int(event_sequence); previous_dropped = int(dropped_record_count)
        record = {
            "binary_offset": offset, "event": int(event), "event_sequence": int(event_sequence),
            "release_sequence": int(release_sequence), "solve_sequence": int(solve_sequence),
            "release_tick": int(release_tick), "start_tick": int(start_tick), "finish_tick": int(finish_tick),
            "plan_tick": int(plan_tick), "plan_age_ticks": int(plan_age_ticks),
            "plan_age_s": plan_age_seconds(int(plan_age_ticks)),
            "release_due_count": int(release_due_count), "release_mutex_miss_count": int(release_mutex_miss_count),
            "semaphore_coalesced_count": int(semaphore_coalesced_count), "dropped_record_count": int(dropped_record_count),
            "queue_high_water_count": (int(queue_high_water_count) if queue_high_water_count is not None else None),
            "solve_us": int(solve_us), "solve_tick_gap": int(solve_tick_gap), "model_id": int(model_id),
            "solver_iterations": int(solver_iterations), "clamp_mask": int(clamp_mask),
            "numeric_flags": int(numeric_flags), "fallback_reason": int(fallback_reason),
            "release_us": int(release_us), "start_us": int(start_us), "finish_us": int(finish_us),
            "release_to_start_us": nonwrapping_elapsed(int(start_us), int(release_us)),
            "start_to_finish_us": nonwrapping_elapsed(int(finish_us), int(start_us)),
            "primal_residual": float(primal), "dual_residual": float(dual), "optimizer_max_abs": float(optimizer_max),
            "estimator": list(map(float, estimator)), "reference": list(map(float, reference)),
            "reference_frame": ("local_Xref0" if schema_version == 2 else "legacy_world_mixed"),
            "rotor_state": list(map(float, rotor)), "first_action_raw": list(map(float, raw_action)),
            "first_action_clamped": list(map(float, clamped_action)), "progress_vector": list(map(float, progress)),
        }
        raw_time_us = int(finish_us or start_us or release_us)
        record["time_s"] = raw_time_us / 1e6 if raw_time_us else None
        # Aliases feed the schema-independent event classifier.
        record.update({f"first_u{i + 1}": float(clamped_action[i]) for i in range(4)})
        record.update({"model": int(model_id), "residual": max(abs(float(primal)), abs(float(dual))),
                       "progress": float(progress[0]), "reference_progress": float(progress[1])})
        records.append(record)
        if plan_age_ticks != UINT32_MAX and plan_age_ticks > UINT32_NONWRAPPING_MAX:
            validation["errors"].append(
                f"record[{offset // record_size}].plan_age_ticks_underflow_suspected={plan_age_ticks}")
    if records and records[0]["event"] != 0:
        validation["errors"].append("first_record_is_not_stream_header")
    if records and records[0]["event_sequence"] != 1:
        validation["errors"].append(
            f"first_event_sequence={records[0]['event_sequence']},expected=1")
    tick_layout = infer_tick_layout(records)
    for record in records:
        record["tick_layout"] = tick_layout
        record["clock_domains"] = {
            "release_tick": ("freertos_tick" if tick_layout == "corrected_freertos"
                             else "legacy_stabilizer_tick"),
            "start_tick": "freertos_tick",
            "finish_tick": "freertos_tick",
            "plan_tick": "stabilizer_tick",
            "solve_tick_gap": "stabilizer_release_interval",
            "release_us_start_us_finish_us": "shared_monotonic_usec",
        }
    validation.update({"records": len(records), "sequence_gap_count": sequence_gaps,
                       "tick_layout": tick_layout,
                       "dropped_record_count_max": max((r["dropped_record_count"] for r in records), default=0),
                       "queue_high_water_count_max": max((r["queue_high_water_count"] or 0 for r in records), default=0)})
    if validation["dropped_record_count_max"]:
        validation["errors"].append(f"reported_dropped_records={validation['dropped_record_count_max']}")
    validation["valid"] = not validation["errors"] and bool(records)
    validation["payload_sha256"] = hashlib.sha256(payload).hexdigest()
    return records, validation


def parse_binary_diag(path: Path) -> tuple[list[dict[str, Any]], dict[str, Any]]:
    """Decode the legacy compact diagnostic container."""
    if not path.exists():
        return [], {"present": False, "valid": False, "errors": [],
                    "container": "compact_binary"}
    records, validation = _parse_binary_payload(path.read_bytes())
    validation["container"] = "compact_binary"
    return records, validation


def mmap_slot_checksum(raw_reservation_and_payload: bytes) -> int:
    """Return the controller's FNV-1a checksum over slot bytes 8..311."""
    checksum = MMAP_FNV_OFFSET_BASIS
    for value in raw_reservation_and_payload:
        checksum = ((checksum ^ value) * MMAP_FNV_PRIME) & UINT32_MAX
    return checksum


def parse_mmap_diag(path: Path, *, expected_firmware_tick_us: int | None = None
                    ) -> tuple[list[dict[str, Any]], dict[str, Any]]:
    """Decode and validate the fixed taskless mmap diagnostic container."""
    validation: dict[str, Any] = {
        "present": path.exists(), "valid": False, "authoritative": True,
        "container": "taskless_mmap", "errors": [],
        "expected_file_size": MMAP_FILE_SIZE,
    }
    if not path.exists():
        return [], validation
    payload = path.read_bytes()
    validation["bytes"] = len(payload)
    if len(payload) != MMAP_FILE_SIZE:
        validation["errors"].append(
            f"mmap_file_size={len(payload)},expected={MMAP_FILE_SIZE}")
    if len(payload) < MMAP_HEADER_SIZE:
        validation["errors"].append(f"truncated_mmap_header_bytes={len(payload)}")
        return [], validation
    if MMAP_HEADER_STRUCT.size != 88:
        raise AssertionError("internal mmap header layout mismatch")
    (magic, version, header_size, slot_size, record_size, capacity,
     firmware_tick_us, payload_magic, payload_version, reserved0,
     next_sequence, overflow_count, committed_count) = MMAP_HEADER_STRUCT.unpack_from(payload)
    header = {
        "magic": magic, "version": version, "header_size": header_size,
        "slot_size": slot_size, "record_size": record_size, "capacity": capacity,
        "firmware_tick_us": firmware_tick_us, "payload_magic": payload_magic,
        "payload_version": payload_version, "next_sequence": next_sequence,
        "overflow_count": overflow_count, "committed_count": committed_count,
    }
    validation["taskless_metadata"] = header
    expected_constants = {
        "magic": MMAP_MAGIC, "version": MMAP_VERSION,
        "header_size": MMAP_HEADER_SIZE, "slot_size": MMAP_SLOT_SIZE,
        "record_size": BINARY_RECORD_SIZE, "capacity": MMAP_CAPACITY,
        "payload_magic": BINARY_MAGIC, "payload_version": BINARY_VERSION,
    }
    for name, expected in expected_constants.items():
        if header[name] != expected:
            validation["errors"].append(
                f"mmap_header.{name}={header[name]},expected={expected}")
    if reserved0 != 0:
        validation["errors"].append(f"mmap_header.reserved0={reserved0}")
    if any(payload[32:64]) or any(payload[88:MMAP_HEADER_SIZE]):
        validation["errors"].append("mmap_header.reserved_bytes_nonzero")
    if firmware_tick_us <= 0 or firmware_tick_us > 1_000_000:
        validation["errors"].append(
            f"mmap_header.firmware_tick_us_invalid={firmware_tick_us}")
    if (expected_firmware_tick_us is not None and
            firmware_tick_us != expected_firmware_tick_us):
        validation["errors"].append(
            f"mmap_header.firmware_tick_us={firmware_tick_us},"
            f"config={expected_firmware_tick_us}")
    if next_sequence < 1:
        validation["errors"].append(f"mmap_header.next_sequence_invalid={next_sequence}")
    reservation_count = max(0, int(next_sequence) - 1)
    inspect_count = min(reservation_count, MMAP_CAPACITY)
    expected_overflow = max(0, reservation_count - MMAP_CAPACITY)
    if overflow_count != expected_overflow:
        validation["errors"].append(
            f"mmap_header.overflow_count={overflow_count},expected={expected_overflow}")
    if overflow_count:
        validation["errors"].append(f"mmap_overflow_count={overflow_count}")

    committed_payloads: list[tuple[int, bytes]] = []
    committed_sequences: set[int] = set()
    uncommitted_reserved = 0
    checksum_failures = 0
    for slot_index in range(inspect_count):
        offset = MMAP_HEADER_SIZE + slot_index * MMAP_SLOT_SIZE
        if offset + MMAP_SLOT_SIZE > len(payload):
            validation["errors"].append(f"mmap_slot[{slot_index}].truncated")
            break
        commit_sequence, reservation_sequence = struct.unpack_from("<QQ", payload, offset)
        expected_sequence = slot_index + 1
        raw_record = payload[offset + 16:offset + 312]
        stored_checksum, slot_reserved = struct.unpack_from("<II", payload, offset + 312)
        if commit_sequence == 0:
            uncommitted_reserved += 1
            validation["errors"].append(
                f"mmap_slot[{slot_index}].uncommitted_reserved_sequence={expected_sequence}")
            continue
        if commit_sequence != reservation_sequence:
            validation["errors"].append(
                f"mmap_slot[{slot_index}].torn_commit={commit_sequence},"
                f"reservation={reservation_sequence}")
        if commit_sequence != expected_sequence or reservation_sequence != expected_sequence:
            validation["errors"].append(
                f"mmap_slot[{slot_index}].sequence_out_of_range_or_wrong_slot="
                f"commit:{commit_sequence},reservation:{reservation_sequence},"
                f"expected:{expected_sequence}")
        if commit_sequence in committed_sequences:
            validation["errors"].append(
                f"mmap_duplicate_sequence={commit_sequence}")
        committed_sequences.add(commit_sequence)
        checksum = mmap_slot_checksum(payload[offset + 8:offset + 312])
        if stored_checksum != checksum:
            checksum_failures += 1
            validation["errors"].append(
                f"mmap_slot[{slot_index}].checksum=0x{stored_checksum:08x},"
                f"expected=0x{checksum:08x}")
        if slot_reserved != 0:
            validation["errors"].append(
                f"mmap_slot[{slot_index}].reserved={slot_reserved}")
        if len(raw_record) == BINARY_RECORD_SIZE:
            event_sequence = struct.unpack_from("<I", raw_record, 12)[0]
            if event_sequence != commit_sequence or event_sequence != reservation_sequence:
                validation["errors"].append(
                    f"mmap_slot[{slot_index}].payload_event_sequence={event_sequence},"
                    f"container_sequence={commit_sequence}")
            committed_payloads.append((int(commit_sequence), raw_record))
    available_slot_count = max(0, min(
        MMAP_CAPACITY, (len(payload) - MMAP_HEADER_SIZE) // MMAP_SLOT_SIZE))
    for slot_index in range(inspect_count, available_slot_count):
        offset = MMAP_HEADER_SIZE + slot_index * MMAP_SLOT_SIZE
        commit_sequence, reservation_sequence = struct.unpack_from("<QQ", payload, offset)
        if commit_sequence != 0 or reservation_sequence != 0:
            validation["errors"].append(
                f"mmap_slot[{slot_index}].data_above_reserved_count="
                f"commit:{commit_sequence},reservation:{reservation_sequence}")
    expected_sequences = set(range(1, inspect_count + 1))
    missing = sorted(expected_sequences - committed_sequences)
    if missing:
        validation["errors"].append(
            "mmap_sequence_holes=" + ",".join(map(str, missing[:32])))
    if committed_count != len(committed_sequences):
        validation["errors"].append(
            f"mmap_header.committed_count={committed_count},"
            f"observed={len(committed_sequences)}")

    committed_payloads.sort(key=lambda item: item[0])
    compact_payload = b"".join(item[1] for item in committed_payloads)
    records, compact_validation = _parse_binary_payload(
        compact_payload, present=bool(committed_payloads))
    validation.update({
        "schema_version": compact_validation.get("schema_version"),
        "record_size": compact_validation.get("record_size"),
        "records": len(records),
        "sequence_gap_count": compact_validation.get("sequence_gap_count", 0),
        "dropped_record_count_max": compact_validation.get("dropped_record_count_max", 0),
        "queue_high_water_count_max": compact_validation.get("queue_high_water_count_max", 0),
        "tick_layout": compact_validation.get("tick_layout", "unavailable"),
        "payload_sha256": hashlib.sha256(compact_payload).hexdigest(),
        "reserved_count": reservation_count,
        "inspected_slot_count": inspect_count,
        "committed_slot_count": len(committed_sequences),
        "uncommitted_reserved_count": uncommitted_reserved,
        "checksum_failure_count": checksum_failures,
    })
    validation["errors"].extend(
        f"payload:{error}" for error in compact_validation.get("errors", []))
    validation["valid"] = not validation["errors"] and bool(records)
    return records, validation


def _sha256_file(path: Path) -> str | None:
    if not path.is_file():
        return None
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def resolve_absolute_action_profile(config: dict[str, Any] | None) -> tuple[dict[str, Any] | None, str | None]:
    """Resolve command-to-RPM parameters with explicit source provenance."""
    config = config or {}
    contract = config.get("acceptance_contract", {})
    resolved = contract.get("resolved_plant_profile", {})
    coefficients = resolved.get("rpm_to_thrust")
    full_thrust = resolved.get("normalized_command_full_thrust_n")
    if coefficients is not None and full_thrust is not None:
        try:
            coefficients = tuple(float(value) for value in coefficients)
            full_thrust = float(full_thrust)
        except (TypeError, ValueError):
            return None, "run-config motor conversion parameters are malformed"
        if (len(coefficients) != 3 or not all(math.isfinite(value) for value in coefficients)
                or coefficients[2] <= 0 or not math.isfinite(full_thrust) or full_thrust <= 0):
            return None, "run-config motor conversion parameters are invalid"
        return {
            "rpm_to_thrust": coefficients,
            "normalized_command_full_thrust_n": full_thrust,
            "provenance": "run_config.acceptance_contract.resolved_plant_profile",
            "inferred": False,
            "profile_name": resolved.get("name", resolved.get("profile_name")),
        }, None

    canonical_path = (Path(__file__).resolve().parents[2] /
                      "apps/controller_tinympc_eigen/tools/crazysim_runtime_profile.py")
    actual_hash = _sha256_file(canonical_path)
    recorded_hash = contract.get("plant_model_sha256")
    model = config.get("model")
    profile_name = resolved.get("name", resolved.get("profile_name"))
    configured_full_thrust = config.get("pwm_thrust_full_n")
    identity_matches = (
        model == "cf21B_500" and profile_name == CANONICAL_CF21B_500_PROFILE_NAME
        and isinstance(recorded_hash, str) and recorded_hash == actual_hash
        and isinstance(configured_full_thrust, (int, float))
        and math.isclose(float(configured_full_thrust),
                         CANONICAL_CF21B_500_FULL_THRUST_N,
                         rel_tol=0.0, abs_tol=1e-12))
    if not identity_matches:
        return None, (
            "absolute-action motor profile unavailable or canonical cf21B_500 "
            "model/name/source identity does not match")
    return {
        "rpm_to_thrust": CANONICAL_CF21B_500_RPM_TO_THRUST,
        "normalized_command_full_thrust_n": CANONICAL_CF21B_500_FULL_THRUST_N,
        "provenance": "local_canonical_cf21B_500_profile",
        "inferred": True,
        "profile_name": CANONICAL_CF21B_500_PROFILE_NAME,
        "source_path": str(canonical_path),
        "source_sha256": actual_hash,
        "identity": {"model": model, "profile_name": profile_name,
                     "recorded_plant_model_sha256": recorded_hash},
    }, None


def normalized_actions_to_rpm(actions: np.ndarray, profile: dict[str, Any]) -> np.ndarray:
    """Reproduce firmware PWM quantization and CrazySim's PWM-to-RPM map."""
    actions_f32 = np.asarray(actions, dtype=np.float32)
    pwm = np.trunc(np.clip(actions_f32, 0.0, 1.0) * np.float32(65535.0)).astype(np.uint16)
    normalized = pwm.astype(np.float64) / 65535.0
    thrust = normalized * normalized * float(profile["normalized_command_full_thrust_n"])
    a, b, c = map(float, profile["rpm_to_thrust"])
    discriminant = np.maximum(b * b - 4.0 * c * (a - thrust), 0.0)
    rpm = (-b + np.sqrt(discriminant)) / (2.0 * c)
    maximum_discriminant = max(b * b - 4.0 * c * (
        a - float(profile["normalized_command_full_thrust_n"])), 0.0)
    maximum_rpm = (-b + math.sqrt(maximum_discriminant)) / (2.0 * c)
    rpm = np.clip(rpm, 0.0, maximum_rpm)
    return np.where(pwm < 7000, 0.0, rpm)


def _apply_binary_alignment(records: list[dict[str, Any]], data: dict[str, np.ndarray],
                            config: dict[str, Any] | None, slope: float, offset: float,
                            *, use_microseconds: bool, tick_us: int | None = None) -> None:
    for record in records:
        if use_microseconds:
            raw = (record["finish_us"] or record["start_us"] or record["release_us"]) / 1e6
        else:
            raw = (record["finish_tick"] or record["start_tick"]) * float(tick_us) / 1e6
        record["time_s"] = slope * raw + offset if raw else None
        if record["time_s"] is not None:
            index = int(np.argmin(np.abs(data["time_s"] - record["time_s"])))
            if all(key in data for key in ("x_m", "y_m", "z_m")):
                record.update({f"truth_{axis}": float(data[f"{axis}_m"][index]) for axis in "xyz"})
                record.update({f"est_{axis}": float(record["estimator"][i]) for i, axis in enumerate("xyz")})
            if all(key in data for key in ("vx_mps", "vy_mps", "vz_mps")):
                estimate = np.asarray(record["estimator"][3:6]); truth = np.asarray([data[f"v{axis}_mps"][index] for axis in "xyz"])
                record["velocity_error_mps"] = float(np.linalg.norm(estimate - truth))
            if all(key in data for key in ("qw", "qx", "qy", "qz")):
                estimate_q = np.asarray([record["estimator"][9], *record["estimator"][6:9]])
                truth_q = np.asarray([data[key][index] for key in ("qw", "qx", "qy", "qz")])
                denominator = np.linalg.norm(estimate_q) * np.linalg.norm(truth_q)
                dot = float(np.clip(abs(np.dot(estimate_q, truth_q) / denominator), 0, 1)) if denominator else 0.0
                record["attitude_error_deg"] = float(np.rad2deg(2 * np.arccos(dot)))
            if all(key in data for key in ("wx_radps", "wy_radps", "wz_radps")):
                truth_rate = np.asarray([data[key][index] for key in ("wx_radps", "wy_radps", "wz_radps")])
                record["body_rate_error_rad_s"] = float(np.linalg.norm(np.asarray(record["estimator"][10:13]) - truth_rate))
            if (config or {}).get("trajectory") == "circle" and all(key in data for key in ("x_m", "y_m")):
                record["cross_track_m"] = abs(float(np.hypot(data["x_m"][index], data["y_m"][index] - .75) - .75))


def _align_binary_absolute_actions(records: list[dict[str, Any]], data: dict[str, np.ndarray],
                                   config: dict[str, Any] | None = None) -> dict[str, Any]:
    result: dict[str, Any] = {
        "observable": False, "method": "absolute_rpm_reference_pattern",
        "maximum_uncertainty_s": None, "matched_transition_count": 0,
        "clock_offset_s": None, "match_tolerance_rpm": ABSOLUTE_RPM_MATCH_TOLERANCE,
    }
    ref_keys = [f"rpm_ref_{i}" for i in range(1, 5)]
    if "time_s" not in data or not all(key in data for key in ref_keys):
        result["reason"] = "state RPM-reference columns unavailable"; return result
    profile, profile_error = resolve_absolute_action_profile(config)
    if profile is None:
        result["reason"] = profile_error
        return result
    result["motor_mapping"] = profile
    solves = [record for record in records if record.get("event") == 1 and
              (record.get("finish_us") or record.get("start_us") or record.get("release_us"))]
    if len(solves) < 3:
        result["reason"] = "fewer than three solve records with microsecond timestamps"; return result
    state_ref = np.column_stack([data[key] for key in ref_keys])
    finite = np.all(np.isfinite(state_ref), axis=1)
    transition_mask = (np.max(np.abs(np.diff(state_ref, axis=0)), axis=1) > .5
                       ) & finite[1:] & finite[:-1]
    transition_indices = np.flatnonzero(transition_mask) + 1
    transition_vectors = state_ref[transition_indices]
    result["state_transition_count"] = int(len(transition_indices))
    if len(transition_indices) < 3:
        result["reason"] = "fewer than three actual RPM-reference transitions"; return result

    candidates: list[tuple[int, int, float, float]] = []
    ambiguous = unmatched = 0
    for solve_index, record in enumerate(solves):
        action = np.asarray(record.get("first_action_clamped", []), dtype=float)
        if action.shape != (4,) or not np.all(np.isfinite(action)):
            unmatched += 1
            continue
        predicted_rpm = normalized_actions_to_rpm(action, profile)
        matches = np.flatnonzero(
            np.max(np.abs(transition_vectors - predicted_rpm), axis=1)
            <= ABSOLUTE_RPM_MATCH_TOLERANCE)
        if len(matches) != 1:
            ambiguous += int(len(matches) > 1)
            unmatched += int(len(matches) == 0)
            continue
        state_match = int(matches[0])
        firmware_time = (record["finish_us"] or record["start_us"] or
                         record["release_us"]) / 1e6
        candidates.append((solve_index, state_match, firmware_time,
                           float(data["time_s"][transition_indices[state_match]])))
    state_match_counts: dict[int, int] = {}
    for _, state_match, _, _ in candidates:
        state_match_counts[state_match] = state_match_counts.get(state_match, 0) + 1
    unique_candidates = [candidate for candidate in candidates
                         if state_match_counts[candidate[1]] == 1]
    result.update({
        "ambiguous_pattern_count": ambiguous,
        "unmatched_pattern_count": unmatched,
        "duplicate_state_match_count": len(candidates) - len(unique_candidates),
        "unique_candidate_count": len(unique_candidates),
    })
    unique_candidates.sort(key=lambda item: item[2])
    chains: list[list[tuple[int, int, float, float]]] = []
    for candidate in unique_candidates:
        previous = [chain for chain in chains
                    if chain[-1][2] < candidate[2] and chain[-1][1] < candidate[1]]
        chains.append((max(previous, key=len) + [candidate]) if previous else [candidate])
    chain = max(chains, key=len) if chains else []
    if len(chain) < 3:
        result["reason"] = "fewer than three unique ordered absolute-action matches"; return result
    x = np.asarray([candidate[2] for candidate in chain])
    y = np.asarray([candidate[3] for candidate in chain])
    slope, offset = np.polyfit(x, y, 1)
    if not .5 <= slope <= 2.0:
        result["reason"] = f"implausible fitted firmware-to-sim clock slope {slope:.6g}"; return result
    residuals = np.abs(y - (slope * x + offset))
    maximum = float(np.max(residuals))
    result.update({
        "observable": True, "firmware_clock_source": "microseconds",
        "matched_transition_count": len(chain), "maximum_uncertainty_s": maximum,
        "mean_absolute_residual_s": float(np.mean(residuals)),
        "clock_offset_s": float(offset), "clock_slope": float(slope),
        "causal_alignment_valid": maximum <= .020,
    })
    _apply_binary_alignment(records, data, config, float(slope), float(offset),
                            use_microseconds=True)
    return result


def align_binary_to_state(records: list[dict[str, Any]], data: dict[str, np.ndarray],
                          config: dict[str, Any] | None = None) -> dict[str, Any]:
    absolute = _align_binary_absolute_actions(records, data, config)
    if absolute.get("observable"):
        return absolute
    fallback = _align_binary_transition_deltas(records, data, config)
    fallback["absolute_action_attempt"] = absolute
    return fallback


def _align_binary_transition_deltas(records: list[dict[str, Any]], data: dict[str, np.ndarray],
                                    config: dict[str, Any] | None = None) -> dict[str, Any]:
    """Align firmware microseconds to RPM-reference command transitions."""
    result: dict[str, Any] = {"observable": False, "method": "rpm_ref_transition", "maximum_uncertainty_s": None,
                              "matched_transition_count": 0, "clock_offset_s": None}
    ref_keys = [f"rpm_ref_{i}" for i in range(1, 5)]
    if "time_s" not in data or not all(key in data for key in ref_keys):
        result["reason"] = "state RPM-reference columns unavailable"; return result
    solves = [r for r in records if r["event"] == 1]
    if len(solves) < 2:
        result["reason"] = "fewer than two binary solve records"; return result
    microsecond_solves = [r for r in solves if r["finish_us"] or r["start_us"] or r["release_us"]]
    if len(microsecond_solves) >= 2:
        solves = microsecond_solves
        use_microseconds = True
        diag_times = np.asarray([(r["finish_us"] or r["start_us"] or r["release_us"]) / 1e6 for r in solves])
        result["firmware_clock_source"] = "microseconds"
    else:
        if any(r.get("tick_layout") != "corrected_freertos" for r in solves):
            result["reason"] = "legacy mixed tick layout requires shared microsecond timestamps"; return result
        tick_us = configured_firmware_tick_us(config)
        if tick_us is None:
            result["reason"] = "firmware tick period unavailable for tick-only alignment"; return result
        solves = [r for r in solves if r["finish_tick"] or r["start_tick"]]
        if len(solves) < 2:
            result["reason"] = "fewer than two solves with a shared clock timestamp"; return result
        use_microseconds = False
        # start_tick and finish_tick share the FreeRTOS domain.  release_tick
        # is a stabilizer tick and must never be substituted here.
        diag_times = np.asarray([
            (r["finish_tick"] or r["start_tick"]) * tick_us / 1e6 for r in solves])
        result["firmware_clock_source"] = "freertos_ticks"
        result["firmware_tick_us"] = tick_us
    diag_actions = np.asarray([r["first_action_clamped"] for r in solves])
    diag_steps = np.max(np.abs(np.diff(diag_actions, axis=0)), axis=1)
    diag_mask = diag_steps > .01
    diag_transition = diag_times[1:][diag_mask]
    diag_delta = np.diff(diag_actions, axis=0)[diag_mask]
    state_ref = np.column_stack([data[key] for key in ref_keys])
    state_scale = max(float(np.nanmax(np.abs(state_ref))), 1.0)
    state_steps = np.nanmax(np.abs(np.diff(state_ref, axis=0)), axis=1) / state_scale
    state_mask = state_steps > .01
    state_transition = data["time_s"][1:][state_mask]
    state_delta = np.diff(state_ref, axis=0)[state_mask]
    if len(diag_transition) < 2 or len(state_transition) < 2:
        result["reason"] = "fewer than two action transitions in either stream"; return result
    # Match the signed four-motor transition pattern before considering time.
    compatible: list[tuple[int, int]] = []
    for i, first in enumerate(diag_delta):
        for j, second in enumerate(state_delta):
            denominator = float(np.linalg.norm(first) * np.linalg.norm(second))
            cosine = float(np.dot(first, second) / denominator) if denominator else -1.0
            if cosine >= .80:
                compatible.append((i, j))
    # Longest order-preserving compatible set (small streams; exhaustive DP).
    chains: list[list[tuple[int, int]]] = []
    for pair in compatible:
        previous = [chain for chain in chains if chain[-1][0] < pair[0] and chain[-1][1] < pair[1]]
        chains.append((max(previous, key=len) + [pair]) if previous else [pair])
    chain = max(chains, key=len) if chains else []
    if len(chain) < 2:
        result["reason"] = "fewer than two order-preserving motor-pattern matches"; return result
    x = np.asarray([diag_transition[i] for i, _ in chain]); y = np.asarray([state_transition[j] for _, j in chain])
    slope, offset = np.polyfit(x, y, 1)
    predicted = slope * x + offset; residuals = np.abs(y - predicted)
    if not .5 <= slope <= 2.0:
        result["reason"] = f"implausible fitted firmware-to-sim clock slope {slope:.6g}"; return result
    maximum = float(np.max(residuals))
    result.update({"observable": True, "matched_transition_count": len(chain), "maximum_uncertainty_s": maximum,
                   "mean_absolute_residual_s": float(np.mean(residuals)), "clock_offset_s": float(offset),
                   "clock_slope": float(slope), "causal_alignment_valid": maximum <= .020})
    for record in records:
        if use_microseconds:
            raw = (record["finish_us"] or record["start_us"] or record["release_us"]) / 1e6
        else:
            raw = (record["finish_tick"] or record["start_tick"]) * tick_us / 1e6
        record["time_s"] = slope * raw + offset if raw else None
        if record["time_s"] is not None:
            index = int(np.argmin(np.abs(data["time_s"] - record["time_s"])))
            if all(key in data for key in ("x_m", "y_m", "z_m")):
                record.update({f"truth_{axis}": float(data[f"{axis}_m"][index]) for axis in "xyz"})
                record.update({f"est_{axis}": float(record["estimator"][i]) for i, axis in enumerate("xyz")})
            if all(key in data for key in ("vx_mps", "vy_mps", "vz_mps")):
                estimate = np.asarray(record["estimator"][3:6]); truth = np.asarray([data[f"v{axis}_mps"][index] for axis in "xyz"])
                record["velocity_error_mps"] = float(np.linalg.norm(estimate - truth))
            if all(key in data for key in ("qw", "qx", "qy", "qz")):
                estimate_q = np.asarray([record["estimator"][9], *record["estimator"][6:9]])
                truth_q = np.asarray([data[key][index] for key in ("qw", "qx", "qy", "qz")])
                denominator = np.linalg.norm(estimate_q) * np.linalg.norm(truth_q)
                dot = float(np.clip(abs(np.dot(estimate_q, truth_q) / denominator), 0, 1)) if denominator else 0.0
                record["attitude_error_deg"] = float(np.rad2deg(2 * np.arccos(dot)))
            if all(key in data for key in ("wx_radps", "wy_radps", "wz_radps")):
                truth_rate = np.asarray([data[key][index] for key in ("wx_radps", "wy_radps", "wz_radps")])
                record["body_rate_error_rad_s"] = float(np.linalg.norm(np.asarray(record["estimator"][10:13]) - truth_rate))
            if (config or {}).get("trajectory") == "circle" and all(key in data for key in ("x_m", "y_m")):
                record["cross_track_m"] = abs(float(np.hypot(data["x_m"][index], data["y_m"][index] - .75) - .75))
    return result


def first_index(mask: np.ndarray) -> int | None:
    indices = np.flatnonzero(mask)
    return int(indices[0]) if indices.size else None


def quaternion_tilt_deg(data: dict[str, np.ndarray]) -> np.ndarray | None:
    required = ("qw", "qx", "qy", "qz")
    if not all(key in data for key in required):
        return None
    qw, qx, qy, qz = (data[key] for key in required)
    roll = np.arctan2(2 * (qw * qx + qy * qz), 1 - 2 * (qx * qx + qy * qy))
    pitch = np.arcsin(np.clip(2 * (qw * qy - qz * qx), -1, 1))
    return np.rad2deg(np.hypot(roll, pitch))


def state_events(data: dict[str, np.ndarray], launch: float) -> list[dict[str, Any]]:
    if "time_s" not in data:
        return []
    t = data["time_s"]
    events: list[dict[str, Any]] = []
    def emit(kind: str, index: int | None, **values: Any) -> None:
        if index is not None:
            events.append({"time_s": float(t[index]), "kind": kind, **values})
    airborne = t >= launch
    if "contacts" in data:
        emit("contact", first_index(airborne & (data["contacts"] > 0)), contacts=1)
    tilt = quaternion_tilt_deg(data)
    if tilt is not None:
        emit("physical_divergence_onset", first_index(airborne & (tilt >= 30.0)), metric="tilt_deg", value=30.0)
    if all(key in data for key in ("wx_radps", "wy_radps", "wz_radps")):
        rate = np.sqrt(sum(data[key] ** 2 for key in ("wx_radps", "wy_radps", "wz_radps")))
        emit("high_angular_rate", first_index(airborne & (rate >= 20.0)), metric="angular_rate_rad_s", value=20.0)
    return events


def diag_events(records: list[dict[str, Any]]) -> list[dict[str, Any]]:
    events: list[dict[str, Any]] = []
    last_model: Any = None
    have_model = False
    last_action: np.ndarray | None = None
    for record in records:
        time = record["time_s"]
        if time is None:
            continue
        def emit(kind: str, **fields: Any) -> None:
            events.append({"time_s": float(time), "kind": kind, **fields})
        event_name = str(pick(record, ("event", "phase", "kind")) or "").lower()
        if any(key in record for key in ("planner_release", "release_seq")) or "release" in event_name:
            emit("planner_release", sequence=pick(record, ("release_seq", "plan_seq", "sequence")))
        if any(key in record for key in ("planner_start", "start_seq")) or "start" in event_name:
            emit("planner_start", sequence=pick(record, ("start_seq", "plan_seq", "sequence")))
        if any(key in record for key in ("planner_finish", "finish_seq")) or "finish" in event_name:
            emit("planner_finish", sequence=pick(record, ("finish_seq", "plan_seq", "sequence")))
        if any(key in record for key in ("solve_us", "iterations", "solver_status")):
            emit("solver_observation", solve_us=pick(record, ("solve_us", "solve_time_us")),
                 iterations=pick(record, ("iterations", "solver_iterations")))
        fallback = pick(record, ("fallback", "hover_fallback", "plan_fallback", "fallback_reason"))
        age = pick(record, ("plan_age_s", "plan_age_ms", "age_s"))
        age_s = float(age) / 1000.0 if "plan_age_ms" in record and isinstance(age, float) else age
        binary_fallback = int(fallback) if "binary_offset" in record and isinstance(fallback, (int, float)) else None
        if binary_fallback == 3 or (binary_fallback is None and
                (fallback not in (None, 0.0, "0", "none", "NONE") or (isinstance(age_s, float) and age_s > .060))):
            emit("stale_plan_fallback", reason=FALLBACK_REASONS.get(binary_fallback, str(fallback or "plan_age")), plan_age_s=age_s)
        elif binary_fallback not in (None, 0):
            emit("motor_fallback_transition", reason=FALLBACK_REASONS.get(binary_fallback, f"unknown_{binary_fallback}"), plan_age_s=age_s)
        model = pick(record, ("model", "model_id", "model_index", "selected_model"))
        if model is not None and model != last_model:
            if have_model:
                emit("model_switch", previous=str(last_model), model=str(model))
            last_model = model
            have_model = True
        residual = pick(record, ("residual", "primal_residual", "dual_residual", "solver_residual"))
        if isinstance(residual, float) and (not math.isfinite(residual) or abs(residual) > 1.0):
            emit("solver_residual_anomaly", residual=residual)
        numeric_flags = record.get("numeric_flags")
        optimizer_max = record.get("optimizer_max_abs")
        numeric_fault_flags = (int(numeric_flags) & 0xFFFF
                               if isinstance(numeric_flags, (int, float)) else 0)
        scalar_trigger_flags = (int(numeric_flags) >> 16
                                if isinstance(numeric_flags, (int, float)) else 0)
        scalar_trigger_labels = [label for bit, label in SCALAR_TRIGGER_LABELS.items()
                                 if scalar_trigger_flags & bit]
        if scalar_trigger_labels:
            emit("diagnostic_scalar_trigger", scalar_trigger_flags=scalar_trigger_flags,
                 triggers=scalar_trigger_labels)
        if numeric_fault_flags != 0 or (
                isinstance(optimizer_max, float) and (not math.isfinite(optimizer_max) or optimizer_max > 1e6)):
            emit("optimizer_numerical_excursion", numeric_fault_flags=numeric_fault_flags,
                 numeric_flags=numeric_flags, optimizer_max_abs=optimizer_max)
        clamp = pick(record, ("clamp_mask", "u_clamp_mask", "clamp"))
        if clamp not in (None, 0.0, "0", "0x00"):
            emit("actuator_clamp", clamp=str(clamp))
        est = [pick(record, (f"est_{axis}", f"est_{axis}_m")) for axis in "xyz"]
        truth = [pick(record, (f"truth_{axis}", f"truth_{axis}_m", f"sim_{axis}")) for axis in "xyz"]
        errors: dict[str, float] = {}
        if all(isinstance(value, float) for value in est + truth):
            error = float(np.linalg.norm(np.asarray(est) - np.asarray(truth)))
            errors["position_error_m"] = error
        for key in ("velocity_error_mps", "attitude_error_deg", "body_rate_error_rad_s"):
            if isinstance(record.get(key), float): errors[key] = float(record[key])
        if (errors.get("position_error_m", 0) > .15 or errors.get("velocity_error_mps", 0) > .5
                or errors.get("attitude_error_deg", 0) > 15 or errors.get("body_rate_error_rad_s", 0) > 2):
            emit("estimator_truth_excursion", **errors)
        cross = pick(record, ("cross_track_m", "cross_track_error_m", "reference_error_m"))
        if isinstance(cross, float) and cross > .15:
            emit("cross_track_recovery_demand", cross_track_m=cross)
        if any(key in record for key in ("progress", "reference_progress", "reference_x", "ref_x")):
            emit("reference_observation", progress=pick(record, ("progress", "reference_progress")),
                 cross_track_m=cross)
        action_values = [record[key] for key in ACTION_KEYS if key in record and isinstance(record[key], float)]
        if len(action_values) >= 4:
            action = np.asarray(action_values[:4])
            if last_action is not None and np.max(np.abs(action - last_action)) > .5 and (np.any(action < .05) or np.any(action > .95)):
                emit("fresh_plan_rail_oscillation", max_action_step=float(np.max(np.abs(action - last_action))))
            last_action = action
    return events


def binary_lifecycle_events(records: list[dict[str, Any]], alignment: dict[str, Any],
                            causal_eligible: bool) -> list[dict[str, Any]]:
    if not alignment.get("observable"):
        return []
    slope = float(alignment["clock_slope"]); offset = float(alignment["clock_offset_s"])
    use_microseconds = alignment.get("firmware_clock_source") == "microseconds"
    tick_layout = next((record.get("tick_layout") for record in records
                        if record["event"] == 1), "unavailable")
    if not use_microseconds and tick_layout != "corrected_freertos":
        return []
    events: list[dict[str, Any]] = []
    previous_coalesced = previous_mutex = 0
    for record in records:
        # Fallback-transition records are published by the control callback at
        # the instant a plan becomes stale.  Their cumulative release counters
        # can therefore be the only pre-divergence evidence of coalescing when
        # the next solve record is delayed until after the vehicle departs.
        if record["event"] != 1:
            counter_time = record.get("time_s")
            if isinstance(counter_time, float) and math.isfinite(counter_time):
                coalesced = record["semaphore_coalesced_count"]
                if coalesced > previous_coalesced:
                    events.append({"time_s": counter_time, "kind": "semaphore_coalescing",
                                   "count_delta": coalesced - previous_coalesced,
                                   "causal_eligible": causal_eligible})
                mutex = record["release_mutex_miss_count"]
                if mutex > previous_mutex:
                    events.append({"time_s": counter_time, "kind": "mutex_release_miss",
                                   "count_delta": mutex - previous_mutex,
                                   "causal_eligible": causal_eligible})
                previous_coalesced, previous_mutex = coalesced, mutex
            continue
        if use_microseconds:
            release_raw = record["release_us"]
            start_raw = record["start_us"]
            finish_raw = record["finish_us"]
            scale = 1e6
        else:
            release_raw = record["release_tick"]
            start_raw = record["start_tick"]
            finish_raw = record["finish_tick"]
            tick_us = alignment.get("firmware_tick_us")
            if not isinstance(tick_us, (int, float)) or tick_us <= 0:
                continue
            scale = 1e6 / float(tick_us)
        start_latency_raw = nonwrapping_elapsed(start_raw, release_raw)
        solve_duration_raw = nonwrapping_elapsed(finish_raw, start_raw)
        if start_latency_raw is None or solve_duration_raw is None:
            continue
        release_time = slope * release_raw / scale + offset
        start_time = slope * start_raw / scale + offset
        finish_time = slope * finish_raw / scale + offset
        events.extend([
            {"time_s": release_time, "kind": "planner_release", "sequence": record["release_sequence"], "causal_eligible": causal_eligible},
            {"time_s": start_time, "kind": "planner_start", "sequence": record["release_sequence"], "causal_eligible": causal_eligible},
            {"time_s": finish_time, "kind": "planner_finish", "sequence": record["solve_sequence"], "causal_eligible": causal_eligible},
        ])
        start_latency = slope * start_latency_raw / scale
        solve_duration = slope * solve_duration_raw / scale
        if start_latency > .020:
            events.append({"time_s": start_time, "kind": "scheduler_start_latency", "latency_s": start_latency, "causal_eligible": causal_eligible})
        if solve_duration > .020:
            events.append({"time_s": finish_time, "kind": "long_worker_solve", "duration_s": solve_duration, "causal_eligible": causal_eligible})
        coalesced = record["semaphore_coalesced_count"]
        if coalesced > previous_coalesced:
            events.append({"time_s": release_time, "kind": "semaphore_coalescing", "count_delta": coalesced - previous_coalesced, "causal_eligible": causal_eligible})
        mutex = record["release_mutex_miss_count"]
        if mutex > previous_mutex:
            events.append({"time_s": release_time, "kind": "mutex_release_miss", "count_delta": mutex - previous_mutex, "causal_eligible": causal_eligible})
        previous_coalesced, previous_mutex = coalesced, mutex
    return events


def causally_preceding_kinds(events: list[dict[str, Any]], launch: float,
                             divergence: dict[str, Any] | None) -> list[str]:
    """Return eligible precursor kinds from the active flight only."""
    if divergence is None:
        return []
    return [event["kind"] for event in events
            if launch <= event["time_s"] < divergence["time_s"]
            and event.get("causal_eligible", True)]


def rotor_lag_events(data: dict[str, np.ndarray]) -> list[dict[str, Any]]:
    keys = [f"rpm_{i}" for i in range(1, 5)]
    refs = [f"rpm_ref_{i}" for i in range(1, 5)]
    if "time_s" not in data or not all(key in data for key in keys + refs):
        return []
    actual = np.column_stack([data[key] for key in keys]); reference = np.column_stack([data[key] for key in refs])
    error = np.max(np.abs(actual - reference), axis=1)
    scale = np.maximum(np.max(np.abs(reference), axis=1), 1.0)
    index = first_index(error / scale > .20)
    return [] if index is None else [{"time_s": float(data["time_s"][index]), "kind": "rotor_phase_lag", "relative_rpm_error": float(error[index] / scale[index])}]


def timing_validity(config: dict[str, Any], summary: dict[str, Any]) -> dict[str, Any]:
    rt = config.get("realtime_factor"); fw = config.get("firmware_time_factor")
    requested = config.get("launch_time_s", config.get("launch_time")); actual = summary.get("launch_time_s")
    launch_ok = requested is None or actual is None or abs(float(requested) - float(actual)) <= .5
    return {"realtime_factor": rt, "firmware_time_factor": fw, "launch_time_matches_request": launch_ok,
            "valid": bool(launch_ok and (rt is None or float(rt) == 1.0) and (fw is None or float(fw) == .8))}


def binary_terminal_coverage(records: list[dict[str, Any]], data: dict[str, np.ndarray],
                             state_event_list: list[dict[str, Any]],
                             alignment: dict[str, Any]) -> dict[str, Any]:
    """Prove the async diagnostic drain covers the causal observation window."""
    if not alignment.get("observable"):
        return {"valid": False, "target_kind": None, "target_time_s": None,
                "last_aligned_diagnostic_time_s": None, "coverage_gap_s": None,
                "reason": "binary/state alignment unavailable"}
    divergence = next((event for event in state_event_list
                       if event["kind"] == "physical_divergence_onset"), None)
    if divergence is not None:
        target = float(divergence["time_s"]); target_kind = "physical_divergence_onset"
    elif "time_s" in data and len(data["time_s"]):
        target = float(data["time_s"][-1]); target_kind = "active_state_end"
    else:
        return {"valid": False, "target_kind": None, "target_time_s": None,
                "last_aligned_diagnostic_time_s": None, "coverage_gap_s": None,
                "reason": "state coverage target unavailable"}
    aligned = [float(record["time_s"]) for record in records
               if isinstance(record.get("time_s"), float) and math.isfinite(record["time_s"])]
    if not aligned:
        return {"valid": False, "target_kind": target_kind, "target_time_s": target,
                "last_aligned_diagnostic_time_s": None, "coverage_gap_s": None,
                "reason": "no aligned diagnostic record timestamps"}
    last = max(aligned); gap = max(0.0, target - last)
    valid = gap <= .0200001
    return {"valid": valid, "target_kind": target_kind, "target_time_s": target,
            "last_aligned_diagnostic_time_s": last, "coverage_gap_s": gap,
            "maximum_allowed_gap_s": .020,
            "reason": None if valid else "diagnostic stream ends more than 20 ms before coverage target"}


def diagnostic_observability(records: list[dict[str, Any]]) -> dict[str, bool]:
    """State explicitly whether a failure class was measurable in this run."""
    available = {key for record in records for key in record}
    def has(*names: str) -> bool: return any(name in available for name in names)
    return {
        "planner_timestamps": has("planner_release", "release_seq", "release_tick", "release_us", "planner_start", "start_seq", "start_tick", "start_us", "planner_finish", "finish_seq", "finish_tick", "finish_us"),
        "plan_age_or_fallback": has("plan_age_s", "plan_age_ms", "fallback", "hover_fallback", "fallback_reason"),
        "solver_model_residual_clamp": has("model", "model_id", "model_index", "residual", "primal_residual", "clamp_mask"),
        "estimator_vs_truth": has("est_x", "est_x_m") and has("truth_x", "truth_x_m", "sim_x"),
        "reference_progress": has("progress", "reference_progress"),
        "cross_track": has("cross_track_m", "cross_track_error_m"),
        "first_action": has("u1", "first_u1", "action_1", "motor_1"),
    }


def analyze_run(run: Path) -> tuple[dict[str, Any], list[dict[str, Any]], dict[str, np.ndarray]]:
    config_path = run / "config.json"
    if not config_path.exists(): config_path = run / "run_config.json"
    config = json.loads(config_path.read_text()) if config_path.exists() else {}
    summary = json.loads((run / "summary.json").read_text()) if (run / "summary.json").exists() else {}
    launch = float(summary.get("launch_time_s", config.get("launch_time_s", config.get("launch_time", 0.0))))
    data = load_csv(run / "state.csv") if (run / "state.csv").exists() else {}
    diagnostic_config = config.get("mpc_diagnostic", {})
    expected_tick_us = diagnostic_config.get("firmware_tick_us")
    expected_tick_us = int(expected_tick_us) if expected_tick_us is not None else None
    mmap_path = run / "mpc_diag.mmap"
    compact_path = run / "mpc_diag.bin"
    if mmap_path.exists():
        binary_records, binary_validation = parse_mmap_diag(
            mmap_path, expected_firmware_tick_us=expected_tick_us)
        diagnostic_source = "taskless_mmap"
        if compact_path.exists():
            compact_records, compact_validation = parse_binary_diag(compact_path)
            matches = (compact_validation.get("payload_sha256") ==
                       binary_validation.get("payload_sha256"))
            binary_validation["derived_compact_cross_check"] = {
                "present": True, "matches": matches,
                "records": len(compact_records),
                "valid": compact_validation.get("valid", False),
            }
            if not compact_validation.get("valid"):
                binary_validation["errors"].append(
                    "derived_compact_binary_validation_failed")
            if not matches:
                binary_validation["errors"].append(
                    "derived_compact_binary_payload_mismatch")
            binary_validation["valid"] = not binary_validation["errors"] and bool(binary_records)
        else:
            binary_validation["derived_compact_cross_check"] = {
                "present": False, "matches": None,
            }
    else:
        binary_records, binary_validation = parse_binary_diag(compact_path)
        diagnostic_source = "binary" if binary_records else "text_compatibility"
    binary_alignment = align_binary_to_state(binary_records, data, config) if binary_records else {
        "observable": False, "method": "rpm_ref_transition", "maximum_uncertainty_s": None,
        "matched_transition_count": 0, "clock_offset_s": None, "reason": "binary diagnostic stream unavailable"}
    text_records = parse_diag(run / "firmware.log", launch)
    # Binary is authoritative. Text remains useful for pre-instrumentation runs.
    records = binary_records if binary_records else text_records
    state_event_list = state_events(data, launch)
    terminal_coverage = binary_terminal_coverage(binary_records, data, state_event_list, binary_alignment)
    binary_causal_eligible = bool(binary_validation.get("valid") and binary_validation.get("authoritative")
                                  and binary_alignment.get("causal_alignment_valid")
                                  and terminal_coverage.get("valid"))
    diagnostic_container_present = bool(binary_validation.get("present"))
    diagnostic_events = diag_events(records)
    if binary_records:
        for event in diagnostic_events:
            event["causal_eligible"] = binary_causal_eligible
    events = (state_event_list + diagnostic_events + rotor_lag_events(data)
              + binary_lifecycle_events(binary_records, binary_alignment, binary_causal_eligible))
    events.sort(key=lambda event: event["time_s"])
    divergence = next((event for event in events if event["kind"] == "physical_divergence_onset"), None)
    preceding = causally_preceding_kinds(events, launch, divergence)
    classifications = {
        "stale_plan_fallback_before_divergence": "stale_plan_fallback" in preceding,
        "fresh_plan_rail_oscillation_before_divergence": "fresh_plan_rail_oscillation" in preceding and "stale_plan_fallback" not in preceding,
        "estimator_excursion_before_divergence": "estimator_truth_excursion" in preceding,
        "rotor_lag_before_divergence": "rotor_phase_lag" in preceding,
        "scheduler_start_latency_before_divergence": "scheduler_start_latency" in preceding,
        "long_worker_solve_before_divergence": "long_worker_solve" in preceding,
        "semaphore_coalescing_before_divergence": "semaphore_coalescing" in preceding,
        "mutex_release_miss_before_divergence": "mutex_release_miss" in preceding,
    }
    report = {"run": str(run), "launch_time_s": launch, "samples": int(len(data.get("time_s", []))),
              "diagnostic_records": len(records), "diagnostic_source": diagnostic_source,
              "binary_validation": binary_validation, "binary_state_alignment": binary_alignment,
              "binary_terminal_coverage": terminal_coverage,
              "binary_causal_eligibility": {
                  "eligible": binary_causal_eligible,
                  "requires_valid_stream": bool(binary_validation.get("valid")),
                  "requires_authoritative_schema": bool(binary_validation.get("authoritative")),
                  "requires_causal_alignment": bool(binary_alignment.get("causal_alignment_valid")),
                  "requires_terminal_coverage": bool(terminal_coverage.get("valid")),
                  "inconclusive_reason": (None if binary_causal_eligible else
                      "binary integrity validation failed" if diagnostic_container_present and not binary_validation.get("valid") else
                      "legacy binary schema is non-authoritative" if diagnostic_container_present and not binary_validation.get("authoritative") else
                      "binary/state alignment unavailable or exceeds 20 ms" if diagnostic_container_present and not binary_alignment.get("causal_alignment_valid") else
                      "terminal diagnostic coverage ends more than 20 ms before causal target" if diagnostic_container_present and not terminal_coverage.get("valid") else
                      "binary diagnostic stream unavailable"),
              },
              "timing": timing_validity(config, summary),
              "events": events, "classifications": classifications,
              "diagnostic_observability": diagnostic_observability(records),
              "data_availability": {"state": bool(data), "diagnostic": bool(records), "estimator_truth": any("est_x" in r and "truth_x" in r for r in records)}}
    return report, events, data


def write_event_csv(path: Path, reports: list[dict[str, Any]]) -> None:
    fields = (
        "run", "time_s", "kind", "causal_eligible", "reason",
        "plan_age_s", "count_delta", "latency_s", "duration_s",
        "solve_us", "iterations", "residual", "numeric_fault_flags",
        "numeric_flags", "optimizer_max_abs", "clamp", "max_action_step",
        "position_error_m", "velocity_error_mps", "attitude_error_deg",
        "body_rate_error_rad_s", "relative_rpm_error", "cross_track_m",
        "progress", "sequence", "metric", "value", "contacts",
    )
    with path.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields, extrasaction="ignore"); writer.writeheader()
        for report in reports:
            for event in report["events"]:
                writer.writerow({"run": report["run"], **event})


def plot_comparison(out: Path, reports: list[dict[str, Any]], data_sets: list[dict[str, np.ndarray]]) -> None:
    fig, axes = plt.subplots(2, 2, figsize=(14, 9), sharex=False)
    event_axis = axes[1, 1]
    event_styles = {
        "stale_plan_fallback": ("x", "tab:red"),
        "semaphore_coalescing": ("|", "tab:orange"),
        "estimator_truth_excursion": ("^", "tab:purple"),
        "solver_residual_anomaly": ("s", "tab:brown"),
        "cross_track_recovery_demand": ("D", "tab:green"),
        "physical_divergence_onset": ("o", "tab:pink"),
        "contact": ("X", "black"),
    }
    for run_index, (report, data) in enumerate(zip(reports, data_sets)):
        if "time_s" not in data:
            continue
        contact = any(event["kind"] == "contact" for event in report["events"])
        label = Path(report["run"]).name + (" [contact]" if contact else " [no contact]")
        active_time = data["time_s"] - float(report["launch_time_s"])
        if all(key in data for key in ("x_m", "y_m")):
            axes[0, 0].plot(data["x_m"], data["y_m"], label=label)
        tilt = quaternion_tilt_deg(data)
        if tilt is not None:
            axes[0, 1].plot(active_time, tilt, label=label)
        rpm_keys = [f"rpm_{motor}" for motor in range(1, 5)]
        ref_keys = [f"rpm_ref_{motor}" for motor in range(1, 5)]
        if all(key in data for key in rpm_keys + ref_keys):
            rpm = np.column_stack([data[key] for key in rpm_keys])
            rpm_ref = np.column_stack([data[key] for key in ref_keys])
            axes[1, 0].plot(active_time, np.max(np.abs(rpm_ref - rpm), axis=1),
                            label=label)
        for event in report["events"]:
            style = event_styles.get(event["kind"])
            if style is None or event["time_s"] < report["launch_time_s"]:
                continue
            marker, color = style
            event_axis.scatter(event["time_s"] - report["launch_time_s"], run_index,
                               marker=marker, color=color, s=34, alpha=.8)
    axes[0, 0].set(xlabel="world x (m)", ylabel="world y (m)",
                   title="Measured trajectory")
    axes[0, 0].axis("equal")
    axes[0, 1].set(xlabel="active time (s)", ylabel="tilt (deg)",
                   title="Attitude departure")
    axes[0, 1].axhline(30.0, color="k", linestyle="--", alpha=.35)
    axes[1, 0].set(xlabel="active time (s)", ylabel="max |RPM ref - realized|",
                   title="Rotor tracking error")
    event_axis.set(xlabel="active time (s)", title="Aligned diagnostic events",
                   yticks=range(len(reports)),
                   yticklabels=[Path(report["run"]).name for report in reports])
    handles = [plt.Line2D([], [], linestyle="", marker=marker, color=color,
                          label=kind, markersize=7)
               for kind, (marker, color) in event_styles.items()]
    event_axis.legend(handles=handles, fontsize=7, loc="best")
    for axis in (axes[0, 0], axes[0, 1], axes[1, 0]):
        axis.legend(fontsize=6)
        axis.grid(alpha=.2)
    event_axis.grid(axis="x", alpha=.2)
    fig.suptitle("Direct-MPC stable/failure comparison")
    fig.tight_layout(); fig.savefig(out, dpi=150); plt.close(fig)


def json_safe(value: Any) -> Any:
    if isinstance(value, float) and not math.isfinite(value): return None
    if isinstance(value, dict): return {key: json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)): return [json_safe(item) for item in value]
    return value


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--run", action="append", required=True, type=Path)
    parser.add_argument("--out", required=True, type=Path)
    args = parser.parse_args(); args.out.mkdir(parents=True, exist_ok=True)
    results = [analyze_run(path) for path in args.run]
    reports = [item[0] for item in results]
    (args.out / "direct_mpc_failure_report.json").write_text(json.dumps(json_safe({"runs": reports}), indent=2, sort_keys=True, allow_nan=False) + "\n")
    write_event_csv(args.out / "direct_mpc_events.csv", reports)
    plot_comparison(args.out / "direct_mpc_comparison.png", reports, [item[2] for item in results])


if __name__ == "__main__":
    main()
