#!/usr/bin/env python3
"""Synthetic contract tests for direct-MPC event analysis."""
from __future__ import annotations
import csv
import json
import subprocess
import struct
import sys
import tempfile
import unittest
from pathlib import Path
import numpy as np
import analyze_direct_mpc_failures as analyzer


def binary_record(event: int, event_sequence: int, finish_s: float = 0.0,
                  action: tuple[float, ...] = (0, 0, 0, 0), fallback: int = 0,
                  dropped: int = 0, use_microseconds: bool = True,
                  estimator_override: list[float] | None = None,
                  start_latency_s: float = .005, solve_duration_s: float = .005,
                  coalesced: int = 0, mutex_miss: int = 0,
                  queue_high_water: int = 1,
                  release_tick_override: int | None = None,
                  start_tick_override: int | None = None,
                  plan_age_ticks_override: int | None = None,
                  numeric_flags: int = 0,
                  tick_us: int = 1000) -> bytes:
    ticks = int(round(finish_s * 1e6 / tick_us))
    start_s = max(0.0, finish_s - solve_duration_s); release_s = max(0.0, start_s - start_latency_s)
    start_tick = (int(round(start_s * 1e6 / tick_us)) if start_tick_override is None
                  else start_tick_override)
    release_tick = (int(round(release_s * 1e6 / tick_us)) if release_tick_override is None
                    else release_tick_override)
    plan_age_ticks = (ticks - release_tick if plan_age_ticks_override is None
                      else plan_age_ticks_override)
    integers = [event, event_sequence, event_sequence - 1, max(0, event_sequence - 1),
                release_tick, start_tick, ticks, release_tick, plan_age_ticks, event_sequence - 1,
                mutex_miss, coalesced, dropped, queue_high_water,
                int(solve_duration_s * 1e6), 20, 2, 5, 0, numeric_flags, fallback]
    estimator = estimator_override or [0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0]
    floats = [.01, .02, 1.0] + estimator + [0.0] * 12 + [0.5] * 4 + list(action) + list(action) + [0.0] * 4
    return analyzer.BINARY_STRUCT.pack(analyzer.BINARY_MAGIC, analyzer.BINARY_VERSION,
                                       analyzer.BINARY_RECORD_SIZE, *integers,
                                       int(release_s * 1e6) if use_microseconds else 0,
                                       int(start_s * 1e6) if use_microseconds else 0,
                                       int(finish_s * 1e6) if use_microseconds else 0, *floats)


def mmap_container(records: dict[int, bytes], *, next_sequence: int | None = None,
                   committed: set[int] | None = None, firmware_tick_us: int = 1000,
                   overflow_count: int = 0, header_overrides: dict[str, int] | None = None,
                   corrupt_checksum_sequence: int | None = None,
                   write_order: list[int] | None = None) -> bytes:
    """Build the controller's fixed mmap ABI without depending on write order."""
    committed = set(records) if committed is None else committed
    next_sequence = next_sequence if next_sequence is not None else max(records, default=0) + 1
    values = {
        "magic": analyzer.MMAP_MAGIC, "version": analyzer.MMAP_VERSION,
        "header_size": analyzer.MMAP_HEADER_SIZE,
        "slot_size": analyzer.MMAP_SLOT_SIZE,
        "record_size": analyzer.BINARY_RECORD_SIZE,
        "capacity": analyzer.MMAP_CAPACITY,
        "firmware_tick_us": firmware_tick_us,
        "payload_magic": analyzer.BINARY_MAGIC,
        "payload_version": analyzer.BINARY_VERSION,
        "reserved0": 0, "next_sequence": next_sequence,
        "overflow_count": overflow_count, "committed_count": len(committed),
    }
    values.update(header_overrides or {})
    container = bytearray(analyzer.MMAP_FILE_SIZE)
    analyzer.MMAP_HEADER_STRUCT.pack_into(container, 0, *values.values())
    for sequence in write_order or list(records):
        record = records[sequence]
        if len(record) != analyzer.BINARY_RECORD_SIZE:
            raise ValueError("mmap payload must be one v2 record")
        offset = analyzer.MMAP_HEADER_SIZE + (sequence - 1) * analyzer.MMAP_SLOT_SIZE
        struct.pack_into("<Q", container, offset + 8, sequence)
        container[offset + 16:offset + 312] = record
        checksum = analyzer.mmap_slot_checksum(container[offset + 8:offset + 312])
        if sequence == corrupt_checksum_sequence:
            checksum ^= 1
        struct.pack_into("<I", container, offset + 312, checksum)
        if sequence in committed:
            struct.pack_into("<Q", container, offset, sequence)
    return bytes(container)


def write_run(root: Path, name: str, diag: list[str], *, contact: bool = False,
              divergent: bool = True, realized_tracks_reference: bool = False,
              divergence_index: int = 25, firmware_tick_us: int = 1000) -> Path:
    run = root / name; run.mkdir()
    (run / "config.json").write_text(json.dumps({
        "trajectory": "circle", "realtime_factor": 1.0,
        "firmware_time_factor": .8, "launch_time_s": 1.0,
        "mpc_diagnostic": {"firmware_tick_us": firmware_tick_us},
    }))
    (run / "summary.json").write_text(json.dumps({"launch_time_s": 1.0, "crashed": contact}))
    (run / "firmware.log").write_text("\n".join(diag) + "\n")
    fields = ["time_s", "x_m", "y_m", "z_m", "vx_mps", "vy_mps", "vz_mps", "qw", "qx", "qy", "qz", "wx_radps", "wy_radps", "wz_radps", "rpm_1", "rpm_2", "rpm_3", "rpm_4", "rpm_ref_1", "rpm_ref_2", "rpm_ref_3", "rpm_ref_4", "contacts"]
    with (run / "state.csv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields); writer.writeheader()
        for i in range(40):
            t = .8 + .02 * i; sample_divergent = divergent and i >= divergence_index
            rpm_ref = 16000 if divergent and i >= 25 else (13000 if divergent and i >= 18 else 10000)
            realized = rpm_ref if realized_tracks_reference else 10000
            writer.writerow({"time_s": t, "x_m": 0, "y_m": 0, "z_m": 1, "vx_mps": 0, "vy_mps": 0, "vz_mps": 0, "qw": .96 if sample_divergent else 1, "qx": .28 if sample_divergent else 0, "qy": 0, "qz": 0, "wx_radps": 25 if sample_divergent else 0, "wy_radps": 0, "wz_radps": 0, "rpm_1": realized, "rpm_2": realized, "rpm_3": realized, "rpm_4": realized, "rpm_ref_1": rpm_ref, "rpm_ref_2": rpm_ref, "rpm_ref_3": rpm_ref, "rpm_ref_4": rpm_ref, "contacts": int(contact and i == 32)})
    return run


def explicit_motor_profile_config() -> dict:
    return {
        "model": "cf21B_500",
        "acceptance_contract": {"resolved_plant_profile": {
            "name": "synthetic_test_profile",
            "rpm_to_thrust": list(analyzer.CANONICAL_CF21B_500_RPM_TO_THRUST),
            "normalized_command_full_thrust_n": .2,
        }},
    }


def absolute_alignment_record(firmware_time_s: float,
                              action: tuple[float, float, float, float]) -> dict:
    timestamp_us = int(round(firmware_time_s * 1e6))
    return {
        "event": 1, "release_us": timestamp_us, "start_us": timestamp_us,
        "finish_us": timestamp_us, "release_tick": 0, "start_tick": 0,
        "finish_tick": 0, "first_action_clamped": list(action),
        "estimator": [0.0] * 13,
    }


def absolute_alignment_data(actions: list[tuple[float, float, float, float]],
                            transition_times: list[float]) -> dict[str, np.ndarray]:
    profile, error = analyzer.resolve_absolute_action_profile(explicit_motor_profile_config())
    if error or profile is None:
        raise AssertionError(error)
    references = np.vstack([
        np.zeros(4),
        *(analyzer.normalized_actions_to_rpm(np.asarray(action), profile)
          for action in actions),
    ])
    result = {"time_s": np.asarray([0.0, *transition_times], dtype=float)}
    result.update({f"rpm_ref_{motor + 1}": references[:, motor]
                   for motor in range(4)})
    return result


class DirectFailureAnalyzerTest(unittest.TestCase):
    def test_absolute_action_alignment_uses_unique_exact_patterns(self):
        actions = [
            (.20, .31, .42, .53), (.24, .36, .48, .60),
            (.29, .43, .57, .71), (.35, .50, .65, .80),
        ]
        records = [absolute_alignment_record(time, action)
                   for time, action in zip((1., 2., 3., 4.), actions)]
        data = absolute_alignment_data(actions, [1.3, 2.1, 2.9, 3.7])
        alignment = analyzer._align_binary_absolute_actions(
            records, data, explicit_motor_profile_config())
        self.assertTrue(alignment["observable"])
        self.assertTrue(alignment["causal_alignment_valid"])
        self.assertEqual(alignment["matched_transition_count"], 4)
        self.assertAlmostEqual(alignment["clock_slope"], .8, places=9)
        self.assertAlmostEqual(alignment["clock_offset_s"], .5, places=9)
        self.assertEqual(alignment["motor_mapping"]["provenance"],
                         "run_config.acceptance_contract.resolved_plant_profile")
        self.assertFalse(alignment["motor_mapping"]["inferred"])

    def test_absolute_action_alignment_excludes_repeated_ambiguous_patterns(self):
        first = (.20, .31, .42, .53)
        actions = [first, (.24, .36, .48, .60), first,
                   (.29, .43, .57, .71), (.35, .50, .65, .80)]
        solve_actions = [actions[index] for index in (0, 1, 3, 4)]
        records = [absolute_alignment_record(time, action)
                   for time, action in zip((1., 2., 3., 4.), solve_actions)]
        data = absolute_alignment_data(actions, [1.3, 2.1, 2.5, 2.9, 3.7])
        alignment = analyzer._align_binary_absolute_actions(
            records, data, explicit_motor_profile_config())
        self.assertTrue(alignment["observable"])
        self.assertEqual(alignment["ambiguous_pattern_count"], 1)
        self.assertEqual(alignment["matched_transition_count"], 3)

    def test_absolute_action_alignment_rejects_wrong_canonical_model_identity(self):
        config = {
            "model": "not_cf21B_500", "pwm_thrust_full_n": .2,
            "acceptance_contract": {
                "plant_model_sha256": "incorrect",
                "resolved_plant_profile": {
                    "name": analyzer.CANONICAL_CF21B_500_PROFILE_NAME,
                },
            },
        }
        profile, reason = analyzer.resolve_absolute_action_profile(config)
        self.assertIsNone(profile)
        self.assertIn("identity does not match", reason)

    def test_absolute_action_alignment_rejects_outlier_and_insufficient_matches(self):
        actions = [
            (.20, .31, .42, .53), (.24, .36, .48, .60),
            (.29, .43, .57, .71), (.35, .50, .65, .80),
        ]
        records = [absolute_alignment_record(time, action)
                   for time, action in zip((1., 2., 3., 4.), actions)]
        outlier = analyzer._align_binary_absolute_actions(
            records, absolute_alignment_data(actions, [1.5, 2.5, 3.5, 4.6]),
            explicit_motor_profile_config())
        self.assertTrue(outlier["observable"])
        self.assertFalse(outlier["causal_alignment_valid"])
        self.assertGreater(outlier["maximum_uncertainty_s"], .020)

        unmatched_action = (.91, .82, .73, .64)
        insufficient_records = [
            absolute_alignment_record(1., actions[0]),
            absolute_alignment_record(2., actions[1]),
            absolute_alignment_record(3., unmatched_action),
        ]
        insufficient_data = absolute_alignment_data(
            [actions[0], actions[1], actions[2]], [1.5, 2.5, 3.5])
        insufficient = analyzer._align_binary_absolute_actions(
            insufficient_records, insufficient_data, explicit_motor_profile_config())
        self.assertFalse(insufficient["observable"])
        self.assertEqual(insufficient["unique_candidate_count"], 2)
        self.assertIn("fewer than three unique", insufficient["reason"])

    def test_stable_has_no_failure_classification(self):
        with tempfile.TemporaryDirectory() as temp:
            run = write_run(Path(temp), "stable", ["TINYMPC-DIAG t_s=1.1 plan_seq=2 model=level residual=.01 clamp_mask=0"], divergent=False)
            report, _, _ = analyzer.analyze_run(run)
            self.assertFalse(any(report["classifications"].values()))
            self.assertTrue(report["timing"]["valid"])

    def test_stale_fallback_precedes_divergence(self):
        with tempfile.TemporaryDirectory() as temp:
            run = write_run(Path(temp), "stale", ["TINYMPC-DIAG t_s=1.12 fallback=stale plan_age_s=.287", "TINYMPC-DIAG t_s=1.14 fallback=stale plan_age_s=.290"], contact=True)
            report, events, _ = analyzer.analyze_run(run)
            self.assertTrue(report["classifications"]["stale_plan_fallback_before_divergence"])
            self.assertIn("contact", [event["kind"] for event in events])

    def test_fresh_rail_oscillation_precedes_divergence(self):
        with tempfile.TemporaryDirectory() as temp:
            run = write_run(Path(temp), "osc", ["TINYMPC-DIAG t_s=1.12 u1=0 u2=0 u3=1 u4=1 plan_age_s=.002", "TINYMPC-DIAG t_s=1.14 u1=1 u2=1 u3=0 u4=0 plan_age_s=.002"])
            report, _, _ = analyzer.analyze_run(run)
            self.assertTrue(report["classifications"]["fresh_plan_rail_oscillation_before_divergence"])

    def test_cli_writes_all_artifacts(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp); run = write_run(root, "run", []) ; out = root / "out"
            subprocess.run([sys.executable, str(Path(analyzer.__file__)), "--run", str(run), "--out", str(out)], check=True)
            self.assertTrue((out / "direct_mpc_failure_report.json").exists())
            self.assertTrue((out / "direct_mpc_events.csv").exists())
            self.assertGreater((out / "direct_mpc_comparison.png").stat().st_size, 1000)

    def test_binary_v2_is_authoritative_validated_and_aligned(self):
        with tempfile.TemporaryDirectory() as temp:
            run = write_run(Path(temp), "binary", ["TINYMPC-DIAG t_s=1.0 fallback=stale"])
            payload = b"".join([
                binary_record(0, 1), binary_record(1, 2, 1.10, (.2, .2, .2, .2)),
                binary_record(1, 3, 1.16, (.5, .5, .5, .5)),
                binary_record(1, 4, 1.30, (.8, .8, .8, .8)),
            ])
            (run / "mpc_diag.bin").write_bytes(payload)
            report, events, _ = analyzer.analyze_run(run)
            self.assertEqual(report["diagnostic_source"], "binary")
            self.assertTrue(report["binary_validation"]["valid"])
            self.assertEqual(report["binary_validation"]["sequence_gap_count"], 0)
            self.assertTrue(report["binary_validation"]["authoritative"])
            self.assertEqual(report["binary_validation"]["record_size"], 296)
            self.assertEqual(report["binary_validation"]["queue_high_water_count_max"], 1)
            self.assertTrue(report["binary_state_alignment"]["observable"])
            self.assertLessEqual(report["binary_state_alignment"]["maximum_uncertainty_s"], .001)
            self.assertIn("planner_finish", [event["kind"] for event in events])
            self.assertTrue(report["diagnostic_observability"]["cross_track"])

    def test_binary_validation_reports_sequence_and_drop_faults(self):
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp) / "mpc_diag.bin"
            path.write_bytes(binary_record(0, 1) + binary_record(
                1, 3, 1.1, dropped=2, queue_high_water=65))
            records, validation = analyzer.parse_binary_diag(path)
            self.assertEqual(len(records), 2)
            self.assertFalse(validation["valid"])
            self.assertEqual(validation["sequence_gap_count"], 1)
            self.assertEqual(validation["dropped_record_count_max"], 2)
            self.assertEqual(validation["queue_high_water_count_max"], 65)
            self.assertTrue(any("queue_high_water_count_exceeds_capacity" in error
                                for error in validation["errors"]))

    def test_scalar_trigger_bits_are_not_numeric_faults(self):
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp) / "mpc_diag.bin"
            scalar_bits = (1 | 2 | 4 | 8) << 16
            path.write_bytes(binary_record(0, 1) + binary_record(
                1, 2, 1.1, numeric_flags=scalar_bits))
            records, validation = analyzer.parse_binary_diag(path)
            self.assertTrue(validation["valid"])
            events = analyzer.diag_events(records)
            self.assertNotIn("optimizer_numerical_excursion",
                             {event["kind"] for event in events})
            trigger = next(event for event in events
                           if event["kind"] == "diagnostic_scalar_trigger")
            self.assertEqual(trigger["triggers"], [
                "residual_threshold", "body_rate_above_5_rad_s",
                "tilt_above_15_deg", "reference_demand_above_0_15_m",
            ])

            path.write_bytes(binary_record(0, 1) + binary_record(
                1, 2, 1.1, numeric_flags=1))
            low_records, _ = analyzer.parse_binary_diag(path)
            self.assertIn("optimizer_numerical_excursion",
                          {event["kind"] for event in analyzer.diag_events(low_records)})

    def test_tick_clock_fallback_and_full_estimator_excursion(self):
        with tempfile.TemporaryDirectory() as temp:
            run = write_run(Path(temp), "ticks", [], firmware_tick_us=1250)
            bad_estimator = [0, 0, 1, 2, 0, 0, 0, 0, .25, .968, 4, 0, 0]
            (run / "mpc_diag.bin").write_bytes(b"".join([
                binary_record(0, 1, use_microseconds=False, tick_us=1250),
                binary_record(1, 2, 1.10, (.2,) * 4, use_microseconds=False, tick_us=1250),
                binary_record(1, 3, 1.16, (.5,) * 4, use_microseconds=False, tick_us=1250),
                binary_record(1, 4, 1.30, (.8,) * 4, use_microseconds=False,
                              estimator_override=bad_estimator, tick_us=1250),
            ]))
            report, events, _ = analyzer.analyze_run(run)
            self.assertEqual(report["binary_state_alignment"]["firmware_clock_source"], "freertos_ticks")
            self.assertEqual(report["binary_state_alignment"]["firmware_tick_us"], 1250)
            self.assertAlmostEqual(report["binary_state_alignment"]["clock_slope"], 1.0,
                                   places=6)
            excursions = [event for event in events if event["kind"] == "estimator_truth_excursion"]
            self.assertTrue(excursions)
            self.assertGreater(excursions[-1]["velocity_error_mps"], .5)
            self.assertGreater(excursions[-1]["attitude_error_deg"], 15)
            self.assertGreater(excursions[-1]["body_rate_error_rad_s"], 2)

    def test_tick_only_alignment_requires_recorded_tick_period(self):
        with tempfile.TemporaryDirectory() as temp:
            run = write_run(Path(temp), "missing_tick_period", [])
            config = json.loads((run / "config.json").read_text())
            del config["mpc_diagnostic"]
            (run / "config.json").write_text(json.dumps(config))
            (run / "mpc_diag.bin").write_bytes(b"".join([
                binary_record(0, 1, use_microseconds=False),
                binary_record(1, 2, 1.10, (.2,) * 4, use_microseconds=False),
                binary_record(1, 3, 1.16, (.5,) * 4, use_microseconds=False),
                binary_record(1, 4, 1.30, (.8,) * 4, use_microseconds=False),
            ]))
            report, _, _ = analyzer.analyze_run(run)
            alignment = report["binary_state_alignment"]
            self.assertFalse(alignment["observable"])
            self.assertIn("tick period unavailable", alignment["reason"])
            self.assertFalse(report["binary_causal_eligibility"]["eligible"])

    def test_mixed_tick_domains_never_drive_latency_or_plan_age(self):
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp) / "mpc_diag.bin"
            path.write_bytes(b"".join([
                binary_record(0, 1),
                binary_record(1, 2, 2.020, (.5,) * 4,
                              start_latency_s=.012, solve_duration_s=.008,
                              release_tick_override=2500,
                              start_tick_override=2012,
                              plan_age_ticks_override=5),
            ]))
            records, validation = analyzer.parse_binary_diag(path)
            self.assertTrue(validation["valid"])
            solve = records[1]
            self.assertEqual(solve["release_tick"], 2500)
            self.assertEqual(solve["start_tick"], 2012)
            self.assertEqual(solve["plan_age_s"], .005)
            self.assertEqual(solve["release_to_start_us"], 12_000)
            alignment = {"observable": True, "firmware_clock_source": "microseconds",
                         "clock_slope": 1.0, "clock_offset_s": 0.0}
            events = analyzer.binary_lifecycle_events(records, alignment, True)
            latency = next(event for event in events
                           if event["kind"] == "planner_start")["time_s"] - next(
                               event for event in events
                               if event["kind"] == "planner_release")["time_s"]
            self.assertAlmostEqual(latency, .012, places=9)
            self.assertLess(latency, 1.0)

            path.write_bytes(b"".join([
                binary_record(0, 1, use_microseconds=False),
                binary_record(1, 2, 2.020, (.5,) * 4,
                              use_microseconds=False,
                              release_tick_override=2500,
                              start_tick_override=2012,
                              plan_age_ticks_override=5),
            ]))
            tick_only_records, tick_only_validation = analyzer.parse_binary_diag(path)
            self.assertTrue(tick_only_validation["valid"])
            self.assertEqual(tick_only_validation["tick_layout"], "legacy_mixed")
            self.assertEqual(tick_only_records[1]["clock_domains"]["release_tick"],
                             "legacy_stabilizer_tick")
            tick_alignment = {"observable": True,
                              "firmware_clock_source": "freertos_ticks",
                              "firmware_tick_us": 1000,
                              "clock_slope": 1.0, "clock_offset_s": 0.0}
            self.assertEqual(
                analyzer.binary_lifecycle_events(
                    tick_only_records, tick_alignment, True), [])

    def test_corrected_freertos_release_tick_enables_tick_lifecycle(self):
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp) / "mpc_diag.bin"
            path.write_bytes(b"".join([
                binary_record(0, 1, use_microseconds=False),
                binary_record(1, 2, 2.020, (.5,) * 4,
                              use_microseconds=False,
                              release_tick_override=2010,
                              start_tick_override=2012,
                              plan_age_ticks_override=5),
            ]))
            records, validation = analyzer.parse_binary_diag(path)
            self.assertTrue(validation["valid"])
            self.assertEqual(validation["tick_layout"], "corrected_freertos")
            self.assertEqual(records[1]["clock_domains"]["release_tick"],
                             "freertos_tick")
            self.assertEqual(records[1]["clock_domains"]["plan_tick"],
                             "stabilizer_tick")
            alignment = {"observable": True,
                         "firmware_clock_source": "freertos_ticks",
                         "firmware_tick_us": 1000,
                         "clock_slope": 1.0, "clock_offset_s": 0.0}
            events = analyzer.binary_lifecycle_events(records, alignment, True)
            release = next(event for event in events
                           if event["kind"] == "planner_release")
            start = next(event for event in events
                         if event["kind"] == "planner_start")
            self.assertAlmostEqual(start["time_s"] - release["time_s"], .002,
                                   places=9)
            self.assertNotIn("scheduler_start_latency",
                             {event["kind"] for event in events})

    def test_uint32_underflow_plan_age_is_rejected(self):
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp) / "mpc_diag.bin"
            underflow = (2012 - 2500) & analyzer.UINT32_MAX
            path.write_bytes(binary_record(
                1, 1, 2.020, plan_age_ticks_override=underflow))
            records, validation = analyzer.parse_binary_diag(path)
            self.assertIsNone(records[0]["plan_age_s"])
            self.assertFalse(validation["valid"])
            self.assertTrue(any("plan_age_ticks_underflow_suspected" in error
                                for error in validation["errors"]))

    def test_alignment_rejects_timestamp_only_matches_with_wrong_motor_pattern(self):
        with tempfile.TemporaryDirectory() as temp:
            run = write_run(Path(temp), "wrong_pattern", [])
            (run / "mpc_diag.bin").write_bytes(b"".join([
                binary_record(0, 1), binary_record(1, 2, 1.10, (0, 0, 0, 0)),
                binary_record(1, 3, 1.16, (1, -1, 1, -1)),
                binary_record(1, 4, 1.30, (-1, 1, -1, 1)),
            ]))
            report, _, _ = analyzer.analyze_run(run)
            self.assertFalse(report["binary_state_alignment"]["observable"])

    def test_launch_tolerance_and_starvation_subtypes(self):
        with tempfile.TemporaryDirectory() as temp:
            run = write_run(Path(temp), "subtypes", [])
            (run / "summary.json").write_text(json.dumps({"launch_time_s": 1.006}))
            (run / "mpc_diag.bin").write_bytes(b"".join([
                binary_record(0, 1), binary_record(1, 2, 1.10, (.2,) * 4),
                binary_record(1, 3, 1.16, (.5,) * 4, start_latency_s=.03, solve_duration_s=.03, coalesced=1),
                binary_record(1, 4, 1.30, (.8,) * 4, mutex_miss=1),
            ]))
            report, events, _ = analyzer.analyze_run(run)
            kinds = {event["kind"] for event in events}
            self.assertTrue(report["timing"]["launch_time_matches_request"])
            self.assertTrue({"scheduler_start_latency", "long_worker_solve", "semaphore_coalescing", "mutex_release_miss"} <= kinds)
            for key in ("scheduler_start_latency_before_divergence", "long_worker_solve_before_divergence",
                        "semaphore_coalescing_before_divergence", "mutex_release_miss_before_divergence"):
                self.assertTrue(report["classifications"][key], key)

    def test_fallback_record_preserves_pre_divergence_counter_deltas(self):
        records = [
            {"event": 0, "time_s": 0.0, "semaphore_coalesced_count": 0,
             "release_mutex_miss_count": 0},
            {"event": 2, "time_s": 1.2, "semaphore_coalesced_count": 2,
             "release_mutex_miss_count": 1},
        ]
        alignment = {"observable": True, "clock_slope": 1.0,
                     "clock_offset_s": 0.0,
                     "firmware_clock_source": "microseconds"}
        events = analyzer.binary_lifecycle_events(records, alignment, True)
        self.assertEqual(
            [(event["kind"], event["count_delta"]) for event in events],
            [("semaphore_coalescing", 2), ("mutex_release_miss", 1)])
        self.assertTrue(all(event["time_s"] == 1.2 for event in events))

    def test_prelaunch_diagnostic_excursion_is_not_a_precursor(self):
        events = [
            {"time_s": -0.01, "kind": "estimator_truth_excursion",
             "causal_eligible": True},
            {"time_s": 1.2, "kind": "stale_plan_fallback",
             "causal_eligible": True},
            {"time_s": 1.3, "kind": "physical_divergence_onset"},
        ]
        divergence = events[-1]
        self.assertEqual(
            analyzer.causally_preceding_kinds(events, 1.0, divergence),
            ["stale_plan_fallback"])

    def test_invalid_or_unalignable_binary_never_drives_causal_classification(self):
        def valid_payload(*, gap: bool = False, dropped: bool = False) -> bytes:
            sequences = (1, 2, 4, 5) if gap else (1, 2, 3, 4)
            return b"".join([
                binary_record(0, sequences[0]),
                binary_record(1, sequences[1], 1.10, (.1,) * 4),
                binary_record(1, sequences[2], 1.16, (.8,) * 4, fallback=3, start_latency_s=.03,
                              solve_duration_s=.03, coalesced=1, dropped=int(dropped)),
                binary_record(1, sequences[3], 1.30, (1.0,) * 4, mutex_miss=1, dropped=int(dropped)),
            ])
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            fixtures = {
                "bad_magic": bytearray(valid_payload()),
                "sequence_gap": valid_payload(gap=True),
                "reported_drop": valid_payload(dropped=True),
                "valid_unalignable": b"".join([
                    binary_record(0, 1), binary_record(1, 2, 1.10, (0, 0, 0, 0)),
                    binary_record(1, 3, 1.16, (1, -1, 1, -1), fallback=3, start_latency_s=.03, coalesced=1),
                    binary_record(1, 4, 1.30, (-1, 1, -1, 1), mutex_miss=1),
                ]),
            }
            fixtures["bad_magic"][0] ^= 0xFF
            for name, payload in fixtures.items():
                run = write_run(root, name, [], realized_tracks_reference=True)
                (run / "mpc_diag.bin").write_bytes(bytes(payload))
                report, events, _ = analyzer.analyze_run(run)
                self.assertFalse(report["binary_causal_eligibility"]["eligible"], name)
                self.assertTrue(report["binary_causal_eligibility"]["inconclusive_reason"], name)
                self.assertFalse(any(report["classifications"].values()), name)
                self.assertFalse(any(event.get("causal_eligible") for event in events
                                     if event["kind"].startswith("planner_") or event["kind"] in {
                                         "scheduler_start_latency", "long_worker_solve",
                                         "semaphore_coalescing", "mutex_release_miss"}), name)

    def test_valid_aligned_but_terminally_truncated_stream_is_noncausal(self):
        with tempfile.TemporaryDirectory() as temp:
            run = write_run(Path(temp), "truncated", [], realized_tracks_reference=True,
                            divergence_index=35)
            (run / "mpc_diag.bin").write_bytes(b"".join([
                binary_record(0, 1), binary_record(1, 2, 1.10, (.1,) * 4),
                binary_record(1, 3, 1.16, (.8,) * 4, fallback=3,
                              start_latency_s=.03, solve_duration_s=.03, coalesced=1),
                binary_record(1, 4, 1.30, (1.0,) * 4, mutex_miss=1),
            ]))
            report, events, _ = analyzer.analyze_run(run)
            self.assertTrue(report["binary_validation"]["valid"])
            self.assertTrue(report["binary_state_alignment"]["causal_alignment_valid"])
            coverage = report["binary_terminal_coverage"]
            self.assertEqual(coverage["target_kind"], "physical_divergence_onset")
            self.assertAlmostEqual(coverage["last_aligned_diagnostic_time_s"], 1.30, places=6)
            self.assertGreater(coverage["coverage_gap_s"], .020)
            self.assertFalse(coverage["valid"])
            self.assertFalse(report["binary_causal_eligibility"]["eligible"])
            self.assertIn("terminal diagnostic coverage", report["binary_causal_eligibility"]["inconclusive_reason"])
            self.assertFalse(any(report["classifications"].values()))
            self.assertTrue(any(event.get("causal_eligible") is False for event in events
                                if event["kind"] == "stale_plan_fallback"))

    def test_taskless_mmap_golden_stream_is_preferred_and_authoritative(self):
        with tempfile.TemporaryDirectory() as temp:
            run = write_run(Path(temp), "mmap_golden", [])
            records = {
                1: binary_record(0, 1),
                2: binary_record(1, 2, 1.10, (.2,) * 4),
                3: binary_record(1, 3, 1.16, (.5,) * 4),
                4: binary_record(1, 4, 1.30, (.8,) * 4),
            }
            payload = b"".join(records.values())
            (run / "mpc_diag.mmap").write_bytes(mmap_container(records))
            (run / "mpc_diag.bin").write_bytes(payload)
            report, _, _ = analyzer.analyze_run(run)
            validation = report["binary_validation"]
            self.assertEqual(report["diagnostic_source"], "taskless_mmap")
            self.assertTrue(validation["valid"])
            self.assertEqual(validation["container"], "taskless_mmap")
            self.assertEqual(validation["taskless_metadata"]["next_sequence"], 5)
            self.assertEqual(validation["committed_slot_count"], 4)
            self.assertTrue(validation["derived_compact_cross_check"]["matches"])
            self.assertTrue(report["binary_causal_eligibility"]["eligible"])

    def test_taskless_mmap_concurrent_out_of_order_commits_sort_by_sequence(self):
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp) / "mpc_diag.mmap"
            payloads = {
                1: binary_record(0, 1),
                2: binary_record(1, 2, 1.1, (.2,) * 4),
                3: binary_record(1, 3, 1.2, (.3,) * 4),
            }
            path.write_bytes(mmap_container(payloads, write_order=[3, 1, 2]))
            records, validation = analyzer.parse_mmap_diag(path, expected_firmware_tick_us=1000)
            self.assertTrue(validation["valid"])
            self.assertEqual([record["event_sequence"] for record in records], [1, 2, 3])

    def test_taskless_mmap_uncommitted_killed_slot_is_noncausal(self):
        with tempfile.TemporaryDirectory() as temp:
            run = write_run(Path(temp), "mmap_killed", [])
            payloads = {
                1: binary_record(0, 1),
                2: binary_record(1, 2, 1.1, (.2,) * 4),
            }
            (run / "mpc_diag.mmap").write_bytes(
                mmap_container(payloads, committed={1}, next_sequence=3))
            report, _, _ = analyzer.analyze_run(run)
            validation = report["binary_validation"]
            self.assertFalse(validation["valid"])
            self.assertEqual(validation["uncommitted_reserved_count"], 1)
            self.assertTrue(any("uncommitted_reserved" in error
                                for error in validation["errors"]))
            self.assertFalse(report["binary_causal_eligibility"]["eligible"])

    def test_taskless_mmap_checksum_corruption_is_invalid(self):
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp) / "mpc_diag.mmap"
            payloads = {1: binary_record(0, 1), 2: binary_record(1, 2, 1.1)}
            path.write_bytes(mmap_container(
                payloads, corrupt_checksum_sequence=2))
            _, validation = analyzer.parse_mmap_diag(path)
            self.assertFalse(validation["valid"])
            self.assertEqual(validation["checksum_failure_count"], 1)
            self.assertTrue(any("checksum=" in error for error in validation["errors"]))

    def test_taskless_mmap_hole_is_invalid(self):
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp) / "mpc_diag.mmap"
            payloads = {1: binary_record(0, 1), 3: binary_record(1, 3, 1.1)}
            path.write_bytes(mmap_container(payloads, next_sequence=4))
            _, validation = analyzer.parse_mmap_diag(path)
            self.assertFalse(validation["valid"])
            self.assertTrue(any("mmap_sequence_holes=2" in error
                                for error in validation["errors"]))

    def test_taskless_mmap_overflow_and_header_corruption_are_invalid(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            payloads = {1: binary_record(0, 1)}
            fixtures = {
                "overflow": mmap_container(payloads, overflow_count=1),
                "magic": mmap_container(
                    payloads, header_overrides={"magic": analyzer.MMAP_MAGIC ^ 1}),
                "timing": mmap_container(payloads, firmware_tick_us=0),
            }
            for name, payload in fixtures.items():
                path = root / f"{name}.mmap"
                path.write_bytes(payload)
                _, validation = analyzer.parse_mmap_diag(path)
                self.assertFalse(validation["valid"], name)
                self.assertTrue(validation["errors"], name)


if __name__ == "__main__": unittest.main()
