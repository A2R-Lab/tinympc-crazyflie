#!/usr/bin/env python3
"""Regressions for mixed GAP8/STM32 UART framing and recovery."""

import math
import unittest

from sim_vision_uart_link import (
    VisionParser, corrupted_stream, flow_packet, gate_packet,
    queue_holdoff_margin_ms,
)


class VisionUartLinkTest(unittest.TestCase):
    def test_clean_interleaved_stream(self):
        parser = VisionParser()
        parser.feed(b"".join(gate_packet(i) + flow_packet(i)
                             for i in range(1, 1001)))
        self.assertEqual(parser.stats.gate_ok, 1000)
        self.assertEqual(parser.stats.flow_ok, 1000)
        self.assertEqual(parser.stats.crc_error, 0)

    def test_duplicates_do_not_publish_new_samples(self):
        parser = VisionParser()
        parser.feed(gate_packet(7) * 2 + flow_packet(7) * 2)
        self.assertEqual(parser.stats.gate_ok, 1)
        self.assertEqual(parser.stats.flow_ok, 1)
        self.assertEqual(parser.stats.duplicate, 2)

    def test_crc_valid_nonfinite_payload_is_rejected(self):
        parser = VisionParser()
        parser.feed(gate_packet(1, (math.nan,) * 8))
        self.assertEqual(parser.stats.gate_ok, 0)
        self.assertEqual(parser.stats.invalid, 1)

    def test_crc_valid_implausible_gate_output_is_rejected(self):
        parser = VisionParser()
        parser.feed(gate_packet(1, (1.0e10,) * 8))
        self.assertEqual(parser.stats.gate_ok, 0)
        self.assertEqual(parser.stats.invalid, 1)

    def test_faulted_stream_always_recovers_to_sentinel(self):
        for seed in range(250):
            parser = VisionParser()
            faulted, sentinel = corrupted_stream(80, seed)
            parser.feed(faulted + sentinel)
            self.assertEqual(parser.gate_sequences[-1], 83, seed)
            self.assertEqual(parser.flow_sequences[-1], 83, seed)

    def test_queue_covers_two_complete_mixed_bursts(self):
        self.assertGreaterEqual(512, 2 * (44 + 168))
        self.assertGreater(queue_holdoff_margin_ms(), 40.0)


if __name__ == "__main__":
    unittest.main()
