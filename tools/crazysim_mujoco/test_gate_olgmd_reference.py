import math
from pathlib import Path
import struct
import sys
import unittest
import zlib

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from gate_olgmd_reference import (  # noqa: E402
    GATE_CORNERS_BODY,
    GATE_CORNERS_HEADER,
    THREAT_BODY,
    THREAT_HEADER,
    Olgmd1Reference,
    downsample_box_2x2,
    gate_corners_packet,
    resize_vertical_area_160_to_96,
    threat_packet,
)


class GateOlgmdReferenceTest(unittest.TestCase):
    def test_box_downsample_rounds_and_checks_abi(self):
        frame = np.arange(160 * 160, dtype=np.uint8).reshape(160, 160)
        output = downsample_box_2x2(frame)
        self.assertEqual(output.shape, (80, 80))
        expected = (int(frame[0, 0]) + int(frame[0, 1]) +
                    int(frame[1, 0]) + int(frame[1, 1]) + 2) // 4
        self.assertEqual(int(output[0, 0]), expected)
        with self.assertRaises(ValueError):
            downsample_box_2x2(np.zeros((80, 80), dtype=np.uint8))

    def test_vertical_area_resize_matches_periodic_weights(self):
        frame = np.repeat(np.arange(160, dtype=np.uint8)[:, None], 160, axis=1)
        output = resize_vertical_area_160_to_96(frame)
        self.assertEqual(output.shape, (96, 160))
        self.assertEqual(tuple(int(output[row, 0]) for row in range(3)), (0, 2, 4))

    def test_first_frame_is_non_authoritative_warmup(self):
        model = Olgmd1Reference()
        result = model.step(np.full((160, 160), 128, dtype=np.uint8))
        self.assertTrue(result.warmup)
        self.assertFalse(result.imminent_threat)

    def test_static_and_global_exposure_do_not_immediately_collide(self):
        model = Olgmd1Reference()
        base = np.full((160, 160), 80, dtype=np.uint8)
        self.assertTrue(model.step(base).warmup)
        for _ in range(6):
            self.assertFalse(model.step(base).imminent_threat)
        # Global illumination changes exercise FFI; a single step cannot
        # satisfy the rolling spike-window declaration.
        self.assertFalse(model.step(np.full_like(base, 160)).imminent_threat)

    def test_paper_integer_spikes_accumulate_over_window(self):
        model = Olgmd1Reference(
            spike_threshold=0.50, collision_window=6, collision_spikes=6)
        frame = np.full((160, 160), 128, dtype=np.uint8)
        model.step(frame)
        # The model clamps its output to 0.5, so the paper equation contributes
        # exactly one spike per static frame at this isolated test threshold.
        results = [model.step(frame) for _ in range(6)]
        self.assertTrue(all(result.spike_count >= 1 for result in results))
        self.assertFalse(results[-2].imminent_threat)
        self.assertTrue(results[-1].imminent_threat)
        self.assertGreaterEqual(results[-1].accumulated_spikes, 6)

    def test_both_polarities_execute(self):
        for background, foreground in ((0, 255), (255, 0)):
            model = Olgmd1Reference()
            frame = np.full((160, 160), background, dtype=np.uint8)
            model.step(frame)
            last = None
            for half_size in (5, 10, 20, 30, 40, 60):
                looming = frame.copy()
                looming[80-half_size:80+half_size,
                        80-half_size:80+half_size] = foreground
                last = model.step(looming)
                frame = looming
            self.assertIsNotNone(last)
            self.assertTrue(math.isfinite(last.membrane_potential))
            self.assertGreater(last.ffi_on + last.ffi_off, 0.0)

    def test_dual_packets_have_exact_sizes_crc_and_fields(self):
        threat = threat_packet(7, 123, True)
        self.assertEqual(len(threat), 16)
        self.assertEqual(zlib.crc32(threat[:-4]) & 0xFFFFFFFF,
                         struct.unpack("<I", threat[-4:])[0])
        self.assertEqual(THREAT_BODY.unpack(threat[:-4]),
                         (THREAT_HEADER, 123, 7, 1, 0))

        corners = np.asarray((-.25, .1, 1.25, .1, 1.25, .9, -.25, .9), np.float32)
        gate = gate_corners_packet(7, 123, True, corners)
        self.assertEqual(len(gate), 48)
        self.assertEqual(zlib.crc32(gate[:-4]) & 0xFFFFFFFF,
                         struct.unpack("<I", gate[-4:])[0])
        unpacked = GATE_CORNERS_BODY.unpack(gate[:-4])
        self.assertEqual(unpacked[:5], (GATE_CORNERS_HEADER, 123, 7, 1, 0))
        np.testing.assert_allclose(unpacked[5:], corners)


if __name__ == "__main__":
    unittest.main()
