#!/usr/bin/env python3
"""Regression checks for the source-matched GAP8 deployment path."""

import argparse
import struct
import unittest
import zlib

import sim_gap8_deployment_suite as deployment
from sim_monocular_flow_suite import SCENES


class Gap8DeploymentTest(unittest.TestCase):
    def test_wire_packet_is_168_bytes_and_crc_protected(self):
        header = b"\x90\x19\x08\x35"
        payload = struct.pack("<II f BBH", 1, 0, 0.2, 9, 0, 0)
        payload += struct.pack("<" + "f" * 36, *([0.0] * 36))
        message = header + payload
        packet = message + struct.pack("<I", zlib.crc32(message) & 0xffffffff)
        self.assertEqual(len(packet), 168)
        self.assertEqual(struct.unpack("<I", packet[-4:])[0], zlib.crc32(packet[:-4]) & 0xffffffff)
        corrupted = bytearray(packet); corrupted[50] ^= 1
        self.assertNotEqual(struct.unpack("<I", corrupted[-4:])[0], zlib.crc32(corrupted[:-4]) & 0xffffffff)

    def test_balanced_frontend_detects_broad_front_box(self):
        args = argparse.Namespace(snapshots=15, pass_error=0.35,
                                  balanced_features=True, out=None)
        result = deployment.run_scene(SCENES[0], args)
        self.assertTrue(result["pass"])
        # The source-identical 27-feature/one-iteration frontend has a larger
        # bias than the former 48-feature/three-iteration Python approximation.
        self.assertLess(float(result["error_m"]), 0.20)

    def test_narrow_box_is_detected_but_remains_inaccurate(self):
        args = argparse.Namespace(snapshots=15, pass_error=0.35,
                                  balanced_features=True, out=None)
        result = deployment.run_scene(next(s for s in SCENES if s.name == "narrow"), args)
        self.assertGreater(result["valid_controller_ticks"], 0)
        self.assertGreater(float(result["error_m"]), args.pass_error)


if __name__ == "__main__":
    unittest.main()
