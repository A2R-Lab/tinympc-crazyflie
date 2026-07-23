#!/usr/bin/env python3
"""Resource/synchronization audit regressions for the GAP8 deployment source."""

import os
import unittest
from pathlib import Path

import audit_gap8_firmware as audit


ROOT = Path(os.environ.get("NANOCOCKPIT_ROOT", "/tmp/tinympc-nanocockpit"))
APP = Path(__file__).resolve().parents[1]
WORKSPACE = APP.parents[1]


@unittest.skipUnless((ROOT / "src/gap/examples/pulp-frontnet/main.c").exists(),
                     "NanoCockpit checkout unavailable")
class Gap8FirmwareAuditTest(unittest.TestCase):
    def test_combined_alternating_firmware_fits_resources(self):
        result = audit.audit(ROOT, cnn_ms=50.0, flow_ms=50.0)
        self.assertTrue(result["pass"], result["checks"])
        self.assertEqual(result["network"]["directional_peak_bytes"], 153600)
        self.assertEqual(result["l2"]["estimated_runtime_peak_bytes"], 355532)
        self.assertEqual(result["l2"]["unallocated_headroom_bytes"], 168756)
        self.assertLess(result["uart"]["utilization"], 0.30)

    def test_stm32_link_hardening_is_in_deployment_sources(self):
        link = (APP / "src/gate8_link.c").read_text()
        uart = (WORKSPACE / "crazyflie-firmware/src/drivers/src/uart1.c").read_text()
        config = (APP / "app-config").read_text()
        self.assertIn("#define QUEUE_LENGTH 512", uart)
        self.assertIn("queueDrops++", uart)
        self.assertIn("resetAiDeck();", link)
        self.assertIn("uxTaskGetStackHighWaterMark", link)
        self.assertIn("g_invalidRx++", link)
        self.assertIn("CONFIG_DECK_AI=n", config)


if __name__ == "__main__":
    unittest.main()
