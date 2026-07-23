#!/usr/bin/env python3
"""Camera concurrency and watchdog regressions."""

import unittest

from sim_gap8_camera_pipeline import CameraWatchdog, max_concurrent_inferences


class Gap8CameraPipelineTest(unittest.TestCase):
    def test_unserialized_overrun_can_launch_two_cluster_jobs(self):
        self.assertGreater(max_concurrent_inferences(90, 30, False), 1)

    def test_serialized_callbacks_never_overlap_cluster_jobs(self):
        for cnn_ms in range(1, 201):
            self.assertLessEqual(max_concurrent_inferences(cnn_ms, 100, True), 1)

    def test_watchdog_recovers_only_a_stalled_capture(self):
        watchdog = CameraWatchdog()
        self.assertFalse(watchdog.poll(300_000, "consume"))
        self.assertFalse(watchdog.poll(300_000, "crop"))
        self.assertFalse(watchdog.poll(600_000, "wait_capture", True))
        self.assertFalse(watchdog.poll(300_000, "wait_capture"))  # cooldown
        self.assertTrue(watchdog.poll(600_000, "wait_capture"))
        self.assertEqual(watchdog.recoveries, 1)
        self.assertFalse(watchdog.poll(700_000, "wait_capture"))
        self.assertTrue(watchdog.poll(1_200_000, "wait_capture"))


if __name__ == "__main__":
    unittest.main()
