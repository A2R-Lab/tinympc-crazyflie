#!/usr/bin/env python3
"""Regression tests for the offline sector obstacle estimator."""

import argparse
import math
import unittest

import sim_flow_obstacle_sectors as sim


def defaults(**changes):
    values = dict(out=None, frames=60, dt=1 / 30, sectors=9, fov_deg=70.0,
                  start_x=0.0, start_y=-0.15, vx=0.0, vy=0.15,
                  yaw=0.0, yaw_rate=0.0, box_x=1.10, box_y=0.0,
                  box_width=0.80, box_depth=0.20, confidence=0.8,
                  flow_noise=0.0, dropout=0.0, seed=7)
    values.update(changes)
    return argparse.Namespace(**values)


class SectorSimulatorTest(unittest.TestCase):
    def test_dead_ahead_box_converges(self):
        rows, _ = sim.simulate(defaults())
        valid = [r for r in rows if r["cyl_valid"]]
        self.assertGreater(len(valid), 50)
        self.assertAlmostEqual(float(valid[-1]["cyl_world_x"]), 1.0, delta=0.03)
        self.assertAlmostEqual(float(valid[-1]["cyl_world_y"]), 0.0, delta=0.06)

    def test_flow_and_velocity_sign_cancel_in_depth_ratio(self):
        positive, _ = sim.simulate(defaults(vy=0.15, start_y=-0.15))
        negative, _ = sim.simulate(defaults(vy=-0.15, start_y=0.15))
        p = [r for r in positive if r["cyl_valid"]][-1]
        n = [r for r in negative if r["cyl_valid"]][-1]
        self.assertAlmostEqual(float(p["cyl_world_x"]), float(n["cyl_world_x"]), delta=0.02)
        self.assertAlmostEqual(float(p["cyl_world_y"]), -float(n["cyl_world_y"]), delta=0.03)

    def test_pure_forward_center_splits_cluster_and_is_rejected(self):
        rows, _ = sim.simulate(defaults(vx=0.15, vy=0.0, start_y=0.0,
                                        frames=30, box_x=1.30))
        self.assertFalse(any(r["cyl_valid"] for r in rows))
        center_az = 0.0
        self.assertAlmostEqual(0.15 * math.sin(center_az), 0.0)

    def test_moderate_noise_and_dropout_remain_near_box(self):
        rows, _ = sim.simulate(defaults(flow_noise=0.006, dropout=0.05,
                                        frames=90, seed=11))
        valid = [r for r in rows if r["cyl_valid"]]
        self.assertGreater(len(valid), 50)
        self.assertAlmostEqual(float(valid[-1]["cyl_world_x"]), 1.0, delta=0.12)
        self.assertAlmostEqual(float(valid[-1]["cyl_world_y"]), 0.0, delta=0.12)

    def test_stationary_has_no_estimate(self):
        rows, _ = sim.simulate(defaults(vx=0.0, vy=0.0))
        self.assertFalse(any(r["cyl_valid"] for r in rows))

    def test_forward_looming_makes_center_sectors_observable(self):
        estimator = sim.FirmwareMirror()
        vx, distance = 1.0, 1.0
        sectors = []
        for i in range(9):
            az = math.radians(-32 + i * 8)
            q = math.tan(az)
            angular = vx * math.sin(az) / distance
            sectors.append(sim.Sector(q, angular * (1 + q*q), 0.2,
                                      flow_y=vx / distance))
        result = None
        for _ in range(3):
            result = estimator.update(sectors, vx, 0.0, 0.0, 0.0, 0.0, 0.0)
        self.assertTrue(result["cylinder"]["valid"])
        self.assertAlmostEqual(float(result["cylinder"]["world_x"]), 1.0, delta=0.08)

    def test_reused_uart_payload_does_not_accumulate_evidence(self):
        estimator = sim.FirmwareMirror()
        sectors = []
        for i in range(9):
            az = math.radians(-32 + i * 8)
            q = math.tan(az)
            sectors.append(sim.Sector(q, math.sin(az)*(1 + q*q), 0.2,
                                      flow_y=1.0))
        first = estimator.update(sectors, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                 new_sample=True)
        self.assertFalse(first["cylinder"]["valid"])
        for _ in range(20):
            repeated = estimator.update(sectors, 1.0, 0.0, 0.0,
                                        0.0, 0.0, 0.0, new_sample=False)
        self.assertEqual(estimator.obs_hits, 1)
        self.assertFalse(repeated["cylinder"]["valid"])


if __name__ == "__main__":
    unittest.main()
