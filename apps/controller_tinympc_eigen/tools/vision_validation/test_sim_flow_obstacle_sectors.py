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

    def test_stationary_estimator_drift_and_weak_flow_are_rejected(self):
        estimator = sim.FirmwareMirror()
        sectors = []
        for i in range(9):
            az = math.radians(-32 + i * 8)
            q = math.tan(az)
            # Mirrors the hardware failure: low-confidence apparent flow plus
            # about 0.12 m/s of estimator drift while the vehicle is at rest.
            flow = 0.22 if 3 <= i <= 5 else 0.0
            confidence = 0.08 if 3 <= i <= 5 else 0.0
            sectors.append(sim.Sector(q, flow, confidence))
        results = [estimator.update(sectors, -0.06, -0.10, 0.0,
                                    0.0, 0.0, 0.0)
                   for _ in range(6)]
        self.assertTrue(all(not result["cluster"]["fresh"] for result in results))
        self.assertTrue(all(
            result["cluster"]["reject"] == "low_displacement"
            for result in results))
        self.assertFalse(any(result["cylinder"]["valid"] for result in results))

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
        for _ in range(5):
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

    @staticmethod
    def obstacle_sectors(distance=1.0, flow_y=0.0):
        sectors = []
        for i in range(9):
            az = math.radians(-32 + i * 8)
            q = math.tan(az)
            angular = -math.cos(az) / distance
            sectors.append(sim.Sector(q, angular * (1 + q*q), 0.4,
                                      flow_y=flow_y))
        return sectors

    def test_two_of_three_survives_one_missing_candidate(self):
        estimator = sim.FirmwareMirror()
        good = self.obstacle_sectors()
        empty = [sim.Sector(0.0, 0.0, 0.0) for _ in range(9)]
        estimator.update(good, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0)
        estimator.update(empty, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0)
        result = estimator.update(good, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0)
        self.assertTrue(result["cluster"]["valid"])

    def test_single_edge_sector_requires_temporal_persistence(self):
        estimator = sim.FirmwareMirror()
        sectors = [sim.Sector(0.0, 0.0, 0.0) for _ in range(9)]
        edge = 0
        az = math.radians(-32 + edge * 8)
        q = math.tan(az)
        distance = 0.5
        angular = -math.cos(az) / distance
        sectors[edge] = sim.Sector(q, angular * (1 + q*q), 0.4)

        first = estimator.update(sectors, 0.0, 1.0, 0.0,
                                 0.0, 0.0, 0.0)
        second = estimator.update(sectors, 0.0, 1.0, 0.0,
                                  0.0, 0.0, 0.0)
        self.assertTrue(first["cluster"]["fresh"])
        self.assertFalse(first["cluster"]["valid"])
        self.assertTrue(second["cluster"]["valid"])
        self.assertTrue(second["cylinder"]["valid"])
        self.assertAlmostEqual(float(second["cluster"]["range"]), distance,
                               delta=0.02)

    def test_persistent_edge_foreground_beats_far_wall(self):
        estimator = sim.FirmwareMirror()
        sectors = [sim.Sector(0.0, 0.0, 0.0) for _ in range(9)]
        for index, distance in ((0, 0.5), (4, 2.7), (5, 2.7)):
            az = math.radians(-32 + index * 8)
            q = math.tan(az)
            angular = -math.cos(az) / distance
            sectors[index] = sim.Sector(q, angular * (1 + q*q), 0.4)

        first = estimator.update(sectors, 0.0, 1.0, 0.0,
                                 0.0, 0.0, 0.0)
        second = estimator.update(sectors, 0.0, 1.0, 0.0,
                                  0.0, 0.0, 0.0)
        self.assertFalse(first["cluster"]["valid"])
        self.assertTrue(second["cluster"]["valid"])
        self.assertLess(float(second["cluster"]["range"]), 0.75)

    def test_nearest_validated_map_component_beats_stronger_wall(self):
        estimator = sim.FirmwareMirror()
        for _ in range(5):
            estimator._vote(2.7, 0.0, 1.0)
        estimator._vote(0.5, 0.0, 1.0)
        estimator._extract(0.0, 0.0, 0.0)

        self.assertTrue(estimator.cylinder["valid"])
        self.assertAlmostEqual(float(estimator.cylinder["body_x"]), 0.5,
                               delta=0.02)
        self.assertAlmostEqual(float(estimator.cylinder["body_y"]), 0.0,
                               delta=0.02)
        self.assertLess(float(estimator.cylinder["confidence"]), 0.3)

    def test_track_association_rejects_small_component_switch(self):
        estimator = sim.FirmwareMirror()
        estimator._vote(0.8, 0.0, 1.0)
        estimator._extract(0.0, 0.0, 0.0)
        estimator._vote(0.45, 0.0, 1.0)
        estimator._extract(0.0, 0.0, 0.0)

        self.assertAlmostEqual(float(estimator.cylinder["body_x"]), 0.8,
                               delta=0.02)

    def test_far_component_requires_four_persistent_votes(self):
        estimator = sim.FirmwareMirror()
        for vote in range(1, 5):
            estimator._vote(2.0, 0.0, 1.0)
            estimator._extract(0.0, 0.0, 0.0)
            self.assertEqual(bool(estimator.cylinder["valid"]), vote >= 4)

    def test_one_meter_boundary_requires_four_persistent_votes(self):
        estimator = sim.FirmwareMirror()
        for vote in range(1, 5):
            estimator._vote(0.9, 0.0, 1.0)
            estimator._extract(0.0, 0.0, 0.0)
            self.assertEqual(bool(estimator.cylinder["valid"]), vote >= 4)

    def test_spatially_inconsistent_candidates_do_not_persist(self):
        estimator = sim.FirmwareMirror()
        estimator.update(self.obstacle_sectors(0.7), 0.0, 1.0, 0.0,
                         0.0, 0.0, 0.0)
        result = estimator.update(self.obstacle_sectors(2.0), 0.0, 1.0, 0.0,
                                  0.0, 0.0, 0.0)
        self.assertFalse(result["cluster"]["valid"])

    def test_parallax_looming_disagreement_is_rejected(self):
        estimator = sim.FirmwareMirror()
        sectors = self.obstacle_sectors(1.0, flow_y=4.0)
        result = estimator.update(sectors, 1.0, 1.0, 0.0,
                                  0.0, 0.0, 0.0)
        self.assertTrue(any(d.get("reject") == "depth_disagreement"
                            for d in result["sectors"]))

    def test_noncontiguous_foreground_group_beats_background(self):
        estimator = sim.FirmwareMirror()
        sectors = self.obstacle_sectors(2.0)
        for i in (1, 4, 7):
            az = math.radians(-32 + i * 8)
            q = math.tan(az)
            angular = -math.cos(az)
            sectors[i] = sim.Sector(q, angular * (1 + q*q), 0.9)
        result = estimator.update(sectors, 0.0, 1.0, 0.0,
                                  0.0, 0.0, 0.0)
        self.assertTrue(result["cluster"]["fresh"])
        self.assertLess(float(result["cluster"]["range"]), 1.4)

    def test_pure_yaw_and_low_texture_are_negative(self):
        estimator = sim.FirmwareMirror()
        yaw = [sim.Sector(math.tan(math.radians(-32+i*8)), 0.5, 0.5)
               for i in range(9)]
        self.assertFalse(estimator.update(yaw, 0.0, 0.0, 0.5,
                                         0.0, 0.0, 0.0)["cluster"]["fresh"])
        low = [sim.Sector(0.0, 0.2, 0.005) for _ in range(9)]
        self.assertFalse(estimator.update(low, 0.2, 0.0, 0.0,
                                         0.0, 0.0, 0.0)["cluster"]["fresh"])


if __name__ == "__main__":
    unittest.main()
