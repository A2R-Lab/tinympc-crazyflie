#!/usr/bin/env python3

import unittest

import create_obstacle_corpus as corpus


class ObstacleCorpusDesignTest(unittest.TestCase):
    def test_positive_pose_matrix_is_complete(self):
        cases = corpus.positive_cases(27)
        expected = {
            (distance, orientation, offset)
            for distance in corpus.DISTANCES
            for orientation in corpus.ORIENTATIONS
            for offset in corpus.OFFSETS
        }
        actual = {
            (case["distance_m"], case["orientation_deg"],
             case["lateral_offset_m"])
            for case in cases
        }
        self.assertEqual(actual, expected)
        self.assertEqual(len(cases), 90)

    def test_negative_motion_texture_lighting_matrix_is_complete(self):
        cases = corpus.negative_cases(27)
        motions = {"stationary", "translation", "translation_yaw", "pure_yaw"}
        expected = {
            (motion, texture, lighting)
            for motion in motions
            for texture in corpus.TEXTURES
            for lighting in corpus.LIGHTING
        }
        actual = {
            (case["motion"], case["texture"], case["lighting"])
            for case in cases
        }
        self.assertEqual(actual, expected)
        self.assertEqual(len(cases), 48)


if __name__ == "__main__":
    unittest.main()
