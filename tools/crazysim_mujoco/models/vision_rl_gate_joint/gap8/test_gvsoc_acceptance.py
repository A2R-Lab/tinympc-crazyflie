import importlib.util
import unittest
from pathlib import Path

import numpy as np

MODULE = Path(__file__).with_name("check_gvsoc_acceptance.py")
SPEC = importlib.util.spec_from_file_location("gvsoc_acceptance", MODULE)
acceptance = importlib.util.module_from_spec(SPEC); SPEC.loader.exec_module(acceptance)


class GvsocSemanticAcceptanceTest(unittest.TestCase):
    def test_confidence_and_individual_corners_are_reported(self):
        reference = np.zeros((2, 12), dtype=np.float64)
        decoded = reference.copy()
        reference[:, 11] = decoded[:, 11] = 2.0
        visible = np.asarray([True, True])
        target = np.full((2, 4, 2), .5, dtype=np.float64)
        metrics = acceptance.semantic_metrics(decoded, reference, visible, target)
        self.assertEqual(metrics["confidence_classification_agreement"], 1.0)
        self.assertEqual(metrics["visible_gate_recall_degradation"], 0.0)
        self.assertEqual(metrics["visible_per_corner_error_increase_px"], 0.0)

    def test_confidence_recall_degradation_is_not_hidden_by_corner_centroid(self):
        reference = np.zeros((2, 12), dtype=np.float64)
        decoded = reference.copy()
        reference[:, 11] = 2.0
        decoded[:, 11] = -2.0
        # Opposing corner errors retain the same centroid, but per-corner error
        # and confidence recall must still expose the semantic degradation.
        decoded[:, 3] = 2.0
        decoded[:, 5] = -2.0
        visible = np.asarray([True, True])
        target = np.full((2, 4, 2), .5, dtype=np.float64)
        metrics = acceptance.semantic_metrics(decoded, reference, visible, target)
        self.assertEqual(metrics["confidence_classification_agreement"], 0.0)
        self.assertEqual(metrics["visible_gate_recall_degradation"], 1.0)
        self.assertGreater(metrics["visible_per_corner_error_increase_px"], 0.0)

    def test_runtime_admission_mirrors_bridge_then_controller(self):
        # Ordered TL/TR/BR/BL square accepted by both the joint bridge and
        # the controller's rotation-invariant quadrilateral predicate.
        square = np.asarray(((.30, .30), (.70, .30), (.70, .70), (.30, .70)))
        corner_logits = np.log(square / (1.0 - square)).reshape(-1)
        reference = np.zeros((3, 12), dtype=np.float64)
        decoded = reference.copy()
        reference[:, 3:11] = corner_logits
        decoded[:, 3:11] = corner_logits
        reference[:, 11] = 2.0  # All three are float-runtime admissions.
        decoded[:, 11] = np.asarray((2.0, -2.0, -2.0))
        visible = np.asarray((True, True, False))
        target = np.repeat(square[None, :, :], 3, axis=0)

        metrics = acceptance.semantic_metrics(decoded, reference, visible, target)
        admission = metrics["runtime_gate_admission"]
        self.assertEqual(admission["float_counts"], {
            "true_positive": 2, "false_positive": 1,
            "false_negative": 0, "true_negative": 0,
        })
        self.assertEqual(admission["gvsoc_counts"], {
            "true_positive": 1, "false_positive": 0,
            "false_negative": 1, "true_negative": 1,
        })
        self.assertAlmostEqual(admission["agreement"], 1.0 / 3.0)
        self.assertAlmostEqual(admission["visible_recall_degradation"], .5)

    def test_runtime_admission_rejects_quad_that_only_confidence_accepts(self):
        # A bow tie has high confidence but fails both bridge and controller
        # convexity; the admission result must not silently become confidence-only.
        bow_tie = np.asarray(((.30, .30), (.70, .70), (.70, .30), (.30, .70)))
        logits = np.log(bow_tie / (1.0 - bow_tie)).reshape(-1)
        values = np.zeros((1, 12), dtype=np.float64)
        values[0, 3:11] = logits
        values[0, 11] = 2.0
        metrics = acceptance.semantic_metrics(
            values, values, np.asarray((True,)), bow_tie[None, :, :])
        self.assertEqual(metrics["float_confidence_counts"]["true_positive"], 1)
        self.assertEqual(metrics["runtime_gate_admission"]["float_counts"]["true_positive"], 0)


if __name__ == "__main__": unittest.main()
