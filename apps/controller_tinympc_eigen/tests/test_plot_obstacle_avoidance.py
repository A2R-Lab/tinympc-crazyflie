import importlib.util
import tempfile
import unittest
from pathlib import Path

SCRIPT = Path(__file__).resolve().parents[1] / 'tools' / 'plot_obstacle_avoidance.py'
SPEC = importlib.util.spec_from_file_location('plot_obstacle_avoidance', SCRIPT)
plot = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(plot)


class PlotTests(unittest.TestCase):
    def test_excluded_geometry(self):
        polygon = plot.clip_polygon([(-1, -1), (1, -1), (1, 1), (-1, 1)], 1, 0, 0.25)
        self.assertEqual(len(polygon), 4)
        self.assertTrue(all(x >= 0.25 for x, _ in polygon))
        self.assertEqual(set(plot.boundary_segment(1, 0, 0.25, (-1, 1, -1, 1))),
                         {(0.25, -1), (0.25, 1)})
        self.assertEqual(plot.clip_polygon([(-1, -1), (1, -1), (1, 1), (-1, 1)], 1, 0, 2), [])

    def test_asof_sample_matching_and_clearing(self):
        def avoid(sample, count=1, enabled=1, fresh=1, fault=0):
            return {'dgAvoid.sample': sample, 'dgAvoid.count': count, 'dgAvoid.enabled': enabled,
                    'dgAvoid.fresh': fresh, 'dgAvoid.fault': fault}
        def plane(sample):
            return {'dgPlane0.sample': sample, 'dgPlane0.nx': 1, 'dgPlane0.ny': 0, 'dgPlane0.b': 2}
        rows = [(0, 'plane0', plane(4)), (0, 'avoid', avoid(4)),
                (1, 'avoid', avoid(5)), (2, 'plane0', plane(5)),
                (3, 'avoid', avoid(5, fresh=0, fault=1)),
                (4, 'avoid', avoid(6, count=0)),
                (5, 'avoid', avoid(5, enabled=0))]
        self.assertEqual(len(plot.snapshot(rows, 0)[1]), 1)
        self.assertEqual(plot.snapshot(rows, 1)[1:], ([], [0]))
        self.assertEqual(len(plot.snapshot(rows, 2)[1]), 1)
        self.assertEqual(len(plot.snapshot(rows, 3)[1]), 1, 'fault/stale planes remain applied')
        self.assertEqual(plot.snapshot(rows, 4)[1:], ([], []))
        self.assertEqual(plot.snapshot(rows, 5)[1:], ([], []))

    def test_missing_and_invalid_telemetry(self):
        self.assertEqual(plot.snapshot([], 10), ({}, [], []))
        rows = [(0, 'avoid', {'dgAvoid.enabled': 1, 'dgAvoid.count': 2, 'dgAvoid.sample': 1}),
                (0, 'plane0', {'dgPlane0.sample': 1, 'dgPlane0.nx': float('nan'),
                               'dgPlane0.ny': 0, 'dgPlane0.b': 1})]
        self.assertEqual(plot.snapshot(rows, 1)[1:], ([], [0, 1]))

    def test_actual_activation_overrides_enabled_configuration(self):
        plane = {'dgPlane0.sample': 7, 'dgPlane0.nx': 1, 'dgPlane0.ny': 0, 'dgPlane0.b': 2}
        configured = {'dgAvoid.sample': 7, 'dgAvoid.count': 1, 'dgAvoid.enabled': 1,
                      'dgAvoid.active': 0, 'dgAvoid.fresh': 0, 'dgAvoid.fault': 1}
        rows = [(0, 'plane0', plane), (0, 'avoid', configured),
                (1, 'avoid', dict(configured, **{'dgAvoid.active': 1}))]
        self.assertEqual(plot.snapshot(rows, 0)[1:], ([], []))
        self.assertEqual(len(plot.snapshot(rows, 1)[1]), 1)

    def test_demo_roundtrip_and_overwrite_protection(self):
        with tempfile.TemporaryDirectory() as temporary:
            directory = plot.create_demo(temporary)
            metadata, rows = plot.read_run(directory)
            self.assertTrue(metadata['synthetic'])
            self.assertEqual(len(plot.snapshot(rows, 1)[1]), 0)
            self.assertEqual(len(plot.snapshot(rows, 3)[1]), 1)
            self.assertEqual(len(plot.snapshot(rows, 5)[1]), 2)
            with self.assertRaises(ValueError):
                plot.create_demo(temporary)

    def test_demo_video(self):
        try:
            import matplotlib
            import PIL
        except ImportError:
            self.skipTest('optional matplotlib/Pillow unavailable')
        with tempfile.TemporaryDirectory() as temporary:
            directory = plot.create_demo(temporary)
            video = plot.render(directory, directory / 'demo.gif', fps=1, playback_speed=4)
            self.assertGreater(video.stat().st_size, 1000)


if __name__ == '__main__':
    unittest.main()
