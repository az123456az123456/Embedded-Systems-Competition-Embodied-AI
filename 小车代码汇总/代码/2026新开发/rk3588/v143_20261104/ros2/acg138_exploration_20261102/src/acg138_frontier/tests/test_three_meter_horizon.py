"""Range limit, scan health, and mapping invalid-return semantics."""
import math
import struct
import unittest
from pathlib import Path
import yaml
from acg138_frontier.exploration_motion import scan_clearance, plan_frontier
from acg138_frontier.core import Grid
from acg138_frontier.dynamic_map import DynamicMap


class ThreeMeterTests(unittest.TestCase):
    def clearance(self, ranges):
        return scan_clearance(ranges, -math.pi, math.pi / 250, .05, 12., 0, 0, 0)

    def test_map_horizon_does_not_truncate_live_clearance(self):
        front,circle=self.clearance([4.] * 500)
        self.assertAlmostEqual(circle,4.)
        self.assertGreater(front,3.9)
        self.assertLessEqual(front,4.)

    def test_exact_three_meters_and_near_still_considered(self):
        rays = [4.] * 500
        rays[250] = 3.
        self.assertEqual(self.clearance(rays), (3., 3.))
        rays[250] = .2
        self.assertEqual(self.clearance(rays), (.2, .2))
        rays[250] = 3.00001
        front,circle=self.clearance(rays)
        self.assertAlmostEqual(front,3.00001)
        self.assertAlmostEqual(circle,3.00001)

    def test_genuine_invalid_coverage_not_relaxed(self):
        with self.assertRaisesRegex(ValueError, 'lidar_coverage_incomplete'):
            self.clearance([float('nan')] * 500)

    def test_official_filter_boundary_nan_and_slam_config(self):
        root = Path(__file__).parents[1] / 'config'
        p = yaml.safe_load((root / 'laser_filters.yaml').read_text())['scan_to_scan_filter_chain']['ros__parameters']['filter1']['params']
        next_float = struct.unpack('<f', struct.pack('<I', 0x40400001))[0]
        self.assertLess(3., p['upper_threshold'])
        self.assertGreaterEqual(next_float, p['upper_threshold'])
        self.assertTrue(math.isnan(p['upper_replacement_value']))
        cfg = yaml.safe_load((root / 'slam.yaml').read_text())['slam_toolbox']['ros__parameters']
        self.assertEqual(cfg['resolution'], .025)
        self.assertEqual(cfg['max_laser_range'], 3.)

    def test_filtered_far_does_not_clear_or_paint_map(self):
        g = Grid(300, 300, .03, -4.5, -4.5, (-1,) * 90000)
        layer = DynamicMap()
        layer.bind(g)
        for n in range(4):
            stamp = 1000000000 + n * 100000000
            self.assertFalse(layer.observe(stamp, stamp + 10000000, (0, 0, 0),
                                           [float('nan')] * 500, -math.pi,
                                           math.pi / 250, .05, 12.))
        self.assertEqual(layer.cells(), list(g.cells))

    def test_three_cm_float32_can_plan(self):
        res = struct.unpack('<f', struct.pack('<f', .03))[0]
        width = 120
        cells = tuple(0 if 10 < x < 110 and 10 < y < 110 else -1
                      for y in range(width) for x in range(width))
        p = plan_frontier((width, width, res, -width * res / 2, -width * res / 2, 0, cells), (0, 0, 0), [])
        self.assertTrue(p['path'])

    def test_two_point_five_cm_float32_can_plan(self):
        res = struct.unpack('<f', struct.pack('<f', .025))[0]
        width = 144
        cells = tuple(0 if 12 < x < 132 and 12 < y < 132 else -1
                      for y in range(width) for x in range(width))
        p = plan_frontier((width, width, res, -width * res / 2,
                           -width * res / 2, 0, cells), (0, 0, 0), [])
        self.assertTrue(p['path'])
