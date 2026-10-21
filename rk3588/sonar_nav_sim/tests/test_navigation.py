"""Contracts that remain important when simulated telemetry is replaced by IO."""
from dataclasses import replace
import math
import unittest
from unittest.mock import patch

from navsim.localization import Localizer
from navsim.mapping import OccupancyGrid
from navsim.model import Command, Config, Pose, RangeReading, SensorPacket
from navsim.navigator import Navigator
from navsim.planner import astar


def packet(t, left=0, right=0, gyro=0.0, ranges=()):
    return SensorPacket(t, left, right, gyro, tuple(ranges))


class MappingTests(unittest.TestCase):
    def test_no_echo_and_fault_do_not_clear_unknown_or_occupied_cells(self):
        config = Config()
        grid = OccupancyGrid(config)
        cell = grid.cell(1.8, 1.1)
        grid.values[grid.index(*cell)] = 2.0
        before = grid.values[:]
        seen = grid.seen[:]
        for status, value in (("no_echo", None), ("no_echo", 4.5),
                              ("fault", None), ("ok", math.nan), ("ok", math.inf)):
            self.assertFalse(grid.update(Pose(), config.mounts[0],
                                         RangeReading("front", 0, .1, value, status)))
        self.assertEqual(grid.values, before)
        self.assertEqual(grid.seen, seen)

    def test_unknown_space_is_not_a_planning_corridor(self):
        config = replace(Config(), width_m=2, height_m=2, robot_radius_m=.1)
        grid = OccupancyGrid(config)
        grid.mark_body_free(Pose(.5, .5, 0))
        self.assertNotIn(grid.cell(1.5, 1.5), grid.planning_cells())
        self.assertEqual(astar((0, 0), (1, 1), {(0, 0), (1, 1)}), [])


class LocalizationTests(unittest.TestCase):
    def test_yaw_history_interpolates_through_wrap_boundary(self):
        localizer = Localizer(Config(), Pose(1, 1, math.radians(179)))
        localizer.history.clear()
        localizer.history.extend(((0, Pose(1, 1, math.radians(179))),
                                  (.1, Pose(1.2, 1, math.radians(-179)))))
        pose = localizer.at(.05)
        self.assertAlmostEqual(pose.x, 1.1)
        self.assertAlmostEqual(abs(pose.yaw), math.pi)
        self.assertIsNone(localizer.at(-.01))
        self.assertIsNone(localizer.at(.2))

    def test_pose_history_does_not_interpolate_across_telemetry_gap(self):
        localizer = Localizer(Config(), Pose())
        localizer.calibrated = True
        localizer.update(packet(0), Command())
        localizer.update(packet(.1), Command())
        self.assertFalse(localizer.update(packet(1.0), Command()))
        self.assertTrue(localizer.update(packet(1.1), Command()))
        self.assertIsNone(localizer.at(.5))

    def test_calibration_requires_stationary_commands_and_encoder_counts(self):
        localizer = Localizer(Config(), Pose())
        for i in range(50):
            localizer.update(packet(i * .05, gyro=.002), Command(.1, 0))
        self.assertFalse(localizer.calibrated)
        for i in range(50, 90):
            localizer.update(packet(i * .05, gyro=.002), Command())
        self.assertTrue(localizer.calibrated)
        self.assertAlmostEqual(localizer.gyro_bias, .002)


class NavigationTests(unittest.TestCase):
    def ready_navigator(self):
        nav = Navigator(Config(), Pose())
        nav.localizer.calibrated = True
        nav.start()
        nav.ingest(packet(0))
        return nav

    def test_mapping_uses_acquisition_pose_not_delivery_pose(self):
        nav = self.ready_navigator()
        ticks = round(.08 / nav.config.metres_per_tick)
        reading = RangeReading("front", 0, .1, 1.0)
        with patch.object(nav.grid, "update", wraps=nav.grid.update) as update:
            nav.ingest(packet(.1, ticks, ticks, ranges=(reading,)))
        self.assertGreater(nav.pose.x, 1.17)
        self.assertEqual(update.call_count, 1)
        measured_pose = update.call_args.args[0]
        self.assertAlmostEqual(measured_pose.x, 1.1)

    def test_future_and_duplicate_ranges_cannot_rewrite_measurement_history(self):
        nav = self.ready_navigator()
        good = RangeReading("front", 0, .1, 1.0)
        nav.ingest(packet(.1, ranges=(good,)))
        future = RangeReading("front", 5, 5.1, .05)
        duplicate = replace(good, range_m=.05)
        nav.ingest(packet(.2, ranges=(future, duplicate)))
        self.assertEqual(nav.ranges["front"], good)

    def test_invalid_ok_distance_is_not_ready_telemetry(self):
        for distance in (None, math.inf, math.nan, -1.0, 10.0):
            with self.subTest(distance=distance):
                nav = self.ready_navigator()
                readings = tuple(RangeReading(name, .1, .1,
                                              distance if name == "front" else 1.0)
                                 for name in ("front", "left", "right"))
                command = nav.ingest(packet(.1, ranges=readings))
                self.assertFalse(nav._telemetry_ready())
                self.assertEqual(command, Command())

    def test_expired_range_data_halts_motion(self):
        nav = self.ready_navigator()
        readings = tuple(RangeReading(name, 0, .1, 1.0)
                         for name in ("front", "left", "right"))
        nav.ingest(packet(.1, ranges=readings))
        command = None
        for i in range(2, 10):
            command = nav.ingest(packet(i * .1))
        self.assertEqual(command, Command())
        self.assertEqual(nav.state, "SENSOR_WAIT")

    def test_emergency_stop_prevents_new_motion_commands(self):
        nav = self.ready_navigator()
        nav.pause(emergency=True)
        readings = tuple(RangeReading(name, .1, .1, 1.0)
                         for name in ("front", "left", "right"))
        self.assertEqual(nav.ingest(packet(.1, ranges=readings)), Command())
        self.assertEqual(nav.state, "ESTOP")


if __name__ == "__main__":
    unittest.main()
