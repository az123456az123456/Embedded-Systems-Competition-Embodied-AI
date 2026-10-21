from dataclasses import replace
import math
import unittest

from navsim.model import Command, Config, Pose
from navsim.simulator import Simulator
from navsim.world import World, make_world


def quiet_config(**kwargs):
    return replace(Config(), sonar_noise_m=0, sonar_dropout=0,
                   gyro_bias_rad_s=0, gyro_noise_rad_s=0,
                   wheel_scale_error=0, slip_fraction=0, **kwargs)


class WorldTests(unittest.TestCase):
    def test_rays_and_circle_corner(self):
        world = World(8, 6, [(3, 2, 1, 1)], Pose(1, 1, 0))
        self.assertAlmostEqual(world.raycast(1, 2.5, 0, 10), 2)
        self.assertAlmostEqual(world.raycast(1, 1, -math.pi / 2, 10), 1)
        self.assertAlmostEqual(world.raycast(1, 1, 0, 2), 2)
        self.assertEqual(world.raycast(3.5, 2.5, 0, 10), 0)
        self.assertTrue(world.collides(Pose(2.9, 1.9, 0), .15))
        self.assertFalse(world.collides(Pose(2.85, 1.85, 0), .15))

    def test_scenes_have_valid_starts(self):
        for scene in ("apartment", "open", "corridor"):
            world = make_world(scene)
            self.assertFalse(world.collides(world.start_pose, Config().robot_radius_m))


class SimulatorTests(unittest.TestCase):
    def test_acquisition_timestamp_latency_and_new_samples_only(self):
        sim = Simulator(quiet_config(), World(8, 6, [], Pose(1, 2, 0)))
        self.assertEqual(sim.advance(.05, Command()).ranges, ())
        packet = sim.advance(.05, Command())
        self.assertEqual(len(packet.ranges), 3)
        self.assertEqual({r.name for r in packet.ranges}, {"front", "left", "right"})
        self.assertTrue(all(r.stamp_s == 0 and r.received_s == .1 for r in packet.ranges))
        ranges = {r.name: r for r in packet.ranges}
        # The 60-degree cone sees the lower boundary even though its centre
        # ray would not hit anything within the maximum range.
        self.assertEqual(ranges["front"].status, "ok")
        self.assertAlmostEqual(ranges["front"].range_m, 4.0)
        self.assertAlmostEqual(ranges["right"].range_m, 1.84)
        self.assertAlmostEqual(ranges["left"].range_m, 2.0)
        self.assertEqual(sim.advance(.01, Command()).ranges, ())

    def test_no_echo_is_not_a_max_range_measurement(self):
        sim = Simulator(quiet_config(), World(20, 20, [], Pose(10, 10, 0)))
        readings = sim.advance(.1, Command()).ranges
        self.assertTrue(all(r.status == "no_echo" and r.range_m is None for r in readings))

    def test_cone_detects_obstacle_away_from_central_ray(self):
        world = World(10, 10, [(2.0, 2.4, .2, .2)], Pose(1, 2, 0))
        sim = Simulator(quiet_config(), world)
        front = next(r for r in sim.advance(.1, Command()).ranges if r.name == "front")
        self.assertEqual(front.status, "ok")
        self.assertLess(front.range_m, 1.3)

    def test_near_blind_zone_reports_invalid(self):
        sim = Simulator(quiet_config(), World(4, 4, [], Pose(3.81, 2, 0)))
        front = next(r for r in sim.advance(.1, Command()).ranges if r.name == "front")
        self.assertEqual(front.status, "fault")
        self.assertIsNone(front.range_m)

    def test_encoder_integral_and_stationary_gyro(self):
        config = quiet_config()
        sim = Simulator(config, World(8, 6, [], Pose(1, 2, 0)))
        packet = sim.advance(2, Command(.2, 0))
        distance = .2 * (2 - sim.MOTOR_TIME_CONSTANT_S * (1 - math.exp(-2 / sim.MOTOR_TIME_CONSTANT_S)))
        self.assertLess(abs(packet.left_ticks * config.metres_per_tick - distance), config.metres_per_tick)
        self.assertEqual(packet.left_ticks, packet.right_ticks)
        self.assertAlmostEqual(sim.truth_pose.x, 1 + distance)
        self.assertAlmostEqual(packet.gyro_z_rad_s, 0)
        self.assertAlmostEqual(sim.travel_m, distance)

    def test_long_step_samples_each_period(self):
        sim = Simulator(quiet_config(), World(8, 6, [], Pose(1, 2, 0)))
        packet = sim.advance(1, Command(.2, 0))
        fronts = [r for r in packet.ranges if r.name == "front"]
        self.assertEqual(len(fronts), 10)
        self.assertAlmostEqual(fronts[-1].stamp_s, .9)
        self.assertTrue(all(r.received_s <= 1 + 1e-9 for r in packet.ranges))

    def test_collision_stops_translation_but_not_encoder(self):
        config = quiet_config()
        sim = Simulator(config, World(2, 2, [], Pose(1.5, 1, 0)))
        packet = sim.advance(5, Command(.2, 0))
        self.assertLessEqual(sim.truth_pose.x + config.robot_radius_m, 2)
        self.assertEqual(sim.collision_count, 1)
        first_ticks = packet.left_ticks
        packet = sim.advance(2, Command(.2, 0))
        self.assertEqual(sim.collision_count, 1)
        self.assertGreater(packet.left_ticks, first_ticks)
        sim.advance(2, Command(-.2, 0))
        self.assertLess(sim.truth_pose.x, 1.6)

    def test_seed_reproduces_same_schedule(self):
        first = Simulator(Config(), make_world())
        second = Simulator(Config(), make_world())
        for _ in range(100):
            self.assertEqual(first.advance(.02, Command(.1, .1)),
                             second.advance(.02, Command(.1, .1)))
        self.assertEqual(first.truth_pose, second.truth_pose)

    def test_invalid_dt_rejected(self):
        sim = Simulator(quiet_config(), make_world())
        for dt in (-1, math.nan, math.inf):
            with self.assertRaises(ValueError):
                sim.advance(dt, Command())


if __name__ == "__main__":
    unittest.main()
