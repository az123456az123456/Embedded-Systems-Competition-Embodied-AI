#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""任务4 导航核心单元测试。"""

import math
import unittest

from navigation import GridMap, NavController, Obstacle, Odometry, Pose, wrap_angle


class GridMapTest(unittest.TestCase):
    def test_astar_finds_path(self):
        grid = GridMap(
            width_mm=3000.0, height_mm=3000.0, cell_size_mm=100.0,
            robot_radius_mm=100.0,
            obstacles=[Obstacle(1500, 1500, 150)],
        )
        start = (200.0, 200.0)
        goal = (2800.0, 2800.0)
        path = grid.plan(start, goal)
        self.assertIsNotNone(path)
        self.assertGreaterEqual(len(path), 2)
        self.assertAlmostEqual(path[0][0], start[0])
        self.assertAlmostEqual(path[0][1], start[1])
        self.assertAlmostEqual(path[-1][0], goal[0])
        self.assertAlmostEqual(path[-1][1], goal[1])

    def test_astar_blocked_goal(self):
        grid = GridMap(
            width_mm=1000.0, height_mm=1000.0, cell_size_mm=100.0,
            robot_radius_mm=200.0,
            obstacles=[Obstacle(900, 900, 300)],
        )
        self.assertIsNone(grid.plan((100, 100), (900, 900)))

    def test_wrap_angle(self):
        self.assertAlmostEqual(wrap_angle(3 * math.pi), math.pi)
        self.assertAlmostEqual(wrap_angle(-3 * math.pi), -math.pi)
        self.assertAlmostEqual(wrap_angle(0), 0.0)


class OdometryTest(unittest.TestCase):
    def test_straight(self):
        odom = Odometry(Pose(0, 0, 0.0), track_width_mm=160.0)
        odom.update(2.0, 1000.0, 1000.0)
        self.assertAlmostEqual(odom.pose.x_mm, 2000.0, places=3)
        self.assertAlmostEqual(odom.pose.y_mm, 0.0, places=3)
        self.assertAlmostEqual(odom.pose.theta_rad, 0.0, places=6)

    def test_spin(self):
        odom = Odometry(Pose(0, 0, 0.0), track_width_mm=160.0)
        # 右轮 200、左轮 -200 -> w = 400/160 = 2.5 rad/s
        odom.update(0.1, -200.0, 200.0)
        self.assertAlmostEqual(odom.pose.theta_rad, 0.25, places=4)
        self.assertAlmostEqual(odom.pose.x_mm, 0.0, places=6)


class NavControllerTest(unittest.TestCase):
    def setUp(self):
        self.ctrl = NavController(reached_radius_mm=50.0, obstacle_stop_cm=30.0)

    def test_drive_forward(self):
        action = self.ctrl.step(Pose(0, 0, 0.0), (1000, 0))
        self.assertEqual(action.state, self.ctrl.STATE_DRIVE)
        self.assertGreater(action.v_mm_s, 0)
        self.assertLess(abs(action.w_mrad_s), 1)

    def test_turn_to_side(self):
        action = self.ctrl.step(Pose(0, 0, 0.0), (0, 1000))
        self.assertEqual(action.state, self.ctrl.STATE_TURN)
        self.assertEqual(action.v_mm_s, 0)
        self.assertGreater(action.w_mrad_s, 0)

    def test_reached(self):
        action = self.ctrl.step(Pose(0, 0, 0.0), (30, 0))
        self.assertEqual(action.state, self.ctrl.STATE_REACHED)

    def test_obstacle_stop(self):
        action = self.ctrl.step(Pose(0, 0, 0.0), (1000, 0), distance_cm=20)
        self.assertEqual(action.state, self.ctrl.STATE_OBSTACLE)
        self.assertEqual(action.v_mm_s, 0)
        self.assertEqual(action.w_mrad_s, 0)


if __name__ == "__main__":
    unittest.main()
