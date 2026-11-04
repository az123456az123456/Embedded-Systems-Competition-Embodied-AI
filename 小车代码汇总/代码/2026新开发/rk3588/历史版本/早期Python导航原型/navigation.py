#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""任务4 自主导航核心库。

提供：
  - 栅格地图 + A* 路径规划
  - 差速里程计
  - 路径点导航控制器（转向/前进/到达/遇障停车）

本模块不依赖硬件，也不启动串口，可单独单元测试。
"""

from __future__ import annotations

import heapq
import math
from dataclasses import dataclass
from typing import Iterable, List, Optional, Sequence, Tuple


def wrap_angle(angle: float) -> float:
    """把角度归一化到 [-pi, pi]。"""

    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


@dataclass
class Pose:
    """机器人位姿（毫米 / 弧度）。"""

    x_mm: float = 0.0
    y_mm: float = 0.0
    theta_rad: float = 0.0

    def distance_to(self, x_mm: float, y_mm: float) -> float:
        return math.hypot(self.x_mm - x_mm, self.y_mm - y_mm)


@dataclass(frozen=True)
class Obstacle:
    """圆形障碍物。"""

    x_mm: float
    y_mm: float
    radius_mm: float


class GridMap:
    """带障碍物膨胀的栅格地图。"""

    def __init__(
        self,
        width_mm: float = 4000.0,
        height_mm: float = 4000.0,
        cell_size_mm: float = 100.0,
        robot_radius_mm: float = 150.0,
        obstacles: Sequence[Obstacle] = (),
    ) -> None:
        self.width_mm = float(width_mm)
        self.height_mm = float(height_mm)
        self.cell_size_mm = float(cell_size_mm)
        self.robot_radius_mm = float(robot_radius_mm)
        self.obstacles = list(obstacles)
        self.cols = max(1, int(math.ceil(self.width_mm / self.cell_size_mm)))
        self.rows = max(1, int(math.ceil(self.height_mm / self.cell_size_mm)))
        self.occupied: set[tuple[int, int]] = set()
        self._mark_obstacles()

    def _mark_obstacles(self) -> None:
        for col in range(self.cols):
            for row in range(self.rows):
                cx, cy = self.cell_center(col, row)
                for ob in self.obstacles:
                    limit = ob.radius_mm + self.robot_radius_mm + self.cell_size_mm * 0.5
                    if math.hypot(cx - ob.x_mm, cy - ob.y_mm) <= limit:
                        self.occupied.add((col, row))
                        break

    def cell_center(self, col: int, row: int) -> Tuple[float, float]:
        return (
            (col + 0.5) * self.cell_size_mm,
            (row + 0.5) * self.cell_size_mm,
        )

    def point_to_cell(self, x_mm: float, y_mm: float) -> Tuple[int, int]:
        col = min(self.cols - 1, max(0, int(x_mm // self.cell_size_mm)))
        row = min(self.rows - 1, max(0, int(y_mm // self.cell_size_mm)))
        return col, row

    def in_bounds(self, col: int, row: int) -> bool:
        return 0 <= col < self.cols and 0 <= row < self.rows

    def is_free(self, col: int, row: int) -> bool:
        return self.in_bounds(col, row) and (col, row) not in self.occupied

    def neighbors(self, col: int, row: int) -> Iterable[Tuple[int, int]]:
        for dc, dr in (
            (1, 0), (-1, 0), (0, 1), (0, -1),
            (1, 1), (1, -1), (-1, 1), (-1, -1),
        ):
            nc, nr = col + dc, row + dr
            if not self.is_free(nc, nr):
                continue
            if dc and dr and not (self.is_free(col + dc, row) and self.is_free(col, row + dr)):
                continue
            yield nc, nr

    @staticmethod
    def move_cost(col_a: int, row_a: int, col_b: int, row_b: int) -> float:
        return math.sqrt((col_b - col_a) ** 2 + (row_b - row_a) ** 2)

    def plan(
        self, start_mm: Tuple[float, float], goal_mm: Tuple[float, float]
    ) -> Optional[List[Tuple[float, float]]]:
        """A* 规划，返回途经点（mm）；无路返回 None。"""

        start_cell = self.point_to_cell(*start_mm)
        goal_cell = self.point_to_cell(*goal_mm)
        if not self.is_free(*start_cell) or not self.is_free(*goal_cell):
            return None

        open_heap: list[tuple[float, int, int]] = [(0.0, start_cell[0], start_cell[1])]
        came_from: dict[tuple[int, int], tuple[int, int]] = {}
        g_score: dict[tuple[int, int], float] = {start_cell: 0.0}

        while open_heap:
            _, col, row = heapq.heappop(open_heap)
            node = (col, row)
            if node == goal_cell:
                return self._reconstruct(came_from, start_cell, goal_cell, start_mm, goal_mm)
            for nb in self.neighbors(col, row):
                tentative = g_score[node] + self.move_cost(col, row, *nb)
                if tentative < g_score.get(nb, math.inf):
                    came_from[nb] = node
                    g_score[nb] = tentative
                    heuristic = math.hypot(nb[0] - goal_cell[0], nb[1] - goal_cell[1])
                    heapq.heappush(open_heap, (tentative + heuristic, nb[0], nb[1]))
        return None

    def _reconstruct(
        self,
        came_from: dict[tuple[int, int], tuple[int, int]],
        start_cell: Tuple[int, int],
        goal_cell: Tuple[int, int],
        start_mm: Tuple[float, float],
        goal_mm: Tuple[float, float],
    ) -> List[Tuple[float, float]]:
        cells: List[Tuple[int, int]] = []
        node = goal_cell
        while node != start_cell:
            cells.append(node)
            node = came_from[node]
        cells.reverse()
        points = [start_mm]
        for col, row in cells:
            points.append(self.cell_center(col, row))
        points.append(goal_mm)
        return points


class Odometry:
    """基于左右轮速度的差速里程计。"""

    def __init__(self, pose: Optional[Pose] = None, track_width_mm: float = 160.0) -> None:
        self.pose = Pose() if pose is None else pose
        self.track_width_mm = float(track_width_mm)

    def update(
        self, dt: float, speed_left_mm_s: float, speed_right_mm_s: float
    ) -> Pose:
        dt = max(0.0, float(dt))
        v = (float(speed_left_mm_s) + float(speed_right_mm_s)) / 2.0
        w = (float(speed_right_mm_s) - float(speed_left_mm_s)) / self.track_width_mm
        self.pose.theta_rad = wrap_angle(self.pose.theta_rad + w * dt)
        self.pose.x_mm += v * math.cos(self.pose.theta_rad) * dt
        self.pose.y_mm += v * math.sin(self.pose.theta_rad) * dt
        return self.pose

    def reset(self, pose: Optional[Pose] = None) -> None:
        self.pose = Pose() if pose is None else pose


class NavAction:
    __slots__ = ("state", "v_mm_s", "w_mrad_s", "note")

    def __init__(self, state: str, v_mm_s: float = 0.0, w_mrad_s: float = 0.0, note: str = "") -> None:
        self.state = state
        self.v_mm_s = float(v_mm_s)
        self.w_mrad_s = float(w_mrad_s)
        self.note = note

    def __repr__(self) -> str:
        return f"NavAction({self.state}, v={self.v_mm_s:.0f}, w={self.w_mrad_s:.0f})"


class NavController:
    """把“当前位置 + 目标点 + 传感器”转成 SET_TWIST 指令。"""

    STATE_STOP = "STOP"
    STATE_TURN = "TURN"
    STATE_DRIVE = "DRIVE"
    STATE_REACHED = "REACHED"
    STATE_OBSTACLE = "OBSTACLE"

    def __init__(
        self,
        max_v_mm_s: float = 800.0,
        max_w_mrad_s: float = 1000.0,
        align_threshold_rad: float = 0.12,
        reached_radius_mm: float = 100.0,
        obstacle_stop_cm: float = 30.0,
        obstacle_slow_cm: float = 60.0,
        k_angle: float = 3.0,
        k_forward: float = 1.5,
    ) -> None:
        self.max_v_mm_s = float(max_v_mm_s)
        self.max_w_mrad_s = float(max_w_mrad_s)
        self.align_threshold_rad = float(align_threshold_rad)
        self.reached_radius_mm = float(reached_radius_mm)
        self.obstacle_stop_cm = float(obstacle_stop_cm)
        self.obstacle_slow_cm = float(obstacle_slow_cm)
        self.k_angle = float(k_angle)
        self.k_forward = float(k_forward)

    @staticmethod
    def _clamp(value: float, limit: float) -> float:
        return max(-limit, min(limit, value))

    def step(
        self,
        pose: Pose,
        waypoint: Optional[Tuple[float, float]],
        distance_cm: Optional[float] = None,
        obstacle_flag: bool = False,
    ) -> NavAction:
        """输入一帧位姿和目标点，返回期望运动。"""

        if obstacle_flag or (
            distance_cm is not None and 0 <= distance_cm <= self.obstacle_stop_cm
        ):
            return NavAction(self.STATE_OBSTACLE, 0.0, 0.0, "遇障停车")

        if waypoint is None:
            return NavAction(self.STATE_STOP, 0.0, 0.0, "无目标点")

        wx, wy = waypoint
        dx, dy = wx - pose.x_mm, wy - pose.y_mm
        distance = math.hypot(dx, dy)
        if distance <= self.reached_radius_mm:
            return NavAction(self.STATE_REACHED, 0.0, 0.0, "到达路径点")

        bearing = wrap_angle(math.atan2(dy, dx) - pose.theta_rad)

        if abs(bearing) > self.align_threshold_rad:
            # tanh 限幅：大角度快速转，小角度渐进收敛，避免来回振荡。
            w = math.copysign(
                self.max_w_mrad_s * math.tanh(self.k_angle * abs(bearing)), bearing
            )
            return NavAction(
                self.STATE_TURN,
                0.0,
                self._clamp(w, self.max_w_mrad_s),
                f"偏角 {math.degrees(bearing):.1f}°",
            )

        v = self._clamp(self.k_forward * distance, self.max_v_mm_s)
        w = self._clamp(
            math.copysign(
                self.max_w_mrad_s * math.tanh(self.k_angle * abs(bearing)), bearing
            ),
            self.max_w_mrad_s,
        )
        if distance_cm is not None and distance_cm <= self.obstacle_slow_cm:
            v *= 0.5
        return NavAction(self.STATE_DRIVE, v, w, f"前进 {distance:.0f}mm")
