"""Navigation consumes telemetry only: it never receives a world or true pose."""
from __future__ import annotations
import math

from .localization import Localizer
from .mapping import OccupancyGrid
from .model import Command, Config, Pose, RangeReading, SensorPacket, wrap
from .planner import astar, choose_frontier


class Navigator:
    def __init__(self, config: Config, initial_pose: Pose):
        self.config = config
        self.localizer = Localizer(config, initial_pose)
        self.grid = OccupancyGrid(config)
        self.grid.mark_body_free(initial_pose)
        self.ranges: dict[str, RangeReading] = {}
        self.command = Command()
        self.trajectory = [Pose(initial_pose.x, initial_pose.y, initial_pose.yaw)]
        self.path: list[tuple[float, float]] = []
        self.goal: tuple[float, float] | None = None
        self.state = "PAUSED"
        self.message = "点击开始探索"
        self.enabled = False
        self.manual_goal = False
        self.now = 0.0
        self._scan_remaining = 2 * math.pi + 0.15
        self._last_yaw = initial_pose.yaw
        self._last_plan_s = -100.0
        self._last_visit_s = -100.0
        self._visits: dict[tuple[int, int], int] = {}
        self._blocked_goals: set[tuple[int, int]] = set()
        self._recovery_until = 0.0
        self._recovery_direction = 1.0
        self._last_position = (initial_pose.x, initial_pose.y)
        self._progress_s = 0.0
        self._idle_scans = 0

    @property
    def pose(self) -> Pose:
        return self.localizer.pose

    def pause(self, emergency: bool = False) -> None:
        self.enabled = False
        self.command = Command()
        self.state = "ESTOP" if emergency else "PAUSED"
        self.message = "急停，点击开始可重新探索" if emergency else "已暂停"

    def start(self) -> None:
        self.enabled = True
        self.state = "EXPLORE"
        self.message = "准备探索"

    def explore(self) -> None:
        self.manual_goal = False
        self.goal = None
        self.path = []
        self._blocked_goals.clear()
        self._idle_scans = 0
        self._scan_remaining = max(self._scan_remaining, 0.8)
        self.start()

    def set_goal(self, x: float, y: float) -> bool:
        free = self.grid.planning_cells()
        goal = self.grid.cell(x, y)
        start = self.grid.cell(self.pose.x, self.pose.y)
        route = astar(start, goal, free)
        if not route:
            self.message = "请选择与小车连通、且留有车体空间的已知空地"
            return False
        self.goal = self.grid.cell_center(*goal)
        self.path = [self.grid.cell_center(*c) for c in route]
        self.manual_goal = True
        self._scan_remaining = 0.0
        self._progress_s = self.now
        self.start()
        return True

    def ingest(self, packet: SensorPacket) -> Command:
        previous_yaw = self.pose.yaw
        updated = self.localizer.update(packet, self.command)
        if math.isfinite(packet.stamp_s):
            self.now = max(self.now, packet.stamp_s)
        mounts = {m.name: m for m in self.config.mounts}
        for reading in packet.ranges:
            if (reading.name not in mounts or not math.isfinite(reading.stamp_s)
                    or not math.isfinite(reading.received_s)):
                continue
            previous = self.ranges.get(reading.name)
            if previous and reading.stamp_s <= previous.stamp_s:
                continue
            if not reading.stamp_s <= reading.received_s <= packet.stamp_s + 1e-6:
                continue
            if (reading.status == "ok" and (reading.range_m is None
                    or not math.isfinite(reading.range_m)
                    or not self.config.sonar_min_m <= reading.range_m <= self.config.sonar_max_m)):
                reading = RangeReading(reading.name, reading.stamp_s, reading.received_s, None, "fault")
            self.ranges[reading.name] = reading
            historical = self.localizer.at(reading.stamp_s)
            if historical is not None and updated and self.localizer.calibrated:
                self.grid.update(historical, mounts[reading.name], reading)
        if updated and self.localizer.calibrated:
            self.grid.mark_body_free(self.pose)
            old = self.trajectory[-1]
            if math.hypot(self.pose.x - old.x, self.pose.y - old.y) > 0.025 or abs(wrap(self.pose.yaw - old.yaw)) > 0.12:
                self.trajectory.append(Pose(self.pose.x, self.pose.y, self.pose.yaw))
            if len(self.trajectory) > 12000:
                self.trajectory = self.trajectory[::2]
            if self.enabled and self._scan_remaining > 0:
                self._scan_remaining = max(0.0, self._scan_remaining - abs(wrap(self.pose.yaw - previous_yaw)))
        self.command = self._control(updated)
        return self.command

    def _telemetry_ready(self) -> bool:
        return all(name in self.ranges and 0 <= self.now - self.ranges[name].stamp_s <= self.config.stale_after_s
                   and (self.ranges[name].status == "no_echo" or
                        (self.ranges[name].status == "ok" and self.ranges[name].range_m is not None
                         and math.isfinite(self.ranges[name].range_m)
                         and self.config.sonar_min_m <= self.ranges[name].range_m <= self.config.sonar_max_m))
                   for name in ("front", "left", "right"))

    def _plan(self) -> bool:
        self._last_plan_s = self.now
        free = self.grid.planning_cells()
        start = self.grid.cell(self.pose.x, self.pose.y)
        # Existing footprint is measured occupied by the robot, but a conservative
        # free-space halo can still exclude its centre. Scan rather than teleport.
        if start not in free:
            self.path = []
            return False
        if self.manual_goal and self.goal:
            route = astar(start, self.grid.cell(*self.goal), free)
        else:
            route = choose_frontier(self.grid, start, free, self._visits, self._blocked_goals)
        self.path = [self.grid.cell_center(*c) for c in route]
        if route:
            self.goal = self.path[-1]
            self._last_position = (self.pose.x, self.pose.y)
            self._progress_s = self.now
            return True
        self.goal = None
        return False

    def _control(self, updated: bool) -> Command:
        if not self.enabled:
            return Command()
        if not updated or not self.localizer.valid:
            self.state, self.message = "SENSOR_WAIT", "等待有效编码器与 IMU 数据"
            return Command()
        if not self.localizer.calibrated:
            self.state, self.message = "CALIBRATING", "静止估计陀螺仪零偏"
            return Command()
        if not self._telemetry_ready():
            self.state, self.message = "SENSOR_WAIT", "测距异常或过期，停车等待恢复"
            return Command()
        if self._scan_remaining > 0:
            self.state, self.message = "SCANNING", "原地转动，用三个固定探头观察四周"
            return Command(0.0, self.config.max_turn_rad_s * 0.85)
        if self.now - self._last_visit_s > 1.0:
            cell = self.grid.cell(self.pose.x, self.pose.y)
            self._visits[cell] = self._visits.get(cell, 0) + 1
            self._last_visit_s = self.now
        if self.now < self._recovery_until:
            self.state, self.message = "AVOIDING", "遇障停车后转向，重新观察与规划"
            return Command(0.0, self._recovery_direction * self.config.max_turn_rad_s * 0.75)
        if not self.path:
            if not self._plan():
                self._idle_scans += 1
                if self._idle_scans <= 3:
                    self._scan_remaining = math.pi * 0.8
                    self.state, self.message = "SCANNING", "没有可达的新观察点，继续观察"
                    return Command()
                self.state, self.message = "NO_FRONTIER", "当前没有可达探索点；可重置或点击恢复探索"
                self.enabled = False
                return Command()
            self._idle_scans = 0
        while self.path and math.hypot(self.path[0][0] - self.pose.x, self.path[0][1] - self.pose.y) < 0.17:
            self.path.pop(0)
        if not self.path:
            self.goal = None
            if self.manual_goal:
                self.manual_goal = False
                self.enabled = False
                self.state, self.message = "ARRIVED", "已到达目标，点击恢复探索继续"
            else:
                self._scan_remaining = math.pi * 0.85
                self.state, self.message = "SCANNING", "到达观察点，继续扫描建图"
            return Command()
        if self.now - self._last_plan_s >= 1.8:
            free = self.grid.planning_cells()
            # Refresh against newly observed obstacles; do not execute a stale path.
            if any(self.grid.cell(*point) not in free for point in self.path):
                self.path = []
                self._last_plan_s = self.now
                self.state, self.message = "REPLANNING", "地图变化，重新规划路线"
                return Command()
            self._last_plan_s = self.now
        if math.hypot(self.pose.x - self._last_position[0], self.pose.y - self._last_position[1]) > 0.12:
            self._last_position = (self.pose.x, self.pose.y)
            self._progress_s = self.now
        if self.now - self._progress_s > 18:
            if self.goal:
                self._blocked_goals.add(self.grid.cell(*self.goal))
            self.path = []
            self.goal = None
            self._scan_remaining = math.pi
            self._progress_s = self.now
            return Command()
        tx, ty = self.path[0]
        bearing = wrap(math.atan2(ty - self.pose.y, tx - self.pose.x) - self.pose.yaw)
        angular = max(-self.config.max_turn_rad_s, min(self.config.max_turn_rad_s, 2.2 * bearing))
        if abs(bearing) > 0.40:
            self.state, self.message = "TURNING", "对准规划路径"
            return Command(0.0, angular)
        front = self.ranges["front"]
        if front.status != "ok" or front.range_m is None:
            self.path = []
            self._scan_remaining = math.pi * 0.7
            self.state, self.message = "SCANNING", "前方距离未知，停下补充观测"
            return Command()
        if front.range_m <= self.config.stop_distance_m:
            left, right = self.ranges["left"], self.ranges["right"]
            self._recovery_direction = 1.0 if (left.range_m or 0) >= (right.range_m or 0) else -1.0
            self._recovery_until = self.now + 2.0
            if self.goal:
                self._blocked_goals.add(self.grid.cell(*self.goal))
            self.path = []
            self.state, self.message = "AVOIDING", "前方接近障碍，立即停止前进"
            return Command()
        speed = self.config.max_speed_m_s * max(0.15, 1 - abs(bearing) / 0.65)
        speed *= min(1.0, max(0.1, (front.range_m - self.config.stop_distance_m) / 0.40))
        self.state, self.message = ("NAVIGATING", "前往指定目标") if self.manual_goal else ("EXPLORING", "沿已探测空地前往新的观察点")
        return Command(speed, angular)
