"""Wheel odometry with bias-calibrated gyro and timestamped pose history."""
from __future__ import annotations

from collections import deque
import math

from .model import Command, Config, Pose, SensorPacket, wrap


class Localizer:
    def __init__(self, config: Config, initial_pose: Pose):
        self.config = config
        self.pose = Pose(initial_pose.x, initial_pose.y, initial_pose.yaw)
        self.history = deque([(0.0, Pose(self.pose.x, self.pose.y, self.pose.yaw))])
        self.previous: SensorPacket | None = None
        self.gyro_bias = 0.0
        self.calibrated = False
        self.calibration_progress = 0.0
        self._bias_samples: list[float] = []
        self._calibration_start: float | None = None
        self.valid = True
        self.distance_m = 0.0
        self.speed_m_s = 0.0

    def update(self, packet: SensorPacket, last_command: Command) -> bool:
        values = (packet.stamp_s, packet.gyro_z_rad_s)
        if (not all(isinstance(v, (int, float)) and math.isfinite(v) for v in values)
                or type(packet.left_ticks) is not int or type(packet.right_ticks) is not int
                or not packet.encoder_valid or not packet.imu_valid):
            self.valid = False
            return False
        previous = self.previous
        if previous is None:
            self.previous = packet
            self.history.clear()
            self.history.append((packet.stamp_s, Pose(self.pose.x, self.pose.y, self.pose.yaw)))
            return False
        dt = packet.stamp_s - previous.stamp_s
        if dt <= 0:
            self.valid = False
            return False
        self.previous = packet
        if dt > 0.30:
            # A gap is not an invitation to integrate a stale gyro over it.
            self.valid = False
            self.history.clear()
            self.history.append((packet.stamp_s, Pose(self.pose.x, self.pose.y, self.pose.yaw)))
            return False
        dl = (packet.left_ticks - previous.left_ticks) * self.config.metres_per_tick
        dr = (packet.right_ticks - previous.right_ticks) * self.config.metres_per_tick
        wheel_limit = (self.config.max_speed_m_s + self.config.max_turn_rad_s * self.config.track_width_m) * dt + 0.03
        if max(abs(dl), abs(dr)) > wheel_limit * 2:
            self.valid = False
            return False
        self.valid = True
        stationary = (abs(dl) + abs(dr) < self.config.metres_per_tick * 2
                      and abs(last_command.linear_m_s) < 1e-6
                      and abs(last_command.angular_rad_s) < 1e-6)
        if not self.calibrated:
            if stationary and abs(packet.gyro_z_rad_s) < math.radians(8):
                if self._calibration_start is None:
                    self._calibration_start = packet.stamp_s
                self._bias_samples.append(packet.gyro_z_rad_s)
                span = packet.stamp_s - self._calibration_start
                self.calibration_progress = min(1.0, span / 1.5)
                if span >= 1.5 and len(self._bias_samples) >= 20:
                    self.gyro_bias = sum(self._bias_samples) / len(self._bias_samples)
                    self.calibrated = True
            else:
                self._bias_samples.clear()
                self._calibration_start = None
                self.calibration_progress = 0.0
        if self.calibrated:
            wheel_turn = (dr - dl) / self.config.track_width_m
            gyro_turn = (packet.gyro_z_rad_s - self.gyro_bias) * dt
            if stationary:
                self.gyro_bias += min(0.05, dt / 12.0) * (packet.gyro_z_rad_s - self.gyro_bias)
                turn = 0.0
            else:
                turn = 0.25 * wheel_turn + 0.75 * gyro_turn
            distance = (dl + dr) / 2
            mid = self.pose.yaw + turn / 2
            self.pose = Pose(self.pose.x + distance * math.cos(mid),
                             self.pose.y + distance * math.sin(mid), wrap(self.pose.yaw + turn))
            self.distance_m += abs(distance)
            self.speed_m_s = distance / dt
        self.history.append((packet.stamp_s, Pose(self.pose.x, self.pose.y, self.pose.yaw)))
        while len(self.history) > 2 and self.history[1][0] < packet.stamp_s - 8:
            self.history.popleft()
        return True

    def at(self, stamp_s: float) -> Pose | None:
        if not self.history or stamp_s < self.history[0][0] - 1e-6 or stamp_s > self.history[-1][0] + 1e-6:
            return None
        for i in range(1, len(self.history)):
            t1, p1 = self.history[i]
            if t1 >= stamp_s:
                t0, p0 = self.history[i - 1]
                if t1 - t0 > 0.30:
                    return None
                f = (stamp_s - t0) / max(1e-9, t1 - t0)
                return Pose(p0.x + f * (p1.x - p0.x), p0.y + f * (p1.y - p0.y),
                            wrap(p0.yaw + f * wrap(p1.yaw - p0.yaw)))
        return self.history[-1][1]
