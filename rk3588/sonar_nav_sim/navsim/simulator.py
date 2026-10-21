"""Differential-drive physics, encoders, IMU and three delayed sonar cones.

This module deliberately models no perfect range image or obstacle contacts in
the sensor contract. A real RobotIO adapter can supply the same SensorPacket.
"""
from __future__ import annotations

from collections import deque
import math
import random

from .model import Command, Config, Pose, RangeReading, SensorPacket, wrap
from .world import World


class Simulator:
    MOTOR_TIME_CONSTANT_S = 0.12
    PHYSICS_STEP_S = 0.01
    SONAR_RAYS = 13

    def __init__(self, config: Config, world: World):
        config.validate()
        self.config = config
        self.world = world
        self.truth_pose = Pose(world.start_pose.x, world.start_pose.y, world.start_pose.yaw)
        if world.collides(self.truth_pose, config.robot_radius_m):
            raise ValueError("Robot starts inside an obstacle")
        self.time_s = 0.0
        self.collision_count = 0
        self.travel_m = 0.0
        self._rng = random.Random(config.seed)
        self._left_speed = self._right_speed = 0.0
        self._left_distance = self._right_distance = 0.0
        self._true_angular = 0.0
        self._in_contact = False
        self._left_slip = self._right_slip = 0.0
        self._next_slip_s = 0.0
        self._next_sample_s = config.sonar_period_s
        self._pending: deque[RangeReading] = deque()
        self._sample_sonar(0.0)

    def advance(self, dt_s: float, command: Command) -> SensorPacket:
        if not math.isfinite(dt_s) or dt_s < 0:
            raise ValueError("dt_s must be finite and non-negative")
        if not all(math.isfinite(x) for x in (command.linear_m_s, command.angular_rad_s)):
            raise ValueError("Motor command must be finite")
        config = self.config
        v = max(-config.max_speed_m_s, min(config.max_speed_m_s, command.linear_m_s))
        omega = max(-config.max_turn_rad_s, min(config.max_turn_rad_s, command.angular_rad_s))
        left_target = v - omega * config.track_width_m / 2
        right_target = v + omega * config.track_width_m / 2
        end_s = self.time_s + dt_s
        # Sampling at integration boundaries preserves real acquisition times
        # even if a replay caller advances several seconds at once.
        while self.time_s < end_s - 1e-12:
            if self.time_s >= self._next_slip_s - 1e-12:
                self._left_slip = config.slip_fraction * self._rng.uniform(0.0, 2.0)
                self._right_slip = config.slip_fraction * self._rng.uniform(0.0, 2.0)
                self._next_slip_s += 0.25
            step = min(self.PHYSICS_STEP_S, end_s - self.time_s,
                       self._next_sample_s - self.time_s,
                       self._next_slip_s - self.time_s)
            if step > 1e-12:
                self._integrate(step, left_target, right_target)
                self.time_s += step
            if self.time_s >= self._next_sample_s - 1e-10:
                self._sample_sonar(self._next_sample_s)
                self._next_sample_s += config.sonar_period_s
        self.time_s = end_s
        received: list[RangeReading] = []
        while self._pending and self._pending[0].received_s <= end_s + 1e-10:
            received.append(self._pending.popleft())
        gyro = (self._true_angular + config.gyro_bias_rad_s +
                self._rng.gauss(0.0, config.gyro_noise_rad_s))
        return SensorPacket(
            stamp_s=end_s,
            left_ticks=round(self._left_distance / config.metres_per_tick),
            right_ticks=round(self._right_distance / config.metres_per_tick),
            gyro_z_rad_s=gyro,
            ranges=tuple(received),
        )

    def _integrate(self, dt: float, left_target: float, right_target: float) -> None:
        # Integrate the first-order wheel response analytically. Encoder counts
        # track rotating wheels; when blocked, they may still increase (slip).
        decay = math.exp(-dt / self.MOTOR_TIME_CONSTANT_S)
        old_left, old_right = self._left_speed, self._right_speed
        left_d = left_target * dt + (old_left - left_target) * self.MOTOR_TIME_CONSTANT_S * (1 - decay)
        right_d = right_target * dt + (old_right - right_target) * self.MOTOR_TIME_CONSTANT_S * (1 - decay)
        self._left_speed = left_target + (old_left - left_target) * decay
        self._right_speed = right_target + (old_right - right_target) * decay
        self._left_distance += left_d
        self._right_distance += right_d
        left_ground = left_d * (1 + self.config.wheel_scale_error) * (1 - self._left_slip)
        right_ground = right_d * (1 - self.config.wheel_scale_error) * (1 - self._right_slip)
        d_yaw = (right_ground - left_ground) / self.config.track_width_m
        distance = (left_ground + right_ground) / 2
        pose = self.truth_pose
        middle_yaw = pose.yaw + d_yaw / 2
        candidate = Pose(pose.x + distance * math.cos(middle_yaw),
                         pose.y + distance * math.sin(middle_yaw), wrap(pose.yaw + d_yaw))
        if self.world.collides(candidate, self.config.robot_radius_m):
            if not self._in_contact and abs(distance) > 1e-10:
                self.collision_count += 1
            self._in_contact = True
            # A circular footprint can turn without translating at a wall.
            self.truth_pose = Pose(pose.x, pose.y, candidate.yaw)
        else:
            self.truth_pose = candidate
            self.travel_m += abs(distance)
            if abs(distance) > 1e-10:
                self._in_contact = False
        self._true_angular = d_yaw / dt

    def _sample_sonar(self, stamp_s: float) -> None:
        config = self.config
        pose = self.truth_pose
        c, s = math.cos(pose.yaw), math.sin(pose.yaw)
        for mount in config.mounts:
            x = pose.x + c * mount.x - s * mount.y
            y = pose.y + s * mount.x + c * mount.y
            yaw = pose.yaw + mount.yaw
            nearest = min(
                self.world.raycast(x, y,
                                   yaw + config.sonar_fov_rad * (i / (self.SONAR_RAYS - 1) - 0.5),
                                   config.sonar_max_m)
                for i in range(self.SONAR_RAYS)
            )
            status, measured = "ok", None
            if self._rng.random() < config.sonar_dropout:
                status = "fault"
            elif nearest < config.sonar_min_m:
                # Inside the blind zone the measurement is invalid, not free.
                status = "fault"
            elif nearest >= config.sonar_max_m - 1e-9:
                status = "no_echo"
            else:
                measured = min(config.sonar_max_m,
                               max(config.sonar_min_m,
                                   nearest + self._rng.gauss(0.0, config.sonar_noise_m)))
            self._pending.append(RangeReading(mount.name, stamp_s,
                                               stamp_s + config.sonar_latency_s,
                                               measured, status))
