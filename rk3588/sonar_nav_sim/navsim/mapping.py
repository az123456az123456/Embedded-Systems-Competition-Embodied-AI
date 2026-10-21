"""Conservative cone inverse sensor model; no access to the hidden world."""
from __future__ import annotations

import math
from .model import Config, Pose, RangeReading, SensorMount, wrap


class OccupancyGrid:
    def __init__(self, config: Config):
        self.width = config.width_m
        self.height = config.height_m
        self.resolution = config.resolution_m
        self.cols = math.ceil(self.width / self.resolution)
        self.rows = math.ceil(self.height / self.resolution)
        self.values = [0.0] * (self.cols * self.rows)
        self.seen = [0] * len(self.values)
        self.revision = 0
        self.config = config

    def index(self, col: int, row: int) -> int:
        return row * self.cols + col

    def in_bounds(self, col: int, row: int) -> bool:
        return 0 <= col < self.cols and 0 <= row < self.rows

    def cell(self, x: float, y: float) -> tuple[int, int]:
        return int(math.floor(x / self.resolution)), int(math.floor(y / self.resolution))

    def cell_center(self, col: int, row: int) -> tuple[float, float]:
        return ((col + 0.5) * self.resolution, (row + 0.5) * self.resolution)

    def cell_state(self, col: int, row: int) -> int:
        if not self.in_bounds(col, row):
            return 1
        value = self.values[self.index(col, row)]
        if value >= 0.72:
            return 1
        if value <= -0.30:
            return 0
        return -1

    def known_fraction(self) -> float:
        return sum(v <= -0.30 or v >= 0.72 for v in self.values) / len(self.values)

    def free_area_m2(self) -> float:
        return sum(v <= -0.30 for v in self.values) * self.resolution ** 2

    def _add(self, col: int, row: int, evidence: float) -> None:
        if self.in_bounds(col, row):
            i = self.index(col, row)
            self.values[i] = max(-4.0, min(4.0, self.values[i] + evidence))
            self.seen[i] += 1

    def mark_body_free(self, pose: Pose) -> None:
        # The occupied robot footprint itself is the only initial free-space prior.
        radius = self.config.robot_radius_m + 0.03
        c, r = self.cell(pose.x, pose.y)
        n = math.ceil(radius / self.resolution)
        for row in range(r - n, r + n + 1):
            for col in range(c - n, c + n + 1):
                x, y = self.cell_center(col, row)
                if math.hypot(x - pose.x, y - pose.y) <= radius:
                    self._add(col, row, -1.0)
        self.revision += 1

    def update(self, pose: Pose, mount: SensorMount, reading: RangeReading) -> bool:
        distance = reading.range_m
        if (reading.status != "ok" or distance is None or not math.isfinite(distance)
                or not self.config.sonar_min_m <= distance <= self.config.sonar_max_m):
            # No echo, out of range and communication failure are UNKNOWN, never FREE.
            return False
        co, si = math.cos(pose.yaw), math.sin(pose.yaw)
        sx = pose.x + mount.x * co - mount.y * si
        sy = pose.y + mount.x * si + mount.y * co
        direction = pose.yaw + mount.yaw
        half = self.config.sonar_fov_rad / 2
        band = max(self.resolution * 0.8, 0.08)
        radius = distance + band
        c0, r0 = self.cell(sx - radius, sy - radius)
        c1, r1 = self.cell(sx + radius, sy + radius)
        for row in range(max(0, r0), min(self.rows, r1 + 1)):
            for col in range(max(0, c0), min(self.cols, c1 + 1)):
                x, y = self.cell_center(col, row)
                dx, dy = x - sx, y - sy
                length = math.hypot(dx, dy)
                if length < self.config.sonar_min_m or length > radius:
                    continue
                angle = abs(wrap(math.atan2(dy, dx) - direction))
                if angle > half:
                    continue
                if length < distance - band - 0.04:
                    self._add(col, row, -0.46)
                elif abs(length - distance) < band:
                    # A wide-beam echo locates an ARC, not a precise point. Weak
                    # positive evidence must accumulate across distinct observations.
                    self._add(col, row, 0.16 * (1.0 - 0.35 * angle / max(half, 1e-6)))
        self.revision += 1
        return True

    def planning_cells(self) -> set[tuple[int, int]]:
        clearance = self.config.robot_radius_m + 0.06
        n = math.ceil(clearance / self.resolution)
        offsets = [(dc, dr) for dc in range(-n, n + 1) for dr in range(-n, n + 1)
                   if math.hypot(dc * self.resolution, dr * self.resolution) <= clearance]
        free = {(c, r) for r in range(self.rows) for c in range(self.cols) if self.cell_state(c, r) == 0}
        # Unknown space is also a footprint constraint, not implicitly traversable.
        return {(c, r) for c, r in free if all((c + dc, r + dr) in free for dc, dr in offsets)}
