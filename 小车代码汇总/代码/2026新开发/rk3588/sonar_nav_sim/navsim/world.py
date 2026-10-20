"""Hidden ground-truth geometry used only by the simulator and UI evaluator."""
from __future__ import annotations

from dataclasses import dataclass
import math

from .model import Config, Pose


@dataclass
class World:
    width: float
    height: float
    rectangles: list[tuple[float, float, float, float]]
    start_pose: Pose
    name: str = "custom"

    def __post_init__(self) -> None:
        if not all(math.isfinite(v) and v > 0 for v in (self.width, self.height)):
            raise ValueError("World dimensions must be positive")
        for x, y, width, height in self.rectangles:
            if not all(math.isfinite(v) for v in (x, y, width, height)):
                raise ValueError("Obstacle coordinates must be finite")
            if width <= 0 or height <= 0:
                raise ValueError("Obstacle dimensions must be positive")

    def raycast(self, x: float, y: float, angle: float, max_range: float) -> float:
        """Return first boundary/rectangle hit, or max_range if there is none.

        Outer boundaries are implicit and therefore need no duplicate rectangles.
        A ray originating in an obstacle or outside the room reports distance 0.
        """
        if max_range < 0 or not math.isfinite(max_range):
            raise ValueError("max_range must be finite and non-negative")
        if not 0 <= x <= self.width or not 0 <= y <= self.height:
            return 0.0
        dx, dy = math.cos(angle), math.sin(angle)
        distance = max_range
        if dx > 1e-12:
            distance = min(distance, (self.width - x) / dx)
        elif dx < -1e-12:
            distance = min(distance, -x / dx)
        if dy > 1e-12:
            distance = min(distance, (self.height - y) / dy)
        elif dy < -1e-12:
            distance = min(distance, -y / dy)
        for rx, ry, width, height in self.rectangles:
            near, far = 0.0, distance
            for origin, direction, low, high in (
                (x, dx, rx, rx + width), (y, dy, ry, ry + height)
            ):
                if abs(direction) < 1e-12:
                    if origin < low or origin > high:
                        near = math.inf
                        break
                else:
                    a, b = (low - origin) / direction, (high - origin) / direction
                    if a > b:
                        a, b = b, a
                    near, far = max(near, a), min(far, b)
                    if near > far:
                        break
            if near <= far:
                distance = min(distance, near)
        return max(0.0, distance)

    def collides(self, pose: Pose, radius: float) -> bool:
        if (pose.x - radius < 0 or pose.y - radius < 0 or
                pose.x + radius > self.width or pose.y + radius > self.height):
            return True
        for x, y, width, height in self.rectangles:
            nearest_x = min(max(pose.x, x), x + width)
            nearest_y = min(max(pose.y, y), y + height)
            if (pose.x - nearest_x) ** 2 + (pose.y - nearest_y) ** 2 < radius ** 2:
                return True
        return False


def make_world(name: str = "apartment", config: Config | None = None) -> World:
    """Create reproducible layouts with doorways wide enough for the robot.

    Layout coordinates scale with configured room size. Real geometry remains
    private: navigation receives measurements, never this object.
    """
    config = config or Config()
    w, h = config.width_m, config.height_m
    sx, sy = w / 7.2, h / 5.6
    layouts = {
        "apartment": [
            (3.45, 0.00, 0.16, 1.95),
            (3.45, 3.05, 0.16, 2.55),
            (3.61, 3.72, 1.00, 0.16),
            (5.70, 3.72, 1.50, 0.16),
            (0.85, 3.45, 1.30, 0.65),
            (5.35, 0.80, 0.90, 1.05),
            (4.35, 4.45, 0.75, 0.55),
        ],
        "open": [
            (2.70, 2.25, 0.75, 0.75),
            (5.20, 3.80, 0.75, 0.55),
        ],
        "corridor": [
            (0.00, 1.95, 2.70, 0.16),
            (3.85, 1.95, 3.35, 0.16),
            (0.00, 3.45, 4.85, 0.16),
            (5.95, 3.45, 1.25, 0.16),
            (1.75, 4.10, 0.60, 0.75),
            (5.55, 0.65, 0.80, 0.60),
        ],
    }
    if name not in layouts:
        raise ValueError(f"Unknown scene {name!r}; choose apartment, open or corridor")
    rectangles = [(x * sx, y * sy, rw * sx, rh * sy)
                  for x, y, rw, rh in layouts[name]]
    world = World(w, h, rectangles, Pose(1.1 * sx, 1.1 * sy, 0.0), name)
    if world.collides(world.start_pose, config.robot_radius_m):
        raise ValueError("Room is too small for the configured robot and scene")
    return world
