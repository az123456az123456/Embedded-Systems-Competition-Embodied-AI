"""Shared contracts: metres, seconds, radians; body x forward, y left, z up."""
from __future__ import annotations

from dataclasses import asdict, dataclass, field
import math
from typing import Protocol


def wrap(angle: float) -> float:
    return (angle + math.pi) % (2 * math.pi) - math.pi


@dataclass
class Pose:
    x: float = 1.1
    y: float = 1.1
    yaw: float = 0.0


@dataclass(frozen=True)
class SensorMount:
    name: str
    x: float
    y: float
    yaw: float


@dataclass
class Config:
    # Assumed geometry: replace these with measured hardware values later.
    width_m: float = 7.2
    height_m: float = 5.6
    resolution_m: float = 0.10
    wheel_radius_m: float = 0.035
    track_width_m: float = 0.26
    ticks_per_revolution: int = 1440  # Includes quadrature multiplication and gearbox.
    robot_radius_m: float = 0.19
    max_speed_m_s: float = 0.22
    max_turn_rad_s: float = 0.65
    sonar_period_s: float = 0.10
    sonar_latency_s: float = 0.10
    sonar_fov_rad: float = math.radians(60)
    sonar_min_m: float = 0.03
    sonar_max_m: float = 4.5
    sonar_noise_m: float = 0.008
    sonar_dropout: float = 0.012
    gyro_bias_rad_s: float = math.radians(0.10)
    gyro_noise_rad_s: float = math.radians(0.12)
    wheel_scale_error: float = 0.004
    slip_fraction: float = 0.012
    stale_after_s: float = 0.65
    stop_distance_m: float = 0.32  # From the front sensor, not from the centre.
    seed: int = 7

    @property
    def mounts(self) -> tuple[SensorMount, ...]:
        return (
            SensorMount("front", 0.17, 0.0, 0.0),
            SensorMount("left", 0.0, 0.16, math.pi / 2),
            SensorMount("right", 0.0, -0.16, -math.pi / 2),
        )

    @property
    def metres_per_tick(self) -> float:
        return 2 * math.pi * self.wheel_radius_m / self.ticks_per_revolution

    def validate(self) -> None:
        for name, value in asdict(self).items():
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
                raise ValueError(f"{name} must be a finite number")
        if type(self.seed) is not int or type(self.ticks_per_revolution) is not int:
            raise ValueError("seed and ticks_per_revolution must be integers")
        positive = ("width_m", "height_m", "resolution_m", "wheel_radius_m",
                    "track_width_m", "ticks_per_revolution", "robot_radius_m",
                    "max_speed_m_s", "max_turn_rad_s", "sonar_period_s",
                    "sonar_max_m", "stale_after_s")
        for name in positive:
            if not math.isfinite(getattr(self, name)) or getattr(self, name) <= 0:
                raise ValueError(f"{name} must be finite and positive")
        if not 0 <= self.sonar_dropout <= 1 or not 0 < self.sonar_fov_rad <= math.pi:
            raise ValueError("Invalid sonar dropout or field of view")
        if self.sonar_min_m < 0 or self.sonar_min_m >= self.sonar_max_m:
            raise ValueError("Invalid sonar measurement limits")
        if self.sonar_latency_s < 0 or self.sonar_noise_m < 0:
            raise ValueError("Sensor latency/noise must be non-negative")
        if self.gyro_noise_rad_s < 0 or not 0 <= self.slip_fraction < 0.5 or not 0 <= self.wheel_scale_error < 0.2:
            raise ValueError("Invalid gyro noise, slip or wheel-scale error")
        if not self.sonar_min_m < self.stop_distance_m < self.sonar_max_m:
            raise ValueError("Stopping distance must be within the sonar measurement limits")
        if self.width_m / self.resolution_m * self.height_m / self.resolution_m > 200000:
            raise ValueError("Grid too large; increase resolution_m (limit 200000 cells)")


@dataclass(frozen=True)
class RangeReading:
    name: str
    stamp_s: float  # Acquisition time; mapping uses pose history at this time.
    received_s: float
    range_m: float | None
    status: str = "ok"  # ok / no_echo / fault; no_echo does not imply free space.


@dataclass(frozen=True)
class SensorPacket:
    stamp_s: float
    left_ticks: int
    right_ticks: int
    gyro_z_rad_s: float
    ranges: tuple[RangeReading, ...] = ()  # Only newly delivered sonar samples.
    encoder_valid: bool = True
    imu_valid: bool = True


@dataclass(frozen=True)
class Command:
    linear_m_s: float = 0.0
    angular_rad_s: float = 0.0


class RobotIO(Protocol):
    """Replace this boundary with an RK3588 telemetry/motor adapter.

    No map or simulator truth is available to the navigation algorithm.
    A real motor adapter must implement its own command timeout/watchdog.
    """
    def advance(self, dt_s: float, command: Command) -> SensorPacket: ...


def packet_dict(packet: SensorPacket) -> dict:
    return asdict(packet)


def packet_from_dict(data: dict) -> SensorPacket:
    values = dict(data)
    values["ranges"] = tuple(RangeReading(**r) for r in values.get("ranges", ()))
    return SensorPacket(**values)
