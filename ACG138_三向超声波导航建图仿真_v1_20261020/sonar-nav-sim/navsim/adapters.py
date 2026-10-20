"""Record/replay and a transport-neutral hardware boundary.

Nothing in this module opens a serial port, network socket, or motor device.
The transport owner must provide a command sink with a device-side watchdog.
"""
from __future__ import annotations

from collections import deque
from dataclasses import asdict, replace
import json
import math
from pathlib import Path
import threading
import time
from typing import Callable, TextIO

from .model import Command, Config, Pose, RangeReading, SensorPacket, packet_dict, packet_from_dict


def _finite(value: object, name: str, *, nonnegative: bool = False) -> None:
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError(f"{name} must be a finite number")
    if nonnegative and value < 0:
        raise ValueError(f"{name} must be non-negative")


def _validate_packet(packet: SensorPacket, config: Config) -> None:
    if not isinstance(packet, SensorPacket):
        raise ValueError("Expected SensorPacket")
    _finite(packet.stamp_s, "packet.stamp_s", nonnegative=True)
    _finite(packet.gyro_z_rad_s, "gyro_z_rad_s")
    for name in ("left_ticks", "right_ticks"):
        if type(getattr(packet, name)) is not int:
            raise ValueError(f"{name} must be an integer cumulative count")
    for name in ("encoder_valid", "imu_valid"):
        if type(getattr(packet, name)) is not bool:
            raise ValueError(f"{name} must be boolean")
    for reading in packet.ranges:
        if not isinstance(reading, RangeReading) or reading.name not in ("front", "left", "right"):
            raise ValueError("Range sensor must be front, left, or right")
        _finite(reading.stamp_s, "range.stamp_s", nonnegative=True)
        _finite(reading.received_s, "range.received_s", nonnegative=True)
        if not reading.stamp_s <= reading.received_s <= packet.stamp_s + 1e-9:
            raise ValueError("Range times must satisfy acquisition <= receipt <= packet time")
        if reading.status not in ("ok", "no_echo", "fault"):
            raise ValueError("Range status must be ok, no_echo, or fault")
        if reading.status == "ok":
            _finite(reading.range_m, "range_m")
            if not config.sonar_min_m <= reading.range_m <= config.sonar_max_m:
                raise ValueError("Range outside configured measurement limits")
        elif reading.range_m is not None:
            raise ValueError("no_echo/fault readings must have null range_m")


def _validate_command(command: Command) -> None:
    _finite(command.linear_m_s, "command.linear_m_s")
    _finite(command.angular_rad_s, "command.angular_rad_s")


def _validate_pose(pose: Pose) -> None:
    for name in ("x", "y", "yaw"):
        _finite(getattr(pose, name), f"estimated_pose.{name}")


def _reject_json_constant(value: str) -> None:
    raise ValueError(f"Non-finite JSON number: {value}")


def _validate_metadata(metadata: dict) -> None:
    # Validate all nested numbers, not only the subset understood here.
    json.dumps(metadata, allow_nan=False)
    if "config" in metadata:
        config = Config(**metadata["config"])
        config.validate()
    if "initial_pose" in metadata:
        _validate_pose(Pose(**metadata["initial_pose"]))
    if "scenario" in metadata and not isinstance(metadata["scenario"], str):
        raise ValueError("metadata.scenario must be a string")


class JsonlRecorder:
    """Append one complete, replayable sensor/control frame per JSON line."""

    def __init__(self, path: str | Path, config: Config | None = None, *, metadata: dict | None = None):
        self.config = config or Config()
        self.config.validate()
        self.path = Path(path)
        metadata_line = None
        if metadata is not None:
            if not isinstance(metadata, dict) or any(k in metadata for k in ("kind", "version")):
                raise ValueError("metadata must be a dict without reserved kind/version keys")
            _validate_metadata(metadata)
            metadata_line = json.dumps({"kind": "metadata", "version": 1, **metadata},
                                       ensure_ascii=False, allow_nan=False)
        self._stream = self.path.open("w", encoding="utf-8")
        self._last_stamp: float | None = None
        if metadata_line is not None:
            self._stream.write(metadata_line + "\n")
            self._stream.flush()

    def write(self, packet: SensorPacket, command: Command, estimated_pose: Pose) -> None:
        _validate_packet(packet, self.config)
        _validate_command(command)
        _validate_pose(estimated_pose)
        if self._last_stamp is not None and packet.stamp_s <= self._last_stamp:
            raise ValueError("Recorded packet timestamps must strictly increase")
        record = {"kind": "frame", "version": 1, "packet": packet_dict(packet),
                  "command": asdict(command), "estimated_pose": asdict(estimated_pose)}
        self._stream.write(json.dumps(record, ensure_ascii=False, allow_nan=False) + "\n")
        self._stream.flush()
        self._last_stamp = packet.stamp_s

    def close(self) -> None:
        self._stream.close()

    def flush(self) -> None:
        self._stream.flush()

    def __enter__(self) -> JsonlRecorder:
        return self

    def __exit__(self, *_exc: object) -> None:
        self.close()


class JsonlReplay:
    """Read each original sensor frame exactly once, independent of commands.

    advance() returns one logged frame per call; its dt_s is validated but does
    not change recorded timestamps. Consumers use differences between packet
    timestamps for estimation. At EOF, finished is true and further calls return
    the last frame. Never connect this object to a motor command sink.
    """

    def __init__(self, path: str | Path, config: Config | None = None):
        self.config = config or Config()
        self._explicit_config = config is not None
        self.config.validate()
        self.path = Path(path)
        self._stream: TextIO = self.path.open(encoding="utf-8")
        self._line = 0
        self._read_stamp: float | None = None
        self.metadata: dict = {}
        self._seen_record = False
        self._last_packet: SensorPacket | None = None
        self.recorded_command = Command()
        self.recorded_pose = Pose()
        self.finished = False
        try:
            self._next = self._read_next()
            if self._next is None:
                raise ValueError(f"{self.path}: no frame records")
        except Exception:
            self.close()
            raise

    def _read_next(self) -> tuple[SensorPacket, Command, Pose] | None:
        for line in self._stream:
            self._line += 1
            if not line.strip():
                continue
            try:
                record = json.loads(line, parse_constant=_reject_json_constant)
                if isinstance(record, dict) and record.get("kind") == "metadata":
                    if self._seen_record or record.get("version") != 1:
                        raise ValueError("Metadata is allowed only as the first record")
                    self.metadata = {k: v for k, v in record.items() if k not in ("kind", "version")}
                    _validate_metadata(self.metadata)
                    if "config" in self.metadata and not self._explicit_config:
                        self.config = Config(**self.metadata["config"])
                        self.config.validate()
                    self._seen_record = True
                    continue
                if not isinstance(record, dict) or record.get("kind") != "frame" or record.get("version") != 1:
                    raise ValueError("Expected a version 1 frame record")
                self._seen_record = True
                packet = packet_from_dict(record["packet"])
                command = Command(**record["command"])
                pose = Pose(**record["estimated_pose"])
                _validate_packet(packet, self.config)
                _validate_command(command)
                _validate_pose(pose)
                if self._read_stamp is not None and packet.stamp_s <= self._read_stamp:
                    raise ValueError("Packet timestamps must strictly increase")
                self._read_stamp = packet.stamp_s
                return packet, command, pose
            except (KeyError, TypeError, ValueError, AttributeError) as exc:
                raise ValueError(f"{self.path}:{self._line}: {exc}") from exc
        return None

    def advance(self, dt_s: float, command: Command) -> SensorPacket:
        _finite(dt_s, "dt_s")
        if dt_s <= 0:
            raise ValueError("dt_s must be positive")
        _validate_command(command)
        if self.finished:
            assert self._last_packet is not None
            return self._last_packet
        assert self._next is not None
        self._last_packet, self.recorded_command, self.recorded_pose = self._next
        self._next = self._read_next()
        if self._next is None:
            self.finished = True
            self.close()
        return self._last_packet

    def close(self) -> None:
        self._stream.close()


    def __enter__(self) -> JsonlReplay:
        return self

    def __exit__(self, *_exc: object) -> None:
        self.close()


class QueueHardwareIO:
    """Bounded, thread-safe telemetry queue plus a deliberately explicit sink.

    submit() is called by a future transport receiver; advance() is called by
    the navigation loop. Freshness uses monotonic receipt time AND the sensor
    sample's age when received. All clocks inside SensorPacket share one domain.
    The hardware sample clock need not equal the PC monotonic clock.

    No background timer is created. advance() must keep running for the software
    timeout to send a stop. An independent motor/FPGA watchdog is mandatory.
    """

    def __init__(self, config: Config | None = None,
                 command_sink: Callable[[Command], None] | None = None, *,
                 max_queue: int = 128, clock: Callable[[], float] = time.monotonic,
                 require_ranges: bool = True):
        self.config = config or Config()
        self.config.validate()
        if type(max_queue) is not int or max_queue < 1:
            raise ValueError("max_queue must be a positive integer")
        self._sink = command_sink
        self._clock = clock
        self._require_ranges = require_ranges
        self._max_queue = max_queue
        self._queue: deque[tuple[SensorPacket, float, float]] = deque()
        self._lock = threading.Lock()
        self._advance_lock = threading.Lock()
        self._last_submit_stamp: float | None = None
        self._last_packet: SensorPacket | None = None
        self._last_receipt: float | None = None
        self._last_sample_host: float | None = None
        self._latest_ranges: dict[str, RangeReading] = {}
        self.dropped_packets = 0
        self.last_command = Command()
        self.stop_reason = "waiting_for_telemetry"
        self.closed = False

    @property
    def queued_packets(self) -> int:
        with self._lock:
            return len(self._queue)

    def submit(self, packet: SensorPacket, *, sampled_monotonic_s: float | None = None) -> None:
        """Validate and queue telemetry. Reject repeats/reordering and overflow old data."""
        _validate_packet(packet, self.config)
        now = self._clock()
        _finite(now, "receipt clock")
        sample_host = now if sampled_monotonic_s is None else sampled_monotonic_s
        _finite(sample_host, "sampled_monotonic_s")
        if sample_host > now:
            raise ValueError("Sample acquisition cannot be in the future")
        with self._lock:
            if self.closed:
                raise RuntimeError("Hardware adapter is closed")
            if self._last_submit_stamp is not None and packet.stamp_s <= self._last_submit_stamp:
                raise ValueError("Telemetry timestamps must strictly increase; reset adapter after device reboot")
            if len(self._queue) == self._max_queue:
                self._queue.popleft()
                self.dropped_packets += 1
            self._queue.append((packet, now, sample_host))
            self._last_submit_stamp = packet.stamp_s

    def _send(self, command: Command) -> None:
        try:
            if self._sink is not None:
                self._sink(command)
        except Exception:
            self.last_command = Command()
            self.stop_reason = "command_sink_error"
            # Best-effort stop; failure must still reach the caller.
            try:
                self._sink(Command())
            except Exception:
                pass
            raise
        self.last_command = command

    def advance(self, dt_s: float, command: Command) -> SensorPacket:
        with self._advance_lock:
            try:
                _finite(dt_s, "dt_s")
                if dt_s <= 0:
                    raise ValueError("dt_s must be positive")
                _validate_command(command)
            except (ValueError, AttributeError):
                self.stop_reason = "invalid_control_input"
                self._send(Command())
                raise
            now = self._clock()
            _finite(now, "receipt clock")
            with self._lock:
                if self.closed:
                    raise RuntimeError("Hardware adapter is closed")
                incoming = list(self._queue)
                self._queue.clear()
            ranges: list[RangeReading] = []
            for packet, received, sample_host in incoming:
                self._last_packet = packet
                self._last_receipt = received
                self._last_sample_host = sample_host
                ranges.extend(packet.ranges)
                for reading in packet.ranges:
                    previous = self._latest_ranges.get(reading.name)
                    if previous is None or reading.stamp_s > previous.stamp_s:
                        self._latest_ranges[reading.name] = reading
            packet = self._last_packet
            reason = ""
            receipt_age = math.inf if self._last_receipt is None else now - self._last_receipt
            sample_age = math.inf if self._last_sample_host is None else now - self._last_sample_host
            if packet is None:
                reason = "waiting_for_telemetry"
                packet = SensorPacket(0.0, 0, 0, 0.0, encoder_valid=False, imu_valid=False)
            elif min(receipt_age, sample_age) < 0 or max(receipt_age, sample_age) > self.config.stale_after_s:
                reason = "telemetry_stale"
            elif not incoming:
                reason = "no_new_telemetry"
            elif not packet.encoder_valid or not packet.imu_valid:
                reason = "invalid_encoder_or_imu"
            elif self._require_ranges:
                for name in ("front", "left", "right"):
                    reading = self._latest_ranges.get(name)
                    if reading is None or reading.status != "ok":
                        reason = f"sonar_{name}_unavailable"
                        break
                    age = packet.stamp_s - reading.stamp_s + sample_age
                    if age > self.config.stale_after_s:
                        reason = f"sonar_{name}_stale"
                        break
            if reason:
                limited = Command()
            else:
                limited = Command(
                    max(-self.config.max_speed_m_s, min(self.config.max_speed_m_s, command.linear_m_s)),
                    max(-self.config.max_turn_rad_s, min(self.config.max_turn_rad_s, command.angular_rad_s)))
            self.stop_reason = reason
            self._send(limited)
            if reason in ("telemetry_stale", "waiting_for_telemetry", "no_new_telemetry"):
                return replace(packet, ranges=(), encoder_valid=False, imu_valid=False)
            return replace(packet, ranges=tuple(ranges))

    def close(self) -> None:
        with self._advance_lock:
            with self._lock:
                if self.closed:
                    return
                self.closed = True
                self._queue.clear()
            self.stop_reason = "closed"
            self._send(Command())
