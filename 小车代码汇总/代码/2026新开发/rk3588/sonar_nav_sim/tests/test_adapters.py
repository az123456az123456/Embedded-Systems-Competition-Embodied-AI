"""Boundary tests: replay determinism, invalid telemetry, and fail-stop commands."""
from dataclasses import asdict, replace
import json
import math
from pathlib import Path
import tempfile
import threading
import unittest

from navsim.adapters import JsonlRecorder, JsonlReplay, QueueHardwareIO
from navsim.model import Command, Config, Pose, RangeReading, SensorPacket, packet_dict


def packet(stamp=1.0, ticks=100, *, ranges=True, **kwargs):
    readings = tuple(RangeReading(name, stamp - .1, stamp, 1.2)
                     for name in ("front", "left", "right")) if ranges else ()
    return SensorPacket(stamp, ticks, ticks + 3, .02, readings, **kwargs)


class Clock:
    def __init__(self):
        self.now = 100.0

    def __call__(self):
        return self.now


class ReplayTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.path = Path(self.tmp.name) / "trip.jsonl"

    def tearDown(self):
        self.tmp.cleanup()

    def write_lines(self, packets):
        lines = [{"kind": "frame", "version": 1, "packet": packet_dict(p),
                  "command": asdict(Command()), "estimated_pose": asdict(Pose())}
                 for p in packets]
        self.path.write_text("\n".join(json.dumps(line) for line in lines) + "\n", encoding="utf-8")

    def test_round_trip_preserves_samples_and_ignores_new_commands(self):
        frames = [packet(1.0), packet(1.037, 105), packet(1.2, 120, ranges=False)]
        metadata = {"config": asdict(Config()), "initial_pose": asdict(Pose()), "scenario": "room"}
        with JsonlRecorder(self.path, metadata=metadata) as recorder:
            for item in frames:
                recorder.write(item, Command(.1, .2), Pose(1.2, 1.3, .4))
        replay = JsonlReplay(self.path)
        self.assertEqual(replay.metadata, metadata)
        for i, expected in enumerate(frames):
            self.assertEqual(replay.advance(.04, Command(-99, 100)), expected)
            self.assertEqual(replay.finished, i == len(frames) - 1)
            self.assertEqual(replay.recorded_command, Command(.1, .2))
        self.assertEqual(replay.advance(.04, Command()), frames[-1])

    def test_no_metadata_compatible_and_repeatable(self):
        frames = [packet(), packet(1.1, 130)]
        self.write_lines(frames)
        outputs = []
        for _ in range(2):
            replay = JsonlReplay(self.path)
            self.assertEqual(replay.metadata, {})
            outputs.append([replay.advance(.01, Command()) for _ in frames])
        self.assertEqual(outputs[0], outputs[1])

    def test_record_rejects_invalid_sensor_and_time(self):
        with JsonlRecorder(self.path) as recorder:
            for bad in (replace(packet(), stamp_s=math.nan),
                        replace(packet(), gyro_z_rad_s=math.inf),
                        replace(packet(), left_ticks=1.5),
                        replace(packet(), encoder_valid="yes"),
                        replace(packet(), ranges=(RangeReading("front", .9, 1., None),)),
                        replace(packet(), ranges=(RangeReading("front", .9, 1., 20.),)),
                        replace(packet(), ranges=(RangeReading("front", 1.1, 1., 1.),))):
                with self.subTest(packet=bad), self.assertRaises(ValueError):
                    recorder.write(bad, Command(), Pose())
            recorder.write(packet(), Command(), Pose())
            with self.assertRaises(ValueError):
                recorder.write(packet(), Command(), Pose())

    def test_replay_rejects_malformed_or_nonmonotonic_records(self):
        self.path.write_text('{"kind":"frame","version":1,"packet":NaN}\n')
        with self.assertRaisesRegex(ValueError, "Non-finite"):
            JsonlReplay(self.path)
        self.write_lines([packet(), packet(.9)])
        replay = JsonlReplay(self.path)
        with self.assertRaisesRegex(ValueError, "strictly increase"):
            replay.advance(.04, Command())
        replay.close()
        self.path.write_text("\n")
        with self.assertRaisesRegex(ValueError, "no frame records"):
            JsonlReplay(self.path)

    def test_metadata_config_controls_range_limits(self):
        config = replace(Config(), sonar_max_m=6.)
        p = replace(packet(), ranges=(RangeReading("front", .9, 1., 5.),))
        with JsonlRecorder(self.path, config, metadata={"config": asdict(config)}) as recorder:
            recorder.write(p, Command(), Pose())
        self.assertEqual(JsonlReplay(self.path).advance(.04, Command()), p)


class HardwareQueueTests(unittest.TestCase):
    def setUp(self):
        self.clock = Clock()
        self.sent = []
        self.io = QueueHardwareIO(command_sink=self.sent.append, clock=self.clock, max_queue=3)

    def test_waiting_and_no_new_packet_stop_immediately(self):
        result = self.io.advance(.04, Command(.1, .2))
        self.assertFalse(result.encoder_valid)
        self.assertEqual(self.sent[-1], Command())
        self.io.submit(packet())
        self.io.advance(.04, Command(.1, .2))
        self.assertEqual(self.sent[-1], Command(.1, .2))
        result = self.io.advance(.04, Command(.1, .2))
        self.assertFalse(result.encoder_valid)
        self.assertFalse(result.imu_valid)
        self.assertEqual(result.ranges, ())
        self.assertEqual(self.sent[-1], Command())
        self.assertEqual(self.io.stop_reason, "no_new_telemetry")

    def test_command_limits_and_cumulative_ticks_survive_queue_overflow(self):
        for i in range(10):
            self.io.submit(packet(1. + i * .1, i * 100))
        self.assertEqual(self.io.queued_packets, 3)
        self.assertEqual(self.io.dropped_packets, 7)
        result = self.io.advance(.04, Command(10., -10.))
        self.assertEqual(result.left_ticks, 900)
        self.assertEqual(self.sent[-1], Command(self.io.config.max_speed_m_s, -self.io.config.max_turn_rad_s))

    def test_receipt_timeout_stops_and_marks_odometry_invalid(self):
        self.io.submit(packet())
        self.clock.now += self.io.config.stale_after_s + .01
        result = self.io.advance(.04, Command(.1, 0))
        self.assertFalse(result.encoder_valid)
        self.assertEqual(self.io.stop_reason, "telemetry_stale")
        self.assertEqual(self.sent[-1], Command())

    def test_old_acquisition_cannot_be_made_fresh_by_late_arrival(self):
        self.io.submit(packet(), sampled_monotonic_s=self.clock.now - 1.)
        result = self.io.advance(.04, Command(.1, 0))
        self.assertFalse(result.encoder_valid)
        self.assertEqual(self.io.stop_reason, "telemetry_stale")
        self.assertEqual(self.sent[-1], Command())

    def test_nonfinite_control_attempt_sends_stop(self):
        self.io.submit(packet())
        self.io.advance(.04, Command(.1, 0))
        with self.assertRaises(ValueError):
            self.io.advance(.04, Command(math.nan, 0))
        self.assertEqual(self.sent[-1], Command())

    def test_sonar_acquisition_age_matters_even_with_new_receipt(self):
        self.io.submit(replace(packet(), ranges=(RangeReading("front", .1, 1., 1.2),)))
        self.io.advance(.04, Command(.1, 0))
        self.assertEqual(self.io.stop_reason, "sonar_front_stale")
        self.assertEqual(self.sent[-1], Command())

    def test_invalid_or_missing_sonar_and_invalid_imu_stop(self):
        for p in (packet(ranges=False),
                  replace(packet(), ranges=(RangeReading("front", .9, 1., None, "no_echo"),)),
                  packet(imu_valid=False)):
            io = QueueHardwareIO(command_sink=self.sent.append, clock=self.clock)
            io.submit(p)
            io.advance(.04, Command(.1, 0))
            self.assertTrue(io.stop_reason)
            self.assertEqual(self.sent[-1], Command())

    def test_out_of_order_or_duplicate_sample_rejected(self):
        self.io.submit(packet())
        for p in (packet(), packet(.9)):
            with self.assertRaises(ValueError):
                self.io.submit(p)

    def test_receiver_and_navigation_threads_share_bounded_queue(self):
        errors = []
        done = threading.Event()

        def receiver():
            try:
                for i in range(1000):
                    self.io.submit(packet(1. + i * .01, i))
            except Exception as exc:
                errors.append(exc)
            finally:
                done.set()

        thread = threading.Thread(target=receiver)
        thread.start()
        while not done.is_set():
            self.io.advance(.04, Command(.1, 0))
            self.assertLessEqual(self.io.queued_packets, 3)
        thread.join()
        result = self.io.advance(.04, Command())
        self.assertEqual(result.left_ticks, 999)
        self.assertEqual(errors, [])

    def test_sink_failure_attempts_stop_and_close_stops(self):
        attempts = []

        def sink(command):
            attempts.append(command)
            if command != Command():
                raise OSError("disconnected")

        io = QueueHardwareIO(command_sink=sink, clock=self.clock)
        io.submit(packet())
        with self.assertRaises(OSError):
            io.advance(.04, Command(.1, 0))
        self.assertEqual(attempts[-1], Command())
        self.assertEqual(io.stop_reason, "command_sink_error")
        io.close()
        self.assertTrue(io.closed)
        self.assertEqual(attempts[-1], Command())
        with self.assertRaises(RuntimeError):
            io.submit(packet(2.))


if __name__ == "__main__":
    unittest.main()
