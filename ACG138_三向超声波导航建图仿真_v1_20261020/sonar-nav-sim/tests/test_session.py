"""Integration checks for recorded sensors, estimation, mapping and CLI replay."""
from contextlib import redirect_stdout, redirect_stderr
from dataclasses import asdict
import io
import json
from pathlib import Path
import tempfile
import unittest

from navsim.adapters import JsonlReplay
from navsim.model import Command, Config, Pose
from navsim.navigator import Navigator
from navsim.session import Session
import run_sim


class SessionReplayTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.root = Path(self.tmp.name)

    def tearDown(self):
        self.tmp.cleanup()

    def record_trip(self, path):
        session = Session(Config(seed=13), "open")
        session.record(path)
        session.start()
        # Include calibration, a full scan, forward motion, pause, and resume.
        # Variable step durations make a replay's accidental fixed-dt assumption visible.
        for i in range(480):
            if i == 390:
                session.pause()
            if i == 410:
                session.start()
            session.step(.04 if i % 2 == 0 else .06)
        return session

    def test_sensor_replay_reproduces_pose_map_trajectory_and_bias_exactly(self):
        path = self.root / "trip.jsonl"
        session = self.record_trip(path)
        try:
            source = JsonlReplay(path)
            try:
                navigator = Navigator(source.config, Pose(**source.metadata["initial_pose"]))
                frames = 0
                while not source.finished:
                    packet = source.advance(.1, Command(100, -100))
                    navigator.command = source.recorded_command
                    navigator.ingest(packet)
                    self.assertEqual(asdict(navigator.pose), asdict(source.recorded_pose))
                    frames += 1
                self.assertEqual(frames, session.frames)
                self.assertEqual(navigator.localizer.gyro_bias, session.navigator.localizer.gyro_bias)
                self.assertEqual(navigator.grid.values, session.navigator.grid.values)
                self.assertEqual(navigator.grid.seen, session.navigator.grid.seen)
                self.assertEqual(navigator.trajectory, session.navigator.trajectory)
                self.assertGreater(navigator.localizer.distance_m, .1)
            finally:
                source.close()
        finally:
            session.close()

    def test_cli_replay_consumes_all_frames_and_exports_matching_map(self):
        path = self.root / "trip.jsonl"
        session = self.record_trip(path)
        try:
            expected_pose = asdict(session.navigator.pose)
            expected_cells = list(session.navigator.grid.values)
            expected_frames = session.frames
        finally:
            session.close()
        output = io.StringIO()
        errors = io.StringIO()
        target = self.root / "replayed"
        with redirect_stdout(output), redirect_stderr(errors):
            code = run_sim.main(["--replay", str(path), "--export", str(target)])
        self.assertEqual(code, 0, errors.getvalue())
        result = json.loads(output.getvalue())
        self.assertEqual(result["frames"], expected_frames)
        self.assertEqual(result["estimated_pose"], expected_pose)
        self.assertEqual(json.loads((target / "map.json").read_text())["log_odds"], expected_cells)
        self.assertEqual((target / "telemetry.jsonl").read_bytes(), path.read_bytes())

    def test_session_reset_restarts_time_and_log_from_first_sample(self):
        session = Session(Config(seed=13), "open")
        try:
            session.start()
            for _ in range(8):
                session.step(.05)
            session.reset()
            self.assertFalse(session.running)
            self.assertEqual(session.frames, 0)
            session.step(.05)
            with JsonlReplay(session.log_path) as replay:
                first = replay.advance(.05, Command())
                self.assertAlmostEqual(first.stamp_s, .05)
                self.assertTrue(replay.finished)
        finally:
            session.close()

    def test_replay_export_into_source_directory_keeps_telemetry_unchanged(self):
        path = self.root / "telemetry.jsonl"
        session = Session(Config(seed=13), "open")
        session.record(path)
        for _ in range(5):
            session.step(.05)
        session.close()
        original = path.read_bytes()
        with redirect_stdout(io.StringIO()), redirect_stderr(io.StringIO()):
            code = run_sim.main(["--replay", str(path), "--export", str(self.root)])
        self.assertEqual(code, 0)
        self.assertEqual(path.read_bytes(), original)

    def test_mid_trip_record_switch_is_rejected_without_losing_existing_log(self):
        session = Session()
        try:
            session.step(.05)
            original = session.log_path
            with self.assertRaises(ValueError):
                session.record(self.root / "partial.jsonl")
            self.assertEqual(session.log_path, original)
            session.step(.05)
            self.assertTrue(original.exists())
        finally:
            session.close()

    def test_replay_export_does_not_overwrite_source_named_map_json(self):
        path = self.root / "map.json"
        session = Session()
        session.record(path)
        session.step(.05)
        session.close()
        original = path.read_bytes()
        with redirect_stdout(io.StringIO()), redirect_stderr(io.StringIO()):
            code = run_sim.main(["--replay", str(path), "--export", str(self.root)])
        self.assertEqual(code, 2)
        self.assertEqual(path.read_bytes(), original)


if __name__ == "__main__":
    unittest.main()
