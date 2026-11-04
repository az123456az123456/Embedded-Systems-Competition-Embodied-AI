"""Application orchestration and exports; algorithms remain independent of GUI/IO."""
from __future__ import annotations
from dataclasses import asdict
from pathlib import Path
import tempfile

from .model import Config, Command
from .exports import export_navigation
from .navigator import Navigator
from .simulator import Simulator
from .world import make_world


class Session:
    def __init__(self, config: Config | None = None, scenario: str = "apartment"):
        self.config = config or Config()
        self.config.validate()
        self.scenario = scenario
        self.recorder = None
        self._temporary = None
        self.running = False
        self.reset(scenario)

    def reset(self, scenario: str | None = None) -> None:
        if self.recorder:
            self.recorder.close()
            self.recorder = None
        if self._temporary:
            self._temporary.cleanup()
        if scenario:
            self.scenario = scenario
        self.world = make_world(self.scenario, self.config)
        self.simulator = Simulator(self.config, self.world)
        self.navigator = Navigator(self.config, self.world.start_pose)
        self.running = False
        self.frames = 0
        self.last_packet = None
        self._temporary = tempfile.TemporaryDirectory(prefix="sonar-nav-")
        self.record(Path(self._temporary.name) / "telemetry.jsonl")

    def start(self) -> None:
        self.running = True
        self.navigator.start()

    def pause(self) -> None:
        self.running = False
        self.navigator.pause()

    def estop(self) -> None:
        self.running = False
        self.navigator.pause(emergency=True)

    def explore(self) -> None:
        self.running = True
        self.navigator.explore()

    def set_goal(self, x: float, y: float) -> bool:
        accepted = self.navigator.set_goal(x, y)
        if accepted:
            self.running = True
        return accepted

    def step(self, dt: float = 0.05) -> None:
        command = self.navigator.command if self.running else Command()
        packet = self.simulator.advance(dt, command)
        self.last_packet = packet
        self.navigator.ingest(packet)
        if not self.navigator.enabled:
            self.running = False
        self.frames += 1
        if self.recorder:
            self.recorder.write(packet, command, self.navigator.pose)

    def record(self, path: Path) -> None:
        from .adapters import JsonlRecorder
        if self.frames > 0:
            raise ValueError("为保证日志可完整回放，请在重置后、第一帧之前选择记录路径")
        if self.recorder:
            self.recorder.close()
        Path(path).parent.mkdir(parents=True, exist_ok=True)
        self.recorder = JsonlRecorder(path, self.config, metadata={"config": asdict(self.config),
                                      "scenario": self.scenario,
                                      "initial_pose": asdict(self.world.start_pose)})
        self.log_path = Path(path)

    def summary(self) -> dict:
        return {"scenario": self.scenario, "simulated_seconds": self.simulator.time_s,
                "frames": self.frames, "state": self.navigator.state,
                "known_fraction": self.navigator.grid.known_fraction(),
                "known_free_area_m2": self.navigator.grid.free_area_m2(),
                "estimated_distance_m": self.navigator.localizer.distance_m,
                "true_distance_m": self.simulator.travel_m,
                "collision_events": self.simulator.collision_count,
                "estimated_pose": asdict(self.navigator.pose),
                "true_pose": asdict(self.simulator.truth_pose),
                "gyro_bias_estimate_rad_s": self.navigator.localizer.gyro_bias,
                "note": "粗略超声波建图与里程计；没有回环或全局定位修正，真值只供评分。"}

    def export(self, directory: Path) -> Path:
        if self.recorder:
            self.recorder.flush()
        return export_navigation(self.navigator, self.config, directory, self.summary(), self.log_path)

    def close(self) -> None:
        self.estop()
        if self.recorder:
            self.recorder.close()
            self.recorder = None
        if self._temporary:
            self._temporary.cleanup()
            self._temporary = None
