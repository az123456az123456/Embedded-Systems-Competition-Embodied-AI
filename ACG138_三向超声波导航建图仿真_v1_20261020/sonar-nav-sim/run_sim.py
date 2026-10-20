#!/usr/bin/env python3
"""Desktop simulation, reproducible headless runs, and sensor-log replay."""
from __future__ import annotations
import argparse
from dataclasses import asdict
import json
import os
from pathlib import Path
import sys

from navsim.model import Config, Command, Pose


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="三向 A02YYUW + 编码器 + IMU 导航建图仿真")
    parser.add_argument("--scenario", choices=("apartment", "open", "corridor"), default="apartment")
    parser.add_argument("--config", type=Path, help="读取机器人/传感器参数 JSON")
    parser.add_argument("--seed", type=int, help="可复现的噪声随机种子")
    parser.add_argument("--headless", action="store_true", help="不打开窗口")
    parser.add_argument("--seconds", type=float, default=120.0, help="无窗口仿真时长，默认120秒")
    parser.add_argument("--export", type=Path, help="导出地图、轨迹、摘要和传感器日志")
    parser.add_argument("--record", type=Path, help="另存实时传感器 JSONL 日志（无窗口模式）")
    parser.add_argument("--replay", type=Path, help="只读回放JSONL、重新估计位姿和地图（无窗口）")
    parser.add_argument("--screenshot", type=Path, help="离屏渲染窗口PNG后退出")
    parser.add_argument("--demo-seconds", type=float, default=60, help="截图前仿真时长")
    parser.add_argument("--write-config", type=Path, help="写出默认参数文件后退出")
    args = parser.parse_args(argv)
    try:
        config = Config(**json.loads(args.config.read_text(encoding="utf-8"))) if args.config else Config()
        if args.seed is not None:
            config.seed = args.seed
        config.validate()
        if args.write_config:
            args.write_config.write_text(json.dumps(asdict(config), ensure_ascii=False, indent=2), encoding="utf-8")
            return 0
        if not 0 < args.seconds <= 86400 or not 0 <= args.demo_seconds <= 86400:
            parser.error("仿真时长必须在0～86400秒之间")
        if args.replay:
            return replay(args.replay, args.config, args.export)
        if args.headless:
            from navsim.session import Session
            session = Session(config, args.scenario)
            try:
                if args.record:
                    session.record(args.record)
                session.start()
                while session.simulator.time_s < args.seconds - 1e-8:
                    session.step(min(0.05, args.seconds - session.simulator.time_s))
                if args.export:
                    session.export(args.export)
                print(json.dumps(session.summary(), ensure_ascii=False, indent=2))
            finally:
                session.close()
            return 0
        if args.record or args.export:
            parser.error("--record/--export 用于 --headless；窗口中请使用“导出地图与记录”")
        if args.screenshot:
            os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
        try:
            from navsim.gui import run_gui
        except ImportError as exc:
            print("窗口需要 PySide6。请运行：python3 -m pip install -r requirements.txt", file=sys.stderr)
            print(f"详细信息：{exc}", file=sys.stderr)
            return 2
        if not args.screenshot and sys.platform.startswith("linux") and not (os.getenv("DISPLAY") or os.getenv("WAYLAND_DISPLAY")):
            print("当前没有图形桌面。请在电脑桌面终端启动，或使用 --headless。", file=sys.stderr)
            return 2
        return run_gui(config, args.scenario, args.screenshot, args.demo_seconds)
    except (ValueError, OSError, TypeError) as exc:
        print(f"启动失败：{exc}", file=sys.stderr)
        return 2


def replay(path: Path, explicit_config: Path | None, export: Path | None) -> int:
    from navsim.adapters import JsonlReplay
    from navsim.navigator import Navigator
    from navsim.exports import export_navigation
    config = Config(**json.loads(explicit_config.read_text(encoding="utf-8"))) if explicit_config else None
    with JsonlReplay(path, config) as source:
        config = source.config
        initial = Pose(**source.metadata.get("initial_pose", asdict(Pose())))
        navigator = Navigator(config, initial)
        count = 0
        # Commands in the log are the inputs that actually generated its sensor
        # data. Replaying them is essential for stationary-bias calibration.
        while not source.finished:
            packet = source.advance(0.05, Command())
            navigator.command = source.recorded_command
            navigator.ingest(packet)
            count += 1
        if export:
            summary = {"mode": "read_only_replay", "frames": count,
                       "last_timestamp_s": navigator.now, "known_fraction": navigator.grid.known_fraction(),
                       "estimated_pose": asdict(navigator.pose), "source": str(path)}
            export_navigation(navigator, config, export, summary, path)
        print(json.dumps({"mode": "read_only_replay", "frames": count,
                          "known_fraction": navigator.grid.known_fraction(),
                          "estimated_pose": asdict(navigator.pose)}, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
