#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""任务4 自主导航演示：A* 规划 + 里程计 + 差速控制 + 虚拟 FPGA。

用法（无板卡）：
    python nav_demo.py
    python nav_demo.py --obstacles "1500,1500,250;2500,2500,200" --duration 30
"""

from __future__ import annotations

import argparse
import os
import subprocess
import sys
import tempfile
import time
from pathlib import Path

from fpga_link import FpgaLink
from fpga_protocol import Mode, StatusFlag
from navigation import GridMap, NavController, Obstacle, Odometry, Pose
from semantic_parser import parse_instruction


def _parse_obstacles(spec: str) -> list[Obstacle]:
    result: list[Obstacle] = []
    if not spec:
        return result
    for part in spec.split(";"):
        part = part.strip()
        if not part:
            continue
        x, y, r = (float(v.strip()) for v in part.split(","))
        result.append(Obstacle(x, y, r))
    return result


def _wait_for_pty(path: Path, timeout_s: float = 5.0) -> str:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if path.exists():
            text = path.read_text(encoding="ascii").strip()
            if text:
                return text
        time.sleep(0.02)
    raise RuntimeError(f"等待虚拟 FPGA PTY 超时: {path}")


def _spawn_virtual_fpga(pty_file: Path, args: argparse.Namespace) -> subprocess.Popen:
    script = Path(__file__).resolve().parent / "virtual_fpga.py"
    cmd = [
        sys.executable,
        str(script),
        "--pty-file", str(pty_file),
        "--status-hz", str(args.status_hz),
        "--watchdog-ms", str(args.watchdog_ms),
        "--track-width-mm", str(args.track_width_mm),
        "--max-distance-cm", str(args.max_distance_cm),
        "--start-x-mm", str(args.start_x_mm),
        "--start-y-mm", str(args.start_y_mm),
        "--start-deg", str(args.start_deg),
    ]
    if args.obstacles:
        cmd += ["--obstacles", args.obstacles]
    env = dict(os.environ)
    env["PYTHONPATH"] = str(Path(__file__).resolve().parent) + os.pathsep + env.get("PYTHONPATH", "")
    return subprocess.Popen(cmd, env=env)


def main() -> int:
    parser = argparse.ArgumentParser(description="任务4 自主导航演示（虚拟 FPGA）")
    parser.add_argument("--start-x-mm", type=float, default=200.0)
    parser.add_argument("--start-y-mm", type=float, default=200.0)
    parser.add_argument("--start-deg", type=float, default=0.0)
    parser.add_argument("--goal-x-mm", type=float, default=3800.0)
    parser.add_argument("--goal-y-mm", type=float, default=3800.0)
    parser.add_argument("--map-width-mm", type=float, default=4000.0)
    parser.add_argument("--map-height-mm", type=float, default=4000.0)
    parser.add_argument("--cell-size-mm", type=float, default=100.0)
    parser.add_argument("--robot-radius-mm", type=float, default=150.0)
    parser.add_argument("--obstacles", default="1500,1500,250;2500,2500,200",
                        help="障碍物 x,y,r;x,y,r（mm）")
    parser.add_argument("--status-hz", type=float, default=20.0)
    parser.add_argument("--watchdog-ms", type=int, default=500)
    parser.add_argument("--track-width-mm", type=float, default=160.0)
    parser.add_argument("--max-distance-cm", type=int, default=500)
    parser.add_argument("--duration", type=float, default=60.0)
    parser.add_argument("--replan-cooldown-s", type=float, default=2.0)
    parser.add_argument("--log", default=None, help="可选 CSV 日志路径")
    parser.add_argument("--instruction", default=None,
                        help='中文指令，如 "走到右上角" / "停止"')
    args = parser.parse_args()

    obstacles = _parse_obstacles(args.obstacles)
    grid = GridMap(
        width_mm=args.map_width_mm,
        height_mm=args.map_height_mm,
        cell_size_mm=args.cell_size_mm,
        robot_radius_mm=args.robot_radius_mm,
        obstacles=obstacles,
    )
    start = (args.start_x_mm, args.start_y_mm)
    goal = (args.goal_x_mm, args.goal_y_mm)
    if args.instruction:
        intent = parse_instruction(args.instruction)
        if intent.action == "STOP":
            print("[语义] 指令为停止，不启动导航")
            return 0
        if intent.zone:
            margin = 200.0
            zones = {
                "top-left": (margin, margin),
                "bottom-left": (margin, args.map_height_mm - margin),
                "top-right": (args.map_width_mm - margin, margin),
                "bottom-right": (args.map_width_mm - margin, args.map_height_mm - margin),
                "center": (args.map_width_mm / 2.0, args.map_height_mm / 2.0),
                "goal": (args.goal_x_mm, args.goal_y_mm),
            }
            if intent.zone in zones:
                goal = zones[intent.zone]
                print(f"[语义] {args.instruction} -> 目标区域 {intent.zone} = {goal}")
    path = grid.plan(start, goal)
    if not path:
        print(f"[导航] 起点或终点在障碍内，或无可达路径：{start} -> {goal}")
        return 1
    print(f"[导航] A* 路径点数={len(path)} 起点={start} 终点={goal}")

    fd, tmp_name = tempfile.mkstemp(prefix="nav_", suffix=".tty")
    os.close(fd)
    pty_file = Path(tmp_name)
    pty_file.unlink(missing_ok=True)

    proc = _spawn_virtual_fpga(pty_file, args)
    link = FpgaLink(port=None, heartbeat_ms=100, watchdog_ms=args.watchdog_ms)
    log_fp = None
    try:
        slave = _wait_for_pty(pty_file)
        link = FpgaLink(port=slave, heartbeat_ms=100, watchdog_ms=args.watchdog_ms)
        link.start()
        link.set_mode(Mode.VISION)

        odom = Odometry(Pose(args.start_x_mm, args.start_y_mm, __import__("math").radians(args.start_deg)),
                        track_width_mm=args.track_width_mm)
        controller = NavController()
        waypoints = list(path)
        started = time.monotonic()
        last_update = started
        last_print = 0.0
        last_replan = 0.0
        reached_goal = False
        log_fp = open(args.log, "w", encoding="utf-8") if args.log else None
        if log_fp:
            log_fp.write("t,x_mm,y_mm,theta_rad,v_mm_s,w_mrad_s,distance_cm,state,note\n")

        while True:
            if proc.poll() is not None:
                raise RuntimeError("虚拟 FPGA 进程意外退出")
            now = time.monotonic()
            if now - started >= args.duration:
                print("[导航] 超时")
                return 2
            time.sleep(0.02)
            status = link.status
            if status is None:
                continue
            dt = min(0.2, max(0.0, now - last_update))
            last_update = now
            odom.update(dt, status.speed_left_mm_s, status.speed_right_mm_s)
            pose = odom.pose
            distance_cm = status.distance_cm if status.distance_valid else None
            obstacle_flag = bool(status.flags & StatusFlag.OBSTACLE)

            if waypoints:
                action = controller.step(pose, waypoints[0], distance_cm, obstacle_flag)
            else:
                action = controller.step(pose, None, distance_cm, obstacle_flag)

            if action.state == controller.STATE_REACHED:
                waypoints.pop(0)
                if waypoints:
                    action = controller.step(pose, waypoints[0], distance_cm, obstacle_flag)
                else:
                    reached_goal = True
                    action = controller.step(pose, None, distance_cm, obstacle_flag)

            if action.state == controller.STATE_OBSTACLE and waypoints and now - last_replan >= args.replan_cooldown_s:
                new_path = grid.plan((pose.x_mm, pose.y_mm), goal)
                if new_path:
                    waypoints = list(new_path)
                    last_replan = now
                    action = controller.step(pose, waypoints[0], distance_cm, obstacle_flag)
                    print(f"[导航] 遇障重新规划，新路径点={len(waypoints)}")

            link.set_twist(int(action.v_mm_s), int(action.w_mrad_s))

            if now - last_print >= 0.5 or reached_goal:
                last_print = now
                print(
                    f"[导航] t={now - started:6.2f} x={pose.x_mm:7.1f} y={pose.y_mm:7.1f} "
                    f"θ={pose.theta_rad:6.2f} 动作={action.state} "
                    f"v={action.v_mm_s:5.0f} w={action.w_mrad_s:5.0f} "
                    f"dist={distance_cm if distance_cm is not None else '-'}cm"
                )
                if log_fp:
                    log_fp.write(
                        f"{now - started:.3f},{pose.x_mm:.1f},{pose.y_mm:.1f},{pose.theta_rad:.4f},"
                        f"{action.v_mm_s:.1f},{action.w_mrad_s:.1f},"
                        f"{distance_cm if distance_cm is not None else ''},{action.state},{action.note}\n"
                    )

            if reached_goal:
                link.stop()
                print("[导航] ✅ 已到达目标点")
                return 0
    finally:
        if log_fp is not None:
            log_fp.close()
        try:
            link.stop()
        except Exception:
            pass
        try:
            link.close()
        except Exception:
            pass
        if proc.poll() is None:
            proc.terminate()
            try:
                proc.wait(timeout=3)
            except subprocess.TimeoutExpired:
                proc.kill()
        pty_file.unlink(missing_ok=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
