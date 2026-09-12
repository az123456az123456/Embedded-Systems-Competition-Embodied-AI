#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""虚拟 FPGA：通过 PTY 模拟协议 v2 的串口设备。

用途：在没有 FPGA 板卡时，让 RK3588 视觉程序走真实二进制协议和状态回传。
启动后会把 PTY 从设备路径写入 --pty-file，task6_mission.py 用该路径作为
--serial 参数即可完成闭环联调。
"""

from __future__ import annotations

import argparse
import math
import os
import pty
import select
import signal
import sys
import time
import tty
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

from fpga_protocol import (
    Command,
    ErrorCode,
    Frame,
    FrameParser,
    INVALID_DISTANCE_CM,
    Mode,
    StopReason,
    StatusFlag,
    encode_frame,
    make_ack,
    make_status,
    unpack_i16,
)


@dataclass
class VirtualState:
    speed_left_mm_s: float = 0.0
    speed_right_mm_s: float = 0.0
    target_v_mm_s: float = 0.0
    target_w_mrad_s: float = 0.0
    distance_mm: float = 2000.0
    yaw_centideg: float = 0.0
    mode: Mode = Mode.VISION
    last_heartbeat_at: float = 0.0
    watchdog_tripped: bool = False
    fault_code: int = 0


class VirtualFpga:
    """最小可用的 FPGA 运动/传感器模型。"""

    def __init__(
        self,
        track_width_mm: float = 160.0,
        watchdog_ms: int = 500,
        max_distance_cm: int = 500,
    ) -> None:
        self.track_width_mm = float(track_width_mm)
        self.watchdog_s = max(0.1, watchdog_ms / 1000.0)
        self.max_distance_mm = max_distance_cm * 10.0
        self.state = VirtualState()
        self._seq = 0
        self._parser = FrameParser()

    def _next_seq(self) -> int:
        self._seq = (self._seq + 1) & 0xFF
        return self._seq

    def make_frame(self, cmd: Command, payload: bytes = b"") -> bytes:
        return encode_frame(cmd, self._next_seq(), payload)

    def handle_frame(self, frame: Frame) -> Optional[bytes]:
        """处理下行命令，返回需要立即回给主机的帧。"""

        now = time.monotonic()
        state = self.state

        if frame.cmd == Command.HEARTBEAT:
            state.last_heartbeat_at = now
            state.watchdog_tripped = False
            return self.make_frame(Command.ACK, make_ack(frame.cmd, ErrorCode.OK))

        if frame.cmd == Command.SET_TWIST:
            if len(frame.payload) != 4:
                return self.make_frame(
                    Command.ACK, make_ack(frame.cmd, ErrorCode.BAD_PAYLOAD, len(frame.payload))
                )
            if state.watchdog_tripped or state.fault_code:
                return self.make_frame(
                    Command.ACK, make_ack(frame.cmd, ErrorCode.NOT_READY, state.fault_code)
                )
            state.target_v_mm_s = float(unpack_i16(frame.payload, 0))
            state.target_w_mrad_s = float(unpack_i16(frame.payload, 2))
            return self.make_frame(Command.ACK, make_ack(frame.cmd, ErrorCode.OK))

        if frame.cmd == Command.STOP:
            reason = frame.payload[0] if frame.payload else int(StopReason.NORMAL)
            state.target_v_mm_s = 0.0
            state.target_w_mrad_s = 0.0
            if reason == int(StopReason.WATCHDOG):
                state.watchdog_tripped = True
                state.last_heartbeat_at = 0.0
            return self.make_frame(Command.ACK, make_ack(frame.cmd, ErrorCode.OK, reason))

        if frame.cmd == Command.SET_MODE:
            if len(frame.payload) != 1 or frame.payload[0] > int(Mode.IDLE):
                return self.make_frame(
                    Command.ACK, make_ack(frame.cmd, ErrorCode.BAD_PAYLOAD, len(frame.payload))
                )
            state.mode = Mode(frame.payload[0])
            return self.make_frame(Command.ACK, make_ack(frame.cmd, ErrorCode.OK))

        if frame.cmd == Command.CLEAR_FAULT:
            state.fault_code = 0
            state.watchdog_tripped = False
            state.last_heartbeat_at = now
            return self.make_frame(Command.ACK, make_ack(frame.cmd, ErrorCode.OK))

        return self.make_frame(Command.ACK, make_ack(frame.cmd, ErrorCode.BAD_COMMAND))

    def update(self, dt: float) -> None:
        """积分速度、航向和目标距离。"""

        state = self.state
        now = time.monotonic()
        if now - state.last_heartbeat_at > self.watchdog_s:
            state.target_v_mm_s = 0.0
            state.target_w_mrad_s = 0.0
            state.watchdog_tripped = True

        # 一阶速度响应，避免模拟器比真实执行器更理想。
        alpha = min(1.0, max(0.0, dt * 8.0))
        wheel_span = state.target_w_mrad_s * self.track_width_mm / 2000.0
        target_left = state.target_v_mm_s - wheel_span
        target_right = state.target_v_mm_s + wheel_span
        state.speed_left_mm_s += (target_left - state.speed_left_mm_s) * alpha
        state.speed_right_mm_s += (target_right - state.speed_right_mm_s) * alpha

        actual_v = (state.speed_left_mm_s + state.speed_right_mm_s) / 2.0
        actual_w = (
            (state.speed_right_mm_s - state.speed_left_mm_s) / self.track_width_mm
        )
        state.distance_mm = min(
            self.max_distance_mm,
            max(100.0, state.distance_mm - actual_v * dt),
        )
        state.yaw_centideg = (
            state.yaw_centideg + math.degrees(actual_w * dt) * 100.0
        ) % 36000.0

    def status_frame(self) -> bytes:
        state = self.state
        flags = StatusFlag.LINK_OK | StatusFlag.ULTRASONIC_VALID | StatusFlag.IMU_VALID
        if abs(state.speed_left_mm_s) > 1 or abs(state.speed_right_mm_s) > 1:
            flags |= StatusFlag.MOVING
        if state.watchdog_tripped:
            flags |= StatusFlag.WATCHDOG | StatusFlag.ESTOP
        if state.fault_code:
            flags |= StatusFlag.FAULT

        distance_cm = int(round(state.distance_mm / 10.0))
        if distance_cm > INVALID_DISTANCE_CM - 1:
            distance_cm = INVALID_DISTANCE_CM
        payload = make_status(
            int(round(state.speed_left_mm_s)),
            int(round(state.speed_right_mm_s)),
            distance_cm,
            int(round(state.yaw_centideg)),
            flags,
        )
        return self.make_frame(Command.STATUS, payload)


def _write_pty_path(path: str, slave_name: str) -> None:
    target = Path(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    tmp = target.with_suffix(target.suffix + ".tmp")
    tmp.write_text(slave_name + "\n", encoding="ascii")
    tmp.replace(target)


def _run(args: argparse.Namespace) -> int:
    master_fd, slave_fd = pty.openpty()
    slave_name = os.ttyname(slave_fd)
    tty.setraw(master_fd)
    tty.setraw(slave_fd)
    os.set_blocking(master_fd, False)

    if args.pty_file:
        _write_pty_path(args.pty_file, slave_name)
    print(f"[虚拟FPGA] PTY={slave_name}", flush=True)

    model = VirtualFpga(
        track_width_mm=args.track_width_mm,
        watchdog_ms=args.watchdog_ms,
        max_distance_cm=args.max_distance_cm,
    )
    model.state.distance_mm = args.distance_cm * 10.0
    model.state.last_heartbeat_at = time.monotonic()

    running = True

    def request_stop(_signum, _frame) -> None:
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)

    started_at = time.monotonic()
    last_update = started_at
    status_period = 1.0 / max(1.0, args.status_hz)
    next_status = started_at

    try:
        while running:
            now = time.monotonic()
            if args.duration > 0 and now - started_at >= args.duration:
                break

            timeout = min(0.02, max(0.0, next_status - now))
            ready, _, _ = select.select([master_fd], [], [], timeout)
            if ready:
                try:
                    chunk = os.read(master_fd, 4096)
                except OSError:
                    break
                if not chunk:
                    continue
                for frame in model._parser.feed(chunk):
                    response = model.handle_frame(frame)
                    if response:
                        os.write(master_fd, response)

            now = time.monotonic()
            model.update(max(0.0, min(0.05, now - last_update)))
            last_update = now

            if now >= next_status:
                os.write(master_fd, model.status_frame())
                next_status = now + status_period
    finally:
        try:
            model.state.target_v_mm_s = 0.0
            model.state.target_w_mrad_s = 0.0
            os.write(master_fd, model.status_frame())
        except OSError:
            pass
        os.close(master_fd)
        os.close(slave_fd)
        if args.pty_file:
            try:
                Path(args.pty_file).unlink()
            except FileNotFoundError:
                pass

    return 0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="协议 v2 虚拟 FPGA（PTY 串口模拟器）")
    parser.add_argument("--pty-file", default="/tmp/virtual_fpga.tty",
                        help="把 PTY 从设备路径写入此文件")
    parser.add_argument("--duration", type=float, default=0.0,
                        help="运行秒数，0 表示持续运行")
    parser.add_argument("--status-hz", type=float, default=10.0,
                        help="STATUS 上报频率")
    parser.add_argument("--watchdog-ms", type=int, default=500,
                        help="未收到 HEARTBEAT 后停车的时间")
    parser.add_argument("--distance-cm", type=int, default=200,
                        help="初始超声波距离")
    parser.add_argument("--max-distance-cm", type=int, default=500)
    parser.add_argument("--track-width-mm", type=float, default=160.0)
    return parser.parse_args()


if __name__ == "__main__":
    sys.exit(_run(parse_args()))
