#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""RK3588 端 FPGA 通信层。

FpgaLink 负责：
  - 打包/发送协议 v2 命令
  - 后台接收 STATUS/ACK/FAULT
  - 周期发送 HEARTBEAT
  - 通信超时后自动发送 STOP

没有传入串口时对象仍可创建，所有发送动作变成安全的 no-op，
方便电脑端单独运行视觉脚本。
"""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass
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
    make_set_mode,
    make_set_twist,
    make_stop,
    parse_ack,
    parse_fault,
    parse_status,
)


@dataclass(frozen=True)
class FpgaStatus:
    """FPGA 最新状态。"""

    speed_left_mm_s: int
    speed_right_mm_s: int
    distance_cm: int
    yaw_centideg: int
    flags: StatusFlag
    received_at: float

    @property
    def distance_valid(self) -> bool:
        return (
            self.distance_cm != INVALID_DISTANCE_CM
            and bool(self.flags & StatusFlag.ULTRASONIC_VALID)
        )


class FpgaLink:
    """协议 v2 串口客户端。"""

    def __init__(
        self,
        port: Optional[str] = None,
        baud: int = 115200,
        heartbeat_ms: int = 100,
        watchdog_ms: int = 500,
        verbose: bool = True,
    ) -> None:
        self.port = port
        self.baud = int(baud)
        self.heartbeat_ms = max(20, int(heartbeat_ms))
        self.watchdog_ms = max(self.heartbeat_ms * 2, int(watchdog_ms))
        self.verbose = verbose

        self._ser = None
        self._parser = FrameParser()
        self._seq = 0
        self._lock = threading.Lock()
        self._status_lock = threading.Lock()
        self._stop_event = threading.Event()
        self._threads: list[threading.Thread] = []
        self._status: Optional[FpgaStatus] = None
        self._last_status_at: Optional[float] = None
        self._watchdog_tripped = False
        self._fault: Optional[tuple[int, int]] = None

    @property
    def connected(self) -> bool:
        return self._ser is not None and bool(getattr(self._ser, "is_open", False))

    @property
    def status(self) -> Optional[FpgaStatus]:
        with self._status_lock:
            return self._status

    @property
    def fault(self) -> Optional[tuple[int, int]]:
        with self._status_lock:
            return self._fault

    @property
    def watchdog_tripped(self) -> bool:
        return self._watchdog_tripped

    def start(self) -> bool:
        """打开串口并启动接收/心跳线程；未指定端口时返回 False。"""

        if not self.port:
            if self.verbose:
                print("[链路] 未指定 --serial，仅运行视觉逻辑（不通车）")
            return False

        try:
            import serial
        except ImportError as exc:
            raise RuntimeError("缺少 pyserial，请先安装 requirements.txt") from exc

        try:
            self._ser = serial.Serial(self.port, self.baud, timeout=0.02, write_timeout=0.2)
        except Exception as exc:
            if self.verbose:
                print(f"[链路] 串口打开失败：{exc}")
            self._ser = None
            return False

        self._stop_event.clear()
        self._parser.reset()
        self._last_status_at = time.monotonic()
        self._watchdog_tripped = False
        self._threads = [
            threading.Thread(target=self._rx_loop, name="fpga-rx", daemon=True),
            threading.Thread(target=self._heartbeat_loop, name="fpga-heartbeat", daemon=True),
        ]
        for thread in self._threads:
            thread.start()

        if self.verbose:
            print(f"[链路] 已连接 FPGA {self.port} @ {self.baud}")
        return True

    def close(self) -> None:
        """退出前安全停车并关闭串口。"""

        if self.connected:
            self.stop(StopReason.OPERATOR)
        self._stop_event.set()
        for thread in self._threads:
            thread.join(timeout=0.5)
        self._threads.clear()
        if self._ser is not None:
            try:
                self._ser.close()
            except Exception:
                pass
            self._ser = None

    def send_command(self, cmd: Command, payload: bytes = b"") -> bool:
        """发送一条命令，返回是否实际写入串口。"""

        seq = self._next_seq()
        frame = self._encode(cmd, seq, payload)
        with self._lock:
            if not self.connected:
                return False
            try:
                self._ser.write(frame)
                self._ser.flush()
                return True
            except Exception as exc:
                if self.verbose:
                    print(f"[链路] 发送失败：{exc}")
                return False

    def set_twist(self, v_mm_s: int, w_mrad_s: int) -> bool:
        """设置线速度与角速度；正值 w 表示逆时针左转。"""

        v = max(-3000, min(3000, int(v_mm_s)))
        w = max(-3000, min(3000, int(w_mrad_s)))
        return self.send_command(Command.SET_TWIST, make_set_twist(v, w))

    def stop(self, reason: StopReason = StopReason.NORMAL) -> bool:
        return self.send_command(Command.STOP, make_stop(reason))

    def set_mode(self, mode: Mode) -> bool:
        return self.send_command(Command.SET_MODE, make_set_mode(mode))

    def clear_fault(self) -> bool:
        return self.send_command(Command.CLEAR_FAULT)

    def is_status_fresh(self, max_age_ms: Optional[int] = None) -> bool:
        """STATUS 是否新鲜且链路未触发看门狗。"""

        if self._last_status_at is None or self._watchdog_tripped:
            return False
        limit = self.watchdog_ms if max_age_ms is None else int(max_age_ms)
        return (time.monotonic() - self._last_status_at) * 1000.0 <= limit

    def distance_cm(self) -> Optional[int]:
        """取得新鲜且有效的超声波距离，否则返回 None。"""

        status = self.status
        if status is None or not self.is_status_fresh() or not status.distance_valid:
            return None
        return status.distance_cm

    def _next_seq(self) -> int:
        self._seq = (self._seq + 1) & 0xFF
        return self._seq

    @staticmethod
    def _encode(cmd: Command, seq: int, payload: bytes) -> bytes:
        from fpga_protocol import encode_frame

        return encode_frame(cmd, seq, payload)

    def _rx_loop(self) -> None:
        assert self._ser is not None
        while not self._stop_event.is_set():
            try:
                chunk = self._ser.read(256)
            except Exception as exc:
                if self.verbose:
                    print(f"[链路] 接收失败：{exc}")
                break
            if not chunk:
                continue
            for frame in self._parser.feed(chunk):
                self._process_frame(frame)

    def _heartbeat_loop(self) -> None:
        next_heartbeat = time.monotonic()
        while not self._stop_event.wait(0.02):
            now = time.monotonic()
            if now >= next_heartbeat:
                self.send_command(Command.HEARTBEAT)
                next_heartbeat = now + self.heartbeat_ms / 1000.0
            self._check_watchdog(now)

    def _check_watchdog(self, now: float) -> None:
        if self._last_status_at is None or self._watchdog_tripped:
            return
        age_ms = (now - self._last_status_at) * 1000.0
        if age_ms <= self.watchdog_ms:
            return

        self._watchdog_tripped = True
        if self.verbose:
            print(f"[链路] STATUS 超时 {age_ms:.0f} ms，自动发送 STOP")
        self.stop(StopReason.WATCHDOG)

    def _process_frame(self, frame: Frame) -> None:
        if frame.cmd == Command.STATUS:
            try:
                speed_l, speed_r, distance, yaw, flags, _ = parse_status(frame.payload)
            except ValueError as exc:
                if self.verbose:
                    print(f"[链路] 坏 STATUS：{exc}")
                return
            now = time.monotonic()
            with self._status_lock:
                first_status = self._status is None
                self._status = FpgaStatus(
                    speed_left_mm_s=speed_l,
                    speed_right_mm_s=speed_r,
                    distance_cm=distance,
                    yaw_centideg=yaw,
                    flags=flags,
                    received_at=now,
                )
                self._last_status_at = now
                self._watchdog_tripped = False
            if first_status and self.verbose:
                print(f"[链路] 收到 FPGA STATUS：距离={distance}cm "
                      f"轮速=({speed_l},{speed_r})mm/s")
            return

        if frame.cmd == Command.ACK:
            try:
                acked_cmd, code, detail = parse_ack(frame.payload)
            except ValueError as exc:
                if self.verbose:
                    print(f"[链路] 坏 ACK：{exc}")
                return
            if code != ErrorCode.OK and self.verbose:
                print(f"[链路] CMD 0x{acked_cmd:02X} 返回错误 {code.name} detail={detail}")
            return

        if frame.cmd == Command.FAULT:
            try:
                fault = parse_fault(frame.payload)
            except ValueError as exc:
                if self.verbose:
                    print(f"[链路] 坏 FAULT：{exc}")
                return
            with self._status_lock:
                self._fault = fault
            if self.verbose:
                print(f"[链路] FPGA 故障 code={fault[0]} info={fault[1]}")
