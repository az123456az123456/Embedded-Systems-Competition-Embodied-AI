#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""FPGA <-> RK3588 二进制通信协议 v2。

帧格式::

    AA 55 | CMD | SEQ | LEN | PAYLOAD | CRC_L CRC_H

CRC 使用 CRC-16/CCITT-FALSE（初始值 0xFFFF、多项式 0x1021），
计算范围是 CMD、SEQ、LEN 和 PAYLOAD，不包含帧头与 CRC 本身。
所有多字节整数均为小端序。
"""

from __future__ import annotations

import struct
from dataclasses import dataclass
from enum import IntEnum, IntFlag
from typing import Iterable, List, Tuple


SOF1 = 0xAA
SOF2 = 0x55
HEADER_SIZE = 5
CRC_SIZE = 2
MAX_PAYLOAD = 32


class Command(IntEnum):
    """协议命令码。"""

    # 下行：RK3588 -> FPGA
    SET_TWIST = 0x01
    STOP = 0x02
    SET_MODE = 0x03
    CLEAR_FAULT = 0x04
    HEARTBEAT = 0x10

    # 上行：FPGA -> RK3588
    STATUS = 0x81
    ACK = 0x82
    FAULT = 0x83


class Mode(IntEnum):
    """FPGA 控制模式。"""

    MANUAL = 0
    OBSTACLE = 1
    VISION = 2
    IDLE = 3


class StopReason(IntEnum):
    """停车原因，仅用于诊断。"""

    NORMAL = 0
    WATCHDOG = 1
    OPERATOR = 2
    FAULT = 3


class ErrorCode(IntEnum):
    """ACK/FAULT 错误码。"""

    OK = 0
    BAD_COMMAND = 1
    BAD_PAYLOAD = 2
    NOT_READY = 3
    CRC_ERROR = 4
    WATCHDOG = 5
    ESTOP = 6


class StatusFlag(IntFlag):
    """STATUS 帧的故障与有效性标志。"""

    LINK_OK = 1 << 0
    MOVING = 1 << 1
    OBSTACLE = 1 << 2
    ESTOP = 1 << 3
    IMU_VALID = 1 << 4
    ULTRASONIC_VALID = 1 << 5
    WATCHDOG = 1 << 6
    FAULT = 1 << 7


INVALID_DISTANCE_CM = 0xFFFF


def crc16_ccitt(data: bytes, crc: int = 0xFFFF) -> int:
    """计算 CRC-16/CCITT-FALSE。"""

    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def encode_frame(cmd: int, seq: int, payload: bytes = b"") -> bytes:
    """编码一帧。非法长度或命令码会抛出 ValueError。"""

    if not 0 <= int(cmd) <= 0xFF:
        raise ValueError("cmd 必须在 0..255")
    if not 0 <= int(seq) <= 0xFF:
        raise ValueError("seq 必须在 0..255")
    if len(payload) > MAX_PAYLOAD:
        raise ValueError(f"payload 不能超过 {MAX_PAYLOAD} 字节")

    body = bytes((int(cmd), int(seq), len(payload))) + bytes(payload)
    crc = crc16_ccitt(body)
    return bytes((SOF1, SOF2)) + body + struct.pack("<H", crc)


@dataclass(frozen=True)
class Frame:
    """已完成校验的一帧。"""

    cmd: int
    seq: int
    payload: bytes


class FrameParser:
    """串口字节流的增量帧解析器。

    feed() 可以接收任意分片；遇到噪声、坏 CRC 或重复帧头时会自动重新同步。
    """

    def __init__(self) -> None:
        self._buf = bytearray()
        self.crc_errors = 0

    def reset(self) -> None:
        self._buf.clear()

    def feed(self, data: bytes) -> List[Frame]:
        self._buf.extend(data)
        frames: List[Frame] = []

        while True:
            start = self._find_start()
            if start < 0:
                if len(self._buf) > 1:
                    del self._buf[:-1]
                break
            if start:
                del self._buf[:start]
            if len(self._buf) < HEADER_SIZE:
                break

            payload_len = self._buf[4]
            if payload_len > MAX_PAYLOAD:
                del self._buf[0]
                continue

            frame_len = HEADER_SIZE + payload_len + CRC_SIZE
            if len(self._buf) < frame_len:
                break

            raw = bytes(self._buf[:frame_len])
            del self._buf[:frame_len]
            body = raw[2:-2]
            expected = struct.unpack_from("<H", raw, len(raw) - 2)[0]
            actual = crc16_ccitt(body)
            if expected != actual:
                self.crc_errors += 1
                # 当前帧头可能只是数据中的偶发序列，跳过一个字节重新找帧头。
                self._buf[:0] = raw[1:]
                continue

            frames.append(Frame(cmd=body[0], seq=body[1], payload=body[3:]))

        return frames

    def _find_start(self) -> int:
        for i in range(max(0, len(self._buf) - 1)):
            if self._buf[i] == SOF1 and self._buf[i + 1] == SOF2:
                return i
        return -1


def pack_i16(value: int) -> bytes:
    """打包有符号 16 位整数。"""

    return struct.pack("<h", int(value))


def pack_u16(value: int) -> bytes:
    """打包无符号 16 位整数。"""

    return struct.pack("<H", int(value))


def unpack_i16(payload: bytes, offset: int) -> int:
    """从 payload 指定偏移读取有符号 16 位整数。"""

    return struct.unpack_from("<h", payload, offset)[0]


def unpack_u16(payload: bytes, offset: int) -> int:
    """从 payload 指定偏移读取无符号 16 位整数。"""

    return struct.unpack_from("<H", payload, offset)[0]


def make_set_twist(v_mm_s: int, w_mrad_s: int) -> bytes:
    """SET_TWIST payload：前进速度 mm/s + 角速度 mrad/s。"""

    return pack_i16(v_mm_s) + pack_i16(w_mrad_s)


def make_stop(reason: StopReason | int = StopReason.NORMAL) -> bytes:
    """STOP payload：停车原因。"""

    return bytes((int(reason) & 0xFF,))


def make_set_mode(mode: Mode | int) -> bytes:
    """SET_MODE payload。"""

    return bytes((int(mode) & 0xFF,))


def make_status(
    speed_left_mm_s: int,
    speed_right_mm_s: int,
    distance_cm: int,
    yaw_centideg: int,
    flags: StatusFlag | int,
    reserved: int = 0,
) -> bytes:
    """STATUS payload，共 12 字节。"""

    return b"".join(
        (
            pack_i16(speed_left_mm_s),
            pack_i16(speed_right_mm_s),
            pack_u16(distance_cm),
            pack_u16(yaw_centideg),
            pack_u16(int(flags)),
            pack_u16(reserved),
        )
    )


def make_ack(acked_cmd: int, code: ErrorCode | int, detail: int = 0) -> bytes:
    """ACK payload：被确认的命令、错误码、附加信息。"""

    return bytes((int(acked_cmd) & 0xFF, int(code) & 0xFF, int(detail) & 0xFF))


def make_fault(code: int, info: int = 0) -> bytes:
    """FAULT payload：16 位故障码和附加信息。"""

    return pack_u16(code) + pack_u16(info)


def parse_status(payload: bytes) -> Tuple[int, int, int, int, StatusFlag, int]:
    """解析 STATUS payload。"""

    if len(payload) != 12:
        raise ValueError(f"STATUS 长度应为 12，实际为 {len(payload)}")
    return (
        unpack_i16(payload, 0),
        unpack_i16(payload, 2),
        unpack_u16(payload, 4),
        unpack_u16(payload, 6),
        StatusFlag(unpack_u16(payload, 8)),
        unpack_u16(payload, 10),
    )


def parse_ack(payload: bytes) -> Tuple[int, ErrorCode, int]:
    """解析 ACK payload。"""

    if len(payload) != 3:
        raise ValueError(f"ACK 长度应为 3，实际为 {len(payload)}")
    return int(payload[0]), ErrorCode(payload[1]), int(payload[2])


def parse_fault(payload: bytes) -> Tuple[int, int]:
    """解析 FAULT payload。"""

    if len(payload) != 4:
        raise ValueError(f"FAULT 长度应为 4，实际为 {len(payload)}")
    return unpack_u16(payload, 0), unpack_u16(payload, 2)


def iter_frame_bytes(data: bytes) -> Iterable[Frame]:
    """测试辅助函数：一次性解析一段完整数据，忽略不完整尾部。"""

    return FrameParser().feed(data)
