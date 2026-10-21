#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""协议 v2 单元测试。"""

import unittest

from fpga_protocol import (
    Command,
    FrameParser,
    Mode,
    StatusFlag,
    crc16_ccitt,
    encode_frame,
    make_set_mode,
    make_set_twist,
    make_status,
    parse_status,
)


class ProtocolTest(unittest.TestCase):
    def test_crc_known_vector(self) -> None:
        self.assertEqual(crc16_ccitt(b"123456789"), 0x29B1)

    def test_round_trip(self) -> None:
        raw = encode_frame(Command.SET_TWIST, 7, make_set_twist(-300, 450))
        frames = FrameParser().feed(raw)
        self.assertEqual(len(frames), 1)
        self.assertEqual(frames[0].cmd, Command.SET_TWIST)
        self.assertEqual(frames[0].seq, 7)
        self.assertEqual(frames[0].payload, make_set_twist(-300, 450))

    def test_fragmented_stream_and_noise(self) -> None:
        f1 = encode_frame(Command.HEARTBEAT, 1)
        f2 = encode_frame(Command.SET_MODE, 2, make_set_mode(Mode.VISION))
        stream = b"\x00\xffnoise" + f1 + f2
        parser = FrameParser()
        frames = []
        for byte in stream:
            frames.extend(parser.feed(bytes((byte,))))
        self.assertEqual([f.cmd for f in frames], [Command.HEARTBEAT, Command.SET_MODE])
        self.assertEqual(frames[1].payload[0], Mode.VISION)

    def test_bad_crc_recovers(self) -> None:
        good = encode_frame(Command.STATUS, 3, make_status(1, 2, 34, 5, 0))
        bad = bytearray(encode_frame(Command.STATUS, 4, make_status(0, 0, 0, 0, 0)))
        bad[-1] ^= 0xFF
        parser = FrameParser()
        frames = parser.feed(bytes(bad) + good)
        self.assertGreaterEqual(parser.crc_errors, 1)
        self.assertTrue(any(f.seq == 3 for f in frames))

    def test_status_payload(self) -> None:
        flags = StatusFlag.LINK_OK | StatusFlag.ULTRASONIC_VALID
        payload = make_status(-120, 130, 45, 35000, flags, reserved=9)
        speed_l, speed_r, distance, yaw, parsed_flags, reserved = parse_status(payload)
        self.assertEqual((speed_l, speed_r, distance, yaw), (-120, 130, 45, 35000))
        self.assertEqual(parsed_flags, flags)
        self.assertEqual(reserved, 9)

    def test_reject_bad_payload_length(self) -> None:
        with self.assertRaises(ValueError):
            parse_status(b"\x00")


if __name__ == "__main__":
    unittest.main()
