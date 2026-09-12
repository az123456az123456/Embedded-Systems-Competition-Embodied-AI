#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""虚拟 FPGA + FpgaLink 端到端测试。"""

import os
import subprocess
import sys
import tempfile
import time
import unittest
from pathlib import Path

from fpga_link import FpgaLink
from fpga_protocol import Mode


def wait_until(predicate, timeout=4.0, interval=0.02):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        value = predicate()
        if value:
            return value
        time.sleep(interval)
    return None


class VirtualFpgaLinkTest(unittest.TestCase):
    def test_real_pty_link(self) -> None:
        if not hasattr(os, "openpty"):
            self.skipTest("当前平台不支持 PTY")

        here = Path(__file__).resolve().parent
        with tempfile.TemporaryDirectory() as tmp:
            pty_file = Path(tmp) / "virtual_fpga.tty"
            proc = subprocess.Popen(
                [
                    sys.executable,
                    str(here / "virtual_fpga.py"),
                    "--pty-file",
                    str(pty_file),
                    "--duration",
                    "8",
                    "--status-hz",
                    "20",
                    "--watchdog-ms",
                    "300",
                ],
                cwd=here,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
            )

            try:
                self.assertTrue(
                    wait_until(lambda: pty_file.exists() and pty_file.read_text().strip()),
                    "虚拟 FPGA 未生成 PTY",
                )
                tty = pty_file.read_text().strip()
                link = FpgaLink(
                    tty,
                    heartbeat_ms=50,
                    watchdog_ms=500,
                    verbose=False,
                )
                self.assertTrue(link.start())
                try:
                    self.assertTrue(link.set_mode(Mode.VISION))
                    self.assertTrue(link.set_twist(250, 0))
                    first = wait_until(lambda: link.status, timeout=2.0)
                    self.assertIsNotNone(first)
                    first_distance = first.distance_cm

                    moving = wait_until(
                        lambda: link.status
                        and link.status.distance_cm < first_distance - 1,
                        timeout=2.0,
                    )
                    self.assertIsNotNone(moving, "虚拟 FPGA 距离没有下降")
                    self.assertTrue(link.is_status_fresh())

                    self.assertTrue(link.stop())
                    stopped = wait_until(
                        lambda: link.status
                        and abs(link.status.speed_left_mm_s) < 5
                        and abs(link.status.speed_right_mm_s) < 5,
                        timeout=2.0,
                    )
                    self.assertIsNotNone(stopped, "虚拟 FPGA 没有停车")
                finally:
                    link.close()
            finally:
                proc.terminate()
                try:
                    proc.wait(timeout=2)
                except subprocess.TimeoutExpired:
                    proc.kill()
                    proc.wait(timeout=2)
                output = proc.stdout.read() if proc.stdout else ""
                if proc.stdout:
                    proc.stdout.close()
                if proc.returncode not in (0, -15):
                    self.fail(f"虚拟 FPGA 异常退出：{proc.returncode}\n{output}")


if __name__ == "__main__":
    unittest.main()
