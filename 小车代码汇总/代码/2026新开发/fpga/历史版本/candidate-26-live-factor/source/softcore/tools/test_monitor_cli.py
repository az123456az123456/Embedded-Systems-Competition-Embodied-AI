#!/usr/bin/env python3
"""CLI/host parser checks; no sockets are opened and no privilege is needed."""
import pathlib
import subprocess
import unittest

PROGRAM = pathlib.Path(__file__).resolve().parent / "softcore-monitor"


class MonitorCLI(unittest.TestCase):
    def call(self, *args):
        return subprocess.run([str(PROGRAM), *args], text=True, capture_output=True,
                              timeout=5, check=False)

    def test_parser_and_liveness(self):
        result = self.call("--self-test")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("PASS", result.stdout)

    def test_help_describes_passive_protocol(self):
        result = self.call("--help")
        self.assertEqual(result.returncode, 0)
        for text in ["0x88b7", "Sends no frames", "default 10", "heartbeat"]:
            self.assertIn(text, result.stdout)

    def test_reject_bad_cli_before_opening_socket(self):
        cases = [
            ["--unknown"], ["--timeout"], ["--count"], ["--interface"],
            ["--timeout", "-1"], ["--timeout", "nan"], ["--timeout", "1.5"],
            ["--timeout", "4294967296"], ["--count", "18446744073709551616"],
            ["--count", "+1"], ["--interface", ""], ["--interface", "a" * 16],
            ["--count", " 1"], ["--count", "1 "],
            ["--self-test", "--count", "1"],
        ]
        for args in cases:
            with self.subTest(args=args):
                result = self.call(*args)
                self.assertEqual(result.returncode, 1)
                self.assertNotIn("Listening", result.stdout)


if __name__ == "__main__":
    unittest.main()
