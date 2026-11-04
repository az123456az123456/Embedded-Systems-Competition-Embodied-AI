"""Verify the real server keeps its first pairing code across reconnect/restart.

All processes run DEMO/headless on 127.0.0.1: no camera, FPGA or motor I/O.
Existing files are tested by bytes/inode/mtime, not by a second token loader.
"""
from contextlib import contextmanager
import http.client
import json
import os
from pathlib import Path
import socket
import stat
import subprocess
import tempfile
import time
import unittest

ROOT = Path(__file__).resolve().parents[1]
APP = Path(os.environ.get("PERCEPTION_TEST_APP", str(ROOT / "build/perception-dashboard"))).resolve()


def free_port():
    with socket.socket() as probe:
        probe.bind(("127.0.0.1", 0))
        return probe.getsockname()[1]


def identity(path):
    info = path.stat()
    return (path.read_bytes(), info.st_ino, info.st_mtime_ns, stat.S_IMODE(info.st_mode))


def request(port, token, method="GET", path="/api/state", body=None):
    connection = http.client.HTTPConnection("127.0.0.1", port, timeout=1)
    headers = {"Authorization": "Bearer " + token}
    if body is not None:
        body = json.dumps(body).encode()
        headers["Content-Type"] = "application/json"
    try:
        connection.request(method, path, body, headers)
        response = connection.getresponse()
        return response.status, json.loads(response.read())
    finally:
        connection.close()


class PairingPersistence(unittest.TestCase):
    @contextmanager
    def running(self, code_file, cwd):
        port = free_port()
        process = subprocess.Popen([
            str(APP), "--demo", "--headless", "--seconds", "2", "--phone-bind", "127.0.0.1",
            "--phone-port", str(port), "--pair-token-file", str(code_file.resolve()),
        ], cwd=cwd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        try:
            deadline = time.monotonic() + 5
            while time.monotonic() < deadline:
                if process.poll() is not None:
                    stdout, stderr = process.communicate()
                    self.fail(f"Native service exited before pairing test: {process.returncode}; {stdout}; {stderr}")
                if code_file.is_file():
                    # Generation is finished only once the complete line exists.
                    raw = code_file.read_bytes()
                    if raw.endswith(b"\n"):
                        token = raw.splitlines()[0].decode()
                        try:
                            status, state = request(port, token)
                        except (OSError, http.client.HTTPException):
                            status, state = None, {}
                        if status == 200:
                            self.assertTrue(state["demo"] and state["unified"])
                            self.assertFalse(state["motor"]["enabled"], "DEMO must not open a physical motor link")
                            yield port, token
                            break
                time.sleep(.01)
            else:
                self.fail("Native pairing service did not become ready")
        finally:
            try:
                stdout, stderr = process.communicate(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                stdout, stderr = process.communicate(timeout=2)
                self.fail("Native service did not stop at its --seconds deadline")
            self.assertEqual(process.returncode, 0, stdout + stderr)

    def assert_connect_reuses_code(self, port, token, code_file):
        before = identity(code_file)
        sessions = []
        for _ in range(3):
            status, response = request(port, token, "POST", "/api/connect", {})
            self.assertEqual(status, 200)
            self.assertTrue(response["ok"])
            self.assertFalse(response["armed"], "pairing/reconnecting must not arm motors")
            self.assertRegex(response["session"], r"^[0-9a-f]{32}$")
            sessions.append(response["session"])
            self.assertEqual(identity(code_file), before, "session reconnect must not rewrite pairing code")
        self.assertEqual(len(set(sessions)), len(sessions), "connection session changes independently of permanent code")
        return sessions[-1]

    def test_first_generated_code_survives_process_restart_and_cwd_change(self):
        with tempfile.TemporaryDirectory(prefix="pairing-persist-generated-") as temporary:
            root = Path(temporary)
            code_file = root / "permanent-code.txt"
            first_cwd, second_cwd = root / "cwd-first", root / "cwd-restart"
            first_cwd.mkdir(); second_cwd.mkdir()
            self.assertFalse(code_file.exists())
            with self.running(code_file, first_cwd) as (port, token):
                self.assertRegex(token, r"^[0-9]{6}$")
                self.assertEqual(stat.S_IMODE(code_file.stat().st_mode), 0o600)
                original = identity(code_file)
                first_session = self.assert_connect_reuses_code(port, token, code_file)
            self.assertEqual(identity(code_file), original)
            with self.running(code_file, second_cwd) as (port, restarted_token):
                self.assertEqual(restarted_token, token)
                self.assertEqual(request(port, token)[0], 200, "original code still authenticates after process restart")
                self.assertEqual(identity(code_file), original)
                restarted_session = self.assert_connect_reuses_code(port, token, code_file)
                self.assertNotEqual(restarted_session, first_session)
            self.assertEqual(identity(code_file), original)

    def test_preexisting_leading_zero_and_legacy_long_codes_are_never_replaced(self):
        for content in (b"000042\r\n", b"legacy-permanent-pair-token\n"):
            with self.subTest(kind="six-digit-leading-zero" if content.startswith(b"0") else "legacy-long"):
                with tempfile.TemporaryDirectory(prefix="pairing-persist-existing-") as temporary:
                    root = Path(temporary)
                    code_file = root / "existing-code.txt"
                    code_file.write_bytes(content)
                    code_file.chmod(0o600)
                    original = identity(code_file)
                    with self.running(code_file, root) as (port, token):
                        self.assertEqual(token, content.splitlines()[0].decode())
                        self.assertEqual(request(port, token)[0], 200)
                        self.assert_connect_reuses_code(port, token, code_file)
                    self.assertEqual(identity(code_file), original)

    def test_invalid_existing_code_fails_without_overwrite(self):
        with tempfile.TemporaryDirectory(prefix="pairing-persist-invalid-") as temporary:
            root = Path(temporary)
            code_file = root / "invalid-code.txt"
            code_file.write_bytes(b"12345\n")
            code_file.chmod(0o600)
            original = identity(code_file)
            result = subprocess.run([
                str(APP), "--demo", "--headless", "--seconds", "1", "--phone-bind", "127.0.0.1",
                "--phone-port", str(free_port()), "--pair-token-file", str(code_file),
            ], cwd=root, capture_output=True, text=True, timeout=5)
            self.assertNotEqual(result.returncode, 0)
            self.assertIn("Pair code must be 6 digits", result.stderr)
            self.assertEqual(identity(code_file), original)


if __name__ == "__main__":
    unittest.main(verbosity=2)
