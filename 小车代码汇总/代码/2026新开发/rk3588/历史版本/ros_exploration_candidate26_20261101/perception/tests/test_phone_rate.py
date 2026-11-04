#!/usr/bin/env python3
"""Measure the real publisher using localhost and simulated motor outputs only."""
import base64
import http.client
import json
from pathlib import Path
import socket
import subprocess
import tempfile
import time

ROOT = Path(__file__).resolve().parents[1]
checks = 0


def check(value, message):
    global checks
    checks += 1
    assert value, message


class Video:
    def __init__(self, port):
        self.connection = http.client.HTTPConnection("127.0.0.1", port, timeout=3)
        self.connection.request("GET", "/api/video.mjpg", headers={"Authorization": "Bearer 012345"})
        self.response = self.connection.getresponse()
        check(self.response.status == 200, "MJPEG rejected")

    def image(self):
        while self.response.readline().strip() != b"--robotframe":
            pass
        headers = {}
        while True:
            line = self.response.readline()
            if line == b"\r\n":
                break
            check(bool(line), "MJPEG closed during headers")
            name, value = line.decode().split(":", 1)
            headers[name.lower()] = value.strip()
        data = self.response.read(int(headers["content-length"]))
        check(data[:2] == b"\xff\xd8" and data[-2:] == b"\xff\xd9", "JPEG payload invalid")
        check(self.response.read(2) == b"\r\n", "MJPEG trailer invalid")
        return json.loads(base64.b64decode(headers["x-frame-meta"]))

    def close(self):
        self.response.close()
        self.connection.close()


def run(mode):
    with socket.socket() as available:
        available.bind(("127.0.0.1", 0))
        port = available.getsockname()[1]
    with tempfile.TemporaryDirectory(prefix="v35-phone-rate-") as tmp:
        token = Path(tmp) / "token"
        token.write_text("012345\n")
        process = subprocess.Popen([str(ROOT / "build/phone-rate-probe"), str(port), str(token), mode],
                                   stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        video = None

        def request(method="GET", path="/api/state", data=None):
            connection = http.client.HTTPConnection("127.0.0.1", port, timeout=2)
            connection.request(method, path, json.dumps(data) if data else None,
                               {"Authorization": "Bearer 012345"})
            response = connection.getresponse()
            answer = response.status, json.loads(response.read())
            connection.close()
            return answer

        try:
            for _ in range(100):
                try:
                    _, state = request()
                    break
                except OSError:
                    time.sleep(.01)
            else:
                raise AssertionError("service did not start")
            time.sleep(.15)
            _, state = request()
            check(state["phone"]["encodes"] == 0, "publisher encodes without clients")
            video = Video(port)
            first = video.image()
            started = time.monotonic()
            count = 0
            ids = [first["frame_id"]]
            ages = []
            while time.monotonic() - started < 1.8:
                frame = video.image()
                count += 1
                ids.append(frame["frame_id"])
                ages.append(frame["age_ms"])
            elapsed = time.monotonic() - started
            fps = count / elapsed
            expected = 30 if mode in ("auto30", "off30") else 15 if mode == "cap15" else 40
            check(all(b > a for a, b in zip(ids, ids[1:])), "duplicate or regressing image IDs")
            _, state = request()
            check(abs(state["phone"]["target_fps"] - expected) < .01, (mode, "wrong target", state["phone"]))
            if mode == "slow40":
                check(8 < fps < 18, ("slow JPEG did not limit throughput", fps))
                check(state["phone"]["superseded"] > 5, "slow encode must overwrite latest slot")
                check(max(b-a for a, b in zip(ids, ids[1:])) >= 2, "slow encoder accumulated old frames")
                check(max(ages) < 200, ("slow encoder source age is unbounded", max(ages)))
            else:
                check(expected * .78 < fps <= expected * 1.08, (mode, "unexpected delivered FPS", fps))
            print(f"RATE mode={mode} target={expected} received_fps={fps:.3f} frames={count} max_age_ms={max(ages)} encodes={state['phone']['encodes']}", flush=True)
            video.close()
            video = None
            # A nonreading receiver must not hold the control state mutex or
            # build a source/JPEG queue. Use small TCP receive capacity to force it.
            if mode == "auto40":
                slow = socket.socket()
                slow.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4096)
                slow.connect(("127.0.0.1", port))
                slow.sendall(b"GET /api/video.mjpg HTTP/1.1\r\nHost: localhost\r\nAuthorization: Bearer 012345\r\n\r\n")
                _, connected = request("POST", "/api/connect", {})
                session = connected["session"]
                time.sleep(.08)  # Let the independent control loop seed demo ACK.
                latencies = []
                for sequence in range(1, 16):
                    begin = time.monotonic()
                    code, state = request("POST", "/api/control", dict(
                        session=session, seq=sequence, op="arm" if sequence == 1 else "drive",
                        throttle=0.1, steer=0, hold=False))
                    latencies.append(time.monotonic()-begin)
                    check(code == 200 and state["armed"], "slow video disconnected/disarmed healthy control")
                    time.sleep(.07)
                _, transport = request()
                check(transport["phone"]["slow_disconnects"] > 0,
                      "nonreading client must be disconnected, not hold a TCP video backlog")
                check(max(latencies) < .4, ("slow client blocked control API", max(latencies)))
                slow.close()
                time.sleep(.1)
                video = Video(port)
                resumed = video.image()
                _, current = request()
                check(resumed["age_ms"] < 200 and current["vision"]["frame_id"] - resumed["frame_id"] < 10,
                      "reconnected video replayed queued old frames")
                print(f"SLOW_CLIENT control_requests=15 max_control_ms={max(latencies)*1000:.2f} resumed_age_ms={resumed['age_ms']} latest_only=yes", flush=True)
                video.close()
                video = None
            time.sleep(.3)
            _, stopped = request()
            before = stopped["phone"]["encodes"]
            time.sleep(.2)
            _, stopped = request()
            check(stopped["phone"]["video_clients"] == 0 and stopped["phone"]["encodes"] == before,
                  "publisher continued encoding without a client")
        finally:
            if video:
                video.close()
            process.terminate()
            stdout, stderr = process.communicate(timeout=5)
            check(process.returncode == 0, (stdout, stderr))
            check(b"012345" not in stdout + stderr, "pairing secret leaked")


if __name__ == "__main__":
    for mode in ("auto30", "auto40", "cap15", "off30", "slow40"):
        run(mode)
    print(f"PASS production auto/capped publisher, slow-client control isolation and link-wrapped slow JPEG: {checks} checks; host demo fixtures only")
