"""Real AF_PACKET IMU+MOT1/ACK1 test in an isolated user/network namespace.

Run through unshare --user --map-root-user --net with
ACG138_TEST_NETNS=1. It never opens the board or an existing host interface.
"""
import binascii
import http.client
import json
import os
from pathlib import Path
import select
import socket
import struct
import subprocess
import tempfile
import threading
import time

ROOT = Path(__file__).resolve().parents[1]
HOST_MAC = bytes.fromhex("02 ac 13 80 01 02")
FPGA_MAC = bytes.fromhex("02 ac 13 80 00 01")
SPLIT = os.environ.get("ACG138_TEST_SPLIT_INTERFACES") == "1"

def main():
    assert os.environ.get("ACG138_TEST_NETNS") == "1", "private network namespace required"
    # The caller creates a fresh namespace with only loopback. Refuse any
    # existing physical/veth interfaces before creating test-only devices.
    # /sys may still be the parent's mount; netlink reflects this namespace.
    links = json.loads(subprocess.check_output(["ip", "-j", "link"], text=True))
    assert {item["ifname"] for item in links} == {"lo"}
    def ip(*args): subprocess.run(["ip", *args], check=True)
    ip("link", "set", "lo", "up")
    ip("link", "add", "rk-test", "type", "veth", "peer", "name", "fpga-test")
    for name, mac in (("rk-test", HOST_MAC), ("fpga-test", FPGA_MAC)):
        ip("link", "set", name, "address", ":".join(f"{b:02x}" for b in mac))
        ip("link", "set", name, "up")
    wire = socket.socket(socket.AF_PACKET, socket.SOCK_RAW, socket.htons(3))
    wire.bind(("fpga-test", 0)); wire.setblocking(False)
    stopped, no_acks, no_imu = threading.Event(), threading.Event(), threading.Event()
    received, acks, imu_sent, errors = [], [], [], []
    def fpga():
        sequence, deadline = 0, time.monotonic()
        while not stopped.is_set():
            now = time.monotonic()
            if now >= deadline:
                deadline = now + .01
                if not no_imu.is_set():
                    sequence += 1; payload = bytearray(64)
                    payload[:4] = b"IMU9"; payload[4:8] = bytes([1, 0x92, 0, 15])
                    struct.pack_into(">II", payload, 8, sequence, sequence * 10)
                    struct.pack_into(">h", payload, 20, 16384)
                    payload[39], payload[41] = 0x71, 0x68
                    struct.pack_into(">I", payload, 52, sequence * 10)
                    struct.pack_into(">H", payload, 58, 100)
                    wire.send(b"\xff" * 6 + FPGA_MAC + b"\x88\xb5" + payload)
                    imu_sent.append(sequence)
            ready, _, _ = select.select([wire], [], [], .003)
            if not ready: continue
            try: frame = wire.recv(65536)
            except BlockingIOError: continue
            if frame[12:14] != b"\x88\xb6" or frame[14:18] != b"MOT1": continue
            payload = frame[14:32]
            if binascii.crc_hqx(payload[:16], 0xffff) != struct.unpack_from(">H", payload, 16)[0]:
                errors.append("bad MOT1 CRC"); continue
            received.append(dict(sequence=struct.unpack_from(">I", payload, 6)[0],
                                 armed=bool(payload[5] & 1), left=struct.unpack_from(">h", payload, 10)[0],
                                 right=struct.unpack_from(">h", payload, 12)[0], mask=payload[14], invert=payload[15]))
            if no_acks.is_set(): continue
            reply = bytearray(payload); reply[:4] = b"ACK1"; reply[5] = 4 | (payload[5] & 1)
            struct.pack_into(">H", reply, 16, binascii.crc_hqx(reply[:16], 0xffff))
            wire.send(HOST_MAC + FPGA_MAC + b"\x88\xb6" + reply + bytes(28)); acks.append(received[-1]["sequence"])
    thread = threading.Thread(target=fpga); thread.start()
    with tempfile.TemporaryDirectory() as tmp:
        token = Path(tmp) / "token"; token.write_text("012345\n")
        with socket.socket() as s: s.bind(("127.0.0.1", 0)); port = s.getsockname()[1]
        binary = os.environ.get("PERCEPTION_TEST_APP", str(ROOT / "build/perception-dashboard"))
        log = open(Path(tmp) / "app.log", "w+")
        command = [binary, "--headless", "--no-detect", "--camera", "/dev/null", "--interface", "rk-test", "--phone-bind", "127.0.0.1", "--phone-port", str(port), "--pair-token-file", str(token), "--seconds", "20"]
        if SPLIT: command += ["--imu-interface", "no-such-imu", "--motor-interface", "rk-test"]
        process = subprocess.Popen(command, stdout=log, stderr=log)
        def request(method, path, body=None):
            c = http.client.HTTPConnection("127.0.0.1", port, timeout=2)
            c.request(method, path, json.dumps(body) if body else None, {"Authorization": "Bearer 012345"})
            r = c.getresponse(); value = r.status, json.loads(r.read()); c.close(); return value
        checks = 0
        def check(condition, label):
            nonlocal checks
            assert condition, label; checks += 1
        try:
            until = time.monotonic() + 5
            while time.monotonic() < until:
                try:
                    _, state = request("GET", "/api/state")
                    if state["motor"]["link_ok"] and (SPLIT or state["imu"]["valid"] and not state["imu"]["calibrating"]): break
                except (OSError, http.client.HTTPException): pass
                time.sleep(.02)
            else:
                log.flush(); log.seek(0)
                raise AssertionError(("raw motor+IMU startup failed", state if 'state' in locals() else {}, log.read()))
            check(state["motor"]["enabled"] and len(acks) > 0, "real AF_PACKET ACK window established")
            if SPLIT:
                check(not state["imu"]["socket_open"] and not state["imu"]["valid"] and state["motor"]["link_ok"], "failed IMU socket does not suppress motor ACK")
            else:
                check(state["imu"]["valid"] and state["imu"]["accel"][2] > 9.7, "real raw IMU decoded, calibrated and published")
            _, owner = request("POST", "/api/connect", {})
            session, sequence = owner["session"], 0
            def control(op, **fields):
                nonlocal sequence
                sequence += 1
                return request("POST", "/api/control", dict(session=session, seq=sequence, op=op, **fields))
            time.sleep(.04)
            check(control("arm")[0] == 200, "real ACK permits arm")
            if SPLIT:
                code, rejected = control("drive", throttle=.4, steer=0, hold=True)
                check(code == 409 and rejected["error"] == "imu_not_ready_for_heading_hold", "heading hold refusal identifies missing IMU")
                check(control("arm")[0] == 200, "manual mode can explicitly rearm without IMU")
            for _ in range(8): control("drive", throttle=.6, steer=0, hold=False); time.sleep(.05)
            check(any(v["armed"] and v["left"] > 200 and v["right"] > 200 for v in received), "HTTP joystick becomes transmitted positive MOT1 PWM")
            for direction, sign in (("cw", 1), ("ccw", -1)):
                control("drive", throttle=0, steer=0, hold=False); time.sleep(.06)
                marker = len(received)
                for _ in range(8): control("rotate", direction=direction, speed=.3); time.sleep(.05)
                check(any(v["armed"] and v["left"] * sign > 100 and v["right"] * sign < -100 for v in received[marker:]), "raw " + direction + " polarity")
            control("drive", throttle=0, steer=0, hold=False); time.sleep(.08)
            check(received[-1]["left"] == received[-1]["right"] == 0, "release transmits PWM zero")
            control("drive", throttle=.4, steer=0, hold=False); time.sleep(.45)
            check(not received[-1]["armed"] and received[-1]["left"] == received[-1]["right"] == 0, "350 ms command loss transmits disarm even with live IMU and ACK")
            if not SPLIT and "fresh" in state["imu"]:
                no_imu.set(); time.sleep(.36)
                boundary = request("GET", "/api/state")[1]["imu"]
                check(boundary["valid"] and not boundary["fresh"] and 300 < boundary["sample_age_ms"] < 500, "telemetry separates display validity from 300 ms heading-control freshness")
                no_imu.clear(); time.sleep(.08)
            check(control("arm")[0] == 200, "rearm with continuing real ACK")
            no_acks.set()
            end = time.monotonic() + .5
            while time.monotonic() < end: control("drive", throttle=.5, steer=0, hold=False); time.sleep(.04)
            check(not request("GET", "/api/state")[1]["armed"], "ACK loss disarms despite continuing joystick commands")
            check(not received[-1]["armed"] and received[-1]["left"] == received[-1]["right"] == 0, "ACK-loss STOP observed on the real virtual wire")
            no_imu.set(); time.sleep(.6)
            check(not request("GET", "/api/state")[1]["imu"]["valid"], "real IMU loss becomes invalid, no cached fake freshness")
            check(all(v["mask"] == 10 and v["invert"] == 0 for v in received), "final wheel mapping propagated")
            check(not errors, "all raw transmitted MOT1 CRCs valid")
            print(f"PASS {checks} real AF_PACKET checks, MOT1={len(received)} ACK1={len(acks)} IMU9={len(imu_sent)} split_interfaces={SPLIT}; isolated veth, no board or motors")
        finally:
            process.terminate(); process.wait(timeout=5); stopped.set(); thread.join(timeout=2); wire.close(); log.close()
            ip("link", "del", "rk-test")

if __name__ == "__main__": main()
