import os
"""Exercise the real native server in DEMO mode. No camera, FPGA or motor I/O."""
import base64
import http.client
import io
import json
from pathlib import Path
import socket
import subprocess
import tempfile
import time

ROOT = Path(__file__).resolve().parents[1]
TOKEN = "test-token-1234"


def main():
    with socket.socket() as probe:
        probe.bind(("127.0.0.1", 0))
        port = probe.getsockname()[1]
    with tempfile.TemporaryDirectory(prefix="robot-phone-api-") as temporary:
        token = Path(temporary) / "token"
        token.write_text(TOKEN + "\n")
        process = subprocess.Popen([
            str(ROOT / "build/perception-dashboard"), "--demo", "--headless", "--phone-bind", "127.0.0.1",
            "--phone-port", str(port), "--pair-token-file", str(token), "--seconds", "20",
        ], stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, env=dict(os.environ,FOLLOW_GEOMETRY_FILE=str(ROOT / "tests/synthetic-bearing-fixture.json")))
        checks = 0

        def request(method, path, body=None, auth=TOKEN):
            connection = http.client.HTTPConnection("127.0.0.1", port, timeout=2)
            headers = {"Authorization": "Bearer " + auth}
            if body is not None:
                body = json.dumps(body).encode()
                headers["Content-Type"] = "application/json"
            connection.request(method, path, body, headers)
            response = connection.getresponse()
            code, data = response.status, json.loads(response.read())
            connection.close()
            return code, data

        def check(condition, label):
            nonlocal checks
            assert condition, label
            checks += 1

        try:
            for _ in range(100):
                try:
                    code, state = request("GET", "/api/state")
                    if state["motor"]["link_ok"] and state["imu"]["valid"]:
                        break
                except (OSError, http.client.HTTPException):
                    pass
                time.sleep(0.02)
            else:
                raise AssertionError("Server did not become ready")
            check(code == 200 and state["demo"] and not state["armed"], "safe demo startup")
            check("session" not in state, "telemetry does not expose controller session")
            check(state["unified"] and not state["navigation"]["ready"], "one unified pipeline and honest navigation status")
            check(state["motor"]["left_mask"] == 10 and state["motor"]["invert_mask"] == 15, "final four-wheel layout")
            check(state["vision"]["model"] == "industrial-yolov8s-int8-rk3588.rknn", "current custom YOLOv8s default")
            check(not state["motor"]["enabled"], "demo cannot send physical motor I/O")
            passive = state["motor"]["softcore_diagnostics"]
            check(passive["protocol"] == "NAV1" and passive["passive"] and not passive["received"] and not passive["socket_open"], "demo reports passive FPGA diagnostics honestly without creating hardware traffic")
            check(state["phone"]["encodes"] == 0 and state["phone"]["snapshot_copies"] == 0, "no video client means zero image copies and zero JPEG work")
            check(request("GET", "/api/state", auth="wrong-token")[0] == 401, "token gate")
            code, state = request("POST", "/api/connect", {})
            session, sequence = state["session"], 0
            check(code == 200 and len(session) == 32, "session handshake")
            time.sleep(.05)

            def control(op, **fields):
                nonlocal sequence
                sequence += 1
                return request("POST", "/api/control", dict(session=session, seq=sequence, op=op, **fields))

            code, state = control("arm")
            check(state["armed"], "explicit arm")
            check("session" not in state, "control response does not expose controller session")
            code, busy = request("POST", "/api/connect", {})
            check(code == 409, "single active controller")
            check("session" not in busy, "rejected second controller cannot steal session")
            check("session" not in request("GET", "/api/state")[1], "polling cannot steal active session")
            code, rejected = request("POST", "/api/control", dict(session="not-the-owner", seq=1, op="arm"))
            check(code == 403 and "session" not in rejected, "invalid owner response hides live session")
            for _ in range(7):
                code, state = control("drive", throttle=.5, steer=.2, hold=False)
                time.sleep(.05)
            code, state = request("GET", "/api/state")
            check(state["motor"]["left"] > state["motor"]["right"] > 0, "right-turn differential")
            check(max(abs(state["motor"]["left"]), abs(state["motor"]["right"])) <= .6, "duty limit")
            code, state = control("drive", throttle=0, steer=0, hold=False)
            check(state["armed"] and state["motor"]["left"] == state["motor"]["right"] == 0, "release is immediate zero, remains armed")
            old = dict(session=session, seq=1, op="drive", throttle=1, steer=0)
            check(request("POST", "/api/control", old)[0] == 409, "old sequence rejected")
            code, state = control("drive", throttle=.4, steer=0, hold=True)
            check(code == 200 and state["mode"] == "heading_hold", "straight heading hold")
            time.sleep(.5)
            code, state = request("GET", "/api/state")
            check(not state["armed"] and state["motor"]["left"] == state["motor"]["right"] == 0, "control lease expires without renewing from polling")
            check(control("arm")[0] == 200, "fresh manual rearm")
            for direction, sign in (("cw", 1), ("ccw", -1)):
                for _ in range(8):
                    code, state = control("rotate", direction=direction, speed=1)
                    time.sleep(.05)
                code, state = request("GET", "/api/state")
                left, right = state["motor"]["left"], state["motor"]["right"]
                check(code == 200 and state["mode"] == "rotate_" + direction, "explicit rotation mode")
                check(left * sign > 0 and right * sign < 0 and abs(left + right) < .001, "rotation directions")
                check(abs(left) <= .35 and abs(right) <= .35, "rotation actual duty capped")
                code, state = control("idle")
                check(state["armed"] and state["motor"]["left"] == state["motor"]["right"] == 0, "rotation release immediate zero")
            control("rotate", direction="cw", speed=.3)
            time.sleep(.45)
            code, state = request("GET", "/api/state")
            check(not state["armed"] and state["motor"]["left"] == state["motor"]["right"] == 0, "rotation independent lease expires")
            check(control("arm")[0] == 200, "fresh rearm after rotation")

            video = http.client.HTTPConnection("127.0.0.1", port, timeout=2)
            video.request("GET", "/api/video.mjpg", headers={"Authorization": "Bearer " + TOKEN})
            response = video.getresponse()
            check(response.status == 200, "authenticated stream")
            check(response.readline().strip() == b"--robotframe", "multipart boundary")
            headers = {}
            while True:
                line = response.readline()
                if line in (b"\r\n", b"\n"):
                    break
                key, value = line.decode().split(":", 1)
                headers[key.lower()] = value.strip()
            meta = json.loads(base64.b64decode(headers["x-frame-meta"]))
            jpeg = response.read(int(headers["content-length"]))
            check(meta["frame_id"] == int(headers["x-frame-id"]), "frame metadata synchronization")
            check(meta["detections_frame_id"] > 0 and meta["box_age_ms"] < 350 and meta["box_max_age_ms"] == 350, "independent detection source timestamps")
            check(meta["width"] == 864 and meta["height"] == 486 and meta["age_ms"] < 500, "fresh source dimensions")
            check(jpeg.startswith(b"\xff\xd8") and jpeg.endswith(b"\xff\xd9"), "complete JPEG frame")
            phone_perf = request("GET", "/api/state")[1]["phone"]
            check(phone_perf["encodes"] > 0 and phone_perf["snapshot_copies"] > 0, "opening video resumes bounded image copy and JPEG work")
            try:
                from PIL import Image
                image = Image.open(io.BytesIO(jpeg))
                image.load()
                check(image.size == (864, 486), "JPEG actual decode")
            except ImportError:
                pass
            check(len(meta["detections"]) == 2 and len({d["id"] for d in meta["detections"]}) == 2, "stable per-eye IDs")
            chosen = meta["detections"][0]
            code, state = control("select", frame_id=meta["frame_id"], target_id=chosen["id"])
            check(code == 200 and state["selected_id"] == chosen["id"], "select by frame and track")
            for _ in range(3):
                code, state = control("idle")
                check(code == 200 and state["selected_id"] == chosen["id"] and state["motor"]["left"] == state["motor"]["right"] == 0, "idle keeps selection and never moves")
                time.sleep(.05)
            for _ in range(4):
                code, state = control("follow")
                check(code == 200 and state["mode"] == "follow", "follow remains associated")
                time.sleep(.05)
            response.close()
            video.close()
            code, state = control("zero")
            check(code == 200 and not state["armed"], "zero queues main-owned IMU action and disarms")
            time.sleep(.08)
            code, state = request("GET", "/api/state")
            # DEMO continues rotating at up to 24.5 deg/s during the 80 ms
            # observation delay, plus one 33 ms publication interval.
            check(abs(state["imu"]["yaw"]) < 3.5, "same IMU instance heading zero while demo keeps turning")
            check(control("arm")[0] == 200, "rearm after reference zero")
            code, state = control("stop", emergency=True)
            check(not state["armed"] and state["motor"]["left"] == state["motor"]["right"] == 0, "STOP disarms")
            check(control("drive", throttle=.5, steer=0)[0] == 409, "cannot restart using drive after STOP")
            code, state = control("configure", left_mask=5, invert_mask=12)
            check(code == 200 and not state["armed"] and state["motor"]["left_mask"] == 5, "mapping change disarms")
            code, state = request("POST", "/api/connect", {})
            check(code == 200 and state["session"] != session, "inactive controller replacement")
            check(control("arm")[0] == 403, "old owner rejected")
            session, sequence = state["session"], 0
            time.sleep(.05)
            check(control("arm")[0] == 200, "new owner sequence restarts safely")
            check(control("rotate", direction="cw", speed=-1)[0] == 409, "invalid rotation rejected")
            check(not request("GET", "/api/state")[1]["armed"], "invalid rotation disarms")
            check(control("navigate")[0] == 409, "unconnected navigation cannot start motors")
            check(control("arm")[0] == 200, "explicit arm before physical-unit navigation")
            sequence += 1
            code, state = request("POST", "/api/navigation/claim", dict(session=session, seq=sequence, op="claim"))
            ticket = state.get("navigation_ticket", "")
            check(code == 200 and ticket and state["motor"]["twist_mode"], "production claim installs zero MOT4 without movement")
            sequence += 1
            code, state = request("POST", "/api/navigation/twist", dict(session=session, seq=sequence, op="twist", navigation_ticket=ticket, linear_mps=.3, angular_rps=-.2))
            check(code == 200 and state["mode"] == "navigation_velocity" and state["motor"]["linear_mps"] == .3 and state["motor"]["angular_rps"] == -.2, "production private API uses physical velocity units")
            check(state["motor"]["left"] == state["motor"]["right"] == 0, "physical targets never masquerade as requested PWM")
            check(control("drive", throttle=.3, steer=.2)[0] == 200, "manual takeover remains compatible")
            state = request("GET", "/api/state")[1]
            check(not state["motor"]["twist_mode"] and state["motor"]["linear_mps"] == 0, "manual takeover clears old velocity target")
            check(control("drive", throttle=0, steer=0)[0] == 200, "explicit zero before second claim")
            sequence += 1
            code, state = request("POST", "/api/navigation/claim", dict(session=session, seq=sequence, op="claim"))
            ticket = state.get("navigation_ticket", "")
            check(code == 200 and ticket, "second explicit zero claim")
            sequence += 1
            code, state = request("POST", "/api/navigation/twist", dict(session=session, seq=sequence, op="twist", navigation_ticket=ticket, linear_mps=.301, angular_rps=0))
            check(code == 409 and not state["armed"], "overrange velocity refuses and disarms")
            check(request("GET", "/unknown")[0] == 404, "unknown path")
            check(TOKEN not in (ROOT / "phone/phone_service.cc").read_text(), "no embedded pairing secret")
            print(f"PASS {checks} unified HTTP/video/control checks (DEMO, no hardware)")
        finally:
            process.terminate()
            try:
                stdout, stderr = process.communicate(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                raise
            assert TOKEN not in stdout + stderr, "Pairing token must not leak into logs"
            if process.returncode not in (0, -15):
                raise AssertionError((process.returncode, stdout, stderr))


if __name__ == "__main__":
    main()
