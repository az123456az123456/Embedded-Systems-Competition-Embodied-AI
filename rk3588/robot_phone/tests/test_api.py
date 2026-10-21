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
            str(ROOT / "build/robot-phone-server"), "--demo", "--bind", "127.0.0.1",
            "--port", str(port), "--token-file", str(token), "--seconds", "20",
        ], stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
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
            check(not state["motor"]["enabled"], "demo cannot send physical motor I/O")
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
            check(meta["width"] == 864 and meta["height"] == 486 and meta["age_ms"] < 500, "fresh source dimensions")
            check(jpeg.startswith(b"\xff\xd8") and jpeg.endswith(b"\xff\xd9"), "complete JPEG frame")
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
            check(code == 200 and not state["armed"] and abs(state["imu"]["roll"]) < .0001 and abs(state["imu"]["pitch"]) < .0001 and abs(state["imu"]["yaw"]) < .0001, "zero reference also disarms")
            check(control("arm")[0] == 200, "rearm after reference zero")
            code, state = control("stop", emergency=True)
            check(not state["armed"] and state["motor"]["left"] == state["motor"]["right"] == 0, "STOP disarms")
            check(control("drive", throttle=.5, steer=0)[0] == 409, "cannot restart using drive after STOP")
            code, state = control("configure", left_mask=5, invert_mask=12)
            check(code == 200 and not state["armed"] and state["motor"]["left_mask"] == 5, "mapping change disarms")
            code, state = request("POST", "/api/connect", {})
            check(code == 200 and state["session"] != session, "inactive controller replacement")
            check(control("arm")[0] == 403, "old owner rejected")
            check(request("GET", "/unknown")[0] == 404, "unknown path")
            print(f"PASS {checks} native HTTP/video/control API checks (DEMO, no hardware)")
        finally:
            process.terminate()
            try:
                stdout, stderr = process.communicate(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                raise
            if process.returncode not in (0, -15):
                raise AssertionError((process.returncode, stdout, stderr))


if __name__ == "__main__":
    main()
