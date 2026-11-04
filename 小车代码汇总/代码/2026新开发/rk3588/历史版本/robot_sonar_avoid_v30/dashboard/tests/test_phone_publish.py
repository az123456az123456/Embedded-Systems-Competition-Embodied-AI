import os
"""A live preview must not make a frozen detector fresh or keep following moving."""
import base64
import http.client
import json
from pathlib import Path
import socket
import subprocess
import tempfile
import time

ROOT = Path(__file__).resolve().parents[1]

def main():
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        port = s.getsockname()[1]
    with tempfile.TemporaryDirectory() as tmp:
        token = Path(tmp) / "pair.txt"
        token.write_text("012345\n")
        process = subprocess.Popen([str(ROOT / "build/phone-publish-probe"), str(port), str(token)], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        def request(method, path, body=None):
            c = http.client.HTTPConnection("127.0.0.1", port, timeout=2)
            c.request(method, path, json.dumps(body) if body else None, {"Authorization": "Bearer 012345"})
            response = c.getresponse(); result = response.status, json.loads(response.read()); c.close()
            return result
        try:
            for _ in range(50):
                try:
                    _, state = request("GET", "/api/state")
                    if state["vision"]["valid"]: break
                except OSError: pass
                time.sleep(.01)
            else: raise AssertionError("bridge did not publish")
            _, owner = request("POST", "/api/connect", {})
            session, sequence = owner["session"], 0
            def control(op, **fields):
                nonlocal sequence
                sequence += 1
                return request("POST", "/api/control", dict(session=session, seq=sequence, op=op, **fields))
            time.sleep(.12)
            assert control("arm")[0] == 200
            video = http.client.HTTPConnection("127.0.0.1", port, timeout=2)
            video.request("GET", "/api/video.mjpg", headers={"Authorization": "Bearer 012345"})
            response = video.getresponse()
            def image():
                while response.readline().strip() != b"--robotframe": pass
                headers = {}
                while True:
                    line = response.readline()
                    if line == b"\r\n": break
                    key, value = line.decode().split(":", 1); headers[key.lower()] = value.strip()
                meta = json.loads(base64.b64decode(headers["x-frame-meta"]))
                response.read(int(headers["content-length"])); response.read(2)
                return meta
            meta = image()
            assert len(meta["detections"]) == 1 and meta["box_age_ms"] <= 350
            assert meta["detections"][0]["box"] == [216, 200, 286, 300], "phone boxes must use the same compensated coordinates as native preview, preserving source index after validation"
            chosen = meta["detections"][0]["id"]
            assert control("select", frame_id=meta["frame_id"], target_id=chosen)[0] == 200
            # Selection must survive fresh detector results and the client's
            # stationary idle commands, rather than being a one-frame flash.
            selection_end = time.monotonic() + .7
            selected_video_frames = 0
            detection_ids = set()
            while time.monotonic() < selection_end:
                code, state = control("idle")
                assert code == 200 and state["selected_id"] == chosen
                assert state["motor"]["left"] == state["motor"]["right"] == 0
                metadata = image()
                detection_ids.add(metadata["detections_frame_id"])
                assert metadata["detections"][0]["id"] == chosen
                if metadata["selected_id"] == chosen:
                    selected_video_frames += 1
                else:
                    assert selected_video_frames == 0, "selection must not reset on subsequent video frames"
            assert selected_video_frames >= 5 and len(detection_ids) >= 4
            # Continue renewing the motion lease while the detector stops. Only
            # detection source age should stop following; the preview stays live.
            end = time.monotonic() + 1.8
            stopped_for_vision = False
            while time.monotonic() < end:
                code, state = control("follow")
                if state["vision"]["valid"] and state["mode"] == "follow":
                    assert state["motor"]["left"] == state["motor"]["right"], "display-only compensation must not change raw AI control association"
                if not state["vision"]["valid"]:
                    stopped_for_vision = state["motor"]["left"] == state["motor"]["right"] == 0
                time.sleep(.05)
            assert stopped_for_vision, "live preview must not refresh detection age"
            current = request("GET", "/api/state")[1]
            assert current["vision"]["frame_id"] > meta["frame_id"] + 20
            assert not current["vision"]["valid"] and current["selected_id"] == 0
            latest = meta
            for _ in range(100):
                latest = image()
                if latest["box_age_ms"] > 350: break
            assert latest["box_age_ms"] > 350 and latest["detections"] == []
            assert control("select", frame_id=meta["frame_id"], target_id=meta["detections"][0]["id"])[0] == 409
            response.close(); video.close()
            print("PASS shared publisher: compensated preview/source-index association, persistent selected ID across fresh detections/idle/video metadata, raw AI control IDs, independent detection timestamps, stale-box expiry, follow stop, old-frame rejection and fixed leading-zero pairing")
        finally:
            process.terminate(); out, err = process.communicate(timeout=5)
            assert b"012345" not in out + err
            assert process.returncode == 0, (out, err)

if __name__ == "__main__": main()
