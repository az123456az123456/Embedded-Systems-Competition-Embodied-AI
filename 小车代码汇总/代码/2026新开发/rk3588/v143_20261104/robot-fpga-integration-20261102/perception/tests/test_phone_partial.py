#!/usr/bin/env python3
"""Real localhost HTTP/MJPEG regression: partial display and complete control.

The service, authentication, Controller, tracking, aliases, expiry and JPEG
publisher are production code. Sensor/NPU observations are deliberate fixtures;
demo mode prevents opening a hardware motor socket.
"""
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


def check(condition, message):
    global checks
    checks += 1
    assert condition, message


def main():
    with socket.socket() as sock:
        sock.bind(("127.0.0.1", 0))
        port = sock.getsockname()[1]
    with tempfile.TemporaryDirectory(prefix="v35-phone-partial-") as tmp:
        token = Path(tmp) / "pair.txt"
        token.write_text("012345\n")
        freeze = Path(tmp) / "freeze-control"
        process = subprocess.Popen(
            [str(ROOT / "build/phone-partial-probe"), str(port), str(token),
             str(ROOT / "tests/synthetic-bearing-fixture.json"), str(freeze)],
            stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        video = response = None

        def request(method, path, body=None):
            connection = http.client.HTTPConnection("127.0.0.1", port, timeout=2)
            connection.request(method, path, json.dumps(body) if body else None,
                               {"Authorization": "Bearer 012345"})
            result = connection.getresponse()
            answer = result.status, json.loads(result.read())
            connection.close()
            return answer

        try:
            for _ in range(100):
                try:
                    _, state = request("GET", "/api/state")
                    if state["vision"]["valid"]:
                        break
                except OSError:
                    pass
                time.sleep(.01)
            else:
                raise AssertionError("production service did not publish")
            _, owner = request("POST", "/api/connect", {})
            session, sequence = owner["session"], 0

            def control(operation, **fields):
                nonlocal sequence
                sequence += 1
                return request("POST", "/api/control", dict(
                    session=session, seq=sequence, op=operation, **fields))

            time.sleep(.15)  # Allow fresh IMU history and republish after connect.
            check(control("arm")[0] == 200, "demo arm rejected")
            video = http.client.HTTPConnection("127.0.0.1", port, timeout=2)
            video.request("GET", "/api/video.mjpg", headers={"Authorization": "Bearer 012345"})
            response = video.getresponse()

            def image():
                for _ in range(50):
                    line = response.readline()
                    check(bool(line), "MJPEG closed while waiting for a frame")
                    if line.strip() == b"--robotframe":
                        break
                else:
                    raise AssertionError("MJPEG boundary missing")
                headers = {}
                while True:
                    line = response.readline()
                    check(bool(line), "MJPEG closed during headers")
                    if line == b"\r\n":
                        break
                    name, value = line.decode().split(":", 1)
                    headers[name.lower()] = value.strip()
                metadata = json.loads(base64.b64decode(headers["x-frame-meta"]))
                payload = response.read(int(headers["content-length"]))
                check(payload[:2] == b"\xff\xd8" and payload[-2:] == b"\xff\xd9", "real JPEG missing")
                check(response.read(2) == b"\r\n", "MJPEG trailer missing")
                return metadata

            first = image()
            check(first["preview_pair_status"] == "PAIR_PARTIAL", "fixture must never publish a complete display")
            check(first["detections_pair_status"] == "PAIR_COMPLETE", "complete cache was hidden by newer partial")
            check(first["complete_mask"] == 3 and first["control_eligible"], "control channel not a timely complete pair")
            check(first["detections_frame_id"] < first["partial_frame_id"], "source channels were accidentally merged")
            check(len(first["detections"]) == 2 and len(first["partial_detections"]) == 1, "separate box layers missing")
            chosen = first["detections"][0]["id"]
            check(chosen > 0 and first["detections"][0]["selectable"], "complete box cannot be selected")
            code, state = control("select", frame_id=first["frame_id"], target_id=chosen)
            check(code == 200 and state["selected_id"] == chosen,
                  ("selection starved behind partial events", code, state, first))
            bearing = state["selected_target"]
            check(bearing["angle_valid"] and abs(bearing["u_px"] - 480) < .01,
                  ("service did not invert tensor letterbox into physical horizontal pixel", bearing))
            check(abs(bearing["v_px"] - 532.5) < .01,
                  "service did not invert 27/432 preview viewport before physical vertical pixel")
            previous_frame = previous_partial = previous_update = previous_control = 0
            seen_control = set()
            for _ in range(18):
                metadata = image()
                check(metadata["preview_pair_status"] == "PAIR_PARTIAL", "fixture display changed to complete")
                check(metadata["frame_id"] >= previous_frame and metadata["partial_frame_id"] >= previous_partial, "public frame ID went backward")
                check(metadata["partial_update_id"] >= previous_update and metadata["update_id"] >= previous_control, "event ID went backward")
                if metadata["frame_id"] > previous_frame:
                    check(metadata["partial_update_id"] > previous_update, "a new partial frame reused an event ID")
                previous_frame, previous_partial = metadata["frame_id"], metadata["partial_frame_id"]
                previous_update, previous_control = metadata["partial_update_id"], metadata["update_id"]
                check(metadata["detections_frame_id"] < metadata["partial_frame_id"], "partial mislabeled as a complete control pair")
                check(metadata["partial_control_eligible"] is False, "partial has control authority")
                check(all(d["id"] == 0 and not d["selectable"] for d in metadata["partial_detections"]), "partial acquired a selectable ID")
                check(metadata["selected_id"] == chosen and any(d["id"] == chosen for d in metadata["detections"]), "selected orange ID disappeared on a partial update")
                check(metadata["source_epoch"] == metadata["partial_source_epoch"] == 7, "source epoch lost")
                check(metadata["source_sequence"] == metadata["detections_frame_id"], "complete source metadata taken from partial")
                check(metadata["partial_source_sequence"] == metadata["partial_frame_id"], "partial source metadata taken from complete")
                seen_control.add(metadata["detections_frame_id"])
                code, state = control("idle")
                check(code == 200 and state["selected_id"] == chosen, "idle or telemetry cleared selected ID")
                vision = state["vision"]
                check(vision["preview_pair_status"] == "PAIR_PARTIAL" and vision["complete_mask"] == 3, "state mixed partial mask into complete status")
                check(vision["control_eligible"] and vision["partial_complete_mask"] in (1, 2), "state control readiness corrupted")
                code, rejected = control("select", frame_id=metadata["frame_id"], target_id=0)
                check(code == 409 and rejected["selected_id"] == chosen, "unselectable partial event cleared existing target")
            check(len(seen_control) >= 4, "complete control channel stopped advancing while partial display advanced")
            # A real stable display can outlive one object's raw detection.
            # Select the left eye, drop only that object while the right eye
            # continues, and tap its held box using the current preview alias.
            visible = image()
            left = next(d for d in visible["display_detections"] if d["eye"] == 0)
            chosen = left["id"]
            code, state = control("select", frame_id=visible["frame_id"], target_id=chosen)
            check(code == 200 and state["selected_id"] == chosen, "stable-layer tap failed")
            miss = Path(str(freeze) + ".miss")
            miss.touch()
            end = time.monotonic() + 1.0
            held = None
            while time.monotonic() < end:
                metadata = image()
                if any(d["eye"] == 0 for d in metadata["detections"]):
                    continue
                box = next((d for d in metadata["display_detections"] if d["eye"] == 0), None)
                check(metadata["selected_id"] == chosen, "one raw miss cleared selected identity")
                check(box is not None and box["id"] == chosen and box["held"] and box["selectable"],
                      ("display identity/provenance was lost during individual dropout", metadata))
                if box["age_ms"] >= 520:
                    held = metadata
                    break
            check(held is not None, "fixture never exercised a held source older than 500ms")
            code, state = control("select", frame_id=held["frame_id"], target_id=chosen)
            check(code == 200 and state["selected_id"] == chosen and state["selected_target"] is None,
                  ("held-box tap must preserve intent without creating a motor observation", code, state))
            check(state["motor"]["left"] == state["motor"]["right"] == 0, "held tap started movement")
            code, state = control("follow")
            check(code == 200 and state["selected_id"] == chosen and state["motor"]["linear_mps"] == 0 and state["mode"] == "follow" and state["reason"] == "follow_wait_target",
                  "initial follow during miss must retain task with zero motion")
            miss.unlink()
            end = time.monotonic() + .5
            while time.monotonic() < end:
                metadata = image()
                left = next((d for d in metadata["detections"] if d["eye"] == 0), None)
                if left:
                    check(left["id"] == chosen and metadata["selected_id"] == chosen,
                          "reappearing selected object changed identity")
                    stable = next(d for d in metadata["display_detections"] if d["eye"] == 0)
                    check(stable["id"] == chosen and not stable["held"], "selected display did not rejoin live track")
                    break
            else:
                raise AssertionError("selected object did not return")
            code, state = control("follow")
            check(code == 200 and state["mode"] == "follow", "fresh complete pair cannot drive follow in demo")
            imu_missing = Path(str(freeze) + ".imu")
            imu_missing.touch()
            time.sleep(.08)
            code, state = control("follow")
            check(code == 200 and state["mode"] == "follow" and state["selected_id"] == chosen,
                  ("temporary IMU interruption cancels or rejects active follow", code, state))
            check(state["reason"] == "follow_wait_imu" and state["motor"]["linear_mps"] == 0,
                  "missing IMU must retain task with zero physical velocity")
            imu_missing.unlink()
            # Read-only polling cannot extend the 350ms owner motion command.
            time.sleep(.45)
            _, state = request("GET", "/api/state")
            check(state["mode"] == "follow" and state["reason"] == "follow_wait_command" and state["motor"]["linear_mps"] == 0,
                  ("short command gap cancels follow or retains motion", state))
            code, state = control("follow")
            check(code == 200 and state["mode"] == "follow" and state["selected_id"] == chosen,
                  "new owner heartbeat cannot maintain existing follow after short interruption")
            # Drain old stream frames before testing the frozen complete source.
            for _ in range(18):
                image()
            freeze.touch()  # Incomplete/partial observations keep advancing forever.
            stale_seen = False
            frozen_control = None
            end = time.monotonic() + 1.1
            while time.monotonic() < end:
                code, state = control("follow")
                metadata = image()
                if frozen_control is None:
                    frozen_control = metadata["detections_frame_id"]
                check(metadata["detections_frame_id"] == frozen_control, "complete source ID changed without a complete observation")
                check(metadata["partial_frame_id"] > frozen_control and metadata["partial_detections"], "live partial stream was lost during control timeout")
                if metadata["box_age_ms"] > 350:
                    check(not metadata["control_eligible"] and metadata["detections"] == [], "partial renewed an expired complete box")
                if not state["vision"]["valid"]:
                    check(state["motor"]["left"] == state["motor"]["right"] == 0 and state["selected_id"] == chosen and state["mode"] == "follow" and state["reason"] == "follow_wait_target",
                          "stale complete control continued following due to fresh partial frames")
                    stale_seen = True
                time.sleep(.02)
            check(stale_seen, "complete source timestamp was kept alive by partial events")
            print(f"PASS real HTTP/MJPEG partial/control split: {checks} checks; selection/orange IDs survive partials and individual misses; held-box taps retain source provenance; complete source expiry stops follow; demo only")
        finally:
            if response is not None:
                response.close()
            if video is not None:
                video.close()
            process.terminate()
            stdout, stderr = process.communicate(timeout=5)
            check(b"012345" not in stdout + stderr, "pairing secret leaked in output")
            check(process.returncode == 0, (stdout, stderr))


if __name__ == "__main__":
    main()
