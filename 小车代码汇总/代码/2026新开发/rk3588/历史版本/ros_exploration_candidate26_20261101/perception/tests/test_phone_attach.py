#!/usr/bin/env python3
"""Real vision threads -> phone observer, independent of a 10Hz main loop."""
import http.client
import json
from pathlib import Path
import socket
import subprocess
import tempfile
import time
from test_phone_rate import Video

ROOT = Path(__file__).resolve().parents[1]


def main():
    with socket.socket() as available:
        available.bind(("127.0.0.1", 0))
        port = available.getsockname()[1]
    with tempfile.TemporaryDirectory(prefix="v35-phone-attach-") as tmp:
        token = Path(tmp) / "token"
        token.write_text("012345\n")
        probe = str(ROOT / "build/phone-attach-probe")
        process = subprocess.Popen([probe, str(port), str(token), "--stream"], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        video = None
        try:
            for _ in range(100):
                try:
                    connection = http.client.HTTPConnection("127.0.0.1", port, timeout=2)
                    connection.request("GET", "/api/state", headers={"Authorization": "Bearer 012345"})
                    response = connection.getresponse()
                    state = json.loads(response.read());connection.close()
                    if state["vision"]["result_fps"] > 30:
                        break
                except OSError:
                    pass
                time.sleep(.02)
            else:
                raise AssertionError("attached production observer did not receive vision observations")
            assert state["phone"]["encodes"] == 0, "no-client observer encoded or copied images"
            assert state["phone"]["snapshot_copies"] == 0, "no-client observer copied the preview image"
            video = Video(port);first = video.image();ids = [first["frame_id"]]
            started = time.monotonic();count = 0
            while time.monotonic()-started < 2:
                metadata = video.image();ids.append(metadata["frame_id"]);count += 1
                assert metadata["age_ms"] < 200
            fps = count/(time.monotonic()-started)
            assert 30 < fps < 44, ("phone still limited by the 10Hz main loop",fps)
            assert all(b>a for a,b in zip(ids,ids[1:])), "attached observer replayed or regressed frames"
            print(f"PASS attached production vision->observer->JPEG source40fps phone_fps={fps:.3f} frames={count}; main loop10Hz, zero phone_publish calls, no-client zero image copies")
        finally:
            if video:video.close()
            process.terminate();stdout,stderr = process.communicate(timeout=5)
            assert process.returncode == 0, (stdout,stderr)
        lifecycle = subprocess.run([probe,str(port),str(token),"--lifecycle"],stdout=subprocess.PIPE,stderr=subprocess.PIPE,timeout=20)
        assert lifecycle.returncode == 0, (lifecycle.stdout,lifecycle.stderr)
        print(lifecycle.stdout.decode().strip())


if __name__ == "__main__":
    main()
