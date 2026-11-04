"""Real local Wayland client and authenticated phone stream in one DEMO process."""
from pathlib import Path
import http.client
import json
import os
import select
import socket
import subprocess
import tempfile
import time

ROOT = Path(__file__).resolve().parents[1]


def check_view(rotation):
    with socket.socket() as probe:
        probe.bind(("127.0.0.1", 0))
        port = probe.getsockname()[1]
    with tempfile.TemporaryDirectory(prefix="unified-window-") as temporary:
        runtime = Path(temporary)
        token = runtime / "token";token.write_text("012345\n");token.chmod(0o600)
        env = dict(os.environ, XDG_RUNTIME_DIR=str(runtime), WAYLAND_DISPLAY="imu-test",
                   PERCEPTION_EXPECT_FULLSCREEN="1", PERCEPTION_TEST_ROTATION=str(rotation),
                   PERCEPTION_TEST_FRAMES="120")
        server = subprocess.Popen([str(ROOT / "build/test-wayland")],env=env,
                                  stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True)
        client = None
        try:
            assert select.select([server.stdout], [], [], 5)[0], "compositor startup timeout"
            assert server.stdout.readline().strip() == "READY", "compositor failed"
            client = subprocess.Popen([str(ROOT / "build/perception-dashboard"), "--demo", "--seconds", "10",
                "--phone-bind", "127.0.0.1", "--phone-port", str(port), "--pair-token-file", str(token),
                "--rotate", str(rotation)],env=env,stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True)
            for _ in range(100):
                try:
                    connection = http.client.HTTPConnection("127.0.0.1",port,timeout=1)
                    connection.request("GET","/api/state",headers={"Authorization":"Bearer 012345"})
                    response = connection.getresponse();state = json.loads(response.read());connection.close()
                    if response.status == 200 and state["vision"]["npu_workers"] == 3:
                        break
                except (OSError,http.client.HTTPException):
                    pass
                time.sleep(.02)
            else:
                raise AssertionError("phone service did not share live preview")
            assert state["unified"] and state["demo"] and not state["armed"]
            assert state["vision"]["model"] == "industrial-yolov8s-int8-rk3588.rknn"
            assert not state["navigation"]["ready"]
            assert state["motor"]["left_mask"] == 10 and state["motor"]["invert_mask"] == 15
            assert not state["motor"]["enabled"], "DEMO cannot open a physical motor link"
            probe = subprocess.run(["java", "-cp",str(ROOT / "build/android-protocol"), "ProtocolProbe",
                                    f"http://127.0.0.1:{port}",str(token)],capture_output=True,text=True,timeout=5)
            assert probe.returncode == 0, probe.stdout+probe.stderr
            stdout,stderr = client.communicate(timeout=12)
            out,err = server.communicate(timeout=5)
            assert client.returncode == 0, stderr
            assert server.returncode == 0, out+err
            print(f"PASS monochrome view rotation={rotation}: local Wayland frames + authenticated phone telemetry + Android 3-frame JPEG parser, same process")
        finally:
            for process in (client,server):
                if process is not None and process.poll() is None:
                    process.kill();process.communicate(timeout=5)


if __name__ == "__main__":
    check_view(0)
    check_view(90)
