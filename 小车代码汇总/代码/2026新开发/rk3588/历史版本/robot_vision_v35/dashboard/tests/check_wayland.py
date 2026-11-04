"""Check the native demo against the test compositor without physical hardware."""
from pathlib import Path
import os
import select
import subprocess
import tempfile

root = Path(__file__).resolve().parents[1]
app = os.environ.get("PERCEPTION_TEST_APP", str(root / "build/perception-dashboard"))
server_path = root / "build/test-wayland"
def check_mode(extra, fullscreen, rotation=0):
    with tempfile.TemporaryDirectory(prefix="perception-wayland-") as runtime:
        env = dict(os.environ, XDG_RUNTIME_DIR=runtime, WAYLAND_DISPLAY="imu-test",
                   PERCEPTION_EXPECT_FULLSCREEN=str(int(fullscreen)),PERCEPTION_TEST_ROTATION=str(rotation))
        server = subprocess.Popen(
            [str(server_path)], env=env, stdout=subprocess.PIPE,
            stderr=subprocess.PIPE, text=True,
        )
        try:
            assert select.select([server.stdout], [], [], 5)[0], "Test compositor startup timed out"
            first = server.stdout.readline().strip()
            if first != "READY":
                _, err = server.communicate(timeout=5)
                raise AssertionError(f"Test compositor startup failed: {first!r}; {err}")
            client = subprocess.run(
                [app, "--demo", "--no-phone", "--seconds", "5", *extra], env=env,
                capture_output=True, text=True, timeout=15,
            )
            out, err = server.communicate(timeout=5)
            assert client.returncode == 0, (client.returncode, client.stdout, client.stderr, out, err)
            assert server.returncode == 0, (server.returncode, out, err)
            print(f"rotation {rotation}, " + ("default fullscreen: " if fullscreen else "windowed override: ") + out.strip())
        finally:
            if server.poll() is None:
                server.kill()
                server.wait()

if os.environ.get("PERCEPTION_NAVIGATION_TEST"):
    check_mode([],True)
    check_mode(["--rotate","90"],True,90)
    raise SystemExit(0)

check_mode([], True)
check_mode(["--windowed"], False)
check_mode(["--rotate", "90"], True,90)
check_mode(["--rotate", "180"], True,180)
check_mode(["--rotate", "270"], True,270)
check_mode(["--windowed","--rotate", "90"], False,90)
