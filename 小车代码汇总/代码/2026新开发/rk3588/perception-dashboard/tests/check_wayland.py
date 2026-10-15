"""Check the native demo against the test compositor without physical hardware."""
from pathlib import Path
import os
import select
import subprocess
import tempfile

root = Path(__file__).resolve().parents[1]
app = os.environ.get("PERCEPTION_TEST_APP", str(root / "build/perception-dashboard"))
server_path = root / "build/test-wayland"
with tempfile.TemporaryDirectory(prefix="perception-wayland-") as runtime:
    env = dict(os.environ, XDG_RUNTIME_DIR=runtime, WAYLAND_DISPLAY="imu-test")
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
            [app, "--demo", "--seconds", "5"], env=env,
            capture_output=True, text=True, timeout=15,
        )
        out, err = server.communicate(timeout=5)
        assert client.returncode == 0, (client.returncode, client.stdout, client.stderr, out, err)
        assert server.returncode == 0, (server.returncode, out, err)
        print(out.strip())
    finally:
        if server.poll() is None:
            server.kill()
            server.wait()
