"""Exercise the real window client without a pointer, touchscreen or HDMI hardware."""
from pathlib import Path
import json
import hashlib
import os
import platform
import select
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
APP = os.environ.get("PERCEPTION_TEST_APP", str(ROOT / "build/perception-dashboard"))
SERVER = ROOT / "build/test-hdmi-wayland"
RESULTS = []


def check(mode="normal", *, output="auto", rotation=0, windowed=False, activate=True, app=APP):
    with tempfile.TemporaryDirectory(prefix="hdmi-no-input-") as runtime:
        env = dict(os.environ, XDG_RUNTIME_DIR=runtime, WAYLAND_DISPLAY="imu-test",
                   HDMI_TEST_MODE=mode, HDMI_TEST_ROTATION=str(rotation),
                   HDMI_EXPECT_FULLSCREEN=str(int(not windowed)),
                   HDMI_EXPECT_OUTPUT="DSI-1" if output == "DSI-1" else "HDMI-A-1",
                   PERCEPTION_OUTPUT=output,
                   PERCEPTION_WESTON_ACTIVATE="1" if activate else "0")
        server = subprocess.Popen([str(SERVER)], env=env, stdout=subprocess.PIPE,
                                  stderr=subprocess.PIPE, text=True)
        try:
            assert select.select([server.stdout], [], [], 5)[0], "Fixture startup timeout"
            assert server.stdout.readline().strip() == "READY", "Fixture startup failed"
            args = [app, "--demo", "--no-phone", "--seconds", "4", "--rotate", str(rotation)]
            if windowed:
                args.append("--windowed")
            client = subprocess.run(args, env=env, capture_output=True, text=True, timeout=7)
            out, err = server.communicate(timeout=3)
            if mode == "missing":
                assert client.returncode != 0, "Unavailable explicit connector must fail clearly"
                assert "MISSING-9" in client.stderr, client.stderr
            else:
                assert client.returncode == 0, (client.returncode, client.stderr, out, err)
            assert server.returncode == 0, (server.returncode, out, err, client.stderr)
            line = next(line for line in out.splitlines() if line.startswith("RESULT "))
            result = json.loads(line.removeprefix("RESULT "))
            if mode == "baseline":
                assert result["output"] == -1, result
                assert result["activations"] == 0, result
            elif mode not in ("missing", "bounded"):
                assert result["callbacks"] >= 10, result
                assert result["late_activations"] == 0, result
                if not activate or windowed:
                    assert result["activations"] == 0, result
                else:
                    assert 1 <= result["activations"] <= 3, result
            print(f"PASS HDMI no-input mode={mode} output={output} rotation={rotation} "
                  f"windowed={int(windowed)} activate={int(activate)}: {result}")
            RESULTS.append(dict(mode=mode, output=output, rotation=rotation,
                                windowed=windowed, activate=activate, result=result,
                                display_events=[line for line in client.stderr.splitlines()
                                                if line.startswith("DISPLAY ")],
                                app_sha256=hashlib.sha256(Path(app).read_bytes()).hexdigest()))
        finally:
            if server.poll() is None:
                server.kill()
                server.communicate(timeout=3)


if __name__ == "__main__":
    baseline = os.environ.get("PERCEPTION_BASELINE_APP")
    if baseline:
        check("baseline", app=baseline)
        check("no-focus")
    check()
    check(rotation=90)
    check(output="DSI-1")
    check(windowed=True)
    check(activate=False)
    check("configure-delay")
    check("recover")
    check("bounded")
    check("no-outputs")
    check("missing", output="MISSING-9")
    if os.environ.get("HDMI_TEST_REPORT"):
        sources = ["src/main.c", "src/display_output.c", "src/display_output.h",
                   "vendor/xdg-output-protocol.c", "vendor/xdg-output-unstable-v1.xml",
                   "tests/test_hdmi_wayland.c", "tests/check_hdmi_wayland.py"]
        Path(os.environ["HDMI_TEST_REPORT"]).write_text(
            json.dumps(dict(status="PASS", hardware_tested=False,
                            host_machine=platform.machine(),
                            source_sha256={name: hashlib.sha256((ROOT / name).read_bytes()).hexdigest()
                                           for name in sources},
                            cases=RESULTS), indent=2) + "\n")
