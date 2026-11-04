"""Actual native dashboard: HTTP/control and failing interface diagnostics."""
import http.client
import json
import os
from pathlib import Path
import socket
import subprocess
import tempfile
import time

ROOT = Path(__file__).resolve().parents[1]
APP = Path(os.environ.get("PERCEPTION_TEST_APP", ROOT / "build/perception-dashboard"))


def main():
    checks = 0
    with tempfile.TemporaryDirectory(prefix="wronglog-runtime-") as temporary:
        directory = Path(temporary)
        logfile = directory / "wrong.log"
        token = directory / "pair-token"
        secret = "never-log-this-secret-9876"
        token.write_text(secret + "\n")
        env = dict(os.environ, ROBOT_WRONG_LOG_PATH=str(logfile),
                   FOLLOW_GEOMETRY_FILE=str(ROOT / "tests/synthetic-bearing-fixture.json"))
        with socket.socket() as probe:
            probe.bind(("127.0.0.1", 0))
            port = probe.getsockname()[1]
        process = subprocess.Popen([str(APP), "--demo", "--headless", "--phone-bind", "127.0.0.1",
                                    "--phone-port", str(port), "--pair-token-file", str(token)],
                                   env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

        def request(op=None, seq=0, session="", **extra):
            connection = http.client.HTTPConnection("127.0.0.1", port, timeout=1)
            body = {} if op is None else dict(op=op, seq=seq, session=session, **extra)
            connection.request("POST", "/api/connect" if op is None else "/api/control",
                               json.dumps(body), {"Authorization": "Bearer " + secret,
                                                  "Content-Type": "application/json"})
            response = connection.getresponse()
            data = json.loads(response.read())
            code = response.status
            connection.close()
            return code, data

        try:
            for _ in range(100):
                try:
                    code, state = request()
                    break
                except (OSError, http.client.HTTPException):
                    time.sleep(.02)
            else:
                raise AssertionError("real HTTP server did not start")
            session = state["session"]
            assert code == 200
            checks += 1
            time.sleep(.05)
            assert request("arm", 1, session)[1]["armed"]
            checks += 1
            assert request("drive", 2, session, throttle=.3, steer=0, hold=False)[0] == 200
            checks += 1
            time.sleep(.45)
            assert request("drive", 3, session, throttle=.3, steer=0, hold=False)[0] == 409
            checks += 1
            assert request("arm", 4, session)[1]["armed"]
            assert request("drive", 5, session, throttle=0, steer=0)[1]["armed"]
            assert not request("disarm", 6, session, client_reason="telemetry_timeout",
                               client_mono_ms=123, client_state_age_ms=1501,
                               client_lease_age_ms=10, client_diag_id=99)[1]["armed"]
            checks += 3
        finally:
            process.terminate()
            process.wait(timeout=5)
        records = [json.loads(line) for line in logfile.read_text().splitlines()]
        assert any(r.get("reason") == "lease_expired" for r in records)
        assert any(r.get("reason") == "explicit_control_stop" and
                   r["data"].get("sequence") == 6 and
                   r["data"].get("client_reported_unverified", {}).get("reason") == "telemetry_timeout"
                   for r in records)
        assert any(r["data"].get("http_status") == 409 and
                   r["data"].get("sequence") == 3 for r in records)
        assert secret not in logfile.read_text() and session not in logfile.read_text()
        assert not any(r.get("reason") == "joystick_released" for r in records)
        checks += 5

        failed = directory / "network-failed.log"
        env["ROBOT_WRONG_LOG_PATH"] = str(failed)
        result = subprocess.run([str(APP), "--headless", "--no-phone", "--interface", "badiface0",
                                 "--seconds", "1"], env=env, stdout=subprocess.DEVNULL,
                                stderr=subprocess.DEVNULL, timeout=10)
        assert result.returncode == 0
        records = [json.loads(line) for line in failed.read_text().splitlines()]
        for component in ("sensor_network", "imu_network", "motor_network"):
            assert any(r.get("component") == component and
                       r["data"].get("errno", 0) != 0 for r in records), component
            checks += 1
        assert any(r.get("reason") == "pipeline_status_error" for r in records)
        checks += 2
        print(f"PASS wronglog real runtime {checks} checks")


if __name__ == "__main__":
    main()
