#!/usr/bin/env python3
"""Invalid rate must not touch network, media graph, modules or locks."""
import os
from pathlib import Path
import subprocess
import tempfile

root = Path(__file__).resolve().parents[1]
with tempfile.TemporaryDirectory(prefix="camera-fps-test-") as directory:
    area = Path(directory)
    toolbin = area / "bin"
    toolbin.mkdir()
    record = area / "calls"
    # dirname is read-only and used before validation to locate the launcher.
    (toolbin / "dirname").symlink_to("/usr/bin/dirname")
    for name in ("id", "mkdir", "insmod", "ip", "media-ctl", "kill", "sleep", "gst-launch-1.0"):
        command = toolbin / name
        command.write_text('#!/bin/sh\necho "invoked" >>"$FPS_TEST_CALLS"\nexit 77\n')
        command.chmod(0o755)
    for value in ("", "0", "19", "21", "60", "020", "20.0", "20 ", "20\n"):
        env = dict(os.environ, PATH=str(toolbin), CAMERA_TARGET_FPS=value, FPS_TEST_CALLS=str(record))
        result = subprocess.run(["/bin/sh", str(root / "run_isp_camera.sh"), "dual-1080", "100"],
                                env=env, capture_output=True, text=True, timeout=2)
        assert result.returncode == 2, (value, result)
        assert "CAMERA_TARGET_FPS must be 15, 20 or 30" in result.stderr, (value, result)
        assert not record.exists(), (value, record.read_text())
print("PASS launcher rejects invalid FPS before invoking root/module/network/media/session commands")
