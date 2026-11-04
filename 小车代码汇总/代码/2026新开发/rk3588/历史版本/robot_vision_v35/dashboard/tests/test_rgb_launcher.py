#!/usr/bin/env python3
"""Run the actual launcher with a recording native-program fixture; no NIC/ISP."""
import json
import os
from pathlib import Path
import shutil
import subprocess
import tempfile

launcher = Path(__file__).resolve().parents[1] / "run-perception.sh"
checks = 0
with tempfile.TemporaryDirectory(prefix="v35-rgb-launcher-") as tmp:
    root = Path(tmp)
    app = root / "with spaces"
    app.mkdir()
    shutil.copy2(launcher, app / launcher.name)
    record = root / "args.json"
    program = root / "fake-program.py"
    program.write_text("#!/usr/bin/env python3\nimport json,os,sys\nopen(os.environ['V35_RECORD'],'w').write(json.dumps({'args':sys.argv[1:],'source':os.environ['PERCEPTION_CAMERA_SOURCE']}))\n")
    program.chmod(0o755)
    env = dict(os.environ)
    for key in ("PERCEPTION_CAMERA_SOURCE", "PERCEPTION_RGB_INTERFACE", "MODEL_PATH", "LABELS_PATH", "CAMERA_DEVICE"):
        env.pop(key, None)
    env.update(PERCEPTION_BINARY=str(program), V35_RECORD=str(record),
               PERCEPTION_PREPARE_ISP="0", PERCEPTION_VIDEO_WAIT_ATTEMPTS="1",
               FIND_VIDEO_SCRIPT=str(root / "absent-find"), ISP_CAMERA_SCRIPT=str(root / "absent-isp"))
    def run(args=(), extra=None, success=True):
        global checks
        record.unlink(missing_ok=True)
        result = subprocess.run(["sh", str(app / launcher.name), *args], env=env | (extra or {}), text=True, capture_output=True, timeout=5)
        assert (result.returncode == 0) == success, (args, result.returncode, result.stdout, result.stderr)
        checks += 1
        return json.loads(record.read_text()) if success else result
    def val(data, option):
        return data["args"][data["args"].index(option) + 1]
    data = run(["--fullscreen", "--rotate", "0", "--no-phone"])
    assert data["source"] == "fpga-rgb" and val(data, "--camera") == "fpga-rgb:eth0"
    assert val(data, "--model").endswith("/model/acg138-best-e7f26253-int8-rk3588.rknn")
    assert val(data, "--labels").endswith("/model/acg138-best-e7f26253-labels.txt")
    assert data["args"][-4:] == ["--fullscreen", "--rotate", "0", "--no-phone"]
    checks += 4
    data = run(extra={"CAMERA_TARGET_FPS": "40"})
    assert data["source"] == "fpga-rgb" and val(data, "--camera") == "fpga-rgb:eth0"
    checks += 1
    data = run(["--demo"], {"CAMERA_TARGET_FPS": "40", "PERCEPTION_CAMERA_SOURCE": "v4l2"})
    assert data["source"] == "v4l2"
    checks += 1
    data = run(extra={"PERCEPTION_RGB_INTERFACE": "eth1"})
    assert val(data, "--camera") == "fpga-rgb:eth1"
    checks += 1
    data = run(["--camera", "fpga-rgb:eth2"], {"PERCEPTION_CAMERA_SOURCE": "v4l2"})
    assert data["source"] == "fpga-rgb" and val(data, "--camera") == "fpga-rgb:eth2"
    checks += 1
    data = run(["--camera", "/dev/null"])
    assert data["source"] == "v4l2" and val(data, "--camera") == "/dev/null"
    checks += 1
    data = run(["--demo"], {"PERCEPTION_CAMERA_SOURCE": "v4l2", "MODEL_PATH": "/tmp/model with spaces.rknn", "LABELS_PATH": "/tmp/labels with spaces.txt"})
    assert data["source"] == "v4l2" and val(data, "--camera") == "/dev/video41"
    assert val(data, "--model") == "/tmp/model with spaces.rknn" and val(data, "--labels") == "/tmp/labels with spaces.txt"
    checks += 2
    run(extra={"PERCEPTION_CAMERA_SOURCE": "unknown"}, success=False)
    run(["--camera"], success=False)
    run(extra={"PERCEPTION_CAMERA_SOURCE": "v4l2", "CAMERA_DEVICE": "/no/such/video"}, success=False)
print(f"PASS RGB launcher {checks} assertions: default raw bypasses ISP, V4L2 fallback, explicit source, quoted model/labels, invalid argument failure; native program mocked")
