"""Exercise the release installer in temporary directories, never on the board."""
from pathlib import Path
import hashlib
import os
import shutil
import subprocess
import tempfile

app = Path(__file__).resolve().parents[1]

def sums(package):
    (package / "SHA256SUMS").write_text("".join(
        hashlib.sha256(p.read_bytes()).hexdigest() + "  " + p.name + "\n"
        for p in sorted(package.iterdir()) if p.name != "SHA256SUMS"))

with tempfile.TemporaryDirectory(prefix="imu-bias-install-") as directory:
    root = Path(directory)
    package = root / "release"
    dashboard = root / "board app"
    fakebin = root / "test-bin"
    for p in (package, dashboard, fakebin):
        p.mkdir()
    (fakebin / "uname").write_text("#!/bin/sh\necho aarch64\n")
    (fakebin / "uname").chmod(0o755)
    for source, destination in (
        (app / "scripts/install-imu-bias.sh", "install.sh"),
        (app / "build/perception-dashboard.aarch64", "perception-dashboard.aarch64"),
        (app / "run-normal.sh", "run-normal.sh"),
        (app / "run-rotate90.sh", "run-rotate90.sh"),
    ):
        shutil.copy2(source, package / destination)
    sums(package)
    (dashboard / "perception-dashboard.aarch64").write_bytes(b"old application")
    (dashboard / "run-perception.sh").write_text('#!/bin/sh\nprintf "%s\\n" "$@"\n')
    launcher = (dashboard / "run-perception.sh").read_bytes()
    (dashboard / "model.rknn").write_bytes(b"preserve my model")
    env = dict(os.environ, DASHBOARD_DIR=str(dashboard), PATH=str(fakebin)+":"+os.environ["PATH"])

    def install():
        return subprocess.run(["sh", str(package / "install.sh")], env=env,
                              capture_output=True, text=True)

    result = install()
    assert result.returncode == 0, result.stdout + result.stderr
    backups = list(dashboard.glob("imu-bias-backup.*"))
    assert len(backups) == 1
    assert (backups[0] / "perception-dashboard.aarch64").read_bytes() == b"old application"
    assert (dashboard / "perception-dashboard.aarch64").read_bytes() == (package / "perception-dashboard.aarch64").read_bytes()
    assert (dashboard / "run-perception.sh").read_bytes() == launcher
    assert (dashboard / "model.rknn").read_bytes() == b"preserve my model"
    for name, angle in (("run-normal.sh", "0"), ("run-rotate90.sh", "90")):
        result = subprocess.run([str(dashboard / name), "--camera", "with space"],
                                capture_output=True, text=True)
        assert result.returncode == 0
        assert result.stdout.splitlines() == ["--rotate", angle, "--camera", "with space"]
    print("PASS backup, new ARM64 binary, original launcher/model preserved and both orientation arguments")

    (package / "perception-dashboard.aarch64").write_bytes(b"damaged")
    result = install()
    assert result.returncode != 0
    assert list(dashboard.glob("imu-bias-backup.*")) == backups
    assert (dashboard / "perception-dashboard.aarch64").read_bytes() != b"damaged"
    print("PASS corrupted checksum rejected before mutation")

    # A real temporary process verifies /proc executable detection.
    sums(package)
    shutil.copy2(shutil.which("sleep"), dashboard / "perception-dashboard.aarch64")
    before = (dashboard / "perception-dashboard.aarch64").read_bytes()
    process = subprocess.Popen([str(dashboard / "perception-dashboard.aarch64"), "20"])
    try:
        result = install()
        assert result.returncode != 0 and "Close the YOLO/IMU window" in result.stderr
        assert (dashboard / "perception-dashboard.aarch64").read_bytes() == before
        assert list(dashboard.glob("imu-bias-backup.*")) == backups
    finally:
        process.terminate()
        process.wait(timeout=5)
    print("PASS running application detected; update refused without changing files")
