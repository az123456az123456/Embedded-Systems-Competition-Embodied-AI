#!/usr/bin/env python3
"""Isolated receiver update/rollback tests; never writes a real board path."""
import hashlib
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import time

ROOT = Path(__file__).resolve().parents[1]
SCRIPTS = ("fps-update-common.sh", "install-20fps.sh", "rollback-15fps.sh")


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def manifest(directory, names):
    (directory / "SHA256SUMS").write_text("".join(f"{digest(directory / name)}  {name}\n" for name in names))


with tempfile.TemporaryDirectory(prefix="camera-installer-test-") as directory:
    area = Path(directory)
    toolbin = area / "bin"
    toolbin.mkdir()
    for name, value in (("uname", "aarch64"), ("id", "0")):
        path = toolbin / name
        path.write_text(f"#!/bin/sh\nprintf '%s\\n' '{value}'\n")
        path.chmod(0o755)
    source = area / "sleeper.c"
    source.write_text("#include <unistd.h>\nint main(void){for(;;)pause();}\n")
    subprocess.run(["cc", "-O2", str(source), "-o", str(area / "sleeper")], check=True)
    executable = (area / "sleeper").read_bytes()
    old_binary = executable + b"OLD-RECEIVER\n"
    new_binary = executable + b"NEW-20FPS-RECEIVER\n"
    package = area / "package"
    package.mkdir()
    for name in SCRIPTS:
        shutil.copy2(ROOT / "scripts" / name, package / name)
    (package / "isp-probe").write_bytes(new_binary)
    (package / "isp-probe").chmod(0o755)
    manifest(package, (*SCRIPTS, "isp-probe"))
    runtime = area / "isolated-isp"
    runtime.mkdir()
    (runtime / "lib").mkdir()
    (runtime / "lib" / "librkaiq.so").write_bytes(b"EXISTING-ISOLATED-FAKECAM-LIBRARY\n")
    (runtime / "run_isp_camera.sh").write_bytes(b"#!/bin/sh\n# Keep user camera/network settings exactly.\n")
    (runtime / "isp-probe").write_bytes(old_binary)
    (runtime / "isp-probe").chmod(0o751)
    env = dict(os.environ, PATH=str(toolbin) + ":/usr/bin:/bin", ISP_DIR=str(runtime))
    preserved = {name: (runtime / name).read_bytes() for name in ("lib/librkaiq.so", "run_isp_camera.sh")}

    def run(script, *args, success=True):
        result = subprocess.run(["/bin/sh", str(package / script), *map(str, args)],
                                env=env, capture_output=True, text=True, timeout=25)
        assert (result.returncode == 0) == success, (script, result.returncode, result.stdout, result.stderr)
        assert all((runtime / name).read_bytes() == content for name, content in preserved.items())
        return result

    def backup_dirs():
        return set(runtime.glob("fps20-backup.*"))

    # Happy install and rollback, including exact old executable permissions.
    result = run("install-20fps.sh")
    assert "20 FPS receiver installed" in result.stdout
    assert (runtime / "isp-probe").read_bytes() == new_binary
    first_backup, = backup_dirs()
    assert (first_backup / "isp-probe").read_bytes() == old_binary
    run("rollback-15fps.sh", first_backup)
    assert (runtime / "isp-probe").read_bytes() == old_binary
    assert (runtime / "isp-probe").stat().st_mode & 0o777 == 0o751

    # A package checksum failure must create no backup and change no runtime file.
    before = backup_dirs()
    (package / "isp-probe").write_bytes(new_binary + b"corruption")
    result = run("install-20fps.sh", success=False)
    assert "Package checksum verification failed" in result.stderr
    assert backup_dirs() == before and (runtime / "isp-probe").read_bytes() == old_binary
    (package / "isp-probe").write_bytes(new_binary)

    # A damaged backup is refused before replacing the currently installed file.
    run("install-20fps.sh")
    second_backup, = backup_dirs() - before
    (second_backup / "isp-probe").write_bytes(old_binary + b"corruption")
    result = run("rollback-15fps.sh", second_backup, success=False)
    assert "Backup checksum verification failed" in result.stderr
    assert (runtime / "isp-probe").read_bytes() == new_binary

    # A live dashboard blocks install before backup/stop; test process is ours.
    dashboard = area / "perception-dashboard.aarch64"
    dashboard.write_bytes(executable)
    dashboard.chmod(0o755)
    process = subprocess.Popen([str(dashboard)])
    try:
        time.sleep(0.05)
        before = backup_dirs()
        result = run("install-20fps.sh", success=False)
        assert "Close the YOLO/IMU window first" in result.stderr
        assert process.poll() is None and backup_dirs() == before
    finally:
        process.terminate()
        process.wait(timeout=2)

    # A foreign runtime must never be stopped by this installer.
    foreign = area / "foreign"
    foreign.mkdir()
    (foreign / "isp-probe").write_bytes(executable)
    (foreign / "isp-probe").chmod(0o755)
    process = subprocess.Popen([str(foreign / "isp-probe")])
    try:
        time.sleep(0.05)
        before = backup_dirs()
        result = run("install-20fps.sh", success=False)
        assert "Another ISP runtime is active" in result.stderr
        assert process.poll() is None and backup_dirs() == before
    finally:
        process.terminate()
        process.wait(timeout=2)

    # Only the receiver in the exact target runtime is terminated, then replaced.
    (runtime / "isp-probe").write_bytes(old_binary)
    (runtime / "isp-probe").chmod(0o751)
    process = subprocess.Popen([str(runtime / "isp-probe")])
    try:
        time.sleep(0.05)
        before = backup_dirs()
        result = run("install-20fps.sh")
        assert process.wait(timeout=2) == -15
        final_backup, = backup_dirs() - before
        assert (final_backup / "isp-probe").read_bytes() == old_binary
        assert (runtime / "isp-probe").read_bytes() == new_binary
        run("rollback-15fps.sh", final_backup)
        assert (runtime / "isp-probe").read_bytes() == old_binary
    finally:
        if process.poll() is None:
            process.terminate()
            process.wait(timeout=2)

    # A symlinked shared receiver cannot be replaced; the referent is unchanged.
    (runtime / "isp-probe").unlink()
    (runtime / "isp-probe").symlink_to(foreign / "isp-probe")
    result = run("install-20fps.sh", success=False)
    assert "isp-probe is a symlink" in result.stderr
    assert (foreign / "isp-probe").read_bytes() == executable

print("PASS isolated install/rollback, old mode/bytes, library/launcher preserved, package/backup checksum rejection, live dashboard refusal, foreign ISP protection, exact receiver TERM, shared symlink refusal")
