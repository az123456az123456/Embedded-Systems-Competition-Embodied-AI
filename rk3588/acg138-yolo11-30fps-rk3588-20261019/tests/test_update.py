#!/usr/bin/env python3
"""Exercise install/rollback against isolated directories, never a real board."""
import hashlib
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import time
import unittest

ROOT = Path(__file__).resolve().parents[1]
BASELINE = ROOT / "tests/fixtures"
MODEL = ROOT / "models/yolo11s-int8-rk3588.rknn"


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


class UpdateTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.compile_area = tempfile.TemporaryDirectory(prefix="v11-installer-host-")
        cls.addClassCleanup(cls.compile_area.cleanup)
        area = Path(cls.compile_area.name)
        source = area / "sleeper.c"
        source.write_text("#include <unistd.h>\nint main(void){for(;;)pause();}\n")
        subprocess.run(["cc", "-O2", str(source), "-o", str(area / "sleeper")], check=True)
        cls.executable = (area / "sleeper").read_bytes()

    def setUp(self):
        temporary = tempfile.TemporaryDirectory(prefix="v11 update with spaces 'quoted' ")
        self.addCleanup(temporary.cleanup)
        self.area = Path(temporary.name)
        self.package = self.area / "package"
        self.dash = self.area / "dashboard"
        self.isp = self.area / "isolated ISP"
        self.tools = self.area / "mock-bin"
        for folder in (self.package / "scripts", self.package / "payload", self.package / "models",
                       self.dash / "model", self.dash / "lib", self.isp / "lib", self.tools):
            folder.mkdir(parents=True, exist_ok=True)
        for path in (ROOT / "scripts").glob("*.sh"):
            shutil.copy2(path, self.package / "scripts" / path.name)
        shutil.copy2(ROOT / "payload/run-perception.sh", self.package / "payload/run-perception.sh")
        shutil.copy2(ROOT / "payload/run-yolo.sh", self.package / "payload/run-yolo.sh")
        shutil.copyfile(MODEL, self.package / "models/yolo11s-int8-rk3588.rknn")
        (self.package / "models/coco_80_labels_list.txt").write_text("new-coco-labels\n")
        (self.package / "payload/perception-dashboard.aarch64").write_bytes(self.executable + b"NEW-WINDOW")
        (self.package / "payload/isp-probe").write_bytes(self.executable + b"NEW-30-FPS")
        shutil.copy2(BASELINE / "run-perception.sh", self.dash / "run-perception.sh")
        shutil.copy2(BASELINE / "run-dashboard.sh", self.dash / "run-dashboard.sh")
        (self.dash / "perception-dashboard.aarch64").write_bytes(self.executable + b"OLD-WINDOW")
        (self.dash / "perception-dashboard.aarch64").chmod(0o751)
        (self.dash / "model/coco_80_labels_list.txt").write_text("existing-coco-labels\n")
        (self.dash / "model/other-user-model.rknn").write_bytes(b"unrelated model")
        (self.dash / "lib/librknnrt.so").write_bytes(b"existing runtime")
        (self.isp / "isp-probe").write_bytes(self.executable + b"OLD-20-FPS")
        (self.isp / "isp-probe").chmod(0o751)
        (self.isp / "run_isp_camera.sh").write_text("# existing network/camera options\n")
        (self.isp / "lib/librkaiq.so").write_bytes(b"existing isolated FakeCam runtime")
        self.root_entry = self.area / "run-yolo.sh"
        self.root_entry.write_text("# user root entry must stay byte-identical\n")
        for name, value in (("uname", "aarch64"), ("id", "0")):
            path = self.tools / name
            path.write_text(f"#!/bin/sh\nprintf '%s\\n' '{value}'\n")
            path.chmod(0o755)
        self.move_marker = self.area / "move-failed-once"
        move = self.tools / "mv"
        move.write_text('''#!/bin/sh
for argument do last=$argument; done
if [ "${MOCK_FAIL_MOVE_TARGET:-}" = "$last" ] && [ ! -f "$MOCK_MOVE_MARKER" ]; then
    : > "$MOCK_MOVE_MARKER"
    echo 'Injected one-time move failure' >&2
    exit 1
fi
exec /usr/bin/mv "$@"
''')
        move.chmod(0o755)
        self.env = dict(os.environ, PATH=str(self.tools) + ":/usr/bin:/bin",
                        DASHBOARD_DIR=str(self.dash), ISP_DIR=str(self.isp),
                        ROOT_YOLO_SCRIPT=str(self.root_entry),
                        MOCK_MOVE_MARKER=str(self.move_marker))
        self.preserved = {path: path.read_bytes() for path in
                          (self.dash / "run-dashboard.sh", self.dash / "lib/librknnrt.so",
                           self.dash / "model/other-user-model.rknn",
                           self.isp / "run_isp_camera.sh", self.isp / "lib/librkaiq.so")}
        self.old = self.snapshot()
        self.manifest()

    def files(self):
        return (self.dash / "perception-dashboard.aarch64", self.dash / "run-perception.sh",
                self.dash / "model/yolo11s-int8-rk3588.rknn", self.dash / "model/coco_80_labels_list.txt",
                self.isp / "isp-probe", self.root_entry)

    def snapshot(self):
        return {path: (path.read_bytes(), path.stat().st_mode & 0o777) if path.exists() else None
                for path in self.files()}

    def manifest(self):
        paths = sorted(path for path in self.package.rglob("*") if path.is_file() and path.name != "SHA256SUMS")
        (self.package / "SHA256SUMS").write_text("".join(
            f"{digest(path)}  {path.relative_to(self.package)}\n" for path in paths))

    def run_script(self, name, *args, success=True, changes=None):
        env = dict(self.env, **(changes or {}))
        result = subprocess.run(["/bin/sh", str(self.package / "scripts" / name), *map(str, args)],
                                env=env, capture_output=True, text=True, timeout=30)
        self.assertEqual(result.returncode == 0, success,
                         (name, result.returncode, result.stdout, result.stderr))
        for path, content in self.preserved.items():
            self.assertEqual(path.read_bytes(), content, str(path))
        return result

    def backups(self):
        return set(self.dash.glob("v11-30-backup.*"))

    def install(self):
        before = self.backups()
        result = self.run_script("install-update.sh")
        backup, = self.backups() - before
        return result, backup

    def test_install_reinstall_and_restore_old_files_and_modes(self):
        result, backup = self.install()
        self.assertIn("YOLO11s INT8 + 30 FPS update installed", result.stdout)
        self.assertEqual((self.dash / "model/yolo11s-int8-rk3588.rknn").read_bytes(), MODEL.read_bytes())
        self.assertIn(b"model/yolo11s-int8-rk3588.rknn", (self.dash / "run-perception.sh").read_bytes())
        self.install()  # The strictly controlled new launcher is accepted too.
        self.run_script("rollback-update.sh", backup)
        self.assertEqual(self.snapshot(), self.old)

    def test_previously_delivered_v8_launcher_upgrades_and_restores(self):
        launcher = self.dash / "run-perception.sh"
        launcher.write_text(launcher.read_text().replace("|--npu-workers)", ")"))
        self.assertEqual(digest(launcher), "5c9c5e86a999a82326d4da2fe410184e1e61204ac956bb449e8e26e7e31ed355")
        self.old = self.snapshot()
        _, backup = self.install()
        self.run_script("rollback-update.sh", backup)
        self.assertEqual(self.snapshot(), self.old)

    def test_unknown_launcher_changes_refused_before_backup(self):
        path = self.dash / "run-perception.sh"
        path.write_bytes(path.read_bytes() + b"# user custom option\n")
        before = self.snapshot()
        result = self.run_script("install-update.sh", success=False)
        self.assertIn("unknown custom changes", result.stderr)
        self.assertEqual(self.snapshot(), before)
        self.assertFalse(self.backups())

    def test_package_corruption_refused_before_backup(self):
        with (self.package / "payload/isp-probe").open("ab") as stream:
            stream.write(b"CORRUPTION")
        result = self.run_script("install-update.sh", success=False)
        self.assertIn("Package checksum verification failed", result.stderr)
        self.assertEqual(self.snapshot(), self.old)
        self.assertFalse(self.backups())

    def test_wrong_model_refused_even_with_updated_manifest(self):
        (self.package / "models/yolo11s-int8-rk3588.rknn").write_bytes(b"wrong model")
        self.manifest()
        result = self.run_script("install-update.sh", success=False)
        self.assertIn("not the tested YOLO11s INT8 model", result.stderr)
        self.assertEqual(self.snapshot(), self.old)
        self.assertFalse(self.backups())

    def test_partial_update_failure_restores_every_old_file(self):
        for failure_target in (self.dash / "run-perception.sh", self.isp / "isp-probe", self.root_entry):
            with self.subTest(target=failure_target):
                if self.move_marker.exists():
                    self.move_marker.unlink()
                result = self.run_script("install-update.sh", success=False,
                                         changes={"MOCK_FAIL_MOVE_TARGET": str(failure_target)})
                self.assertIn("all previous files were restored", result.stderr)
                self.assertTrue(self.move_marker.exists())
                self.assertEqual(self.snapshot(), self.old)
                self.assertFalse(list(self.dash.glob(".v11-30-stage.*")))
                self.assertFalse(list(self.isp.glob(".isp-probe-v11-30.*")))
                self.assertFalse(list(self.area.glob(".run-yolo-v11-30.*")))

    def test_real_package_binaries_and_labels_install_and_restore(self):
        for name in ("perception-dashboard.aarch64", "isp-probe"):
            source = ROOT / "payload" / name
            binary = source.read_bytes()
            self.assertEqual(binary[:4], b"\x7fELF")
            self.assertEqual(binary[18:20], b"\xb7\x00", "Expected AArch64 machine type")
            shutil.copy2(source, self.package / "payload" / name)
        shutil.copy2(ROOT / "models/coco_80_labels_list.txt", self.package / "models/coco_80_labels_list.txt")
        self.manifest()
        _, backup = self.install()
        self.assertEqual((self.dash / "perception-dashboard.aarch64").read_bytes(),
                         (ROOT / "payload/perception-dashboard.aarch64").read_bytes())
        self.assertEqual((self.isp / "isp-probe").read_bytes(), (ROOT / "payload/isp-probe").read_bytes())
        self.assertEqual((self.dash / "model/coco_80_labels_list.txt").read_bytes(),
                         (ROOT / "models/coco_80_labels_list.txt").read_bytes())
        self.run_script("rollback-update.sh", backup)
        self.assertEqual(self.snapshot(), self.old)

    def test_partial_explicit_rollback_failure_recovers_installed_update(self):
        _, backup = self.install()
        before = self.snapshot()
        result = self.run_script("rollback-update.sh", backup, success=False,
                                 changes={"MOCK_FAIL_MOVE_TARGET": str(self.root_entry)})
        self.assertIn("all previous files were restored", result.stderr)
        self.assertEqual(self.snapshot(), before)

    def test_rollback_corruption_refused_without_changes(self):
        _, backup = self.install()
        (backup / "files/receiver").write_bytes(b"broken old receiver")
        before = self.snapshot()
        result = self.run_script("rollback-update.sh", backup, success=False)
        self.assertIn("Backup checksum verification failed", result.stderr)
        self.assertEqual(self.snapshot(), before)

    def test_missing_model_directory_is_removed_on_restore(self):
        self.preserved.pop(self.dash / "model/other-user-model.rknn")
        shutil.rmtree(self.dash / "model")
        old = self.snapshot()
        _, backup = self.install()
        self.run_script("rollback-update.sh", backup)
        self.assertEqual(self.snapshot(), old)
        self.assertFalse((self.dash / "model").exists())

    def test_shared_symlink_target_is_refused(self):
        destination = self.area / "shared-model.rknn"
        destination.write_bytes(b"shared model")
        (self.dash / "model/yolo11s-int8-rk3588.rknn").symlink_to(destination)
        result = self.run_script("install-update.sh", success=False)
        self.assertIn("Refusing symbolic-link target", result.stderr)
        self.assertEqual(destination.read_bytes(), b"shared model")
        self.assertFalse(self.backups())

    def test_missing_root_entry_removed_on_restore(self):
        self.root_entry.unlink()
        old = self.snapshot()
        _, backup = self.install()
        self.assertTrue(self.root_entry.is_file())
        self.run_script("rollback-update.sh", backup)
        self.assertEqual(self.snapshot(), old)

    def test_installed_root_wrapper_passes_orientation_and_quoted_arguments(self):
        self.preserved.pop(self.dash / "run-dashboard.sh")
        capture = self.area / "root-forwarded.txt"
        (self.dash / "run-dashboard.sh").write_text('#!/bin/sh\nprintf "%s\\n" "$@" > "$TEST_CAPTURE"\n')
        self.install()
        env = dict(os.environ, TEST_CAPTURE=str(capture))
        env.pop("DASHBOARD_DIR", None)
        result = subprocess.run(["sh", str(self.root_entry), "-90", "--model", "a b.rknn"],
                                env=env, capture_output=True, text=True, timeout=2)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(capture.read_text().splitlines(), ["-90", "--model", "a b.rknn"])

    def test_active_dashboard_is_not_killed_and_blocks_update(self):
        process = subprocess.Popen([str(self.dash / "perception-dashboard.aarch64")])
        try:
            time.sleep(0.05)
            result = self.run_script("install-update.sh", success=False)
            self.assertIn("Close the YOLO/IMU window first", result.stderr)
            self.assertIsNone(process.poll())
            self.assertEqual(self.snapshot(), self.old)
            self.assertFalse(self.backups())
        finally:
            process.terminate()
            process.wait(timeout=2)

    def test_foreign_receiver_not_killed(self):
        foreign = self.area / "foreign/isp-probe"
        foreign.parent.mkdir()
        foreign.write_bytes(self.executable)
        foreign.chmod(0o755)
        process = subprocess.Popen([str(foreign)])
        try:
            time.sleep(0.05)
            result = self.run_script("install-update.sh", success=False)
            self.assertIn("Another ISP runtime is active", result.stderr)
            self.assertIsNone(process.poll())
            self.assertEqual(self.snapshot(), self.old)
        finally:
            process.terminate()
            process.wait(timeout=2)

    def test_exact_target_receiver_is_stopped_before_replacement(self):
        process = subprocess.Popen([str(self.isp / "isp-probe")])
        try:
            time.sleep(0.05)
            self.install()
            self.assertEqual(process.wait(timeout=2), -15)
            self.assertEqual((self.isp / "isp-probe").read_bytes(),
                             (self.package / "payload/isp-probe").read_bytes())
        finally:
            if process.poll() is None:
                process.terminate()
                process.wait(timeout=2)


if __name__ == "__main__":
    unittest.main(verbosity=2)
