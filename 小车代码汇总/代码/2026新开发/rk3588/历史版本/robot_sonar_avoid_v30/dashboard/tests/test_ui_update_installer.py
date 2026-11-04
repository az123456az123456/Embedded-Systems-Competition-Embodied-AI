"""Exercise the executable-only UI update and rollback in isolated folders."""
import hashlib
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

SOURCE = Path(__file__).resolve().parents[1]


class UIInstaller(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory(prefix="robot-ui-v124-")
        self.root = Path(self.temporary.name)
        self.app = self.root / "board 'quoted' $(literal) `name`" / "robot-perception"
        self.app.mkdir(parents=True)
        self.target = self.app / "perception-dashboard.aarch64"
        self.target.write_bytes(b"previous executable fixture")
        self.target.chmod(0o751)
        self.keep = {"model/yolo11s-int8-rk3588.rknn": b"model fixture", "pair-token.txt": b"123456\n",
                     "run-dashboard.sh": b"original launcher", "lib/library.so": b"original library"}
        for name, data in self.keep.items():
            path = self.app / name
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_bytes(data)
        self.pkg = self.root / "release"
        (self.pkg / "payload").mkdir(parents=True)
        for name in ("install-ui-update.sh", "rollback-ui-update.sh"):
            shutil.copyfile(SOURCE / "scripts/ui-update" / name, self.pkg / name)
        (self.pkg / "payload/perception-dashboard.aarch64").write_bytes(b"new executable fixture")
        files = [p for p in self.pkg.rglob("*") if p.is_file()]
        (self.pkg / "SHA256SUMS").write_text("".join(
            hashlib.sha256(p.read_bytes()).hexdigest() + "  " + p.relative_to(self.pkg).as_posix() + "\n" for p in files))
        self.env = dict(os.environ, ROBOT_APP_DIR=str(self.app))

    def tearDown(self):
        self.temporary.cleanup()

    def install(self):
        return subprocess.run(["sh", str(self.pkg / "install-ui-update.sh")], env=self.env,
                              capture_output=True, text=True)

    def restore(self, backup):
        return subprocess.run(["sh", str(self.pkg / "rollback-ui-update.sh"), str(backup)],
                              capture_output=True, text=True)

    def unchanged(self):
        for name, data in self.keep.items():
            self.assertEqual((self.app / name).read_bytes(), data)
        self.assertFalse((self.app / ".ui-update-lock").exists())
        self.assertFalse((self.root / "literal").exists())

    def test_update_and_restore_preserve_all_other_files_and_original_mode(self):
        result = self.install()
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertEqual(self.target.read_bytes(), b"new executable fixture")
        self.assertEqual(self.target.stat().st_mode & 0o777, 0o755)
        backup = next(self.app.glob(".ui-v12.4-backup.*"))
        result = self.restore(backup)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertEqual(self.target.read_bytes(), b"previous executable fixture")
        self.assertEqual(self.target.stat().st_mode & 0o777, 0o751)
        self.unchanged()

    def test_tampered_payload_fails_before_backup_or_change(self):
        (self.pkg / "payload/perception-dashboard.aarch64").write_bytes(b"tampered")
        self.assertNotEqual(self.install().returncode, 0)
        self.assertEqual(self.target.read_bytes(), b"previous executable fixture")
        self.assertFalse(list(self.app.glob(".ui-v12.4-backup.*")))
        self.unchanged()

    def test_missing_app_does_not_create_installation(self):
        self.env["ROBOT_APP_DIR"] = str(self.root / "missing")
        self.assertNotEqual(self.install().returncode, 0)
        self.assertFalse((self.root / "missing").exists())
        self.unchanged()

    def test_concurrent_update_is_rejected(self):
        (self.app / ".ui-update-lock").mkdir()
        self.assertNotEqual(self.install().returncode, 0)
        self.assertEqual(self.target.read_bytes(), b"previous executable fixture")

    def test_corrupt_backup_does_not_overwrite_installed_binary(self):
        self.assertEqual(self.install().returncode, 0)
        backup = next(self.app.glob(".ui-v12.4-backup.*"))
        (backup / "previous.aarch64").write_bytes(b"corrupt")
        self.assertNotEqual(self.restore(backup).returncode, 0)
        self.assertEqual(self.target.read_bytes(), b"new executable fixture")
        self.unchanged()

    def test_failed_atomic_rename_keeps_existing_program(self):
        tools = self.root / "tools"
        tools.mkdir()
        (tools / "mv").write_text("#!/bin/sh\nexit 1\n")
        (tools / "mv").chmod(0o755)
        self.env["PATH"] = str(tools) + os.pathsep + self.env["PATH"]
        self.assertNotEqual(self.install().returncode, 0)
        self.assertEqual(self.target.read_bytes(), b"previous executable fixture")
        self.assertFalse(list(self.app.glob(".ui-new.*")))
        self.unchanged()


if __name__ == "__main__":
    unittest.main(verbosity=2)
