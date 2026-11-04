"""Exercise installation and rollback in isolated directories, never /root or /opt."""
from pathlib import Path
import hashlib
import os
import shutil
import subprocess
import tempfile
import unittest

SOURCE = Path(__file__).resolve().parents[1]


class InstallerTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory(prefix="robot-v12-install-")
        self.base = Path(self.tmp.name)
        self.root = self.base / "board root"
        self.isp = self.base / "camera isp"
        self.pkg = self.base / "release"
        self.root.mkdir();self.isp.mkdir();self.pkg.mkdir()
        (self.root / "run-yolo.sh").write_text("#!/bin/sh\necho original\n")
        (self.isp / "isp-probe").write_bytes(b"original receiver")
        (self.isp / "run_isp_camera.sh").write_text("#!/bin/sh\n")
        for name in ("install-update.sh", "rollback-update.sh"):
            shutil.copyfile(SOURCE / name, self.pkg / name)
        self.app = self.pkg / "payload/app"
        (self.app / "model").mkdir(parents=True)
        (self.app / "perception-dashboard.aarch64").write_bytes(b"fixture, not a runnable ELF")
        (self.app / "run-dashboard.sh").write_text("#!/bin/sh\nprintf '%s\\n' \"$@\"\n")
        (self.app / "run-perception.sh").write_text("#!/bin/sh\n")
        (self.app / "model/yolo11s-int8-rk3588.rknn").write_bytes(b"fixture model")
        (self.app / "model/coco_80_labels_list.txt").write_text("person\n")
        (self.pkg / "payload/isp-probe").write_bytes(b"new receiver")
        self.checksums()
        self.env = dict(os.environ, ROBOT_ROOT=str(self.root), ROBOT_ISP_DIR=str(self.isp))

    def tearDown(self):
        self.tmp.cleanup()

    def checksums(self):
        paths = sorted((self.pkg / "payload").rglob("*"))
        paths += [self.pkg / "install-update.sh", self.pkg / "rollback-update.sh"]
        (self.pkg / "SHA256SUMS").write_text("".join(
            hashlib.sha256(p.read_bytes()).hexdigest()+"  "+p.relative_to(self.pkg).as_posix()+"\n"
            for p in paths if p.is_file()))

    def install(self, expected=0, env=None):
        result = subprocess.run(["sh", str(self.pkg / "install-update.sh")],
                                env=env or self.env, capture_output=True, text=True)
        self.assertEqual(result.returncode, expected, result.stdout+result.stderr)
        return result

    def backup(self):
        return next(self.root.glob("robot-perception-v12-backup.*"))

    def rollback(self):
        result = subprocess.run(["sh", str(self.pkg / "rollback-update.sh"), str(self.backup())],
                                capture_output=True, text=True)
        self.assertEqual(result.returncode, 0, result.stderr)

    def test_new_install_preserves_pairing_and_reverts_entry_and_receiver(self):
        legacy = self.root / "robot-phone";legacy.mkdir()
        (legacy / "pair-token.txt").write_text("012345\n")
        self.install()
        target = self.root / "robot-perception"
        self.assertEqual((target / "pair-token.txt").read_text(), "012345\n")
        self.assertEqual((target / "pair-token.txt").stat().st_mode & 0o777, 0o600)
        self.assertEqual((self.isp / "isp-probe").read_bytes(), b"new receiver")
        run = subprocess.run(["sh", str(self.root / "run-yolo.sh"), "-90", "--page", "navigation"], capture_output=True,text=True)
        self.assertEqual(run.returncode, 0, run.stderr)
        self.assertEqual(run.stdout.splitlines(), ["-90", "--page", "navigation"])
        self.rollback()
        self.assertFalse(target.exists())
        self.assertIn("original", (self.root / "run-yolo.sh").read_text())
        self.assertEqual((self.isp / "isp-probe").read_bytes(), b"original receiver")

    def test_existing_application_and_settings_restored(self):
        target = self.root / "robot-perception";target.mkdir()
        (target / "old-version").write_text("v11")
        (target / "pair-token.txt").write_text("existing-long-token\n")
        self.install()
        self.assertFalse((target / "old-version").exists())
        self.rollback()
        self.assertEqual((target / "old-version").read_text(), "v11")
        self.assertEqual((target / "pair-token.txt").read_text(), "existing-long-token\n")

    def test_established_launcher_forwards_and_restores_old_launcher_only(self):
        legacy = self.root / "perception-dashboard";legacy.mkdir()
        launcher = legacy / "run-dashboard.sh"
        original = "#!/bin/sh\nprintf 'old window\\n'\n"
        launcher.write_text(original);launcher.chmod(0o755)
        (legacy / "model.rknn").write_bytes(b"original-model")
        self.install()
        run = subprocess.run([str(launcher), "-90", "--no-phone"], capture_output=True, text=True)
        self.assertEqual(run.returncode, 0, run.stderr)
        self.assertEqual(run.stdout.splitlines(), ["-90", "--no-phone"])
        self.assertEqual((legacy / "model.rknn").read_bytes(), b"original-model")
        self.rollback()
        self.assertEqual(launcher.read_text(), original)
        self.assertEqual(launcher.stat().st_mode & 0o777, 0o755)

    def test_absent_legacy_launcher_is_removed_on_rollback(self):
        legacy = self.root / "perception-dashboard";legacy.mkdir()
        self.install()
        self.assertTrue((legacy / "run-dashboard.sh").exists())
        self.rollback()
        self.assertFalse((legacy / "run-dashboard.sh").exists())

    def test_relative_receiver_symlink_is_restored_without_touching_its_target(self):
        (self.isp / "versions").mkdir()
        original = self.isp / "versions/old-probe"
        original.write_bytes(b"old receiver target");original.chmod(0o751)
        receiver = self.isp / "isp-probe"
        receiver.unlink();receiver.symlink_to("versions/old-probe")
        self.install()
        self.assertFalse(receiver.is_symlink())
        self.rollback()
        self.assertTrue(receiver.is_symlink())
        self.assertEqual(os.readlink(receiver), "versions/old-probe")
        self.assertEqual(receiver.read_bytes(), b"old receiver target")
        self.assertEqual(original.stat().st_mode & 0o777, 0o751)

    def test_tampered_release_has_no_install_side_effects(self):
        (self.app / "perception-dashboard.aarch64").write_bytes(b"tampered")
        self.install(expected=1)
        self.assertFalse((self.root / "robot-perception").exists())
        self.assertFalse(list(self.root.glob("*backup*")))
        self.assertEqual((self.isp / "isp-probe").read_bytes(), b"original receiver")

    def test_missing_camera_installation_fails_before_backup(self):
        (self.isp / "run_isp_camera.sh").unlink()
        self.install(expected=1)
        self.assertFalse(list(self.root.glob("*backup*")))

    def test_shell_metacharacters_in_path_are_literal(self):
        self.root = self.base / "board 'quoted' $(touch NEVER) `literal`"
        self.root.mkdir()
        self.env["ROBOT_ROOT"] = str(self.root)
        self.install()
        run = subprocess.run(["sh", str(self.root / "run-yolo.sh"), "one argument", "-0"],
                             cwd=self.base,capture_output=True,text=True)
        self.assertEqual(run.returncode, 0, run.stderr)
        self.assertEqual(run.stdout.splitlines(), ["one argument", "-0"])
        self.assertFalse((self.base / "NEVER").exists())

    def test_mid_install_failure_restores_saved_application(self):
        target = self.root / "robot-perception";target.mkdir()
        (target / "old-version").write_text("v11")
        tools = self.base / "mock-bin";tools.mkdir()
        real_mv = shutil.which("mv")
        (tools / "mv").write_text("#!/bin/sh\nfor arg in \"$@\"; do\n case \"$arg\" in */run-yolo.sh) exit 1;; esac\ndone\nexec " + real_mv + " \"$@\"\n")
        (tools / "mv").chmod(0o755)
        env = dict(self.env, PATH=str(tools)+os.pathsep+self.env["PATH"])
        self.install(expected=1, env=env)
        self.assertEqual((target / "old-version").read_text(), "v11")
        self.assertIn("original", (self.root / "run-yolo.sh").read_text())


if __name__ == "__main__":
    unittest.main(verbosity=2)
