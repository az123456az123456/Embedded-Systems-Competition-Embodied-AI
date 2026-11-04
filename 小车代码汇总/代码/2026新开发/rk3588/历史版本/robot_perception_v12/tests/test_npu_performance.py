#!/usr/bin/env python3
"""Exercise governor selection only against temporary files, using BusyBox sh."""
import os
from pathlib import Path
import shutil
import stat
import subprocess
import tempfile
import unittest


SCRIPT = Path(__file__).resolve().parents[1] / "scripts/npu-performance.sh"
BUSYBOX = shutil.which("busybox")


@unittest.skipUnless(BUSYBOX, "BusyBox is required for board-shell compatibility")
class NpuPerformanceTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory(prefix="npu-performance-test-")
        self.root = Path(self.temporary.name)
        self.sys = self.root / "sys"
        self.proc = self.root / "proc"
        self.state = self.root / "private-state"
        self.npu = self.sys / "class/devfreq/fdab0000.npu"
        self.npu.mkdir(parents=True)
        self.values = {
            "name": "fdab0000.npu\n",
            "cur_freq": "600000000\n",
            "available_frequencies": "200000000 400000000 600000000 1000000000\n",
            "min_freq": "200000000\n",
            "max_freq": "1000000000\n",
            "governor": "simple_ondemand\n",
            "available_governors": "simple_ondemand performance userspace\n",
            "voltage": "800000\n",
        }
        for name, value in self.values.items():
            (self.npu / name).write_text(value)
        self.load = self.sys / "kernel/debug/rknpu/load"
        self.load.parent.mkdir(parents=True)
        self.load.write_text("NPU load: Core0: 88%, Core1: 91%, Core2: 90%,\n")
        self.thermal = self.sys / "class/thermal/thermal_zone0/policy"
        self.thermal.parent.mkdir(parents=True)
        self.thermal.write_text("step_wise\n")
        self.env = dict(os.environ, SYSFS_ROOT=str(self.sys), PROCFS_ROOT=str(self.proc),
                        NPU_STATE_DIR=str(self.state))

    def tearDown(self):
        self.temporary.cleanup()

    def call(self, mode="status", success=True, env=None):
        result = subprocess.run([BUSYBOX, "sh", str(SCRIPT), mode], env=env or self.env,
                                capture_output=True, text=True, timeout=5)
        if success:
            self.assertEqual(result.returncode, 0, result.stderr + result.stdout)
        else:
            self.assertNotEqual(result.returncode, 0, result.stdout)
        return result

    def assert_limits_untouched(self):
        for name in ("min_freq", "max_freq", "available_frequencies", "voltage"):
            self.assertEqual((self.npu / name).read_text(), self.values[name])
        self.assertEqual(self.thermal.read_text(), "step_wise\n")
        self.assertEqual(self.load.read_text(), "NPU load: Core0: 88%, Core1: 91%, Core2: 90%,\n")

    def test_status_reads_all_fields_without_writing(self):
        result = self.call()
        for value in ("600000000", "1000000000", "simple_ondemand", "Core0: 88%", "Core1: 91%", "Core2: 90%"):
            self.assertIn(value, result.stdout)
        self.assertFalse(self.state.exists())
        self.assertEqual((self.npu / "governor").read_text(), "simple_ondemand\n")
        self.assert_limits_untouched()

    def test_enable_is_private_idempotent_and_restore_uses_first_governor(self):
        self.call("enable")
        self.assertEqual((self.npu / "governor").read_text(), "performance\n")
        backup = self.state / "backup/governor"
        self.assertEqual(backup.read_text(), "simple_ondemand\n")
        self.assertEqual(stat.S_IMODE(self.state.stat().st_mode), 0o700)
        self.assertEqual(stat.S_IMODE((self.state / "backup").stat().st_mode), 0o700)
        self.assertEqual(stat.S_IMODE(backup.stat().st_mode), 0o600)
        saved = backup.read_bytes()
        self.call("enable")
        self.assertEqual(backup.read_bytes(), saved)
        self.assertFalse((self.state / ".lock").exists())
        self.call("restore")
        self.assertEqual((self.npu / "governor").read_text(), "simple_ondemand\n")
        self.assertFalse((self.state / "backup").exists())
        self.call("restore")
        self.assert_limits_untouched()

    def test_restore_without_backup_is_harmless(self):
        self.call("restore")
        self.assertFalse(self.state.exists())
        self.assert_limits_untouched()

    def test_clock_already_at_maximum_reports_no_higher_selection(self):
        (self.npu / "cur_freq").write_text(self.values["max_freq"])
        self.assertIn("already at the configured maximum", self.call().stdout)
        result = self.call("enable")
        self.assertIn("not select a higher frequency", result.stdout)
        self.assert_limits_untouched()

    def test_missing_performance_refuses_all_writes(self):
        (self.npu / "available_governors").write_text("simple_ondemand userspace\n")
        self.assertIn("not an available", self.call("enable", success=False).stderr)
        self.assertFalse(self.state.exists())
        self.assertEqual((self.npu / "governor").read_text(), "simple_ondemand\n")
        self.assert_limits_untouched()

    def test_missing_and_ambiguous_devices_refuse_writes(self):
        second = self.sys / "class/devfreq/other.npu"
        second.mkdir()
        self.assertIn("found 2", self.call("enable", success=False).stderr)
        self.assertFalse(self.state.exists())
        second.rmdir()
        shutil.rmtree(self.npu)
        self.assertIn("found 0", self.call(success=False).stderr)

    def test_name_attribute_can_identify_device(self):
        renamed = self.npu.with_name("platform-devfreq")
        self.npu.rename(renamed)
        self.npu = renamed
        self.call("enable")
        self.call("restore")
        self.assert_limits_untouched()

    def test_restore_rejects_changed_device_and_retains_backup(self):
        self.call("enable")
        self.npu.rename(self.root / "disconnected.npu")
        self.npu.mkdir()
        for name, value in self.values.items():
            (self.npu / name).write_text(value)
        # A new sysfs link with a different canonical target must not inherit
        # the old device's saved governor, even if it is the only NPU present.
        self.npu.rename(self.npu.with_name("new.npu"))
        self.npu = self.npu.with_name("new.npu")
        (self.npu / "governor").write_text("performance\n")
        self.assertIn("different NPU", self.call("restore", success=False).stderr)
        self.assertEqual((self.npu / "governor").read_text(), "performance\n")
        self.assertTrue((self.state / "backup/governor").exists())
        self.assertFalse((self.state / ".lock").exists())

    def test_restore_rejects_unavailable_or_injected_governor(self):
        self.call("enable")
        (self.npu / "available_governors").write_text("performance userspace\n")
        self.assertIn("no longer offered", self.call("restore", success=False).stderr)
        (self.npu / "available_governors").write_text(self.values["available_governors"])
        marker = self.root / "must-not-exist"
        (self.state / "backup/governor").write_text(f"performance; touch {marker}\n")
        self.assertIn("invalid saved", self.call("restore", success=False).stderr)
        self.assertFalse(marker.exists())
        self.assertEqual((self.npu / "governor").read_text(), "performance\n")
        self.assert_limits_untouched()

    def test_rejects_state_and_backup_symlinks(self):
        elsewhere = self.root / "elsewhere"
        elsewhere.mkdir()
        self.state.symlink_to(elsewhere, target_is_directory=True)
        self.assertIn("symbolic link", self.call("enable", success=False).stderr)
        self.state.unlink()
        self.call("enable")
        backup_governor = self.state / "backup/governor"
        backup_governor.unlink()
        replacement = self.root / "injected-governor"
        replacement.write_text("userspace\n")
        backup_governor.symlink_to(replacement)
        self.assertIn("invalid NPU backup", self.call("restore", success=False).stderr)
        self.assertEqual((self.npu / "governor").read_text(), "performance\n")

    def test_existing_lock_prevents_mutation(self):
        self.state.mkdir(mode=0o700)
        (self.state / ".lock").mkdir()
        self.assertIn("state lock", self.call("enable", success=False).stderr)
        self.assertEqual((self.npu / "governor").read_text(), "simple_ondemand\n")
        self.assertTrue((self.state / ".lock").exists())

    def test_proc_driver_load_fallback_and_absent_load(self):
        self.load.unlink()
        proc_load = self.proc / "rknpu/load"
        proc_load.parent.mkdir(parents=True)
        proc_load.write_text("NPU load: Core0: 1%, Core1: 2%, Core2: 3%\n")
        self.assertIn("Core2: 3%", self.call().stdout)
        proc_load.unlink()
        self.assertIn("unavailable (no readable", self.call().stdout)

    def test_shared_state_root_is_rejected_without_chmod(self):
        for unsafe in ("/tmp", "/tmp/.."):
            with self.subTest(unsafe=unsafe):
                environment = dict(self.env, NPU_STATE_DIR=unsafe)
                before = stat.S_IMODE(Path(unsafe).stat().st_mode)
                self.assertIn("shared NPU_STATE_DIR", self.call("enable", success=False, env=environment).stderr)
                self.assertEqual(stat.S_IMODE(Path(unsafe).stat().st_mode), before)


if __name__ == "__main__":
    unittest.main(verbosity=2)
