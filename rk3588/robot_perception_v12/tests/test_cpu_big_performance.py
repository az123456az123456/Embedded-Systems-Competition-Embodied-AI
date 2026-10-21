#!/usr/bin/env python3
"""Test RK3588 big-core governors with fake sysfs and the board's BusyBox shell."""
import os
from pathlib import Path
import shutil
import stat
import subprocess
import tempfile
import unittest


SCRIPT = Path(__file__).resolve().parents[1] / "scripts/cpu-big-performance.sh"
BUSYBOX = shutil.which("busybox")


@unittest.skipUnless(BUSYBOX, "BusyBox is required for board-shell compatibility")
class CpuBigPerformanceTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory(prefix="cpu-big-performance-test-")
        self.root = Path(self.temporary.name)
        self.sys = self.root / "sys"
        self.state = self.root / "private-state"
        self.cpu = self.sys / "devices/system/cpu/cpufreq"
        self.policies = {}
        self.original = {0: "performance", 4: "schedutil", 6: "ondemand"}
        for index, cpus in ((0, "0 1 2 3"), (4, "4 5"), (6, "6 7")):
            node = self.cpu / f"policy{index}"
            node.mkdir(parents=True)
            self.policies[index] = node
            values = {
                "related_cpus": cpus,
                "scaling_cur_freq": "600000" if index == 4 else "2256000",
                "scaling_min_freq": "408000" if index else "1800000",
                "scaling_max_freq": "2256000" if index else "1800000",
                "scaling_available_frequencies": "408000 600000 1800000 2256000",
                "scaling_governor": self.original[index],
                "scaling_available_governors": "performance schedutil ondemand userspace",
            }
            for name, value in values.items():
                (node / name).write_text(value + "\n")
        self.thermal = self.sys / "class/thermal/thermal_zone0/policy"
        self.thermal.parent.mkdir(parents=True)
        self.thermal.write_text("step_wise\n")
        self.voltage = self.sys / "kernel/debug/regulator/vdd_cpu_big/voltage"
        self.voltage.parent.mkdir(parents=True)
        self.voltage.write_text("1000000\n")
        self.untouched = {
            str(path): path.read_bytes()
            for path in self.sys.rglob("*")
            if path.is_file() and (path.name != "scaling_governor" or path.parent == self.policies[0])
        }
        self.env = dict(os.environ, SYSFS_ROOT=str(self.sys), CPU_STATE_DIR=str(self.state))

    def tearDown(self):
        self.temporary.cleanup()

    def call(self, mode="status", success=True, env=None):
        result = subprocess.run([BUSYBOX, "sh", str(SCRIPT), mode], env=env or self.env,
                                capture_output=True, text=True, timeout=5)
        if success:
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        else:
            self.assertNotEqual(result.returncode, 0, result.stdout)
        return result

    def governor(self, index):
        return (self.policies[index] / "scaling_governor").read_text().strip()

    def assert_untouched(self):
        for filename, value in self.untouched.items():
            self.assertEqual(Path(filename).read_bytes(), value, filename)

    def test_status_is_read_only_and_reports_two_big_policies(self):
        result = self.call()
        for expected in ("Related CPUs: 4 5", "Related CPUs: 6 7", "600000", "2256000", "schedutil", "ondemand"):
            self.assertIn(expected, result.stdout)
        self.assertIn("snapshot", result.stdout)
        self.assertFalse(self.state.exists())
        self.assert_untouched()

    def test_enable_restore_preserves_limits_policy0_and_original_governors(self):
        self.call("enable")
        self.assertEqual((self.governor(4), self.governor(6)), ("performance", "performance"))
        self.assertEqual(stat.S_IMODE(self.state.stat().st_mode), 0o700)
        for index in (4, 6):
            saved = self.state / f"backup/policy{index}.governor"
            self.assertEqual(saved.read_text().strip(), self.original[index])
            self.assertEqual(stat.S_IMODE(saved.stat().st_mode), 0o600)
        self.call("enable")
        self.assertEqual((self.state / "backup/policy4.governor").read_text().strip(), "schedutil")
        self.assertEqual((self.state / "backup/policy6.governor").read_text().strip(), "ondemand")
        self.call("restore")
        self.assertEqual((self.governor(4), self.governor(6)), ("schedutil", "ondemand"))
        self.assertFalse((self.state / "backup").exists())
        self.assertFalse((self.state / ".lock").exists())
        self.call("restore")
        self.assert_untouched()

    def test_missing_performance_on_either_policy_refuses_every_write(self):
        (self.policies[6] / "scaling_available_governors").write_text("ondemand userspace\n")
        self.assertIn("both big-core policies", self.call("enable", success=False).stderr)
        self.assertEqual((self.governor(4), self.governor(6)), ("schedutil", "ondemand"))
        self.assertFalse(self.state.exists())

    def test_related_cpus_must_exactly_match_confirmed_rk3588_clusters(self):
        (self.policies[4] / "related_cpus").write_text("0 1\n")
        self.assertIn("unexpected related_cpus", self.call("enable", success=False).stderr)
        self.assertFalse(self.state.exists())
        self.assertEqual((self.governor(4), self.governor(6)), ("schedutil", "ondemand"))

    @unittest.skipIf(os.geteuid() == 0, "read-only sysfs simulation needs an unprivileged test user")
    def test_partial_enable_failure_restores_both_previous_values(self):
        (self.policies[6] / "scaling_governor").chmod(0o444)
        result = self.call("enable", success=False)
        self.assertIn("rolled back both previous governors", result.stderr)
        self.assertEqual((self.governor(4), self.governor(6)), ("schedutil", "ondemand"))
        self.assertTrue((self.state / "backup/policy4.governor").exists())
        self.assertFalse((self.state / ".lock").exists())
        self.assert_untouched()

    @unittest.skipIf(os.geteuid() == 0, "read-only sysfs simulation needs an unprivileged test user")
    def test_partial_restore_failure_returns_both_to_preoperation_values(self):
        self.call("enable")
        (self.policies[6] / "scaling_governor").chmod(0o444)
        result = self.call("restore", success=False)
        self.assertIn("rolled back both previous governors", result.stderr)
        self.assertEqual((self.governor(4), self.governor(6)), ("performance", "performance"))
        self.assertTrue((self.state / "backup/policy6.governor").exists())
        (self.policies[6] / "scaling_governor").chmod(0o644)
        self.call("restore")
        self.assertEqual((self.governor(4), self.governor(6)), ("schedutil", "ondemand"))
        self.assert_untouched()

    def test_restore_rejects_changed_canonical_policy_before_any_write(self):
        self.call("enable")
        old_node = self.root / "old-policy6"
        self.policies[6].rename(old_node)
        self.policies[6].symlink_to(old_node, target_is_directory=True)
        self.assertIn("different CPU policy targets", self.call("restore", success=False).stderr)
        self.assertEqual((self.governor(4), self.governor(6)), ("performance", "performance"))
        self.assertTrue((self.state / "backup/policy4.governor").exists())

    def test_backup_governors_are_data_never_shell_source(self):
        self.call("enable")
        marker = self.root / "must-not-exist"
        (self.state / "backup/policy4.governor").write_text(f"schedutil; touch {marker}\n")
        self.assertIn("invalid saved CPU", self.call("restore", success=False).stderr)
        self.assertFalse(marker.exists())
        self.assertEqual((self.governor(4), self.governor(6)), ("performance", "performance"))
        self.assert_untouched()

    def test_state_symlink_and_operation_lock_are_rejected(self):
        other = self.root / "another-directory"
        other.mkdir()
        self.state.symlink_to(other, target_is_directory=True)
        self.assertIn("symbolic link", self.call("enable", success=False).stderr)
        self.state.unlink()
        self.state.mkdir(mode=0o700)
        (self.state / ".lock").mkdir()
        self.assertIn("state lock", self.call("enable", success=False).stderr)
        self.assertEqual((self.governor(4), self.governor(6)), ("schedutil", "ondemand"))

    def test_restore_without_backup_does_not_create_state(self):
        self.call("restore")
        self.assertFalse(self.state.exists())
        self.assert_untouched()

    def test_shared_state_root_refused_without_chmod(self):
        environment = dict(self.env, CPU_STATE_DIR="/tmp/..")
        before = stat.S_IMODE(Path("/").stat().st_mode)
        self.assertIn("shared CPU_STATE_DIR", self.call("enable", success=False, env=environment).stderr)
        self.assertEqual(stat.S_IMODE(Path("/").stat().st_mode), before)


if __name__ == "__main__":
    unittest.main(verbosity=2)
