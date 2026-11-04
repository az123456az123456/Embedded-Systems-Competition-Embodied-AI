"""Run the read-only board diagnostic against isolated sysfs/proc fixtures."""
from pathlib import Path
import os
import shutil
import subprocess
import tempfile
import unittest


SCRIPT = Path(__file__).resolve().parents[1] / "scripts/diagnose-performance.sh"


class DiagnosticTests(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory(prefix="performance diagnostic ")
        self.addCleanup(temporary.cleanup)
        self.root = Path(temporary.name)
        self.sysroot, self.procroot = self.root / "sys", self.root / "proc"
        self.sysroot.mkdir()
        self.procroot.mkdir()

    def write(self, root, name, value):
        path = root / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(value)

    def run_diagnostic(self, shell="/bin/sh", arguments=()):
        before = {str(p.relative_to(self.root)): p.read_bytes() for p in self.root.rglob("*") if p.is_file()}
        environment = dict(os.environ, SYSFS_ROOT=str(self.sysroot), PROCFS_ROOT=str(self.procroot))
        command = [shell]
        if Path(shell).name == "busybox":
            command.append("sh")
        result = subprocess.run(command + [str(SCRIPT), *arguments], env=environment,
                                capture_output=True, text=True, timeout=10)
        after = {str(p.relative_to(self.root)): p.read_bytes() for p in self.root.rglob("*") if p.is_file()}
        self.assertEqual(before, after, "diagnostic must not change any sysfs/proc fixture")
        return result

    def complete_fixture(self):
        for name, value in {
            "class/devfreq/fdab0000.npu/name": "fdab0000.npu\n",
            "class/devfreq/fdab0000.npu/cur_freq": "1000000000\n",
            "class/devfreq/fdab0000.npu/available_frequencies": "300000000 600000000 1000000000\n",
            "class/devfreq/fdab0000.npu/governor": "rknpu_ondemand\n",
            "class/devfreq/fdab0000.npu/min_freq": "300000000\n",
            "class/devfreq/fdab0000.npu/max_freq": "1000000000\n",
            "class/devfreq/fb000000.gpu/name": "gpu\n",
            "class/devfreq/fb000000.gpu/cur_freq": "999999999\n",
            "kernel/debug/rknpu/load": "NPU load:  Core0: 87%, Core1: 89%, Core2: 85%,\n",
            "kernel/debug/rknpu/version": "0.9.8\n",
            "class/thermal/thermal_zone0/type": "soc-thermal\n",
            "class/thermal/thermal_zone0/temp": "62500\n",
            "devices/system/cpu/cpufreq/policy0/related_cpus": "0 1 2 3\n",
            "devices/system/cpu/cpufreq/policy0/scaling_governor": "schedutil\n",
            "devices/system/cpu/cpufreq/policy0/scaling_cur_freq": "1800000\n",
        }.items():
            self.write(self.sysroot, name, value)
        self.write(self.procroot, "version", "Linux version 6.1.75 test-kernel\n")
        self.write(self.procroot, "uptime", "123.4 987.6\n")
        self.write(self.procroot, "meminfo", "MemTotal: 8000000 kB\nMemAvailable: 5000000 kB\n"
                   "CmaFree: 512000 kB\nUnrelated: 12345 kB\n")
        mapping = "ffff-ffff r-xp 00000000 00:01 123 /root/app lib/librknnrt.so\n"
        self.write(self.procroot, "123/maps", mapping + "ffff-ffff r--p 00000000 00:01 123 /lib/libc.so\n")
        self.write(self.procroot, "456/maps", mapping)

    def test_complete_snapshot_is_read_only_and_does_not_use_host_roots(self):
        self.complete_fixture()
        result = self.run_diagnostic()
        self.assertEqual(result.returncode, 0, result.stderr)
        for value in ("6.1.75 test-kernel", "current_freq=1000000000", "governor=rknpu_ondemand",
                      "Core0: 87%, Core1: 89%, Core2: 85%", "whole NPU device (all processes)",
                      "driver_version=0.9.8", "temp_mC=62500", "current_freq=1800000",
                      "MemAvailable: 5000000 kB", "CmaFree: 512000 kB", "runtime_version=not inferred"):
            self.assertIn(value, result.stdout)
        self.assertEqual(result.stdout.count("/root/app lib/librknnrt.so"), 1)
        self.assertNotIn("999999999", result.stdout)
        self.assertNotIn("Unrelated: 12345", result.stdout)
        self.assertNotIn("libc.so", result.stdout)

    def test_missing_interfaces_are_normal_and_need_no_root_or_python(self):
        result = self.run_diagnostic()
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("load=unavailable", result.stdout)
        self.assertIn("no readable process mapping", result.stdout)
        self.assertIn("No settings were changed", result.stdout)
        self.assertEqual(result.stderr, "")

    def test_older_cpu_layout_and_proc_driver_layout(self):
        self.write(self.sysroot, "devices/system/cpu/cpu4/cpufreq/scaling_governor", "performance\n")
        self.write(self.sysroot, "devices/system/cpu/cpu4/cpufreq/scaling_cur_freq", "2400000\n")
        self.write(self.procroot, "rknpu/load", "Core0: 25%\n")
        self.write(self.procroot, "rknpu/version", "0.9.6\n")
        result = self.run_diagnostic()
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("cpu4", result.stdout)
        self.assertIn("governor=performance", result.stdout)
        self.assertIn("current_freq=2400000", result.stdout)
        self.assertIn("Core0: 25%", result.stdout)
        self.assertIn("driver_version=0.9.6", result.stdout)

    def test_deleted_runtime_mapping_is_distinguished(self):
        self.write(self.procroot, "123/maps",
                   "ffff-ffff r-xp 00000000 00:01 123 /root/lib/librknnrt.so (deleted)\n")
        result = self.run_diagnostic()
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("/root/lib/librknnrt.so (deleted)", result.stdout)

    def test_busybox_shell_matches_posix_shell(self):
        busybox = shutil.which("busybox")
        if not busybox:
            self.skipTest("busybox is not installed on the test host")
        self.complete_fixture()
        result = self.run_diagnostic(busybox)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("Core2: 85%", result.stdout)

    def test_unknown_arguments_fail_without_reading_or_modifying_system(self):
        result = self.run_diagnostic(arguments=("--set-frequency",))
        self.assertEqual(result.returncode, 2)
        self.assertIn("Unknown argument", result.stderr)


if __name__ == "__main__":
    unittest.main()
