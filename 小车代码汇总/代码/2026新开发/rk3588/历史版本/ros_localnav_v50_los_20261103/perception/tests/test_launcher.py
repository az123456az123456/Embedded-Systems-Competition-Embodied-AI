#!/usr/bin/env python3
"""Reuse the isolated launcher regression suite with the new default contract."""
import importlib.util
from pathlib import Path
import socket
import unittest

ROOT = Path(__file__).resolve().parents[1]
OLD_TEST = ROOT / "tests/launcher_fixture.py"
spec = importlib.util.spec_from_file_location("original_launcher_tests", OLD_TEST)
legacy = importlib.util.module_from_spec(spec)
spec.loader.exec_module(legacy)
legacy.LAUNCHER = ROOT / "run-perception.sh"
legacy.DISPATCH = legacy.DISPATCH.replace(
    '"isp_display": os.environ.get("ISP_DISPLAY")}',
    '"isp_display": os.environ.get("ISP_DISPLAY"), "camera_target_fps": os.environ.get("CAMERA_TARGET_FPS")}')


class UpdatedLauncherTest(legacy.LauncherTest):
    def setUp(self):
        super().setUp()
        self.env.pop("CAMERA_TARGET_FPS", None)
        self.env.pop("PERCEPTION_CAMERA_SOURCE", None)
        self.env.pop("PERCEPTION_CAMERA_PROBE_BINARY", None)
        self.env.pop("PERCEPTION_CAMERA_PROBE", None)
        (self.package / "model/acg138-best-e7f26253-int8-rk3588.rknn").write_bytes(b"fixture only; never loaded")

    def test_board_defaults_start_isp_and_rediscover_until_nv12(self):
        self.run_launcher(changes={"MOCK_VIDEO_SEQUENCE": '["/dev/zero", "/dev/null"]',
                                   "MOCK_NV12_AFTER": "2", "LD_LIBRARY_PATH": "/existing/libs"},
                          startup=True)
        self.assertEqual(self.option("--camera"), "/dev/null")
        self.assertEqual(self.option("--model"), str(self.package / "model/acg138-best-e7f26253-int8-rk3588.rknn"))
        self.assertEqual(self.option("--labels"), str(self.package / "model/acg138-best-e7f26253-labels.txt"))
        self.assertEqual(len(self.commands_seen("finder")), 2)
        self.assertEqual(len(self.commands_seen("v4l2-ctl")), 2)
        startup = self.commands_seen("nohup")[0]
        self.assertEqual(startup["args"], ["setsid", "sh", str(self.isp), "dual-1080", "100000"])
        self.assertEqual(startup["camera_target_fps"], "40")
        self.assertEqual(startup["isp_display"], "0")
        self.assertEqual(self.output()["xdg_runtime_dir"], str(self.runtime))
        self.assertEqual(self.output()["wayland_display"], "wayland-existing")

    def test_package_model_fallback_and_auto_without_isp(self):
        # The verified, clearly named YOLO11 model stays selected even when the
        # old demo's YOLOv8 model is absent. There is no silent model downgrade.
        self.isp.unlink()
        (self.legacy / "model/yolov8s.rknn").unlink()
        self.run_launcher(changes={"MOCK_ISP_RUNNING": "0"})
        self.assertEqual(self.option("--model"), str(self.package / "model/acg138-best-e7f26253-int8-rk3588.rknn"))
        self.assertEqual(self.option("--labels"), str(self.package / "model/acg138-best-e7f26253-labels.txt"))
        self.assertFalse(self.commands_seen("nohup"))

    def mock_probe(self, mode, code=0):
        probe = self.package / "camera-link-probe.aarch64"
        probe.write_text("#!/bin/sh\nprintf '%s\\n' '" + mode + "'\nexit " + str(code) + "\n")
        probe.chmod(0o755)

    def test_stale_wayland_environment_rediscovers_real_socket(self):
        self.run_launcher(changes={"WAYLAND_DISPLAY": "deleted-wayland", "MOCK_ISP_RUNNING": "1"})
        self.assertEqual(self.output()["wayland_display"], "wayland-existing")
        self.assertEqual(self.output()["xdg_runtime_dir"], str(self.runtime))
        self.assertFalse(self.commands_seen("nohup"))

    def test_private_wayland_socket_is_reused_without_mouse(self):
        (self.runtime / "wayland-existing").unlink()
        private = self.root / "private-wayland"
        private.mkdir()
        endpoint = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        self.addCleanup(endpoint.close)
        endpoint.bind(str(private / "wayland-private"))
        self.run_launcher(changes={"WAYLAND_DISPLAY": "stale", "MOCK_ISP_RUNNING": "1"})
        self.assertEqual(self.output()["xdg_runtime_dir"], str(private))
        self.assertEqual(self.output()["wayland_display"], "wayland-private")
        self.assertFalse(self.commands_seen("nohup"))

    def test_missing_display_starts_mock_weston_without_input(self):
        (self.runtime / "wayland-existing").unlink()
        self.run_launcher(changes={"MOCK_ISP_RUNNING": "1"})
        calls = self.commands_seen("nohup")
        self.assertEqual(len(calls), 1)
        self.assertIn("weston", calls[0]["args"])
        private = self.root / "private-wayland"
        self.assertEqual(self.output()["xdg_runtime_dir"], str(private))
        self.assertTrue((private / "wayland-0").is_socket())
        config = next(private.glob("weston.*")).read_text()
        self.assertIn("require-input=false", config)
        self.assertIn("idle-time=0", config)
        self.assertIn("locking=false", config)

    def test_missing_display_and_weston_fails_clearly(self):
        (self.runtime / "wayland-existing").unlink()
        (self.commands / "weston").unlink()
        answer = self.run_launcher(changes={"MOCK_ISP_RUNNING": "1"}, expected=1)
        self.assertIn("No Wayland display or system Weston", answer.stderr)
        self.assertFalse(self.result.exists())

    def test_auto_detects_raw_and_runs_isp(self):
        self.mock_probe("v4l2")
        self.run_launcher(startup=True)
        self.assertEqual(self.option("--camera"), "/dev/null")
        self.assertTrue(self.commands_seen("nohup"))

    def test_auto_detects_rgb_without_starting_raw_isp(self):
        self.mock_probe("fpga-rgb")
        self.run_launcher()
        self.assertEqual(self.option("--camera"), "fpga-rgb:eth0")
        self.assertFalse(self.commands_seen("nohup"))
        self.assertFalse(self.commands_seen("v4l2-ctl"))

    def test_old_rgb_override_rejects_raw_bitstream(self):
        self.mock_probe("v4l2")
        answer = self.run_launcher(changes={"PERCEPTION_CAMERA_SOURCE": "fpga-rgb"}, expected=2)
        self.assertIn("protocol mismatch", answer.stderr)
        self.assertFalse(self.commands_seen("nohup"))
        self.assertFalse(self.result.exists())

    def test_no_camera_packets_do_not_launch_wrong_protocol(self):
        self.mock_probe("", 1)
        answer = self.run_launcher(expected=1)
        self.assertIn("No recognized FPGA", answer.stderr)
        self.assertFalse(self.result.exists())

    def test_requested_camera_rate_environment_remains_authoritative(self):
        self.run_launcher(changes={"CAMERA_TARGET_FPS": "20"}, startup=True)
        self.assertEqual(self.commands_seen("nohup")[0]["camera_target_fps"], "20")

    def test_invalid_camera_rate_fails_before_any_side_effect(self):
        result = self.run_launcher(changes={"CAMERA_TARGET_FPS": "31"}, expected=2)
        self.assertIn("CAMERA_TARGET_FPS", result.stderr)
        self.assertFalse(self.events())


if __name__ == "__main__":
    unittest.main(verbosity=2)
