#!/usr/bin/env python3
"""Reuse the isolated launcher regression suite with the new default contract."""
import importlib.util
from pathlib import Path
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
        (self.package / "model/yolo11s-int8-rk3588.rknn").write_bytes(b"fixture only; never loaded")

    def test_board_defaults_start_isp_and_rediscover_until_nv12(self):
        self.run_launcher(changes={"MOCK_VIDEO_SEQUENCE": '["/dev/zero", "/dev/null"]',
                                   "MOCK_NV12_AFTER": "2", "LD_LIBRARY_PATH": "/existing/libs"},
                          startup=True)
        self.assertEqual(self.option("--camera"), "/dev/null")
        self.assertEqual(self.option("--model"), str(self.package / "model/yolo11s-int8-rk3588.rknn"))
        self.assertEqual(self.option("--labels"), str(self.package / "model/coco_80_labels_list.txt"))
        self.assertEqual(len(self.commands_seen("finder")), 2)
        self.assertEqual(len(self.commands_seen("v4l2-ctl")), 2)
        startup = self.commands_seen("nohup")[0]
        self.assertEqual(startup["args"], ["setsid", "sh", str(self.isp), "dual-1080", "100000"])
        self.assertEqual(startup["camera_target_fps"], "30")
        self.assertEqual(startup["isp_display"], "0")
        self.assertEqual(self.output()["xdg_runtime_dir"], "/mock/existing-runtime")
        self.assertEqual(self.output()["wayland_display"], "existing-wayland")

    def test_package_model_fallback_and_auto_without_isp(self):
        # The verified, clearly named YOLO11 model stays selected even when the
        # old demo's YOLOv8 model is absent. There is no silent model downgrade.
        self.isp.unlink()
        (self.legacy / "model/yolov8s.rknn").unlink()
        self.run_launcher(changes={"MOCK_ISP_RUNNING": "0"})
        self.assertEqual(self.option("--model"), str(self.package / "model/yolo11s-int8-rk3588.rknn"))
        self.assertEqual(self.option("--labels"), str(self.package / "model/coco_80_labels_list.txt"))
        self.assertFalse(self.commands_seen("nohup"))

    def test_requested_camera_rate_environment_remains_authoritative(self):
        self.run_launcher(changes={"CAMERA_TARGET_FPS": "20"}, startup=True)
        self.assertEqual(self.commands_seen("nohup")[0]["camera_target_fps"], "20")

    def test_invalid_camera_rate_fails_before_any_side_effect(self):
        result = self.run_launcher(changes={"CAMERA_TARGET_FPS": "31"}, expected=2)
        self.assertIn("CAMERA_TARGET_FPS", result.stderr)
        self.assertFalse(self.events())


if __name__ == "__main__":
    unittest.main(verbosity=2)
