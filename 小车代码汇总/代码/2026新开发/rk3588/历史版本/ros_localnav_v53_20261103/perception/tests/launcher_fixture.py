"""Exercise launch decisions using temporary files and mock commands only.

No camera, real ISP helper, current desktop, service, or model runtime is opened.
The character-device paths below are passed as text to a mocked v4l2-ctl.
"""
from pathlib import Path
import json
import os
import shlex
import shutil
import socket
import subprocess
import sys
import tempfile
import time
import unittest


LAUNCHER = Path(__file__).resolve().parents[1] / "run-perception.sh"
DISPATCH = r'''
from pathlib import Path
import json, os, socket, sys

command = sys.argv[1]
args = sys.argv[2:]
trace = Path(os.environ["MOCK_TRACE"])
entry = {"command": command, "args": args, "cwd": os.getcwd(),
         "isp_display": os.environ.get("ISP_DISPLAY")}
with trace.open("a") as output:
    output.write(json.dumps(entry) + "\n")
counter = Path(os.environ["MOCK_COUNTERS"]) / command
count = int(counter.read_text()) + 1 if counter.exists() else 1
counter.write_text(str(count))
if command == "ps":
    if os.environ.get("MOCK_ISP_RUNNING") == "1":
        print("root 123 1 0 00:00 ? 00:00:00 /usr/bin/isp-probe --camera dual-1080")
elif command == "finder":
    values = json.loads(os.environ.get("MOCK_VIDEO_SEQUENCE", '["/dev/null"]'))
    print(values[min(count - 1, len(values) - 1)])
elif command == "v4l2-ctl":
    ready = count >= int(os.environ.get("MOCK_NV12_AFTER", "1"))
    print("Format Video Capture: Pixel Format: '" + ("NV12" if ready else "RG10") + "'")
elif command == "uname":
    print(os.environ.get("MOCK_ARCH", "aarch64"))
elif command == "binary":
    result = {"argv": args, "ld_library_path": os.environ.get("LD_LIBRARY_PATH"),
              "xdg_runtime_dir": os.environ.get("XDG_RUNTIME_DIR"),
              "wayland_display": os.environ.get("WAYLAND_DISPLAY")}
    Path(os.environ["MOCK_RESULT"]).write_text(json.dumps(result))
elif command == "nohup" and "weston" in args:
    # Real Unix socket endpoint, simulated compositor only. No host Weston,
    # display, input device or system configuration is opened by this fixture.
    runtime = Path(os.environ["XDG_RUNTIME_DIR"])
    name = next(a.split("=", 1)[1] for a in args if a.startswith("--socket="))
    endpoint = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    endpoint.bind(str(runtime / name)); endpoint.close()
# nohup, setsid, sleep and the ISP fixture deliberately do nothing. In
# particular, the background startup command never executes the ISP script.
'''


class LauncherTest(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory(prefix="perception launcher ")
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        self.package = self.root / "application with spaces"
        self.commands = self.root / "mock-bin"
        self.legacy = self.root / "old yolo"
        self.trace = self.root / "trace.jsonl"
        self.result = self.root / "result.json"
        self.runtime = self.root / "wayland-runtime"
        self.runtime.mkdir()
        self.wayland_socket = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        self.wayland_socket.bind(str(self.runtime / "wayland-existing"))
        self.addCleanup(self.wayland_socket.close)
        self.dispatcher = self.root / "dispatch.py"
        self.dispatcher.write_text(DISPATCH)
        for directory in (self.package / "model", self.package / "lib",
                          self.commands, self.legacy / "model", self.legacy / "lib",
                          self.root / "counters"):
            directory.mkdir(parents=True, exist_ok=True)
        self.launcher = self.package / "run-perception.sh"
        shutil.copyfile(LAUNCHER, self.launcher)
        for command in ("ps", "v4l2-ctl", "nohup", "setsid", "sleep", "uname", "weston"):
            self.wrapper(self.commands / command, command)
        # An isolated PATH ensures absent commands cannot fall back to host
        # utilities. These are the only unmocked helper utilities required.
        for command in ("dirname", "sh", "grep", "mkdir", "chmod", "mktemp"):
            executable = shutil.which(command)
            self.assertIsNotNone(executable)
            (self.commands / command).symlink_to(executable)
        self.binary = self.package / "perception-dashboard.aarch64"
        self.wrapper(self.binary, "binary")
        self.finder = self.root / "find video.sh"
        self.wrapper(self.finder, "finder")
        self.isp = self.root / "ISP helper.sh"
        self.wrapper(self.isp, "isp-helper")
        for model in (self.legacy / "model/yolov8s.rknn",
                      self.package / "model/model.rknn"):
            model.write_bytes(b"mock model: never loaded\n")
        for labels in (self.legacy / "model/coco_80_labels_list.txt",
                       self.package / "model/coco_80_labels_list.txt"):
            labels.write_text("person\nbicycle\n")
        self.env = dict(os.environ)
        for key in ("MODEL_PATH", "LABELS_PATH", "CAMERA_DEVICE", "LD_LIBRARY_PATH",
                    "PERCEPTION_BINARY", "YOLO_DIR", "FIND_VIDEO_SCRIPT",
                    "ISP_CAMERA_SCRIPT", "ISP_LOG", "PERCEPTION_PREPARE_ISP",
                    "PERCEPTION_VIDEO_WAIT_ATTEMPTS"):
            self.env.pop(key, None)
        self.env.update(
            PATH=str(self.commands), YOLO_DIR=str(self.legacy),
            FIND_VIDEO_SCRIPT=str(self.finder), ISP_CAMERA_SCRIPT=str(self.isp),
            ISP_LOG=str(self.root / "ISP log.txt"),
            PERCEPTION_BINARY=str(self.binary), PERCEPTION_VIDEO_WAIT_ATTEMPTS="3",
            XDG_RUNTIME_DIR=str(self.runtime), WAYLAND_DISPLAY="wayland-existing",
            PERCEPTION_WAYLAND_RUNTIME=str(self.root / "private-wayland"),
            MOCK_TRACE=str(self.trace), MOCK_RESULT=str(self.result),
            MOCK_COUNTERS=str(self.root / "counters"),
        )

    def wrapper(self, path, command):
        path.write_text("#!/bin/sh\nexec " + shlex.quote(sys.executable) + " " +
                        shlex.quote(str(self.dispatcher)) + " " +
                        shlex.quote(command) + ' "$@"\n')
        path.chmod(0o755)

    def run_launcher(self, args=(), changes=None, expected=0, startup=False):
        env = dict(self.env)
        env.update(changes or {})
        completed = subprocess.run(["/bin/sh", str(self.launcher), *args], env=env,
                                   capture_output=True, text=True, timeout=10)
        self.assertEqual(completed.returncode, expected,
                         (completed.stdout, completed.stderr, self.events()))
        if startup:
            deadline = time.monotonic() + 3
            while not self.commands_seen("nohup") and time.monotonic() < deadline:
                time.sleep(0.01)
            self.assertTrue(self.commands_seen("nohup"), self.events())
        self.assertFalse(self.commands_seen("isp-helper"), "A real ISP helper must never run in this test")
        return completed

    def events(self):
        if not self.trace.exists():
            return []
        return [json.loads(line) for line in self.trace.read_text().splitlines() if line]

    def commands_seen(self, command):
        return [event for event in self.events() if event["command"] == command]

    def output(self):
        self.assertTrue(self.result.exists(), self.events())
        return json.loads(self.result.read_text())

    def option(self, key):
        args = self.output()["argv"]
        positions = [i for i, value in enumerate(args) if value == key]
        self.assertTrue(positions, (key, args))
        return args[positions[-1] + 1]

    def test_board_defaults_start_isp_and_rediscover_until_nv12(self):
        self.run_launcher(changes={"MOCK_VIDEO_SEQUENCE": '["/dev/zero", "/dev/null"]',
                                   "MOCK_NV12_AFTER": "2", "LD_LIBRARY_PATH": "/existing/libs"},
                          startup=True)
        self.assertEqual(self.option("--camera"), "/dev/null")
        self.assertEqual(self.option("--model"), str(self.legacy / "model/yolov8s.rknn"))
        self.assertEqual(self.option("--labels"), str(self.legacy / "model/coco_80_labels_list.txt"))
        self.assertEqual(len(self.commands_seen("finder")), 2)
        self.assertEqual(len(self.commands_seen("v4l2-ctl")), 2)
        self.assertEqual(len(self.commands_seen("sleep")), 1)
        startup = self.commands_seen("nohup")[0]
        self.assertEqual(startup["args"], ["setsid", "sh", str(self.isp), "dual-1080", "100000"])
        self.assertEqual(startup["isp_display"], "0")
        self.assertEqual(startup["cwd"], str(self.legacy))
        self.assertTrue(all(event["cwd"] == str(self.legacy) for event in self.commands_seen("finder")))
        out = self.output()
        self.assertEqual(out["ld_library_path"], ":".join(("/existing/libs", str(self.legacy / "lib"), str(self.package / "lib"))))
        self.assertEqual(out["xdg_runtime_dir"], str(self.runtime))
        self.assertEqual(out["wayland_display"], "wayland-existing")

    def test_existing_isp_is_reused(self):
        self.run_launcher(changes={"MOCK_ISP_RUNNING": "1"})
        self.assertEqual(len(self.commands_seen("ps")), 1)
        self.assertFalse(self.commands_seen("nohup"))
        self.assertEqual(self.option("--camera"), "/dev/null")

    def test_explicit_cli_options_override_environment_and_discovery(self):
        args = ["--camera", "/dev/full", "--model", "custom model.rknn",
                "--labels", "custom labels.txt", "--view", "left",
                "--confidence", "0.42", "--interface", "eth1"]
        self.run_launcher(args, {"MOCK_ISP_RUNNING": "1", "CAMERA_DEVICE": "/dev/zero",
                                 "MODEL_PATH": "environment model.rknn", "LABELS_PATH": "environment labels.txt"})
        self.assertFalse(self.commands_seen("finder"))
        self.assertEqual(self.commands_seen("v4l2-ctl")[0]["args"], ["-d", "/dev/full", "--get-fmt-video"])
        self.assertEqual(self.option("--camera"), "/dev/full")
        self.assertEqual(self.option("--model"), "custom model.rknn")
        self.assertEqual(self.option("--labels"), "custom labels.txt")
        self.assertEqual(self.output()["argv"][-len(args):], args)

    def test_environment_overrides_are_preserved(self):
        self.run_launcher(changes={"MOCK_ISP_RUNNING": "1", "CAMERA_DEVICE": "/dev/zero",
                                   "MODEL_PATH": "/custom/model.rknn", "LABELS_PATH": "/custom/labels.txt"})
        self.assertEqual(self.option("--camera"), "/dev/zero")
        self.assertEqual(self.option("--model"), "/custom/model.rknn")
        self.assertEqual(self.option("--labels"), "/custom/labels.txt")
        self.assertFalse(self.commands_seen("finder"))

    def test_demo_preview_help_have_no_camera_side_effects(self):
        for args in (["--demo"], ["--preview", "preview with spaces.ppm"], ["--help"]):
            with self.subTest(args=args):
                self.run_launcher(args, {"PERCEPTION_PREPARE_ISP": "1"})
        for command in ("ps", "nohup", "finder", "v4l2-ctl", "sleep"):
            self.assertFalse(self.commands_seen(command), self.events())

    def test_missing_option_values_fail_before_camera_preparation(self):
        for flag in ("--camera", "--model", "--labels", "--preview", "--interface", "--confidence"):
            with self.subTest(flag=flag):
                completed = self.run_launcher([flag], expected=2)
                self.assertIn("Missing command-line option value", completed.stderr)
        self.assertFalse(self.events())

    def test_preview_value_is_preserved_even_if_it_looks_like_an_option(self):
        args = ["--preview", "--camera", "--demo"]
        self.run_launcher(args)
        self.assertEqual(self.output()["argv"][-len(args):], args)
        for command in ("ps", "nohup", "finder", "v4l2-ctl", "sleep"):
            self.assertFalse(self.commands_seen(command), self.events())

    def test_package_model_fallback_and_auto_without_isp(self):
        self.isp.unlink()
        (self.legacy / "model/yolov8s.rknn").unlink()
        self.run_launcher()
        self.assertEqual(self.option("--model"), str(self.package / "model/model.rknn"))
        self.assertEqual(self.option("--labels"), str(self.package / "model/coco_80_labels_list.txt"))
        self.assertFalse(self.commands_seen("ps"))
        self.assertFalse(self.commands_seen("v4l2-ctl"))
        self.assertFalse(self.commands_seen("nohup"))
        self.assertFalse(self.output()["ld_library_path"].startswith(":"))

    def test_explicit_skip_isp_keeps_camera_discovery(self):
        self.run_launcher(changes={"PERCEPTION_PREPARE_ISP": "0", "MOCK_NV12_AFTER": "999"})
        self.assertEqual(self.option("--camera"), "/dev/null")
        self.assertEqual(len(self.commands_seen("finder")), 1)
        self.assertFalse(self.commands_seen("ps"))
        self.assertFalse(self.commands_seen("nohup"))
        self.assertFalse(self.commands_seen("v4l2-ctl"))

    def test_missing_required_isp_helper_fails_clearly(self):
        self.isp.unlink()
        completed = self.run_launcher(changes={"PERCEPTION_PREPARE_ISP": "1"}, expected=1)
        self.assertIn("ISP camera script not found", completed.stderr)
        self.assertFalse(self.result.exists())
        self.assertFalse(self.commands_seen("nohup"))

    def test_readiness_timeout_is_bounded_and_never_launches_gui(self):
        completed = self.run_launcher(changes={"MOCK_ISP_RUNNING": "1", "MOCK_NV12_AFTER": "999"}, expected=1)
        self.assertIn("not ready", completed.stderr)
        self.assertEqual(len(self.commands_seen("finder")), 3)
        self.assertEqual(len(self.commands_seen("v4l2-ctl")), 3)
        self.assertEqual(len(self.commands_seen("sleep")), 2)
        self.assertFalse(self.result.exists())

    def test_invalid_settings_fail_without_launching_gui(self):
        for changes in ({"PERCEPTION_PREPARE_ISP": "invalid"},
                        {"PERCEPTION_PREPARE_ISP": "0", "PERCEPTION_VIDEO_WAIT_ATTEMPTS": "0"},
                        {"PERCEPTION_PREPARE_ISP": "0", "PERCEPTION_VIDEO_WAIT_ATTEMPTS": "bad"},
                        {"PERCEPTION_PREPARE_ISP": "0", "PERCEPTION_VIDEO_WAIT_ATTEMPTS": "301"}):
            with self.subTest(changes=changes):
                self.run_launcher(changes=changes, expected=2)
                self.assertFalse(self.result.exists())
        self.assertFalse(self.commands_seen("nohup"))

    def test_invalid_wait_is_rejected_before_isp_startup_checks(self):
        self.run_launcher(changes={"PERCEPTION_PREPARE_ISP": "1", "PERCEPTION_VIDEO_WAIT_ATTEMPTS": "0"}, expected=2)
        self.assertFalse(self.commands_seen("ps"), self.events())
        self.assertFalse(self.commands_seen("nohup"), self.events())

    def test_invalid_device_never_opens_or_launches_gui(self):
        completed = self.run_launcher(changes={"PERCEPTION_PREPARE_ISP": "0",
                                               "MOCK_VIDEO_SEQUENCE": '["/nonexistent/video"]'}, expected=1)
        self.assertIn("not ready", completed.stderr)
        self.assertFalse(self.commands_seen("v4l2-ctl"))
        self.assertFalse(self.result.exists())

    def test_native_architecture_binary_lookup_and_missing_sets_id(self):
        (self.commands / "setsid").unlink()
        self.env.pop("PERCEPTION_BINARY")
        self.run_launcher(startup=True)
        self.assertEqual(self.commands_seen("nohup")[0]["args"], ["sh", str(self.isp), "dual-1080", "100000"])
        self.assertEqual(len(self.commands_seen("uname")), 1)

    def test_missing_v4l2_command_reports_dependency(self):
        (self.commands / "v4l2-ctl").unlink()
        completed = self.run_launcher(expected=1)
        self.assertIn("v4l2-ctl is required", completed.stderr)
        self.assertFalse(self.result.exists())
        self.assertFalse(self.commands_seen("nohup"))


if __name__ == "__main__":
    unittest.main(verbosity=2)
