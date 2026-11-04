#!/usr/bin/env python3
"""Exercise model/runtime/launcher transactions using real ARM payload bytes.

Payloads are copied and hashed only, never executed. All operations are confined
to per-case temporary roots via the installer's explicit fixture option.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import stat
import subprocess
import tempfile


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--busybox", action="store_true")
    ap.add_argument("--package", type=Path)
    ap.add_argument("--report", type=Path)
    args = ap.parse_args()
    source = Path(__file__).resolve().parents[1]
    workspace = source.parents[2]
    dash = workspace / "rk3588/robot_sonar_avoid_v33/dashboard/build/perception-dashboard.aarch64"
    nav = workspace / "rk3588/robot_sonar_avoid_v33/navigation/build/robot-navigation.aarch64"
    old_dash = sha(dash)
    old_nav = sha(nav)
    shell = ["busybox", "sh"] if args.busybox else ["sh"]
    checks = 0
    cases = []

    def ck(condition, message):
        nonlocal checks
        assert condition, message
        checks += 1

    with tempfile.TemporaryDirectory(prefix="acg138-v34-installer-") as temporary:
        root = Path(temporary)
        package = root / "package"
        shutil.copytree(args.package or source, package, ignore=shutil.ignore_patterns("__pycache__", "reports", "tests"))
        # In final-package mode keep and verify the actual publisher seal.
        # Early development snapshots without --package are sealed locally.
        files = sorted(p for p in package.rglob("*") if p.is_file() and p.name != "SHA256SUMS")
        if args.package:
            verified = subprocess.run(["sha256sum", "-c", "SHA256SUMS"], cwd=package, capture_output=True, text=True)
            ck(verified.returncode == 0, "Original supplied manifest failed: " + verified.stdout + verified.stderr)
        else:
            (package / "SHA256SUMS").write_text("".join(f"{sha(p)}  {p.relative_to(package)}\n" for p in files))
        supplied_manifest_sha256 = sha(package / "SHA256SUMS")
        package_binding = {str(p.relative_to(package)): sha(p) for p in files}
        for name in ("dashboard", "navigation"):
            raw = (package / "payload" / name).read_bytes()
            ck(raw[:6] == b"\x7fELF\x02\x01" and raw[18:20] == b"\xb7\x00", "Real AArch64 payload required")
        ck(f"{old_dash} {old_nav}" in (package / "baseline-pairs.sha256").read_text(), "v33 baseline missing")
        tools = root / "tools"
        tools.mkdir()
        commands = {
            "id": '[ "$1" != -u ] || { echo "${TEST_UID:-0}"; exit; }; exec /usr/bin/id "$@"',
            "uname": '[ "$1" != -m ] || { echo "${TEST_ARCH:-aarch64}"; exit; }; exec /usr/bin/uname "$@"',
            "mv": '''last=
for last; do :; done
if [ "${last##*/}" = "${TEST_TARGET:-disabled}" ] && [ ! -f "$TEST_MARKER" ]; then
 : > "$TEST_MARKER"
 case "$TEST_FAILURE" in
 before) exit 73 ;;
 after) /usr/bin/mv "$@"; exit 73 ;;
 corrupt) /usr/bin/mv "$@"; printf corrupt > "$last"; exit 0 ;;
 signal) /usr/bin/mv "$@"; kill -TERM "$PPID"; exit 0 ;;
 esac
fi
exec /usr/bin/mv "$@"''',
        }
        for name, body in commands.items():
            path = tools / name
            path.write_text("#!/bin/sh\nset -eu\n" + body + "\n")
            path.chmod(0o755)
        originals = {
            "restore": (workspace / "rk3588/acg138-restore-20261023/launchers/run-perception.sh").read_bytes(),
            "source-block": (workspace / "rk3588/robot_sonar_avoid_v33/dashboard/run-perception.sh").read_bytes(),
            "raw-default": b'''#!/bin/sh
set -eu
APP_DIR=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
CAMERA_TARGET_FPS=${CAMERA_TARGET_FPS:-40}
MODEL_PATH=${MODEL_PATH:-$APP_DIR/model/industrial-yolov8s-int8-rk3588.rknn}
LABELS_PATH=${LABELS_PATH:-$APP_DIR/model/industrial_labels_list.txt}
printf '%s\\n' "$CAMERA_TARGET_FPS"
''',
        }

        def fixture(name, launcher="source-block", versioned=False):
            fixture_root = root / name
            app = fixture_root / "root/robot-perception"
            navigation = fixture_root / "root/robot-navigation"
            app.mkdir(parents=True)
            navigation.mkdir()
            (fixture_root / "proc").mkdir()
            (fixture_root / "root/perception-dashboard").symlink_to(app, target_is_directory=True)
            shutil.copy2(dash, app / "perception-dashboard.aarch64")
            shutil.copy2(nav, navigation / "robot-navigation.aarch64")
            (navigation / "dashboard.sha256").write_text(old_dash + "\n")
            (app / "run-perception.sh").write_bytes(originals[launcher])
            # Exact real v23 orchestration and config are preserved byte-for-byte.
            shutil.copy2(workspace / "rk3588/robot_navigation_v23/run-dashboard.sh", app / "run-dashboard.sh")
            shutil.copy2(workspace / "rk3588/robot_navigation_v23/run-navigation.sh", navigation / "run-navigation.sh")
            shutil.copy2(workspace / "rk3588/robot_navigation_v23/config.example.json", navigation / "config.json")
            (app / "lib").mkdir()
            for lib in ["librknnrt.so", "libwayland-client.so.0"]:
                (app / "lib" / lib).write_bytes(b"preserved library\n")
            (app / "model").mkdir()
            (app / "model/industrial-yolov8s-int8-rk3588.rknn").write_bytes(b"preserved old model\n")
            for p, data in [(app / "pair-token.txt", b"193820\n"),
                            (app / "phone-preferences.json", b'{"left_mask":10,"invert_mask":15}\n'),
                            (navigation / "map.bin", b"preserved map\n"),
                            (navigation / "wrong.log", b"preserved diagnostics\n")]:
                p.write_bytes(data)
            if versioned:
                for role, filename in roles.items():
                    if role in ("model", "labels", "manifest"):
                        shutil.copy2(package / "payload" / role, app / "model" / filename)
            return fixture_root, app, navigation

        roles = {
            "model": "acg138-best-e7f26253-int8-rk3588.rknn",
            "labels": "acg138-best-e7f26253-labels.txt",
            "manifest": "acg138-best-e7f26253-manifest.json",
            "dashboard": "perception-dashboard.aarch64", "navigation": "robot-navigation.aarch64",
            "launcher": "run-perception.sh", "pair": "dashboard.sha256",
        }

        def targets(app, navigation):
            return {role: (app / "model" / filename if role in ("model", "labels", "manifest")
                           else navigation / filename if role in ("navigation", "pair") else app / filename)
                    for role, filename in roles.items()}

        def snapshot(app, navigation):
            return {str(p.relative_to(app.parent)): (sha(p), stat.S_IMODE(p.stat().st_mode))
                    for base in (app, navigation) for p in base.rglob("*") if p.is_file()}

        def run(fixture_root, command="install.sh", arguments=(), failure=None):
            env = dict(os.environ, PATH=str(tools) + os.pathsep + os.environ["PATH"],
                       ROBOT_V34_DESTDIR=str(fixture_root), TEST_MARKER=str(fixture_root / "injected"),
                       TEST_TOOLS=str(tools))
            if failure:
                env.update(TEST_TARGET=roles[failure[0]], TEST_FAILURE=failure[1])
            invocation = [*shell, str(package / command), *map(str, arguments)]
            if args.busybox:
                # Standalone ash resolves these applets before PATH; override
                # only architecture/uid fixtures and the failure injection seam.
                invocation = ["busybox", "sh", "-c", 'id() { "$TEST_TOOLS/id" "$@"; }; uname() { "$TEST_TOOLS/uname" "$@"; }; mv() { "$TEST_TOOLS/mv" "$@"; }; . "$0"',
                              str(package / command), *map(str, arguments)]
            result = subprocess.run(invocation, env=env,
                                    capture_output=True, text=True, timeout=45)
            return result

        for launcher in originals:
            f, app, navigation = fixture("roundtrip-" + launcher, launcher)
            before = snapshot(app, navigation)
            checked = run(f, arguments=["--check-only"])
            ck(checked.returncode == 0, checked.stdout + checked.stderr)
            ck(snapshot(app, navigation) == before, "Check-only changed installed files")
            installed = run(f)
            ck(installed.returncode == 0, installed.stdout + installed.stderr)
            after = snapshot(app, navigation)
            backup = Path(re.search(r"^Backup: (.+)$", installed.stdout, re.M)[1])
            for role, dest in targets(app, navigation).items():
                if role == "launcher":
                    changed = dest.read_bytes()
                    expected = subprocess.check_output(["awk", "-f", str(package / "transform-launcher.awk")], input=originals[launcher])
                    ck(changed == expected, "Launcher changed outside precise transform")
                    ck(b"acg138-best-e7f26253-int8-rk3588.rknn" in changed, "Model default unchanged")
                    ck(b"acg138-best-e7f26253-labels.txt" in changed, "Labels default unchanged")
                elif role == "pair":
                    ck(dest.read_text() == sha(package / "payload/dashboard") + "\n", "Pair digest incorrect")
                else:
                    ck(sha(dest) == sha(package / "payload" / role), "Installed role mismatch")
            ck(run(f).returncode == 0, "Idempotent reinstall failed")
            ck(snapshot(app, navigation) == after, "Idempotent reinstall changed runtime")
            rolled = run(f, "rollback.sh", [backup])
            ck(rolled.returncode == 0, rolled.stdout + rolled.stderr)
            ck(snapshot(app, navigation) == before, "Rollback did not restore all old files/modes")
            ck(run(f, "rollback.sh", [backup]).returncode == 0, "Idempotent rollback failed")
            cases.append({"case": "roundtrip-" + launcher, "status": "PASS"})

        for role in roles:
            for fault in ("before", "after", "corrupt", "signal"):
                f, app, navigation = fixture("install-fail-" + role + "-" + fault)
                before = snapshot(app, navigation)
                failed = run(f, failure=(role, fault))
                ck(failed.returncode != 0, f"Injected {role}/{fault} install succeeded")
                recovered = snapshot(app, navigation)
                ck(recovered == before, f"Install failed to recover {role}/{fault}: " +
                   repr({k: (before.get(k), recovered.get(k)) for k in before.keys() | recovered.keys()
                         if before.get(k) != recovered.get(k)}) + "\n" + failed.stdout + failed.stderr)
                ck(not (f / "root/.robot-control-v21.lock").exists(), "Failed install leaked lock")
                cases.append({"case": "install-fail-" + role + "-" + fault, "status": "PASS"})

        for role in roles:
            for fault in ("before", "after", "corrupt", "signal"):
                f, app, navigation = fixture("rollback-fail-" + role + "-" + fault, versioned=True)
                initial = snapshot(app, navigation)
                result = run(f)
                ck(result.returncode == 0, result.stdout + result.stderr)
                backup = Path(re.search(r"^Backup: (.+)$", result.stdout, re.M)[1])
                installed = snapshot(app, navigation)
                failed = run(f, "rollback.sh", [backup], failure=(role, fault))
                ck(failed.returncode != 0, f"Injected {role}/{fault} rollback succeeded")
                ck(snapshot(app, navigation) == installed, f"Rollback failed to recover v34 {role}/{fault}")
                ck(run(f, "rollback.sh", [backup]).returncode == 0, "Retry rollback failed")
                ck(snapshot(app, navigation) == initial, "Retry rollback did not restore baseline")
                cases.append({"case": "rollback-fail-" + role + "-" + fault, "status": "PASS"})

        for kind in ("custom-model", "ambiguous-model", "foreign-versioned", "active-process", "mixed-pair"):
            f, app, navigation = fixture("reject-" + kind)
            if kind == "custom-model":
                p = app / "run-perception.sh"
                p.write_bytes(p.read_bytes().replace(b"yolo11s-int8-rk3588", b"unrecognized-custom"))
            elif kind == "ambiguous-model":
                p = app / "run-perception.sh"
                p.write_bytes(p.read_bytes() + b"\nMODEL_PATH=$APP_DIR/model/yolo11s-int8-rk3588.rknn\n")
            elif kind == "foreign-versioned":
                (app / "model" / roles["model"]).write_bytes(b"foreign model\n")
            elif kind == "active-process":
                (f / "proc/99").mkdir()
                (f / "proc/99/exe").symlink_to(app / "perception-dashboard.aarch64")
            elif kind == "mixed-pair":
                (navigation / "robot-navigation.aarch64").write_bytes(b"foreign navigation\n")
            before = snapshot(app, navigation)
            rejected = run(f)
            ck(rejected.returncode != 0, "Unsafe fixture accepted: " + kind)
            ck(snapshot(app, navigation) == before, "Rejection mutated files: " + kind)
            cases.append({"case": "reject-" + kind, "status": "PASS"})

        report = {"status": "PASS", "shell": shell, "checks": checks, "case_count": len(cases),
                  "real_arm_payloads_executed": False, "hardware_tested": False,
                  "original_package_manifest_checked": bool(args.package),
                  "package_manifest_sha256": supplied_manifest_sha256,
                  "test_script_sha256": sha(Path(__file__)),
                  "package_files": package_binding, "old_dashboard_sha256": old_dash,
                  "old_navigation_sha256": old_nav, "cases": cases}
    destination = args.report or source / "reports/v34-installer-independent.json"
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps({k: v for k, v in report.items() if k not in ("package_files", "cases")}, indent=2))


if __name__ == "__main__":
    main()
