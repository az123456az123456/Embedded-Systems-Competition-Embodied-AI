#!/usr/bin/env python3
"""Package the tested app/APK, keeping board images and private state untouched."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import tarfile
import tempfile
import zipfile

ROOT = Path(__file__).resolve().parents[1]
RK = ROOT.parent
FPGA = RK.parent / "fpga/robot_fmc_remote_30fps_v12"
MODELS = RK / "perception-v11-default-30fps/models"
DATE = "20261020"
MODEL_SHA = "96489da6be36fd7700ad0df127c6e91c22b251b17f0f68877b2dc778f5b522b7"
FS_SHA = "b1a9072b8e56ce8d6cb6792855ab1bd1e401fc7bd9f30d114efacd34d6f45382"


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def copy(source, target):
    target.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(source, target)


def add_tree(archive, source, prefix):
    excluded = {"build", ".git", ".local", "__pycache__", ".pytest_cache", "release"}
    for path in sorted(source.rglob("*")):
        relative = path.relative_to(source)
        if path.is_symlink() or not path.is_file() or any(part in excluded for part in relative.parts):
            continue
        if path.name in {"pair-token.txt", "local.properties", "package-v121-output.json"} or path.suffix in {".keystore", ".jks", ".pyc"}:
            continue
        # Public APK is included explicitly at the root, avoiding duplicate builds.
        if "dist" in relative.parts and path.suffix in {".apk", ".idsig"}:
            continue
        archive.write(path, str(Path(prefix) / relative))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()
    output = args.output_dir.resolve();output.mkdir(parents=True, exist_ok=True)
    arm = ROOT / "build/perception-dashboard.aarch64"
    apk = ROOT / "android/dist/ACG138-Robot-Control-debug.apk"
    model = MODELS / "yolo11s-int8-rk3588.rknn"
    bitstream = FPGA / "impl/pnr/robot_fmc_remote_30fps_v12.fs"
    assert sha(model) == MODEL_SHA, "Model changed"
    assert sha(bitstream) == FS_SHA, "FPGA bitstream changed"
    assert arm.read_bytes()[:4] == b"\x7fELF" and int.from_bytes(arm.read_bytes()[18:20], "little") == 183
    assert b"turbojpeg-simd" in arm.read_bytes(), "ARM executable missing SIMD JPEG"
    assert b"v12.1-control-imu-turbo-yolo11-30fps" in arm.read_bytes(), "Wrong ARM version"
    assert (ROOT / "android/dist/android-v121-verification.json").is_file()
    apk_name = "ACG138-Robot-Control-v12.1.apk"
    tar_name = "robot-perception-v12.1-rk3588-20261020.tar.gz"
    zip_name = "ACG138_手机预览与性能优化_v12.1_20261020.zip"
    readme_name = "v12.1_安装与更新说明_20261020.txt"
    copy(apk, output / apk_name)
    copy(ROOT / "README.md", output / readme_name)
    preview = ROOT / "android/dist/ui-preview/perception-connected-host.png"
    copy(preview, output / "v12.1-phone-preview.png")
    with tempfile.TemporaryDirectory(prefix="robot-v121-package-") as temporary:
        release = Path(temporary) / "robot-perception-v12.1-update"
        app = release / "payload/app"
        copy(arm, app / "perception-dashboard.aarch64")
        for name in ("run-dashboard.sh", "run-perception.sh"):
            copy(ROOT / name, app / name)
        for path in (ROOT / "scripts").glob("*.sh"):
            if path.name != "build-arm64.sh":copy(path, app / "scripts" / path.name)
        for path in MODELS.iterdir():
            if path.is_file():copy(path, app / "model" / path.name)
        copy(ROOT / "README.md", app / "README.md")
        copy(ROOT / "assets/FONT-LICENSE", app / "licenses/FONT-LICENSE")
        copy(ROOT / "vendor/jpeg_turbo/LICENSE.md", app / "licenses/libjpeg-turbo-LICENSE.md")
        copy(ROOT / "vendor/jpeg_turbo/PROVENANCE.json", app / "licenses/libjpeg-turbo-PROVENANCE.json")
        for name in ("install-update.sh", "rollback-update.sh"):
            copy(ROOT / name, release / name)
        # Optional receiver stays byte-identical; documented update skips it.
        old_receiver = RK / "robot-perception-v12-rk3588-20261020/robot-perception-v12-update/payload/isp-probe"
        copy(old_receiver, release / "payload/isp-probe")
        files = [path for path in sorted(release.rglob("*")) if path.is_file()]
        (release / "SHA256SUMS").write_text("".join(sha(path)+"  "+path.relative_to(release).as_posix()+"\n" for path in files))
        with tarfile.open(output / tar_name, "w:gz") as archive:
            archive.add(release, arcname=release.name)
    verification = {
        "version": "12.1", "date": DATE,
        "model_sha256": MODEL_SHA, "model_matches_v11": True,
        "fpga_unchanged_sha256": FS_SHA, "no_new_fpga_resources": True,
        "board_findings_from_user": {"imu": "wiring corrected, works", "motor": "motor power restored, no control algorithm change requested"},
        "changes": ["phone preview center crop and matching selection coordinates", "larger inset joystick, original motor mapping", "SIMD JPEG", "no image copies or encodes without video clients", "original launcher compatibility", "read-only diagnostics"],
        "limits": ["No physical Android/RK3588 FPS acceptance test this round", "No promise of recovering 24 inference FPS", "Screen previews contain synthetic data", "No FPGA bitstream update needed"],
        "files": {name: {"bytes": (output/name).stat().st_size, "sha256": sha(output/name)} for name in (apk_name, tar_name, readme_name, "v12.1-phone-preview.png")},
    }
    (output / "v12.1_release_verification_20261020.json").write_text(json.dumps(verification, ensure_ascii=False, indent=2)+"\n")
    with zipfile.ZipFile(output / zip_name, "w", compression=zipfile.ZIP_DEFLATED, compresslevel=6) as archive:
        for name in (*verification["files"], "v12.1_release_verification_20261020.json"):
            archive.write(output / name, name)
        add_tree(archive, ROOT, "source/rk3588/robot_perception_v12")
        add_tree(archive, FPGA, "source/fpga/robot_fmc_remote_30fps_v12")
        for name in ("motor_crosswire", "scheduler_stress"):
            assert (FPGA / "reports" / name / "verification.json").is_file()
        assert not any("/.local/" in name or name.endswith((".keystore", "/pair-token.txt")) for name in archive.namelist())
    names = [apk_name, tar_name, zip_name, readme_name, "v12.1-phone-preview.png", "v12.1_release_verification_20261020.json"]
    (output / "v12.1_SHA256SUMS_20261020.txt").write_text("".join(sha(output/name)+"  "+name+"\n" for name in names))
    print(json.dumps({name: {"bytes": (output/name).stat().st_size, "sha256": sha(output/name)} for name in names}, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
