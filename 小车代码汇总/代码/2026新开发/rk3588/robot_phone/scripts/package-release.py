#!/usr/bin/env python3
"""Package already verified binaries; never package tokens or signing keys."""
from pathlib import Path
import argparse
import hashlib
import json
import shutil
import tarfile
import tempfile
import zipfile


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    module = Path(__file__).resolve().parents[1]
    development = module.parent.parent
    fpga = development / "fpga/robot_fmc_remote"
    workspace = next(p for p in module.parents if (p / "YOLO/model.rknn").is_file())
    parser = argparse.ArgumentParser()
    parser.add_argument("destination", type=Path)
    args = parser.parse_args()
    output = args.destination.resolve()
    output.mkdir(parents=True, exist_ok=True)
    verification = json.loads((module / "reports/build_verification.json").read_text())
    apk = module / "android/dist/ACG138-Robot-Control-debug.apk"
    binary = module / "build/robot-phone-server.aarch64"
    full_fs = fpga / "impl/pnr/robot_fmc_remote.fs"
    assert digest(apk) == verification["android"]["apk_sha256"]
    assert digest(binary) == verification["arm64"]["sha256"]
    assert digest(full_fs) == verification["fpga"]["full_sha256"]
    for name, sha in verification["code_sha256"].items():
        assert digest(module / name) == sha, f"Source changed after verification: {name}"

    artifacts = []
    for source, name in [
        (apk, apk.name),
        (full_fs, "robot_fmc_remote.fs"),
        (fpga / "impl/pnr/fmc_remote_motor_only.fs", "fmc_remote_motor_only.fs"),
        (fpga / "release/ACG138_FPGA_四电机远控_双目_IMU_20261015.zip",
         "ACG138_FPGA_四电机远控_双目_IMU_20261015.zip"),
    ]:
        target = output / name
        temporary_copy = target.with_name(target.name + ".tmp")
        shutil.copyfile(source, temporary_copy)
        temporary_copy.replace(target)
        artifacts.append(target)

    with tempfile.TemporaryDirectory(prefix="robot-phone-release-") as temporary:
        runtime = Path(temporary) / "robot-phone"
        runtime.mkdir()
        for name in ["README.md", "run-robot-phone.sh"]:
            shutil.copy2(module / name, runtime / name)
        for directory in ["docs", "model", "reports"]:
            shutil.copytree(module / directory, runtime / directory)
        (runtime / "scripts").mkdir()
        shutil.copy2(module / "scripts/diagnose-network.sh", runtime / "scripts/diagnose-network.sh")
        shutil.copy2(binary, runtime / binary.name)
        (runtime / binary.name).chmod(0o755)
        shutil.copy2(workspace / "YOLO/model.rknn", runtime / "model/model.rknn")
        (runtime / "SHA256SUMS").write_text("".join(
            f"{digest(p)}  {p.relative_to(runtime)}\n"
            for p in sorted(runtime.rglob("*")) if p.is_file()))
        archive = output / "robot-phone-rk3588-20261015.tar.gz"
        with tarfile.open(archive, "w:gz") as tar:
            tar.add(runtime, arcname="robot-phone")
        with tarfile.open(archive) as tar:
            names = tar.getnames()
            assert "robot-phone/robot-phone-server.aarch64" in names
            assert not any("pair-token" in n or "keystore" in n for n in names)
        artifacts.append(archive)

    source_archive = output / "ACG138手机遥控_源码_20261015.zip"
    excluded = {"build", ".local", "__pycache__", "impl", "release", ".git"}
    prefix = "2026新开发"
    with zipfile.ZipFile(source_archive, "w", zipfile.ZIP_DEFLATED) as archive:
        for base, relative in [(module, "rk3588/robot_phone"), (fpga, "fpga/robot_fmc_remote")]:
            for path in sorted(base.rglob("*")):
                rel = path.relative_to(base)
                if not path.is_file() or excluded.intersection(rel.parts):
                    continue
                if path.name == "pair-token.txt" or path.suffix in {".ses", ".rknn", ".onnx", ".keystore", ".jks", ".o", ".pyc"}:
                    continue
                archive.write(path, str(Path(prefix) / relative / rel))
        assert archive.testzip() is None
        assert not any("keystore" in n or "pair-token.txt" in n for n in archive.namelist())
    artifacts.append(source_archive)
    manifest = {"date": "2026-10-15", "hardware_tested": False,
                "files": {p.name: {"bytes": p.stat().st_size, "sha256": digest(p)} for p in artifacts}}
    (output / "手机遥控交付清单.json").write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + "\n")
    (output / "手机遥控_SHA256SUMS").write_text("".join(f"{digest(p)}  {p.name}\n" for p in artifacts))
    print(json.dumps(manifest, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
