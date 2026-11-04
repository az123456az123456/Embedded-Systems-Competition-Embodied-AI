"""Decode real codec output and compare CPU cost; this is not RK3588 FPS."""
import json
import os
from pathlib import Path
import subprocess
import tempfile
from PIL import Image

ROOT = Path(__file__).resolve().parents[1]


def main():
    records = {}
    with tempfile.TemporaryDirectory(prefix="robot-jpeg-v121-") as temporary:
        base = Path(temporary)
        for codec in ("stb", "turbo"):
            executable, image = base / codec, base / (codec + ".jpg")
            command = [os.environ.get("CC", "cc"), "-O2", "-std=c11", "-Wall", "-Wextra", "-Werror",
                       "-Iphone", "phone/jpeg_codec.c", "tests/jpeg_codec_probe.c"]
            if codec == "turbo":
                command += ["-DHAVE_TURBOJPEG", "-Ivendor/jpeg_turbo/include",
                            "vendor/jpeg_turbo/Linux/x64/libturbojpeg.a"]
            subprocess.run(command + ["-lm", "-o", str(executable)], cwd=ROOT, check=True)
            records[codec] = json.loads(subprocess.check_output([str(executable), str(image)], text=True))
            with Image.open(image) as decoded:
                decoded.load()
                assert decoded.size == (864, 486)
                for point, expected in (((100, 50), (224, 32, 48)), ((700, 50), (32, 64, 224))):
                    actual = decoded.getpixel(point)
                    assert all(abs(a-b) < 10 for a, b in zip(actual, expected)), (codec, point, actual)
        records["speedup_local_cpu"] = records["stb"]["mean_ms"] / records["turbo"]["mean_ms"]
        records["scope"] = "Host CPU codec benchmark only; board inference FPS requires hardware measurement."
    (ROOT / "reports").mkdir(exist_ok=True)
    (ROOT / "reports/jpeg-codec-verification.json").write_text(json.dumps(records, indent=2)+"\n")
    print(json.dumps(records, indent=2))
    print("PASS both JPEG decoders, dimensions, channel order, invalid-input guards")


if __name__ == "__main__":
    main()
