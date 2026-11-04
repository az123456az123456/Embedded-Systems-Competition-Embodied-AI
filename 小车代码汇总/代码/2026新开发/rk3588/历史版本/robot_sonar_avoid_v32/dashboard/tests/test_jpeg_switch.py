"""Changing codec build flags must not silently reuse the previous codec."""
import json
from pathlib import Path
import socket
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]


def main():
    records = []
    with tempfile.TemporaryDirectory(prefix="jpeg-switch-v121-") as temporary:
        for codec in ("stb", "turbo", "stb", "turbo"):
            build = subprocess.run(["make", "-j4", "all", "JPEG="+codec], cwd=ROOT,
                                   capture_output=True, text=True)
            assert build.returncode == 0, build.stdout+build.stderr
            with socket.socket() as probe:
                probe.bind(("127.0.0.1", 0));port = probe.getsockname()[1]
            run = subprocess.run([str(ROOT / "build/perception-dashboard"), "--demo", "--headless",
                                  "--seconds", "3", "--phone-bind", "127.0.0.1", "--phone-port", str(port),
                                  "--pair-token-file", str(Path(temporary) / "pair.txt")],
                                 capture_output=True, text=True, timeout=8)
            assert run.returncode == 0, run.stderr
            stats = [line for line in run.stderr.splitlines() if "PHONE_PERF" in line]
            expected = "stb-scalar" if codec == "stb" else "turbojpeg-simd"
            assert stats and "codec="+expected in stats[-1], stats
            assert "encodes=0" in stats[-1] and "frame_copies=0" in stats[-1], stats
            records.append({"requested": codec, "observed": stats[-1]})
    (ROOT / "reports/jpeg-switch-verification.json").write_text(json.dumps(records, indent=2)+"\n")
    print("PASS codec build cache STB -> Turbo -> STB -> Turbo, zero-client image copies and encodes")


if __name__ == "__main__":
    main()
