#!/usr/bin/env python3
"""Build and run the ROS-independent postprocessing core."""

from pathlib import Path
import os
import subprocess
import tempfile


PACKAGE = Path(__file__).resolve().parents[1]
INCLUDE = PACKAGE / "include"
SOURCE = PACKAGE / "src" / "postprocess.cpp"
TEST = PACKAGE / "tests" / "test_postprocess.cpp"


def run(command, env=None):
    result = subprocess.run(command, check=False, capture_output=True,
                            text=True, env=env)
    if result.returncode:
        raise RuntimeError("command failed: " + " ".join(map(str, command)) +
                           "\n" + result.stdout + result.stderr)
    return result


def main():
    for sanitizer in (False, True):
        with tempfile.TemporaryDirectory(prefix="acg138-rk-postprocess-") as directory:
            binary = Path(directory) / "test_postprocess"
            flags = ["-std=c++17", "-Wall", "-Wextra", "-Wpedantic", "-Werror",
                     "-g", "-O1", "-fno-omit-frame-pointer"]
            if sanitizer:
                flags += ["-fsanitize=address,undefined"]
            command = ["c++", *flags, "-I", str(INCLUDE), str(SOURCE), str(TEST),
                       "-o", str(binary)]
            run(command)
            environment = dict(os.environ)
            environment["ASAN_OPTIONS"] = "detect_leaks=0:halt_on_error=1"
            environment["UBSAN_OPTIONS"] = "halt_on_error=1:print_stacktrace=1"
            result = run([str(binary)], env=environment)
            print(("sanitized " if sanitizer else "") + result.stdout, end="")


if __name__ == "__main__":
    main()
