#!/usr/bin/env python3
"""Compile and run the production local-avoidance core without board/network IO.

Every binary is freshly compiled into a private temporary directory. Reports
bind the exact source content before/after execution; no existing V35 artifact
or FPGA source is modified. Full gateway/Android/ARM tests are separate.
"""
import argparse
import concurrent.futures
import hashlib
import json
import os
from pathlib import Path
import subprocess
import tempfile
import time

NAV = Path(__file__).resolve().parents[1]
RK = NAV.parent
ROOT = NAV.parents[2]
FW = ROOT / "fpga/robot_control_v38/softcore/firmware"


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path,
                        default=NAV / "tests/reports/v38-direction-regression")
    options = parser.parse_args()
    output = options.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    build = Path(tempfile.mkdtemp(prefix="v36-direction-tests-"))
    cases = [
        ("direction-fallback-v36", "test_direction_fallback_v36.cc", False, False),
        ("direction-fallback-v36-asan-ubsan", "test_direction_fallback_v36.cc", False, True),
        ("navcore", "test_navcore.cc", False, False),
        ("straight-assist", "test_straight_assist.cc", True, False),
        ("reactive-avoidance", "test_reactive_avoidance.cc", True, False),
        ("reactive-velocity", "test_reactive_avoidance.cc", True, False),
        ("sonar-policy", "test_sonar_avoidance_policy.cc", False, False),
        ("three-sonar-policy", "test_three_sonar_policy.cc", False, False),
        ("reactive-safety-v33", "test_reactive_safety_v33.cc", False, False),
        ("local-rotation", "test_local_rotation_map.cc", False, False),
        ("encoder-window", "test_encoder_window.cc", False, False),
    ]
    sources = [NAV / "navcore.cc", NAV / "navcore.h", NAV / "navcore_reactive.inc",
               RK / "common/wrong_log.cc", RK / "common/wrong_log.h",
               FW / "astar.c", FW / "astar.h", Path(__file__).resolve()]
    sources += sorted({NAV / "tests" / src for _, src, _, _ in cases})
    sources += sorted((NAV / "tests/fixtures").rglob("*.json"))
    before = {str(p.relative_to(ROOT)): sha(p) for p in sources}
    subprocess.run(["gcc", "-O2", "-c", str(FW / "astar.c"), "-o", str(build / "astar.o")], check=True)

    def execute(case):
        name, source, astar, sanitize = case
        binary = build / name
        command = ["g++", "-std=c++17", "-Wall", "-Wextra", "-pthread", "-I" + str(FW)]
        command += ["-O1", "-g", "-fsanitize=address,undefined", "-fno-omit-frame-pointer"] if sanitize else ["-O2"]
        if name == "reactive-velocity":
            command += ["-DV28_TEST_VELOCITY"]
        command += [str(NAV / "navcore.cc"), str(NAV / "tests" / source), str(RK / "common/wrong_log.cc")]
        if astar:
            command += [str(build / "astar.o")]
        command += ["-o", str(binary)]
        started = time.monotonic()
        result = subprocess.run(command, text=True, capture_output=True, timeout=180)
        compile_log = output / (name + ".compile.log")
        compile_log.write_text(result.stdout + result.stderr)
        record = dict(name=name, compile_command=command, compile_returncode=result.returncode)
        if result.returncode == 0:
            environment = os.environ.copy()
            if sanitize:
                # This executor is traced; LeakSanitizer aborts after the test
                # because ptrace is unsupported. Address/UB checking remains
                # enabled and stops on its first real instrumentation failure.
                environment["ASAN_OPTIONS"] = "detect_leaks=0:halt_on_error=1"
                environment["UBSAN_OPTIONS"] = "halt_on_error=1:print_stacktrace=1"
                record["sanitizer_scope"] = "AddressSanitizer/UndefinedBehaviorSanitizer enabled; leak scan unsupported under traced executor and disabled explicitly"
            result = subprocess.run([str(binary)], cwd=NAV, env=environment,
                                    text=True, capture_output=True, timeout=80)
            log = output / (name + ".log")
            log.write_text(result.stdout + result.stderr)
            record.update(returncode=result.returncode, executable_sha256=sha(binary),
                          evidence=log.name, evidence_sha256=sha(log),
                          output_tail=(result.stdout + result.stderr).splitlines()[-1:])
        record["duration_s"] = time.monotonic() - started
        return record

    # Keep sanitizer compile isolated from parallel compiler memory pressure.
    normal = [case for case in cases if not case[3]]
    with concurrent.futures.ThreadPoolExecutor(max_workers=2) as workers:
        results = list(workers.map(execute, normal))
    results += [execute(case) for case in cases if case[3]]
    after = {str(p.relative_to(ROOT)): sha(p) for p in sources}
    passed = before == after and all(r["compile_returncode"] == 0 and r.get("returncode") == 0 for r in results)
    report = dict(result="PASS" if passed else "FAIL", version="v38-source-grace-direction-regression",
                  hardware_tested=False, network_accessed=False, temp_build=str(build),
                  source_sha256=before, source_unchanged=before == after, cases=results,
                  scope="Production Navigator source and existing host regressions; physical observations follow previous wheel command. Budget/preference boundary preconditions are explicitly seeded. No FPGA/NPU/phone hardware claim.")
    path = output / "verification.json"
    path.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n")
    print(json.dumps(dict(result=report["result"], cases=len(results), report=str(path)), ensure_ascii=False))
    raise SystemExit(0 if passed else 1)


if __name__ == "__main__":
    main()
