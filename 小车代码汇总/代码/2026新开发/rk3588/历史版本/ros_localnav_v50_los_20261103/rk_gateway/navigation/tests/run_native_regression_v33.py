#!/usr/bin/env python3
"""Replay final compiled navigation tests with immutable source/evidence hashes.

Compile first using the production Makefile. This runner never accesses board
interfaces; HTTP tests bind loopback fixtures only. It does not rebuild binaries.
"""
import hashlib
import json
import os
from pathlib import Path
import re
import subprocess
import time

NAV = Path(__file__).resolve().parents[1]
ROOT = NAV.parent
REPORTS = ROOT / "reports"
EVIDENCE = NAV / "reports" / "native-v33-final"
NATIVE = [
    "protocol", "navcore", "sonar-censored", "vehicle-profile",
    "runtime-profile-state", "input-pipeline-replay", "imu-alignment",
    "localization-diagnostics", "sensor-bridge", "sonar-diagnostics",
    "map-snapshot", "map-goal", "gateway", "gateway-straight",
    "gateway-source-clock", "gateway-concurrency", "gateway-core-handoff",
    "gateway-preflight", "wronglog-navigation", "gateway-observation-barrier",
    "firmware-client", "actual-astar-drive", "bootstrap-navigation",
    "straight-assist", "straight-runtime", "navigation-timeline",
    "encoder-window", "controller-navigation-lifecycle", "reactive-avoidance",
    "reactive-velocity", "sonar-policy", "local-rotation", "local-guard-status",
    "three-sonar-policy", "reactive-safety-v33",
]


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def source_snapshot():
    paths = [p for p in NAV.rglob("*") if p.is_file()
             and not {"build", "reports"}.intersection(p.relative_to(NAV).parts)
             and (p.suffix in {".cc", ".h", ".inc", ".py", ".sh", ".c", ".json"}
                  or p.name == "Makefile")]
    paths += [ROOT / "common" / name for name in ("wrong_log.cc", "wrong_log.h")]
    paths += [ROOT / "dashboard" / "phone" / name for name in (
        "controller.cc", "controller.h", "motor_link.cc", "motor_link.h",
        "follow_geometry.cc", "follow_geometry.h")]
    return {os.path.relpath(p, ROOT): sha(p) for p in sorted(paths)}


def main():
    REPORTS.mkdir(exist_ok=True)
    EVIDENCE.mkdir(parents=True, exist_ok=True)
    commands = [(name, [str(NAV / "build" / ("test-" + name))]) for name in NATIVE]
    commands += [
        ("cli", ["python3", "tests/test_runtime.py"]),
        ("launcher", ["python3", "tests/test_unified_launcher.py"]),
        ("launcher-busybox", ["python3", "tests/test_unified_launcher.py", "--busybox"]),
        ("profile-cli", ["python3", "tests/test_vehicle_profile.py"]),
    ]
    before = source_snapshot()
    executable_before = {os.path.relpath(Path(cmd[0]), ROOT): sha(Path(cmd[0]))
                         for _, cmd in commands if Path(cmd[0]).is_absolute()}
    report = dict(result="RUNNING", version="v33-scan-lease",
                  hardware_tested=False, network_scope="loopback HTTP fixtures only",
                  compile_scope="production Makefile final compiled tests; no binary rebuild in this run",
                  source_sha256_before=before, executable_sha256=executable_before,
                  cases=[], source_unchanged=False)
    report_path = REPORTS / "navigation-native-regression-v33.json"
    try:
        for name, command in commands:
            log = EVIDENCE / (name + ".log")
            started = time.monotonic()
            with log.open("w") as output:
                output.write("command=" + json.dumps(command) + "\n")
                output.flush()
                completed = subprocess.run(command, cwd=NAV, stdout=output,
                                           stderr=subprocess.STDOUT, timeout=120)
            report["cases"].append(dict(name=name, command=command,
                exit_code=completed.returncode, duration_s=time.monotonic() - started,
                evidence=os.path.relpath(log, ROOT), evidence_sha256=sha(log)))
            # Report only counts actually printed by the fresh executable.
            # Named test cases are distinct from assertion/check totals.
            output = log.read_text()
            checks = re.findall(r"\bchecks=(\d+)\b|\b(\d+) checks\b|\bPASS\s+(\d+)\b[^\n]*\bchecks\b", output)
            if checks:
                report["cases"][-1]["reported_checks"] = int(next(x for x in checks[-1] if x))
            named = re.search(r"\bcases=(\d+)\b", output)
            if named:
                report["cases"][-1]["reported_named_cases"] = int(named[1])
            if completed.returncode:
                raise RuntimeError(name + " failed; see " + str(log))
        after = source_snapshot()
        report["source_sha256_after"] = after
        report["source_unchanged"] = before == after
        if before != after:
            raise RuntimeError("source changed during final native replay")
        report["executable_unchanged"] = all(
            sha(ROOT / name) == digest for name, digest in executable_before.items())
        if not report["executable_unchanged"]:
            raise RuntimeError("test executable changed during final native replay")
        report["result"] = "PASS"
        report["commands_passed"] = len(commands)
        report["reported_checks_total"] = sum(case.get("reported_checks", 0) for case in report["cases"])
        report["commands_without_assertion_count"] = [case["name"] for case in report["cases"] if "reported_checks" not in case]
        report["count_semantics"] = "Fresh executed command count and explicit per-program checks only; programs without printed assertion totals remain uncounted. Named cases are not added to assertion totals."
    except Exception as error:
        report["result"] = "FAIL"
        report["error"] = str(error)
        report["source_sha256_after"] = source_snapshot()
        raise
    finally:
        report_path.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n")
    print("PASS", len(commands), "native/CLI/launcher cases; source and executable hashes unchanged")
    print(report_path)


if __name__ == "__main__":
    main()
