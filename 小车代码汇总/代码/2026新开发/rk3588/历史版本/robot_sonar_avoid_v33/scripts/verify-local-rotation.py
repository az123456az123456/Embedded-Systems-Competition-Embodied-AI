#!/usr/bin/env python3
"""Portable native replay of the sanitized, complete recorded occupancy map.

No sockets, sensors, FPGA, motors, RK executable or original private log needed.
The baseline files are exact frozen v29 sources and are compiled only by tests.
"""
import hashlib
import json
import pathlib
import re
import shutil
import subprocess
import time

ROOT = pathlib.Path(__file__).resolve().parents[1]
NAV = ROOT / "navigation"
REPORTS = ROOT / "reports"
BUILD = NAV / "build" / "local-rotation-replay"
BASE = NAV / "tests" / "fixtures" / "baseline-v29"
FIXTURE = NAV / "tests" / "fixtures" / "local-rotation-map-20261027.json"


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def run(name, command):
    path = REPORTS / ("v29_2-" + name + ".log")
    with path.open("w") as log:
        # Relative source/build paths keep the verification portable.
        log.write("command=" + json.dumps(command) + "\n")
        log.flush()
        proc = subprocess.run(command, cwd=NAV, stdout=log, stderr=subprocess.STDOUT)
    if proc.returncode:
        raise RuntimeError(str(path) + " failed, exit=" + str(proc.returncode))
    return path


def main():
    REPORTS.mkdir(exist_ok=True)
    BUILD.mkdir(parents=True, exist_ok=True)
    fixture = json.loads(FIXTURE.read_text())
    assert len(fixture["cells"]) == 4096
    assert set(fixture["cells"]) <= {"?", ".", "#"}
    manifest_path = BASE / "origin.json"
    manifest = json.loads(manifest_path.read_text())
    for name, digest in manifest["files_sha256"].items():
        assert sha(BASE / name) == digest, ("modified frozen baseline", name)

    compiler = shutil.which("g++")
    assert compiler, "g++ is needed for native source replay"
    flags = ["g++", "-O2", "-std=c++17", "-Wall", "-Wextra", "-pthread"]
    relbuild = "build/local-rotation-replay/"
    evidence = []
    basic = "tests/test_local_rotation_map.cc"
    # A baseline failure is expressed as an expected rejected geometry result,
    # not a deliberately failed compiler/process that could hide other errors.
    evidence.append(run("map-baseline-build", flags + [
        "-DV29_2_EXPECT_LEGACY", '-DNAVCORE_HEADER="fixtures/baseline-v29/navcore.h"',
        "tests/fixtures/baseline-v29/navcore.cc", basic, "-o", relbuild + "baseline"]))
    evidence.append(run("map-baseline", [relbuild + "baseline", "tests/fixtures/local-rotation-map-20261027.json"]))
    evidence.append(run("map-current-build", flags + ["navcore.cc", basic, "-o", relbuild + "current"]))
    evidence.append(run("map-current", [relbuild + "current", "tests/fixtures/local-rotation-map-20261027.json"]))
    evidence.append(run("status-build", flags + [
        "navcore.cc", "imu_alignment.cc", "sensor_bridge.cc", "protocol.cc", "gateway.cc",
        "map_snapshot.cc", "vendor/http_server.cc", "vendor/json_flat.cc",
        "tests/test_local_guard_status.cc", "../common/wrong_log.cc", "-o", relbuild + "status"]))
    evidence.append(run("status", [relbuild + "status"]))
    checks = {}
    for name in ("map-baseline", "map-current", "status"):
        text = (REPORTS / ("v29_2-" + name + ".log")).read_text()
        matches = re.findall(r"\b(\d+) checks\b", text)
        assert len(matches) == 1 and "PASS " in text, (name, text)
        checks[name] = int(matches[0])

    nav_sources = ["navcore.cc", "navcore.h", "navcore_reactive.inc", "main.cc", "vehicle_geometry.h",
                   "imu_alignment.cc", "imu_alignment.h", "sensor_bridge.cc", "sensor_bridge.h",
                   "protocol.cc", "protocol.h", "gateway.cc", "gateway.h", "map_snapshot.cc", "map_snapshot.h",
                   "sonar_diagnostics.h", "vendor/http_server.cc", "vendor/http_server.h",
                   "vendor/json_flat.cc", "vendor/json_flat.h", "vendor/picojson.h",
                   "tests/test_local_rotation_map.cc", "tests/test_local_guard_status.cc",
                   "tests/fixtures/local-rotation-map-20261027.json"]
    sources = [NAV / p for p in nav_sources]
    sources += [ROOT / "common/wrong_log.cc", ROOT / "common/wrong_log.h", pathlib.Path(__file__).resolve()]
    sources += [BASE / name for name in manifest["files_sha256"]] + [manifest_path]
    missing = [str(p) for p in sources if not p.is_file()]
    assert not missing, missing
    # Physical defaults used in this exact fixture: conservative body bound.
    radius = (max(.145417, .16725) ** 2 + max(.137, .115) ** 2) ** .5 + .08
    x, y = fixture["pose_m_rad"][:2]
    minimum = ((-.3 - x) ** 2 + (-.2 - y) ** 2) ** .5
    report = {
        "result": "PASS", "version": "v29.2-local-rotation-map-guard",
        "created_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        "hardware_tested": False, "arm_programs_executed": False,
        "uses_private_log_at_runtime": False, "network_accessed": False,
        "tests": checks, "checks_total": sum(checks.values()),
        "recorded_fixture": {
            "source_event_mono_ms": fixture["source_event_mono_ms"],
            "map_revision": fixture["source_map_revision"], "cells": len(fixture["cells"]),
            "occupied": fixture["cells"].count("#"), "free": fixture["cells"].count("."),
            "unknown": fixture["cells"].count("?"), "blocking_cell": [29, 29],
            "minimum_cell_center_distance_m": minimum, "complete_rotation_radius_m": radius,
            "baseline_result": "AVOID_WAIT with zero angular target",
            "current_result": "stop-first edge then negative angular target towards farther right source",
            "map_values_observation_times_revision_unchanged": True,
        },
        "coverage": [
            "exact original 4096-cell classified map, no cell rewriting",
            "ten microturn angles in both senses and sixteen independent rounding bounds",
            "production AVOID_SELECT to AVOID_TURN_CLEAR and following clockwise target",
            "inner-circle occupied cell and touching circle boundary still rejected",
            "translated and curved sweeps receive no pure-rotation exemption",
            "near sensor/range direction floor, map edge and unknown actual body remain blocked",
            "missing or expired sources keep zero SENSOR_WAIT and prolonged absence terminates",
            "real encoder jump remains latched with zero target",
            "production status JSON local_guard and motion_gate use actual aligned source provenance",
        ],
        "limits": [
            "native production-source replay, not physical vehicle test",
            "original recorded map is already TTL-classified; import uses the replay time without inventing historical per-cell times",
            "source age and encoder protections intentionally remain enabled",
            "this geometry change does not classify encoder slip or recover missing raw UART frames",
        ],
        "source_sha256": {str(p.relative_to(ROOT)): sha(p) for p in sources},
        "evidence_sha256": {str(p.relative_to(ROOT)): sha(p) for p in evidence},
    }
    output = REPORTS / "v29_2-map-guard.json"
    output.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n")
    print("PASS v29.2 map guard:", checks, "total", report["checks_total"])


if __name__ == "__main__":
    main()
