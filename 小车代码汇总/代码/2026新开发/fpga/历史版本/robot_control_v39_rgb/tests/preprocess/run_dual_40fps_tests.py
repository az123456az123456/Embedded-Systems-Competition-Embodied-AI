#!/usr/bin/env python3
"""Source-bound real DVP -> bounded CDC FIFO -> RGP1/GMII RTL regression.

Run from any directory. This exercises production RTL with Verilator; it does
not measure physical camera, PHY, RK3588 or NPU performance.
"""
from pathlib import Path
import argparse
import datetime
import hashlib
import json
import os
import re
import shutil
import subprocess
import time

PROJECT = Path(__file__).resolve().parents[2]
ROOT = PROJECT.parents[1]
SOURCES = [
    "preprocess/bayer_demosaic.v", "preprocess/rgb_bilinear_resize.v",
    "preprocess/resize_x_q11.hex", "preprocess/rgb_letterbox.v",
    "preprocess/rgb_capture_bridge.v", "preprocess/rgb_stream_fifo.v",
    "preprocess/rgb_packet_tx.v", "imu/imu_ethernet_tx.v",
    "tests/preprocess/dual_capture_packet_top.sv",
    "tests/preprocess/test_dual_capture_packet.cpp",
    "tests/preprocess/capture_golden.h",
    "tests/preprocess/run_dual_40fps_tests.py",
]
RESULT = re.compile(
    r"RESULT case=(\S+) source_fps=([\d.]+) complete_pairs=(\d+) "
    r"epoch2_pairs=(\d+) packets=(\d+) fifo_peak=\((\d+),(\d+)\) "
    r"source_drops=\((\d+),(\d+)\) pair_faults=(\d+) "
    r"max_pair_age_ms=([\d.]+) max_source_skew_ps=(\d+) ifg_min=(\d+) "
    r"control_events=(\d+) control_reserved_cycles=(\d+) tx_cycles=(\d+) checks=(\d+)"
)


def hashes():
    return {name: hashlib.sha256((PROJECT / name).read_bytes()).hexdigest()
            for name in SOURCES}


def command(args, log, env):
    started = time.monotonic()
    with log.open("w") as output:
        result = subprocess.run(args, cwd=PROJECT, env=env, stdout=output,
                                stderr=subprocess.STDOUT, timeout=300)
    assert result.returncode == 0, (args, result.returncode, str(log))
    return round(time.monotonic() - started, 3)


def parse_case(log):
    data = log.read_text()
    found = RESULT.findall(data)
    assert len(found) == 1 and "PASS dual DVP" in data, str(log)
    (name, fps, pairs, epoch2, packets, peak0, peak1, drop0, drop1,
     faults, age, skew, ifg, events, reserved, cycles, checks) = found[0]
    out = {
        "name": name, "status": "PASS", "source_fps": float(fps),
        "complete_pairs": int(pairs), "epoch2_complete_pairs": int(epoch2),
        "packets": int(packets), "fifo_peak_words": [int(peak0), int(peak1)],
        "source_drops": [int(drop0), int(drop1)], "pair_faults": int(faults),
        "max_complete_pair_age_ms": float(age),
        "max_source_skew_ps": int(skew), "minimum_interpacket_gap_cycles": int(ifg),
        "control_events": int(events), "control_reserved_cycles": int(reserved),
        "tx_cycles": int(cycles), "checks": int(checks),
    }
    out["actual_control_reserved_fraction"] = int(reserved) / int(cycles)
    assert max(out["fifo_peak_words"]) <= 2044
    assert out["minimum_interpacket_gap_cycles"] >= 12
    assert out["max_complete_pair_age_ms"] < 1880 * 1120 * 1000 / 84135000
    if name in ("sync", "skew-200-pclk"):
        assert out["complete_pairs"] == 3 and out["packets"] == 7680
        assert out["source_drops"] == [0, 0] and out["pair_faults"] == 0
    elif name == "overflow-30000-pclk-recover":
        assert out["complete_pairs"] >= 3 and sum(out["source_drops"]) > 0
        assert out["pair_faults"] > 0 and max(out["fifo_peak_words"]) == 2044
    elif name == "epoch-midstream":
        assert out["epoch2_complete_pairs"] >= 3
    else:
        raise AssertionError(name)
    return out


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--report", type=Path, default=(
        ROOT / "integration/robot_vision_v35/reports/dual-capture-packet-40fps.json"))
    args = parser.parse_args()
    build = PROJECT / "build/preprocess-dual40"
    build.mkdir(parents=True, exist_ok=True)
    before = hashes()
    env = dict(os.environ, CCACHE_DISABLE="1", CCACHE_TEMPDIR="/tmp")
    verilator = shutil.which("verilator")
    assert verilator, "Verilator is required"
    compile_args = [verilator, "--cc", "--exe", "--build", "-j", "2",
                    "-Wno-fatal", "--top-module", "dual_capture_packet_top",
                    "--Mdir", "build/preprocess-dual40", "-CFLAGS", "-O2 -std=c++17"]
    compile_args += [name for name in SOURCES if name.endswith((".v", ".sv"))]
    compile_args += [str(PROJECT / "tests/preprocess/test_dual_capture_packet.cpp")]
    report = {"status": "RUNNING", "result": "RUNNING",
              "hardware_tested": False, "source_sha256": before}
    try:
        report["verilator_version"] = subprocess.check_output(
            [verilator, "--version"], text=True).strip()
        report["compile_seconds"] = command(
            compile_args, PROJECT / "build/preprocess-dual40-compile.log", env)
        cases = []
        executable = build / "Vdual_capture_packet_top"
        for selected in ("sync", "skew", "overflow", "epoch"):
            log = PROJECT / f"build/preprocess-dual40-{selected}.log"
            seconds = command([str(executable), selected], log, env)
            case = parse_case(log)
            case.update(wall_seconds=seconds, log=str(log.relative_to(PROJECT)),
                        log_sha256=hashlib.sha256(log.read_bytes()).hexdigest())
            cases.append(case)
        assert hashes() == before, "Source changed during verification"
        report.update(
            status="PASS", result="PASS", cases=cases,
            total_checks=sum(case["checks"] for case in cases),
            generated_at=datetime.datetime.now(datetime.timezone.utc).isoformat(),
            input={"format": "RAW8 BGGR Bayer", "width_per_eye": 960,
                   "height": 1080, "nominal_fps": 40, "hts": 1880, "vts": 1120,
                   "pclk_hz": 84135000, "actual_source_fps": 84135000 / (1880 * 1120),
                   "hblank_clocks": 920, "source_generation": "time-indexed physical DVP waveform; no frame queue"},
            transport={"tx_clock_hz": 125000000,
                       "fifo_per_eye_words": 2048, "fifo_word_bits": 36,
                       "almost_full_threshold_words": 2044,
                       "packet_ram_banks": 2, "packet_ram_words_per_bank": 1024,
                       "rgb_payload_bytes_per_eye_frame": 640 * 640 * 3,
                       "payload_mbps_at_nominal_40fps": 640 * 640 * 3 * 2 * 40 * 8 / 1e6,
                       "wire_mbps_at_nominal_40fps_including_minimum_ifg": (1018 + 12) * 2560 * 40 * 8 / 1e6,
                       "control_reservation": "nonpreemptive 160 TX clocks requested every 5000 clocks, deferred to packet boundary"},
            checked=[
                "Real two rgb_capture_bridge instances: demosaic + Q11 resize + letterbox; no preprocessing bypass",
                "Real two 2048x36 dual-clock rgb_stream_fifo instances and real rgb_packet_tx",
                "Real coherent epoch mailbox pulses; same epoch retained; new epoch waits next source VSYNC",
                "All RGB payload bytes against independent CPU reflect101 demosaic + fixed-point Q11 golden",
                "Every Ethernet FCS, RGP1 application CRC32, eye/epoch/source sequence/row/part/dimensions",
                "No duplicate chunks; only both 1280-chunk eyes of the same epoch+sequence count as complete pair",
                "Complete-pair timestamp matches earlier source capture, with 1ms registered Gray sampling quantization",
                "Source phase is measured from accepted physical SOF and explicitly asserted",
                "Normal 3 pairs each, 200-PCLK skew, 30000-PCLK overflow abort followed by four recovered pairs, and midstream epoch reset",
                "Overflow old seq1 never completes; partial old epoch never becomes a new complete tensor",
                "Minimum 12-clock IFG, nonpreemptive control stalls, FIFO almost-full limit and bounded normal pair age",
            ],
            scope=[
                "Functional RTL simulation only. No physical FPGA/PHY/RK/NPU FPS measurement or inference accuracy claim.",
                "Read-only software golden tensors and bitset scoreboards do not buffer or supply RTL input/output traffic.",
                "Timestamp Gray synchronizer and mailbox are functionally exercised; analog metastability/CDC placement is not simulated.",
                "Large unsynchronized source skew is intentionally dropped; pass means bounded abort/recovery, not arbitrary phase tolerance.",
                "Clock timing and source registers must also pass official full-project P&R/timing audit.",
            ],
            compile_command=compile_args)
    except Exception as error:
        report.update(status="FAIL", result="FAIL", error=str(error))
        raise
    finally:
        args.report.parent.mkdir(parents=True, exist_ok=True)
        args.report.write_text(json.dumps(report, indent=2, ensure_ascii=False) + "\n")
    print(json.dumps({"status": report["status"], "checks": report["total_checks"],
                      "report": str(args.report)}, ensure_ascii=False))


if __name__ == "__main__":
    main()
