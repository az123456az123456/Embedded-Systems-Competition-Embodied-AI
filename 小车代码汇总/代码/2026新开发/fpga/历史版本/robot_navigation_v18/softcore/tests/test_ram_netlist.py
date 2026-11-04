#!/usr/bin/env python3
"""Verify every initialized firmware RAM bit in the final Gowin netlist.

This deliberately accepts only the reviewed mapping: 32 one-bit SDPB planes,
each with the complete word-address bus, grouped into four byte write enables.
A changed mapping must be reviewed rather than silently accepted by this test.
"""
import argparse
import hashlib
import json
from pathlib import Path
import re
import struct


ROOT = Path(__file__).resolve().parents[2]


def digest(data):
    return hashlib.sha256(data).hexdigest()


def check(condition, message):
    if not condition:
        raise AssertionError(message)


def parameter_value(text):
    text = text.strip()
    match = re.fullmatch(r"(?:\d+)'([bhd])([0-9a-fA-F_]+)", text)
    if match:
        return int(match[2].replace("_", ""), {"b": 2, "h": 16, "d": 10}[match[1]])
    return int(text, 10)


def compact(text):
    return re.sub(r"\s+", "", text)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("netlist", type=Path)
    parser.add_argument("--firmware", type=Path,
                        default=ROOT / "softcore/firmware/firmware.hex")
    parser.add_argument("--primitive-model", type=Path, required=True,
                        help="Gowin gw5a/prim_sim.v, to verify omitted INIT defaults")
    parser.add_argument("--pnr-report", type=Path)
    parser.add_argument("--baseline-pnr-report", type=Path,
                        default=ROOT.parent / "robot_fmc_remote_30fps_v12/reports/robot_fmc_remote_30fps_v12.rpt.txt")
    parser.add_argument("--output", type=Path,
                        default=ROOT / "reports/ram_init_verification.json")
    args = parser.parse_args()
    if not args.baseline_pnr_report.exists():
        args.baseline_pnr_report = ROOT / "tools/baseline_v12/reports/robot_fmc_remote_30fps_v12.rpt.txt"

    raw = args.netlist.read_bytes()
    netlist = raw.decode("utf-8")
    source = re.search(r"\bmodule\s+robot_nav_soc\b.*?\bendmodule\b", netlist, re.S)
    check(source is not None, "robot_nav_soc module is missing")
    source = source[0]
    firmware_raw = args.firmware.read_bytes()
    expected = [int(value, 16) for value in firmware_raw.decode().split()]
    check(len(expected) == 16384, "firmware must initialize all 16384 RAM words")
    check(all(0 <= word <= 0xffffffff for word in expected), "invalid firmware word")

    primitive_raw = args.primitive_model.read_bytes()
    primitive = re.search(r"\bmodule\s+SDPB\b.*?\bendmodule\b",
                          primitive_raw.decode("utf-8"), re.S)
    check(primitive is not None, "SDPB model is missing")
    default_chunks = dict(re.findall(
        r"parameter\s+INIT_RAM_([0-9A-Fa-f]{2})\s*=\s*([^;]+);", primitive[0]))
    check(len(default_chunks) == 64, "expected 64 INIT defaults in vendor SDPB model")
    check(all(parameter_value(value) == 0 for value in default_chunks.values()),
          "omitted SDPB INIT chunks are not all zero")

    instances = re.findall(r"^\s*(SP|SDPB)\s+(ram_\w+)\s*\((.*?)\);", source, re.M | re.S)
    check(len(instances) == 32, f"expected 32 CPU BSRAM blocks, found {len(instances)}")
    reconstructed = [0] * 16384
    bits = set()
    planes = []
    byte_enables = {}
    read_enables = set()
    for kind, name, port_text in instances:
        check(kind == "SDPB", f"unexpected CPU RAM primitive {kind}")
        ports = {key: compact(value) for key, value in re.findall(
            r"\.(\w+)\s*\((.*?)\)\s*(?:,|$)", port_text, re.S)}
        output = re.search(r"ram_q\[(\d+)\]", ports.get("DO", ""))
        check(output is not None, f"missing output bit mapping for {name}")
        bit = int(output[1])
        check(bit not in bits and 0 <= bit < 32, f"duplicate/invalid bit {bit}")
        bits.add(bit)
        check(ports.get("ADA") == "mem_addr_Z[15:2]" and
              ports.get("ADB") == "mem_addr_Z[15:2]", f"wrong address mapping: {name}")
        check(ports.get("CLKA") == "clk50m_d" and
              ports.get("CLKB") == "clk50m_d", f"wrong RAM clocks: {name}")
        check(ports.get("DI", "").endswith(f",mem_wdata_Z[{bit}]}}"),
              f"wrong RAM write-data bit: {name}")
        check(ports.get("RESET") == "GND" and ports.get("OCE") == "GND",
              f"unexpected RAM reset/output mode: {name}")
        check(ports.get("BLKSELA") == "{GND,GND,GND}" and
              ports.get("BLKSELB") == "{GND,GND,GND}", f"wrong RAM block selection: {name}")
        lane = bit // 8
        lane_enable = byte_enables.setdefault(lane, ports.get("CEA"))
        check(ports.get("CEA") == lane_enable, f"inconsistent byte write enable: {name}")
        read_enables.add(ports.get("CEB"))

        parameters = dict(re.findall(
            rf"defparam\s+{re.escape(name)}\.(\w+)\s*=\s*([^;]+);", source))
        for key in ("BIT_WIDTH_0", "BIT_WIDTH_1"):
            check(parameter_value(parameters.get(key, "0")) == 1,
                  f"unexpected {key}: {name}")
        check(parameter_value(parameters.get("READ_MODE", "0")) == 0,
              f"unexpected read pipeline: {name}")
        chunks = {int(key[9:], 16): parameter_value(value)
                  for key, value in parameters.items() if key.startswith("INIT_RAM_")}
        check(all(0 <= chunk < 64 for chunk in chunks), f"invalid INIT index: {name}")
        packed = sum(value << (chunk * 256) for chunk, value in chunks.items())
        for address in range(16384):
            reconstructed[address] |= ((packed >> address) & 1) << bit
        planes.append({"bit": bit, "instance": name,
                       "primitive": kind, "explicit_init_chunks": len(chunks),
                       "write_enable": ports.get("CEA"), "read_enable": ports.get("CEB")})
    check(bits == set(range(32)), "RAM does not cover all 32 data bits")
    check(len(set(byte_enables.values())) == 4, "byte lanes must have four write-enable nets")
    check(len(read_enables) == 1, "all RAM planes must share a read-enable net")
    mismatches = [i for i, (got, want) in enumerate(zip(reconstructed, expected)) if got != want]
    check(not mismatches, f"firmware RAM mismatch at word addresses {mismatches[:8]}")

    expected_image = struct.pack("<16384I", *expected)
    reconstructed_image = struct.pack("<16384I", *reconstructed)
    report = {
        "verified": True,
        "scope": "Synthesized CPU RAM configuration and address/data wiring; no board test",
        "mapping": "32 SDPB primitives, 16384 words x 1 bit per plane",
        "ram_bytes": 65536,
        "words_compared": 16384,
        "bits_compared": 524288,
        "mismatched_words": len(mismatches),
        "cpu_bsram_blocks": 32,
        "vendor_omitted_init_chunks_default_to_zero": True,
        "no_separate_external_flash_firmware_load": True,
        "firmware_sha256": digest(firmware_raw),
        "initialized_image_sha256": digest(expected_image),
        "netlist_reconstructed_image_sha256": digest(reconstructed_image),
        "netlist_sha256": digest(raw),
        "primitive_model_sha256": digest(primitive_raw),
        "planes": sorted(planes, key=lambda item: item["bit"]),
    }
    if args.pnr_report:
        pnr_raw = args.pnr_report.read_bytes()
        pnr_text = pnr_raw.decode()
        usage = re.search(r"BSRAM\s*\|\s*(\d+)\s*/\s*(\d+)", pnr_text)
        check(usage is not None, "PNR report has no BSRAM utilization")
        report["total_design_bsram_blocks"] = int(usage[1])
        report["available_device_bsram_blocks"] = int(usage[2])
        report["pnr_report_sha256"] = digest(pnr_raw)
        if args.baseline_pnr_report.exists():
            baseline_raw = args.baseline_pnr_report.read_bytes()
            baseline_text = baseline_raw.decode()
            baseline_usage = re.search(r"BSRAM\s*\|\s*(\d+)\s*/\s*(\d+)", baseline_text)
            check(baseline_usage is not None, "baseline PNR has no BSRAM utilization")
            baseline_total = int(baseline_usage[1])
            physical_registerfile = int(usage[1]) - baseline_total - len(instances)
            registerfile_macros = re.findall(r"^\s*SDPB\s+(cpuregs_\w+)\s*\(", netlist, re.M)
            check(len(registerfile_macros) == 2, "expected duplicated CPU registerfile read ports")
            for kind in ("SDPB", "pROM"):
                old = re.search(rf"--{kind}\s*\|\s*(\d+)", baseline_text)
                new = re.search(rf"--{kind}\s*\|\s*(\d+)", pnr_text)
                check(old is not None and new is not None, f"missing {kind} resource breakdown")
                if kind == "pROM":
                    check(old[1] == new[1], "baseline camera ROM resource count changed")
                else:
                    check(int(new[1]) - int(old[1]) == len(instances) + physical_registerfile,
                          "SDPB resource delta does not match CPU RAM/registerfile accounting")
            report["baseline_design_bsram_blocks"] = baseline_total
            report["baseline_pnr_report_sha256"] = digest(baseline_raw)
            report["cpu_registerfile_logical_sdpb_macros"] = registerfile_macros
            report["cpu_registerfile_physical_bsram_blocks_from_pnr_delta"] = physical_registerfile
            report["total_cpu_added_bsram_blocks"] = int(usage[1]) - baseline_total
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + "\n")
    print(f"PASS: 65536 firmware bytes match all 32 synthesized BSRAM planes; {args.output}")


if __name__ == "__main__":
    main()
