#!/usr/bin/env python3
"""Check/collect a completed Gowin 30 FPS build; no board-I/O signoff."""
import argparse
from collections import Counter
import hashlib
import html
import json
from pathlib import Path
import re
import shutil
import xml.etree.ElementTree as ET


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def rows(markup):
    result = []
    for row in re.findall(r"<tr[^>]*>(.*?)</tr>", markup, re.S):
        result.append([
            html.unescape(re.sub(r"<[^>]+>", "", cell)).strip()
            for cell in re.findall(r"<t[dh][^>]*>(.*?)</t[dh]>", row, re.S)
        ])
    return result


def section(markup, start, end):
    return markup.split('name="' + start + '"', 1)[1].split(
        'name="' + end + '"', 1)[0]


def pin_rows(path):
    markup = path.read_text()
    table = markup.split('name="Pinout_by_Port_Name"', 1)[1].split(
        "</table>", 1)[0]
    return {row[0]: row for row in rows(table)[1:]}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--build-dir", required=True, type=Path)
    parser.add_argument("--baseline", type=Path)
    args = parser.parse_args()
    project = Path(__file__).resolve().parents[1]
    baseline = args.baseline or project.with_name("imu_camera_dual_1080_20fps")
    build = args.build_dir.resolve()
    name = "imu_camera_dual_1080_30fps"
    pnr = build / "impl/pnr"
    log = (build / "gowin.log").read_text()
    assert "Bitstream generation completed" in log
    assert "[100%] Power analysis completed" in log
    assert not re.search(r"^ERROR", log, re.M), "Gowin build error"

    tcl = (project / "build_30fps.tcl").read_text()
    inputs = re.findall(r"^add_file (\S+)$", tcl, re.M)
    assert len(inputs) == 32 and len(set(inputs)) == 32
    project_files = [node.attrib["path"] for node in ET.parse(
        project / (name + ".gprj")).findall(".//File")
        if node.attrib.get("enable", "1") == "1"]
    assert project_files == inputs, "GUI and command-line source lists differ"
    hashes = {file: digest(project / file) for file in inputs}
    assert hashes == {file: digest(build / file) for file in inputs}, \
        "Packaged sources differ from compilation inputs"
    changed = [file for file in inputs if digest(baseline / file) != hashes[file]]
    assert changed == ["camera/ov5640_init_table_raw.v"], changed
    auxiliary = {file: digest(project / file) for file in (
        "camera/ov5640_af.hex", "build_30fps.tcl")}
    assert auxiliary == {file: digest(build / file) for file in auxiliary}

    pins = pin_rows(pnr / (name + ".pin.html"))
    old_pins = pin_rows(baseline / "reports/20fps/imu_camera_dual_1080_20fps.pin.html")
    assert len(pins) == 46 and pins == old_pins, "Pin report changed from 20 FPS"
    cst = (project / "imu_camera_dual.cst").read_text()
    locations = dict(re.findall(r'IO_LOC "([^"]+)" (\w+);', cst))
    assert len(locations) == 46 and pins.keys() == locations.keys()
    attr_column = {"IO_TYPE": 7, "DRIVE": 8, "PULL_MODE": 9, "BANK_VCCIO": 17}
    for port, attrs in re.findall(r'IO_PORT "([^"]+)" ([^;]+);', cst):
        assert pins[port][2].split("/")[0] == locations[port]
        assert pins[port][3] == "Y", "Pin constraint ignored: " + port
        for key, value in re.findall(r"(\w+)=([\w.]+)", attrs):
            assert pins[port][attr_column[key]] == value, (port, key, value)
    pin_summary = {port: {"location": row[2], "direction": row[4],
        "io_type": row[7], "bank_vccio": row[17]} for port, row in pins.items()}

    timing_html = (pnr / (name + "_tr_content.html")).read_text()
    summary = dict(row[:2] for row in rows(section(timing_html,
        "STA_Tool_Run_Summary", "Clock_Report")) if len(row) == 2)
    timing = {
        "setup": int(summary["Numbers of Setup Violated Endpoints"]),
        "hold": int(summary["Numbers of Hold Violated Endpoints"]),
        "frequency_table": [],
    }
    assert timing["setup"] == 0 and timing["hold"] == 0, timing
    for row in rows(section(timing_html, "Max_Frequency_Report",
            "Total_Negative_Slack_Report"))[1:]:
        if len(row) == 6 and row[0].isdigit():
            constraint, actual = [float(v.split("(")[0]) for v in row[2:4]]
            assert actual >= constraint, row
            timing["frequency_table"].append({"clock": row[1],
                "constraint_mhz": constraint, "actual_fmax_mhz": actual,
                "logic_levels": int(row[4])})
    assert len(timing["frequency_table"]) == 5
    kinds = ("Setup", "Hold", "Recovery", "Removal")
    ends = ("Hold_Slack_Table", "Recovery_Slack_Table", "Removal_Slack_Table",
            "MIN_PULSE_WIDTH_TABLE")
    for kind, end in zip(kinds, ends):
        path_rows = rows(section(timing_html, kind + "_Slack_Table", end))
        slacks = [float(row[1]) for row in path_rows
            if len(row) >= 2 and row[0].isdigit()]
        assert slacks and min(slacks) >= 0, (kind, slacks[:1])
        timing[kind.lower() + "_minimum_reported_slack_ns"] = min(slacks)
    timing.update({
        "paths_analyzed": int(summary["Numbers of Paths Analyzed"]),
        "endpoints_analyzed": int(summary["Numbers of Endpoints Analyzed"]),
        "pins_verified": 46, "synthesis_inputs_verified": 32,
        "constraints_unchanged_vs_20fps": True, "hardware_tested": False,
        "scope": "Declared internal STA only; external camera/RGMII IO not signed off",
    })

    warning_lines = [line for line in log.splitlines() if line.startswith("WARN")]
    codes = Counter(re.search(r"\((\w+)\)", line)[1] for line in warning_lines)
    assert codes["PR1014"] == 3
    modified_warnings = [line for line in warning_lines
        if any(file in line for file in changed)]
    assert not modified_warnings
    old_release = json.loads((baseline / "reports/20fps/release_verification.json").read_text())
    normal = lambda lines: [re.sub(r'/tmp/acg138_camera(?:20|30)_build_20261019/',
        'BUILD/', line) for line in lines]
    warnings_unchanged = normal(warning_lines) == normal(old_release["warnings"]["lines"])
    assert warnings_unchanged, "Warnings differ from archived 20 FPS warning list"

    output = project / "reports/30fps"
    output.mkdir(parents=True, exist_ok=True)
    target_pnr = project / "impl/pnr"
    target_pnr.mkdir(parents=True, exist_ok=True)
    fs = pnr / (name + ".fs")
    assert digest(fs) != old_release["bitstream_sha256"], "Bitstream unchanged from 20 FPS"
    shutil.copy2(fs, target_pnr / fs.name)
    suffixes = (".pin.html", ".power.html", ".rpt.html", ".rpt.txt", ".log",
        ".timing_paths", ".tr.html", "_tr_cata.html", "_tr_content.html")
    for suffix in suffixes:
        source = pnr / (name + suffix)
        shutil.copy2(source, output / source.name)
    shutil.copy2(build / "gowin.log", output / "gowin.log")
    report = {
        "date": "2026-10-19", "project": name, "top": "imu_camera_dual_top",
        "device": "GW5AT-LV60PG484AC1/I0", "tool_version": "V1.9.12.03",
        "scope": "dual OV5640 camera + MPU9250 filtered stream, nominal30FPS, no motors",
        "bitstream": "impl/pnr/" + fs.name, "bitstream_sha256": digest(fs),
        "bitstream_bytes": fs.stat().st_size, "hardware_tested": False,
        "timing": timing, "synthesis_inputs": hashes, "auxiliary_inputs": auxiliary,
        "synthesis_input_project_list_verified": True,
        "changed_synthesis_inputs_vs_20fps": changed,
        "pin_report_matches_cst": True, "pin_report_matches_20fps": True,
        "all_pin_report_attributes_match_20fps": True, "pins": pin_summary,
        "warnings": {"count": len(warning_lines), "codes": dict(codes),
            "all_source_warnings_in_unchanged_files": True,
            "warnings_in_modified_sources": modified_warnings,
            "normalized_warning_list_unchanged_vs_20fps": warnings_unchanged,
            "review": "Inherited width/unused/delay warnings; three PR1014 generic clock routing warnings remain for clk50m_d and both camera PCLK inputs. No warnings target the changed ROM.",
            "lines": warning_lines},
        "limits": [
            "30 FPS hardware operation is not tested. External camera DATA/HREF/VSYNC relative to PCLK and RGMII board timing are not signed off.",
            "SDC declares 100 MHz camera clock envelopes but has no full external input/output delay model. Internal setup/hold pass is not external IO verification.",
            "PR1014 generic clock routing warnings remain for clk50m and two camera PCLK inputs.",
            "Encrypted Gowin FIFO models are unchanged. Physical FIFO depth/overflow and sustained RK3588 capture/ISP throughput require board tests.",
            "Sensor PLL/PCLK are unchanged from the user-tested 20 FPS release. VTS and AEC/banding limits change to target 30.048 FPS. YOLO inference FPS is independent.",
        ],
    }
    (output / "timing_summary.json").write_text(json.dumps(timing, indent=2) + "\n")
    (output / "release_verification.json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps({"bitstream_sha256": digest(fs), "timing": timing,
        "pins_verified": 46, "inputs_verified": 32,
        "changed_inputs": changed, "warning_codes": dict(codes)}, indent=2))


if __name__ == "__main__":
    main()
