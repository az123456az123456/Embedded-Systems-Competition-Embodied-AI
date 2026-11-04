#!/usr/bin/env python3
"""Compile the production core against the exact frozen FPGA C server."""
import hashlib
import json
import os
from pathlib import Path
import subprocess
import tempfile
import zlib

package = Path(__file__).resolve().parents[1]
report_dir = package / "tests" / "reports"
report_dir.mkdir(exist_ok=True)
results = []

def run(command, **kwargs):
    result = subprocess.run(command, capture_output=True, text=True, **kwargs)
    if result.returncode:
        raise RuntimeError("command failed: " + repr(command) + "\n" + result.stdout + result.stderr)
    return result

for sanitizers in (False, True):
    with tempfile.TemporaryDirectory(prefix="acg138-map-bridge-") as temporary:
        work = Path(temporary)
        flags = ["-g", "-O1", "-fno-omit-frame-pointer"]
        if sanitizers:
            flags += ["-fsanitize=address,undefined"]
        commands = []
        for name in ("astar", "nav_protocol"):
            command = ["cc", "-std=c11", "-Wall", "-Wextra", "-Werror", *flags,
                       "-c", str(package / "tests/reference" / (name + ".c")),
                       "-o", str(work / (name + ".o"))]
            run(command)
            commands.append(command)
        command = ["c++", "-std=c++17", "-Wall", "-Wextra", "-Wpedantic", "-Werror", *flags,
                   "-I", str(package / "include"), "-I", str(package / "tests"),
                   str(package / "src/map_core.cpp"), str(package / "src/nav2_wire.cpp"),
                   str(package / "src/raw_ethernet.cpp"), str(package / "tests/test_map_bridge.cpp"),
                   str(work / "astar.o"), str(work / "nav_protocol.o"), "-pthread", "-o", str(work / "test")]
        run(command)
        commands.append(command)
        env = dict(os.environ)
        # LeakSanitizer cannot inspect processes under some restricted tracing
        # environments. ASan bounds/use-after-free and UBSan remain active.
        env["ASAN_OPTIONS"] = "detect_leaks=0:halt_on_error=1"
        env["UBSAN_OPTIONS"] = "halt_on_error=1:print_stacktrace=1"
        result = run([str(work / "test")], timeout=45, env=env)
        results.append({"asan_ubsan": sanitizers, "passed": True,
                        "output": result.stdout, "commands": commands})

def crc16(data):
    c = 0xffff
    for value in data:
        c ^= value << 8
        for _ in range(8):
            c = ((c << 1) ^ (0x1021 if c & 0x8000 else 0)) & 0xffff
    return c

def body(op):
    data = bytearray(64)
    data[:4] = b'NAV2'
    data[4:6] = bytes([1, op])
    data[8:16] = (op).to_bytes(4, 'big') + (1).to_bytes(4, 'big')
    if op == 2:
        data[7] = 32
        data[32:] = bytes(range(32))
    return data

def packet(data, bad_crc=False, bad_fcs=False, wrong_destination=False):
    destination = bytes.fromhex('02ac13800001') if not wrong_destination else bytes.fromhex('02ac13800002')
    frame = destination + bytes.fromhex('02010203040588b9') + data
    check = crc16(data) ^ int(bad_crc)
    frame += check.to_bytes(2, 'big')
    fcs = zlib.crc32(frame) ^ int(bad_fcs)
    return bytes([0x55] * 7 + [0xd5]) + frame + fcs.to_bytes(4, 'little')

with tempfile.TemporaryDirectory(prefix='acg138-nav2-rtl-') as temporary:
    work = Path(temporary)
    malformed = body(1)
    malformed[0] = ord('X')
    frames = (packet(body(1)) + packet(body(1), bad_crc=True)
              + packet(body(1), bad_fcs=True) + packet(malformed)
              + packet(body(1), wrong_destination=True) + packet(body(2)))
    (work / 'frames.hex').write_text(''.join(f'{value:02x}\n' for value in frames))
    command = ['iverilog', '-g2012', '-s', 'tb_nav2_receiver', '-o', str(work / 'receiver.vvp'),
               str(package / 'tests/reference/nav_packet_rx.v'), str(package / 'tests/tb_nav2_receiver.sv')]
    run(command)
    result = run(['vvp', str(work / 'receiver.vvp')], cwd=work, timeout=15)
    results.append({'actual_fpga_receiver_rtl': True, 'passed': True,
                    'output': result.stdout, 'command': command})
files = [p for p in package.rglob("*") if p.is_file() and "reports" not in p.parts
         and "__pycache__" not in p.parts]
hashes = {str(p.relative_to(package)): hashlib.sha256(p.read_bytes()).hexdigest()
          for p in sorted(files)}
report = {"host_only": True, "ros2_node_compiled": False,
          "board_tested": False, "firmware_oracle": "v36 raw nav_protocol.c + astar.c",
          "results": results, "sha256": hashes}
output = report_dir / "host-verification.json"
output.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n")
for result in results:
    print(result["output"], end="")
print("report=" + str(output))
