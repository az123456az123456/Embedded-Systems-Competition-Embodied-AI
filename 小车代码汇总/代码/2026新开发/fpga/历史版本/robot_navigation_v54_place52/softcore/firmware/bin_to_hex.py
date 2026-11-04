#!/usr/bin/env python3
"""Create the approved complete 80KiB image, low bank and tail nibble planes."""
import argparse
import hashlib
import json
from pathlib import Path
import struct


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("binary", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    profile_path = Path(__file__).with_name("memory_profile.json")
    profile = json.loads(profile_path.read_text())
    if (profile["ram_bytes"], profile["low_bank_bytes"], profile["tail_bank_bytes"],
            profile["stack_bytes"], profile["image_end_limit"]) != (81920, 65536, 16384, 4096, 76504):
        parser.error("memory profile differs from the explicitly approved 80KiB layout")
    data = args.binary.read_bytes()
    binary_bytes = len(data)
    if len(data) > profile["image_end_limit"]:
        parser.error("firmware binary exceeds approved 76504-byte image limit")
    data += bytes(profile["ram_bytes"] - len(data))
    outputs = {}
    def write(path, text, width, words, offset):
        path.write_text(text)
        outputs[path.name] = {"width_bits": width, "words": words, "byte_offset": offset,
                              "sha256": hashlib.sha256(path.read_bytes()).hexdigest()}
    write(args.output, "".join(f"{word:08x}\n" for (word,) in struct.iter_unpack("<I", data)),
          32, 20480, 0)
    low = data[:profile["low_bank_bytes"]]
    write(args.output.with_name(f"{args.output.stem}_low.hex"),
          "".join(f"{word:08x}\n" for (word,) in struct.iter_unpack("<I", low)), 32, 16384, 0)
    for lane in range(4):
        filename = args.output.with_name(f"{args.output.stem}_lane{lane}.hex")
        write(filename, "".join(f"{byte:02x}\n" for byte in data[lane::4]), 8, 20480, lane)
        tail_lane = data[profile["low_bank_bytes"]+lane::4]
        for name, shift in (("lo", 0), ("hi", 4)):
            filename = args.output.with_name(f"{args.output.stem}_tail_lane{lane}_{name}.hex")
            write(filename, "".join(f"{(byte >> shift) & 15:x}\n" for byte in tail_lane),
                  4, 4096, profile["low_bank_bytes"] + lane)
    metadata = {"schema": 1, "profile": profile, "input_binary_bytes": binary_bytes,
                "input_sha256": hashlib.sha256(args.binary.read_bytes()).hexdigest(),
                "complete_image_sha256": hashlib.sha256(data).hexdigest(),
                "memory_profile_sha256": hashlib.sha256(profile_path.read_bytes()).hexdigest(),
                "outputs": outputs,
                "scope": "Initialization bytes only; whole ELF/BSS size and runtime/RTL are separate gates"}
    args.output.with_name(f"{args.output.stem}.memory.json").write_text(json.dumps(metadata, indent=2)+"\n")


if __name__ == "__main__":
    main()
