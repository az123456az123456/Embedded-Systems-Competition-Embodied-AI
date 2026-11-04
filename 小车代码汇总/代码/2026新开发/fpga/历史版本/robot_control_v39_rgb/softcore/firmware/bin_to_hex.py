#!/usr/bin/env python3
"""Create a complete 64 KiB little-endian RAM image and 8-bit lane images."""
import argparse
from pathlib import Path
import struct


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("binary", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    data = args.binary.read_bytes()
    if len(data) > 65536:
        parser.error("firmware binary exceeds 64 KiB RAM")
    data += bytes(65536 - len(data))
    args.output.write_text("".join(f"{word:08x}\n" for (word,) in struct.iter_unpack("<I", data)))
    for lane in range(4):
        filename = args.output.with_name(f"{args.output.stem}_lane{lane}.hex")
        filename.write_text("".join(f"{byte:02x}\n" for byte in data[lane::4]))


if __name__ == "__main__":
    main()
