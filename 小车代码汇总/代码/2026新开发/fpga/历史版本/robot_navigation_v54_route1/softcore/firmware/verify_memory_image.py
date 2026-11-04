#!/usr/bin/env python3
"""Check every initialized byte across the approved low/tail and lane views."""
import argparse
import hashlib
import json
from pathlib import Path
import re
import struct


def verify_images(image):
    image = Path(image)
    manifest = json.loads(image.with_name(image.stem + ".memory.json").read_text())
    profile = manifest["profile"]
    assert (profile["ram_bytes"], profile["low_bank_bytes"], profile["tail_bank_bytes"],
            profile["stack_bytes"], profile["image_end_limit"]) == (81920,65536,16384,4096,76504)
    def values(path, digits, count):
        lines = path.read_text().splitlines()
        assert len(lines) == count, f"{path.name}: wrong initialization depth"
        assert all(re.fullmatch(r"[0-9a-fA-F]{"+str(digits)+"}", line) for line in lines), path.name
        assert hashlib.sha256(path.read_bytes()).hexdigest() == manifest["outputs"][path.name]["sha256"], path.name
        return [int(line,16) for line in lines]
    words = values(image,8,20480)
    full = struct.pack("<20480I", *words)
    low_path = image.with_name(image.stem + "_low.hex")
    low = struct.pack("<16384I", *values(low_path,8,16384))
    assert full[:65536] == low, "low bank differs from whole image"
    for lane in range(4):
        path = image.with_name(image.stem + f"_lane{lane}.hex")
        assert bytes(values(path,2,20480)) == full[lane::4], f"lane {lane} byte order"
        nibble = [values(image.with_name(image.stem+f"_tail_lane{lane}_{side}.hex"),1,4096)
                  for side in ("lo","hi")]
        actual = bytes(nibble[0][i] + 16*nibble[1][i] for i in range(4096))
        assert actual == full[65536+lane::4], f"tail lane {lane} nibble order"
    assert hashlib.sha256(full).hexdigest() == manifest["complete_image_sha256"]
    length = manifest["input_binary_bytes"]
    assert 0 <= length <= 76504
    assert hashlib.sha256(full[:length]).hexdigest() == manifest["input_sha256"]
    assert full[length:] == bytes(81920-length), "uninitialized/padded memory must be zero"
    return {"status":"PASS", "ram_bytes":81920, "words_checked":20480,
            "all_lane_and_tail_bits_match":True, "zero_padding_checked":81920-length,
            "scope":"Initialization image consistency only; synthesis and runtime are separate gates"}


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("image",type=Path)
    args=parser.parse_args()
    try:
        print(json.dumps(verify_images(args.image)))
    except (AssertionError,KeyError,ValueError,OSError) as error:
        parser.exit(1,"RAM initialization verification failed: "+str(error)+"\n")


if __name__ == "__main__":
    main()
