#!/usr/bin/env python3
"""Check a capture bundle on a PC; Python stdlib only, no NPU or image library."""
import argparse
import hashlib
import json
from pathlib import Path
import zlib


def ppm(path, width):
    data = path.read_bytes()
    header = f"P6\n{width} 640\n255\n".encode()
    if not data.startswith(header) or len(data) != len(header) + width * 640 * 3:
        raise ValueError(f"Invalid P6 RGB dimensions/extent: {path}")
    return data[len(header):]


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("directory", type=Path)
    ap.add_argument("--expect-letterbox", action="store_true", help="Require 35-left/36-right pad114 in both eyes")
    ap.add_argument("--rtl-pattern", action="store_true", help="Compare TX testbench's independent RGB pattern")
    ap.add_argument("--report", type=Path)
    args = ap.parse_args()
    root = args.directory.resolve()
    metadata = json.loads((root / "metadata.json").read_text())
    if metadata.get("schema") != 1 or metadata.get("format") != "RGB888" or metadata.get("width") != 1280 or metadata.get("height") != 640:
        raise ValueError("Capture metadata format/dimensions differ")
    if metadata.get("complete_mask") != 3 or not 1 <= metadata.get("epoch", 0) <= 65535:
        raise ValueError("Not a verified complete stereo session")
    checksums = {}
    expected_names = {"stereo.rgb", "left.rgb", "right.rgb", "stereo.ppm", "left.ppm", "right.ppm"}
    if set(metadata["files"]) != expected_names:
        raise ValueError("Unexpected capture artifact names")
    for name, expected in metadata["files"].items():
        data = (root / name).read_bytes()
        if len(data) != expected["bytes"] or f"{zlib.crc32(data):08x}" != expected["crc32"]:
            raise ValueError(f"Capture checksum mismatch: {name}")
        checksums[name] = hashlib.sha256(data).hexdigest()
    stereo = ppm(root / "stereo.ppm", 1280)
    if stereo != (root / "stereo.rgb").read_bytes():
        raise ValueError("Stereo PPM and RGB pixels differ")
    warnings = []
    for eye, name in enumerate(["left", "right"]):
        pixels = ppm(root / f"{name}.ppm", 640)
        expected = b"".join(stereo[(y * 1280 + eye * 640) * 3:(y * 1280 + eye * 640 + 640) * 3] for y in range(640))
        if pixels != expected or pixels != (root / f"{name}.rgb").read_bytes():
            raise ValueError(f"{name} eye/row order differs from complete stereo frame")
        padding_ok = all(pixels[y * 1920:y * 1920 + 105] == bytes([114]) * 105 and
                         pixels[y * 1920 + 604 * 3:(y + 1) * 1920] == bytes([114]) * 108 for y in range(640))
        if args.expect_letterbox and not padding_ok:
            raise ValueError(f"{name} eye lacks the expected original-model letterbox114 padding")
        if not padding_ok:
            warnings.append(f"{name}: no 35/36 pad114; normal for transport-only synthetic patterns, check on real preprocessor")
        if args.rtl_pattern:
            seq = metadata["sequence"]
            predicted = bytes(channel for n in range(640 * 640) for channel in (
                (n + seq * 3 + eye * 11) & 255, (n // 256 + eye * 47) & 255, (n * 7 + eye * 53 + seq) & 255))
            if pixels != predicted:
                raise ValueError(f"{name} RGB differs from independent TX testbench formula")
    result = {"status": "PASS", "sequence": metadata["sequence"], "epoch": metadata["epoch"],
              "dimensions": [1280, 640], "eyes": [[640, 640], [640, 640]], "complete_mask": 3,
              "sha256": checksums, "rtl_pattern_checked": args.rtl_pattern,
              "letterbox_required": args.expect_letterbox, "warnings": warnings,
              "scope": "Stored byte/CRC/dimensions/eye consistency only; no inference, optics or hardware FPS measurement"}
    if args.report:
        args.report.write_text(json.dumps(result, indent=2, ensure_ascii=False) + "\n")
    print(json.dumps(result, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
