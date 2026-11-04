#!/usr/bin/env python3
"""FPC1, local raw-Ethernet image calibration. No motor-control capabilities.

Uses only Python's standard library on Buildroot. AF_PACKET requires root or
CAP_NET_RAW. Dark/bright preparation can also run offline on a PC.
"""
import argparse
import binascii
import json
import hashlib
import importlib.util
import select
import socket
import struct
import sys
import time
from pathlib import Path

ETHERTYPE = 0x88BB
FPGA_MAC = "02:ac:13:80:00:01"
WIDTH = 960
OPS = {"begin": 1, "write": 2, "commit": 3, "bypass": 4, "gamma": 5, "info": 6}
STATUS = {0: "ok", 1: "fpn_transaction_state", 2: "column_order", 3: "gain_out_of_range",
          4: "table_incomplete", 5: "busy", 6: "internal_ack_mismatch",
          7: "request_id_conflict", 8: "stale_request_id"}


def crc16(data):
    return binascii.crc_hqx(data, 0xFFFF)


def mac_bytes(text):
    value = bytes.fromhex(text.replace(":", ""))
    if len(value) != 6:
        raise ValueError("MAC address must contain exactly six bytes")
    return value


def make_request(op, eye, request_id, column=0, dark=0, gain=0, value=0):
    if isinstance(op, str):
        op = OPS[op]
    if eye not in (0, 1) or op not in OPS.values() or not 0 < request_id <= 0xFFFFFFFF:
        raise ValueError("invalid operation, eye or nonzero request ID")
    if op == OPS["write"]:
        if not (0 <= column < WIDTH and 0 <= dark <= 255 and 1 <= gain <= 1023 and value == 0):
            raise ValueError("WRITE requires column 0..959, dark U8, gain Q2.8 U10")
    elif op == OPS["gamma"]:
        if column or dark or gain or value not in (0, 1):
            raise ValueError("GAMMA value must be 0 or 1")
    elif column or dark or gain or value:
        raise ValueError("unused request fields must be zero")
    body = bytearray(64)
    struct.pack_into(">4sBBBBIHBBH", body, 0, b"FPC1", 1, op, eye, 0,
                     request_id, column, dark, value, gain)
    return bytes(body)


def encode_ethernet(body, source, destination=FPGA_MAC):
    if len(body) != 64:
        raise ValueError("FPC1 body must contain exactly 64 bytes")
    # The NIC appends preamble/FCS; AF_PACKET supplies Ethernet header + body.
    return mac_bytes(destination) + mac_bytes(source) + struct.pack(">H", ETHERTYPE) + body + struct.pack(">H", crc16(body))


def decode_ack(frame, source=FPGA_MAC, destination=None):
    if len(frame) != 80:
        raise ValueError("ACK Ethernet length must be 80 bytes (NIC strips FCS)")
    if frame[6:12] != mac_bytes(source) or frame[12:14] != struct.pack(">H", ETHERTYPE):
        raise ValueError("ACK source MAC or EtherType mismatch")
    if destination is not None and frame[:6] not in (mac_bytes(destination), b"\xff" * 6):
        raise ValueError("ACK destination mismatch")
    body = frame[14:78]
    if crc16(body) != struct.unpack_from(">H", frame, 78)[0]:
        raise ValueError("ACK application CRC16 mismatch")
    if body[:5] != b"FPC1\x01" or body[5] not in OPS.values() or body[6] not in (0, 1) or body[7] != 0x80:
        raise ValueError("ACK magic/version/op/eye/flags mismatch")
    if body[23] != 0 or any(body[44:]):
        raise ValueError("ACK reserved bytes are nonzero")
    return {"op": body[5], "eye": body[6], "request_id": struct.unpack_from(">I", body, 8)[0],
            "column": struct.unpack_from(">H", body, 12)[0], "dark": body[14], "value": body[15],
            "gain_q8": struct.unpack_from(">H", body, 16)[0], "status": body[18],
            "status_name": STATUS.get(body[18], "fpn_status_%d" % body[18]),
            "table_valid": bool(body[19] & 1), "expected_column": struct.unpack_from(">H", body, 20)[0],
            "gamma_enabled": bool(body[22] & 1), "table_generation": struct.unpack_from(">I", body, 24)[0],
            "applied_frame": struct.unpack_from(">I", body, 28)[0],
            "duplicate_count": struct.unpack_from(">I", body, 32)[0],
            "accepted_commands": struct.unpack_from(">I", body, 36)[0],
            "last_request_id": struct.unpack_from(">I", body, 40)[0]}


class Link:
    def __init__(self, interface, target=FPGA_MAC, timeout=0.5, retries=10):
        if not interface or "/" in interface or len(interface) > 15:
            raise ValueError("invalid Linux network interface name")
        self.interface, self.target = interface, target
        self.source = (Path("/sys/class/net") / interface / "address").read_text().strip()
        mac_bytes(self.source)
        self.timeout, self.retries = timeout, retries
        self.sock = socket.socket(socket.AF_PACKET, socket.SOCK_RAW, socket.htons(ETHERTYPE))
        self.sock.bind((interface, 0))
        self.sock.setblocking(False)
        self.next_id = None

    def transact(self, body):
        expected = (body[5], body[6], struct.unpack_from(">I", body, 8)[0],
                    struct.unpack_from(">H", body, 12)[0], body[14], body[15],
                    struct.unpack_from(">H", body, 16)[0])
        frame = encode_ethernet(body, self.source, self.target)
        for attempt in range(self.retries):
            # An identical body and ID are used on every retry, never a new ID.
            self.sock.send(frame)
            deadline = time.monotonic() + self.timeout
            while True:
                wait = deadline - time.monotonic()
                if wait <= 0 or not select.select([self.sock], [], [], wait)[0]:
                    break
                raw, addr = self.sock.recvfrom(2048)
                if len(addr) > 2 and addr[2] == getattr(socket, "PACKET_OUTGOING", 4):
                    continue
                try:
                    ack = decode_ack(raw, self.target, self.source)
                except ValueError:
                    continue
                if (ack["op"], ack["eye"], ack["request_id"], ack["column"], ack["dark"], ack["value"], ack["gain_q8"]) != expected:
                    continue
                if ack["status"] != 0:
                    raise RuntimeError("FPGA rejected request: " + json.dumps(ack, sort_keys=True))
                return ack
        raise TimeoutError("No valid FPC1 ACK after %d identical retries; no further table writes sent" % self.retries)

    def synchronize(self, eye=0):
        # INFO never changes/replays the mutating-command ID cache. It can
        # establish the next ID after host process restart or a lost ACK.
        info_id = (time.monotonic_ns() & 0xFFFFFFFF) or 1
        ack = self.transact(make_request("info", eye, info_id))
        self.next_id = ((ack["last_request_id"] + 1) & 0xFFFFFFFF) or 1
        return ack

    def command(self, op, eye=0, **fields):
        if self.next_id is None:
            self.synchronize(eye)
        request_id = self.next_id
        ack = self.transact(make_request(op, eye, request_id, **fields))
        self.next_id = ((request_id + 1) & 0xFFFFFFFF) or 1
        return ack


def load_coefficients(path, eye):
    doc = json.loads(Path(path).read_text())
    if doc.get("width") != WIDTH:
        raise ValueError("coefficient width mismatch; require 960")
    expected_eye = {"left": 0, "right": 1}.get(doc.get("eye"), doc.get("eye"))
    if expected_eye is not None and expected_eye != eye:
        raise ValueError("coefficient eye does not match --eye")
    if doc.get("schema") == "FPN-COLUMN-RAW8-Q2.8-v1":
        if doc.get("raw_bits") != 8 or doc.get("gain_fraction_bits") != 8:
            raise ValueError("require RAW8 / gain Q2.8 calibration")
        dark, gain = doc.get("dark", []), doc.get("gain", [])
        if len(dark) != WIDTH or len(gain) != WIDTH:
            raise ValueError("exactly 960 dark and gain values required")
        columns = [{"dark": d, "gain_q8": g} for d, g in zip(dark, gain)]
    elif doc.get("schema") == "fpc1-column-q8-v1":
        columns = doc.get("columns", [])
    else:
        raise ValueError("unknown FPN coefficient schema")
    if len(columns) != WIDTH:
        raise ValueError("exactly 960 column coefficients are required")
    for index, item in enumerate(columns):
        if set(item) != {"dark", "gain_q8"} or not isinstance(item["dark"], int) or not isinstance(item["gain_q8"], int):
            raise ValueError("column %d requires integer dark and gain_q8 only" % index)
        make_request("write", eye, 1, column=index, dark=item["dark"], gain=item["gain_q8"])
    return columns


def prepare(args):
    if not args.locked_capture:
        raise ValueError("prepare requires --locked-capture and real locked-exposure/gain RAW8 dark/flat frames")
    if args.width != WIDTH or args.height <= 0:
        raise ValueError("production coefficient width is 960; height must be positive")
    # Reuse the reviewed phase-aware physical-column calibration. Do not
    # silently substitute whole-column averaging if row-phase model fails.
    helper = Path(__file__).resolve().with_name("calibrate_fpn.py")
    if not helper.is_file():
        helper = Path(__file__).resolve().parents[1] / "fpn" / "calibrate_fpn.py"
    if not helper.is_file():
        raise ValueError("calibrate_fpn.py companion is missing; install both tools")
    spec = importlib.util.spec_from_file_location("fpc1_calibration", helper)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    result = module.calibrate_raw_standard_library(args.dark, args.bright, args.width, args.height,
                                                   min_span=args.min_span,
                                                   max_phase_mismatch=args.max_phase_mismatch,
                                                   max_dark_phase_delta=args.max_dark_phase_delta)
    for warning in result.get("warnings", []):
        print("FPC1 warning: " + warning, file=sys.stderr)
    result["eye"] = "left" if args.eye == 0 else "right"
    result["input_sha256"] = {str(Path(name)): hashlib.sha256(Path(name).read_bytes()).hexdigest()
                               for name in args.dark + args.bright}
    Path(args.output).write_text(json.dumps(result, indent=2) + "\n")
    print("Prepared %d RAW8 column coefficients: %s (validation_mode=%s)" %
          (WIDTH, args.output, result.get("validation_mode", "shared-column-default")))


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--interface", default="eth0")
    p.add_argument("--target", default=FPGA_MAC)
    p.add_argument("--timeout", type=float, default=0.5)
    p.add_argument("--retries", type=int, default=10)
    sub = p.add_subparsers(dest="command", required=True)
    for name in ("info", "bypass", "upload", "gamma"):
        q = sub.add_parser(name)
        q.add_argument("--eye", type=int, choices=(0, 1), default=0)
        if name == "upload":
            q.add_argument("coefficients")
        if name == "gamma":
            q.add_argument("value", choices=("on", "off"))
    q = sub.add_parser("prepare", help="offline RAW8 dark/flat-field coefficient generation")
    q.add_argument("--eye", type=int, choices=(0, 1), required=True)
    q.add_argument("--width", type=int, default=WIDTH)
    q.add_argument("--height", type=int, default=1080)
    q.add_argument("--dark", nargs="+", required=True)
    q.add_argument("--bright", nargs="+", required=True)
    q.add_argument("--locked-capture", action="store_true")
    q.add_argument("--min-span", type=float, default=16.0)
    q.add_argument("--max-phase-mismatch", type=float, default=0.08)
    q.add_argument("--max-dark-phase-delta", type=float, default=2.0,
                   help="maximum odd/even-row dark mean difference in RAW8 DN (default 2.0); raising this permits approximate shared-column evaluation, not phase-specific correction")
    q.add_argument("--output", required=True)
    args = p.parse_args(argv)
    if args.command == "prepare":
        prepare(args)
        return 0
    if not 0.05 <= args.timeout <= 30 or not 1 <= args.retries <= 100:
        raise ValueError("timeout must be 0.05..30s; retries must be 1..100")
    # Validate the WHOLE coefficient file before BEGIN disables current FPN.
    coefficients = load_coefficients(args.coefficients, args.eye) if args.command == "upload" else None
    link = Link(args.interface, args.target, args.timeout, args.retries)
    initial = link.synchronize(args.eye)
    if args.command == "info":
        print(json.dumps(initial, indent=2, sort_keys=True))
        return 0
    if args.command == "upload":
        begin = link.command("begin", args.eye)
        if begin["table_valid"] or begin["expected_column"] != 0:
            raise RuntimeError("BEGIN ACK did not confirm disabled table / column zero; upload aborted")
        for column, item in enumerate(coefficients):
            ack = link.command("write", args.eye, column=column, dark=item["dark"], gain=item["gain_q8"])
            if ack["expected_column"] != column + 1:
                raise RuntimeError("WRITE ACK column counter mismatch; COMMIT not sent")
            if (column + 1) % 120 == 0:
                print("Uploaded eye %d: %d/%d" % (args.eye, column + 1, WIDTH), file=sys.stderr)
        ack = link.command("commit", args.eye)
        # COMMIT can be accepted pending the next SOF; verify INFO separately.
        print(json.dumps({"commit_ack": ack, "note": "If table_valid is false, INFO after next camera SOF must confirm application."}, indent=2))
    else:
        fields = {"value": int(args.value == "on")} if args.command == "gamma" else {}
        print(json.dumps(link.command(args.command, args.eye, **fields), indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except (ValueError, OSError, RuntimeError, TimeoutError) as exc:
        print("FPC1 error: %s" % exc, file=sys.stderr)
        sys.exit(1)
