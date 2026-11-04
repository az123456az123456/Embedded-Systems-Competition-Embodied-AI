#!/usr/bin/env python3
"""One bounded, logged shell transaction on the user-authorized RK console.
Never steals an occupied tty, logs raw evidence, does not manage ROS sessions.
"""
import argparse
import hashlib
import json
from pathlib import Path
import re
import shlex
import subprocess
import time
import uuid
import serial

p = argparse.ArgumentParser()
p.add_argument('--command-file', required=True)
p.add_argument('--output', required=True)
p.add_argument('--timeout', type=float, default=15)
a = p.parse_args()
command = Path(a.command_file).read_text()
if len(command.encode()) > 3500:
    raise SystemExit('Console command too long; transfer files separately')
occupied = subprocess.run(['fuser', '/dev/ttyUSB0'], capture_output=True, text=True)
if occupied.returncode == 0:
    raise SystemExit('Console occupied: ' + occupied.stdout)
out = Path(a.output)
out.parent.mkdir(parents=True, exist_ok=True)
token = 'NAV3_' + uuid.uuid4().hex
begin = token + '_BEGIN'
end = token + '_END'
shell = "printf '\\n%s\\n' " + shlex.quote(begin) + '; sh -c ' + shlex.quote(command)
shell += "; printf '\\n%s:%s\\n' " + shlex.quote(end) + ' "$?"\r'
data = bytearray()
with serial.Serial('/dev/ttyUSB0', 1500000, timeout=0.1, write_timeout=5,
                   xonxoff=False, rtscts=False, dsrdtr=False, exclusive=True) as s:
    s.write(shell.encode()); s.flush()
    deadline = time.monotonic() + a.timeout
    found = None
    pattern = re.compile(rb'(?:^|\n)' + end.encode() + rb':(\d+)\r?\n')
    while time.monotonic() < deadline:
        data.extend(s.read(8192))
        found = pattern.search(data)
        if found:
            break
out.with_suffix('.raw').write_bytes(data)
text = data.decode('utf-8', 'replace').replace('\r', '')
start = text.find('\n' + begin + '\n')
clean = text[start + len(begin) + 2:] if start >= 0 else text
clean = clean.split('\n' + end + ':')[0]
out.with_suffix('.txt').write_text(clean)
record = {'command_file': str(Path(a.command_file).resolve()),
          'command_sha256': hashlib.sha256(command.encode()).hexdigest(),
          'tty': '/dev/ttyUSB0', 'baud': 1500000, 'flow_control': 'none',
          'complete': found is not None,
          'exit_code': int(found.group(1)) if found else None}
out.with_suffix('.json').write_text(json.dumps(record, indent=2) + '\n')
print(clean)
print(json.dumps(record))
raise SystemExit(record['exit_code'] if found else 124)
