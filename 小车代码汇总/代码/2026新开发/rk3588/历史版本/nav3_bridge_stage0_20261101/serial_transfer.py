#!/usr/bin/env python3
"""Transfer a small stage delivery archive with checksum over the serial console."""
import argparse
import base64
import hashlib
from pathlib import Path
import re
import shlex
import subprocess
import time
import uuid
import serial
p=argparse.ArgumentParser()
p.add_argument('file');p.add_argument('--output',required=True)
a=p.parse_args();src=Path(a.file);data=src.read_bytes()
if len(data)>250000:raise SystemExit('Archive too large for this bounded console transfer')
digest=hashlib.sha256(data).hexdigest()
dest='/root/nav3-stage0-'+digest[:16]+'.tar.gz'
busy=subprocess.run(['fuser','/dev/ttyUSB0'],capture_output=True,text=True)
if busy.returncode==0:raise SystemExit('Console occupied: '+busy.stdout)
token='NAV3_COPY_'+uuid.uuid4().hex
payload=base64.b64encode(data).decode()
raw=bytearray()
with serial.Serial('/dev/ttyUSB0',1500000,timeout=0.1,write_timeout=5,
                   xonxoff=False,rtscts=False,dsrdtr=False,exclusive=True) as s:
    # Existing identical-content archive may be replaced; no production file is touched.
    # A shell reads a complete heredoc BEFORE executing any commands on its line.
    # Disable echo in a separate command, otherwise all base64 text is echoed.
    s.write(b'stty -echo\r');s.flush();time.sleep(.1)
    s.write(("base64 -d > "+shlex.quote(dest)+" <<'"+token+"'\n").encode());s.flush()
    time.sleep(.1)
    for i in range(0,len(payload),512):
        s.write((payload[i:i+512]+'\n').encode());s.flush();time.sleep(.003)
    s.write((token+"\nstty echo; printf '\\n'; sha256sum "+shlex.quote(dest)+"; printf '\\n"+token+"_DONE\\n'\n").encode());s.flush()
    deadline=time.monotonic()+15
    while time.monotonic()<deadline:
        raw.extend(s.read(8192))
        if re.search(rb'(?:^|\n)'+token.encode()+rb'_DONE\r?\n',raw):break
text=raw.decode('utf-8','replace').replace('\r','')
Path(a.output).write_text(text)
print(text[-1500:])
if not re.search(r'(?m)^'+digest+r'\s+'+re.escape(dest)+r'$',text):
    raise SystemExit('Remote SHA256 not verified; do not extract')
print('VERIFIED '+dest)
