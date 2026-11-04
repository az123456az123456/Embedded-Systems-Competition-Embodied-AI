#!/usr/bin/env python3
"""One bounded, exclusive RK console command; does not issue motion commands."""
import argparse, pathlib, secrets, serial, time, re
p=argparse.ArgumentParser();p.add_argument('script');p.add_argument('--port',default='/dev/ttyUSB0');p.add_argument('--timeout',type=float,default=20);p.add_argument('--output',required=True);a=p.parse_args()
script=pathlib.Path(a.script).read_text();token='RKDONE_'+secrets.token_hex(8)
# A single quoted sh script avoids the interactive console interpreting newlines.
quote=lambda x:"'"+x.replace("'","'\\''")+"'"
cmd="sh -c "+quote(script)+"; printf '\\n"+token+"=%s\\n' \"$?\"\r"
raw=b''
done=re.compile(rb'(?:\r?\n)'+token.encode()+rb'=([0-9]+)\r?\n')
with serial.Serial(a.port,1500000,timeout=.1,write_timeout=2,rtscts=False,dsrdtr=False,xonxoff=False,exclusive=True) as s:
 s.write(b'\x15');s.write(cmd.encode());s.flush();end=time.monotonic()+a.timeout
 while time.monotonic()<end:
  raw+=s.read(65536)
  if done.search(raw):break
pathlib.Path(a.output).write_bytes(raw)
text=raw.decode('utf-8','replace');print(text[-16000:])
match=done.search(raw)
if not match:raise SystemExit('console command deadline; inspect raw transcript before another command')
raise SystemExit(int(match.group(1)))
