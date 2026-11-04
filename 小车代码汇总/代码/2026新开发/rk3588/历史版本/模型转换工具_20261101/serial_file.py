"""Bounded serial file transfer, with SHA256 verification before final rename.

Uploads only to the separate model-validation directory, never production files.
Temporarily restores a raw terminal; no FPGA or motor APIs are used.
"""
import argparse, base64, hashlib, json, pathlib, secrets, shlex, time
import serial

p=argparse.ArgumentParser()
p.add_argument('action',choices=['upload','download'])
p.add_argument('local');p.add_argument('remote')
p.add_argument('--port',default='/dev/ttyUSB0')
p.add_argument('--rate',type=int,default=45000)
p.add_argument('--log',required=True)
a=p.parse_args()
assert a.remote.startswith('/root/model-validation-last-20261102/')
assert '\n' not in a.remote and '\r' not in a.remote
local=pathlib.Path(a.local); token='SERIAL_'+secrets.token_hex(10)
raw=bytearray(); started=time.monotonic()

with serial.Serial(a.port,1500000,timeout=.1,write_timeout=5,
                   rtscts=False,dsrdtr=False,xonxoff=False,exclusive=True) as s:
 def read_until(marker,timeout):
  end=time.monotonic()+timeout; data=bytearray()
  while time.monotonic()<end:
   data.extend(s.read(max(1,min(65536,s.in_waiting))))
   if marker in data:
    raw.extend(data);return bytes(data)
  raw.extend(data);raise RuntimeError('serial deadline: '+marker.decode())
 def command(script):
  s.write(b'\x15');s.write(('sh -c '+shlex.quote(script)+'\r').encode());s.flush()
 if a.action=='upload':
  data=local.read_bytes(); sha=hashlib.sha256(data).hexdigest()
  target=shlex.quote(a.remote);part=shlex.quote(a.remote+'.part')
  directory=shlex.quote(str(pathlib.PurePosixPath(a.remote).parent))
  duration=int(len(data)/a.rate)+90
  script=(f'mkdir -p {directory}; saved=$(stty -g); '
    'trap \'stty "$saved"\' EXIT HUP INT TERM; '
    f'stty raw -echo -ixon -ixoff; printf "\\n{token}_READY\\n"; '
    f'timeout {duration} head -c {len(data)} > {part}; '
    'stty "$saved"; trap - EXIT HUP INT TERM; '
    f'actual=$(sha256sum {part}); actual=${{actual%% *}}; '
    f'if [ "$actual" = {sha} ]; then mv {part} {target}; '
    f'printf "\\n{token}_PASS\\n"; else printf "\\n{token}_FAIL:%s\\n" "$actual"; fi')
  command(script);read_until((token+'_READY\n').encode(),10)
  begin=time.monotonic();last=begin
  for offset in range(0,len(data),4096):
   chunk=data[offset:offset+4096]
   s.write(chunk)
   due=begin+(offset+len(chunk))/a.rate
   if due>time.monotonic():time.sleep(due-time.monotonic())
   if time.monotonic()-last>=20:
    print(json.dumps({'sent':offset+len(chunk),'total':len(data)}),flush=True);last=time.monotonic()
  s.flush();reply=read_until((token+'_PASS').encode(),30)
  report={'action':'upload','remote':a.remote,'bytes':len(data),'sha256':sha,'verified':True}
 else:
  script=f'printf "\\n{token}_BEGIN\\n"; base64 {shlex.quote(a.remote)}; printf "\\n{token}_END\\n"'
  command(script);reply=read_until(('\n'+token+'_END').encode(),60)
  normalized=reply.replace(b'\r',b'')
  content=normalized.split(('\n'+token+'_BEGIN\n').encode(),1)[1].split(('\n'+token+'_END').encode(),1)[0]
  data=base64.b64decode(b''.join(content.split()),validate=True)
  local.write_bytes(data)
  report={'action':'download','remote':a.remote,'bytes':len(data),'sha256':hashlib.sha256(data).hexdigest()}
report['elapsed_seconds']=time.monotonic()-started
pathlib.Path(a.log).write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps(report),flush=True)
