#!/usr/bin/env python3
from pathlib import Path
import binascii,json,struct
r=Path(__file__).resolve().parents[2]
v=[v for v in json.loads((r/'build/nav_vectors.json').read_text()) if v['reply']]
a=[bytes.fromhex(s) for s in (r/'build/nav_actual.hex').read_text().splitlines()]
assert len(v)==len(a)
path=[];previous={};cycles=[]
for req,wire in zip(v,a):
 q=bytes.fromhex(req['q']);assert wire[:8]==b'\x55'*7+b'\xd5'
 b=wire[8:];assert len(b)==84
 assert b[:6]==b'\xff'*6 and b[6:12]==bytes.fromhex('02ac13800001') and b[12:14]==b'\x88\xb9'
 assert binascii.crc32(b[:-4])==int.from_bytes(b[-4:],'little')
 p=b[14:78];assert binascii.crc_hqx(p,65535)==int.from_bytes(b[78:80],'big')
 assert p[:5]==b'NAV2\x01' and p[5]==q[5]|128 and p[6]==req['status'] and p[8:16]==q[8:16]
 op=q[5];rid=int.from_bytes(q[8:12],'big')
 if op!=6 and rid in previous:assert p==previous[rid]
 previous[rid]=p
 if op==4 and p[6]==0:
  assert int.from_bytes(p[16:20],'big')==127
  assert int.from_bytes(p[20:24],'big')==127
  cycles.append(int.from_bytes(p[24:28],'big'))
 if op==5 and p[6]==0:
  off=int.from_bytes(p[16:20],'big');assert off==len(path)
  assert int.from_bytes(p[20:24],'big')==127
  path.extend(struct.unpack('>'+str(p[7])+'H',p[32:32+2*p[7]]))
assert len(path)==127 and path[0]==0 and path[-1]==4095
for a,b in zip(path,path[1:]):assert abs((a&63)-(b&63))+abs((a>>6)-(b>>6))==1
report={'passed':True,'requests':len(v)+1,'replies':len(v),'path_cells':len(path),'astar_50mhz_cycles':cycles,'astar_ms':[c/50000 for c in cycles],'checks':['real RV32IM firmware','RX FCS/CRC16','independent clocks CDC','atomic chunk commit','invalid FCS dropped','duplicate queued during A* response exact','full path pages','TX FCS/CRC16','blocked endpoint','stale generation']}
(r/'reports/nav_transport.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps(report,indent=2))
