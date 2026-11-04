#!/usr/bin/env python3
from pathlib import Path
import binascii,json,struct
base=Path(__file__).resolve().parents[2]
vectors=[];rid=0
mac=bytes.fromhex('02ac13800001');host=bytes.fromhex('02ac13800002')
def packet(op,gen=1,a=0,b=0,count=0,data=b'',status=0,bad=None):
 global rid
 if op!=6:rid+=1
 q=b'NAV2'+bytes([1,op,0,count])+struct.pack('>6I',rid if op!=6 else 0,gen,a,b,0,0)+data.ljust(32,b'\0')
 raw=mac+host+b'\x88\xb9'+q+struct.pack('>H',binascii.crc_hqx(q,65535))
 raw+=struct.pack('<I',binascii.crc32(raw))
 if bad=='fcs':raw=raw[:-1]+bytes([raw[-1]^1])
 vectors.append({'frame':(b'\x55'*7+b'\xd5'+raw).hex(),'q':q.hex(),'status':status,'reply':bad is None})
 return q
packet(6,gen=0)
packet(1,a=binascii.crc32(bytes(512)))
packet(3,status=4)
for offset in range(0,512,32):packet(2,a=offset,count=32)
packet(3)
q=packet(4,b=4095);plan=rid
vectors.append(dict(vectors[-1]))
for offset in range(0,127,16):packet(5,a=offset,b=plan,count=16)
packet(5,a=127,b=plan,count=16,status=1)
packet(6,gen=0,bad='fcs')
packet(6,gen=0)
packet(4,a=4096,b=4095,status=1)
packet(1,gen=2,a=binascii.crc32(bytes([255])*512))
for offset in range(0,512,32):packet(2,gen=2,a=offset,count=32,data=bytes([255])*32)
packet(3,gen=2)
packet(4,gen=2,b=4095,status=7)
packet(5,gen=1,a=0,b=plan,count=16,status=3)
packet(6,gen=0)
(base/'build').mkdir(exist_ok=True)
(base/'build/nav_vectors.hex').write_text('\n'.join(v['frame'] for v in vectors)+'\n')
(base/'build/nav_expect.hex').write_text('\n'.join('1' if v['reply'] else '0' for v in vectors)+'\n')
(base/'build/nav_vectors.json').write_text(json.dumps(vectors,indent=2))
(base/'build/nav_count.vh').write_text('`define NAV_VECTORS %d\n'%len(vectors))
print('Generated',len(vectors),'Ethernet vectors')
