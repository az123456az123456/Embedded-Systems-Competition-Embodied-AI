"""Receive-only SNS1 preflight. No send/sendto, motor or configuration API."""
import binascii,json,socket,time
s=socket.socket(socket.AF_PACKET,socket.SOCK_RAW,socket.htons(0x88b8))
s.bind(('eth0',0));s.settimeout(.5)
rows=[];end=time.monotonic()+4
while time.monotonic()<end and len(rows)<20:
    try:data=s.recv(4096)
    except TimeoutError:continue
    p=data[14:80]
    if len(p)!=66 or p[:5]!=b'SNS1\x01':continue
    if binascii.crc_hqx(p[:64],0xffff)!=int.from_bytes(p[64:66],'big'):continue
    rows.append({'sequence':int.from_bytes(p[8:12],'big'),'illegal_mask':p[62]})
s.close()
passed=len(rows)==20 and all(r['illegal_mask']==0 for r in rows)
print(json.dumps({'case':'SNS1_encoder_lock_after_RAM_reload','pass':passed,'read_only':True,'packets_sent':0,'samples':rows}),flush=True)
raise SystemExit(0 if passed else 1)
