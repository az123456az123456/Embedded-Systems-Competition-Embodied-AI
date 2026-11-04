#!/usr/bin/env python3
"""Read captured Ethernet evidence; never opens a socket or sends a frame."""
from pathlib import Path
import argparse,binascii,json,struct
p=argparse.ArgumentParser();p.add_argument('pcap');p.add_argument('--out',required=True);a=p.parse_args()
b=Path(a.pcap).read_bytes();out=Path(a.out);out.mkdir(parents=True,exist_ok=True)
if b[:4]==b'\xd4\xc3\xb2\xa1': endian='<';scale=1000000
elif b[:4]==b'\xa1\xb2\xc3\xd4': endian='>';scale=1000000
elif b[:4]==b'\x4d\x3c\xb2\xa1': endian='<';scale=1000000000
else:raise SystemExit('unsupported pcap header')
if struct.unpack_from(endian+'I',b,20)[0]!=1:raise SystemExit('expected Ethernet DLT')
fpga=bytes.fromhex('02ac13800001');rows=[];cursor=24;frame=0
while cursor<len(b):
    if len(b)-cursor<16:raise SystemExit('truncated pcap record')
    sec,frac,n,original=struct.unpack_from(endian+'IIII',b,cursor);cursor+=16
    data=b[cursor:cursor+n];cursor+=n;frame+=1
    if len(data)!=n:raise SystemExit('truncated packet')
    row={'frame_id':frame,'timestamp_ns':sec*1000000000+frac*(1000000000//scale),
         'captured_length':n,'original_length':original,'frame_hex':data.hex()}
    if n<14 or data[12:14]!=bytes.fromhex('88b9'):row['error']='not NAV3 EtherType';rows.append(row);continue
    row['direction']='TX' if data[:6]==fpga else 'RX' if data[6:12]==fpga else 'OTHER'
    row['src']=data[6:12].hex(':');row['dst']=data[:6].hex(':')
    if n!=272 or original!=272:row['error']='wrong Ethernet length';rows.append(row);continue
    w=data[14:];row['magic']=w[:4].decode('ascii','replace');row['version']=w[4];row['op']=w[5]
    u16=lambda off:int.from_bytes(w[off:off+2],'big')
    u64=lambda off:int.from_bytes(w[off:off+8],'big')
    row.update(seq=u64(8),session=u64(16),run=u64(24),goal=w[32:48].hex(),map_epoch=u64(48),map_revision=u64(56),localization_epoch=u64(64),plan=u64(72),request=u64(80),payload_bytes=u16(88))
    row['crc16_computed']=binascii.crc_hqx(w[:256],0xffff);row['crc16_received']=u16(256)
    payload_bytes=row['payload_bytes']
    row['wire_valid']=row['magic']=='NAV3' and row['version']==1 and row['crc16_computed']==row['crc16_received'] and u16(6)==0 and not any(w[90:96]) and payload_bytes<=160 and not any(w[96+payload_bytes:256])
    if row['op']&128 and payload_bytes>=32:
        q=w[96:];row.update(status=int.from_bytes(q[:2],'big'),device_ms=int.from_bytes(q[4:8],'big'),nav_epoch=int.from_bytes(q[8:16],'big'),state_seq=int.from_bytes(q[16:24],'big'))
        if row['op']==0x81 and payload_bytes==64:
            row.update(capabilities=int.from_bytes(q[32:36],'big'),cpu_hz=int.from_bytes(q[36:40],'big'),info_cookie=int.from_bytes(q[40:48],'big'),current_session=int.from_bytes(q[48:56],'big'),active_crc32=int.from_bytes(q[56:60],'big'),ready=int.from_bytes(q[60:64],'big'))
    rows.append(row)
keys=['seq','session','run','goal','map_epoch','map_revision','localization_epoch','plan','request']
tx=[r for r in rows if r['direction']=='TX' and r.get('wire_valid')]
rx=[r for r in rows if r['direction']=='RX' and r.get('wire_valid')]
pairs=[]
for request in tx:
    matches=[r for r in rx if r['op']==(request['op']|128) and all(r[k]==request[k] for k in keys) and r['timestamp_ns']>=request['timestamp_ns']]
    ack=matches[0] if matches else None
    pairs.append({'tx_frame':request['frame_id'],'tx_op':request['op'],'request_id':request['request'],
       'rx_frame':ack['frame_id'] if ack else None,'status':ack.get('status') if ack else None,
       'rtt_ns':ack['timestamp_ns']-request['timestamp_ns'] if ack else None,
       'pass':bool(ack and ack['status']==0 and ack['payload_bytes']==(64 if request['op']==1 else 32))})
summary={'stage':1,'source_pcap':str(Path(a.pcap).resolve()),'frames':len(rows),'tx_ops':[r['op'] for r in tx],
 'pairs':pairs,'sent_start_or_nva1':any(r.get('op')==0x21 and r['direction']=='TX' for r in rows),
 'pass':len(tx)==5 and [r['op'] for r in tx]==[1,3,2,1,4] and all(p['pass'] for p in pairs),
 'note':'Capture is NAV3 EtherType only; NVA1 exclusion also relies on source/transport audit.'}
(out/'frames.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in rows))
(out/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
print(json.dumps(summary,indent=2));raise SystemExit(0 if summary['pass'] else 1)
