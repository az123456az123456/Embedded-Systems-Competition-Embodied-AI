#!/usr/bin/env python3
"""Verify explicit-load ordering from ROS events + same-host captured Ethernet."""
import argparse,json,hashlib
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('frames');p.add_argument('observer');p.add_argument('--out',required=True);a=p.parse_args()
rows=[json.loads(s) for s in Path(a.frames).read_text().splitlines()]
o=json.loads(Path(a.observer).read_text());events={e['event']:e for e in o['events']};checks=[]
def check(name,value,**evidence):checks.append(dict(case=name,pass_=bool(value),**evidence))
tx=[r for r in rows if r.get('direction')=='TX' and r.get('wire_valid')]
rx=[r for r in rows if r.get('direction')=='RX' and r.get('wire_valid')]
load=events.get('load_request',{});pause=events.get('pause_response',{});done=events.get('second_load_response',{})
writes=[r for r in tx if r['op'] in [0x10,0x11,0x12]]
check('paused_mapping_before_load',pause.get('status') is True and pause.get('ros_ns',float('inf'))<load.get('ros_ns',0),pause=pause,load=load)
check('zero_map_writes_before_explicit_load',bool(load) and not any(r['timestamp_ns']<load['ros_ns'] for r in writes),first_write=writes[0]['frame_id'] if writes else None)
ops=[r['op'] for r in writes]
check('single_begin_16_chunks_commit',ops==[0x10]+[0x11]*16+[0x12],observed_ops=ops)
check('request_queued_and_second_request_rejected',events.get('load_response',{}).get('success') is True and done.get('success') is False,first=events.get('load_response'),second=done)
metas=o.get('metadata_history',[]);frozen=[m for m in metas if m.get('map_load_state')=='FROZEN']
check('frozen_metadata_present',len(frozen)>1,count=len(frozen))
key=lambda m:(m['bitmap_crc32'],tuple(m['origin_m']),m['map_revision'],m['source_map_stamp_ns'])
check('bitmap_geometry_revision_source_stamp_remain_frozen',bool(frozen) and len(set(key(m) for m in frozen))==1,keys=[list(key(m)) for m in frozen[:1]])
check('later_slam_publications_not_uploaded',bool(frozen) and max(m.get('source_updates_after_request',0) for m in frozen)>0,source_updates=max((m.get('source_updates_after_request',0) for m in frozen),default=0))
check('no_busy_map_rejections',not any(r['op'] in [0x90,0x91,0x92] and r.get('status')!=0 for r in rx))
paths=o.get('path_events',[]);first=next((i for i,e in enumerate(paths) if e['count']>0),None)
check('nonempty_path_not_cleared_after_publication',first is not None and all(e['count']>0 for e in paths[first:]),path_events=paths)
check('no_start_nva1_transmitted',not any(r.get('direction')=='TX' and (r.get('family')=='NVA1' or r.get('op')==0x21) for r in rows))
states=[r for r in rx if r.get('op')==0x40];payload=lambda r:bytes.fromhex(r['frame_hex'])[110:270]
check('no_fpga_navigation_owner',bool(states) and all(int.from_bytes(payload(r)[124:128],'big')==0 for r in states),state_count=len(states))
check('observer_completed',o.get('final_phase')=='done' and not o.get('exception'),final_phase=o.get('final_phase'),exception=o.get('exception'))
s={'pass':all(c['pass_'] for c in checks),'checks':[{('pass' if k=='pass_' else k):v for k,v in c.items()} for c in checks],'sources':{str(Path(p).resolve()):hashlib.sha256(Path(p).read_bytes()).hexdigest() for p in [a.frames,a.observer]},'scope':'Physical stationary planning only. Exact-map freezing and explicit load. No START, NVA1 or vehicle-motion test.'}
Path(a.out).write_text(json.dumps(s,indent=2)+'\n');print(json.dumps({'pass':s['pass'],'checks':[(c['case'],c['pass']) for c in s['checks']]}));raise SystemExit(0 if s['pass'] else 1)
