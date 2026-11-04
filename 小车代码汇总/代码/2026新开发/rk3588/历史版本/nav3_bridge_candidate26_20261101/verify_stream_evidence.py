#!/usr/bin/env python3
"""Offline same-session POSE/STATE trace audit; never opens a network socket."""
import argparse,collections,hashlib,json,statistics
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('frames');p.add_argument('observer');p.add_argument('--out',required=True);p.add_argument('--required-pose-hz',type=float,default=40,help='User accepted 40 Hz on 2026-11-01; scheduler remains nominal 50 Hz');a=p.parse_args()
rows=[json.loads(s) for s in Path(a.frames).read_text().splitlines()];obs=json.loads(Path(a.observer).read_text())
tx=[r for r in rows if r.get('direction')=='TX' and r.get('wire_valid')]
rx=[r for r in rows if r.get('direction')=='RX' and r.get('wire_valid')]
def u(r,o,n,signed=False):return int.from_bytes(bytes.fromhex(r['frame_hex'])[110+o:110+o+n],'big',signed=signed)
def dist(xs):return {'count':len(xs),'min':min(xs),'median':statistics.median(xs),'max':max(xs)} if xs else {'count':0}
def hz(times):return (len(times)-1)*1e9/(times[-1]-times[0]) if len(times)>1 else 0
poses=[r for r in tx if r['op']==0x30];states=[r for r in rx if r['op']==0x40];pubs=[r for r in obs['state_messages'] if r.get('available')]
checks=[]
def check(name,ok,**detail):checks.append(dict(case=name,pass_=bool(ok),**detail))
check('continuous_pose_observed',len(poses)>100)
check('independent_pose_transport_and_source_monotonic',all(b['seq']>c['seq'] and u(b,16,8)>u(c,16,8) and u(b,24,8)>u(c,24,8) for c,b in zip(poses,poses[1:])))
check('source_age_at_encode_not_refreshed',all(u(r,36,8)<=250000000 for r in poses),source_age_ns=dist([u(r,36,8) for r in poses]))
check('all_pose_use_acknowledged_anchor',all(any(t['op']==4 and t['session']==r['session'] and u(t,0,8)==u(r,48,8) and t['frame_id']<r['frame_id'] and any(k['op']==0x84 and k['request']==t['request'] and k['session']==t['session'] and k['frame_id']<r['frame_id'] and k.get('status')==0 for k in rx) for t in tx) for r in poses))
check('one_session_binding_no_auto_rebind',sum(r['op']==3 for r in tx)==1)
check('no_start_permit_or_owner',not any(r.get('direction')=='TX' and (r.get('family')=='NVA1' or r.get('op')==0x21) for r in rows) and all(u(r,124,4)==0 for r in states))
check('no_stream_fault_published',all(not r.get('stream_fault') for r in pubs))
# Match every published view to an exact actual FPGA STATE (not nearest time).
index=collections.defaultdict(list)
for r in states:index[(r['session'],u(r,16,8),u(r,24,8),u(r,120,4))].append(r)
names=['IDLE','PLANNING','FOLLOWING','AVOIDING','ARRIVED','CANCELLED']
fields={'avoid_stage':(1,1),'reason':(2,2),'flags':(4,4),'nav_epoch':(8,8),'pose_age_ms':(44,4),'waypoint_index':(48,2),'path_count':(50,2),'remaining_mm':(52,4),'permit_remaining_ms':(56,4),'authorization_remaining_ms':(60,4),'active_crc32':(64,4),'owner':(124,4),'frame_token':(144,4)}
bad=[]
for j,s in enumerate(pubs):
 candidates=index[(s['session_id'],s['state_seq'],s['pose_seq'],s['device_ms'])]
 def matches(r):
  return (u(r,0,1)<6 and names[u(r,0,1)]==s['state'] and all(u(r,*off)==s[k] for k,off in fields.items())
   and [u(r,32,4,True),u(r,36,4,True),u(r,40,4,True)]==s['pose_mm_urad']
   and all(s[k]==r[q] for k,q in [('run_generation','run'),('goal_id','goal'),('map_epoch','map_epoch'),('map_revision','map_revision'),('localization_epoch','localization_epoch'),('plan_id','plan')]))
 if not any(matches(r) for r in candidates):bad.append(j)
check('ros_state_fields_equal_exact_wire_state',bool(pubs) and not bad,mismatched_publication_indexes=bad)
sent={(r['session'],u(r,16,8)) for r in poses}
fresh=[r for r in states if (r['session'],u(r,24,8)) in sent and poses[0]['timestamp_ns']<=r['timestamp_ns']<=poses[-1]['timestamp_ns']]
check('fpga_observed_fresh_pose',bool(fresh) and all(u(r,44,4)<=250 and u(r,4,4)&32 for r in fresh),pose_age_ms=dist([u(r,44,4) for r in fresh]))
for c in checks:c['pass']=c.pop('pass_')
timing={'pose_count':len(poses),'pose_hz':hz([r['timestamp_ns'] for r in poses]),'pose_interval_ms':dist([(b['timestamp_ns']-c['timestamp_ns'])/1e6 for c,b in zip(poses,poses[1:])]),'ros_state_count':len(pubs),'ros_state_hz':hz([r['observed_ns'] for r in pubs]),'wire_state_count':len(states),'wire_state_hz':hz([r['timestamp_ns'] for r in states]),'target_pose_hz':50,'rate_is_measured_not_realtime_guarantee':True}
functional=all(c['pass'] for c in checks);rate_ok=timing['pose_hz']>=a.required_pose_hz
summary={'stage':4,'pass':functional and rate_ok,'functional_pass':functional,'accepted_pose_hz':a.required_pose_hz,'acceptance_basis':'User explicitly confirmed: 40hz足够 (2026-11-01). No firmware freshness/permission changes.','nominal_50hz_acceptance':'NOT_MET' if timing['pose_hz']<50 else 'OBSERVED','overall':'PASS' if functional and rate_ok else 'PARTIAL','checks':checks,'timing':timing,'path_observed_this_run':bool(obs.get('path')),'input_sha256':{str(Path(f).resolve()):hashlib.sha256(Path(f).read_bytes()).hexdigest() for f in [a.frames,a.observer]},'scope':'Static POSE/STATE, measured average over this capture and user-accepted rate; not a real-time rate guarantee. No motion, permission, physical timing or live v40 claim.'}
Path(a.out).write_text(json.dumps(summary,indent=2)+'\n');print(json.dumps(summary,indent=2));raise SystemExit(0 if summary['pass'] else 1)
