#!/usr/bin/env python3
"""Read-only Stage3 success evidence: exact IDs, real map, pages and ROS path."""
import argparse
import hashlib
import json
import math
import zlib
from pathlib import Path

p = argparse.ArgumentParser()
p.add_argument('frames')
p.add_argument('observer')
p.add_argument('--map-summary', required=True)
p.add_argument('--out', required=True)
a = p.parse_args()
rows = [json.loads(s) for s in Path(a.frames).read_text().splitlines()]
obs = json.loads(Path(a.observer).read_text())
maps = json.loads(Path(a.map_summary).read_text())
tx = [r for r in rows if r.get('direction') == 'TX' and r.get('wire_valid')]
rx = [r for r in rows if r.get('direction') == 'RX' and r.get('wire_valid')]
payload = lambda r: bytes.fromhex(r['frame_hex'])[110:270]
def u(r, off, size, signed=False):
    return int.from_bytes(payload(r)[off:off+size], 'big', signed=signed)
ids = ('session','run','goal','map_epoch','map_revision','localization_epoch')
ack_ids = ids + ('seq','plan','request')
def ack_for(req):
    return next((r for r in rx if r['op'] == (req['op'] | 128)
                 and r['frame_id'] > req['frame_id']
                 and all(r[k] == req[k] for k in ack_ids)), None)
grid = obs.get('goal_map') or obs.get('local_map')
meta = obs.get('goal_metadata') or obs.get('metadata') or {}
bitmap = bytearray(512)
if grid and len(grid['data']) == 4096:
    for i,v in enumerate(grid['data']):
        if v != 0: bitmap[i//8] |= 1 << (i%8)
checks = []
for g in [r for r in tx if r['op'] == 0x20]:
    q = {'goal_frame':g['frame_id'], 'identity':{k:g[k] for k in ids}, 'pass':False}
    checks.append(q)
    try:
        pose_seq = u(g,12,8)
        poses = [r for r in tx if r['op'] == 0x30 and r['session'] == g['session']
                 and r['map_epoch'] == g['map_epoch'] and r['map_revision'] == g['map_revision']
                 and u(r,16,8) == pose_seq and r['frame_id'] < g['frame_id']]
        assert len(poses) == 1, 'missing/ambiguous source POSE'
        pose = poses[0]
        states = [r for r in rx if r['op'] == 0x40 and r['session'] == g['session']
                  and r['map_epoch'] == g['map_epoch'] and r['map_revision'] == g['map_revision']
                  and pose['frame_id'] < r['frame_id'] < g['frame_id']
                  and u(r,24,8) == pose_seq and u(r,4,4) & 32 and u(r,44,4) <= 250]
        assert states, 'no exact fresh POSE STATE confirmation'
        ack = ack_for(g)
        assert ack and ack['status'] == 0, 'GOAL ACK missing or rejected'
        events = [r for r in rx if r['op'] == 0x43 and r['frame_id'] > g['frame_id']
                  and all(r[k] == g[k] for k in ids) and u(r,24,8) == pose_seq]
        assert events, 'no exact PLAN_RESULT'
        event = events[0]
        assert u(event,0,2) == 0, 'PLAN_RESULT status=' + str(u(event,0,2))
        assert event['plan'] > 0 and u(event,46,2) == 1, 'path validity flag or plan ID'
        count = u(event,40,2)
        assert 1 <= count <= 4096 and u(event,8,8) == ack['nav_epoch'], 'count/nav_epoch'
        # PoseStamped carries a world-coordinate goal, not a map revision. A newer
        # actual map may commit before the bridge handles it. Reconstruct the
        # exact GOAL revision from wire evidence, never substitute an adjacent map.
        transaction = next((t for t in maps['transactions'] if t['pass']
                            and t['session'] == g['session'] and t['revision'] == g['map_revision']), None)
        assert transaction, 'committed map proof missing'
        bitmap = bytearray(512); offsets = set()
        for chunk in [r for r in tx if r['op'] == 0x11 and r['session'] == g['session']
                      and r['map_epoch'] == g['map_epoch'] and r['map_revision'] == g['map_revision']
                      and u(r,0,8) == transaction['transaction'] and r['frame_id'] < g['frame_id']]:
            off,n = u(chunk,8,2),u(chunk,10,2)
            assert n == 32 and off <= 480 and off % 32 == 0, 'invalid map chunk'
            bitmap[off:off+32] = payload(chunk)[16:48]; offsets.add(off)
        crc = zlib.crc32(bitmap)
        assert len(offsets) == 16 and crc == transaction['crc32_recomputed'], 'exact map CRC'
        begins = [r for r in tx if r['op'] == 0x10 and r['session'] == g['session']
                  and r['map_revision'] == g['map_revision']]
        assert begins, 'map geometry absent'
        b = begins[-1]; ox,oy = u(b,4,4,True),u(b,8,4,True)
        def cell(x,y):
            assert 0 <= x-ox < 6400 and 0 <= y-oy < 6400, 'outside map'
            return ((y-oy)//100)*64 + (x-ox)//100
        start = cell(u(pose,0,4,True),u(pose,4,4,True))
        goal = cell(u(g,0,4,True),u(g,4,4,True))
        assert abs(u(g,0,4,True)/1000-obs['goal']['x']) <= .000501 and abs(u(g,4,4,True)/1000-obs['goal']['y']) <= .000501, 'different world-coordinate goal'
        assert start != goal, 'this acceptance requires distinct start and goal'
        assert (u(event,42,2),u(event,44,2)) == (start,goal), 'PLAN endpoints'
        assert u(event,52,4) == u(b,0,4), 'frame token'
        pages = [r for r in tx if r['op'] == 0x41 and r['frame_id'] > event['frame_id']
                 and all(r[k] == g[k] for k in ids) and r['plan'] == event['plan']]
        cells = []; page_proof = []
        for off in range(0,count,64):
            n = min(64,count-off)
            req = next((r for r in pages if u(r,0,2) == off and u(r,2,2) == n
                        and u(r,4,4) == u(event,48,4)), None)
            assert req, 'missing exact page request'
            reply = ack_for(req)
            assert reply and reply['status'] == 0 and reply['payload_bytes'] == 32+2*n, 'page ACK/size'
            assert (u(reply,24,2),u(reply,26,2)) == (off,n), 'page offset/count'
            cells.extend(u(reply,32+2*i,2) for i in range(n))
            page_proof.append({'offset':off,'count':n,'tx_frame':req['frame_id'],'rx_frame':reply['frame_id']})
        assert cells[0] == start and cells[-1] == goal and len(set(cells)) == count, 'path endpoints/duplicates'
        assert all(0 <= c < 4096 and not bitmap[c//8] & (1 << (c%8)) for c in cells), 'blocked/out-of-range cell'
        assert all(abs(x%64-y%64)+abs(x//64-y//64) == 1 for x,y in zip(cells,cells[1:])), 'not four-connected'
        expected = [[ox/1000+(c%64+.5)*.1,oy/1000+(c//64+.5)*.1] for c in cells]
        path = obs.get('path')
        assert path and path['frame'] == meta['frame_id'] and len(path['points']) == count, 'ROS path frame/size'
        assert all(math.isclose(x,a,abs_tol=1e-6) and math.isclose(y,b,abs_tol=1e-6)
                   for (x,y),(a,b) in zip(path['points'],expected)), 'ROS path differs from FPGA pages'
        q.update(pass_=True, pose_frame=pose['frame_id'], pose_seq=pose_seq,
                 pose_state_frames=[r['frame_id'] for r in states], goal_ack_frame=ack['frame_id'],
                 plan_result_frame=event['frame_id'], plan_id=event['plan'], path_count=count,
                 planned_revision=u(event,48,4), cells=cells, pages=page_proof,
                 cycles=u(event,36,4), cpu_hz=u(event,56,4), map_crc32=crc,
                 observer_selection_map_revision=meta.get('map_revision'),
                 actual_goal_map_revision=g['map_revision'])
        q['pass'] = q.pop('pass_')
    except (AssertionError,KeyError,IndexError,TypeError) as e:
        q['reason'] = str(e)
forbidden = [r['frame_id'] for r in rows if r.get('direction') == 'TX'
             and (r.get('family') == 'NVA1' or r.get('op') == 0x21)]
summary = {'stage':3,'source_sha256':{str(Path(f).resolve()):hashlib.sha256(Path(f).read_bytes()).hexdigest()
                                  for f in [a.frames,a.observer,a.map_summary]},
           'cases':checks,'forbidden_motion_frames':forbidden,
           'later_busy_map_begin_ack_frames':[r['frame_id'] for r in rx if r['op'] == 0x90 and r.get('status') == 10],
           'pass':any(c['pass'] for c in checks) and not forbidden,
           'scope':'Exact same-task nonempty FPGA path and ROS publication; no driving acceptance.'}
Path(a.out).write_text(json.dumps(summary,indent=2)+'\n')
print(json.dumps(summary,indent=2))
raise SystemExit(0 if summary['pass'] else 1)
