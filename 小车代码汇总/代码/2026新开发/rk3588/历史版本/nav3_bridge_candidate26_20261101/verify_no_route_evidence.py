#!/usr/bin/env python3
"""Offline correlation of actual POSE / GOAL / NO_ROUTE / ROS evidence.

Input frames.jsonl is produced by verify_map_pcap.py from the original PCAP.
This is a failure-path check, never the nonempty-path stage-3 acceptance gate.
"""
import argparse
import json
from pathlib import Path

p = argparse.ArgumentParser()
p.add_argument('frames')
p.add_argument('observer')
p.add_argument('--out', required=True)
a = p.parse_args()
rows = [json.loads(s) for s in Path(a.frames).read_text().splitlines()]
observed = json.loads(Path(a.observer).read_text())
tx = [r for r in rows if r.get('direction') == 'TX' and r.get('wire_valid')]
rx = [r for r in rows if r.get('direction') == 'RX' and r.get('wire_valid')]
payload = lambda r: bytes.fromhex(r['frame_hex'])[110:270]
number = lambda r, off, size: int.from_bytes(payload(r)[off:off+size], 'big')
identity = ('session', 'run', 'goal', 'map_epoch', 'map_revision', 'localization_epoch')
ack_identity = identity + ('seq', 'plan', 'request')
goals = [r for r in tx if r['op'] == 0x20]
cases = []
for g in goals:
    pose_seq = number(g, 12, 8)
    poses = [r for r in tx if r['op'] == 0x30 and r['session'] == g['session']
             and r['map_epoch'] == g['map_epoch'] and r['map_revision'] == g['map_revision']
             and number(r, 16, 8) == pose_seq and r['frame_id'] < g['frame_id']]
    pose_states = [r for r in rx if poses and r['op'] == 0x40
                   and r['session'] == g['session'] and r['map_epoch'] == g['map_epoch']
                   and r['map_revision'] == g['map_revision']
                   and number(r, 24, 8) == pose_seq and number(r, 4, 4) & 32
                   and poses[0]['frame_id'] < r['frame_id'] < g['frame_id']]
    ack = [r for r in rx if r['op'] == 0xa0 and r['frame_id'] > g['frame_id']
           and all(r[k] == g[k] for k in ack_identity)]
    results = [r for r in rx if r['op'] == 0x43 and r['frame_id'] > g['frame_id']
               and all(r[k] == g[k] for k in identity) and number(r, 24, 8) == pose_seq]
    accepted = bool(ack and ack[0]['status'] == 0)
    no_route = bool(results and all(number(r, 0, 2) == 9 and number(r, 40, 2) == 0
                                    and number(r, 46, 2) == 0 for r in results))
    cases.append({'goal_tx_frame': g['frame_id'],
                  'identity': {k: g[k] for k in identity}, 'pose_seq': pose_seq,
                  'pose_tx_frames': [r['frame_id'] for r in poses],
                  'pose_fresh_state_frames': [r['frame_id'] for r in pose_states],
                  'goal_ack_frames': [r['frame_id'] for r in ack],
                  'plan_result_frames': [r['frame_id'] for r in results],
                  'goal_accepted': accepted, 'no_route': no_route,
                  'pass': bool(poses and pose_states and accepted and no_route)})
motion = [r['frame_id'] for r in rows if r.get('direction') == 'TX'
          and (r.get('family') == 'NVA1' or r.get('op') == 0x21)]
metadata = observed.get('metadata') or {}
ros_match = bool(observed.get('goal') and observed.get('path') is None
                 and metadata.get('nav3_plan_status') == 9 and metadata.get('path_cells') == 0)
summary = {'scope': 'stage3 actual NO_ROUTE failure path only',
           'frames_file': str(Path(a.frames).resolve()),
           'observer_file': str(Path(a.observer).resolve()),
           'cases': cases, 'ros_no_route_empty_path': ros_match,
           'forbidden_motion_frames': motion,
           'path_page_frames': [r['frame_id'] for r in tx if r['op'] == 0x41],
           'pass': bool(cases) and all(c['pass'] for c in cases) and ros_match and not motion,
           'nonempty_path_gate': 'BLOCKED',
           'note': 'NO_ROUTE correctness does not pass the required nonempty PATH_PAGE/ROS path gate.'}
Path(a.out).write_text(json.dumps(summary, indent=2) + '\n')
print(json.dumps(summary, indent=2))
raise SystemExit(0 if summary['pass'] else 1)
