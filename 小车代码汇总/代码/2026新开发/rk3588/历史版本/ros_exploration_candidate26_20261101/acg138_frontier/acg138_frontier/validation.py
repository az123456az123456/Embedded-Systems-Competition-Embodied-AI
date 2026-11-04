import binascii
import math
from .core import Grid,Identity

def parse_identity(v):
    values=[v[k] for k in ('session_id','run_generation','plan_id','map_epoch','map_revision','localization_epoch','nav_epoch')]
    if any(isinstance(x,bool) or not isinstance(x,int) or not 0<=x<2**64 for x in values):raise ValueError('invalid identity integer')
    goal=v['goal_id']
    if not isinstance(goal,str) or len(goal)!=32 or len(bytes.fromhex(goal))!=16:raise ValueError('invalid goal ID')
    return Identity(v['session_id'],v['run_generation'],goal.lower(),v['plan_id'],v['map_epoch'],v['map_revision'],v['localization_epoch'],v['nav_epoch'])

def local_from_ros(m):
    q=m.info.origin.orientation
    if abs(q.x)>1e-6 or abs(q.y)>1e-6 or abs(q.z)>1e-6 or abs(abs(q.w)-1)>1e-6:raise ValueError('local map rotation')
    g=Grid(m.info.width,m.info.height,m.info.resolution,m.info.origin.position.x,m.info.origin.position.y,tuple(m.data),0.,m.header.frame_id);g.validate()
    if g.width!=64 or g.height!=64 or abs(g.resolution-.1)>1e-6 or any(v not in (0,100) for v in g.cells):raise ValueError('not candidate26 costmap')
    return g

def bitmap_crc(g):
    b=bytearray(512)
    for i,v in enumerate(g.cells):
        if v!=0:b[i//8]|=1<<(i%8)
    return binascii.crc32(b)&0xffffffff

def metadata_matches(g,m,s):
    try:
        if g is None or not m['uploaded'] or not m['state_crc_verified'] or m['dry_run'] or m['map_load_state']!='FROZEN':return False
        if m.get('upload_error') or s.get('state_stale') or s.get('stream_fault') or not s.get('available'):return False
        if not (s.get('flags',0)&32) or not 0<=s['host_receipt_age_ms']<=250 or not 0<=s['pose_age_ms']<=250 or s['host_receipt_age_ms']+s['pose_age_ms']>250 or s['map_epoch']!=m['map_epoch'] or s['map_revision']!=m['map_revision']:return False
        if g.frame!=m['frame_id'] or abs(g.resolution-m['resolution_m'])>1e-6 or abs(g.origin_x-m['origin_m'][0])>1e-6 or abs(g.origin_y-m['origin_m'][1])>1e-6:return False
        return bitmap_crc(g)==m['bitmap_crc32']==s['active_crc32']
    except (KeyError,TypeError,ValueError,OverflowError):return False
