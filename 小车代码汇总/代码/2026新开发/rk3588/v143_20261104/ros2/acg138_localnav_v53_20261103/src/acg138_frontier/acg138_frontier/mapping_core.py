"""Stopped-only mapping lifecycle. No velocity, path follower or motor API."""
from dataclasses import dataclass
from pathlib import Path
import copy, json, math, os

def mapping_start_ready(dry_run, explorer_ready, bridge_alive, state):
    """Service availability, not a grant of motion or a claim that SLAM is ready."""
    return not dry_run and explorer_ready and bridge_alive and state in ('IDLE','LOADED','ERROR')

def current_tf_pose(x, y, yaw, source_age_ns, now_monotonic_ns):
    """Current map->base TF, with its original age; never re-age a laser pose."""
    if not all(math.isfinite(v) for v in (x,y,yaw)) or not 0<=source_age_ns<=250000000 or now_monotonic_ns<source_age_ns:
        raise ValueError('fresh actual SLAM TF required')
    return {'xyyaw':[x,y,yaw],'source_monotonic_ns':str(now_monotonic_ns-source_age_ns)}

def slam_parameters(profile, mode, graph=None, pose=None):
    """Select SLAM's existing mapping/localization mode, without altering tuning."""
    if mode not in ('mapping','localization'):raise ValueError('invalid SLAM mode')
    result=copy.deepcopy(profile)
    p=result['slam_toolbox']['ros__parameters']
    p.update(mode=mode,map_name='/slam/map',paused_new_measurements=False)
    if mode=='localization':
        if not graph or pose is None or len(pose)!=3 or not all(math.isfinite(v) for v in pose):
            raise ValueError('saved graph and actual finite pose required')
        p.update(map_file_name=str(graph),map_start_pose=list(pose),map_start_at_dock=False)
    return result

class RequestLedger:
    """Eight idempotent replies; replay never renews a source deadline."""
    def __init__(self):
        self.last_id=0
        self.cache={}

    def begin(self, operation, request_id, generation, produced_ns, ttl_ms, now_ns):
        key=(operation,generation,produced_ns,ttl_ms)
        if request_id in self.cache:
            old,result=self.cache[request_id]
            if old!=key:raise ValueError('request ID reused with different fields')
            return result
        if request_id<=self.last_id or generation<=0:
            raise ValueError('old mapping request/generation')
        if not 0<ttl_ms<=500 or not 0<=now_ns-produced_ns<ttl_ms*1000000:
            raise ValueError('expired mapping request')
        self.last_id=request_id
        self.cache[request_id]=(key,None)
        while len(self.cache)>8:del self.cache[next(iter(self.cache))]
        return None

    def finish(self, request_id, success, status, message):
        key,_=self.cache[request_id]
        self.cache[request_id]=(key,(success,status,message))

@dataclass
class MappingSession:
    state: str = 'IDLE'
    generation: int = 0
    deadline_ms: int = 0
    reason: str = ''
    fixed_stamp_ns: int = 0
    expected_session: int = 0
    loaded_session: int = 0
    loaded_revision: int = 0
    loaded_crc32: int = 0

    def start(self, now_ms, stopped):
        if not stopped or self.state not in ('IDLE','LOADED','ERROR'):
            raise ValueError('start requires a stopped vehicle and no active mapping task')
        self.generation += 1
        self.state='STARTING';self.reason='waiting_for_new_SLAM_map_and_fresh_TF'
        self.deadline_ms=now_ms+30000;self.fixed_stamp_ns=0
        self.loaded_session=self.loaded_revision=self.loaded_crc32=0
        return self.generation

    def ready(self, generation, now_ms, map_ready, tf_age_ms):
        if generation!=self.generation or self.state!='STARTING':return False
        if not map_ready or not 0<=tf_age_ms<=250:return False
        self.state='EXPLORING';self.reason='RK_frontier_via_manual_channel';self.deadline_ms=0
        return True

    def stop(self, now_ms):
        # Invalidates an in-flight start and every old process/service callback.
        self.generation+=1;self.state='STOPPING';self.reason='waiting_for_FPGA_cancel_and_stop'
        self.deadline_ms=now_ms+10000
        return self.generation

    def stopped(self, generation, now_ms, confirmed):
        if generation!=self.generation or self.state!='STOPPING' or not confirmed:return False
        self.state='SAVING';self.reason='pause_and_serialize_pose_graph';self.deadline_ms=now_ms+30000;return True

    def saved(self, generation, now_ms, stamp_ns, posegraph_verified, map_verified):
        if generation!=self.generation or self.state!='SAVING':return False
        if stamp_ns<=0 or not posegraph_verified or not map_verified:return False
        self.fixed_stamp_ns=stamp_ns;self.state='LOCALIZING';self.reason='rebind_saved_graph_and_fresh_TF';self.deadline_ms=now_ms+30000;return True

    def localized(self, generation, now_ms, session, tf_age_ms, stopped):
        if generation!=self.generation or self.state!='LOCALIZING':return False
        if session<=0 or not stopped or not 0<=tf_age_ms<=250:return False
        self.expected_session=session;self.state='LOADING';self.reason='waiting_for_actual_COMMIT_and_CRC';self.deadline_ms=now_ms+15000;return True

    def committed(self, generation, metadata, fpga_state):
        if generation!=self.generation or self.state!='LOADING':return False
        try:
            valid=(metadata['source_map_stamp_ns']==self.fixed_stamp_ns
                and metadata['uploaded'] and metadata['state_crc_verified'] and not metadata['dry_run']
                and metadata['map_load_state']=='FROZEN' and not metadata.get('upload_error')
                and fpga_state['session_id']==self.expected_session
                and fpga_state['map_revision']==metadata['map_revision']
                and fpga_state['map_epoch']==metadata['map_epoch']
                and fpga_state['active_crc32']==metadata['bitmap_crc32']
                and fpga_state['available'] and not fpga_state['state_stale'] and not fpga_state['stream_fault']
                and fpga_state['host_receipt_age_ms']+fpga_state['pose_age_ms']<=250)
            if not valid:return False
            self.loaded_session=self.expected_session;self.loaded_revision=metadata['map_revision'];self.loaded_crc32=metadata['bitmap_crc32']
            self.state='LOADED';self.reason='map_committed_to_FPGA';self.deadline_ms=0;return True
        except (KeyError,TypeError,ValueError):return False

    def fail(self, reason):
        self.generation+=1;self.state='ERROR';self.reason=str(reason);self.deadline_ms=0

    def tick(self, now_ms):
        if self.deadline_ms and now_ms>=self.deadline_ms:self.fail('mapping_stage_timeout')

def save_grid(directory, grid):
    """Atomic immutable PGM/YAML plus lossless original occupancy values."""
    grid.validate()
    path=Path(directory);path.mkdir(mode=0o700,parents=True,exist_ok=False)
    # map_server trinary conventions: 0=occupied, 254=free, 205=unknown.
    pixels=bytearray()
    for y in range(grid.height-1,-1,-1):
        pixels.extend(205 if v<0 else 254 if v<=25 else 0 if v>=65 else 205
                      for v in grid.cells[y*grid.width:(y+1)*grid.width])
    files={
        'map.pgm':f'P5\n{grid.width} {grid.height}\n255\n'.encode()+pixels,
        'map.yaml':(f'image: map.pgm\nmode: trinary\nresolution: {grid.resolution:.12g}\n'
                    f'origin: [{grid.origin_x:.12g}, {grid.origin_y:.12g}, {grid.yaw:.12g}]\n'
                    'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n').encode(),
        'map.json':json.dumps({'width':grid.width,'height':grid.height,'resolution':grid.resolution,
                    'origin':[grid.origin_x,grid.origin_y,grid.yaw],'frame':grid.frame,'cells':list(grid.cells)},allow_nan=False,separators=(',',':')).encode()}
    for name,data in files.items():
        with open(path/(name+'.tmp'),'xb') as f:f.write(data);f.flush();os.fsync(f.fileno())
        os.replace(path/(name+'.tmp'),path/name)
    return path
