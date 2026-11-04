"""Frontier selection only. No velocities, tracking, obstacle manoeuvres or wire I/O.
The local grid MUST be the bitmap acknowledged by candidate26, not /map cropped
again at the current pose. Distances below rank goals; FPGA remains the planner.
"""
from dataclasses import dataclass, field
from collections import deque
from math import cos, sin, floor, hypot, isfinite
from enum import Enum

@dataclass(frozen=True)
class Grid:
    width: int
    height: int
    resolution: float
    origin_x: float
    origin_y: float
    cells: tuple
    yaw: float = 0.0
    frame: str = "map"

    def validate(self):
        if (not 0 < self.width <= 16384 or not 0 < self.height <= 16384
            or self.width*self.height > 1000000 or len(self.cells) != self.width*self.height
            or not all(isfinite(v) for v in (self.resolution,self.origin_x,self.origin_y,self.yaw))
            or not .005 <= self.resolution <= 1 or not self.frame
            or any(v < -1 or v > 100 for v in self.cells)):
            raise ValueError("invalid or oversized OccupancyGrid")

    def xy(self, i):
        x,y=(i%self.width+.5)*self.resolution,(i//self.width+.5)*self.resolution
        c,s=cos(self.yaw),sin(self.yaw)
        return self.origin_x+c*x-s*y,self.origin_y+s*x+c*y

    def cell(self, x, y):
        if not isfinite(x) or not isfinite(y): return None
        c,s=cos(self.yaw),sin(self.yaw); x,y=x-self.origin_x,y-self.origin_y
        ix,iy=floor((c*x+s*y)/self.resolution),floor((-s*x+c*y)/self.resolution)
        return iy*self.width+ix if 0<=ix<self.width and 0<=iy<self.height else None

    def neighbours(self, i):
        x,y=i%self.width,i//self.width
        if x: yield i-1
        if x+1<self.width: yield i+1
        if y: yield i-self.width
        if y+1<self.height: yield i+self.width

@dataclass(frozen=True)
class Target:
    x: float
    y: float
    local_cell: int
    frontier_key: tuple
    frontier_size: int
    intermediate: bool
    graph_distance: int

@dataclass(frozen=True)
class Selection:
    target: object
    reason: str
    clusters: int
    reachable_cells: int


def select_frontier(global_map, committed_local, robot_xy, excluded=(), min_cluster=3,
                    min_progress=.25, stand_off=.8):
    """Four-connected free/unknown frontiers; choose reachable stand-off cell.
    Boundary targets are *explicit intermediate goals*, never clamped frontier
    coordinates. Search space is the committed, inflated 64x64 bitmap. A global
    free-space flood-fill only rejects disconnected frontiers; no route is sent.
    """
    g,l=global_map,committed_local;g.validate();l.validate()
    if l.width!=64 or l.height!=64 or abs(l.resolution-.1)>1e-6 or abs(l.yaw)>1e-9 or l.frame!=g.frame:
        raise ValueError("local geometry does not match candidate26")
    if any(v not in (0,100) for v in l.cells):
        raise ValueError("local bitmap must contain only acknowledged free/blocked cells")
    start=l.cell(*robot_xy);gs=g.cell(*robot_xy)
    if start is None or gs is None or l.cells[start]!=0 or not 0<=g.cells[gs]<=25:
        return Selection(None,"BLOCKED_START",0,0)
    # Flood fill is connectivity/ranking, not a replacement for FPGA A*.
    reachable={start:0};queue=deque([start])
    while queue:
        i=queue.popleft()
        for n in l.neighbours(i):
            if n not in reachable and l.cells[n]==0:
                reachable[n]=reachable[i]+1;queue.append(n)
    known={gs};queue=deque([gs]);front=set()
    while queue:
        i=queue.popleft()
        for n in g.neighbours(i):
            if g.cells[n]==-1: front.add(i)
            elif 0<=g.cells[n]<=25 and n not in known: known.add(n);queue.append(n)
    clusters=[]
    while front:
        seed=min(front);front.remove(seed);cluster=[seed];queue=deque([seed])
        while queue:
            i=queue.popleft()
            for n in g.neighbours(i):
                if n in front:front.remove(n);queue.append(n);cluster.append(n)
        if len(cluster)>=min_cluster:clusters.append(cluster)
    candidates=[]
    for cluster in clusters:
        # A real member closest to the centroid; centroid may lie in a wall.
        points=[g.xy(i) for i in cluster]
        cx=sum(x for x,y in points)/len(points);cy=sum(y for x,y in points)/len(points)
        fx,fy=min(points,key=lambda p:((p[0]-cx)**2+(p[1]-cy)**2,p))
        key=(round(fx/.2),round(fy/.2))
        if key in excluded:continue
        outside=l.cell(fx,fy) is None
        initial=hypot(fx-robot_xy[0],fy-robot_xy[1])
        eligible=[]
        for i,d in reachable.items():
            x,y=l.xy(i)
            # Entire path was found in inflated local free cells. Also require
            # current global evidence free; a new obstacle must not be targeted.
            gi=g.cell(x,y)
            if gi is None or not 0<=g.cells[gi]<=25 or hypot(x-robot_xy[0],y-robot_xy[1])<min_progress:continue
            distance=hypot(fx-x,fy-y)
            if initial-distance < min_progress:continue
            if not outside and distance>stand_off:continue
            # Stay off the physical window edge. FPGA inflation already blocks
            # the boundary; this minimum inset protects reduced test bitmaps too.
            ix,iy=i%64,i//64
            if outside and min(ix,iy,63-ix,63-iy)<3:continue
            eligible.append((distance,d,i,x,y))
        if not eligible:continue
        distance,d,i,x,y=min(eligible)
        # Deterministic score favours short reachable approaches and larger gain.
        score=d*.1 + distance*.2 - min(len(cluster)*g.resolution,4.)*.3
        candidates.append((score,i,Target(x,y,i,key,len(cluster),outside,d)))
    if not candidates:
        reason="NO_FRONTIER" if not clusters else "NO_REACHABLE_FRONTIER"
        return Selection(None,reason,len(clusters),len(reachable))
    return Selection(min(candidates,key=lambda t:(t[0],t[1]))[2],"TARGET",len(clusters),len(reachable))

class Mode(str,Enum):
    IDLE="IDLE"
    SELECTING="SELECTING"
    NAVIGATING="NAVIGATING"
    ARRIVED="ARRIVED"
    NO_FRONTIER="NO_FRONTIER"
    ERROR="ERROR"

@dataclass(frozen=True)
class Identity:
    session: int
    run: int
    goal: str
    plan: int
    map_epoch: int
    revision: int
    localization_epoch: int
    nav_epoch: int

@dataclass
class Explorer:
    """Mode machine. All asynchronous completions carry a local generation.
    CANCELLED/safety expiry NEVER mean permission to choose and start another
    goal. Only explicit NO_ROUTE may retry; late callbacks cannot restart.
    """
    mode: Mode=Mode.IDLE
    generation: int=0
    phase: str="IDLE"
    reason: str=""
    identity: object=None
    target: object=None
    excluded: set=field(default_factory=set)
    deadline_ms: int=0
    last_progress_ms: int=0
    last_state_seq: int=0
    last_state_received_ms: int=0
    last_now_ms: int=0
    known_cells: int=0
    retries: int=0

    def begin(self, now_ms, authorized):
        if not authorized:raise ValueError("explicit phone authority required")
        if self.mode not in (Mode.IDLE,Mode.NO_FRONTIER,Mode.ERROR):raise ValueError("already exploring")
        self.generation+=1;self.mode=Mode.SELECTING;self.phase="WAIT_STOPPED_MAP"
        self.reason="";self.identity=None;self.target=None;self.excluded.clear();self.retries=0
        self.last_progress_ms=now_ms;self.last_now_ms=now_ms;self.known_cells=0
        return self.generation

    def stop(self, reason="USER_STOP"):
        old=self.identity;self.generation+=1;self.mode=Mode.IDLE;self.phase="CANCEL_PENDING"
        self.reason=reason;self.identity=None;self.target=None
        return old

    def map_ready(self, generation, selection, stopped, crc_verified):
        if generation!=self.generation or self.mode!=Mode.SELECTING or self.phase!="WAIT_STOPPED_MAP":return False
        if not stopped or not crc_verified:return False
        if selection.target is None:
            self.mode=Mode.NO_FRONTIER if selection.reason=="NO_FRONTIER" else Mode.ERROR
            self.reason=selection.reason;self.phase="IDLE";return False
        self.target=selection.target;self.phase="WAIT_PLAN";return True

    def planned(self, generation, identity, ok):
        if generation!=self.generation or self.mode!=Mode.SELECTING or self.phase!="WAIT_PLAN":return False
        if not ok:
            if self.target:self.excluded.add(self.target.frontier_key)
            self.retries+=1;self.phase="WAIT_STOPPED_MAP";self.reason="NO_ROUTE"
            if self.retries>=8:self.mode=Mode.ERROR;self.reason="RETRY_LIMIT"
            return False
        self.identity=identity;self.phase="WAIT_START";return True

    def started(self, generation, identity, now_ms, accepted):
        if generation!=self.generation or identity!=self.identity or self.phase!="WAIT_START":return False
        if not accepted:self.mode=Mode.ERROR;self.reason="START_REJECTED";self.phase="CANCEL_PENDING";return False
        self.mode=Mode.NAVIGATING;self.phase="RUNNING";self.deadline_ms=now_ms+120000
        self.last_state_received_ms=now_ms;self.last_state_seq=0;return True

    def observe(self, identity, state, reason, state_seq, now_ms, source_age_ms):
        # state_seq argument is a fresh sample sequence (POSE sequence on the
        # existing ROS schema), not firmware state-transition counter.
        if self.mode!=Mode.NAVIGATING or self.identity is None or state_seq<=self.last_state_seq:return "IGNORE"
        keys=("session","run","goal","map_epoch","revision","localization_epoch")
        if any(getattr(identity,k)!=getattr(self.identity,k) for k in keys):return "IGNORE"
        if identity.plan<self.identity.plan or identity.nav_epoch<self.identity.nav_epoch:return "IGNORE"
        if identity.nav_epoch!=self.identity.nav_epoch and state not in ("ARRIVED","CANCELLED"):return "IGNORE"
        if state=="ARRIVED" and (reason!=13 or identity.nav_epoch>self.identity.nav_epoch+1):return "IGNORE"
        if source_age_ms<0 or source_age_ms>250:return "IGNORE"
        self.last_state_seq=state_seq;self.last_state_received_ms=now_ms
        if state=="ARRIVED":
            self.mode=Mode.ARRIVED;self.phase="WAIT_STOP_CONFIRM";return "CANCEL_AND_CONFIRM_STOP"
        if state=="CANCELLED":
            self.mode=Mode.ERROR;self.phase="CANCEL_PENDING";self.reason="FPGA_CANCELLED:"+str(reason)
            return "STOP"  # manual takeover/expired lease/reference reset never auto-resume
        if reason==3: # candidate26 NV_R_NO_ROUTE
            self.mode=Mode.ERROR;self.reason="NO_ROUTE";self.phase="RETRY_AFTER_STOP";return "CANCEL_AND_CONFIRM_STOP"
        return "WAIT"

    def stopped(self, generation, confirmed):
        if generation!=self.generation or not confirmed:return False
        retry=self.mode==Mode.ERROR and self.phase=="RETRY_AFTER_STOP"
        if self.mode!=Mode.ARRIVED and not retry:return False
        if self.target:self.excluded.add(self.target.frontier_key)
        if retry:self.retries+=1
        if self.retries>=8:self.mode=Mode.ERROR;self.reason="RETRY_LIMIT";return False
        self.identity=None;self.mode=Mode.SELECTING;self.phase="WAIT_STOPPED_MAP";return True

    def tick(self, now_ms, observed_cells):
        if self.mode not in (Mode.SELECTING,Mode.NAVIGATING,Mode.ARRIVED):return "WAIT"
        reason=""
        if now_ms<self.last_now_ms:reason="HOST_CLOCK_REVERSED"
        self.last_now_ms=now_ms
        if observed_cells>self.known_cells+10:
            self.known_cells=observed_cells;self.last_progress_ms=now_ms
        if now_ms-self.last_progress_ms>=300000:reason="EXPLORATION_NO_PROGRESS"
        if self.mode==Mode.NAVIGATING:
            if now_ms>=self.deadline_ms:reason="GOAL_TIMEOUT"
            if now_ms-self.last_state_received_ms>250:reason="STATE_STALE"
        if reason:
            self.mode=Mode.ERROR;self.phase="CANCEL_PENDING";self.reason=reason;return "STOP"
        return "WAIT"
