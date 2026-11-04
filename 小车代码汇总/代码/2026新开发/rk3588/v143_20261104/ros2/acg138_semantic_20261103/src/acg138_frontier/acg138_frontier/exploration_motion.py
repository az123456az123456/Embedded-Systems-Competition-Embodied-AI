"""RK-only frontier selection over the full SLAM grid. No FPGA wire API."""
import math,time
from collections import deque
from array import array

def straight_continuation(a,b,c):
    ux,uy=b[0]-a[0],b[1]-a[1];vx,vy=c[0]-b[0],c[1]-b[1]
    return ux*vx+uy*vy>0 and abs(ux*vy-uy*vx)<=1e-9

def tracking_target(path,cursor,pose):
    """Metric lookahead on ONE straight segment, never skip a corner.

    Fine grids put the first two centres only centimetres from the axle;
    aiming at them amplifies sub-cell lateral offsets into large yaw errors.
    Cursor ownership stays with the controller, so no unvisited path is lost.
    """
    i=cursor
    while (i+1<len(path) and i>0 and math.dist(path[i],pose[:2])<.08 and
           straight_continuation(path[i-1],path[i],path[i+1])):
        i+=1
    return path[i]

def scan_clearance(ranges,angle_min,angle_increment,range_min,range_max,yaw,tx,ty):
    """Use the full valid scan for live clearance; 3m limits map updates only.

    Do not turn range-filtered NaNs into free-space evidence, and do not count
    genuine invalid rays as healthy. Mapping has its own RangeFilter -> NaN.
    """
    front=circle=float('inf');seen=valid=front_valid=0
    for i,r in enumerate(ranges):
        a=angle_min+i*angle_increment+yaw
        ahead=abs(math.atan2(math.sin(a),math.cos(a)))<.65
        if ahead:seen+=1
        good=(math.isfinite(r) and range_min<=r<=range_max) or r==float('inf')
        if not good:continue
        valid+=1
        if ahead:front_valid+=1
        if not math.isfinite(r):continue
        x,y=tx+r*math.cos(a),ty+r*math.sin(a);circle=min(circle,math.hypot(x,y))
        if x>0 and abs(y)<.22:front=min(front,x)
    if valid<len(ranges)*.6 or seen<5 or front_valid<seen*.6 or abs(angle_increment)*len(ranges)<6.0:
        raise ValueError('lidar_coverage_incomplete')
    return front,circle

def footprint_cells(g, pose, width=.30, front=.1589, rear=.1942):
    """Positive-area overlap of grid squares with the oriented vehicle body.

    This is a target mask for inflation only, never evidence of observed free
    space. Length retains the existing vehicle profile; width is user-selected.
    """
    if not all(math.isfinite(v) and 0<v<=2 for v in (width,front,rear)):
        raise ValueError('invalid_body_geometry')
    w,h,res,ox,oy,yaw,_=g
    gc,gs=math.cos(yaw),math.sin(yaw);bc,bs=math.cos(pose[2]),math.sin(pose[2])
    p=[]
    for x,y in ((-rear,-width/2),(front,-width/2),(front,width/2),(-rear,width/2)):
        dx,dy=pose[0]+bc*x-bs*y-ox,pose[1]+bs*x+bc*y-oy
        p.append(((gc*dx+gs*dy)/res,(-gs*dx+gc*dy)/res))
    axes=((1,0),(0,1),(p[1][1]-p[0][1],p[0][0]-p[1][0]),
          (p[2][1]-p[1][1],p[1][0]-p[2][0]))
    spans=[(a,b,min(x*a+y*b for x,y in p),max(x*a+y*b for x,y in p),
            .5*(abs(a)+abs(b)),1e-9*math.hypot(a,b)) for a,b in axes]
    result=set()
    for y in range(max(0,math.floor(min(q[1] for q in p))),min(h,math.ceil(max(q[1] for q in p)))):
        for x in range(max(0,math.floor(min(q[0] for q in p))),min(w,math.ceil(max(q[0] for q in p)))):
            if all(hi>(x+.5)*a+(y+.5)*b-r+eps and lo<(x+.5)*a+(y+.5)*b+r-eps
                   for a,b,lo,hi,r,eps in spans):result.add(y*w+x)
    return result

def unobserved_body_cells(g,pose,width=.30,front=.1589,rear=.1942):
    """Unknown squares wholly inside the CURRENT body, not free-map evidence.

    The lidar cannot observe through the chassis under its own origin. These
    cells may seed/continue a local route, but remain UNKNOWN in the published
    map. Never exempt occupied cells or even partially external blind cells.
    Recompute at every pose: this is not a retained trail of cleared space.
    """
    w,_,res,ox,oy,yaw,cells=g
    candidates=footprint_cells(g,pose,width,front,rear)
    gc,gs=math.cos(yaw),math.sin(yaw);bc,bs=math.cos(pose[2]),math.sin(pose[2])
    delta=pose[2]-yaw
    half=res*.5*(abs(math.cos(delta))+abs(math.sin(delta)))
    result=set()
    for i in candidates:
        if cells[i]>=0:continue
        gx,gy=(i%w+.5)*res,(i//w+.5)*res
        dx,dy=ox+gc*gx-gs*gy-pose[0],oy+gs*gx+gc*gy-pose[1]
        bx,by=bc*dx+bs*dy,-bs*dx+bc*dy
        if bx-half>=-rear and bx+half<=front and abs(by)+half<=width/2:
            result.add(i)
    return result

def plan_frontier(g, pose, visited, radius=.15, body_width=.30,
                  body_front=.1589, body_rear=.1942, self_footprint_exemption=True, requested_goal=None):
    """Inflate occupied/unknown squares; choose a reachable frontier stand-off.

    Returns a four-connected path. External unknown space is never traversed. The worker
    has a wall-time bound; its failure cannot block the separate drive watchdog.
    """
    w,h,res,ox,oy,yaw,cells=g
    if w<=0 or h<=0 or w*h>1000000 or len(cells)!=w*h or not .02-1e-9<=res<=.25:
        raise ValueError('unsupported_map_geometry')
    if not all(math.isfinite(v) for v in (res,ox,oy,yaw,*pose)):
        raise ValueError('non_finite_map_or_pose')
    if not math.isfinite(radius) or not 0<=radius<=1:raise ValueError('invalid_inflation_radius')
    if any(v<-1 or v>100 for v in cells):raise ValueError('invalid_occupancy_value')
    footprint=footprint_cells(g,pose,body_width,body_front,body_rear)
    self_unknown=unobserved_body_cells(g,pose,body_width,body_front,body_rear) if self_footprint_exemption else set()
    if not self_footprint_exemption:footprint=set()
    deadline=time.monotonic()+4
    c,s=math.cos(yaw),math.sin(yaw)
    def check(i):
        if not (i&1023) and time.monotonic()>deadline:raise ValueError('frontier_planning_budget_exceeded')
    def adjacent(i):
        x,y=i%w,i//w
        if x:yield i-1
        if x+1<w:yield i+1
        if y:yield i-w
        if y+1<h:yield i+w
    free=bytearray(0<=v<=25 or i in self_unknown for i,v in enumerate(cells))
    safe=bytearray(free)
    n=math.ceil(radius/res+.5)
    offsets=[(dx,dy) for dy in range(-n,n+1) for dx in range(-n,n+1)
             if math.hypot(max(0,abs(dx)-.5)*res,max(0,abs(dy)-.5)*res)<=radius+1e-9]
    # Inflate obstacle boundaries only; interior blocked cells cannot add cells
    # beyond a boundary. Out-of-map space is also blocked.
    boundary=[]
    for i in range(w*h):
        check(i)
        if not free[i] and any(free[j] for j in adjacent(i)):boundary.append(i)
        if free[i] and min(i%w+.5,w-i%w-.5,i//w+.5,h-i//w-.5)*res<=radius:safe[i]=0
    # Same square-distance stencil, but each row is contiguous. Slice assignment
    # clears it in C rather than visiting every inflated target in Python.
    rows=[]
    for dy in range(-n,n+1):
        xs=[dx for dx,yy in offsets if yy==dy]
        if xs:rows.append((dy,min(xs),max(xs)))
    footprint_before=[(i,safe[i]) for i in footprint]
    zero_row=bytes(2*n+1)
    for k,i in enumerate(boundary):
        check(k);x,y=i%w,i//w
        for dy,lo,hi in rows:
            yy=y+dy
            if 0<=yy<h:
                left=max(0,x+lo);right=min(w,x+hi+1)
                safe[yy*w+left:yy*w+right]=zero_row[:right-left]
    # Restore local start values only; measured obstacles and map-edge
    # exclusions remain blocked. The original occupancy grid is never edited.
    for i,value in footprint_before:safe[i]=value
    px,py=pose[0]-ox,pose[1]-oy
    sx,sy=math.floor((c*px+s*py)/res),math.floor((-s*px+c*py)/res)
    if not (0<=sx<w and 0<=sy<h) or not safe[sy*w+sx]:raise ValueError('robot_cell_blocked_after_inflation')
    start=sy*w+sx;parent=array('i',[-1])*(w*h);distance=array('i',[-1])*(w*h)
    def world(i):
        x,y=(i%w+.5)*res,(i//w+.5)*res
        return (ox+c*x-s*y,oy+s*x+c*y)
    # Keep the first target the production controller will actually consume:
    # one <8cm cursor advance followed by metric lookahead on that straight run.
    # Reconstructing every full route to determine this would be unbounded work.
    initial_target=array('i',[-1])*(w*h);initial_target[start]=start
    parent[start]=start;distance[start]=0;q=deque([start]);k=0
    while q:
        i=q.popleft();k+=1;check(k)
        for j in adjacent(i):
            if safe[j] and parent[j]<0:
                parent[j]=i;distance[j]=distance[i]+1;q.append(j)
                if (i==start or (parent[i]==start and math.dist(world(i),pose[:2])<.08) or
                    (initial_target[i]==i and math.dist(world(i),pose[:2])<.08 and
                     straight_continuation(world(parent[i]),world(i),world(j)))):
                    initial_target[j]=j
                else:initial_target[j]=initial_target[i]
    if requested_goal is not None:
        if len(requested_goal)!=2 or not all(math.isfinite(v) for v in requested_goal):
            raise ValueError('invalid_external_frontier')
        # Upstream chooses frontier centroids (at unknown edges). Stop in the
        # closest reachable inflated-free cell within 0.5m, never drive unknown.
        best=None;score=None
        for i in range(w*h):
            check(i)
            if distance[i]<0:continue
            xy=world(i);d=math.dist(xy,requested_goal)
            if d>.5:continue
            value=(d,distance[i])
            if score is None or value<score:best=i;score=value
        if best is None:return dict(path=[],safe=safe,grid=g,reason='external_frontier_unreachable',frontiers=1)
        path=[];i=best
        while i!=start:path.append(world(i));i=parent[i]
        path.append(world(start));path.reverse()
        if len(path)==1:path.append(path[0])
        return dict(path=path,safe=safe,grid=g,reason='external_frontier_route_ready',frontiers=1,goal=world(best))
    frontier=bytearray(w*h)
    for i in range(w*h):
        check(i)
        if 0<=cells[i]<=25 and any(cells[j]<0 and j not in self_unknown for j in adjacent(i)):frontier[i]=1
    labels=array('i',[-1])*(w*h);gains=[];seeds=[]
    for i in range(w*h):
        check(i)
        if not frontier[i] or labels[i]>=0:continue
        label=len(gains);cluster=[];q=deque([i]);labels[i]=label
        while q:
            j=q.popleft();cluster.append(j);check(len(cluster))
            for z in adjacent(j):
                if frontier[z] and labels[z]<0:labels[z]=label;q.append(z)
        gains.append(len(cluster))
        if len(cluster)>=3:seeds.extend(cluster)
    # Propagate each frontier's identity inward to safe, observed stand-off cells.
    near=array('i',[-1])*(w*h);q=deque(seeds)
    for i in seeds:near[i]=0
    depth=math.ceil(radius/res)+3;k=0
    while q:
        i=q.popleft();k+=1;check(k)
        if near[i]>=depth:continue
        for j in adjacent(i):
            if free[j] and near[j]<0:near[j]=near[i]+1;labels[j]=labels[i];q.append(j)
    best=None;score=-1;motion_cache={};motion_rejections=0;motion_blocks=[]
    for i in range(w*h):
        check(i)
        if distance[i]<0 or distance[i]*res<.15 or near[i]<0:continue
        xy=world(i)
        if any(math.hypot(xy[0]-v[0],xy[1]-v[1])<.25 for v in visited):continue
        target_id=initial_target[i]
        if target_id not in motion_cache:
            target=world(target_id)
            angle=math.atan2(target[1]-pose[1],target[0]-pose[0])-pose[2]
            error=math.atan2(math.sin(angle),math.cos(angle))
            obstruction=body_motion_block(g,pose,target,abs(error)>.35,body_width,body_front,body_rear)
            motion_cache[target_id]=obstruction is None
            if obstruction is not None and len(motion_blocks)<8:
                motion_blocks.append(dict(obstruction,pose=list(pose),target=list(target)))
        if not motion_cache[target_id]:motion_rejections+=1;continue
        value=math.sqrt(gains[labels[i]])/(.25+distance[i]*res+near[i]*res)
        if value>score:score=value;best=i
    if best is None:return dict(path=[],safe=safe,grid=g,
        reason='frontiers_require_blocked_initial_motion' if motion_rejections else 'no_reachable_frontier',
        frontiers=len(seeds),initial_motion_rejections=motion_rejections,
        initial_motion_blocks=motion_blocks,self_unknown_cells=len(self_unknown))
    path=[];i=best
    while i!=start:path.append(world(i));i=parent[i]
    path.append(world(start));path.reverse()
    return dict(path=path,safe=safe,grid=g,reason='frontier_route_ready',frontiers=len(seeds),goal=world(best),
                initial_motion_rejections=motion_rejections,initial_motion_checks=len(motion_cache),
                initial_motion_blocks=motion_blocks,self_unknown_cells=len(self_unknown))

def on_safe_grid(plan,x,y):
    w,h,res,ox,oy,yaw,_=plan['grid'];c,s=math.cos(yaw),math.sin(yaw)
    dx,dy=x-ox,y-oy;ix,iy=math.floor((c*dx+s*dy)/res),math.floor((-s*dx+c*dy)/res)
    return 0<=ix<w and 0<=iy<h and bool(plan['safe'][iy*w+ix])

def route_clear_latest(g, pose, path, cursor, radius=.15,
                       body_width=.30, body_front=.1589, body_rear=.1942):
    """Recheck a route against CURRENT occupancy, never its cached planning mask.

    World coordinates survive map growth/origin shifts. Unknown stays blocked;
    the existing inflation-only exemption applies just to the current body.
    Bounded work fails closed instead of starving the drive heartbeat.
    """
    w,h,res,ox,oy,yaw,cells=g
    if not path or len(path)>4096 or not 1<=cursor<len(path):return False
    if not (0<w and 0<h and len(cells)==w*h and .02-1e-9<=res<=.25):return False
    c,s=math.cos(yaw),math.sin(yaw);checked=set()
    footprint=footprint_cells(g,pose,body_width,body_front,body_rear)
    self_unknown=unobserved_body_cells(g,pose,body_width,body_front,body_rear)
    n=math.ceil(radius/res+.5)
    offsets=[(dx,dy) for dy in range(-n,n+1) for dx in range(-n,n+1)
             if math.hypot(max(0,abs(dx)-.5)*res,max(0,abs(dy)-.5)*res)<=radius+1e-9]
    deadline=time.monotonic()+.015
    def check(x,y):
        px,py=x-ox,y-oy
        ix,iy=math.floor((c*px+s*py)/res),math.floor((-s*px+c*py)/res)
        if not (0<=ix<w and 0<=iy<h):return False
        i=iy*w+ix
        if i in checked:return True
        if time.monotonic()>deadline:raise ValueError('live_route_check_budget_exceeded')
        checked.add(i)
        if not 0<=cells[i]<=25 and i not in self_unknown:return False
        if min(ix+.5,w-ix-.5,iy+.5,h-iy-.5)*res<=radius:return False
        if i in footprint:return True
        for dx,dy in offsets:
            xx,yy=ix+dx,iy+dy
            if not (0<=xx<w and 0<=yy<h):return False
            j=yy*w+xx
            if not 0<=cells[j]<=25 and j not in self_unknown:return False
        return True
    previous=pose[:2]
    for point in path[cursor:]:
        if time.monotonic()>deadline:raise ValueError('live_route_check_budget_exceeded')
        distance=math.hypot(point[0]-previous[0],point[1]-previous[1])
        if not math.isfinite(distance) or distance>20:return False
        steps=max(1,math.ceil(distance/(res*.5)))
        for k in range(steps+1):
            t=k/steps
            if not check(previous[0]+t*(point[0]-previous[0]),previous[1]+t*(point[1]-previous[1])):return False
        previous=point
    return True

def body_motion_clear(g,pose,target,rotating,body_width=.30,body_front=.1589,body_rear=.1942):
    return body_motion_block(g,pose,target,rotating,body_width,body_front,body_rear) is None

def body_motion_block(g,pose,target,rotating,body_width=.30,body_front=.1589,body_rear=.1942):
    """Latest-map body envelope for the next motion; no footprint erasing.

    Translation covers a 30cm lookahead in the heading and target directions.
    Rotation checks the full swept circle of the rectangular body. This is
    deliberately separate from the planner's inflation-only start exemption.
    """
    w,h,res,ox,oy,yaw,cells=g
    if not all(math.isfinite(v) for v in (*pose,*target)):return {'kind':'invalid_pose_or_target'}
    self_unknown=unobserved_body_cells(g,pose,body_width,body_front,body_rear)
    def blocked(x,y):
        if not (0<=x<w and 0<=y<h):return {'kind':'outside_map','cell':[x,y]}
        v=cells[y*w+x]
        if 0<=v<=25 or y*w+x in self_unknown:return None
        c,s=math.cos(yaw),math.sin(yaw);gx,gy=(x+.5)*res,(y+.5)*res
        wx,wy=ox+c*gx-s*gy,oy+s*gx+c*gy
        return {'kind':'unknown' if v<0 else 'occupied','cell':[x,y],'value':v,
                'world_xy':[wx,wy],'distance_m':math.hypot(wx-pose[0],wy-pose[1])}
    if rotating:
        radius=math.hypot(max(body_front,body_rear),body_width/2)
        c,s=math.cos(yaw),math.sin(yaw);dx,dy=pose[0]-ox,pose[1]-oy
        cx,cy=(c*dx+s*dy)/res,(-s*dx+c*dy)/res
        n=math.ceil(radius/res+1)
        for y in range(math.floor(cy)-n,math.floor(cy)+n+1):
            for x in range(math.floor(cx)-n,math.floor(cx)+n+1):
                if math.hypot(max(0,abs(x+.5-cx)-.5)*res,max(0,abs(y+.5-cy)-.5)*res)>radius:continue
                obstruction=blocked(x,y)
                if obstruction is not None:return dict(obstruction,motion='rotation')
        return None
    angle=math.atan2(target[1]-pose[1],target[0]-pose[0])
    error=math.atan2(math.sin(angle-pose[2]),math.cos(angle-pose[2]))
    travel=min(.30,max(.08,math.hypot(target[0]-pose[0],target[1]-pose[1])))
    width=body_width+2*travel*abs(math.sin(error))+res
    front=body_front+travel+res/2;rear=body_rear+res/2
    c,s=math.cos(yaw),math.sin(yaw);bc,bs=math.cos(pose[2]),math.sin(pose[2])
    for bx,by in ((-rear,-width/2),(-rear,width/2),(front,-width/2),(front,width/2)):
        dx,dy=pose[0]+bc*bx-bs*by-ox,pose[1]+bs*bx+bc*by-oy
        if not (0<=(c*dx+s*dy)/res<w and 0<=(-s*dx+c*dy)/res<h):return {'kind':'outside_map','motion':'translation'}
    for i in sorted(footprint_cells(g,pose,width,front,rear)):
        obstruction=blocked(i%w,i//w)
        if obstruction is not None:return dict(obstruction,motion='translation')
    return None
