"""One short, lidar-confirmed first translation; published map is never edited."""
import math
from .exploration_motion import footprint_cells,unobserved_body_cells


def startup_admission(g,pose,ranges,a0,da,rmin,rmax,laser_yaw,tx,ty,
                      width=.30,front=.1589,rear=.1942):
    w,h,res,ox,oy,gyaw,cells=g
    if len(ranges)<180 or abs(da)*len(ranges)<6 or not all(math.isfinite(v) for v in (*pose,a0,da,rmin,rmax,laser_yaw,tx,ty)):
        return False,{'reason':'startup_scan_invalid'}
    # Only an 8cm straight first step. Current-body blind squares retain the
    # existing exemption; external UNKNOWN requires new measured ray evidence.
    full_width=width+res;full_front=front+.08+res/2;full_rear=rear+res/2
    gc,gs=math.cos(gyaw),math.sin(gyaw);bc,bs=math.cos(pose[2]),math.sin(pose[2])
    for x,y in ((-full_rear,-full_width/2),(-full_rear,full_width/2),(full_front,-full_width/2),(full_front,full_width/2)):
        dx,dy=pose[0]+bc*x-bs*y-ox,pose[1]+bs*x+bc*y-oy
        if not (0<=(gc*dx+gs*dy)/res<w and 0<=(-gs*dx+gc*dy)/res<h):return False,{'reason':'startup_outside_map'}
    exempt=unobserved_body_cells(g,pose,width,front,rear)
    blocked=[i for i in footprint_cells(g,pose,full_width,full_front,full_rear) if not 0<=cells[i]<=25 and i not in exempt]
    occupied=[i for i in blocked if cells[i]>=0]
    # At most 200cm² of UNKNOWN and two isolated noisy cells. Larger map
    # obstacles keep their veto even when one sparse scan disagrees.
    if len(blocked)*res*res>.0200001 or len(occupied)>2:
        return False,{'reason':'startup_map_block_too_large','blocked_cells':len(blocked),'occupied_cells':len(occupied)}
    for i in occupied:
        x,y=i%w,i//w
        neighbours=sum(cells[yy*w+xx]>25 for yy in range(max(0,y-1),min(h,y+2)) for xx in range(max(0,x-1),min(w,x+2)) if yy*w+xx!=i)
        if neighbours>1:return False,{'reason':'startup_obstacle_cluster','cell':[x,y]}
    half=res*math.sqrt(.5);certified=0
    for i in blocked:
        gx,gy=(i%w+.5)*res,(i//w+.5)*res
        wx,wy=ox+gc*gx-gs*gy,oy+gs*gx+gc*gy
        dx,dy=wx-pose[0],wy-pose[1];bx,by=bc*dx+bs*dy,-bs*dx+bc*dy
        if math.hypot(bx,by)>.30:return False,{'reason':'startup_cell_outside_near_field','cell':[i%w,i//w]}
        lx,ly=bx-tx,by-ty;distance=math.hypot(lx,ly);angle=math.atan2(ly,lx);votes=0
        window=math.asin(min(.99,half/max(distance,half)))
        for k,r in enumerate(ranges):
            delta=math.atan2(math.sin(a0+k*da+laser_yaw-angle),math.cos(a0+k*da+laser_yaw-angle))
            if abs(delta)>window or not math.isfinite(r) or not rmin<=r<=rmax:continue
            if r<=distance+half+.05:return False,{'reason':'startup_live_lidar_obstacle','cell':[i%w,i//w]}
            votes+=1
        if votes<2:return False,{'reason':'startup_cell_unobserved','cell':[i%w,i//w]}
        certified+=1
    # The full translation envelope is checked against every finite raw hit.
    for k,r in enumerate(ranges):
        if not math.isfinite(r) or not rmin<=r<=rmax:continue
        angle=a0+k*da+laser_yaw;x,y=tx+r*math.cos(angle),ty+r*math.sin(angle)
        if -full_rear-.03<=x<=full_front+.08 and abs(y)<=full_width/2+.03:
            return False,{'reason':'startup_live_lidar_obstacle'}
    return True,{'reason':'startup_lidar_verified_step','temporary_cells':certified,'occupied_cells':len(occupied),'max_travel_m':.08}


class StartupEscape:
    def __init__(self,now):
        self.expires=now+12.;self.used=False;self.active=False;self.origin=None;self.since=0.;self.detail=None
    def available(self,now):return not self.used and now<=self.expires
    def begin(self,odom,now):
        if not self.available(now):return False
        self.used=True;self.active=True;self.origin=odom;self.since=now;return True
    def finish(self):self.active=False
    def progress(self,odom,now):
        if not self.active:return 'startup_inactive'
        if now-self.since>=3.:self.finish();return 'startup_step_timeout'
        if math.hypot(odom[0]-self.origin[0],odom[1]-self.origin[1])>=.08:
            self.finish();return 'startup_complete_replan'
        if abs(math.atan2(math.sin(odom[2]-self.origin[2]),math.cos(odom[2]-self.origin[2])))>.12:
            self.finish();return 'startup_heading_deviation'
        return None
