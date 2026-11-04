"""Bounded RK exploration retreat; no FPGA navigation or authorization API."""
import math
from .exploration_motion import footprint_cells,unobserved_body_cells


def rear_scan_clear(ranges,angle_min,increment,minimum,maximum,laser_yaw,tx,ty,
                    width=.30,rear=.1942,distance=.15):
    """Fresh scan required by caller. Missing rear rays are NOT free space."""
    total=finite=0
    for k,r in enumerate(ranges):
        a=angle_min+k*increment+laser_yaw
        behind=abs(math.atan2(math.sin(a-math.pi),math.cos(a-math.pi)))<.8
        if behind:total+=1
        valid=math.isfinite(r) and minimum<=r<=maximum
        if behind and valid:finite+=1
        if not valid:continue
        x,y=tx+r*math.cos(a),ty+r*math.sin(a)
        if -rear-distance-.05<=x<=0 and abs(y)<=width/2+.04:return False
    return total>=10 and finite>=.6*total


def reverse_block_detail(g,pose,width=.30,front=.1589,rear=.1942,distance=.15):
    """Whole swept body, with clearance, in CURRENT map; no history bypass."""
    w,h,res,ox,oy,gyaw,cells=g
    if not all(math.isfinite(v) for v in (*pose,distance)) or not 0<=distance<=.2:return {'reason':'invalid_reverse_geometry'}
    free_inside=unobserved_body_cells(g,pose,width,front,rear)
    # Include stopping clearance behind the bumper, never erase occupied cells.
    full_width=width+.04;full_front=front+.02;full_rear=rear+distance+.05
    c,s=math.cos(pose[2]),math.sin(pose[2]);gc,gs=math.cos(gyaw),math.sin(gyaw)
    for x,y in ((-full_rear,-full_width/2),(-full_rear,full_width/2),(full_front,-full_width/2),(full_front,full_width/2)):
        dx,dy=pose[0]+c*x-s*y-ox,pose[1]+s*x+c*y-oy
        if not (0<=(gc*dx+gs*dy)/res<w and 0<=(-gs*dx+gc*dy)/res<h):return {'reason':'reverse_outside_map'}
    for i in sorted(footprint_cells(g,pose,full_width,full_front,full_rear)):
        if not 0<=cells[i]<=25 and i not in free_inside:
            gx,gy=(i%w+.5)*res,(i//w+.5)*res
            wx,wy=ox+gc*gx-gs*gy,oy+gs*gx+gc*gy
            dx,dy=wx-pose[0],wy-pose[1]
            return {'reason':'reverse_unknown' if cells[i]<0 else 'reverse_obstacle',
                    'kind':'unknown' if cells[i]<0 else 'occupied','cell':[i%w,i//w],
                    'value':cells[i],'world_xy':[wx,wy],'body_xy':[c*dx+s*dy,-s*dx+c*dy],
                    'pose':list(pose),'distance_m':math.hypot(dx,dy)}
    return None

def reverse_block(g,pose,width=.30,front=.1589,rear=.1942,distance=.15):
    detail=reverse_block_detail(g,pose,width,front,rear,distance)
    return detail['reason'] if detail else None


class Retreat:
    def __init__(self):
        self.active=False;self.attempts=0;self.start=None;self.since=0.;self.best=0.;self.progress_at=0.
    def cancel(self):self.active=False
    def begin(self,odom,now):
        if self.attempts>=3:return False
        self.start=odom;self.since=self.progress_at=now;self.best=0.;self.active=True;self.attempts+=1
        return True
    def update(self,g,pose,odom,now,rear_ok,width=.30,front=.1589,rear=.1942):
        if not self.active:return False,'recovery_inactive'
        x,y,a=self.start;dx,dy=odom[0]-x,odom[1]-y
        distance=-(dx*math.cos(a)+dy*math.sin(a))
        lateral=abs(-dx*math.sin(a)+dy*math.cos(a))
        angle=abs(math.atan2(math.sin(odom[2]-a),math.cos(odom[2]-a)))
        reason=None
        if distance<-.03 or distance>.23 or lateral>.05 or angle>.15:reason='recovery_pose_deviation'
        elif distance>=.15:reason='reverse_complete_replan'
        elif not rear_ok:reason='reverse_lidar_blocked_or_unobserved'
        else:reason=reverse_block(g,pose,width,front,rear,max(0.,.15-distance))
        if distance>self.best+.01:self.best=distance;self.progress_at=now
        if reason is None and (now<self.since or now-self.since>=6 or now-self.progress_at>=1.5):reason='reverse_no_progress_stop'
        if reason is not None:self.active=False;return False,reason
        # Brake before reversing. Only real odometry displacement ends retreat.
        if now-self.since<.15:return False,'reverse_brake_before_motion'
        return True,'reversing_to_clear_turn'
