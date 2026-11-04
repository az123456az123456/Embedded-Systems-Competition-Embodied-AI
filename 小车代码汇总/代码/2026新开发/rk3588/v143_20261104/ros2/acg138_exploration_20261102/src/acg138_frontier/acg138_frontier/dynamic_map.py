"""Observation-driven occupancy corrections; no clock-only obstacle deletion.

SLAM retains its graph. This layer corrects the displayed/planning occupancy
grid using finite, time-aligned lidar returns. No robot control APIs here.
"""
from array import array
import math
import numpy as np
from .map_geometry import StableMapGeometry
from .stable_history import StableHistory


class DynamicMap:
    def __init__(self, confirmations=3, clear_range=6., max_rays=360):
        if not 2<=confirmations<=10 or not .1<=clear_range<=12 or not 1<=max_rays<=720:
            raise ValueError('invalid_dynamic_map_configuration')
        self.confirmations=confirmations;self.clear_range=clear_range;self.max_rays=max_rays
        self.grid=None;self.resets=0;self.accepted=0;self.rejected=0
        self.last_stamp=0;self.last_rejection='';self.last_stats={}
        self.override=array('b');self.votes=bytearray();self.changed=set();self.previous_free=set()

    def reset(self,reason='explicit_reset',preserve_stable=False):
        n=len(self.grid.cells) if self.grid else 0
        stamp=self.last_stamp
        retained={}
        if preserve_stable:
            # Keep the latest safety correction on committed cells too. Merely
            # retaining FREE history could otherwise resurrect free space under
            # a newly observed obstacle when map->odom corrects the robot pose.
            # These overrides already passed multi-scan confirmation. A SLAM
            # pose correction must not erase the cells seen for <1 second.
            # Single-scan tentative hits remain transient (not permanent ghosts).
            retained={i:self.override[i] for i in self.changed}
            retained.update((i,100) for i in self.current_hits if self.history.values[i]>=0)
            self.history.break_confirmation()
        else:
            self.history=StableHistory(n)
        self.override=array('b',[-1])*n;self.votes=bytearray(n)
        self.changed=set();self.previous_free=set();self.last_stamp=0;self.resets+=1
        if preserve_stable:self.last_stamp=stamp
        self.current_hits=set();self.previous_hits=set();self.hit_votes=bytearray(n)
        for i,value in retained.items():self.override[i]=value
        self.changed.update(retained)
        self.last_stats={};self.last_rejection=''
        self.last_reset_reason=reason

    def reference_correction(self):
        # map->odom corrects the odometry estimate; it does not rename the map
        # frame. Discard tentative projections, retain all confirmed evidence.
        # A new mapping session / grid frame / source-clock reset still clears all.
        self.reset('map_odom_correction_keep_stable',preserve_stable=True)

    def bind(self,g):
        if g.frame!='map' or not .02-1e-9<=g.resolution<=.25:raise ValueError('unsupported_dynamic_map_geometry')
        if self.grid is None:self.geometry=StableMapGeometry()
        g=self.geometry.align(g)
        key=lambda v:(v.width,v.height,v.resolution,v.origin_x,v.origin_y,v.yaw,v.frame)
        old=self.grid;old_values=getattr(self,'base_cells',None)
        self.grid=g;self.base_cells=self.geometry.values
        if old is None:self.reset('first_session_grid');return
        if key(old)!=key(g):
            # SLAM grows its grid as exploration progresses. Array indices then
            # change but world cells do not. Keep observed corrections rather
            # than resurrecting cleared ghosts from the historical SLAM grid.
            if not self._reindex(old,g,old_values):self.reset('incompatible_grid_reference')
            return
        # Newly occupied SLAM cells invalidate older free corrections. Repeated
        # old occupied cells must not resurrect an obstacle cleared by new rays.
        if self.changed:
            overlay=np.frombuffer(self.override,dtype=np.int8)
            invalid=np.flatnonzero((overlay==0)&(old_values<=25)&(self.base_cells>25))
            overlay[invalid]=-1;np.frombuffer(self.votes,dtype=np.uint8)[invalid]=0
            self.changed.difference_update(invalid.tolist());self.previous_free.difference_update(invalid.tolist())
        self._block_new_obstacles(np.flatnonzero((old_values<=25)&(self.base_cells>25)))

    def _block_new_obstacles(self,indices):
        self.history.invalidate_free(indices)
        # Fresh occupied SLAM data must outrank retained FREE history. Keep a
        # separate veto until actual clearing observations remove the obstacle.
        for i in indices:
            if self.history.values[i]>=0:
                self.override[i]=100;self.changed.add(int(i));self.votes[i]=0
                self.previous_free.discard(int(i))

    def _reindex(self,old,g,old_values):
        if (old.frame!=g.frame or abs(old.resolution-g.resolution)>1e-9 or
            abs(old.yaw-g.yaw)>1e-9):return False
        c,s=math.cos(g.yaw),math.sin(g.yaw)
        dx,dy=old.origin_x-g.origin_x,old.origin_y-g.origin_y
        fx,fy=(c*dx+s*dy)/g.resolution,(-s*dx+c*dy)/g.resolution
        ox,oy=round(fx),round(fy)
        if abs(fx-ox)>1e-4 or abs(fy-oy)>1e-4:return False
        self.history.reindex(old.width,old.height,g.width,g.height,ox,oy)
        # Apply exactly the same fresh-map obstacle rule to committed history.
        new_i=np.flatnonzero(self.base_cells>25)
        hx=new_i%g.width-ox;hy=new_i//g.width-oy
        valid=(hx>=0)&(hx<old.width)&(hy>=0)&(hy<old.height)
        old_i=hy[valid]*old.width+hx[valid]
        new_obstacles=new_i[valid][old_values[old_i]<=25]
        def indices(items):
            i=np.fromiter(items,dtype=np.int32,count=len(items));x=i%old.width+ox;y=i//old.width+oy
            valid=(x>=0)&(x<g.width)&(y>=0)&(y<g.height)
            return i[valid],(y[valid]*g.width+x[valid])
        n=len(g.cells);override=array('b',[-1])*n;votes=bytearray(n);hit_votes=bytearray(n)
        i,j=indices(self.changed);old_overlay=np.frombuffer(self.override,dtype=np.int8)
        # A genuinely new SLAM obstacle takes precedence over old clearing.
        invalid=(old_overlay[i]==0)&(old_values[i]<=25)&(self.base_cells[j]>25)
        invalidated=set(j[invalid].tolist())
        np.frombuffer(override,dtype=np.int8)[j[~invalid]]=old_overlay[i[~invalid]]
        changed=set(j[~invalid].tolist())
        i,j=indices(self.previous_free)
        np.frombuffer(votes,dtype=np.uint8)[j]=np.frombuffer(self.votes,dtype=np.uint8)[i]
        if invalidated:np.frombuffer(votes,dtype=np.uint8)[list(invalidated)]=0
        previous_free=set(j.tolist())-invalidated
        i,j=indices(self.previous_hits)
        np.frombuffer(hit_votes,dtype=np.uint8)[j]=np.frombuffer(self.hit_votes,dtype=np.uint8)[i]
        previous_hits=set(j.tolist())
        _,current=indices(self.current_hits)
        self.changed=changed;self.override=override;self.votes=votes;self.hit_votes=hit_votes
        self.previous_free=previous_free;self.previous_hits=previous_hits;self.current_hits=set(current.tolist())
        self._block_new_obstacles(new_obstacles)
        return True

    def _reject(self,reason):
        self.rejected+=1;self.last_rejection=reason;return False

    def observe(self,stamp_ns,now_ns,laser_pose,ranges,angle_min,angle_increment,range_min,range_max,clearing_ranges=None):
        if self.grid is None:return self._reject('waiting_SLAM_grid')
        if stamp_ns<=0 or not 0<=now_ns-stamp_ns<=250000000:return self._reject('stale_or_future_scan')
        if stamp_ns<=self.last_stamp:return self._reject('duplicate_or_out_of_order_scan')
        if (not 1<=len(ranges)<=8192 or not all(math.isfinite(v) for v in (*laser_pose,angle_min,angle_increment,range_min,range_max))
            or angle_increment==0 or not 0<=range_min<range_max<=100):return self._reject('invalid_scan_geometry')
        if clearing_ranges is not None and len(clearing_ranges)!=len(ranges):
            return self._reject('clearing_scan_geometry_mismatch')
        g=self.grid;c,s=math.cos(g.yaw),math.sin(g.yaw)
        px,py=laser_pose[0]-g.origin_x,laser_pose[1]-g.origin_y
        gx,gy=(c*px+s*py)/g.resolution,(-s*px+c*py)/g.resolution
        if not 0<=gx<g.width or not 0<=gy<g.height:return self._reject('laser_outside_map')
        hits=set();free=set();bins={};valid=0;endpoints=[None]*len(ranges)
        # Every finite endpoint is marked. For clearing, retain the nearest
        # finite ray in each bucket, bounding work without skipping near hits.
        for k,r in enumerate(ranges):
            if not math.isfinite(r) or not range_min<=r<=range_max:continue
            valid+=1;a=angle_min+k*angle_increment+laser_pose[2]-g.yaw
            ca,sa=math.cos(a),math.sin(a)
            ex,ey=gx+r*ca/g.resolution,gy+r*sa/g.resolution
            endpoints[k]=(ex,ey)
            x,y=math.floor(ex),math.floor(ey)
            if 0<=x<g.width and 0<=y<g.height:hits.add(y*g.width+x)
            bucket=k*self.max_rays//len(ranges)
            if bucket not in bins or r<bins[bucket][0]:bins[bucket]=(r,ca,sa,False)
        # Separately filtered, SAME acquisition/frame/geometry (checked by the
        # node). Far finite returns never become endpoints or walls. Their
        # actually traversed ray inside clear_range is free-space evidence,
        # including unknown cells; no-return and blind sectors remain unknown.
        far_bins={}
        if clearing_ranges is not None:
            for k,r in enumerate(clearing_ranges):
                if not math.isfinite(r) or not range_min<=r<=range_max:continue
                bucket=k*self.max_rays//len(ranges)
                if bucket not in far_bins or r<far_bins[bucket][0]:far_bins[bucket]=(r,k)
            for bucket,(r,k) in far_bins.items():
                # Nearest return in the bucket is an occlusion bound, including
                # near returns. Never clear behind a nearer filtered return.
                if r<=self.clear_range or (bucket in bins and bins[bucket][0]<=r):continue
                a=angle_min+k*angle_increment+laser_pose[2]-g.yaw
                bins[bucket]=(r,math.cos(a),math.sin(a),True)
        far_count=sum(b[3] for b in bins.values())
        measured_hits=len(hits)
        walls,segments=self._wall_cells(endpoints)
        hits.update(walls)
        if not valid and not far_count:
            self.previous_free.clear()
            self.history.break_confirmation()
            return self._reject('no_finite_returns')
        # Sample at half-cell intervals. Only actual traversed cells are
        # cleared; never fill sectors or erase cells behind a return.
        guard=max(.05,1.5*g.resolution)
        far_cells=set();far_old_obstacles=set();far_unknown=set()
        for r,ca,sa,far_only in bins.values():
            stop=min(self.clear_range,r-guard)
            steps=max(0,math.floor((stop-range_min)/(g.resolution*.5)))
            for k in range(steps+1):
                distance=range_min+k*g.resolution*.5
                if distance>stop:break
                x,y=math.floor(gx+distance*ca/g.resolution),math.floor(gy+distance*sa/g.resolution)
                if not 0<=x<g.width or not 0<=y<g.height:break
                i=y*g.width+x
                if far_only:
                    far_cells.add(i)
                    if g.cells[i]>25 or self.override[i]==100:far_old_obstacles.add(i)
                    if g.cells[i]==-1 and self.override[i]==-1:far_unknown.add(i)
                free.add(i)
        free.difference_update(hits)
        if stamp_ns-self.last_stamp>250000000:self.previous_free.clear();self.previous_hits.clear()
        cleared=0
        for i in hits:
            self.hit_votes[i]=min(2,self.hit_votes[i]+1) if i in self.previous_hits else 1
            if self.hit_votes[i]>=2:self.override[i]=100;self.changed.add(i)
            self.votes[i]=0
        for i in free:
            self.votes[i]=min(self.confirmations,self.votes[i]+1) if i in self.previous_free else 1
            if self.votes[i]>=self.confirmations:
                if self.override[i]!=0:cleared+=1
                self.override[i]=0;self.changed.add(i)
        # First observations remain visible in the current frame. Only stable
        # two-scan hits enter history, so a displaced single-frame projection
        # cannot leave a permanent black dot behind the moving robot.
        self.current_hits=hits;self.previous_hits=hits
        self.history.observe(stamp_ns,free,hits)
        self.previous_free=free;self.last_stamp=stamp_ns;self.accepted+=1;self.last_rejection=''
        self.last_stats={'finite_returns':valid,'hit_cells':len(hits),'free_ray_cells':len(free),
                         'newly_cleared_cells':cleared,'correction_cells':len(self.changed),
                         'tentative_hit_cells':sum(self.hit_votes[i]<2 for i in hits),
                         'measured_endpoint_cells':measured_hits,'supported_wall_segments':segments,
                         'supported_wall_extra_cells':len(hits)-measured_hits}
        self.last_stats.update(far_clearing_rays=far_count,far_old_obstacle_cells=len(far_old_obstacles-hits),
                               far_observed_free_cells=len(far_cells-hits),far_unknown_cells=len(far_unknown-hits))
        return True

    def _wall_cells(self,points):
        """Short surface rasterization, never gap filling across invalid rays.

        Four consecutive finite beams must support a near-straight surface.
        Adjacent endpoints must be <=10 cm apart, independent of map resolution; chord
        residual <=15 mm. This is a local surface approximation, not a claim
        that every pair of returns is a wall. Same history/clearing as endpoints.
        """
        g=self.grid;segments=set();cells=set()
        gap=.10/g.resolution;residual=.015/g.resolution
        for i in range(len(points)-3):
            p=points[i:i+4]
            if any(q is None for q in p):continue
            if any(math.hypot(b[0]-a[0],b[1]-a[1])>gap+1e-9 for a,b in zip(p,p[1:])):continue
            dx,dy=p[-1][0]-p[0][0],p[-1][1]-p[0][1];length=math.hypot(dx,dy)
            if length<.5:continue
            if any(abs(dx*(q[1]-p[0][1])-dy*(q[0]-p[0][0]))/length>residual for q in p[1:-1]):continue
            # Monotone along chord: do not connect folded/zig-zag returns.
            projections=[(q[0]-p[0][0])*dx+(q[1]-p[0][1])*dy for q in p]
            if any(b<a for a,b in zip(projections,projections[1:])):continue
            segments.update((i,i+1,i+2))
        for i in segments:
            a,b=points[i],points[i+1]
            steps=max(1,math.ceil(math.hypot(b[0]-a[0],b[1]-a[1])*4))
            for j in range(steps+1):
                x=math.floor(a[0]+(b[0]-a[0])*j/steps);y=math.floor(a[1]+(b[1]-a[1])*j/steps)
                if 0<=x<g.width and 0<=y<g.height:cells.add(y*g.width+x)
        return cells,len(segments)

    def cells(self):
        if self.grid is None:return []
        if len(self.changed)>4096 or self.history.free_count or self.history.occupied_count:
            overlay=np.frombuffer(self.override,dtype=np.int8)
            base=np.where(self.history.values>=0,self.history.values,self.base_cells)
            result=np.where(overlay>=0,overlay,base).tolist()
        else:
            result=list(self.grid.cells)
            for i in self.changed:result[i]=self.override[i]
        for i in self.current_hits:result[i]=100
        return result

    def diagnostics(self):
        return {'accepted_scans':self.accepted,'rejected_scans':self.rejected,'last_rejection':self.last_rejection,
                'reset_count':self.resets,'confirmations':self.confirmations,'clear_range_m':self.clear_range,
                'last_scan_stamp_ns':self.last_stamp,'last_reset_reason':getattr(self,'last_reset_reason','not_initialized'),
                'geometry_regrids':self.geometry.regrids if self.grid else 0,
                'retained_known_cells_on_map_update':self.geometry.retained_unknown if self.grid else 0,
                **self.history.diagnostics(),**self.last_stats}
