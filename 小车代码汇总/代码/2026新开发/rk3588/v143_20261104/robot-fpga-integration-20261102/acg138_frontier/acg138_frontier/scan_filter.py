"""Mapping-only speckle rejection. Raw /scan remains the collision input.

No averaging across a near/far edge and no invalid return converted to free.
Isolated finite returns can pass on the next scan if they agree in odom space.
This is consistency filtering, not per-beam motion deskew.
"""
import math

class MappingScanFilter:
    def __init__(self):
        self.previous={};self.last_stamp=0;self.geometry=None;self.stats={}

    def process(self,stamp_ns,now_ns,pose,ranges,angle_min,angle_increment,range_min,range_max):
        if not 0<stamp_ns<=now_ns or now_ns-stamp_ns>250000000:raise ValueError('stale_or_future_scan')
        if stamp_ns<=self.last_stamp:raise ValueError('duplicate_or_old_scan')
        if (not 4<=len(ranges)<=8192 or not all(math.isfinite(x) for x in (*pose,angle_min,angle_increment,range_min,range_max))
            or angle_increment==0 or not 0<range_min<range_max<=100):raise ValueError('invalid_scan_geometry')
        shape=(len(ranges),angle_min,angle_increment,range_min,range_max)
        if shape!=self.geometry or stamp_ns-self.last_stamp>250000000:self.previous={}
        circular=abs(abs(angle_increment)*len(ranges)-2*math.pi)<abs(angle_increment)*1.5
        valid=lambda r:math.isfinite(r) and range_min<=r<=range_max
        output=[float('nan')]*len(ranges);current={};kept=isolated=confirmed=0
        for i,r in enumerate(ranges):
            if not valid(r):continue
            a=angle_min+i*angle_increment+pose[2]
            x,y=pose[0]+r*math.cos(a),pose[1]+r*math.sin(a)
            cell=math.floor(x/.1),math.floor(y/.1)
            current.setdefault(cell,[]).append((x,y))
            # Retain close returns, even a thin pole; raw safety also stays intact.
            supported=r<=.45
            for delta in (-2,-1,1,2):
                j=i+delta
                if circular:j%=len(ranges)
                if not 0<=j<len(ranges) or not valid(ranges[j]):continue
                if abs(r-ranges[j])<=.06+.02*r:supported=True;break
            if not supported:
                isolated+=1
                for dx in (-1,0,1):
                    for dy in (-1,0,1):
                        if any((x-px)**2+(y-py)**2<=.08**2 for px,py in self.previous.get((cell[0]+dx,cell[1]+dy),())):
                            supported=True
                if supported:confirmed+=1
            if supported:output[i]=r;kept+=1
        self.previous=current;self.last_stamp=stamp_ns;self.geometry=shape
        self.stats={'finite_input':sum(valid(r) for r in ranges),'retained':kept,'isolated':isolated,
                    'temporally_confirmed':confirmed,'removed':sum(valid(r) for r in ranges)-kept}
        return output
