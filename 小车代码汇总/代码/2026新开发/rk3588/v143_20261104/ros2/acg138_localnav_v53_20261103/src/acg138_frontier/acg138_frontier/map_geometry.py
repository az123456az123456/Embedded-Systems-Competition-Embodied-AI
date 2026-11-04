"""Stable world-cell lattice for expanding SLAM occupancy grids.

Changing a grid's bounding box is not a localization reset. Keep the first
lattice in a session and conservatively resample incoming cells onto it.
Unknown/partial coverage cannot become free, and any occupied overlap wins.
No temporal extrapolation, blur, blind-sector filling, or control APIs.
"""
import math
import numpy as np
from dataclasses import replace


def _snap(v):
    n=round(v)
    return float(n) if abs(v-n)<1e-4 else v


class StableMapGeometry:
    MAX_CELLS=1000000

    def __init__(self):
        self.anchor=None
        self.bounds=None
        self.regrids=0
        self.retained_unknown=0

    def align(self,g):
        # Validate once in the batch representation used below. Repeated Python
        # scans/conversions of a million-cell tuple delay fresh lidar callbacks.
        if (not 0<g.width<=16384 or not 0<g.height<=16384 or g.width*g.height>self.MAX_CELLS
                or len(g.cells)!=g.width*g.height or not g.frame
                or not all(math.isfinite(v) for v in (g.resolution,g.origin_x,g.origin_y,g.yaw))
                or not .005<=g.resolution<=1):raise ValueError('invalid or oversized OccupancyGrid')
        raw=np.asarray(g.cells)
        if not np.all(np.isfinite(raw)) or np.any(raw<-1) or np.any(raw>100):
            raise ValueError('invalid occupancy value')
        source=raw.astype(np.int8).reshape(g.height,g.width)
        a=self.anchor
        if (a is None or a.frame!=g.frame or abs(a.resolution-g.resolution)>1e-9
                or abs(a.yaw-g.yaw)>1e-9):
            # A changed resolution/reference really is incompatible. The caller
            # discards evidence in that case, rather than stretching history.
            self.anchor=g;self.bounds=(0,0,g.width,g.height)
            self.values=source.ravel()
            return g
        c,s=math.cos(a.yaw),math.sin(a.yaw)
        dx,dy=g.origin_x-a.origin_x,g.origin_y-a.origin_y
        fx=_snap((c*dx+s*dy)/a.resolution)
        fy=_snap((-s*dx+c*dy)/a.resolution)
        x0,y0,x1,y1=self.bounds
        bounds=(min(x0,math.floor(fx)),min(y0,math.floor(fy)),
                max(x1,math.ceil(fx+g.width)),max(y1,math.ceil(fy+g.height)))
        x0,y0,x1,y1=bounds;w,h=x1-x0,y1-y0
        if w>16384 or h>16384 or w*h>self.MAX_CELLS:raise ValueError('stable_map_capacity_exceeded')
        ox=a.origin_x+(c*x0-s*y0)*a.resolution
        oy=a.origin_y+(s*x0+c*y0)*a.resolution
        if (w==g.width and h==g.height and abs(fx-x0)<1e-4 and abs(fy-y0)<1e-4):
            result=replace(g,origin_x=ox,origin_y=oy)
            cells=source
        else:
            cells=np.full((h,w),-1,dtype=np.int8)
            integral=fx==round(fx) and fy==round(fy)
            if integral:
                # Aligned expansion: exact row copies, no resampling penalty.
                px,py=int(fx)-x0,int(fy)-y0
                cells[py:py+g.height,px:px+g.width]=source
            else:
                # For each destination square, inspect ALL overlapping source
                # squares (at most 2x2). Never use nearest-neighbor free filling.
                left=max(x0,math.floor(fx));right=min(x1,math.ceil(fx+g.width))
                top=max(y0,math.floor(fy));bottom=min(y1,math.ceil(fy+g.height))
                xs=np.array([(math.floor(_snap(x-fx)),math.ceil(_snap(x+1-fx))-1)
                             for x in range(left,right)],dtype=np.int32)
                ys=np.array([(math.floor(_snap(y-fy)),math.ceil(_snap(y+1-fy))-1)
                             for y in range(top,bottom)],dtype=np.int32)
                maximum=np.zeros((bottom-top,right-left),dtype=np.int8)
                unknown=np.zeros(maximum.shape,dtype=np.bool_)
                for xi in (0,1):
                    x=xs[:,xi];xvalid=(x>=0)&(x<g.width);xc=np.clip(x,0,g.width-1)
                    for yi in (0,1):
                        y=ys[:,yi];yvalid=(y>=0)&(y<g.height);yc=np.clip(y,0,g.height-1)
                        valid=yvalid[:,None]&xvalid[None,:]
                        values=source[yc[:,None],xc[None,:]]
                        unknown|=~valid|(values<0)
                        np.maximum(maximum,np.where(valid,values,0),out=maximum)
                cells[top-y0:bottom-y0,left-x0:right-x0]=np.where((maximum>25)|~unknown,maximum,-1)
            result=replace(g,width=w,height=h,origin_x=ox,origin_y=oy,cells=tuple(cells.ravel().tolist()))
            self.regrids+=1
        # UNKNOWN is lack of evidence, never evidence that an observed world
        # cell ceased to exist. Retain same-session cells by lattice coordinates
        # across crop/growth/sub-cell resampling. New KNOWN observations still
        # replace old free/occupied values; dynamic clearing remains possible.
        bx,by,bex,bey=self.bounds
        previous=self.values.reshape(bey-by,bex-bx)
        region=cells[by-y0:bey-y0,bx-x0:bex-x0]
        retain=(region<0)&(previous>=0)
        self.retained_unknown=int(np.count_nonzero(retain))
        if self.retained_unknown:
            cells=cells.copy()
            region=cells[by-y0:bey-y0,bx-x0:bex-x0]
            region[retain]=previous[retain]
            result=replace(result,cells=tuple(cells.ravel().tolist()))
        self.bounds=bounds
        self.values=cells.ravel()
        return result
