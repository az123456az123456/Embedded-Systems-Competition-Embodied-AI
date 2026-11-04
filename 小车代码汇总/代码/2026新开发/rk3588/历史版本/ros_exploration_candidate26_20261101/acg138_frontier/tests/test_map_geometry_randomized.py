import math,random,unittest
from dataclasses import replace
from acg138_frontier.core import Grid
from acg138_frontier.map_geometry import StableMapGeometry


class GeometryOracleTests(unittest.TestCase):
    def test_200_maps_against_independent_area_overlap_oracle(self):
        rng=random.Random(31103)
        for case in range(200):
            w,h=rng.randrange(2,8),rng.randrange(2,8);r=.03
            a=Grid(w,h,r,0.,0.,(-1,)*(w*h));normalizer=StableMapGeometry();normalizer.align(a)
            g=replace(a,origin_x=rng.choice([-1.,-.83,-.51,0.,.12,.61,1.])*r,
                        origin_y=rng.choice([-1.,-.79,-.37,0.,.23,.81,1.])*r,
                        cells=tuple(rng.choice([-1,0,20,26,100])for _ in range(w*h)))
            actual=normalizer.align(g)
            # Slow independent oracle: measure rectangular overlap area with
            # every source cell, not index/rounding or vector gather logic.
            for j,value in enumerate(actual.cells):
                x,y=actual.xy(j);left,bottom=x-r/2,y-r/2
                known_area=0.;maximum=0
                for i,v in enumerate(g.cells):
                    sx,sy=g.xy(i)
                    dx=max(0.,min(left+r,sx+r/2)-max(left,sx-r/2))
                    dy=max(0.,min(bottom+r,sy+r/2)-max(bottom,sy-r/2))
                    area=dx*dy
                    if area<1e-14:continue
                    if v>=0:known_area+=area;maximum=max(maximum,v)
                expected=maximum if maximum>25 or known_area>=r*r-1e-12 else -1
                self.assertEqual(value,expected,(case,j,g.origin_x,g.origin_y))

if __name__=='__main__':unittest.main()
