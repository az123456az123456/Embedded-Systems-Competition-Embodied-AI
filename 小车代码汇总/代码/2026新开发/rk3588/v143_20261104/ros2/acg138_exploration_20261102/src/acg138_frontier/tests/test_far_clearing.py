import copy,math,unittest
from dataclasses import replace
from types import SimpleNamespace as S
from acg138_frontier.core import Grid
from acg138_frontier.dynamic_map import DynamicMap
from acg138_frontier.scan_pair import ClearingScanCache

NAN=float('nan')

class FarClearingTests(unittest.TestCase):
    def setUp(self):
        self.g=Grid(300,100,.03,-4.5,-1.5,(-1,)*30000)
        cells=list(self.g.cells)
        for x in (.16,1.5,2.98,3.15):cells[self.g.cell(x,0)]=100
        self.g=replace(self.g,cells=tuple(cells));self.m=DynamicMap(clear_range=3.);self.m.bind(self.g);self.stamp=1000000000
    def scan(self,far=3.29,near=NAN,**kw):
        self.stamp+=100000000
        args=dict(stamp_ns=self.stamp,now_ns=self.stamp,laser_pose=(0,0,0),ranges=[near],angle_min=0.,
                  angle_increment=.01,range_min=.05,range_max=12.,clearing_ranges=[far])
        args.update(kw);return self.m.observe(**args)
    def at(self,x):return self.m.cells()[self.g.cell(x,0)]
    def test_three_far_observations_clear_obstacles_and_observed_unknown_within_horizon(self):
        self.scan();self.scan();self.assertEqual(self.at(.16),100)
        self.assertEqual(self.at(.5),-1) # still requires three independent scans
        self.scan()
        for x in (.16,1.5,2.98):self.assertEqual(self.at(x),0)
        self.assertEqual(self.at(3.15),100) # beyond clearing horizon
        self.assertEqual(self.at(3.29),-1) # far endpoint never painted
        self.assertEqual(self.at(.5),0) # actually traversed unknown is now observed
        self.assertEqual(self.m.cells()[self.g.cell(.5,.15)],-1) # no sector filling
        self.assertGreater(self.m.diagnostics()['far_observed_free_cells'],0)
    def test_invalid_far_returns_do_not_clear(self):
        for r in (NAN,float('inf'),0.,-1.,12.1):
            for _ in range(4):self.assertFalse(self.scan(far=r))
        self.assertEqual(self.at(.16),100)
        self.assertEqual(self.at(.5),-1)
    def test_old_or_duplicate_far_samples_do_not_accumulate(self):
        self.scan();s=self.stamp
        for _ in range(4):self.assertFalse(self.scan(stamp_ns=s,now_ns=s))
        self.assertFalse(self.scan(stamp_ns=s+1,now_ns=s+300000000))
        self.assertEqual(self.at(.16),100)
    def test_real_near_hit_wins_and_occludes_far_ray(self):
        for _ in range(4):self.scan(near=.16)
        self.assertEqual(self.at(.16),100);self.assertEqual(self.at(1.5),100)
    def test_nearest_clearing_return_limits_same_bucket(self):
        # Both rays belong to one work bucket. A close return prevents a far
        # neighbor from erasing an obstacle hidden behind it.
        self.m.max_rays=1
        for _ in range(4):self.scan(ranges=[NAN,NAN],clearing_ranges=[.10,3.29],angle_increment=.0001)
        self.assertEqual(self.at(.16),100)
    def test_wrong_geometry_rejected_without_changes(self):
        self.assertFalse(self.scan(clearing_ranges=[3.29,3.29]))
        self.assertEqual(self.at(.16),100);self.assertEqual(self.m.last_stamp,0)
    def test_unpaired_near_path_unchanged(self):
        for _ in range(4):self.scan(near=1.,clearing_ranges=None)
        self.assertEqual(self.at(.16),0);self.assertEqual(self.at(1.),100);self.assertEqual(self.at(1.5),100)

    def test_far_ray_respects_transformed_origin_and_blind_direction(self):
        for _ in range(3):self.scan(laser_pose=(.3,.2,math.pi/2))
        cells=self.m.cells()
        self.assertEqual(cells[self.g.cell(.3,.7)],0)
        self.assertEqual(cells[self.g.cell(.7,.2)],-1)
        self.assertEqual(self.at(.16),100)

    def test_real_near_obstacle_keeps_unknown_behind_it_unknown(self):
        for _ in range(4):self.scan(near=.3)
        self.assertEqual(self.at(.3),100)
        self.assertEqual(self.at(.5),-1)

    def test_gap_restarts_unknown_confirmations(self):
        self.scan();self.scan();self.stamp+=300000000
        self.scan();self.assertEqual(self.at(.5),-1)
        self.scan();self.assertEqual(self.at(.5),-1)
        self.scan();self.assertEqual(self.at(.5),0)

def scan(stamp=1,frame='laser'):
    return S(header=S(stamp=S(sec=1,nanosec=stamp),frame_id=frame),ranges=[3.29],
             angle_min=0.,angle_max=0.,angle_increment=.01,range_min=.05,range_max=12.,scan_time=.1,time_increment=0.)

class PairingTests(unittest.TestCase):
    def test_exact_timestamp_frame_and_geometry_only(self):
        c=ClearingScanCache();c.put(scan())
        self.assertIsNone(c.take(scan(2)));self.assertIsNone(c.take(scan(frame='other')))
        self.assertEqual(c.take(scan()),[3.29]);self.assertIsNone(c.take(scan()))
    def test_geometry_mismatch_does_not_provide_rays(self):
        for name,value in [('angle_min',1.),('angle_increment',.02),('range_max',20.),('ranges',[3.29,3.29])]:
            c=ClearingScanCache();m=scan();setattr(m,name,value);c.put(m)
            self.assertIsNone(c.take(scan()))
    def test_bounded_cache_and_reset(self):
        c=ClearingScanCache()
        for i in range(20):c.put(scan(i))
        self.assertEqual(len(c.items),8);self.assertIsNone(c.take(scan(0)))
        self.assertEqual(c.take(scan(19)),[3.29]);c.clear();self.assertFalse(c.items)

if __name__=='__main__':unittest.main()
