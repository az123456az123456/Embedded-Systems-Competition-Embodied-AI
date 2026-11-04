"""History survives SLAM bound changes, without making history an obstacle veto."""
import math,random,unittest
from dataclasses import replace
from acg138_frontier.core import Grid
from acg138_frontier.dynamic_map import DynamicMap
from acg138_frontier.map_geometry import StableMapGeometry


class MapHistoryTests(unittest.TestCase):
    def setUp(self):
        self.g=Grid(220,220,.03,-3.305,-3.305,(-1,)*48400)
        self.m=DynamicMap(clear_range=3.);self.m.bind(self.g);self.stamp=1000000000
    def scan(self,near=float('nan'),far=4.,angle=0.,pose=(0,0,0)):
        self.stamp+=100000000
        return self.m.observe(self.stamp,self.stamp,pose,[near],angle,.01,.05,12.,[far])
    def confirm(self,**kw):
        for _ in range(3):self.assertTrue(self.scan(**kw))
    def at(self,x,y=0):return self.m.cells()[self.m.grid.cell(x,y)]
    def grown(self,x,y,w=240,h=240):
        return replace(self.g,width=w,height=h,origin_x=x,origin_y=y,cells=(-1,)*(w*h))
    def test_fractional_expansion_preserves_confirmed_unknown_clearing(self):
        self.confirm();stamp=self.m.last_stamp;resets=self.m.resets
        self.m.bind(self.grown(-3.307,-3.311))
        self.assertEqual(self.at(1),0);self.assertEqual(self.m.last_stamp,stamp)
        self.assertEqual(self.m.resets,resets);self.assertEqual(self.at(1,.3),-1)
    def test_growth_in_all_directions_and_return_do_not_erode_history(self):
        self.confirm();resets=self.m.resets
        for x,y in [(-3.35,-3.3),(-3.3,-3.35),(-3.311,-3.327),(-3.3,-3.3)]:
            self.m.bind(self.grown(x,y));self.assertEqual(self.at(1),0)
        self.assertEqual(self.m.resets,resets)
    def test_60_random_fractional_bounds_preserve_same_world_cells(self):
        self.confirm();observed=[self.m.grid.xy(i) for i in self.m.changed if self.m.override[i]==0]
        rng=random.Random(31003);stamp=self.m.last_stamp
        for _ in range(60):
            self.m.bind(self.grown(-3.3-rng.random()*.3,-3.3-rng.random()*.3))
            cells=self.m.cells()
            self.assertTrue(all(cells[self.m.grid.cell(x,y)]==0 for x,y in observed))
        self.assertEqual(self.m.last_stamp,stamp)
    def test_smaller_incoming_box_does_not_crop_confirmed_history(self):
        self.confirm();self.m.bind(self.grown(-.3,-.3,30,30))
        self.assertEqual(self.at(2),0)
    def test_new_slam_obstacle_supersedes_historical_free(self):
        self.confirm();g=self.grown(-3.307,-3.311);cells=list(g.cells);cells[g.cell(1,0)]=100
        self.m.bind(replace(g,cells=tuple(cells)))
        self.assertEqual(self.at(1),100)
    def test_new_scan_hit_immediately_blocks_historical_free_after_growth(self):
        self.confirm();self.m.bind(self.grown(-3.307,-3.311));self.scan(near=1.)
        self.assertEqual(self.at(1),100)
    def test_dynamic_obstacle_can_still_clear_after_growth(self):
        self.confirm(near=1.,far=1.);self.assertEqual(self.at(1),100)
        self.m.bind(self.grown(-3.307,-3.311))
        self.scan();self.scan();self.assertEqual(self.at(1),100)
        self.scan();self.assertEqual(self.at(1),0)
    def test_growth_does_not_add_votes_or_refresh_source_age(self):
        self.scan();stamp=self.m.last_stamp
        for n in range(1,8):self.m.bind(self.grown(-3.3-n*.001,-3.3))
        self.assertEqual(self.at(1),-1);self.assertEqual(self.m.last_stamp,stamp)
        self.scan();self.assertEqual(self.at(1),-1)
        self.scan();self.assertEqual(self.at(1),0)
    def test_growth_does_not_paint_far_endpoint_or_blind_sector(self):
        self.confirm();self.m.bind(self.grown(-3.307,-3.311,280,280))
        self.assertEqual(self.at(3.5),-1);self.assertEqual(self.at(1,.3),-1)
    def test_rotation_of_robot_does_not_rotate_historical_map(self):
        self.confirm();self.m.bind(self.grown(-3.307,-3.311));self.confirm(pose=(0,0,math.pi/2))
        self.assertEqual(self.at(1),0);self.assertEqual(self.at(0,1),0);self.assertEqual(self.at(1,1),-1)
    def test_reference_reset_still_discards_all_history(self):
        self.confirm();self.m.reset('map_odom_reference_jump')
        self.assertEqual(self.at(1),-1);self.assertEqual(self.m.last_stamp,0)
        self.assertEqual(self.m.diagnostics()['last_reset_reason'],'map_odom_reference_jump')
    def test_new_session_does_not_inherit_extent_or_cells(self):
        self.confirm();self.m.bind(self.grown(-6,-6,400,400))
        self.m.grid=None;self.m.reset('mapping_session_changed');self.m.bind(self.g)
        self.assertEqual(self.m.grid.width,220);self.assertEqual(self.at(1),-1)
    def test_true_grid_rotation_or_resolution_change_resets_history(self):
        for kw in ({'yaw':.01},{'resolution':.04}):
            self.confirm();self.m.bind(replace(self.g,**kw))
            self.assertEqual(self.m.last_stamp,0);self.assertFalse(self.m.changed)
    def test_oversized_union_rejected_without_losing_old_history(self):
        self.confirm();old=self.m.grid;bounds=self.m.geometry.bounds
        with self.assertRaisesRegex(ValueError,'capacity'):self.m.bind(self.grown(1000,1000))
        self.assertIs(self.m.grid,old);self.assertEqual(self.m.geometry.bounds,bounds);self.assertEqual(self.at(1),0)


class ConservativeGeometryTests(unittest.TestCase):
    def test_invalid_input_does_not_mutate_geometry(self):
        n=StableMapGeometry();g=Grid(3,3,.03,0.,0.,(-1,)*9);n.align(g);before=n.values.copy()
        for kw in ({'width':0},{'cells':(-1,)*8},{'cells':(256,)*9},{'cells':(-2,)*9},
                   {'cells':(float('nan'),)*9},{'origin_x':float('inf')},{'resolution':0.}):
            with self.assertRaises(ValueError):n.align(replace(g,**kw))
            self.assertEqual(n.bounds,(0,0,3,3));self.assertTrue((n.values==before).all())
    def test_fractional_free_cell_does_not_fill_unknown_neighbour(self):
        n=StableMapGeometry();g=Grid(3,3,.03,0.,0.,(-1,)*9);n.align(g)
        # One shifted free square overlaps four destination squares; none is
        # wholly observed free, so all must stay unknown.
        a=n.align(replace(g,origin_x=.01,origin_y=.01,cells=(-1,-1,-1,-1,0,-1,-1,-1,-1)))
        self.assertTrue(all(v==-1 for v in a.cells))
    def test_partial_obstacle_cannot_be_erased_by_unknown(self):
        n=StableMapGeometry();g=Grid(3,3,.03,0.,0.,(-1,)*9);n.align(g)
        a=n.align(replace(g,origin_x=.01,origin_y=.01,cells=(-1,-1,-1,-1,100,-1,-1,-1,-1)))
        self.assertEqual(sum(v==100 for v in a.cells),4)
    def test_free_requires_full_coverage_including_source_boundary(self):
        n=StableMapGeometry();g=Grid(3,3,.03,0.,0.,(-1,)*9);n.align(g)
        a=n.align(replace(g,origin_x=.01,origin_y=.01,cells=(0,)*9))
        self.assertEqual(sum(v==0 for v in a.cells),4)
        self.assertEqual(a.cells[0],-1)
    def test_integer_shift_is_exact(self):
        n=StableMapGeometry();g=Grid(3,3,.03,0.,0.,(-1,)*9);n.align(g)
        a=n.align(replace(g,origin_x=-.03,origin_y=.03,cells=tuple(range(9))))
        for i in range(9):
            x=-.03+(i%3+.5)*.03;y=.03+(i//3+.5)*.03
            self.assertEqual(a.cells[a.cell(x,y)],i)
    def test_rotated_grid_uses_grid_axes_for_extent(self):
        n=StableMapGeometry();g=Grid(3,3,.03,0.,0.,(0,)*9,math.pi/2);n.align(g)
        a=n.align(replace(g,origin_x=-.03))
        self.assertEqual((a.width,a.height),(3,4));self.assertEqual(a.yaw,g.yaw)

if __name__=='__main__':unittest.main()
