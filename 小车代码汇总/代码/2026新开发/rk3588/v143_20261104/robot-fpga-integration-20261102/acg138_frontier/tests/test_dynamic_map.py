import math
import unittest
from dataclasses import replace
from acg138_frontier.core import Grid
from acg138_frontier.dynamic_map import DynamicMap


def grid():return Grid(120,120,.05,-3.,-3.,(0,)*14400)

class DynamicMapTests(unittest.TestCase):
    def setUp(self):
        self.g=grid();self.layer=DynamicMap();self.layer.bind(self.g);self.stamp=1000000000
    def scan(self,r=1.,**kw):
        self.stamp+=100000000
        values=dict(stamp_ns=self.stamp,now_ns=self.stamp+10000000,laser_pose=(0,0,0),
                    ranges=[r],angle_min=0.,angle_increment=.01,range_min=.05,range_max=12.)
        values.update(kw)
        return self.layer.observe(**values)
    def at(self,x,y=0):return self.layer.cells()[self.layer.grid.cell(x,y)]

    def test_new_obstacle_marked_first_scan(self):
        self.assertTrue(self.scan());self.assertEqual(self.at(1),100)
    def test_moving_obstacle_clear_after_three_distinct_scans(self):
        cells=list(self.g.cells);cells[self.g.cell(1,0)]=100
        self.g=replace(self.g,cells=tuple(cells));self.layer.bind(self.g)
        self.scan();self.scan(2);self.assertEqual(self.at(1),100)
        self.scan(2);self.assertEqual(self.at(1),100)
        self.scan(2);self.assertEqual(self.at(1),0);self.assertEqual(self.at(2),100)
        self.layer.bind(self.g);self.assertEqual(self.at(1),0) # SLAM's old wall does not resurrect.
    def test_static_wall_never_cleared_and_reappearance_immediate(self):
        for _ in range(5):self.scan(1);self.assertEqual(self.at(1),100)
        for _ in range(3):self.scan(2)
        self.assertEqual(self.at(1),0);self.scan(1);self.assertEqual(self.at(1),100)
    def test_occluded_and_unobserved_not_cleared(self):
        self.scan(2);self.scan(2)
        for _ in range(4):self.scan(1)
        self.assertEqual(self.at(2),100)
        for _ in range(4):self.scan(2,angle_min=math.pi/2)
        self.assertEqual(self.at(2),100)
    def test_missing_returns_are_not_free_rays(self):
        self.scan(1)
        for r in (float('inf'),float('nan'),0,-1,13):
            self.assertFalse(self.scan(r));self.assertEqual(self.at(1),100)
    def test_duplicates_stale_future_and_out_of_order_do_not_confirm(self):
        self.scan();self.scan();self.scan(2);s=self.stamp
        for _ in range(3):self.assertFalse(self.scan(2,stamp_ns=s,now_ns=s+10000000))
        self.assertFalse(self.scan(2,stamp_ns=s-1,now_ns=s+10000000))
        self.assertFalse(self.scan(2,now_ns=self.stamp+400000000))
        self.assertFalse(self.scan(2,now_ns=self.stamp-1))
        self.assertEqual(self.at(1),100)
    def test_long_gap_requires_new_confirmation_streak(self):
        self.scan();self.scan();self.scan(2);self.scan(2);self.stamp+=400000000
        self.scan(2);self.assertEqual(self.at(1),100)
        self.scan(2);self.scan(2);self.assertEqual(self.at(1),0)
    def test_hit_wins_over_free_in_same_scan(self):
        # The far ray crosses the same near endpoint cell.
        self.scan(1)
        for _ in range(4):self.scan(ranges=[1.,2.],angle_increment=.0001)
        self.assertEqual(self.at(1),100)
    def test_grid_expansion_reindexes_old_evidence(self):
        self.scan(1);self.scan(1);stamp=self.layer.last_stamp
        self.g=replace(self.g,width=121,cells=(0,)*(121*120));self.layer.bind(self.g)
        self.assertEqual(self.at(1),100);self.assertEqual(self.layer.last_stamp,stamp)
    def test_cleared_ghost_does_not_reappear_on_expansion(self):
        cells=list(self.g.cells);cells[self.g.cell(1,0)]=100
        self.g=replace(self.g,cells=tuple(cells));self.layer.bind(self.g)
        for _ in range(3):self.scan(2)
        self.assertEqual(self.at(1),0)
        g=replace(self.g,width=140,height=130,origin_x=-3.5,origin_y=-3.25,cells=(0,)*(140*130))
        cells=list(g.cells);cells[g.cell(1,0)]=100;g=replace(g,cells=tuple(cells))
        self.g=g;self.layer.bind(g)
        self.assertEqual(self.at(1),0)
    def test_expansion_new_obstacle_requires_new_clear_votes(self):
        for _ in range(3):self.scan(2)
        g=replace(self.g,width=121,cells=(0,)*(121*120));cells=list(g.cells);cells[g.cell(1,0)]=100
        self.g=replace(g,cells=tuple(cells));self.layer.bind(self.g)
        self.assertEqual(self.at(1),100);self.scan(2);self.assertEqual(self.at(1),100)
        self.scan(2);self.assertEqual(self.at(1),100);self.scan(2);self.assertEqual(self.at(1),0)
    def test_nonintegral_bounding_box_shift_keeps_world_evidence(self):
        self.scan(1);self.scan(1);stamp=self.layer.last_stamp;resets=self.layer.resets
        world=self.layer.grid.xy(self.layer.grid.cell(1,0))
        self.layer.bind(replace(self.g,origin_x=-3.01))
        self.assertEqual(self.layer.last_stamp,stamp);self.assertEqual(self.at(*world),100)
        self.assertEqual(self.layer.resets,resets)
    def test_map_rotation_respected(self):
        self.g=replace(self.g,origin_x=3.,origin_y=-3.,yaw=math.pi/2);self.layer.bind(self.g)
        self.scan(1);self.assertEqual(self.at(1),100)
    def test_new_slam_obstacle_invalidates_old_free_override(self):
        for _ in range(3):self.scan(2)
        self.assertEqual(self.at(1),0)
        cells=list(self.g.cells);cells[self.g.cell(1,0)]=100
        self.g=replace(self.g,cells=tuple(cells));self.layer.bind(self.g)
        self.assertEqual(self.at(1),100)
    def test_reference_reset_discards_old_corrections(self):
        self.scan(1);self.layer.reset();self.assertEqual(len(self.layer.changed),0)
        self.assertEqual(self.layer.last_stamp,0)
        self.assertNotIn('correction_cells',self.layer.diagnostics())
    def test_unknown_cleared_only_by_observation_not_age(self):
        self.layer=DynamicMap()  # New unknown session, not erasing a known grid.
        self.g=replace(self.g,cells=(-1,)*14400);self.layer.bind(self.g)
        self.scan(2);self.scan(2);self.assertEqual(self.at(1),-1)
        self.scan(2);self.assertEqual(self.at(1),0)
        self.assertEqual(self.at(1,.5),-1)
    def test_clear_range_cap_and_endpoint_guard(self):
        self.layer=DynamicMap(clear_range=1.);self.layer.bind(self.g)
        self.scan(1.5);self.scan(1.5)
        for _ in range(3):self.scan(2.)
        self.assertEqual(self.at(1.5),100)
    def test_one_frame_projection_does_not_accumulate(self):
        self.scan(1);self.assertEqual(self.at(1),100)
        self.scan(2,angle_min=math.pi/2)
        self.assertEqual(self.at(1),0) # Original map, not a free-ray assertion.
        self.scan(2,angle_min=math.pi/2)
        self.scan(1)
        self.assertEqual(self.at(0,2),100) # Confirmed obstacle survives occlusion.
    def test_invalid_geometry_has_no_effect(self):
        self.scan()
        for kw in ({'laser_pose':(99,99,0)},{'laser_pose':(float('nan'),0,0)},
                   {'angle_increment':0},{'range_min':13,'range_max':12}):
            self.assertFalse(self.scan(2,**kw));self.assertEqual(self.at(1),100)

if __name__=='__main__':unittest.main()
