import math, unittest
from dataclasses import replace
from types import SimpleNamespace as S
from unittest.mock import Mock
from acg138_frontier.core import Grid
from acg138_frontier.dynamic_map import DynamicMap
from acg138_frontier.stable_history import StableHistory
import test_dynamic_node_history as node_tests
from test_dynamic_node_history import Stamp, message


class StableTimerTests(unittest.TestCase):
    def evidence(self, h, duration, free={1}, hits={2}, start=1_000_000_000):
        for t in range(0, duration+1, 100_000_000):
            h.observe(start+t,free,hits)
        if duration%100_000_000:h.observe(start+duration,free,hits)

    def test_999_1000_1001_ms_exact_boundary(self):
        for elapsed in (999_000_000,1_000_000_000,1_001_000_000):
            h=StableHistory(10);self.evidence(h,elapsed)
            self.assertEqual(h.values[1],0 if elapsed>=h.HOLD_NS else -1)
            self.assertEqual(h.values[2],100 if elapsed>=h.HOLD_NS else -1)

    def test_duplicate_or_reverse_stamp_does_not_confirm(self):
        h=StableHistory(10);h.observe(1_000_000_000,{1},set())
        for _ in range(25):self.assertFalse(h.observe(1_000_000_000,{1},set()))
        self.assertFalse(h.observe(999_999_999,{1},set()));self.assertEqual(h.values[1],-1)

    def test_elapsed_time_without_observations_not_confirmation(self):
        h=StableHistory(10);h.observe(1_000_000_000,{1},set());h.observe(4_000_000_000,{1},set())
        self.assertEqual(h.values[1],-1)

    def test_missing_cell_in_scan_breaks_continuity(self):
        h=StableHistory(10);self.evidence(h,900_000_000)
        h.observe(2_000_000_000,set(),{2});h.observe(2_100_000_000,{1},{2})
        self.assertEqual(h.values[1],-1);self.assertEqual(h.values[2],100)

    def test_gap_249_250_251_ms(self):
        for gap in (249_000_000,250_000_000,251_000_000):
            h=StableHistory(10);self.evidence(h,900_000_000)
            h.observe(1_900_000_000+gap,{1},{2})
            self.assertEqual(h.values[1],0 if gap<=250_000_000 else -1)

    def test_conflicting_evidence_restarts_one_second_period(self):
        h=StableHistory(10);self.evidence(h,900_000_000)
        h.observe(2_000_000_000,set(),{1,2})
        self.evidence(h,900_000_000,start=2_100_000_000)
        self.assertEqual(h.values[1],-1)
        h.observe(3_100_000_000,{1},{2});self.assertEqual(h.values[1],0)

    def test_correction_retains_committed_but_restarts_pending(self):
        h=StableHistory(10);self.evidence(h,2_000_000_000)
        h.break_confirmation();self.assertEqual((h.values[1],h.values[2]),(0,100))
        self.assertEqual(h.pending.size,0)

    def test_replacement_not_permanent_dynamic_obstacle(self):
        h=StableHistory(10);self.evidence(h,2_000_000_000)
        self.evidence(h,900_000_000,free={1,2},hits=set(),start=3_100_000_000)
        self.assertEqual(h.values[2],100)
        h.observe(4_100_000_000,{1,2},set());self.assertEqual(h.values[2],0)
        self.assertEqual((h.free_count,h.occupied_count),(2,0))

    def test_new_hit_never_demotes_committed_history_to_unknown(self):
        h=StableHistory(10);self.evidence(h,2_000_000_000)
        h.observe(3_100_000_000,set(),{1,2});self.assertEqual(h.values[1],0)
        self.assertEqual(h.free_count,1)

    def test_hits_win_conflicting_input_and_no_negative_counts(self):
        h=StableHistory(10);self.evidence(h,2_000_000_000,free={1},hits={1})
        self.assertEqual(h.values[1],100);self.assertEqual(h.free_count,0)
        h.invalidate_free([1]);self.assertEqual(h.occupied_count,1)


class StableMapIntegrationTests(unittest.TestCase):
    def setUp(self):
        self.g=Grid(220,220,.03,-3.305,-3.305,(-1,)*48400)
        self.m=DynamicMap(clear_range=3.);self.m.bind(self.g);self.stamp=900_000_000
    def scan(self,near=1.,pose=(0,0,0),far=None,step=100_000_000):
        self.stamp+=step
        return self.m.observe(self.stamp,self.stamp,pose,[near],0.,.01,.05,12.,None if far is None else [far])
    def stable(self):
        for _ in range(11):self.assertTrue(self.scan())
    def value(self,x,y=0.):return self.m.cells()[self.m.grid.cell(x,y)]
    def test_real_scans_commit_at_one_second_and_survive_correction(self):
        self.stable();self.m.reference_correction()
        self.assertEqual(self.value(.5),0);self.assertEqual(self.value(1.),100)
        self.assertEqual(self.value(.5,.3),-1)
    def test_less_than_one_second_is_not_locked(self):
        for _ in range(10):self.scan()
        self.assertEqual(self.m.history.free_count,0)
        # Below one second is not committed history, but multi-scan confirmed
        # live evidence also survives a same-map TF correction now.
        self.m.reference_correction();self.assertEqual(self.value(.5),0)
    def test_60_fractional_expansions_and_pose_corrections_preserve_history(self):
        self.stable();points=[self.m.grid.xy(i) for i in self.m.changed]
        expected=[self.value(*xy) for xy in points];stamp=self.m.last_stamp
        for k in range(60):
            self.m.bind(replace(self.g,width=240,height=240,origin_x=-3.305-k*.0031,origin_y=-3.305-k*.0017,cells=(-1,)*57600))
            self.m.reference_correction()
            self.assertEqual([self.value(*xy) for xy in points],expected)
        self.assertEqual(self.m.last_stamp,stamp)
    def test_expansion_preserves_pending_timer_without_adding_evidence(self):
        for _ in range(10):self.scan()
        self.m.bind(replace(self.g,origin_x=-3.312))
        self.assertEqual(self.m.history.free_count,0)
        self.scan();self.assertGreater(self.m.history.free_count,0)
    def test_no_returns_break_timer_never_clear_unknown(self):
        for _ in range(10):self.scan()
        self.assertFalse(self.scan(float('nan')));self.scan();self.m.reference_correction()
        self.assertEqual(self.m.history.free_count,0)
        self.assertEqual(self.value(.5),0)  # earlier confirmed evidence retained
        self.assertEqual(self.value(.5,.3),-1)  # no new evidence in blind cells
    def test_immediate_obstacle_blocks_locked_free_through_correction(self):
        self.stable();self.scan(.5);self.assertEqual(self.value(.5),100)
        self.m.reference_correction();self.assertEqual(self.value(.5),100)
        self.assertEqual(self.value(.3),0)
        for _ in range(3):self.scan(1.)
        self.assertEqual(self.value(.5),0)
        self.m.reference_correction();self.assertEqual(self.value(.5),0)
    def test_stable_free_hit_expansion_correction_never_unknown(self):
        self.stable();self.scan(.5)
        for k in range(5):
            self.m.bind(replace(self.g,width=240,height=240,origin_x=-3.305-k*.03,cells=(-1,)*57600))
            self.m.reference_correction()
            self.assertEqual(self.value(.5),100);self.assertEqual(self.value(.3),0)
    def test_new_slam_obstacle_blocks_locked_free(self):
        self.stable();cells=list(self.g.cells);cells[self.g.cell(.5,0)]=100
        self.m.bind(replace(self.g,cells=tuple(cells)));self.m.reference_correction()
        self.assertEqual(self.value(.5),100)
    def test_dynamic_obstacle_clears_live_then_commits_after_one_second(self):
        self.stable()
        for _ in range(3):self.scan(2.)
        self.assertEqual(self.value(1.),0)
        for _ in range(8):self.scan(2.)
        self.m.reference_correction();self.assertEqual(self.value(1.),0)
    def test_far_rays_only_lock_actually_traversed_cells_within_three_meters(self):
        for _ in range(21):self.scan(float('nan'),far=4.)
        self.m.reference_correction();self.assertEqual(self.value(2.),0)
        self.assertEqual(self.value(3.1),-1);self.assertEqual(self.value(1.,.3),-1)
    def test_freshness_and_source_age_not_refreshed_by_correction(self):
        self.stable();stamp=self.m.last_stamp;self.m.reference_correction()
        self.assertEqual(self.m.last_stamp,stamp)
        self.assertFalse(self.m.observe(stamp,stamp,(0,0,0),[1.],0.,.01,.05,12.))
    def test_new_mapping_session_and_real_geometry_change_clear_stable(self):
        self.stable();self.m.bind(replace(self.g,resolution=.04))
        self.assertEqual(self.m.history.free_count,0)
        self.m.grid=None;self.m.reset('mapping_session_changed');self.m.bind(self.g)
        self.assertEqual(self.m.history.occupied_count,0)


class StableNodeIntegrationTests(unittest.TestCase):
    setUp=node_tests.ProductionDynamicNodeTests.setUp
    tick=node_tests.ProductionDynamicNodeTests.tick
    def prepare(self):
        self.n.map_cb(message(stamp=1_000_000_000));self.ref=(0,0,0)
        self.n.pose=lambda t:t
        self.n.tf=S(lookup_transform=lambda target,source,stamp:self.ref if source=='odom' else (0,0,0))
        self.log=Mock();self.n.get_logger=lambda:self.log
    def scan_tick(self,stamp):
        self.now=stamp
        self.n.scan=S(header=S(stamp=Stamp(stamp).to_msg(),frame_id='laser'),ranges=[1.],
                      angle_min=0.,angle_increment=.01,range_min=.05,range_max=12.)
        self.n.scan_received_mono=-1
        self.tick()
    def test_production_tf_correction_keeps_stable_map_and_logs_counts(self):
        self.prepare()
        for k in range(21):self.scan_tick(1_000_000_000+k*100_000_000)
        self.ref=(.11,0,0);self.scan_tick(3_100_000_000)
        self.assertEqual(self.n.layer.last_reset_reason,'map_odom_correction_keep_stable')
        self.assertGreater(self.n.layer.history.free_count,0)
        self.assertTrue(any('MAP_HISTORY_RETAIN' in str(c) for c in self.log.info.call_args_list))
        out=self.maps[-1];self.assertEqual(out.data[self.n.layer.grid.cell(.5,0)],0)
    def test_old_pose_cannot_trigger_correction_or_freshen_history(self):
        self.prepare();self.scan_tick(1_100_000_000);resets=self.n.layer.resets
        self.ref=(10,0,0);self.scan_tick(1_100_000_000)
        self.assertEqual(self.n.layer.resets,resets)
    def test_source_clock_reversal_clears_committed_history(self):
        self.prepare()
        for k in range(21):self.scan_tick(1_000_000_000+k*100_000_000)
        self.now=2_900_000_000;self.tick()
        self.assertEqual(self.n.layer.history.free_count,0)


if __name__=='__main__':unittest.main()
