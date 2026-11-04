import json,math,sys,unittest
from pathlib import Path
from types import SimpleNamespace as S
from unittest.mock import Mock,patch
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'acg138_frontier/tests'))
import test_live_route
from acg138_frontier.mapping_core import RequestLedger
from acg138_frontier.recovery import Retreat
from acg138_frontier.startup_escape import StartupEscape
from acg138_frontier.semantic_sonar_walk import SemanticSonarWalk,sonar_observation
from test_semantic_sonar_walk import wall_scan
class SemanticExecutorTests(unittest.TestCase):
    def setUp(self):
        self.clock=patch('time.monotonic',return_value=100.);self.clock.start();self.addCleanup(self.clock.stop)
        self.p=patch('time.monotonic_ns',return_value=100000000000);self.p.start();self.addCleanup(self.p.stop)
        test_live_route.ProductionExplorerTests.setUp(self)
    def semantic(self,allow=True):
        n=self.n;n.semantic_external=True;n.external_goal=None;n.external_goal_seq=0
        n.semantic_walk=SemanticSonarWalk();n.semantic_sonar=lambda:((1.,1.,1.),(1,1,1),0)
        n.semantic_odom_pose=lambda:((0.,0.,0.),0)
        n.semantic_scan=lambda:((tuple(float('inf') for _ in range(360)),-math.pi,2*math.pi/360,.05,10.),0)
        n.semantic_gate=dict(semantic=True,task='t',gateway_generation='12',produced_mono_ns=100000000000,valid_for_ms=200,allow_motion=allow)
        return n
    def test_closed_semantic_gate_blocks_existing_route(self):
        n=self.semantic(False);n.tick();self.assertFalse(self.messages[-1]['drive']['motion_requested'])
        self.assertEqual(self.messages[-1]['reason'],'semantic_motion_gate_hold')
    def test_expired_gate_does_not_use_fresh_old_route(self):
        n=self.semantic();n.semantic_gate['produced_mono_ns']=99000000000;n.tick()
        self.assertFalse(self.messages[-1]['drive']['motion_requested'])
    def test_no_map_or_external_goal_enters_random_walk(self):
        n=self.semantic();n.grid=None;n.external_goal=None;n.plan=None
        n.pose=Mock(side_effect=AssertionError('map TF must not be queried'))
        n.clearance=Mock(side_effect=AssertionError('frontier path must not run'))
        n.tick()
        self.assertTrue(self.messages[-1]['drive']['motion_requested'])
        self.assertIn(self.messages[-1]['reason'],('semantic_sonar_forward','semantic_sonar_turn'))
        self.assertEqual(self.messages[-1]['strategy'],'semantic_sonar_walk')
        n.planner_pool.submit.assert_not_called();n.pose.assert_not_called();n.clearance.assert_not_called()
    def test_wrong_generation_gate_cannot_move(self):
        n=self.semantic();n.semantic_gate['gateway_generation']='11';n.tick()
        self.assertFalse(self.messages[-1]['drive']['motion_requested'])
    def test_stop_service_cancels_random_walk(self):
        n=self.semantic();n.tick();self.assertTrue(self.messages[-1]['drive']['motion_requested'])
        n.stop_cb(S(),S());n.tick()
        self.assertIsNone(n.plan);self.assertFalse(n.active)
        self.assertFalse(self.messages[-1]['drive']['motion_requested'])
    def test_gate_revocation_stops_in_same_tick(self):
        n=self.semantic();n.tick();n.semantic_gate['allow_motion']=False;n.tick()
        self.assert_zero()
    def assert_zero(self):
        d=self.messages[-1]['drive'];self.assertFalse(d['motion_requested'])
        for key in ('throttle','steer','speed'):self.assertEqual(d[key],0.)
    def sensors(self,scan_age=0,odom_age=0):
        n=self.semantic()
        class FakeTime:
            @staticmethod
            def from_msg(stamp):return S(nanoseconds=stamp)
        self.time_patch=patch.dict(type(n).tick.__globals__,Time=FakeTime,StartupEscape=StartupEscape,Retreat=Retreat)
        self.time_patch.start();self.addCleanup(self.time_patch.stop)
        n.get_clock=lambda:S(now=lambda:S(nanoseconds=100000000000))
        n.scan=S(header=S(stamp=100000000000-scan_age,frame_id='laser'),ranges=[math.inf]*360,
            angle_min=-math.pi,angle_increment=math.tau/360,range_min=.05,range_max=10.)
        n.odom_scan=S(header=S(stamp=100000000000-odom_age),pose=S(pose=S(
            position=S(x=0.,y=0.),orientation=S(z=0.,w=1.))))
        n.tf=S(lookup_transform=Mock(return_value=S(transform=S(
            translation=S(x=0.,y=0.),rotation=S(z=0.,w=1.)))))
        n.semantic_scan=type(n).semantic_scan.__get__(n)
        n.semantic_odom_pose=type(n).semantic_odom_pose.__get__(n)
        return n
    def test_lidar_and_odometry_expiry_zero_all_outputs(self):
        for name in ('scan','odom_scan'):
            with self.subTest(sensor=name):
                n=self.sensors();n.grid=None;n.tick()
                self.assertTrue(self.messages[-1]['drive']['motion_requested'])
                getattr(n,name).header.stamp-=250000001;n.tick();self.assert_zero()
                self.assertEqual(self.messages[-1]['reason'],'stale_lidar' if name=='scan' else 'stale_odom')
    def test_lidar_and_odometry_expiry_also_stop_reverse_recovery(self):
        for name in ('scan','odom_scan'):
            with self.subTest(sensor=name):
                n=self.sensors();n.scan.ranges=wall_scan(.18)[0]
                n.scan.angle_increment=math.tau/720
                n.semantic_sonar=lambda:((.09,.6,.4),(2,2,2),0)
                n.tick();self.assertLess(self.messages[-1]['drive']['throttle'],0.)
                getattr(n,name).header.stamp-=250000001;n.tick();self.assert_zero()
                self.assertEqual(self.messages[-1]['reason'],'stale_lidar' if name=='scan' else 'stale_odom')
    def test_scan_uses_sensor_mount_and_physical_heading_without_map_tf(self):
        n=self.sensors();n.heading_offset=math.pi;n.grid=None
        n.tf.lookup_transform.return_value.transform.rotation=S(z=math.sin(-math.pi/4),w=math.cos(-math.pi/4))
        scan,_=n.semantic_scan()
        self.assertAlmostEqual(scan[5],math.pi/2)
        n.tf.lookup_transform.assert_called_once()
        self.assertEqual(n.tf.lookup_transform.call_args.args[:2],('base_link','laser'))
        _,_,yaw=n.semantic_odom_pose()[0];self.assertAlmostEqual(abs(yaw),math.pi)
    def test_gateway_start_selects_semantic_without_map_or_goal(self):
        n=self.sensors();n.active=False;n.dry_run=False;n.ledger=RequestLedger();n.grid=None
        q=S(request_id=1,run_generation=12,created_monotonic_ns=100000000000,valid_for_ms=500)
        r=n.start_cb(q,S(success=False,status=0,message=''))
        self.assertTrue(r.success,r.message);self.assertTrue(n.semantic_external)
        n.tick();self.assertTrue(self.messages[-1]['drive']['motion_requested'])
    def test_gate_arriving_after_start_still_enters_semantic_mode(self):
        n=self.sensors();gate=dict(n.semantic_gate);n.semantic_gate={}
        n.active=False;n.dry_run=False;n.ledger=RequestLedger();n.grid=None
        q=S(request_id=1,run_generation=12,created_monotonic_ns=100000000000,valid_for_ms=500)
        self.assertTrue(n.start_cb(q,S(success=False,status=0,message='')).success)
        self.assertFalse(n.semantic_external)
        n.semantic_gate_cb(S(data=json.dumps(gate)))
        self.assertTrue(n.semantic_external);n.tick()
        self.assertTrue(self.messages[-1]['drive']['motion_requested'])
    def test_stationary_rotation_counts_as_progress(self):
        n=self.semantic();n.semantic_last_odom=(0.,0.,0.);n.semantic_last_progress=90.
        n.semantic_odom_pose=lambda:((0.,0.,.3),0);n.tick()
        self.assertEqual(n.semantic_last_progress,100.);self.assertEqual(n.semantic_recovery_count,0)
    def test_stalled_executor_does_not_reselect_direction(self):
        n=self.semantic();n.tick();target=n.semantic_walk.hold_yaw
        n.semantic_last_odom=(0.,0.,0.);n.semantic_last_progress=96.
        n.tick();self.assert_zero()
        self.assertEqual(n.semantic_walk.hold_yaw,target)
        self.assertEqual(self.messages[-1]['reason'],'semantic_no_progress')
    def test_blocked_wait_does_not_freeze_obstacle_updates_after_three_seconds(self):
        n=self.semantic();n.tick()
        blocked=(tuple([.18]*360),-math.pi,math.tau/360,.05,10.)
        n.semantic_scan=lambda:(blocked,0)
        n.semantic_sonar=lambda:((.2,.6,.4),(2,2,2),0)
        with patch('time.monotonic',return_value=104.):
            n.tick();self.assert_zero()
            self.assertEqual(self.messages[-1]['reason'],'rotation_recovery_rear_blocked')
        with patch('time.monotonic',return_value=110.):
            n.tick();self.assert_zero()
            self.assertEqual(self.messages[-1]['reason'],'rotation_recovery_rear_blocked')
            clear=(tuple([math.inf]*360),*blocked[1:])
            n.semantic_scan=lambda:(clear,0)
            n.tick()
        self.assertEqual(self.messages[-1]['drive']['op'],'rotate')
        self.assertTrue(self.messages[-1]['drive']['motion_requested'])
    def test_reverse_recovery_starts_with_its_own_progress_timer(self):
        n=self.semantic();n.tick();n.semantic_last_progress=96.
        n.semantic_sonar=lambda:((.09,.6,.4),(2,2,2),0)
        n.semantic_scan=lambda:(wall_scan(.18),0)
        n.tick()
        self.assertLess(self.messages[-1]['drive']['throttle'],0.)
        self.assertEqual(n.semantic_last_progress,100.)
        self.assertEqual(n.semantic_motion_kind,'reverse')
        n.semantic_sonar=Mock(side_effect=ValueError('stale_sonar'));n.tick();self.assert_zero()
    def test_stalled_forward_can_enter_safe_turn_on_fresh_obstacle(self):
        n=self.semantic();n.tick();n.semantic_last_progress=96.
        n.semantic_sonar=lambda:((.09,.6,.4),(2,2,2),0)
        n.tick()
        self.assertEqual(self.messages[-1]['drive']['op'],'rotate')
        self.assertTrue(self.messages[-1]['drive']['motion_requested'])
    def test_sensor_pause_does_not_count_as_failed_motion(self):
        n=self.semantic();n.tick()
        n.semantic_sonar=Mock(side_effect=ValueError('stale_sonar'));n.tick();self.assert_zero()
        n.semantic_last_progress=90.
        n.semantic_sonar=lambda:((1.,1.,1.),(2,2,2),0);n.tick()
        self.assertTrue(self.messages[-1]['drive']['motion_requested'])
    def test_sonar_expiry_zeroes_output_in_same_tick(self):
        n=self.semantic();n.tick();self.assertTrue(self.messages[-1]['drive']['motion_requested'])
        n.semantic_sonar=Mock(side_effect=ValueError('stale_sonar'))
        n.tick();self.assert_zero();self.assertEqual(self.messages[-1]['reason'],'stale_sonar')
    def test_live_sonar_callback_keeps_source_stamp_and_invalidates_fault(self):
        n=self.semantic();n.sonar_snapshot=None;n.semantic_sonar=type(n).semantic_sonar.__get__(n)
        v=dict(received_mono_ns='100000000000',uptime_ms=1000,valid_mask=7,fault_mask=0,distance_mm=[99,500,400],age_ms=[0,10,20])
        n.sonar_cb(S(data=json.dumps(v)));self.assertEqual(n.semantic_sonar()[0],(.099,.5,.4))
        v['valid_mask']=3;v['fault_mask']=4;n.sonar_cb(S(data=json.dumps(v)))
        with self.assertRaisesRegex(ValueError,'waiting_fresh_sonar'):n.semantic_sonar()
    def test_frontier_progress_timeout_does_not_stop_semantic_search(self):
        n=self.semantic();n.last_progress=-201.;n.tick()
        self.assertTrue(n.active);self.assertTrue(self.messages[-1]['drive']['motion_requested'])
    def test_old_goal_cannot_override_new(self):
        n=self.semantic();n.external_goal_seq=3;n.external_goal=(.8,0.)
        n.external_goal_cb(S(data=json.dumps(dict(seq=2,task='t',gateway_generation='12',x=-1,y=-1,produced_mono_ns=100000000000))))
        self.assertEqual(n.external_goal,(.8,0.))
    def test_new_goal_drops_inflight_old_plan(self):
        n=self.semantic();n.external_goal_cb(S(data=json.dumps(dict(seq=2,task='t',gateway_generation='12',x=.5,y=.5,produced_mono_ns=100000000000))))
        self.assertIsNone(n.plan);self.assertIsNone(n.future);self.assertEqual(n.external_goal,(.5,.5))
if __name__=='__main__':unittest.main()
