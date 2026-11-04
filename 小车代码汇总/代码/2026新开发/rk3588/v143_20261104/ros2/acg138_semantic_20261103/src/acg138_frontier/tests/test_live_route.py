import ast, json, math, time, unittest
from pathlib import Path
from types import SimpleNamespace as S
from concurrent.futures import Future
from unittest.mock import Mock,patch
from acg138_frontier import exploration_motion as motion
from acg138_frontier.recovery import Retreat
from acg138_frontier.body_heading import physical_pose,sensor_in_body
from acg138_frontier.semantic_random_walk import SemanticRandomWalk

def grid():
    return (100,100,.03,-1.5,-1.5,0.,[0]*10000)
def obstacle(g,x,y,value=100):
    g[6][math.floor((y-g[4])/g[2])*g[0]+math.floor((x-g[3])/g[2])]=value
def route():return [(0.,0.),(.09,0.),(.4,0.),(.8,0.)]

class LiveRouteTests(unittest.TestCase):
    def test_new_obstacle_old_mask_not_used(self):
        g=grid();self.assertTrue(motion.route_clear_latest(g,(0,0,0),route(),1))
        obstacle(g,.65,0);self.assertFalse(motion.route_clear_latest(g,(0,0,0),route(),1))
    def test_unknown_and_uncertain_stop(self):
        for v in (-1,26,100):
            g=grid();obstacle(g,.65,0,v);self.assertFalse(motion.route_clear_latest(g,(0,0,0),route(),1))
    def test_off_path_refresh_can_continue(self):
        g=grid();obstacle(g,.65,.6);self.assertTrue(motion.route_clear_latest(g,(0,0,0),route(),1))
    def test_origin_shift_same_world_obstacle(self):
        g=grid();g=(120,120,.03,-1.8,-1.8,0.,[0]*14400);obstacle(g,.65,0)
        self.assertFalse(motion.route_clear_latest(g,(0,0,0),route(),1))
    def test_rotated_grid(self):
        g=grid();obstacle(g,.65,0);a=.7;c,s=math.cos(a),math.sin(a)
        q=(*g[:3],c*g[3]-s*g[4],s*g[3]+c*g[4],a,g[6])
        path=[(c*x-s*y,s*x+c*y) for x,y in route()]
        self.assertFalse(motion.route_clear_latest(q,(0,0,a),path,1))
    def test_body_nose_and_turning_corners(self):
        g=grid();self.assertTrue(motion.body_motion_clear(g,(0,0,0),(.09,0),False))
        obstacle(g,.20,.1);self.assertFalse(motion.body_motion_clear(g,(0,0,0),(.09,0),False))
        g=grid();obstacle(g,-.22,0);self.assertFalse(motion.body_motion_clear(g,(0,0,0),(0,1),True))
    def test_external_unknown_never_cleared_by_start_exemption(self):
        g=grid();obstacle(g,.2,0,-1)
        self.assertFalse(motion.body_motion_clear(g,(0,0,0),(.09,0),False))

class ProductionExplorerTests(unittest.TestCase):
    def setUp(self):
        source=Path(motion.__file__).with_name('mapping_explorer.py').read_text()
        cls=next(n for n in ast.parse(source).body if isinstance(n,ast.ClassDef))
        ns={'Node':object,'time':time,'json':json,'math':math,'String':lambda **k:S(**k),
            'plan_frontier':motion.plan_frontier,'route_clear_latest':motion.route_clear_latest,'body_motion_block':motion.body_motion_block,
            'tracking_target':motion.tracking_target,'physical_pose':physical_pose,'sensor_in_body':sensor_in_body,
            'SemanticRandomWalk':SemanticRandomWalk}
        exec(compile(ast.Module(body=[cls],type_ignores=[]),'<production mapping_explorer>','exec'),ns)
        n=ns['MappingExplorer'].__new__(ns['MappingExplorer']);self.n=n;self.messages=[]
        n.semantic_external=False;n.semantic_gate={};n.external_goal=None;n.external_goal_seq=0;n.external_status='WAITING_GOAL'
        n.semantic_walk=SemanticRandomWalk();n.semantic_last_odom=None;n.semantic_last_progress=time.monotonic();n.semantic_recovery_count=0
        n.active=True;n.generation=10;n.gateway_generation=12;n.sequence=0
        n.grid=grid();n.grid_seq=2;n.plan={'path':route(),'goal':(.8,0)};n.cursor=1
        n.future=None;n.planning_generation=10;n.route_epoch=n.planning_epoch=1;n.checked_map=1
        n.blocked_grid=None;n.blocked_pose=None;n.blocked_reason='';n.retry_at=0.
        n.block_detail=None;n.recovery_detail=None;n.heading_offset=0.
        n.retreat=Retreat();n.recovery_reason='';n.forward_progress_pose=None
        n.planned_map=1;n.plan_at=0;n.last_progress=time.monotonic();n.progress_pose=None;n.wait_since=0
        n.visited=[];n.inflation_radius=.15;n.body_width=.30;n.body_front=.1589;n.body_rear=.1942
        n.pose=lambda:((0,0,0),0);n.clearance=lambda:(float('inf'),float('inf'),0)
        n.odom_pose=lambda:(0,0,0)
        n.state='NAVIGATING';n.reason='';n.planner_pool=S(submit=Mock(return_value=Future()))
        n.pub=S(publish=lambda m:self.messages.append(json.loads(m.data)))
    def test_map_obstacle_stops_same_tick_even_with_far_scan(self):
        obstacle(self.n.grid,.65,0);self.n.tick();m=self.messages[-1]
        self.assertFalse(m['drive']['motion_requested']);self.assertEqual(m['drive']['throttle'],0)
        self.assertEqual(m['reason'],'latest_map_blocks_route_stop_replan');self.assertIsNone(self.n.plan)
    def test_unrelated_map_updates_keep_goal_and_cursor(self):
        original=self.n.plan
        for seq in range(2,7):
            self.n.grid_seq=seq;self.n.tick()
            self.assertTrue(self.messages[-1]['drive']['motion_requested'])
            self.assertIs(self.n.plan,original);self.assertEqual(self.n.cursor,1)
        self.n.planner_pool.submit.assert_not_called()
    def test_older_map_result_revalidated_before_drive(self):
        self.n.plan=None;f=Future();f.set_result({'path':route(),'goal':(.8,0),'reason':'ready'})
        self.n.future=f;obstacle(self.n.grid,.65,0);self.n.tick()
        self.assertFalse(self.messages[-1]['drive']['motion_requested'])
    def test_previous_run_completion_does_not_resume_or_raise(self):
        self.n.plan=None;f=Future();f.set_exception(ValueError('old worker failure'))
        self.n.future=f;self.n.planning_generation=9;self.n.tick()
        self.assertEqual(self.messages[-1]['reason'],'computing_frontier_route')
        self.assertFalse(self.messages[-1]['drive']['motion_requested'])
    def test_planner_failure_waits_for_changed_evidence_not_50ms_retry_loop(self):
        self.n.plan=None;f=Future();f.set_exception(ValueError('robot_cell_blocked_after_inflation'))
        self.n.future=f
        with patch.object(time,'monotonic',return_value=100):
            self.n.last_progress=100;self.n.tick()
        for t in (100.05,100.2,101,105):
            self.n.grid_seq+=1
            with patch.object(time,'monotonic',return_value=t):self.n.tick()
            self.assertFalse(self.messages[-1]['drive']['motion_requested'])
            self.assertEqual(self.messages[-1]['reason'],'robot_cell_blocked_after_inflation')
        self.n.planner_pool.submit.assert_not_called()
        self.assertEqual(self.n.wait_since,100)
    def test_no_executable_frontier_reports_actual_block(self):
        self.n.plan=None;f=Future();f.set_result({'path':[],'reason':'frontiers_require_blocked_initial_motion',
            'initial_motion_blocks':[{'kind':'unknown','world_xy':[.2,0.],'motion':'rotation'}],
            'self_unknown_cells':1,'initial_motion_rejections':5})
        self.n.future=f;self.n.tick()
        m=self.messages[-1];self.assertFalse(m['drive']['motion_requested'])
        self.assertEqual(m['body_block']['kind'],'unknown');self.assertEqual(m['self_unknown_cells'],1)
    def test_invalidated_epoch_completion_discarded(self):
        self.n.plan=None;f=Future();f.set_result({'path':route(),'reason':'ready'})
        self.n.future=f;self.n.route_epoch=2;self.n.tick()
        self.assertFalse(self.messages[-1]['drive']['motion_requested']);self.assertIsNone(self.n.plan)
    def test_body_blocked_without_new_map_cannot_move(self):
        self.n.checked_map=self.n.grid_seq;obstacle(self.n.grid,.2,.1);self.n.tick()
        self.assertFalse(self.messages[-1]['drive']['motion_requested'])
        self.assertEqual(self.messages[-1]['reason'],'latest_map_blocks_body_stop_replan')
        self.assertEqual(self.messages[-1]['body_block']['kind'],'occupied')
        self.assertEqual(self.messages[-1]['body_block']['map_seq'],self.n.grid_seq)
    def test_repeated_identical_maps_do_not_replan_blocked_route(self):
        obstacle(self.n.grid,.2,.1);self.n.tick()
        epoch=self.n.route_epoch
        for _ in range(10):
            self.n.grid=(*self.n.grid[:6],list(self.n.grid[6]));self.n.grid_seq+=1;self.n.tick()
            self.assertFalse(self.messages[-1]['drive']['motion_requested'])
        self.n.planner_pool.submit.assert_not_called();self.assertEqual(self.n.route_epoch,epoch)
    def test_wait_timeout_is_not_refreshed_by_planning(self):
        with patch.object(time,'monotonic',return_value=100):
            self.n.last_progress=100;obstacle(self.n.grid,.2,.1);self.n.tick()
        # A changed map permits one new plan, but SELECTING must not reset the deadline.
        self.n.grid=(*self.n.grid[:6],list(self.n.grid[6]));obstacle(self.n.grid,1.,1.)
        with patch.object(time,'monotonic',return_value=110):self.n.tick()
        self.assertEqual(self.n.wait_since,100)
        f=self.n.future;f.set_result({'path':route(),'goal':(.8,0),'reason':'ready'})
        with patch.object(time,'monotonic',return_value=131):self.n.tick()
        self.assertFalse(self.n.active);self.assertEqual(self.n.state,'ERROR')
        self.assertTrue(self.n.reason.startswith('exploration_wait_timeout:'))
        self.assertFalse(self.messages[-1]['drive']['motion_requested'])
    def test_changed_clear_map_replans_and_rechecks_before_motion(self):
        with patch.object(time,'monotonic',return_value=100):
            self.n.last_progress=100;obstacle(self.n.grid,.2,.1);self.n.tick()
        self.n.grid=grid();self.n.grid_seq+=1
        with patch.object(time,'monotonic',return_value=100.2):self.n.tick()
        self.n.planner_pool.submit.assert_not_called()
        with patch.object(time,'monotonic',return_value=101):self.n.tick()
        self.n.planner_pool.submit.assert_called_once()
        self.assertFalse(self.messages[-1]['drive']['motion_requested'])
        self.n.future.set_result({'path':route(),'goal':(.8,0),'reason':'ready'})
        with patch.object(time,'monotonic',return_value=101.1):self.n.tick()
        self.assertTrue(self.messages[-1]['drive']['motion_requested']);self.assertEqual(self.n.wait_since,0)
    def test_actual_saved_map_new_plan_produces_drive_old_plan_stops(self):
        from test_initial_motion import sample
        g,pose,old=sample();self.n.grid=g;self.n.pose=lambda:(pose,0)
        self.n.clearance=lambda:(.34,.34,0)
        self.n.plan={'path':old,'goal':old[-1]};self.n.cursor=1;self.n.tick()
        self.assertFalse(self.messages[-1]['drive']['motion_requested'])
        self.assertEqual(self.messages[-1]['body_block']['motion'],'rotation')
        self.n.blocked_grid=None;self.n.blocked_pose=None
        self.n.plan=motion.plan_frontier(g,pose,[]);self.n.cursor=1;self.n.checked_map=0;self.n.tick()
        self.assertTrue(self.messages[-1]['drive']['motion_requested'])
        self.assertEqual(self.messages[-1]['drive']['op'],'drive')
        self.assertGreater(self.messages[-1]['drive']['throttle'],0)
        self.assertIsNone(self.messages[-1]['body_block'])
    def test_this_saved_map_unknown_under_origin_produces_admissible_intent(self):
        from test_self_unknown import sample
        g,pose=sample();self.n.grid=g;self.n.pose=lambda:(pose,0)
        self.n.clearance=lambda:(.65,.65,0)
        self.n.plan=motion.plan_frontier(g,pose,[]);self.n.cursor=1;self.n.checked_map=0
        self.n.tick();m=self.messages[-1]
        self.assertTrue(m['drive']['motion_requested']);self.assertEqual(m['drive']['op'],'rotate')
        self.assertGreater(m['drive']['speed'],0);self.assertEqual(m['self_unknown_cells'],1)
        self.assertIsNone(m['body_block'])
