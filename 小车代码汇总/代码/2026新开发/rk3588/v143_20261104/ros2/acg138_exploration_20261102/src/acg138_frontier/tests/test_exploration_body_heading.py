import json, math, unittest
from pathlib import Path
from types import SimpleNamespace as S
from acg138_frontier.body_heading import physical_pose,sensor_in_body
from acg138_frontier.exploration_motion import plan_frontier,tracking_target,body_motion_block
from acg138_frontier.recovery import Retreat,reverse_block_detail
from test_live_route import ProductionExplorerTests,grid,obstacle

class BodyHeadingTests(unittest.TestCase):
    def test_arrow_forward_equals_exploration_forward(self):
        for raw in (-math.pi,-2.,0.,1.,math.pi):
            p=physical_pose((1.,2.,raw),math.pi)
            self.assertEqual(p[:2],(1.,2.))
            self.assertAlmostEqual(math.cos(p[2]),-math.cos(raw))
            self.assertAlmostEqual(math.sin(p[2]),-math.sin(raw))
    def test_sensor_reexpression_preserves_world_rays(self):
        for raw in (-2.,0.,1.):
            for offset in (0.,math.pi,.73):
                yaw,x,y=sensor_in_body(-1.57,.03,-.02,offset)
                a=physical_pose((0,0,raw),offset)[2]
                for angle in (-1.,0.,2.):
                    old=(math.cos(raw)*.03-math.sin(raw)*(-.02)+2*math.cos(raw-1.57+angle),
                         math.sin(raw)*.03+math.cos(raw)*(-.02)+2*math.sin(raw-1.57+angle))
                    new=(math.cos(a)*x-math.sin(a)*y+2*math.cos(a+yaw+angle),
                         math.sin(a)*x+math.cos(a)*y+2*math.sin(a+yaw+angle))
                    for u,v in zip(old,new):self.assertAlmostEqual(u,v)
    def test_invalid_heading_rejected(self):
        for v in (math.nan,math.inf):
            with self.assertRaises(ValueError):physical_pose((0,0,0),v)
            with self.assertRaises(ValueError):sensor_in_body(v,0,0,0)
    def test_metric_target_not_first_two_cells(self):
        path=[(.0125+.025*i,.0125) for i in range(8)]
        t=tracking_target(path,2,(0,0,0))
        self.assertEqual(t,path[3]);self.assertGreaterEqual(math.dist(t,(0,0)),.08)
    def test_lookahead_stops_at_corner_or_reversal(self):
        for path in ([(0,0),(.025,0),(.05,0),(.05,.025),(.05,.05)],
                     [(0,0),(.025,0),(.05,0),(.025,0),(0,0)]):
            self.assertEqual(tracking_target(path,1,(0,0,0)),path[2])
    def test_reverse_detail_does_not_exempt_unknown(self):
        g=grid();obstacle(g,-.3,0,-1);d=reverse_block_detail(g,(0,0,0))
        self.assertEqual(d['reason'],'reverse_unknown');self.assertLess(d['body_xy'][0],0)
        self.assertEqual(d['value'],-1);self.assertEqual(g[6][d['cell'][1]*g[0]+d['cell'][0]],-1)
    def test_frontier_and_controller_initial_target_agree_fine_grids(self):
        for res in (.02,.025,.03,.05,.1):
            for yaw in (0.,.12,1.5,3.09):
                w=60;g=(w,w,res,-w*res/2,-w*res/2,0.,tuple(0 if 6<x<54 and 6<y<54 else -1 for y in range(w) for x in range(w)))
                pose=(.008,-.006,yaw);p=plan_frontier(g,pose,[])
                self.assertTrue(p['path'])
                cursor=2 if len(p['path'])>2 and math.dist(p['path'][1],pose[:2])<.08 else 1
                t=tracking_target(p['path'],cursor,pose)
                err=math.atan2(math.sin(math.atan2(t[1]-pose[1],t[0]-pose[0])-yaw),math.cos(math.atan2(t[1]-pose[1],t[0]-pose[0])-yaw))
                self.assertIsNone(body_motion_block(g,pose,t,abs(err)>.35))

class PhysicalExplorerTests(ProductionExplorerTests):
    def test_front_open_rear_unknown_old_heading_blocked_corrected_goes_forward(self):
        for res in (.025,.03):
            w=100;ox=oy=-50*res
            g=(w,w,res,ox,oy,0.,tuple(0 if -.23<(x+.5)*res+ox<.9 and abs((y+.5)*res+oy)<.9 else -1 for y in range(w) for x in range(w)))
            before=tuple(g[6]);raw=(0.,0.,math.pi)
            self.assertFalse(plan_frontier(g,raw,[])['path'])
            pose=physical_pose(raw,math.pi);p=plan_frontier(g,pose,[])
            self.n.grid=g;self.n.pose=lambda:(pose,0);self.n.plan=p;self.n.cursor=1;self.n.checked_map=0
            self.n.tick();m=self.messages[-1]
            self.assertTrue(m['drive']['motion_requested']);self.assertEqual(m['drive']['op'],'drive')
            self.assertGreater(m['drive']['throttle'],0);self.assertFalse(m['recovery_active'])
            self.assertEqual(g[6],before)
    def setup_tf(self):
        n=self.n;n.heading_offset=math.pi
        class FakeTime:
            @staticmethod
            def from_msg(s):return S(nanoseconds=s)
        type(n).pose.__globals__['Time']=FakeTime
        n.get_clock=lambda:S(now=lambda:S(nanoseconds=1_000_000_000))
        def lookup(parent,child,when):
            yaw=math.pi if parent in ('map','odom') else 0.
            return S(header=S(stamp=990_000_000),transform=S(translation=S(x=0.,y=0.,z=0.),rotation=S(z=math.sin(yaw/2),w=math.cos(yaw/2))))
        n.tf=S(lookup_transform=lookup)
    def test_actual_pose_and_odom_methods_share_heading_not_age(self):
        self.setup_tf();n=self.n
        p,age=type(n).pose(n);o=type(n).odom_pose(n)
        self.assertAlmostEqual(p[2],0);self.assertAlmostEqual(o[2],0);self.assertEqual(age,10_000_000)
        r=Retreat();r.begin(o,10)
        self.assertTrue(r.update(grid(),(-.05,0,p[2]),(-.05,0,o[2]),10.3,True)[0])
    def test_actual_clearance_checks_physical_front_after_reexpression(self):
        self.setup_tf();n=self.n
        from acg138_frontier.exploration_motion import scan_clearance
        type(n).clearance.__globals__['scan_clearance']=scan_clearance
        rays=[4.]*360;rays[0]=.25 # raw base -X is physical front
        n.scan=S(header=S(frame_id='laser',stamp=990_000_000),ranges=rays,angle_min=-math.pi,angle_increment=2*math.pi/360,range_min=.05,range_max=12.)
        front,circle,age=type(n).clearance(n)
        self.assertAlmostEqual(front,.25);self.assertAlmostEqual(circle,.25);self.assertEqual(age,10_000_000)
    def test_actual_recovery_rear_scan_uses_same_body_axis(self):
        self.setup_tf();n=self.n
        from acg138_frontier.recovery import rear_scan_clear,reverse_block_detail
        glob=type(n).recovery_step.__globals__;glob['rear_scan_clear']=rear_scan_clear;glob['reverse_block_detail']=reverse_block_detail
        rays=[4.]*360;rays[180]=.25 # raw +X is the physical rear
        n.scan=S(header=S(frame_id='laser',stamp=990_000_000),ranges=rays,angle_min=-math.pi,angle_increment=2*math.pi/360,range_min=.05,range_max=12.)
        moving,reason=type(n).recovery_step(n,(0,0,0),10)
        self.assertFalse(moving);self.assertEqual(reason,'reverse_lidar_blocked_or_unobserved')
        self.assertFalse(n.retreat.active)
        n.scan.ranges=[4.]*360;obstacle(n.grid,-.3,0,-1)
        moving,reason=type(n).recovery_step(n,(0,0,0),10)
        self.assertFalse(moving);self.assertEqual(reason,'reverse_unknown')
        self.assertLess(n.recovery_detail['body_xy'][0],0);self.assertFalse(n.retreat.active)

    def test_stale_tf_still_rejected_after_heading_conversion(self):
        self.setup_tf();self.n.get_clock=lambda:S(now=lambda:S(nanoseconds=1_300_000_000))
        with self.assertRaisesRegex(ValueError,'waiting_fresh_pose'):type(self.n).pose(self.n)
        with self.assertRaisesRegex(ValueError,'reverse_waiting_fresh_odom'):type(self.n).odom_pose(self.n)

if __name__=='__main__':unittest.main()
