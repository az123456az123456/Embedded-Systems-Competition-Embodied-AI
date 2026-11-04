import math,time,unittest
from types import SimpleNamespace as S
from acg138_frontier.startup_escape import StartupEscape,startup_admission
from test_live_route import ProductionExplorerTests,obstacle


def grid():return (120,120,.025,-1.5,-1.5,0.,[0]*14400)
def admit(g,pose=(0,0,0),rays=None):
    return startup_admission(g,pose,rays if rays is not None else [4.]*500,-math.pi,2*math.pi/500,.05,12.,0.,0.,0.)

class AdmissionTests(unittest.TestCase):
    def test_small_unknown_in_step_with_raw_clear_rays(self):
        g=grid();obstacle(g,.22,.10,-1);before=list(g[6]);ok,d=admit(g)
        self.assertTrue(ok);self.assertGreater(d['temporary_cells'],0);self.assertEqual(g[6],before)
    def test_isolated_map_speckle_requires_measured_free_rays(self):
        g=grid();obstacle(g,.22,.1);self.assertTrue(admit(g)[0])
    def test_actual_front_return_vetoes_even_on_white_map(self):
        rays=[4.]*500;rays[250]=.25
        self.assertFalse(admit(grid(),rays=rays)[0])
    def test_nan_sector_does_not_certify_unknown(self):
        g=grid();obstacle(g,.22,.1,-1);rays=[math.nan]*500
        self.assertFalse(admit(g,rays=rays)[0])
    def test_infinite_no_return_does_not_certify_unknown(self):
        g=grid();obstacle(g,.22,.1,-1);self.assertFalse(admit(g,rays=[math.inf]*500)[0])
    def test_larger_occupied_cluster_still_blocks(self):
        g=grid()
        for x,y in ((.20,.10),(.225,.10),(.20,.125)):obstacle(g,x,y)
        self.assertFalse(admit(g)[0])
    def test_large_unknown_region_still_blocks(self):
        g=grid()
        for y in range(54,66):
            for x in range(66,70):g[6][y*120+x]=-1
        self.assertFalse(admit(g)[0])
    def test_rotated_world_and_sensor_convention(self):
        g=grid();obstacle(g,-.22,-.1,-1)
        self.assertTrue(admit(g,(0,0,math.pi))[0])
    def test_map_boundary_never_ignored(self):
        g=(10,10,.025,-.125,-.125,0.,[0]*100);self.assertFalse(admit(g)[0])
    def test_one_use_and_start_window(self):
        s=StartupEscape(10);self.assertFalse(s.begin((0,0,0),23))
        s=StartupEscape(10);self.assertTrue(s.begin((0,0,0),11));s.finish();self.assertFalse(s.begin((0,0,0),12))
    def test_8cm_displacement_stops(self):
        s=StartupEscape(10);s.begin((0,0,0),10);self.assertEqual(s.progress((.081,0,0),10.5),'startup_complete_replan');self.assertFalse(s.active)
    def test_frozen_robot_times_out(self):
        s=StartupEscape(10);s.begin((0,0,0),10);self.assertEqual(s.progress((0,0,0),13),'startup_step_timeout')
    def test_heading_change_stops(self):
        s=StartupEscape(10);s.begin((0,0,0),10);self.assertEqual(s.progress((0,0,.13),10.5),'startup_heading_deviation')

class StartupProductionTests(ProductionExplorerTests):
    def setup_startup(self):
        n=self.n;n.grid=grid();n.startup=StartupEscape(time.monotonic());n.startup_origin=None
        n.scan=S(header=S(frame_id='laser'),ranges=[4.]*500,angle_min=-math.pi,angle_increment=2*math.pi/500,range_min=.05,range_max=12.)
        n.tf=S(lookup_transform=lambda *a:S(transform=S(translation=S(x=0.,y=0.),rotation=S(z=0.,w=1.))))
        type(n).startup_step.__globals__.update(startup_admission=startup_admission,Time=lambda:0)
    def test_first_map_speckle_permits_one_limited_step(self):
        self.setup_startup();obstacle(self.n.grid,.22,.1,-1)
        self.n.tick();m=self.messages[-1]
        self.assertEqual(m['reason'],'startup_lidar_verified_step');self.assertEqual(m['drive']['throttle'],.12)
        self.assertTrue(m['startup_escape']['used']);self.assertTrue(m['drive']['motion_requested'])
        self.n.odom_pose=lambda:(.09,0,0);self.n.tick();m=self.messages[-1]
        self.assertEqual(m['reason'],'startup_complete_replan');self.assertEqual(m['drive']['throttle'],0)
        self.assertFalse(m['startup_escape']['active']);self.assertTrue(m['startup_escape']['used'])
    def test_startup_near_live_return_keeps_zero(self):
        self.setup_startup();obstacle(self.n.grid,.22,.1,-1);self.n.clearance=lambda:(.25,.25,0)
        self.n.tick();self.assertFalse(self.messages[-1]['drive']['motion_requested'])
    def test_after_first_translation_no_startup_grace(self):
        self.setup_startup();self.n.tick();self.n.odom_pose=lambda:(.10,0,0);self.n.tick()
        self.assertTrue(self.n.startup.used)
        obstacle(self.n.grid,.22,.1,-1);self.n.tick();self.assertFalse(self.messages[-1]['drive']['motion_requested'])
