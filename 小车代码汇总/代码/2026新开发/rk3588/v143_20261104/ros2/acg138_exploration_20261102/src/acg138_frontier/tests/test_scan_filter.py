import math,unittest
from acg138_frontier.scan_filter import MappingScanFilter

class ScanFilterTests(unittest.TestCase):
    def setUp(self):self.f=MappingScanFilter();self.t=1000000000
    def run_scan(self,values,pose=(0,0,0),**kw):
        self.t+=100000000
        args=dict(stamp_ns=self.t,now_ns=self.t+10000000,pose=pose,ranges=values,angle_min=-math.pi,
                  angle_increment=2*math.pi/len(values),range_min=.05,range_max=12.)
        args.update(kw);return self.f.process(**args)
    def test_single_spike_suppressed_without_fabricating_free(self):
        a=[float('inf')]*500;a[250]=2.
        self.assertTrue(math.isnan(self.run_scan(a)[250]))
        self.assertEqual(self.run_scan(a)[250],2.) # Persistent thin object retained.
    def test_close_pole_always_retained(self):
        a=[float('nan')]*500;a[250]=.2
        self.assertEqual(self.run_scan(a)[250],.2)
    def test_wall_not_averaged_across_corner(self):
        a=[2.]*250+[4.]*250
        self.assertEqual(self.run_scan(a),a)
    def test_missing_returns_are_nan_not_max_range(self):
        a=[float('inf'),float('nan'),0.,13.]*125
        self.assertTrue(all(math.isnan(v) for v in self.run_scan(a)))
    def test_turn_uses_odom_correspondence_not_same_ray_index(self):
        a=[float('inf')]*500;a[250]=2.
        self.run_scan(a)
        b=[float('inf')]*500;b[240]=2.
        self.assertEqual(self.run_scan(b,(0,0,20*math.pi/500))[240],2.)
    def test_translation_uses_world_points(self):
        a=[float('inf')]*500;a[250]=2.;self.run_scan(a)
        a[250]=1.8
        self.assertEqual(self.run_scan(a,(.2,0,0))[250],1.8)
    def test_different_world_point_not_confirmed(self):
        a=[float('inf')]*500;a[250]=2.;self.run_scan(a)
        self.assertTrue(math.isnan(self.run_scan(a,(.2,0,0))[250]))
    def test_duplicate_stale_gap(self):
        a=[float('inf')]*500;a[250]=2.;self.run_scan(a);old=self.t
        with self.assertRaises(ValueError):self.run_scan(a,stamp_ns=old)
        with self.assertRaises(ValueError):self.run_scan(a,now_ns=self.t+500000000)
        self.t+=300000000
        self.assertTrue(math.isnan(self.run_scan(a)[250]))
    def test_wrap_neighbours_only_for_full_scan(self):
        a=[float('inf')]*500;a[0]=a[-1]=2.
        v=self.run_scan(a);self.assertEqual(v[0],2.);self.assertEqual(v[-1],2.)
        self.f=MappingScanFilter()
        self.assertTrue(math.isnan(self.run_scan(a,angle_increment=.001)[0]))
    def test_small_geometry_rejected(self):
        with self.assertRaises(ValueError):self.run_scan([1.,1.])

if __name__=='__main__':unittest.main()
