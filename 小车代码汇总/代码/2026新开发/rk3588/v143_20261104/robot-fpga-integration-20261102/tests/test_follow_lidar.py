import math
import sys
import unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'acg138_semantic'))
from acg138_semantic.follow_lidar import associate,associate_centered,RangeConfirmation


def scan(objects=(),wall=False,start=-math.pi):
    values=[math.inf]*720
    for i in range(720):
        a=start+i*math.pi/360
        if wall and abs(math.remainder(a,2*math.pi))<.5:values[i]=2/math.cos(a)
        for x,y,radius in objects:
            # Ray / circle intersection provides actual angular extents.
            along=x*math.cos(a)+y*math.sin(a)
            perp=x*math.sin(a)-y*math.cos(a)
            if along>0 and abs(perp)<=radius:
                d=along-math.sqrt(radius*radius-perp*perp)
                values[i]=min(values[i],d)
    return values


def run(values,left=-10,right=10,laser=(0,0,0),body=(0,0,0),previous=None,start=-math.pi):
    return associate(values,start,math.pi/360,.05,12,laser,body,left,right,0,previous)


class FollowAssociationTests(unittest.TestCase):
    def test_person_center(self):
        v=run(scan([(2,0,.15)]))
        self.assertTrue(v['matched']);self.assertAlmostEqual(v['distance_m'],1.86,delta=.04)
        self.assertAlmostEqual(v['bearing_right_deg'],0,delta=.2)
        self.assertTrue(v['forward_clear'] and v['reverse_clear'] and v['turn_clear'])

    def test_right_bearing_sign(self):
        v=run(scan([(2,-.6,.15)]),left=8,right=25)
        self.assertTrue(v['matched']);self.assertGreater(v['bearing_right_deg'],10)

    def test_camera_lateral_origin(self):
        values=scan([(2,.3,.12)])
        v=associate(values,-math.pi,math.pi/360,.05,12,(0,0,0),(0,0,0),-3,3,-.3)
        self.assertTrue(v['matched']);self.assertLess(v['bearing_right_deg'],0)

    def test_wall_is_not_target(self):
        self.assertFalse(run(scan(wall=True))['matched'])

    def test_ambiguous_people(self):
        v=run(scan([(2,-.20,.07),(2,.20,.07)]),left=-12,right=12)
        self.assertFalse(v['matched']);self.assertEqual(v['reason'],'follow_range_ambiguous')

    def test_target_outside_camera_box(self):
        self.assertFalse(run(scan([(2,-1,.15)]))['matched'])

    def test_obstacle_clearance(self):
        v=run(scan([(2,0,.15),(.48,0,.07),(-.45,0,.07),(.1,.31,.04)]))
        self.assertFalse(v['forward_clear']);self.assertFalse(v['reverse_clear']);self.assertFalse(v['turn_clear'])

    def test_missing_scan_sector(self):
        values=scan([(2,0,.15)]);values[:80]=[float('nan')]*80
        v=run(values);self.assertFalse(v['matched']);self.assertFalse(v['turn_clear'])

    def test_identity_jump(self):
        self.assertFalse(run(scan([(3,0,.15)]),previous=(1,0))['matched'])

    def test_moving_chassis_transform(self):
        # Scan in old odom orientation; latest body rotated left by 10 degrees.
        v=run(scan([(2,0,.15)]),left=2,right=18,body=(.1,0,math.radians(10)))
        self.assertTrue(v['matched']);self.assertAlmostEqual(v['bearing_right_deg'],10,delta=1)

    def test_native_lidar_mount_and_physical_front(self):
        # Native forward target at +Y; calibrated laser yaw is -90deg in base,
        # physical body front is base yaw+180deg. Target is native -Y here.
        v=run(scan([(0,-2,.15)]),laser=(0,0,-math.pi/2),body=(0,0,math.pi))
        self.assertTrue(v['matched']);self.assertAlmostEqual(v['bearing_right_deg'],0,delta=.5)

    def test_scan_seam(self):
        self.assertTrue(run(scan([(2,0,.15)],start=0),start=0)['matched'])

    def test_two_legs(self):
        v=run(scan([(2,-.12,.045),(2,.12,.045)]))
        self.assertTrue(v['matched']);self.assertAlmostEqual(v['bearing_right_deg'],0,delta=.5)

    def test_confirmation_needs_distinct_scans(self):
        c=RangeConfirmation();key=(12,1000)
        self.assertFalse(c.update(key,1000000000,True))
        for _ in range(20):self.assertFalse(c.update(key,1000000000,True))
        self.assertFalse(c.update(key,1080000000,True))
        self.assertTrue(c.update(key,1160000000,True))
        self.assertFalse(c.update(key,1200000000,False))
        self.assertFalse(c.update(key,1280000000,True))
        self.assertFalse(c.update(key,1360000000,True))
        self.assertTrue(c.update(key,1440000000,True))
        self.assertFalse(c.update((12,2000),1520000000,True))
        self.assertFalse(c.update((12,2000),1920000000,True))

    def test_uncalibrated_center_then_range(self):
        def centered(values,error=0,yaw=0):
            return associate_centered(values,-math.pi,math.pi/360,.05,12,(0,0,0),(0,0,0),error,yaw)
        values=scan([(2,0,.12)])
        self.assertTrue(centered(values)['matched'])
        v=centered(values,.15)
        self.assertFalse(v['matched']);self.assertTrue(v['turn_clear']);self.assertEqual(v['distance_m'],0)
        self.assertFalse(centered(values,0,3)['matched'])
        self.assertFalse(centered(scan(wall=True))['matched'])
        self.assertFalse(centered(scan([(2,-.20,.04),(2,.20,.04)]))['matched'])
        self.assertFalse(centered(scan([(2,-.8,.12)]))['matched'])

    def test_invalid_geometry(self):
        for values in ([],[1]*20):
            with self.assertRaises(ValueError):run(values)
        with self.assertRaises(ValueError):run(scan(),left=float('nan'))

    def test_four_dm_approach_and_obstacle_corridor(self):
        def observe(objects,previous):
            return associate_centered(scan(objects),-math.pi,math.pi/360,.05,12,
                                      (0,0,0),(0,0,0),0,0,0,previous,.4)
        value=observe([(.65,0,.06)],(.70,0))
        self.assertTrue(value['matched'] and value['forward_clear'])
        value=observe([(.46,0,.06)],(.50,0))
        self.assertTrue(value['matched'] and value['forward_clear'])
        self.assertAlmostEqual(value['distance_m'],.4,delta=.04)
        value=observe([(.46,0,.06),(.32,.15,.03)],(.50,0))
        self.assertFalse(value['forward_clear'])
        value=observe([(.30,0,.05)],(.40,0))
        self.assertFalse(value['forward_clear'])

    def test_four_dm_near_silhouette_still_requires_unique_range(self):
        def observe(values,previous):
            return associate_centered(values,-math.pi,math.pi/360,.05,12,
                                      (0,0,0),(0,0,0),0,0,0,previous,.4)
        self.assertTrue(observe(scan([(.55,0,.15)]),(.6,0))['matched'])
        self.assertFalse(observe(scan(wall=True),(.6,0))['matched'])
        self.assertFalse(observe(scan([(.6,-.22,.04),(.8,.20,.04)]),(.7,0))['matched'])
        for distance in (float('nan'),.3,4):
            with self.assertRaises(ValueError):
                associate_centered(scan(),-math.pi,math.pi/360,.05,12,
                                   (0,0,0),(0,0,0),0,0,desired_distance_m=distance)

    def test_four_dm_tracks_widening_target_from_far_to_near(self):
        for legs in (False,True):
            previous=None
            for i in range(30):
                x=2.-i*.05
                objects=[(x,-.12,.045),(x,.12,.045)] if legs else [(x,0,.15)]
                value=associate_centered(scan(objects),-math.pi,math.pi/360,.05,12,
                                         (0,0,0),(0,0,0),0,0,0,previous,.4)
                self.assertTrue(value['matched'],(legs,x,value))
                previous=(value['x_body_m'],value['y_body_m'])


if __name__=='__main__':unittest.main()
