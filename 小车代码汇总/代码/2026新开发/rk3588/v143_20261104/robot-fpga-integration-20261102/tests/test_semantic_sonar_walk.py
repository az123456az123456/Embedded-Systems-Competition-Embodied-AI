import copy
import math
import unittest
from acg138_frontier.semantic_sonar_walk import SemanticSonarWalk, sonar_observation


def scan(value=math.inf):
    return (tuple([value]*360), -math.pi, math.tau/360, .05, 10.)


def wall_scan(distance=.2185, pose=(0.,0.,0.)):
    # A fixed world wall x=distance, observed from the moving body frame.
    values=[]
    for i in range(720):
        c=math.cos(-math.pi+i*math.tau/720+pose[2])
        values.append((distance-pose[0])/c if c>0 else math.inf)
    return (tuple(values),-math.pi,math.tau/720,.05,12.)


def snapshot(**overrides):
    v=dict(received_mono_ns='100000000000',valid_mask=7,fault_mask=0,
           distance_mm=[1000,1000,1000],age_ms=[0,10,20],uptime_ms=1000)
    v.update(overrides)
    return v


class SemanticSonarWalkTests(unittest.TestCase):
    def update(self, ranges, ids=(1,1,1), yaw=0., lidar=None):
        return self.walk.update(scan() if lidar is None else lidar,(0.,0.,yaw),ranges,ids)

    def setUp(self):
        self.walk=SemanticSonarWalk()

    def test_long_safe_search_never_randomly_turns(self):
        for i in range(200):
            command=self.update((.15,.5,.8),ids=(i,i,i))
            self.assertEqual(command['op'],'drive')
            self.assertGreater(command['throttle'],0)
        self.assertEqual(self.walk.turns,0)

    def test_any_probe_below_150mm_triggers_toward_larger_side(self):
        for ranges, direction in [((.149,.6,.4),'ccw'),((.6,.149,.4),'cw'),((.6,.4,.149),'ccw')]:
            with self.subTest(ranges=ranges):
                self.walk.reset();command=self.update(ranges)
                self.assertEqual(command['op'],'rotate');self.assertEqual(command['direction'],direction)
                self.assertLessEqual(command['speed'],.25)
                self.assertEqual(self.walk.turn_trigger,'sonar_below_150mm')

    def test_equal_sides_have_stable_direction(self):
        command=self.update((.09,.4,.4));direction=command['direction']
        for i in range(1,10):
            command=self.update((.2,.4,.4),(i,i,i))
            self.assertEqual(command['direction'],direction)

    def test_turn_direction_latched_while_side_order_flips(self):
        self.update((.09,.6,.4))
        for i in range(20):
            command=self.update((.25,.3 if i%2 else .5,.5 if i%2 else .3),(i,i,i))
            self.assertEqual(command['op'],'rotate');self.assertEqual(command['direction'],'ccw')

    def test_turn_releases_strictly_above_300mm_without_zero_frame(self):
        self.update((.09,.6,.4))
        self.assertEqual(self.update((.3,.5,.4),(2,2,2))['op'],'rotate')
        command=self.update((.301,.5,.4),(3,3,3))
        self.assertEqual(command['op'],'drive');self.assertGreater(command['throttle'],0)
        self.assertEqual(self.walk.state,self.walk.FORWARD)

    def test_old_front_sample_does_not_release_side_trigger(self):
        command=self.update((.5,.09,.7))
        self.assertEqual(command['op'],'rotate')
        self.assertEqual(self.update((.5,.09,.7),(1,2,2))['op'],'rotate')
        self.assertEqual(self.update((.5,.09,.7),(2,2,2))['op'],'drive')

    def test_persistent_close_side_does_not_trigger_repeated_turns(self):
        self.update((.5,.09,.7));self.update((.5,.09,.7),(2,2,2))
        for i in range(3,50):
            self.assertEqual(self.update((.5,.09,.7),(i,i,i))['op'],'drive')
        self.assertEqual(self.walk.turns,1)
        self.update((.5,.17,.7),(51,51,51))
        self.assertEqual(self.update((.5,.09,.7),(52,52,52))['op'],'rotate')
        self.assertEqual(self.walk.turns,2)

    def test_side_between_trigger_and_rearm_does_not_repeat_turns(self):
        self.update((.5,.149,.7));self.update((.5,.149,.7),(2,2,2))
        for i in range(3,30):
            self.assertEqual(self.update((.5,.16 if i%2 else .149,.7),(i,i,i))['op'],'drive')
        self.assertEqual(self.walk.turns,1)

    def test_body_guard_still_stops_when_both_sweeps_are_blocked(self):
        command=self.update((.2,.5,.5),lidar=scan(.18))
        self.assertFalse(command['motion_requested']);self.assertEqual(self.walk.turns,1)
        self.assertEqual(command['reason'],'rotation_recovery_rear_blocked')

    def test_lidar_stops_before_sonar_threshold_and_enters_turn(self):
        ranges=[math.inf]*360;ranges[180]=.21
        lidar=(tuple(ranges),*scan()[1:])
        command=self.update((.21,.6,.4),lidar=lidar)
        self.assertEqual(command['op'],'rotate');self.assertEqual(command['direction'],'ccw')
        self.assertEqual(self.walk.turn_trigger,'forward_body_clearance')
        # A newer front sonar alone cannot undo the lidar-triggered turn.
        command=self.update((.4,.6,.4),(2,2,2),lidar=lidar)
        self.assertEqual(command['op'],'rotate');self.assertEqual(self.walk.turns,1)
        self.assertEqual(self.update((.4,.6,.4),(3,3,3))['op'],'drive')

    def test_initial_turn_uses_safe_opposite_sweep_when_preferred_is_blocked(self):
        # Front-left point lies outside the body now, but enters it on a
        # left sweep. Right sweep moves the same point away from the body.
        ranges=[math.inf]*360;ranges[230]=.23
        command=self.update((.09,.6,.4),lidar=(tuple(ranges),*scan()[1:]))
        self.assertEqual(command['op'],'rotate');self.assertEqual(command['direction'],'cw')

    def test_rotation_uses_actual_short_body_sweep(self):
        # A nose obstacle 24.5cm from body origin is .10m from the
        # front sonar; it is outside this first clear rotation sweep.
        ranges=[math.inf]*360;ranges[180]=.245
        command=self.update((.09,.6,.4),lidar=(tuple(ranges),*scan()[1:]))
        self.assertEqual(command['op'],'rotate');self.assertTrue(command['motion_requested'])
        ranges=[math.inf]*360;ranges[180]=.17
        command=self.update((.09,.6,.4),lidar=(tuple(ranges),*scan()[1:]))
        self.assertFalse(command['motion_requested']);self.assertEqual(command['reason'],'rotation_recovery_rear_blocked')

    def test_stops_early_enough_to_reserve_turning_room(self):
        command=self.update((.18,.6,.4),lidar=wall_scan(.31))
        self.assertEqual(command['op'],'rotate')
        self.assertEqual(self.walk.turn_trigger,'forward_body_clearance')

    def test_two_degree_wall_stall_backs_safely_then_finishes_same_turn(self):
        initial=self.walk.update(wall_scan(),(0.,0.,0.),(.09,.6,.4),(1,1,1))
        self.assertEqual(initial['op'],'rotate');self.assertEqual(initial['direction'],'ccw')
        yaw=math.radians(2)
        for i,distance in enumerate((0.,.02,.04,.06)):
            pose=(-distance*math.cos(yaw),-distance*math.sin(yaw),yaw)
            command=self.walk.update(wall_scan(pose=pose),pose,(.09,.6,.4),(i+2,)*3)
            if distance==0.:
                self.assertLess(command['throttle'],0.)
                self.assertEqual(command['reason'],'semantic_turn_make_room')
            self.assertTrue(command['motion_requested'])
            self.assertEqual(self.walk.direction,'ccw')
        self.assertEqual(command['op'],'rotate')
        self.assertEqual(self.walk.turns,1)
        pose=(*pose[:2],math.radians(80))
        command=self.walk.update(wall_scan(pose=pose),pose,(.4,.6,.4),(9,9,9))
        self.assertGreater(command['throttle'],0.)
        self.assertEqual(self.walk.state,self.walk.FORWARD)

    def test_rear_obstacle_prevents_make_room_and_removal_resumes_it(self):
        values=list(wall_scan()[0]);values[0]=.25
        blocked=(tuple(values),*wall_scan()[1:])
        self.update((.09,.6,.4),lidar=blocked)
        values=list(wall_scan(pose=(0.,0.,math.radians(2)))[0]);values[0]=.25
        blocked=(tuple(values),*wall_scan()[1:])
        command=self.update((.09,.6,.4),yaw=math.radians(2),lidar=blocked)
        self.assertFalse(command['motion_requested'])
        self.assertEqual(command['reason'],'rotation_recovery_rear_blocked')
        command=self.update((.09,.6,.4),yaw=math.radians(2),lidar=wall_scan(pose=(0.,0.,math.radians(2))))
        self.assertLess(command['throttle'],0.)
        self.assertEqual(self.walk.direction,'ccw')

    def test_recovery_has_total_odometry_limit_per_obstacle(self):
        for x in (0.,-.05,-.1,-.15,-.16):
            # An approaching obstacle keeps the turn blocked despite retreat.
            command=self.walk.update(wall_scan(.18),(x,0.,0.),(.09,.6,.4),(1,1,1))
            if x>-.15:
                self.assertLess(command['throttle'],0.)
            else:
                self.assertFalse(command['motion_requested'])
                self.assertEqual(command['reason'],'rotation_recovery_distance_limit')
        self.assertEqual(self.walk.turns,1)

    def test_recovery_requires_full_scan_coverage(self):
        self.update((.09,.6,.4),lidar=wall_scan(.18))
        with self.assertRaises(ValueError):
            self.update((.09,.6,.4),lidar=scan(math.nan))

    def test_missing_scan_is_never_free_space(self):
        with self.assertRaises(ValueError):
            self.update((1.,1.,1.),lidar=scan(math.nan))

    def test_turn_front_release_ignores_exact_odom_angle(self):
        self.update((.09,.6,.4))
        command=self.update((.4,.5,.4),(2,2,2),yaw=1.05)
        self.assertEqual(command['op'],'drive');self.assertEqual(command['steer'],0.)


class SonarObservationTests(unittest.TestCase):
    def test_order_numeric_censoring_and_source_age_preserved(self):
        ranges,ids,age=sonar_observation(snapshot(distance_mm=[99,400,65534]),100050000000)
        self.assertEqual(ranges,(.099,.4,4.5));self.assertEqual(ids,(1000,990,980));self.assertEqual(age,70000000)

    def test_expiry_includes_queue_and_per_probe_source_age(self):
        for kwargs,now in [(dict(),100250000001),(dict(age_ms=[0,0,451]),100050000000),
                           (dict(received_mono_ns='100100000000'),100000000000)]:
            with self.subTest(kwargs=kwargs):
                with self.assertRaises(ValueError):sonar_observation(snapshot(**kwargs),now)

    def test_invalid_probe_and_unknown_never_become_open(self):
        for change in [dict(valid_mask=3),dict(fault_mask=1),dict(distance_mm=[0,500,500]),
                       dict(distance_mm=[65535,500,500]),dict(age_ms=[0,-1,0]),dict(distance_mm=[math.inf,500,500])]:
            with self.subTest(change=change):
                with self.assertRaises(ValueError):sonar_observation(snapshot(**change),100000000000)

    def test_same_observation_id_survives_packet_updates_and_wrap(self):
        _,ids,_=sonar_observation(snapshot(),100000000000)
        v=snapshot(uptime_ms=1050,age_ms=[50,60,70])
        self.assertEqual(sonar_observation(v,100000000000)[1],ids)
        v=snapshot(uptime_ms=5,age_ms=[10,10,10])
        self.assertEqual(sonar_observation(v,100000000000)[1][0],0xfffffffb)
