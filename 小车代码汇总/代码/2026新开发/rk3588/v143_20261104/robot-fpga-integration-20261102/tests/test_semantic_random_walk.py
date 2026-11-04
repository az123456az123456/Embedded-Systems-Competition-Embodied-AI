import math
import unittest

from acg138_frontier.semantic_random_walk import SemanticRandomWalk, wrap


def open_scan(count=360):
    return (tuple(float('inf') for _ in range(count)), -math.pi,
            2 * math.pi / count, .05, 10.0)


class SemanticRandomWalkTests(unittest.TestCase):
    def test_turn_commands_obey_gateway_speed_contract(self):
        directions = set()
        for error in (-math.pi, -1., -.2, .2, 1., math.pi):
            walk = SemanticRandomWalk(seed=1)
            walk.state = walk.ROTATING; walk.target_yaw = error
            ranges=tuple(.32 if abs(-math.pi+i*math.tau/360)<math.radians(20) else 3. for i in range(360))
            command = walk.update((ranges,*open_scan()[1:]), (0., 0., 0.))
            self.assertEqual(command['op'], 'rotate')
            self.assertGreater(command['speed'], 0)
            self.assertLessEqual(command['speed'], .25)
            directions.add(command['direction'])
        self.assertEqual(directions, {'cw', 'ccw'})

    def test_segment_corrections_obey_gateway_steering_contract(self):
        for error in (-math.pi, -1., -.4, -.1, 0., .1, .4, 1., math.pi):
            walk = SemanticRandomWalk(seed=1)
            walk.state = walk.DRIVING_SEGMENT; walk.target_yaw = error
            walk.start_xy = (0., 0.); walk.segment_length = .6
            command = walk.update(open_scan(), (0., 0., 0.))
            self.assertEqual(command['op'], 'drive')
            self.assertLessEqual(abs(command['steer']), .15)
            self.assertLessEqual(command['throttle'], .30)

    def test_positive_infinity_is_open_space(self):
        walk = SemanticRandomWalk(seed=1)
        command = walk.update(open_scan(), (0.0, 0.0, 0.0))
        self.assertTrue(command['motion_requested'])
        self.assertIn(command['op'], ('drive', 'rotate'))

    def test_safe_alternative_does_not_return_to_last_segment_origin(self):
        walk = SemanticRandomWalk(seed=1)
        walk._choose(open_scan(), (0.0, 0.0, 0.0))
        walk.state = walk.SELECT_DIRECTION
        walk._choose(open_scan(), (1.0, 0.0, 0.0))
        chosen_relative = wrap(walk.target_yaw)
        back_relative = math.pi
        self.assertGreaterEqual(abs(wrap(chosen_relative - back_relative)), math.radians(65))

    def test_many_short_segments_do_not_immediately_double_back(self):
        for seed in range(8):
            walk = SemanticRandomWalk(seed=seed)
            pose = (0., 0., 0.)
            walk.update(open_scan(), pose)
            for _ in range(40):
                origin = pose[:2]
                yaw = walk.target_yaw
                walk.update(open_scan(), (*origin, yaw))
                length = walk.segment_length + .001
                pose = (origin[0] + length * math.cos(yaw), origin[1] + length * math.sin(yaw), yaw)
                command = walk.update(open_scan(), pose)
                back = math.atan2(origin[1] - pose[1], origin[0] - pose[0])
                self.assertTrue(command['motion_requested'])
                self.assertGreaterEqual(abs(wrap(walk.target_yaw - back)), math.radians(65))

    def test_dead_end_still_allows_the_only_safe_way_out(self):
        walk = SemanticRandomWalk(seed=1)
        walk.recent_points = [(0., 0.)]
        scan = open_scan()
        ranges = tuple(2. if abs(wrap(-math.pi + i * math.tau / 360 - math.pi)) < math.radians(65)
                       else .32 for i in range(360))
        command = walk.update((ranges, *scan[1:]), (1., 0., 0.))
        self.assertTrue(command['motion_requested'])
        self.assertLess(abs(wrap(walk.target_yaw - math.pi)), math.radians(30))

    def test_segment_direction_is_latched_despite_scan_noise(self):
        walk = SemanticRandomWalk(seed=1)
        walk.update(open_scan(), (0., 0., 0.))
        target, length = walk.target_yaw, walk.segment_length
        for i in range(10):
            scan = open_scan()
            ranges = tuple(8. + ((j + i) % 5) * .1 for j in range(360))
            walk.update((ranges, *scan[1:]), (0., 0., target))
            self.assertEqual(walk.target_yaw, target)
            self.assertEqual(walk.segment_length, length)

    def test_rotation_stops_for_obstacle_or_unobserved_sector(self):
        for value, reason in ((.2, 'rotation_clearance_blocked'), (math.nan, 'rotation_clearance_unobserved')):
            walk = SemanticRandomWalk(seed=1)
            walk.state = walk.ROTATING;walk.target_yaw = 1.
            ranges = [math.inf] * 360;ranges[:30] = [value] * 30;ranges[170:191]=[.32]*21
            command = walk.update((tuple(ranges), *open_scan()[1:]), (0., 0., 0.))
            self.assertFalse(command['motion_requested']);self.assertEqual(command['reason'], reason)

    def test_scan_mount_rotation_and_translation_reexpress_obstacle(self):
        ranges = [math.inf] * 360;ranges[177:184] = [.4] * 7
        walk = SemanticRandomWalk(seed=1)
        sectors = walk._sectors((tuple(ranges), *open_scan()[1:], math.pi/2, .05, 0.))
        blocked = [i for i, (clear, _) in enumerate(sectors) if clear < .5]
        self.assertTrue(blocked)
        for i in blocked:
            angle = -math.pi + (i + .5) * math.tau / walk.sector_count
            self.assertLess(abs(wrap(angle - math.pi/2)), math.radians(20))

    def test_front_corner_obstacle_stops_a_driving_segment(self):
        walk = SemanticRandomWalk(seed=1)
        walk.state=walk.DRIVING_SEGMENT;walk.target_yaw=0.;walk.start_xy=(0.,0.);walk.segment_length=.6
        ranges=[math.inf]*360;ranges[200:206]=[.32]*6
        command=walk.update((tuple(ranges),*open_scan()[1:]),(0.,0.,0.))
        self.assertFalse(command['motion_requested'])
        self.assertEqual(command['reason'],'segment_clearance_blocked')

    def test_open_space_keeps_heading_across_twenty_short_legs(self):
        for seed in (1, 2, 138):
            walk = SemanticRandomWalk(seed=seed)
            pose = (0., 0., 1.1)
            for leg in range(20):
                command = walk.update(open_scan(), pose)
                self.assertEqual(command['op'], 'drive')
                self.assertGreater(command['throttle'], 0.)
                self.assertAlmostEqual(wrap(walk.target_yaw-1.1), 0.)
                length = walk.segment_length+.001
                pose = (pose[0]+length*math.cos(1.1), pose[1]+length*math.sin(1.1), 1.1)
            self.assertEqual(walk.segments, 19)

    def test_turn_finishes_then_drives_without_reselecting(self):
        # Fixed world obstacle. The scan bearing changes as the robot turns;
        # odometry advances under the actual production commands.
        walk = SemanticRandomWalk(seed=138)
        x = y = yaw = 0.
        rotations = 0
        first_target = None
        for tick in range(240):
            ranges = tuple(.32 if abs(wrap(-math.pi+i*math.tau/360+yaw)) < math.radians(20)
                           else 3. for i in range(360))
            command = walk.update((ranges, *open_scan()[1:]), (x, y, yaw))
            if first_target is None:
                first_target = walk.target_yaw
            if command['op'] == 'rotate':
                self.assertAlmostEqual(walk.target_yaw, first_target)
            if command['op'] == 'rotate':
                rotations += 1
                yaw = wrap(yaw + (1 if command['direction']=='ccw' else -1)*command['speed']*2.5*.05)
            elif command['motion_requested']:
                x += .12*math.cos(yaw)*.05
                y += .12*math.sin(yaw)*.05
                yaw = wrap(yaw-command['steer']*2*.05)
            if math.hypot(x,y) >= .3:
                break
        self.assertGreater(rotations, 0)
        self.assertGreaterEqual(math.hypot(x,y), .3)
        self.assertEqual(walk.state, walk.DRIVING_SEGMENT)

    def test_overshot_turn_uses_safe_opening_instead_of_reversing(self):
        walk=SemanticRandomWalk();walk.state=walk.ROTATING;walk.target_yaw=1.
        walk.start_xy=(0.,0.);walk.segment_length=.6
        command=walk.update(open_scan(), (0.,0.,1.2))
        self.assertEqual(command['op'], 'drive')
        self.assertGreater(command['throttle'], 0.)
        self.assertAlmostEqual(walk.target_yaw, 1.2)
        for i in range(12):
            command=walk.update(open_scan(), (.01*i,.01*i,1.2))
            self.assertEqual(command['op'],'drive')
            self.assertGreater(command['throttle'],0.)

    def test_heading_jump_keeps_safe_forward_translation(self):
        walk=SemanticRandomWalk();walk.update(open_scan(),(0.,0.,0.))
        command=walk.update(open_scan(),(.1,0.,.5))
        self.assertEqual(command['op'],'drive')
        self.assertGreater(command['throttle'],0.)
        self.assertAlmostEqual(command['steer'],0.)

    def test_failed_selection_does_not_record_same_origin_repeatedly(self):
        walk=SemanticRandomWalk()
        walk.update(open_scan(), (0.,0.,0.))
        for i in range(5):
            walk.state=walk.SELECT_DIRECTION
            walk.update(open_scan(), (0.,0.,0.))
        self.assertEqual(walk.recent_points, [(0.,0.)])
        self.assertAlmostEqual(walk.target_yaw, 0.)


if __name__ == '__main__':
    unittest.main()
