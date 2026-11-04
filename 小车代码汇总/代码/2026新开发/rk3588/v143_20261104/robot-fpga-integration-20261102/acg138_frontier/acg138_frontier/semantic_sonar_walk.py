"""Continuous forward search; one latched sonar turn per obstacle encounter."""
import math
from .semantic_random_walk import SemanticRandomWalk


def sonar_observation(value, now_ns):
    """SNS1 front/left/right, preserving kernel receive and source ages."""
    try:
        received = int(value['received_mono_ns'])
        packet_age = now_ns-received
        if not 0 <= packet_age <= 250000000:
            raise ValueError('stale_sonar_packet')
        if value['valid_mask'] != 7 or value['fault_mask'] != 0:
            raise ValueError('invalid_sonar')
        ranges, ages = value['distance_mm'], value['age_ms']
        if len(ranges) != 3 or len(ages) != 3:
            raise ValueError('invalid_sonar')
        if any(type(r) is not int or not 30 <= r <= 65534 for r in ranges):
            raise ValueError('invalid_sonar')
        if any(type(a) is not int or a < 0 or packet_age+a*1000000 > 500000000 for a in ages):
            raise ValueError('stale_sonar')
        uptime = value['uptime_ms']
        if type(uptime) is not int or not 0 <= uptime <= 0xffffffff:
            raise ValueError('invalid_sonar')
        return (tuple(min(r, 4500)/1000. for r in ranges),
                tuple((uptime-a) & 0xffffffff for a in ages), max(ages)*1000000+packet_age)
    except (KeyError, TypeError, OverflowError):
        raise ValueError('invalid_sonar') from None


class SemanticSonarWalk:
    FORWARD = 'FORWARD'
    TURNING = 'TURNING'
    BACKING = 'BACKING'
    TRIGGER_M = .15
    REARM_M = .17

    def __init__(self, width=.30, front=.1589, rear=.1942):
        self.width, self.front, self.rear = width, front, rear
        self.lidar = SemanticRandomWalk()
        self.reset()

    def reset(self):
        self.state = self.FORWARD
        self.direction = 'cw'
        self.armed = [True]*3
        self.trigger_front_id = None
        self.turns = 0
        self.turn_trigger = None
        self.hold_yaw = None
        self.backed_distance = 0.
        self.backing_last_pose = None
        self.backing_yaw = None

    @staticmethod
    def zero(reason):
        return dict(op='drive', throttle=0., steer=0., speed=0.,
                    motion_requested=False, reason=reason)

    def _points(self, scan):
        sectors = self.lidar._sectors(scan)
        if any(not math.isfinite(clear) or coverage < .45 for clear, coverage in sectors):
            raise ValueError('lidar_coverage_incomplete')
        ranges, amin, inc, rmin, rmax = scan[:5]
        yaw, tx, ty = scan[5:] if len(scan) == 8 else (0., 0., 0.)
        return [(tx+r*math.cos(amin+i*inc+yaw), ty+r*math.sin(amin+i*inc+yaw))
                for i, r in enumerate(ranges) if math.isfinite(r) and rmin <= r <= rmax]

    def _clear(self, points, turning, sweep_degrees=15):
        # Check the rectangular body over the commanded sweep, so safe turns
        # are admitted even when a corner's bounding circle touches a wall.
        angles = [0.] if not turning else [math.radians(i)*(1 if self.direction=='ccw' else -1)
                                          for i in range(0, sweep_degrees+1, 3)]
        # Stop early enough to leave room for the farthest body corner to
        # rotate, plus the existing .04m forward stopping allowance.
        nose = (self.front+.02 if turning else
                math.hypot(max(self.front,self.rear)+.02,self.width/2+.02)+.04)
        for angle in angles:
            c, s = math.cos(angle), math.sin(angle)
            for x, y in points:
                bx, by = c*x+s*y, -s*x+c*y
                if -self.rear-.02 <= bx <= nose and abs(by) <= self.width/2+.02:
                    return False
        return True

    def _rear_clear(self, points):
        # Verify the whole body swept through the next .10m, including the
        # current footprint. This never treats missing scan data as clearance.
        return not any(-self.rear-.02-.10 <= x <= self.front+.02
                       and abs(y) <= self.width/2+.02 for x,y in points)

    def update(self, scan, odom, ranges, observation_ids):
        if len(ranges) != 3 or any(not math.isfinite(r) or not .03 <= r <= 4.5 for r in ranges):
            raise ValueError('invalid_sonar')
        points = self._points(scan)
        forward_clear = self._clear(points, False)
        front, left, right = ranges
        # A side probe remaining close after the turn is the same obstacle,
        # not a reason to keep alternating turns on every control tick.
        for i, r in enumerate(ranges):
            if r >= self.REARM_M:
                self.armed[i] = True
        triggered = any(r < self.TRIGGER_M and self.armed[i] for i, r in enumerate(ranges))
        if self.state == self.FORWARD and (triggered or not forward_clear):
            self.state = self.TURNING
            self.direction = 'ccw' if left > right else 'cw'
            self.turn_trigger = 'sonar_below_150mm' if triggered else 'forward_body_clearance'
            # Prefer the larger sonar side, but never latch an obstructed
            # sweep when the opposite direction is already safe.
            if not self._clear(points, True):
                preferred = self.direction
                self.direction = 'cw' if preferred == 'ccw' else 'ccw'
                if not self._clear(points, True):
                    self.direction = preferred
            self.trigger_front_id = observation_ids[0]
            self.hold_yaw = None
            self.backed_distance = 0.
            self.backing_last_pose = None
            self.turns += 1
            for i, r in enumerate(ranges):
                if r < self.REARM_M:
                    self.armed[i] = False
            # A trigger must start a turn even if the old front sample already
            # reads >.30m. Only a later front observation can release it.
        elif (self.state in (self.TURNING,self.BACKING) and front > .30 and forward_clear
              and observation_ids[0] != self.trigger_front_id):
            self.state = self.FORWARD
            self.hold_yaw = odom[2]
            self.backing_last_pose = None
            for i, r in enumerate(ranges):
                if r < self.REARM_M:
                    self.armed[i] = False
        if self.state == self.TURNING and not self._clear(points, True):
            self.state = self.BACKING
            self.backing_last_pose = odom
            self.backing_yaw = odom[2]
        if self.state == self.BACKING:
            previous = self.backing_last_pose
            self.backed_distance += math.hypot(odom[0]-previous[0],odom[1]-previous[1])
            self.backing_last_pose = odom
            # Reserve a usable quarter turn before resuming, rather than
            # alternating tiny turns and retreats at the clearance boundary.
            if self._clear(points, True, 90):
                self.state = self.TURNING
                self.backing_last_pose = None
            else:
                if self.backed_distance >= .15:
                    return self.zero('rotation_recovery_distance_limit')
                if not self._rear_clear(points):
                    return self.zero('rotation_recovery_rear_blocked')
                error = math.atan2(math.sin(self.backing_yaw-odom[2]),math.cos(self.backing_yaw-odom[2]))
                if abs(error) > math.radians(10):
                    return self.zero('rotation_recovery_heading_changed')
                return dict(op='drive', throttle=-.12, steer=max(-.10,min(.10,-error*.45)),
                            speed=0., motion_requested=True, reason='semantic_turn_make_room')
        if self.state == self.TURNING:
            return dict(op='rotate', throttle=0., steer=0., speed=.18,
                        direction=self.direction, motion_requested=True, reason='semantic_sonar_turn')
        if self.hold_yaw is None:
            self.hold_yaw = odom[2]
        error = math.atan2(math.sin(self.hold_yaw-odom[2]), math.cos(self.hold_yaw-odom[2]))
        if abs(error) > math.radians(20):
            self.hold_yaw = odom[2]
            error = 0.
        return dict(op='drive', throttle=.18, steer=max(-.15, min(.15, -error*.45)),
                    speed=0., motion_requested=True, reason='semantic_sonar_forward')
