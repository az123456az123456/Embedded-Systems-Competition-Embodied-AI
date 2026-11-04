"""Small, deterministic lidar/odometry random-walk controller for semantic search.

The controller owns a short segment at a time.  A fresh scan is used only when
choosing a segment and while guarding the segment; odometry owns completion.
This prevents scan noise from changing the command every timer tick.
"""
import math
import random


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class SemanticRandomWalk:
    SELECT_DIRECTION = 'SELECT_DIRECTION'
    ROTATING = 'ROTATING'
    DRIVING_SEGMENT = 'DRIVING_SEGMENT'
    STUCK_RECOVERY = 'STUCK_RECOVERY'

    def __init__(self, seed=138, sector_count=24, segment_min=.4, segment_max=.9):
        self.rng = random.Random(seed)
        self.sector_count = sector_count
        self.segment_min, self.segment_max = segment_min, segment_max
        self.state = self.SELECT_DIRECTION
        self.target_yaw = None
        self.start_xy = None
        self.segment_length = 0.
        self.last_heading = None
        self.recent_points = []
        self.segments = 0

    def reset(self):
        self.__init__(self.rng.randrange(1 << 30), self.sector_count,
                      self.segment_min, self.segment_max)

    def _sectors(self, scan):
        ranges, amin, inc, rmin, rmax = scan[:5]
        yaw, tx, ty = scan[5:] if len(scan) == 8 else (0., 0., 0.)
        if (not ranges or not all(math.isfinite(v) for v in (amin, inc, rmin, rmax, yaw, tx, ty))
                or inc == 0 or not 0 <= rmin < rmax):
            raise ValueError('invalid_lidar_scan')
        if len(ranges) < self.sector_count * 4:
            raise ValueError('lidar_coverage_incomplete')
        width = 2 * math.pi / self.sector_count
        points = []
        for i, value in enumerate(ranges):
            # No return is open only within the scanner's declared range.
            if math.isinf(value) and value > 0:
                value = rmax
            if not math.isfinite(value) or not rmin <= value <= rmax:
                continue
            angle = amin + i * inc + yaw
            x, y = tx + value * math.cos(angle), ty + value * math.sin(angle)
            points.append((math.atan2(y, x), math.hypot(x, y)))
        result = []
        for sector in range(self.sector_count):
            center = -math.pi + (sector + .5) * width
            values = []
            for bearing, value in points:
                angle = wrap(bearing - center)
                if abs(angle) > width * .55:
                    continue
                values.append(value)
            if len(values) < 2:
                result.append((float('-inf'), 0.))
            else:
                result.append((min(values), len(values) / max(1, int(width / abs(inc)))))
        return result

    def _path_clear(self, sectors, rel):
        width = 2 * math.pi / self.sector_count
        return all(clear >= .35 and coverage >= .45
                   for i, (clear, coverage) in enumerate(sectors)
                   if abs(wrap(-math.pi + (i + .5) * width - rel)) <= math.pi / 4 + 1e-9)

    def _directions(self, sectors, odom):
        width = 2 * math.pi / self.sector_count
        px, py, yaw = odom
        # Headings live in odometry coordinates. Reusing a body-relative
        # sector after every turn makes the next leg turn by the same angle.
        rels = [wrap(-math.pi + (i + .5) * width) for i in range(self.sector_count)]
        preferred = wrap(self.last_heading - yaw) if self.last_heading is not None else 0.
        rels += [0.]
        if abs(preferred) <= math.radians(20):
            rels += [preferred]
        candidates = [rel for rel in rels if self._path_clear(sectors, rel)]
        if not candidates:
            raise ValueError('no_lidar_direction')
        # Only a displaced origin identifies the way we came from. A failed
        # selection at the same XY must not turn the forward direction into
        # a forbidden "return" direction.
        origins = [(x, y) for x, y in self.recent_points if math.hypot(x-px, y-py) >= .2]
        if origins:
            ox, oy = origins[-1]
            back = wrap(math.atan2(oy-py, ox-px)-yaw)
            alternatives = [rel for rel in candidates
                            if abs(wrap(rel-back)) >= math.radians(65)]
            if alternatives:
                candidates = alternatives
        return candidates

    def _choose(self, scan, odom):
        sectors = self._sectors(scan)
        candidates = self._directions(sectors, odom)
        width = 2 * math.pi / self.sector_count
        px, py, yaw = odom
        preferred = wrap(self.last_heading-yaw) if self.last_heading is not None else 0.
        origins = [(x, y) for x, y in self.recent_points if math.hypot(x-px, y-py) >= .2]
        # Keep a safe heading over short-leg boundaries. Random selection is
        # needed when that route is blocked, not on every completed leg.
        if abs(preferred) <= math.radians(20) and preferred in candidates:
            rel = preferred
        elif 0. in candidates:
            rel = 0.
        else:
            forward = [rel for rel in candidates if abs(rel) <= math.radians(120)]
            if forward:
                candidates = forward
            # Pick among the nearest openings; a larger distant range must
            # not keep winning over a nearby way out and extend the turn.
            nearest = min(abs(rel) for rel in candidates)
            candidates = [rel for rel in candidates if abs(rel) <= nearest + math.radians(15)]
            def score(rel):
                i = int((rel + math.pi) / width) % self.sector_count
                value = min(sectors[i][0], 1.5) - .65 * abs(rel) + self.rng.random() * .08
                end = (px + .6 * math.cos(yaw+rel), py + .6 * math.sin(yaw+rel))
                if any(math.hypot(end[0]-x, end[1]-y) < .4 for x, y in origins):
                    value -= 1.25
                return value
            rel = max(candidates, key=score)
        if not self.recent_points or math.hypot(px-self.recent_points[-1][0], py-self.recent_points[-1][1]) >= .2:
            self.recent_points = (self.recent_points + [(px, py)])[-6:]
        self.target_yaw = self.last_heading = wrap(yaw + rel)
        self.start_xy = (px, py)
        self.segment_length = self.rng.uniform(self.segment_min, self.segment_max)
        self.state = self.ROTATING if abs(rel) > math.radians(8) else self.DRIVING_SEGMENT

    def update(self, scan, odom):
        """Return a normalized command; caller must reject stale inputs first."""
        if self.state == self.SELECT_DIRECTION:
            self._choose(scan, odom)
        sectors = self._sectors(scan)
        if self.state == self.ROTATING:
            error = wrap(self.target_yaw - odom[2])
            # The job is to reach a safe passage, not settle on an exact
            # yaw. Hand off as soon as forward motion is safe and is not an
            # immediate return toward the previous leg's origin. This also
            # handles stepping across a target angle between odom samples.
            if 0. in self._directions(sectors, odom):
                self.target_yaw = self.last_heading = odom[2]
                self.start_xy = odom[:2]
                self.state = self.DRIVING_SEGMENT
            else:
                if any(not math.isfinite(clear) or coverage < .45 for clear, coverage in sectors):
                    return {'op': 'drive', 'throttle': 0., 'steer': 0., 'speed': 0.,
                            'motion_requested': False, 'reason': 'rotation_clearance_unobserved'}
                circle = min(clear for clear, _ in sectors)
                if circle < .28:
                    return {'op': 'drive', 'throttle': 0., 'steer': 0., 'speed': 0.,
                            'motion_requested': False, 'reason': 'rotation_clearance_blocked'}
                if abs(error) <= math.radians(5):
                    # The opening changed before we reached it: use today's
                    # nearest safe opening instead of chasing the old yaw.
                    self._choose(scan, odom)
                    error = wrap(self.target_yaw-odom[2])
                return {'op': 'rotate', 'throttle': 0., 'steer': 0.,
                        # Gateway exploration speed is normalized and capped
                        # at .25; use the existing mapping turn ceiling (.22).
                        'speed': min(.22, max(.16, abs(error) * .8)),
                        'direction': 'ccw' if error > 0 else 'cw',
                        'motion_requested': True, 'reason': 'semantic_random_rotate'}
        if self.state == self.DRIVING_SEGMENT:
            distance = math.hypot(odom[0] - self.start_xy[0], odom[1] - self.start_xy[1])
            if distance >= self.segment_length:
                self.segments += 1
                self.state = self.SELECT_DIRECTION
                return self.update(scan, odom)
            rel = wrap(self.target_yaw - odom[2])
            if abs(rel) > math.radians(20) and self._path_clear(sectors, 0.):
                # A safe forward leg need not interrupt translation just to
                # regain an exact yaw after slip or an odometry correction.
                self.target_yaw = self.last_heading = odom[2]
                rel = 0.
            if not self._path_clear(sectors, 0.) or not self._path_clear(sectors, rel):
                self.state = self.SELECT_DIRECTION
                return {'op': 'drive', 'throttle': 0., 'steer': 0., 'speed': 0.,
                        'motion_requested': False, 'reason': 'segment_clearance_blocked'}
            return {'op': 'drive', 'throttle': .18, 'steer': max(-.15, min(.15, -rel * .45)),
                    'speed': 0., 'motion_requested': True, 'reason': 'semantic_random_drive'}
        return {'op': 'drive', 'throttle': 0., 'steer': 0., 'speed': 0.,
                'motion_requested': False, 'reason': 'semantic_random_wait'}

