"""Camera cone / raw planar lidar association, independent of ROS and maps.

Coordinates: x forward, y left in the physical body; optical angles positive
right. A cluster is an association hypothesis, never semantic lidar recognition.
"""
import math
import statistics


def finite(v):
    return type(v) in (int, float) and math.isfinite(v)


def associate(ranges, angle_min, increment, range_min, range_max, laser_pose,
              body_pose, ray_left_deg, ray_right_deg, camera_right_m=0., previous=None,
              desired_distance_m=1.):
    """Both poses are in odom, at the original scan and current pose timestamps.

    Reject incomplete coverage, multiple plausible objects, implausible jumps,
    and wide background surfaces. Invalid returns never count as clear space.
    """
    values = (angle_min, increment, range_min, range_max, *laser_pose, *body_pose,
              ray_left_deg, ray_right_deg, camera_right_m, desired_distance_m)
    if (not all(finite(v) for v in values) or len(ranges) < 180 or len(ranges) > 20000
            or not .001 <= abs(increment) <= .1 or range_min < 0 or range_max < 3
            or abs(increment)*len(ranges) < 6.0
            or not -85 < ray_left_deg < ray_right_deg < 85
            or not .4 <= desired_distance_m <= 3):
        raise ValueError('invalid_follow_scan_geometry')
    lx, ly, lyaw = laser_pose
    bx, by, byaw = body_pose
    c, s = math.cos(byaw), math.sin(byaw)
    points = []
    coverage = [False]*36
    for i, distance in enumerate(ranges):
        angle = angle_min+i*increment
        # +inf is the scanner's explicit no-return value, valid up to range_max.
        clear = distance == math.inf
        if not clear and (not finite(distance) or not range_min <= distance <= range_max):
            points.append(None)
            continue
        body_angle = math.remainder(angle+lyaw-byaw, 2*math.pi)
        coverage[int((body_angle+math.pi)/(2*math.pi)*36) % 36] = True
        if clear:
            points.append(None)
            continue
        ox = lx+distance*math.cos(angle+lyaw)-bx
        oy = ly+distance*math.sin(angle+lyaw)-by
        points.append((c*ox+s*oy, -s*ox+c*oy))
    result = dict(matched=False, distance_m=0., bearing_right_deg=0.,
                  x_body_m=0., y_body_m=0., forward_clear=False,
                  reverse_clear=False, turn_clear=False, reason='follow_wait_range')
    # Require all sectors: a side/rear blind spot cannot authorize a rotation.
    if not all(coverage):
        result['reason'] = 'follow_lidar_coverage'
        return result
    physical = [p for p in points if p is not None and
                not (-.1942 <= p[0] <= .1589 and abs(p[1]) <= .15)]
    # The original 70cm corridor would classify a 40cm follow target as an
    # obstacle before reaching the requested distance. Bound the near corridor
    # by the setpoint, while keeping all returns (including the target) checked.
    forward_stop_m = min(.70, desired_distance_m)
    result['forward_clear'] = not any(0 < x < forward_stop_m and abs(y) < .24 for x, y in physical)
    result['reverse_clear'] = not any(-.65 < x < 0 and abs(y) < .24 for x, y in physical)
    result['turn_clear'] = not any(math.hypot(x, y) < .36 for x, y in physical)
    # Small calibration margin, bounded so narrow PPE boxes do not admit the
    # whole scene. The central ray remains the ranking constraint.
    center = (ray_left_deg+ray_right_deg)/2
    half = min(25., (ray_right_deg-ray_left_deg)/2+1.)
    groups, group = [], []
    for point in points+[None]:
        accepted = False
        if point is not None:
            x, y = point
            ray = math.degrees(math.atan2(-y-camera_right_m, x))
            accepted = x > .1 and abs(ray-center) <= half and math.hypot(x, y) <= 12
        if not accepted or (group and math.dist(group[-1], point) > .16):
            if group:
                groups.append(group)
            group = []
        if accepted:
            group.append(point)
    candidates = []
    # Laser angle zero is arbitrary. Join a cluster crossing the scan seam.
    if len(groups)>1 and points[0] is not None and points[-1] is not None and math.dist(groups[0][0],groups[-1][-1])<=.16:
        groups[0]=groups[-1]+groups[0]
        groups.pop()
    for cluster in groups:
        if len(cluster) < 3:
            continue
        x = statistics.median(p[0] for p in cluster)
        y = statistics.median(p[1] for p in cluster)
        width = max(p[1] for p in cluster)-min(p[1] for p in cluster)
        distance = math.hypot(x, y)
        ray = math.degrees(math.atan2(-y-camera_right_m, x))
        error = abs(ray-center)
        # A broad surface extending across the image cone is not evidence of a
        # person. Reject it instead of treating a wall as the selected object.
        touches_left = any(math.degrees(math.atan2(-p[1]-camera_right_m, p[0])) < center-half+.6 for p in cluster)
        touches_right = any(math.degrees(math.atan2(-p[1]-camera_right_m, p[0])) > center+half-.6 for p in cluster)
        center_fraction = 1. if desired_distance_m < .8 and previous is not None else .65
        if width > 1.2 or (touches_left and touches_right) or error > max(2., half*center_fraction):
            continue
        if previous is not None and math.dist((x,y), previous) > .60:
            continue
        candidates.append((error, distance, x, y))
    candidates.sort()
    if not candidates:
        return result
    if len(candidates)==2 and abs(candidates[0][2]-candidates[1][2])<.15 and math.dist(candidates[0][2:],candidates[1][2:])<.35:
        # Two small returns on the same depth plane can be a person's legs.
        # Keep the merge narrower than a torso and inside the selected box.
        x=(candidates[0][2]+candidates[1][2])/2
        y=(candidates[0][3]+candidates[1][3])/2
        candidates=[(0.,math.hypot(x,y),x,y)]
    if len(candidates) > 1:
        # A nearer object may occlude another at the same bearing. Do not silently
        # choose either when more than one cluster fits the selected camera box.
        result['reason'] = 'follow_range_ambiguous'
        return result
    _, distance, x, y = candidates[0]
    result.update(matched=True, distance_m=distance,
                  bearing_right_deg=math.degrees(math.atan2(-y, x)),
                  x_body_m=x, y_body_m=y, reason='follow_range_confirmed')
    return result


def associate_centered(ranges, angle_min, increment, range_min, range_max,
                       laser_pose, body_pose, image_error, yaw_since_capture,
                       camera_right_m=0., previous=None, desired_distance_m=1.):
    """Uncalibrated mode: center first, then a unique +/-8 degree frontal return.

    Assumes the installed cameras face the physical front. This is a conservative
    association heuristic, not a calibrated pixel projection or lidar identity.
    Scan clearance remains available while centering, but no range is trusted.
    """
    if not finite(image_error) or abs(image_error)>1 or not finite(yaw_since_capture):
        raise ValueError('invalid_image_alignment')
    # Follow an existing near hypothesis with enough span for its widening
    # silhouette. Initial acquisition stays narrow; walls and multiple clusters
    # are still rejected. A previous hypothesis never grants motion by itself.
    span = 25. if desired_distance_m < .8 and previous is not None and math.hypot(*previous) < 1.5 else 8.
    value=associate(ranges,angle_min,increment,range_min,range_max,laser_pose,
                    body_pose,-span,span,camera_right_m,previous,desired_distance_m)
    if abs(image_error)>.06 or abs(yaw_since_capture)>2:
        value.update(matched=False,distance_m=0.,bearing_right_deg=0.,
                     x_body_m=0.,y_body_m=0.,reason='follow_wait_center')
    return value


class RangeConfirmation:
    """Three distinct scans spanning 120ms; duplicates never confirm a target."""
    def __init__(self):
        self.key=None
        self.first=self.last=0
        self.count=0

    def update(self,key,scan_ns,matched):
        if key!=self.key or not matched or scan_ns<self.last or scan_ns-self.last>300000000:
            self.key=key
            self.first=self.last=scan_ns
            self.count=1 if matched else 0
            return False
        if scan_ns>self.last:
            if not self.count:self.first=scan_ns
            self.count+=1
            self.last=scan_ns
        return self.count>=3 and scan_ns-self.first>=120000000
