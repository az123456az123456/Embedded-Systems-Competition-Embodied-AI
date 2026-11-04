"""Physical vehicle heading in the existing TF convention (TF stays unchanged)."""
import math


def physical_pose(pose, offset):
    if not all(math.isfinite(v) for v in (*pose, offset)):
        raise ValueError('invalid_body_heading')
    return pose[0], pose[1], math.atan2(math.sin(pose[2]+offset), math.cos(pose[2]+offset))


def sensor_in_body(yaw, x, y, offset):
    """Re-express base_link->sensor in physical body axes, including translation."""
    if not all(math.isfinite(v) for v in (yaw, x, y, offset)):
        raise ValueError('invalid_sensor_body_transform')
    c, s = math.cos(offset), math.sin(offset)
    return math.atan2(math.sin(yaw-offset), math.cos(yaw-offset)), c*x+s*y, -s*x+c*y
