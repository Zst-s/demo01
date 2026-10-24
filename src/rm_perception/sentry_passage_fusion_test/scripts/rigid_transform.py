"""Shared rigid-transform validation and map-association comparison (ROS xyzw)."""
from math import acos, cos, isfinite, sin, sqrt


def normalize_transform(values):
    """Return tx, ty, tz, qx, qy, qz, qw; accept legacy tx, ty, tz, yaw records."""
    if (not isinstance(values, (list, tuple)) or len(values) not in (4, 7) or
            any(type(v) not in (int, float) or not isfinite(v) for v in values)):
        raise ValueError('INVALID_RIGID_TRANSFORM')
    if len(values) == 4:
        x, y, z, yaw = values
        return x, y, z, 0., 0., sin(yaw/2), cos(yaw/2)
    norm2 = sum(v*v for v in values[3:])
    if not isfinite(norm2) or abs(norm2-1.) > .01:
        raise ValueError('INVALID_RIGID_QUATERNION')
    norm = sqrt(norm2)
    return tuple(values[:3]) + tuple(v/norm for v in values[3:])


def rotation_matrix(transform):
    _, _, _, x, y, z, w = normalize_transform(transform)
    return ((1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)),
            (2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)),
            (2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)))


def context_transform(context):
    q = context.map_to_odom_quaternion
    if not isinstance(q, (tuple, list)) or len(q) not in (0, 4):
        raise ValueError('INVALID_CONTEXT_QUATERNION')
    if not q:
        return normalize_transform((context.map_to_odom_x, context.map_to_odom_y,
                                    context.map_to_odom_z, context.map_to_odom_yaw))
    return normalize_transform((context.map_to_odom_x, context.map_to_odom_y,
                                context.map_to_odom_z, *q))


def transforms_close(first, second, max_shift, max_angle):
    first, second = normalize_transform(first), normalize_transform(second)
    shift = sqrt(sum((a-b)**2 for a, b in zip(first[:3], second[:3])))
    # q and -q describe the same rotation.
    dot = abs(sum(a*b for a, b in zip(first[3:], second[3:])))
    angle = 2*acos(min(1., dot))
    return shift <= max_shift and angle <= max_angle
