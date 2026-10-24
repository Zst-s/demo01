"""Select the next 3D clearance probe along a desired passage direction.

The 2D map supplies *candidate locations only*. Nothing in this module is a
passability verdict, static-map correction or motion authorization.
"""
from math import atan2, cos, floor, hypot, isfinite, sin


def covered_by_segment(x, y, segment):
    dx, dy = x-segment['center_x'], y-segment['center_y']
    c, s = cos(segment['yaw']), sin(segment['yaw'])
    along, across = c*dx+s*dy, -s*dx+c*dy
    return (abs(along) <= (segment['travel_length']+segment['robot_length'])/2 and
            abs(across) <= segment['robot_width']/2)


def choose_next_static_conflict(grid, pose, goal, segments=(), max_range=1.5):
    """Find nearest not-yet-verified occupied cell along the pose→goal ray.

    Grid is a nav_msgs/OccupancyGrid-like object with axis-aligned map origin.
    Returns (cell center x, cell center y, yaw), or None. Unknown is never
    treated as known-free or as proof of a tunnel.
    """
    info = grid.info
    numbers = (pose[0], pose[1], goal[0], goal[1], max_range,
               info.resolution, info.origin.position.x, info.origin.position.y)
    if (grid.header.frame_id != 'map' or info.width <= 0 or info.height <= 0 or
            len(grid.data) != info.width*info.height or
            not all(isfinite(v) for v in numbers) or
            info.resolution <= 0 or max_range <= 0 or
            abs(info.origin.orientation.x)+abs(info.origin.orientation.y)+
            abs(info.origin.orientation.z) > 1e-6 or
            abs(info.origin.orientation.w-1.0) > 1e-6):
        raise ValueError('INVALID_GLOBAL_MAP_OR_GOAL')
    dx, dy = goal[0]-pose[0], goal[1]-pose[1]
    distance = hypot(dx, dy)
    if distance < 1e-6:
        return None
    ux, uy = dx/distance, dy/distance
    yaw = atan2(uy, ux)
    step = min(info.resolution/2, .05)
    seen = set()
    cursor = max(info.resolution, .15)
    while cursor <= min(distance, max_range):
        x, y = pose[0]+ux*cursor, pose[1]+uy*cursor
        ix = floor((x-info.origin.position.x)/info.resolution)
        iy = floor((y-info.origin.position.y)/info.resolution)
        if not 0 <= ix < info.width or not 0 <= iy < info.height:
            return None
        key = iy*info.width+ix
        if key not in seen:
            seen.add(key)
            if grid.data[key] >= 65:
                center_x = info.origin.position.x+(ix+.5)*info.resolution
                center_y = info.origin.position.y+(iy+.5)*info.resolution
                if not any(covered_by_segment(center_x, center_y, item)
                           for item in segments):
                    return center_x, center_y, yaw
        cursor += step
    return None
