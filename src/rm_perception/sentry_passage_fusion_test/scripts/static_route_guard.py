"""Fail-closed guard for *unverified* static cells on an isolated planned route.

The correction mask is a historical planning hint, not entry permission.  This
guard prevents a command from carrying the robot into the next wall/unknown
cell when only the preceding short passage has been verified.
"""
from dataclasses import dataclass
from math import floor, hypot, isfinite, sqrt


@dataclass(frozen=True)
class RouteLimit:
    stop: bool
    speed_limit: float
    reason: str
    distance: float


def route_limit(path, pose, speed, *, width, height, resolution, origin_x,
                origin_y, static_cells, verified_cells, deceleration=.8,
                latency=.2, margin=.1, robot_length=.45):
    """Limit speed before the first occupied/unknown *unverified* centre cell.

    This is an additional guard, not a footprint collision checker.  The
    independent segment gate must still validate the full swept body volume.
    """
    if (len(path) < 2 or width <= 0 or height <= 0 or resolution <= 0 or
            len(static_cells) != width*height or len(verified_cells) != width*height or
            not all(isfinite(v) for v in (resolution, origin_x, origin_y, speed,
                                           deceleration, latency, margin, robot_length)) or
            min(deceleration, latency, margin, robot_length) <= 0 or speed < 0 or
            not all(isfinite(v) for point in path for v in point) or
            hypot(path[0][0]-pose[0], path[0][1]-pose[1]) > .5):
        return RouteLimit(True, 0., 'INVALID_STATIC_ROUTE_CONTEXT', 0.)
    travelled = 0.
    for a, b in zip(path, path[1:]):
        length = hypot(b[0]-a[0], b[1]-a[1])
        steps = max(1, int(length/(resolution/2))+1)
        if steps > 20000:
            return RouteLimit(True, 0., 'ROUTE_TOO_LONG', travelled)
        for step in range(steps+1):
            ratio = step/steps
            x = a[0]+ratio*(b[0]-a[0])
            y = a[1]+ratio*(b[1]-a[1])
            ix = floor((x-origin_x)/resolution)
            iy = floor((y-origin_y)/resolution)
            if not (0 <= ix < width and 0 <= iy < height):
                blocked = True
            else:
                index = iy*width+ix
                value = static_cells[index]
                blocked = (value < 0 or value >= 65) and not (
                    value >= 65 and verified_cells[index] == 100)
            if blocked:
                distance = travelled+ratio*length
                reserve = distance-robot_length/2-margin
                stopping = speed*speed/(2*deceleration)+speed*latency
                if reserve <= stopping:
                    return RouteLimit(True, 0., 'STOP_BEFORE_UNVERIFIED_STATIC_CELL', distance)
                limit = max(0., sqrt((deceleration*latency)**2+2*deceleration*reserve)
                            -deceleration*latency)
                return RouteLimit(False, limit, 'APPROACH_UNVERIFIED_STATIC_CELL', distance)
        travelled += length
    return RouteLimit(False, float('inf'), 'NO_UNVERIFIED_STATIC_CELL', float('inf'))
