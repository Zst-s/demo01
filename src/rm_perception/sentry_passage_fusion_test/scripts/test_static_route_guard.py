#!/usr/bin/env python3
"""Do not let a short-segment permit silently authorize the next wall."""
from static_route_guard import route_limit


def check(static, verified, path, pose=(-1., .05), speed=.2):
    return route_limit(path, pose, speed, width=80, height=80,
                       resolution=.1, origin_x=-4., origin_y=-4.,
                       static_cells=static, verified_cells=verified)


def run():
    static = [0]*6400
    verified = [0]*6400
    for x in (39, 40, 50, 51):
        static[40*80+x] = 100
    for x in (39, 40):
        verified[40*80+x] = 100
    path = [(-1., .05), (2., .05)]
    first_only = check(static, verified, path)
    assert not first_only.stop
    assert first_only.reason == 'APPROACH_UNVERIFIED_STATIC_CELL'
    assert first_only.speed_limit > 0
    near_second = check(static, verified, [(.75, .05), (2., .05)],
                        pose=(.75, .05))
    assert near_second.stop and near_second.reason == 'STOP_BEFORE_UNVERIFIED_STATIC_CELL'
    for x in (50, 51):
        verified[40*80+x] = 100
    assert check(static, verified, path).reason == 'NO_UNVERIFIED_STATIC_CELL'
    verified[40*80+50] = 0
    assert check(static, verified, [(.75, .05), (2., .05)],
                 pose=(.75, .05)).stop
    static[40*80+50] = -1
    verified[40*80+50] = 100
    assert check(static, verified, [(.75, .05), (2., .05)],
                 pose=(.75, .05)).stop  # Unknown may never be cleared.
    assert check(static, verified, [(-1., .05), (5., .05)]).reason in (
        'APPROACH_UNVERIFIED_STATIC_CELL', 'STOP_BEFORE_UNVERIFIED_STATIC_CELL')
    assert check(static, verified, [(9., .05), (10., .05)], pose=(9., .05)).stop
    print('PASS: first short proof is not authority to enter a second wall or unknown cell')


if __name__ == '__main__':
    run()
