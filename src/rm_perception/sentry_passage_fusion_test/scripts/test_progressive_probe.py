#!/usr/bin/env python3
"""2D wall conflicts propose one short 3D probe at a time; never grant entry."""
from nav_msgs.msg import OccupancyGrid

from progressive_probe import choose_next_static_conflict


def run():
    grid = OccupancyGrid()
    grid.header.frame_id = 'map'
    grid.info.width = grid.info.height = 80
    grid.info.resolution = .1
    grid.info.origin.position.x = grid.info.origin.position.y = -4.
    grid.info.origin.orientation.w = 1.
    grid.data = [0]*6400
    for x in range(40, 48):
        grid.data[40*80+x] = 100
    first = choose_next_static_conflict(grid, (-1., .05), (2., .05))
    assert first is not None and abs(first[0]-.05) < 1e-6
    verified = [dict(center_x=.05, center_y=.05, yaw=0.,
                     travel_length=.2, robot_length=.45, robot_width=.45)]
    second = choose_next_static_conflict(grid, (-1., .05), (2., .05), verified)
    assert second is not None and second[0] > .35
    grid.data[40*80+44] = -1
    third = choose_next_static_conflict(grid, (-1., .05), (2., .05),
                                        verified, max_range=2.)
    assert third is not None and third[0] > second[0]
    grid.data = [-1]*6400
    assert choose_next_static_conflict(grid, (-1., .05), (2., .05)) is None
    print('Progressive probe: first wall, next unverified cell, unknown never becomes proof')


if __name__ == '__main__':
    run()
