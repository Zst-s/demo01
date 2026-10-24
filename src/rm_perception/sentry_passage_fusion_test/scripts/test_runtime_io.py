#!/usr/bin/env python3
"""Geometry/transport tests do not claim real-vehicle passage validation."""
import unittest
from geometry_msgs.msg import Pose, Twist
from nav_msgs.msg import OccupancyGrid
from passage_runtime import corridor_blocked, planar


class RuntimeIOTest(unittest.TestCase):
    def setUp(self):
        self.grid = OccupancyGrid()
        self.grid.header.frame_id = 'map'
        self.grid.info.width = self.grid.info.height = 40
        self.grid.info.resolution = .1
        self.grid.info.origin.position.x = self.grid.info.origin.position.y = -2.
        self.grid.info.origin.orientation.w = 1.
        self.grid.data = [0]*1600
        self.pose = Pose()
        self.pose.orientation.w = 1.
        self.command = Twist()
        self.command.linear.x = .2

    def test_free(self):
        self.assertFalse(corridor_blocked(self.grid, self.pose, self.command, .32))

    def test_swept_front_obstacle(self):
        self.grid.data[20*40+24] = 100
        self.assertTrue(corridor_blocked(self.grid, self.pose, self.command, .32))

    def test_unknown(self):
        self.grid.data[20*40+20] = -1
        self.assertTrue(corridor_blocked(self.grid, self.pose, self.command, .32))

    def test_outside(self):
        self.pose.position.x = 1.95
        self.assertTrue(corridor_blocked(self.grid, self.pose, self.command, .32))

    def test_wrong_frame(self):
        self.grid.header.frame_id = 'odom'
        self.assertTrue(corridor_blocked(self.grid, self.pose, self.command, .32))

    def test_invalid_command(self):
        self.command.linear.x = float('nan')
        self.assertFalse(planar(self.command))

    def test_actual_rectangle_passes_corridor_rejected_by_enclosing_circle(self):
        self.grid.info.resolution = .02
        self.grid.info.width = self.grid.info.height = 100
        self.grid.info.origin.position.x = self.grid.info.origin.position.y = -1.
        self.grid.data = [0]*10000
        # Walls start at +/- .36: wider than .64 padded body, narrower than its circle.
        for y in range(100):
            py = -1+(y+.5)*.02
            if abs(py) >= .36:
                for x in range(100):
                    self.grid.data[y*100+x] = 100
        self.assertTrue(corridor_blocked(self.grid, self.pose, self.command, .425))
        self.assertFalse(corridor_blocked(self.grid, self.pose, self.command, .425,
                                         length=.58, width=.62))
        self.command.linear.y = .2
        self.assertTrue(corridor_blocked(self.grid, self.pose, self.command, .425,
                                        length=.58, width=.62))

    def test_rectangle_turn_checks_corners_and_does_not_double_inflate(self):
        self.grid.info.resolution = .02
        self.grid.info.width = self.grid.info.height = 100
        self.grid.info.origin.position.x = self.grid.info.origin.position.y = -1.
        self.grid.data = [0]*10000
        self.command.linear.x = 0.
        cell = 50*100+68  # x=.37, y=.01: outside stationary body, inside its rotated sweep.
        self.grid.data[cell] = 100
        self.assertFalse(corridor_blocked(self.grid, self.pose, self.command, .425,
                                         length=.58, width=.62))
        self.command.angular.z = 1.57
        self.assertTrue(corridor_blocked(self.grid, self.pose, self.command, .425,
                                        length=.58, width=.62))
        self.command.angular.z = 0.
        self.grid.data[50*100+50] = 99
        self.assertFalse(corridor_blocked(self.grid, self.pose, self.command, .425,
                                         length=.58, width=.62))
        self.grid.data[50*100+50] = 100
        self.assertTrue(corridor_blocked(self.grid, self.pose, self.command, .425,
                                        length=.58, width=.62))


    # Verify the measured 1 cm rearward offset, including a rotated body.
    def test_offset_outline(self):
        self.grid.info.resolution = .005
        self.grid.info.width = self.grid.info.height = 200
        self.grid.info.origin.position.x = self.grid.info.origin.position.y = -.5
        self.grid.data = [0]*40000
        self.command.linear.x = 0.
        # x=-.318 is in the actual rear-side extension, outside the centered body.
        self.grid.data[100*200+36] = 100
        self.assertFalse(corridor_blocked(self.grid,self.pose,self.command,.425,
            length=.62,width=.58,padding=0.))
        self.assertTrue(corridor_blocked(self.grid,self.pose,self.command,.425,
            length=.62,width=.58,padding=0.,center_x=-.01))
        self.grid.data = [0]*40000
        self.pose.orientation.z = self.pose.orientation.w = 2**-.5
        self.grid.data[36*200+100] = 100  # rotated rear side is -Y
        self.assertTrue(corridor_blocked(self.grid,self.pose,self.command,.425,
            length=.62,width=.58,padding=0.,center_x=-.01))

if __name__ == '__main__':
    unittest.main()
