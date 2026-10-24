#!/usr/bin/env python3
"""Humble Twist -> stamped test gate input. Never publishes to a chassis.

Receipt stamping is only for this local isolated transport; it cannot detect
upstream queued unstamped commands and is NOT a real-robot safety guarantee.
There is no republish timer: absent upstream commands cannot renew freshness.
"""
from math import isfinite

import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from rclpy.node import Node


class ControllerCommandAdapter(Node):
    def __init__(self):
        super().__init__('controller_command_adapter')
        if self.get_namespace() != '/passage_fusion_test':
            raise ValueError('Adapter restricted to /passage_fusion_test')
        self.output = self.create_publisher(
            TwistStamped, '/passage_fusion_test/cmd_vel_desired', 1)
        self.create_subscription(Twist, '/passage_fusion_test/controller_cmd_vel_raw',
                                 self.on_command, 1)

    def on_command(self, command):
        values = (command.linear.x, command.linear.y, command.linear.z,
                  command.angular.x, command.angular.y, command.angular.z)
        msg = TwistStamped()
        msg.header.frame_id = 'base_link'
        msg.header.stamp = self.get_clock().now().to_msg()
        if (all(isfinite(value) for value in values) and
                command.linear.z == 0 and
                command.angular.x == 0 and command.angular.y == 0):
            msg.twist = command
            # This test-only transport models a bounded chassis command mux:
            # saturate excessive controller outputs instead of dropping the
            # entire Twist (which otherwise deadlocks any heading correction).
            msg.twist.linear.x = max(-.3, min(.3, command.linear.x))
            msg.twist.linear.y = max(-.3, min(.3, command.linear.y))
            msg.twist.angular.z = max(-.5, min(.5, command.angular.z))
        self.output.publish(msg)


def main():
    rclpy.init()
    node = ControllerCommandAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
