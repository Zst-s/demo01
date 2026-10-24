#!/usr/bin/env python3
"""Isolated Gazebo test vehicle: gate output is its sole drive input.

The watchdog publishes zero on missing/stale authorization. This is a lab
transport, not a real chassis safety controller or a localization algorithm.
"""
import math
import os
import time

import rclpy
from geometry_msgs.msg import TransformStamped, Twist, TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from tf2_ros import TransformBroadcaster


class MobileLabBridge(Node):
    def __init__(self):
        super().__init__('mobile_lab_bridge')
        if (self.get_namespace() != '/passage_fusion_test' or
                os.environ.get('ROS_DOMAIN_ID') not in
                {str(value) for value in range(219, 240)} or
                os.environ.get('ROS_LOCALHOST_ONLY') != '1'):
            raise ValueError('Mobile bridge requires isolated passage lab domain 219..239')
        if not self.get_parameter('use_sim_time').value:
            raise ValueError('Mobile lab bridge requires simulated clock')
        self.last_auth = None
        self.received_monotonic = 0.
        self.odom = self.create_publisher(
            Odometry, '/passage_fusion_test/odom_map', 10)
        self.drive = self.create_publisher(Twist, '/passage_sim/drive_cmd_vel', 10)
        self.tf = TransformBroadcaster(self)
        self.create_subscription(Odometry, '/passage_sim/sensor_odom',
                                 self.on_odom, qos_profile_sensor_data)
        self.create_subscription(TwistStamped, '/passage_fusion_test/cmd_vel_authorized',
                                 self.on_auth, 10)
        self.create_timer(.05, self.tick)

    def on_odom(self, message):
        if (message.header.frame_id != 'world' or
                not all(math.isfinite(v) for v in (
                    message.pose.pose.position.x, message.pose.pose.position.y,
                    message.pose.pose.position.z)) or
                abs(message.pose.pose.position.z-.55) > .05):
            return
        pose = message.pose.pose
        republished = Odometry()
        republished.header = message.header
        republished.header.frame_id = 'map'  # The lab publishes identity map -> world.
        republished.child_frame_id = 'base_link'
        republished.pose.pose = pose
        republished.pose.pose.position.z = 0.
        republished.twist = message.twist
        self.odom.publish(republished)
        transform = TransformStamped()
        transform.header = republished.header
        transform.child_frame_id = 'base_link'
        transform.transform.translation.x = pose.position.x
        transform.transform.translation.y = pose.position.y
        transform.transform.rotation = pose.orientation
        self.tf.sendTransform(transform)

    def on_auth(self, message):
        self.last_auth = message
        self.received_monotonic = time.monotonic()

    def tick(self):
        output = Twist()
        message = self.last_auth
        now = self.get_clock().now().nanoseconds/1e9
        if message is not None:
            stamp = message.header.stamp.sec+message.header.stamp.nanosec*1e-9
            values = (message.twist.linear.x, message.twist.linear.y,
                      message.twist.angular.z)
            if (message.header.frame_id == 'base_link' and
                    0 <= now-stamp <= .2 and
                    time.monotonic()-self.received_monotonic <= .2 and
                    all(math.isfinite(v) for v in values) and
                    abs(values[0]) <= .3 and abs(values[1]) <= .3 and
                    abs(values[2]) <= .5):
                output.linear.x = values[0]
                output.linear.y = values[1]
                output.angular.z = values[2]
        self.drive.publish(output)


def main():
    rclpy.init()
    node = MobileLabBridge()
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
