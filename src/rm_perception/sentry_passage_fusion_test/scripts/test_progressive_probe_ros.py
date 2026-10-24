#!/usr/bin/env python3
"""Probe coordinator advances only after the previously verified body exits."""
import json
import os
from pathlib import Path
import subprocess
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from tf2_ros import StaticTransformBroadcaster


def run():
    if os.environ.get('ROS_DOMAIN_ID') != '222' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
        raise RuntimeError('Probe coordinator test requires isolated domain 222')
    rclpy.init()
    node = Node('progressive_probe_wire_test')
    map_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
    grid_pub = node.create_publisher(OccupancyGrid, '/passage_fusion_test/map', map_qos)
    odom_pub = node.create_publisher(Odometry, '/passage_fusion_test/odom_map', 10)
    goal_pub = node.create_publisher(PoseStamped, '/passage_fusion_test/desired_goal', 10)
    memory_pub = node.create_publisher(String, '/passage/memory_shadow_status', 10)
    probes = []
    probe_sub = node.create_subscription(
        PoseStamped, '/terrain/clearance/probe', lambda msg: probes.append(msg), 10)
    broadcaster = StaticTransformBroadcaster(node)
    tf = TransformStamped()
    tf.header.frame_id = 'odom'
    tf.child_frame_id = 'map'
    tf.transform.rotation.w = 1.
    grid = OccupancyGrid()
    grid.header.frame_id = 'map'
    grid.info.width = grid.info.height = 80
    grid.info.resolution = .1
    grid.info.origin.position.x = grid.info.origin.position.y = -4.
    grid.info.origin.orientation.w = 1.
    grid.data = [0]*6400
    for x in range(40, 48):
        grid.data[40*80+x] = 100
    robot_x = [-1.]
    segments = []
    process = subprocess.Popen([
        sys.executable, str(Path(__file__).with_name('progressive_probe_coordinator.py')),
        '--ros-args', '-r', '__ns:=/passage_fusion_test',
        '-p', 'evidence_frame:=odom'],
        stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT)

    def pulse():
        stamp = node.get_clock().now().to_msg()
        grid.header.stamp = stamp
        grid_pub.publish(grid)
        odom = Odometry()
        odom.header.frame_id = 'map'
        odom.header.stamp = stamp
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = robot_x[0]
        odom.pose.pose.position.y = .05
        odom.pose.pose.orientation.w = 1.
        odom_pub.publish(odom)
        goal = PoseStamped()
        goal.header = grid.header
        goal.pose.position.x = 2.
        goal.pose.position.y = .05
        goal.pose.orientation.w = 1.
        goal_pub.publish(goal)
        memory_pub.publish(String(data=json.dumps({
            'state': 'KNOWN_PASSAGE' if segments else 'UNKNOWN',
            'map_associated': True, 'segments': segments})))
        broadcaster.sendTransform(tf)
        rclpy.spin_once(node, timeout_sec=.05)

    def wait(predicate, label):
        deadline = time.monotonic()+5
        while time.monotonic() < deadline:
            pulse()
            if predicate():
                return
        raise AssertionError(f'{label}; observed={[(p.pose.position.x, p.pose.position.y) for p in probes]}')

    try:
        wait(lambda: len(probes) >= 1, 'first automatic candidate')
        first = probes[-1]
        assert first.header.frame_id == 'odom' and abs(first.pose.position.x-.05) < .06
        segments.append(dict(segment_id='first', center_x=.05, center_y=.05,
                             yaw=0., travel_length=.2, robot_length=.45,
                             robot_width=.45, verified_width=.45))
        for _ in range(6):
            pulse()
        assert len(probes) == 1, 'Existing segment probe was reset before vehicle exited'
        robot_x[0] = .5
        wait(lambda: len(probes) >= 2, 'next short segment candidate')
        assert probes[-1].pose.position.x > .35
        print('PASS: candidate → hold during fresh validation → next probe after rear exits', flush=True)
    finally:
        process.terminate()
        try:
            process.wait(timeout=3)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=3)
        del probe_sub, broadcaster
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    run()
