#!/usr/bin/env python3
"""Exercise runtime memory -> gate -> chassis-topic; NEVER open a serial port."""
import os
import subprocess
import sys
import tempfile
import time
from pathlib import Path

import rclpy
from action_msgs.msg import GoalStatus, GoalStatusArray
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry, Path as NavPath
from rclpy.qos import QoSProfile, DurabilityPolicy
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster


def main():
    if os.environ.get('ROS_DOMAIN_ID') != '230' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
        raise RuntimeError('Require isolated domain 230, no serial process permitted')
    rclpy.init()
    node = rclpy.create_node('runtime_chain_test')
    pub = lambda typ, topic, qos=1: node.create_publisher(typ, topic, qos)
    map_pub = pub(OccupancyGrid, '/map', QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    grid_pub = pub(OccupancyGrid, '/local_costmap/costmap')
    odom_pub = pub(Odometry, '/Odometry')
    command_pub = pub(Twist, '/passage/nav_cmd_vel')
    plan_pub = pub(NavPath, '/plan')
    active_pub = pub(GoalStatusArray, '/navigate_to_pose/_action/status')
    received = []
    sub = node.create_subscription(Twist, '/passage/chassis_cmd_vel',
                                   lambda m: received.append((time.monotonic(), m.linear.x)), 10)
    static = StaticTransformBroadcaster(node)
    dynamic = TransformBroadcaster(node)
    grid = OccupancyGrid()
    grid.header.frame_id = 'map'
    grid.info.width = grid.info.height = 80
    grid.info.resolution = .1
    grid.info.origin.position.x = grid.info.origin.position.y = -4.
    grid.info.origin.orientation.w = 1.
    grid.data = [0]*6400
    base = TransformStamped()
    base.header.frame_id = 'map'
    base.child_frame_id = 'odom'
    base.transform.rotation.w = 1.
    static.sendTransform(base)
    moving = TransformStamped()
    moving.header.frame_id = 'odom'
    moving.child_frame_id = 'base_link'
    moving.transform.rotation.w = 1.
    odom = Odometry()
    odom.header.frame_id = 'odom'
    odom.child_frame_id = 'base_link'
    odom.pose.pose.orientation.w = 1.
    route = NavPath()
    route.header.frame_id = 'map'
    for x in (0., .5, 1.):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.pose.position.x = x
        pose.pose.orientation.w = 1.
        route.poses.append(pose)
    command = Twist()
    command.linear.x = .1
    active = GoalStatusArray()
    status = GoalStatus()
    status.status = GoalStatus.STATUS_EXECUTING
    active.status_list = [status]
    procs = []
    script = str(Path(__file__).with_name('passage_runtime.py'))
    with tempfile.TemporaryDirectory(prefix='passage-runtime-chain-') as directory:
        try:
            for role in ('memory', 'gate', 'io'):
                args = [sys.executable, script, role, '--ros-args', '-r', '__ns:=/passage']
                if role == 'memory':
                    args += ['-p', 'map_session_id:=runtime-test', '-p', f'record_path:={directory}/memory.json']
                if role == 'io':
                    args += ['-p', 'enable_motion:=true']
                procs.append(subprocess.Popen(args))

            def pump(duration, grid_enabled=True, command_enabled=True, action=True):
                start = time.monotonic()
                while time.monotonic()-start < duration:
                    stamp = node.get_clock().now().to_msg()
                    grid.header.stamp = stamp
                    moving.header.stamp = stamp
                    odom.header.stamp = stamp
                    route.header.stamp = stamp
                    dynamic.sendTransform(moving)
                    odom_pub.publish(odom)
                    plan_pub.publish(route)
                    if grid_enabled:
                        grid_pub.publish(grid)
                    if command_enabled:
                        command_pub.publish(command)
                    active_pub.publish(active if action else GoalStatusArray())
                    rclpy.spin_once(node, timeout_sec=.03)
                    time.sleep(.02)
                assert all(p.poll() is None for p in procs), 'runtime process exited'

            map_pub.publish(grid)
            pump(4.)
            assert any(v > .05 for _, v in received[-20:]), 'runtime never authorized free route'
            pump(1., grid_enabled=False)
            assert received and all(v == 0 for _, v in received[-5:]), 'stale costmap did not stop'
            pump(1.)
            assert any(v > .05 for _, v in received[-15:]), 'fresh inputs did not recover'
            pump(1., action=False)
            assert all(v == 0 for _, v in received[-5:]), 'cancelled goal did not stop'
            pump(1.)
            pump(.8, command_enabled=False)
            assert all(v == 0 for _, v in received[-5:]), 'lost Nav2 command did not stop'
            print('PASS runtime memory -> gate -> output: free route, stale grid, cancellation, lost command')
        finally:
            for proc in procs:
                proc.terminate()
            for proc in procs:
                try:
                    proc.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    proc.kill()
                    proc.wait()
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
