#!/usr/bin/env python3
"""ROS wire test using the message shape captured from live ROGMap, no Gazebo."""
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster


def main():
    if os.environ.get('ROS_DOMAIN_ID') != '220' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
        raise RuntimeError('Live-wire test requires isolated local ROS domain 220')
    rclpy.init()
    node = Node('passage_memory_wire_test')
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
    map_pub = node.create_publisher(OccupancyGrid, '/map', qos)
    proof_pub = node.create_publisher(String, '/terrain/clearance/verified_segment', 10)
    broadcaster = StaticTransformBroadcaster(node)
    received = {'status': None}
    status_sub = node.create_subscription(
        String, '/passage/memory_shadow_status',
        lambda msg: received.update(status=json.loads(msg.data)), 10)
    grid = OccupancyGrid()
    grid.header.frame_id = 'map'
    grid.info.width = grid.info.height = 80
    grid.info.resolution = .1
    grid.info.origin.position.x = grid.info.origin.position.y = -4.
    grid.info.origin.orientation.w = 1.
    grid.data = [0] * 6400
    tf = TransformStamped()
    tf.header.frame_id = 'map'
    tf.child_frame_id = 'odom'
    tf.transform.rotation.w = 1.

    def wait_for(predicate, seconds=7):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.05)
            if predicate():
                return
        raise AssertionError(f'timed out; last status={received["status"]}')

    def start(path):
        return subprocess.Popen([
            sys.executable, str(Path(__file__).with_name('live_memory_shadow.py')),
            '--ros-args', '-p', 'map_session_id:=wire-test-map',
            '-p', f'record_path:={path}', '-p', 'evidence_frame:=odom',
        ], stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)

    def stop(process):
        process.terminate()
        try:
            process.wait(timeout=3)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=3)

    process = None
    try:
        with tempfile.TemporaryDirectory(prefix='passage-wire-') as directory:
            record = Path(directory) / 'memory.json'
            process = start(record)
            map_pub.publish(grid)
            broadcaster.sendTransform(tf)
            wait_for(lambda: received['status'] is not None and
                     proof_pub.get_subscription_count() > 0)
            wait_for(lambda: received['status']['reason'] != 'WAIT_MAP_OR_CLOCK')
            # Numeric fields match the actual ROGMap output observed in the
            # short-segment run; timestamps are rebased to this ROS test clock.
            deadline = time.monotonic() + 5
            while time.monotonic() < deadline and not (
                    received['status'] and received['status']['state'] == 'KNOWN_PASSAGE'):
                stamp = node.get_clock().now().nanoseconds / 1e9
                payload = dict(schema=1, kind='verified_short_segment_observation',
                               frame='odom', probe_x=1.7, probe_y=0, ground_z=-.025,
                               yaw=0, travel_length=.2, robot_length=.45,
                               robot_width=.45, robot_height=.7, resolution=.1,
                               revision=1, confirmed_frames=3, measured_at=stamp,
                               oldest_observation=stamp-.1,
                               newest_observation=stamp,
                               valid_until=stamp+.8, checked_voxels=432)
                proof_pub.publish(String(data=json.dumps(payload)))
                rclpy.spin_once(node, timeout_sec=.1)
            assert received['status']['state'] == 'KNOWN_PASSAGE', received['status']
            assert received['status']['entry_allowed'] is False
            assert received['status']['costmap_override'] is False
            assert received['status']['segment_count'] == 1
            # A second verified short segment must be remembered separately,
            # rather than replacing the first one in the rolling ROGMap window.
            deadline = time.monotonic() + 5
            while time.monotonic() < deadline and received['status'].get('segment_count') != 2:
                stamp = node.get_clock().now().nanoseconds / 1e9
                payload['probe_x'] = 2.7
                payload['measured_at'] = payload['newest_observation'] = stamp
                payload['oldest_observation'] = stamp-.1
                payload['valid_until'] = stamp+.8
                proof_pub.publish(String(data=json.dumps(payload)))
                rclpy.spin_once(node, timeout_sec=.1)
            assert received['status']['segment_count'] == 2, received['status']
            assert record.exists(), 'live proof was not persisted'
            stop(process)
            process = None
            received['status'] = None
            process = start(record)
            wait_for(lambda: received['status'] is not None and
                     received['status']['state'] == 'KNOWN_PASSAGE')
            assert received['status']['segment_count'] == 2
            assert received['status']['entry_allowed'] is False
            grid.data[40*80+55] = 100
            map_pub.publish(grid)
            wait_for(lambda: received['status']['state'] == 'KNOWN_PASSAGE' and
                     received['status']['segment_count'] == 1)
            grid.data[40*80+67] = 100
            map_pub.publish(grid)
            wait_for(lambda: received['status']['state'] == 'SUSPENDED' and
                     received['status']['reason'] == 'MAP_PATCH_CHANGED_ALL_SEGMENTS_REMOVED')
            print('PASS: two ROS proof segments -> disk restart -> individual map-patch veto', flush=True)
    finally:
        if process is not None:
            stop(process)
        del status_sub, broadcaster
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
