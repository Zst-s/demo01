#!/usr/bin/env python3
"""Preview evidence opens only an isolated Costmap, never an entry permit."""
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import time

import rclpy
from geometry_msgs.msg import TransformStamped
from nav2_msgs.msg import Costmap
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from tf2_ros import StaticTransformBroadcaster

WIDTH = 100  # Deliberately differs from the harness's initial 80-cell map.
HEIGHT = 80
CELL = 40 * WIDTH + 40


def main():
    if os.environ.get('ROS_DOMAIN_ID') != '228' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
        raise RuntimeError('Isolated ROS domain 228 required')
    rclpy.init()
    node = rclpy.create_node('virtual_preview_wiring_check')
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
    latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
    map_pub = node.create_publisher(OccupancyGrid, '/map', latched)
    terrain_pub = node.create_publisher(OccupancyGrid, '/terrain/grid', qos)
    preview_pub = node.create_publisher(String, '/terrain/clearance/preview_segment', qos)
    cost = [None]
    status = [None]
    permits = []
    subs = [node.create_subscription(Costmap, '/passage_fusion_test/costmap_raw',
                                     lambda m: cost.__setitem__(0, m), qos),
            node.create_subscription(String, '/passage/memory_shadow_status',
                                     lambda m: status.__setitem__(0, json.loads(m.data)), qos),
            node.create_subscription(String, '/passage_fusion_test/fresh_permit',
                                     lambda m: permits.append(m), qos)]
    broadcaster = StaticTransformBroadcaster(node)
    tf = TransformStamped()
    tf.header.frame_id = 'map'
    tf.child_frame_id = 'odom'
    tf.transform.rotation.w = 1.0
    grid = OccupancyGrid()
    grid.header.frame_id = 'map'
    grid.info.width = WIDTH
    grid.info.height = HEIGHT
    grid.info.resolution = .1
    grid.info.origin.position.x = grid.info.origin.position.y = -4.0
    grid.info.origin.orientation.w = 1.0
    grid.data = [0] * (WIDTH * HEIGHT)
    for y in range(HEIGHT):
        grid.data[y*WIDTH+40] = 100
    terrain = OccupancyGrid()
    terrain.header.frame_id = 'map'
    terrain.info = grid.info
    terrain.data = [0] * (WIDTH * HEIGHT)
    procs = []
    dynamic = [False]
    send_proof = [False]

    def proof():
        now = node.get_clock().now().nanoseconds / 1e9
        return String(data=json.dumps({
            'schema': 1, 'kind': 'virtual_short_segment_preview',
            'frame': 'odom', 'probe_x': 0.0, 'probe_y': 0.0,
            'ground_z': 0.0, 'yaw': 0.0, 'travel_length': .2,
            'robot_length': .45, 'robot_width': .45, 'robot_height': .32,
            'resolution': .1, 'confirmed_frames': 3,
            'measured_at': now, 'oldest_observation': now-.1,
            'newest_observation': now-.01, 'valid_until': now+.8,
            'checked_voxels': 100}))

    def tick():
        if any(p.poll() is not None for p in procs):
            raise AssertionError('Preview host or memory exited')
        stamp = node.get_clock().now().to_msg()
        grid.header.stamp = stamp
        map_pub.publish(grid)
        terrain.header.stamp = stamp
        terrain.data[CELL] = 100 if dynamic[0] else 0
        terrain_pub.publish(terrain)
        broadcaster.sendTransform(tf)
        if send_proof[0]:
            preview_pub.publish(proof())
        rclpy.spin_once(node, timeout_sec=.05)

    def wait_for(predicate, name, timeout=6.0):
        end = time.monotonic()+timeout
        while time.monotonic() < end:
            tick()
            if predicate():
                print('PASS:', name, flush=True)
                return
        value = None if cost[0] is None else cost[0].data[CELL]
        raise AssertionError(f'{name}: cost={value}, status={status[0]}, permits={permits}')

    def cell():
        return (None if cost[0] is None or cost[0].metadata.size_x != WIDTH or
                cost[0].metadata.size_y != HEIGHT else cost[0].data[CELL])

    try:
        with tempfile.TemporaryDirectory(prefix='virtual-wiring-') as directory:
            root = Path(__file__).resolve().parents[1]
            with open(Path(directory)/'harness.log', 'w') as out:
                procs.append(subprocess.Popen([
                    sys.argv[1], '--ros-args', '--params-file', sys.argv[2]],
                    stdout=out, stderr=subprocess.STDOUT))
            with open(Path(directory)/'memory.log', 'w') as out:
                procs.append(subprocess.Popen([
                    sys.executable, str(root/'scripts/live_memory_shadow.py'),
                    '--ros-args', '-r', '__ns:=/passage_fusion_test',
                    '-p', 'map_session_id:=virtual-wiring',
                    '-p', f'record_path:={directory}/must_not_be_written.json',
                    '-p', 'evidence_frame:=odom',
                    '-p', 'publish_test_mask:=true',
                    '-p', 'accept_virtual_preview:=true'],
                    stdout=out, stderr=subprocess.STDOUT))
            try:
                wait_for(lambda: cell() == 254, 'original wall remains')
                send_proof[0] = True
                # Inflation may assign a nonzero traversable cost near the
                # ends of the short verified segment; 254 is the wall.
                wait_for(lambda: cell() is not None and cell() < 254 and status[0] and
                         status[0]['state'] == 'KNOWN_PASSAGE' and
                         status[0]['virtual_preview'] and
                         status[0]['costmap_override'],
                         'virtual ROGMap proof opens isolated planning map')
                assert not permits, 'Preview issued an entry permit'
                assert not Path(directory, 'must_not_be_written.json').exists(), (
                    'Virtual preview persisted a real passage record')
                send_proof[0] = False
                dynamic[0] = True
                wait_for(lambda: cell() == 254,
                         'live obstacle overrides preview opening')
                assert not permits, 'Preview issued an entry permit'
            except BaseException:
                for name in ('harness.log', 'memory.log'):
                    print((Path(directory)/name).read_text()[-5000:], file=sys.stderr)
                raise
            finally:
                for proc in procs:
                    proc.terminate()
                for proc in procs:
                    try:
                        proc.wait(timeout=3)
                    except subprocess.TimeoutExpired:
                        proc.kill()
                        proc.wait(timeout=3)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
