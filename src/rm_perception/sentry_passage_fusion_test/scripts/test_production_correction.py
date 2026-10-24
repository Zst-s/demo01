#!/usr/bin/env python3
"""Exercise the production Nav2 correction layer in a controller-free host."""
import copy
import os
import subprocess
import sys
import tempfile
import time

import rclpy
from nav2_msgs.msg import Costmap
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


WIDTH = 80
CELL = 40 * WIDTH + 40


def main():
    if os.environ.get('ROS_DOMAIN_ID') != '227' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
        raise RuntimeError('This test requires isolated ROS domain 227')
    rclpy.init()
    node = rclpy.create_node('production_correction_check')
    reliable = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
    latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
    map_pub = node.create_publisher(OccupancyGrid, '/map', latched)
    mask_pub = node.create_publisher(OccupancyGrid,
                                     '/passage_fusion_test/preview_static_clearing', reliable)
    terrain_pub = node.create_publisher(OccupancyGrid, '/terrain/grid', reliable)
    latest = [None]
    sub = node.create_subscription(Costmap, '/passage_fusion_test/costmap_raw',
                                   lambda msg: latest.__setitem__(0, msg), reliable)

    grid = OccupancyGrid()
    grid.header.frame_id = 'map'
    grid.info.width = grid.info.height = WIDTH
    grid.info.resolution = 0.1
    grid.info.origin.position.x = grid.info.origin.position.y = -4.0
    grid.info.origin.orientation.w = 1.0
    # A zero map_load_time is normal for some online map publishers.
    grid.data = [0] * (WIDTH * WIDTH)
    for y in range(WIDTH):
        grid.data[y * WIDTH + 40] = 100
    mask = copy.deepcopy(grid)
    mask.data = [0] * len(grid.data)
    for y in range(30, 50):
        mask.data[y * WIDTH + 40] = 100
    terrain = copy.deepcopy(grid)
    terrain.data = [0] * len(grid.data)
    dynamic = [False]
    send_mask = [False]
    proc = None

    def tick():
        if proc.poll() is not None:
            raise AssertionError(f'Fusion host exited: {proc.returncode}')
        stamp = node.get_clock().now().to_msg()
        grid.header.stamp = stamp
        map_pub.publish(grid)
        terrain.header.stamp = stamp
        terrain.data[CELL] = 100 if dynamic[0] else 0
        terrain_pub.publish(terrain)
        if send_mask[0]:
            mask.header.stamp = stamp
            mask_pub.publish(mask)
        rclpy.spin_once(node, timeout_sec=0.04)

    def wait_cost(label, expected, timeout=5.0):
        until = time.monotonic() + timeout
        consecutive = 0
        while time.monotonic() < until:
            tick()
            value = None if latest[0] is None else latest[0].data[CELL]
            consecutive = consecutive + 1 if value == expected else 0
            if consecutive >= 3:
                print(f'PASS: {label} (raw cost {value})', flush=True)
                return
        value = None if latest[0] is None else latest[0].data[CELL]
        raise AssertionError(f'{label}: expected {expected}, got {value}')

    try:
        with tempfile.TemporaryFile(mode='w+t') as output:
            proc = subprocess.Popen([sys.argv[1], '--ros-args', '--params-file', sys.argv[2]],
                                    stdout=output, stderr=subprocess.STDOUT)
            try:
                wait_cost('unchanged static wall', 254)
                send_mask[0] = True
                wait_cost('fresh verified evidence opens wall', 0)
                dynamic[0] = True
                wait_cost('live obstacle vetoes opening', 254)
                dynamic[0] = False
                wait_cost('cleared live obstacle reopens wall', 0)
                send_mask[0] = False
                wait_cost('expired evidence restores wall', 254, timeout=4.0)
            except BaseException:
                output.seek(0)
                print(output.read()[-5000:], file=sys.stderr)
                raise
            finally:
                proc.terminate()
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
