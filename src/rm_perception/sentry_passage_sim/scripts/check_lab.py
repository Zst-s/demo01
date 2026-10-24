#!/usr/bin/env python3
"""Bounded live smoke test; inspect only the isolated lab and publish diagnostic probes."""
import json
import argparse
import os
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.utilities import remove_ros_args
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from visualization_msgs.msg import MarkerArray


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--startup-only', action='store_true')
    parser.add_argument('--mobile', choices=('true', 'false'), default='false')
    args = parser.parse_args(remove_ros_args()[1:])
    domain_id = os.environ.get('ROS_DOMAIN_ID')
    if (domain_id not in {str(value) for value in range(219, 240)} or
            os.environ.get('ROS_LOCALHOST_ONLY') != '1'):
        raise SystemExit('Refusing: use isolated ROS_DOMAIN_ID 219..239 and '
                         'ROS_LOCALHOST_ONLY=1')
    rclpy.init()
    node = Node('passage_lab_check', parameter_overrides=[rclpy.parameter.Parameter('use_sim_time', value=True)])
    received = {}
    specs = [(PointCloud2, '/passage_sim/points', 'cloud'),
             (Odometry, '/passage_sim/sensor_odom', 'odom'),
             (OccupancyGrid, '/terrain/grid', 'grid'),
             (PointCloud2, '/rog_map/raw_occupied', 'occupied'),
             (MarkerArray, '/rog_map/classification_reasons', 'reasons'),
             (String, '/terrain/clearance/status', 'status')]
    subscriptions = [node.create_subscription(typ, topic, lambda msg, key=key: received.update({key: msg}),
                                              qos_profile_sensor_data) for typ, topic, key in specs]
    probe_pub = node.create_publisher(PoseStamped, '/terrain/clearance/probe', 1)

    def wait_for(predicate, seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.1)
            if predicate():
                return
        missing = [key for _, _, key in specs if key not in received]
        raise AssertionError(f'Timeout: missing={missing}, last_status={received.get("status")}')

    try:
        wait_for(lambda: len(received) == len(specs), 30)
        cloud, odom, grid, occupied = [received[key] for key in ('cloud', 'odom', 'grid', 'occupied')]
        assert cloud.header.frame_id == 'passage_lidar' and cloud.width*cloud.height > 100
        assert odom.header.frame_id == 'world' and abs(odom.pose.pose.position.z - .55) < .01
        assert grid.header.frame_id == 'world' and len(grid.data) == grid.info.width*grid.info.height
        assert all(value in (-1, 0, 100) for value in grid.data)
        assert 100 in grid.data and 0 in grid.data
        assert occupied.header.frame_id == 'world' and occupied.width*occupied.height > 10
        reason_counts = {m.ns: len(m.points) for m in received['reasons'].markers if m.ns != 'legend'}
        assert sum(reason_counts.values()) == grid.info.width*grid.info.height
        if args.mobile != 'true':
            assert not any('cmd_vel' in name for name, _ in node.get_topic_names_and_types())
        if args.startup_only:
            print('LAB READY: Gazebo cloud + sensor truth + ROGMap occupancy/reasons/grid received', flush=True)
            return
        results = {}
        for name, y in [('wide', 0.0), ('narrow', 2.0), ('low', -2.0), ('wall', 4.0)]:
            probe = PoseStamped()
            probe.header.frame_id = 'world'
            probe.header.stamp = node.get_clock().now().to_msg()
            probe.pose.position.x = 2.6
            probe.pose.position.y = y
            # Calibrated lab floor-return layer at 0.1 m resolution.
            probe.pose.position.z = -.025
            probe.pose.orientation.w = 1.0
            probe_pub.publish(probe)
            def valid_observation():
                fields = dict(word.split('=', 1) for word in received['status'].data.split() if '=' in word)
                if float(fields.get('probe_y', 'nan')) != y or float(fields.get('probe_x', 'nan')) != 2.6:
                    return False
                if int(fields.get('checked', '0')) == 0 or not 0 <= float(fields.get('input_age', 'inf')) <= .5:
                    return False
                if name == 'wide':
                    return fields.get('reason') in ('UNKNOWN_VOLUME_OR_SUPPORT', 'GEOMETRY_PASS_OBSERVATION_ONLY')
                return fields.get('reason') == 'BODY_VOLUME_OCCUPIED' and int(fields.get('blocked', '0')) > 0
            wait_for(valid_observation, 15)
            results[name] = received['status'].data
        print(json.dumps({'pipeline': 'PASS', 'raw_points': cloud.width*cloud.height,
                          'occupied_voxels': occupied.width*occupied.height,
                          'grid': [grid.info.width, grid.info.height], 'reason_counts': reason_counts,
                          'probes': results, 'motion_topics': 'none'}, indent=2))
    finally:
        del subscriptions
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
