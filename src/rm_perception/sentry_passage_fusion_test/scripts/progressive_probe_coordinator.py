#!/usr/bin/env python3
"""2D candidate → ROGMap 3D clearance probe coordinator.

An occupied map cell can trigger a probe but can never clear a wall or grant
entry. Unknown/failed 3D validation leaves the existing gate closed.
"""
import json
from math import atan2, cos, hypot, isfinite, sin

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

from progressive_probe import choose_next_static_conflict


def stamp_seconds(stamp):
    return stamp.sec+stamp.nanosec*1e-9


class ProgressiveProbeCoordinator(Node):
    def __init__(self, runtime=False):
        remaps = {'/passage_fusion_test/map': '/map',
                  '/passage/memory_shadow_status': '/passage/memory_status'}
        for name in ('odom_map', 'desired_goal', 'probe_status'):
            remaps['/passage_fusion_test/'+name] = '/passage/'+name
        args = ['--ros-args'] + [v for a, b in remaps.items() for v in ('-r', a+':='+b)]
        super().__init__('progressive_probe_coordinator', cli_args=args if runtime else None)
        if self.get_namespace() != ('/passage' if runtime else '/passage_fusion_test'):
            raise ValueError('Probe must run in its isolated test or /passage namespace')
        self.evidence_frame = self.declare_parameter('evidence_frame', 'odom').value
        self.ground_z_offset = self.declare_parameter('ground_z_offset', 0.).value
        self.max_range = self.declare_parameter('max_range', 1.5).value
        if (not self.evidence_frame or not 0 < self.max_range <= 3. or
                not isfinite(self.ground_z_offset)):
            raise ValueError('Invalid probe frame or maximum range')
        self.grid = self.odom = self.goal = self.catalog = None
        self.catalog_at = 0.
        self.active = None
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        map_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, '/passage_fusion_test/map',
                                 lambda msg: setattr(self, 'grid', msg), map_qos)
        self.create_subscription(Odometry, '/passage_fusion_test/odom_map',
                                 lambda msg: setattr(self, 'odom', msg), 10)
        self.create_subscription(PoseStamped, '/passage_fusion_test/desired_goal',
                                 lambda msg: setattr(self, 'goal', msg), 10)
        self.create_subscription(String, '/passage/memory_shadow_status', self.on_catalog, 10)
        self.probe_pub = self.create_publisher(PoseStamped, '/terrain/clearance/probe', 10)
        self.status_pub = self.create_publisher(String, '/passage_fusion_test/probe_status', 10)
        self.create_timer(.2, self.tick)
        self.get_logger().warn('CANDIDATES ONLY: no motion command or passage authorization')

    def on_catalog(self, msg):
        try:
            data = json.loads(msg.data)
            if data['state'] not in ('KNOWN_PASSAGE', 'UNKNOWN') or not data['map_associated']:
                self.catalog = None
                self.active = None
                return
            self.catalog = data.get('segments', [])
            self.catalog_at = self.get_clock().now().nanoseconds/1e9
        except (ValueError, TypeError, KeyError):
            self.catalog = None
            self.active = None

    def report(self, reason, point=None):
        self.status_pub.publish(String(data=json.dumps({
            'state': reason, 'candidate': point,
            'entry_allowed': False, 'costmap_override': False}, sort_keys=True)))

    def tick(self):
        now = self.get_clock().now().nanoseconds/1e9
        if (self.grid is None or self.odom is None or self.goal is None or
                self.catalog is None or now-self.catalog_at > .5 or
                self.odom.header.frame_id != 'map' or self.goal.header.frame_id != 'map' or
                not 0 <= now-stamp_seconds(self.odom.header.stamp) <= .5):
            self.report('WAIT_FRESH_MAP_GOAL_POSE_AND_ASSOCIATION')
            return
        pose = self.odom.pose.pose.position
        target = self.goal.pose.position
        direction = atan2(target.y-pose.y, target.x-pose.x)
        if self.active is not None:
            x, y, yaw = self.active
            if abs(atan2(sin(direction-yaw), cos(direction-yaw))) < .2:
                verified = next((s for s in self.catalog if
                                 hypot(s['center_x']-x, s['center_y']-y) <
                                 max(self.grid.info.resolution, .12)), None)
                # Keep the same probe until the rear of the vehicle has left
                # its verified body volume. This sustains short-lived proof.
                if verified is None or (
                        (pose.x-x)*cos(yaw)+(pose.y-y)*sin(yaw) <=
                        (verified['travel_length']+verified['robot_length'])/2+.05):
                    self.report('HOLD_CURRENT_PROBE', [x, y, yaw])
                    return
            self.active = None
        try:
            candidate = choose_next_static_conflict(
                self.grid, (pose.x, pose.y), (target.x, target.y),
                self.catalog, self.max_range)
            if candidate is None:
                self.report('NO_OBSERVED_STATIC_CONFLICT_IN_RANGE')
                return
            transform = self.buffer.lookup_transform(
                self.evidence_frame, 'map', Time())
        except (ValueError, TransformException):
            self.report('INVALID_MAP_OR_TF')
            return
        x, y, yaw = candidate
        t = transform.transform.translation
        q = transform.transform.rotation
        if (not all(isfinite(v) for v in (t.x, t.y, t.z, q.x, q.y, q.z, q.w)) or
                abs(q.x)+abs(q.y) > .01 or
                abs(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w-1.) > .01):
            self.report('NONPLANAR_OR_INVALID_TF')
            return
        tf_yaw = atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        probe = PoseStamped()
        probe.header.frame_id = self.evidence_frame
        probe.header.stamp = self.get_clock().now().to_msg()
        probe.pose.position.x = t.x+cos(tf_yaw)*x-sin(tf_yaw)*y
        probe.pose.position.y = t.y+sin(tf_yaw)*x+cos(tf_yaw)*y
        probe.pose.position.z = t.z+pose.z+self.ground_z_offset
        probe.pose.orientation.w = cos((yaw+tf_yaw)/2)
        probe.pose.orientation.z = sin((yaw+tf_yaw)/2)
        self.probe_pub.publish(probe)
        self.active = candidate
        self.report('PROBE_SENT_NOT_PERMISSION', list(candidate))


def main():
    rclpy.init()
    node = None
    try:
        node = ProgressiveProbeCoordinator()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
