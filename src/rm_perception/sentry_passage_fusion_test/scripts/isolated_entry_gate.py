#!/usr/bin/env python3
"""Fail-closed passage entry gate for isolated tests or the /passage runtime.

Inputs are an already-associated map-frame path and odometry, desired velocity,
fresh obstacle state, historical segment catalog, and fresh segment permit.
The test mode has no chassis output. Runtime mode publishes only an authorized
intermediate command; the separate output bridge still enforces its own veto.
"""
import json
from math import hypot, isfinite

import rclpy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String

from passage_entry_manager import ControlledSegment, FreshPermit, PassageEntryManager
from static_route_guard import route_limit
from grid_fingerprint import grid_fingerprint


def seconds(stamp):
    return stamp.sec + stamp.nanosec*1e-9


class IsolatedEntryGate(Node):
    def __init__(self, runtime=False):
        remaps = {
            '/passage/memory_shadow_status': '/passage/memory_status',
            '/passage_fusion_test/map': '/map',
            '/passage_fusion_test/test_evidence': '/passage/verified_static_clearing'}
        for name in ('cmd_vel_authorized', 'entry_gate_status', 'fresh_permit',
                     'planned_path', 'odom_map', 'live_obstacle', 'cmd_vel_desired'):
            remaps['/passage_fusion_test/'+name] = '/passage/'+name
        args = ['--ros-args'] + [v for a, b in remaps.items() for v in ('-r', a+':='+b)]
        super().__init__('passage_entry_gate' if runtime else 'isolated_entry_gate',
                         cli_args=args if runtime else None)
        if self.get_namespace() != ('/passage' if runtime else '/passage_fusion_test'):
            raise ValueError('Entry gate must run in its isolated test or /passage namespace')
        self.manager = PassageEntryManager(path_max_age=2.0 if runtime else .5)
        self.catalog = None
        self.catalog_received = 0.
        self.permits = {}
        self.path = None
        self.odom = None
        self.obstacle = None
        self.desired = None
        self.static_map = None
        self.static_hash = None
        self.correction_mask = None
        self.mask_received = 0.
        self.authorized = self.create_publisher(
            TwistStamped, '/passage_fusion_test/cmd_vel_authorized', 10)
        self.status_pub = self.create_publisher(
            String, '/passage_fusion_test/entry_gate_status', 10)
        self.create_subscription(String, '/passage/memory_shadow_status', self.on_catalog, 10)
        self.create_subscription(String, '/passage_fusion_test/fresh_permit', self.on_permit, 10)
        self.create_subscription(Path, '/passage_fusion_test/planned_path', self.on_path, 10)
        self.create_subscription(Odometry, '/passage_fusion_test/odom_map', self.on_odom, 10)
        self.create_subscription(Bool, '/passage_fusion_test/live_obstacle', self.on_obstacle, 10)
        self.create_subscription(TwistStamped, '/passage_fusion_test/cmd_vel_desired',
                                 self.on_desired, 10)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, '/passage_fusion_test/map',
                                 self.on_map, latched)
        self.create_subscription(OccupancyGrid, '/passage_fusion_test/test_evidence',
                                 self.on_mask, 10)
        self.create_timer(.05, self.tick)
        self.get_logger().warn('Runtime gate: missing input stops output' if runtime else
                               'ISOLATED GATE ONLY; not connected to Nav2 or real robot control')

    def now(self):
        return self.get_clock().now().nanoseconds/1e9

    def on_catalog(self, msg):
        try:
            data = json.loads(msg.data)
            if data['state'] not in ('KNOWN_PASSAGE', 'UNKNOWN', 'SUSPENDED'):
                raise ValueError('INVALID_MEMORY_STATE')
            raw = data.get('segments', []) if data['state'] == 'KNOWN_PASSAGE' else []
            if len(raw) > 128 or (data['state'] == 'KNOWN_PASSAGE' and not raw):
                raise ValueError('TOO_MANY_SEGMENTS')
            segments = [ControlledSegment(**part) for part in raw]
            if not all(segment.valid() for segment in segments):
                raise ValueError('INVALID_SEGMENT')
            if len({segment.segment_id for segment in segments}) != len(segments):
                raise ValueError('DUPLICATE_SEGMENT_ID')
            if type(data.get('map_associated')) is not bool:
                raise ValueError('MAP_ASSOCIATION_MISSING')
            fingerprints = (data.get('map_fingerprint'), data.get('mask_fingerprint'))
            if not all(isinstance(value, str) and len(value) == 64 for value in fingerprints):
                raise ValueError('MAP_MASK_PAIR_IDENTITY_MISSING')
            self.catalog = (data['state'], segments, data['map_associated'], *fingerprints)
            self.catalog_received = self.now()
            active = {s.segment_id for s in segments}
            self.permits = {key: value for key, value in self.permits.items() if key in active}
        except (ValueError, TypeError, KeyError):
            self.catalog = None
            self.permits.clear()

    def on_permit(self, msg):
        try:
            data = json.loads(msg.data)
            permit = FreshPermit(**data)
            if self.catalog is not None and any(
                    permit.valid_for(s, self.now()) for s in self.catalog[1]):
                self.permits[permit.segment_id] = permit
        except (ValueError, TypeError, KeyError):
            pass

    def on_path(self, msg):
        self.path = msg if msg.header.frame_id == 'map' and 2 <= len(msg.poses) <= 10000 else None

    def on_odom(self, msg):
        self.odom = msg if msg.header.frame_id == 'map' and msg.child_frame_id == 'base_link' else None

    def on_obstacle(self, msg):
        self.obstacle = (bool(msg.data), self.now())

    def on_desired(self, msg):
        self.desired = msg if msg.header.frame_id == 'base_link' else None

    def on_map(self, msg):
        try:
            new_hash = grid_fingerprint(msg)
        except ValueError:
            new_hash = None
        if new_hash != self.static_hash or new_hash is None:
            # An actually changed map invalidates the paired clearing mask.
            # Repeated publications of identical /map must not interrupt motion.
            self.correction_mask = None
        self.static_map = msg if new_hash is not None else None
        self.static_hash = new_hash

    def on_mask(self, msg):
        self.correction_mask = msg
        self.mask_received = self.now()

    def valid_mask(self, now):
        grid, mask = self.static_map, self.correction_mask
        if (grid is None or mask is None or not 0 <= now-self.mask_received <= .5 or
                grid.header.frame_id != mask.header.frame_id or
                grid.info.width != mask.info.width or
                grid.info.height != mask.info.height or
                grid.info.resolution != mask.info.resolution or
                grid.info.origin != mask.info.origin or
                grid.info.map_load_time != mask.info.map_load_time or
                len(mask.data) != len(grid.data)):
            return False
        if not all(value in (0, 100) for value in mask.data):
            return False
        try:
            return (grid_fingerprint(grid) == self.catalog[3] and
                    grid_fingerprint(mask) == self.catalog[4])
        except ValueError:
            return False

    def publish(self, state, reason, cmd=None):
        now = self.now()
        output = TwistStamped()
        output.header.frame_id = 'base_link'
        output.header.stamp = self.get_clock().now().to_msg()
        if cmd is not None:
            output.twist = cmd
        self.authorized.publish(output)
        self.status_pub.publish(String(data=json.dumps({
            'state': state, 'reason': reason, 'stamp': now,
            'linear_x': output.twist.linear.x,
            'angular_z': output.twist.angular.z}, sort_keys=True)))

    def tick(self):
        now = self.now()
        if (self.catalog is None or now-self.catalog_received > .5 or
                self.path is None or self.odom is None or self.obstacle is None or
                self.desired is None or not self.valid_mask(now)):
            self.publish('BLOCKED', 'MISSING_INPUT')
            return
        if (not 0 <= now-self.obstacle[1] <= .5 or
                not 0 <= now-seconds(self.path.header.stamp) <= self.manager.path_max_age or
                not -0.1 <= now-seconds(self.desired.header.stamp) <= .2 or
                not -0.1 <= now-seconds(self.odom.header.stamp) <= .2):
            self.publish('BLOCKED', 'STALE_OBSTACLE_OR_COMMAND')
            return
        if self.catalog[0] == 'SUSPENDED':
            self.publish('BLOCKED', 'MAP_ASSOCIATION_SUSPENDED')
            return
        if not self.catalog[2]:
            self.publish('BLOCKED', 'MAP_OR_TF_NOT_ASSOCIATED')
            return
        pose = self.odom.pose.pose.position
        speed = hypot(self.odom.twist.twist.linear.x, self.odom.twist.twist.linear.y)
        path = [(p.pose.position.x, p.pose.position.y) for p in self.path.poses]
        if (not all(isfinite(v) for point in path for v in point) or
                not all(isfinite(v) for v in (pose.x, pose.y, speed))):
            self.publish('BLOCKED', 'NONFINITE_MOTION_INPUT')
            return
        decisions = self.manager.evaluate_catalog(
            segments=self.catalog[1], permits=self.permits,
            path=path, path_stamp=seconds(self.path.header.stamp),
            pose=(pose.x, pose.y), pose_stamp=seconds(self.odom.header.stamp),
            speed=speed, now=now,
            obstacle=self.obstacle[0], obstacle_stamp=self.obstacle[1],
            map_associated=self.catalog[0] == 'KNOWN_PASSAGE')
        grid = self.static_map
        route = route_limit(
            path, (pose.x, pose.y), speed,
            width=grid.info.width, height=grid.info.height,
            resolution=grid.info.resolution,
            origin_x=grid.info.origin.position.x,
            origin_y=grid.info.origin.position.y,
            static_cells=grid.data, verified_cells=self.correction_mask.data,
            robot_length=max((s.robot_length for s in self.catalog[1]), default=.45))
        if route.stop:
            self.publish('BLOCKED', route.reason)
            return
        # Before discovering a passage, the rover must be able to explore
        # already-free map cells. The independent static-route guard still
        # brakes before any occupied/unknown cell. A suspended association
        # was rejected above and never reaches this branch.
        if not decisions and self.catalog[0] != 'UNKNOWN':
            self.publish('BLOCKED', 'NO_VALID_CONTROLLED_SEGMENT')
            return
        if any(d.stop for d in decisions):
            reason = next(d.reason for d in decisions if d.stop)
            self.publish('BLOCKED', reason)
            return
        output = TwistStamped().twist
        output.linear.x = self.desired.twist.linear.x
        output.linear.y = self.desired.twist.linear.y
        output.angular.z = self.desired.twist.angular.z
        state = 'ROUTE_CLEAR'
        if route.speed_limit < float('inf'):
            state = 'WAIT_VERIFY'
            demanded = hypot(output.linear.x, output.linear.y)
            if demanded > route.speed_limit > 0:
                output.linear.x *= route.speed_limit/demanded
                output.linear.y *= route.speed_limit/demanded
        if any(d.state == 'WAIT_VERIFY' for d in decisions):
            state = 'WAIT_VERIFY'
            limit = min(route.speed_limit, *(d.speed_limit for d in decisions
                                             if d.state == 'WAIT_VERIFY'))
            demanded = hypot(output.linear.x, output.linear.y)
            if demanded > limit > 0:
                output.linear.x *= limit/demanded
                output.linear.y *= limit/demanded
        elif any(d.state == 'ALLOWED' for d in decisions):
            if route.speed_limit == float('inf'):
                state = 'ALLOWED'
            if abs(output.angular.z) > .1:
                self.publish('BLOCKED', 'TURN_RATE_EXCEEDS_VERIFIED_SWEEP')
                return
        if self.obstacle[0]:
            self.publish('BLOCKED', 'LIVE_OBSTACLE_VETO')
            return
        self.publish(state, 'CURRENT_INPUTS_VALID', output)


def main():
    rclpy.init()
    node = None
    try:
        node = IsolatedEntryGate()
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
