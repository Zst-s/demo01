#!/usr/bin/env python3
"""Read-only NUC check of the live LIO -> ROGMap -> SLAM/TF chain.

This deliberately publishes nothing and never requests chassis motion.
"""

import argparse
import math
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener


def stamp_seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def terrain_at_static_cells(static, terrain, indices, transform):
    """Conservatively sample terrain beneath map cells; diagnosis, not permission."""
    q = transform.transform.rotation
    norm = math.sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w)
    if norm < 1e-9 or not math.isfinite(norm):
        raise ValueError('invalid map-to-terrain rotation')
    x, y, z, w = q.x/norm, q.y/norm, q.z/norm, q.w/norm
    roll = math.atan2(2*(w*x + y*z), 1 - 2*(x*x + y*y))
    pitch = math.asin(max(-1.0, min(1.0, 2*(w*y - z*x))))
    if abs(roll) > 0.1 or abs(pitch) > 0.1:
        raise ValueError('non-planar map-to-terrain transform')
    yaw = math.atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
    c, s = math.cos(yaw), math.sin(yaw)
    tx, ty = transform.transform.translation.x, transform.transform.translation.y
    if not all(math.isfinite(v) for v in (tx, ty, yaw)):
        raise ValueError('invalid map-to-terrain translation')
    half = 0.5 * static.info.resolution * (abs(c) + abs(s))
    origin = terrain.info.origin.position
    resolution = terrain.info.resolution
    if (not math.isfinite(resolution) or resolution <= 0 or
            len(terrain.data) != terrain.info.width * terrain.info.height or
            terrain.info.width == 0 or terrain.info.height == 0):
        raise ValueError('invalid terrain geometry')
    counts = {'occupied': 0, 'graded': 0, 'free': 0,
              'unknown': 0, 'outside': 0}
    for index in indices:
        mx = static.info.origin.position.x + (index % static.info.width + 0.5) * static.info.resolution
        my = static.info.origin.position.y + (index // static.info.width + 0.5) * static.info.resolution
        gx, gy = c*mx - s*my + tx, s*mx + c*my + ty
        x0 = math.floor((gx - half - origin.x) / resolution)
        y0 = math.floor((gy - half - origin.y) / resolution)
        x1 = math.floor((gx + half - 1e-9 - origin.x) / resolution)
        y1 = math.floor((gy + half - 1e-9 - origin.y) / resolution)
        outside = (x0 < 0 or y0 < 0 or x1 >= terrain.info.width or
                   y1 >= terrain.info.height)
        values = [terrain.data[row * terrain.info.width + col]
                  for row in range(max(0, y0), min(y1 + 1, terrain.info.height))
                  for col in range(max(0, x0), min(x1 + 1, terrain.info.width))]
        if any(v == 100 for v in values):
            counts['occupied'] += 1
        elif outside:
            counts['outside'] += 1
        elif not values or any(v < 0 for v in values):
            counts['unknown'] += 1
        elif any(v > 0 for v in values):
            counts['graded'] += 1
        else:
            counts['free'] += 1
    return counts


def path_crosses_claimed_wall(path, static, mask):
    """Check a map-frame path against currently claimed static wall cells."""
    if path.header.frame_id != 'map' or len(path.poses) < 2:
        return False
    resolution = static.info.resolution
    if not math.isfinite(resolution) or resolution <= 0:
        return False
    origin = static.info.origin.position
    points = [(pose.pose.position.x, pose.pose.position.y) for pose in path.poses]
    if not all(math.isfinite(value) for point in points for value in point):
        return False
    samples = 0
    for first, second in zip(points, points[1:]):
        distance = math.hypot(second[0] - first[0], second[1] - first[1])
        steps = max(1, math.ceil(distance / (resolution * 0.5)))
        samples += steps + 1
        if samples > 100000:
            return False
        for step in range(steps + 1):
            x = first[0] + (second[0] - first[0]) * step / steps
            y = first[1] + (second[1] - first[1]) * step / steps
            col = math.floor((x - origin.x) / resolution)
            row = math.floor((y - origin.y) / resolution)
            if 0 <= col < static.info.width and 0 <= row < static.info.height:
                index = row * static.info.width + col
                if static.data[index] == 100 and mask.data[index] == 100:
                    return True
    return False


class PassageDiagnostics(Node):
    def __init__(self, lio, fusion=False, expect_opening=False, expect_path=False):
        super().__init__('real_passage_diagnostics')
        self.clock_now = lambda: self.get_clock().now().nanoseconds * 1e-9
        self.received = {}
        self.grids = {}
        self.path = None
        self.fusion = fusion
        self.expect_opening = expect_opening
        self.expect_path = expect_path
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        cloud_topic = '/cloud_registered_body' if lio == 'fastlio' else '/cloud_registered'
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        specs = [
            ('/livox/lidar/pointcloud', PointCloud2, qos_profile_sensor_data),
            ('/segmentation/ground', PointCloud2, qos_profile_sensor_data),
            (cloud_topic, PointCloud2, qos_profile_sensor_data),
            ('/Odometry', Odometry, qos_profile_sensor_data),
            ('/scan', LaserScan, qos_profile_sensor_data),
            ('/terrain/grid', OccupancyGrid, QoSProfile(depth=1)),
            ('/map', OccupancyGrid, latched),
            ('/terrain/clearance/status', String, QoSProfile(depth=1)),
        ]
        self.optional_topics = set()
        if fusion:
            specs.extend([
                ('/passage/memory_status', String, QoSProfile(depth=1)),
                ('/global_costmap/costmap', OccupancyGrid, latched),
                ('/passage/verified_static_clearing', OccupancyGrid,
                 QoSProfile(depth=1)),
                ('/terrain/clearance/verified_segment', String,
                 QoSProfile(depth=1)),
                ('/passage/entry_gate_status', String, QoSProfile(depth=1)),
                ('/plan', Path, QoSProfile(depth=1)),
            ])
            self.optional_topics.update(('/passage/verified_static_clearing',
                                         '/terrain/clearance/verified_segment',
                                         '/passage/entry_gate_status', '/plan'))
        self.expected_topics = [name for name, _, _ in specs]
        self._topic_subscriptions = []
        for name, msg_type, qos in specs:
            self._topic_subscriptions.append(self.create_subscription(
                msg_type, name, lambda msg, topic=name: self.on_message(topic, msg), qos))

    def on_message(self, topic, msg):
        now = self.clock_now()
        stamp = getattr(getattr(msg, 'header', None), 'stamp', None)
        frame = getattr(getattr(msg, 'header', None), 'frame_id', '')
        age = now - stamp_seconds(stamp) if stamp is not None else None
        previous = self.received.get(topic, {})
        count = previous.get('count', 0) + 1
        info = {'count': count, 'last_rx': now, 'age': age, 'frame': frame}
        if isinstance(msg, PointCloud2):
            info['points'] = msg.width * msg.height
            for key in ('ground_z_median_in_cloud_frame',
                        'raw_z_lower_decile_in_cloud_frame', 'z_sample'):
                if key in previous:
                    info[key] = previous[key]
            if topic in ('/livox/lidar/pointcloud', '/segmentation/ground') and count <= 3:
                try:
                    points = point_cloud2.read_points(
                        msg, field_names=['x', 'y', 'z'], skip_nans=True)
                    if len(points):
                        radius = np.hypot(points['x'], points['y'])
                        z = points['z'][np.logical_and(radius >= 0.7, radius <= 4.0)]
                        if len(z) >= 20:
                            if topic == '/segmentation/ground':
                                info['ground_z_median_in_cloud_frame'] = round(
                                    float(np.median(z)), 3)
                            else:
                                info['raw_z_lower_decile_in_cloud_frame'] = round(
                                    float(np.percentile(z, 10)), 3)
                except (AssertionError, KeyError, TypeError, ValueError):
                    info['z_sample'] = 'unavailable'
        elif isinstance(msg, LaserScan):
            info['finite_ranges'] = sum(math.isfinite(value) for value in msg.ranges)
        elif isinstance(msg, OccupancyGrid):
            info['cells'] = len(msg.data)
            info['known_cells'] = sum(value >= 0 for value in msg.data)
            self.grids[topic] = msg
        elif isinstance(msg, Path):
            info['poses'] = len(msg.poses)
            self.path = msg
        elif isinstance(msg, String):
            info['detail'] = msg.data[:300]
        self.received[topic] = info

    def report(self):
        now = self.clock_now()
        failed = False
        for topic in self.expected_topics:
            info = self.received.get(topic)
            if info is None:
                label = 'OPTIONAL_ABSENT' if topic in self.optional_topics else 'MISSING'
                print(f'{label} {topic}')
                failed |= topic not in self.optional_topics
                continue
            rx_age = now - info['last_rx']
            # A saved /map may be published only once and remain valid through
            # transient-local delivery. Do not call it stale after ten seconds.
            fresh = True if topic == '/map' else 0 <= rx_age <= 2.0
            if topic not in ('/map', '/terrain/clearance/status'):
                fresh = fresh and (info['age'] is None or
                                   -0.2 <= info['age'] <= (60.0 if topic == '/map' else 2.0))
            label = 'LATCHED' if topic == '/map' else 'OK' if fresh else 'STALE'
            if topic not in self.optional_topics:
                failed |= not fresh
            fields = ' '.join(f'{key}={value}' for key, value in info.items()
                              if key not in ('last_rx', 'detail'))
            print(f'{label} {topic} rx_age={rx_age:.2f}s {fields}')
            if 'detail' in info:
                print(f'  {info["detail"]}')
        for parent, child in [('map', 'odom'), ('odom', 'base_link'),
                              ('base_link', 'livox_frame')]:
            try:
                transform = self.tf_buffer.lookup_transform(parent, child, Time())
                t = transform.transform.translation
                stamp = stamp_seconds(transform.header.stamp)
                age = now - stamp if stamp else 0.0
                dynamic = child in ('odom', 'base_link')
                fresh = not dynamic or 0 <= age <= 2.0
                print(f'{"OK" if fresh else "STALE"} TF {parent} -> {child}: '
                      f'xyz=({t.x:.3f},{t.y:.3f},{t.z:.3f}), age={age:.2f}s')
                failed |= not fresh
            except TransformException as exc:
                print(f'MISSING TF {parent} -> {child}: {exc}')
                failed = True
        if self.fusion:
            failed |= self.report_fusion()
        print('READ_ONLY: no costmap correction, passage permit, or velocity command was issued')
        return 1 if failed else 0

    def report_fusion(self):
        static = self.grids.get('/map')
        mask = self.grids.get('/passage/verified_static_clearing')
        final = self.grids.get('/global_costmap/costmap')
        if static is None or final is None:
            print('FUSION_NOT_OBSERVABLE: /map or final Costmap absent')
            return True
        if mask is None:
            print('NO_ACTIVE_VERIFIED_MASK: static wall must remain unchanged')
            return self.expect_opening or self.expect_path
        mask_info = self.received.get('/passage/verified_static_clearing', {})
        final_info = self.received.get('/global_costmap/costmap', {})
        now = self.clock_now()
        if (now - mask_info.get('last_rx', 0) > 1.0 or
                now - final_info.get('last_rx', 0) > 2.0 or
                not -0.1 <= now - stamp_seconds(mask.header.stamp) <= 1.0):
            print('FUSION_SNAPSHOT_STALE: cannot credit an old opening')
            return True
        same_mask_geometry = (static.header.frame_id == mask.header.frame_id == 'map' and
            static.info.width == mask.info.width and
            static.info.height == mask.info.height and
            static.info.resolution == mask.info.resolution and
            static.info.origin == mask.info.origin and
            static.info.map_load_time == mask.info.map_load_time)
        same_costmap_geometry = (static.header.frame_id == final.header.frame_id and
            static.info.width == final.info.width and
            static.info.height == final.info.height and
            abs(static.info.resolution - final.info.resolution) < 1e-7 and
            abs(static.info.origin.position.x - final.info.origin.position.x) < 1e-7 and
            abs(static.info.origin.position.y - final.info.origin.position.y) < 1e-7)
        if not same_mask_geometry or not same_costmap_geometry:
            print('FUSION_GEOMETRY_MISMATCH: /map, evidence mask and Costmap '
                  'cannot be compared cell-for-cell')
            return True
        if len(static.data) != len(mask.data) or len(static.data) != len(final.data):
            print('FUSION_DATA_LENGTH_MISMATCH')
            return True
        claimed = [i for i, (wall, proof) in enumerate(zip(static.data, mask.data))
                   if wall == 100 and proof == 100]
        nonlethal = sum(0 <= final.data[i] < 99 for i in claimed)
        reblocked = len(claimed) - nonlethal
        print(f'FUSION_STATIC_WALL_CELLS claimed={len(claimed)} '
              f'final_nonlethal={nonlethal} final_blocked_or_inscribed={reblocked}')
        terrain = self.grids.get('/terrain/grid')
        terrain_info = self.received.get('/terrain/grid', {})
        terrain_age = now - stamp_seconds(terrain.header.stamp) if terrain else float('inf')
        if (claimed and terrain is not None and
                0 <= now - terrain_info.get('last_rx', 0) <= 2.0 and
                -0.1 <= terrain_age <= 2.0):
            try:
                transform = self.tf_buffer.lookup_transform(
                    terrain.header.frame_id, static.header.frame_id,
                    Time.from_msg(terrain.header.stamp))
                counts = terrain_at_static_cells(static, terrain, claimed, transform)
                print('FUSION_TERRAIN_AT_CLAIMED_WALL '
                      + ' '.join(f'{key}={value}' for key, value in counts.items()))
                if counts['occupied']:
                    print('TERRAIN_VETO_PRESENT: TerrainLayer can re-close these cells; '
                          'compare 3D body-volume proof with the 2D terrain projection')
            except (TransformException, ValueError) as exc:
                print(f'FUSION_TERRAIN_COMPARISON_UNAVAILABLE: {exc}')
        if claimed and reblocked:
            print('CHECK_TERRAIN_AND_INFLATION: later layers may have closed '
                  'the verified static gap; do not claim P3 passed')
        if not claimed:
            print('NO_STATIC_WALL_CLEARED: no verified mask cell overlaps a /map wall')
        path_crosses = False
        if self.path is not None:
            path_info = self.received.get('/plan', {})
            path_age = now - stamp_seconds(self.path.header.stamp)
            if (0 <= now - path_info.get('last_rx', 0) <= 2.0 and
                    -0.1 <= path_age <= 2.0):
                path_crosses = path_crosses_claimed_wall(
                    self.path, static, mask)
            print(f'FUSION_PATH poses={len(self.path.poses)} '
                  f'fresh={-0.1 <= path_age <= 2.0} '
                  f'crosses_claimed_static_wall={path_crosses}')
        elif self.expect_path:
            print('FUSION_PATH_MISSING: no /plan received during observation')
        return ((self.expect_opening and (not claimed or nonlethal == 0)) or
                (self.expect_path and not path_crosses))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--lio', choices=['fastlio', 'pointlio'], default='fastlio')
    parser.add_argument('--seconds', type=float, default=8.0)
    parser.add_argument('--fusion', action='store_true',
                        help='Inspect live /map, verified mask and final Costmap; read-only')
    parser.add_argument('--expect-opening', action='store_true',
                        help='Fail if no verified static wall cell is nonlethal in final Costmap')
    parser.add_argument('--expect-path', action='store_true',
                        help='Also fail unless a fresh /plan crosses a currently claimed static wall')
    args = parser.parse_args()
    if not 2.0 <= args.seconds <= 30.0:
        parser.error('--seconds must be between 2 and 30')
    rclpy.init()
    node = PassageDiagnostics(args.lio, args.fusion or args.expect_opening or args.expect_path,
                              args.expect_opening or args.expect_path, args.expect_path)
    try:
        until = time.monotonic() + args.seconds
        while rclpy.ok() and time.monotonic() < until:
            rclpy.spin_once(node, timeout_sec=0.1)
        code = node.report()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(code)


if __name__ == '__main__':
    main()
