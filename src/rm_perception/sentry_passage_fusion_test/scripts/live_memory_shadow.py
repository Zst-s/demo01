#!/usr/bin/env python3
"""Runtime: classified ROGMap hollow columns -> map-associated structure memory.

Legacy shadow/isolated modes keep their clearance diagnostic message interface.
Runtime never subscribes to body-clearance proofs or issues fresh entry permits.
No motion command is published by this node; the entry gate migration is separate.
An explicit map_session_id is required: matching this string alone is NOT proof
that two SLAM sessions share the same pose graph.
"""
import hashlib
import json
import math
import os
import copy

import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

from global_passage_memory import ConfirmedPassage, MapContext
from passage_memory_v2 import PassageMemoryV2
from continuous_passage_memory import ContinuousPassageMemory
from grid_fingerprint import grid_fingerprint
from structure_observation import parse_structure, structure_map_cells
from rigid_transform import context_transform, normalize_transform, transforms_close


def yaw_of(q):
    norm = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w
    if not math.isfinite(norm) or abs(norm - 1.0) > .01 or (
            abs(q.x) > .01 or abs(q.y) > .01):
        raise ValueError('INVALID_PLANAR_QUATERNION')
    return math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))


def map_context(grid, transform, map_session_id, allow_tilt=False):
    info = grid.info
    if (grid.header.frame_id != 'map' or not map_session_id or
            info.width <= 0 or info.height <= 0 or
            len(grid.data) != info.width * info.height or
            not math.isfinite(info.resolution) or info.resolution <= 0 or
            abs(yaw_of(info.origin.orientation)) > .001):
        raise ValueError('INVALID_MAP')
    t = transform.transform.translation
    q = transform.transform.rotation
    rigid = normalize_transform((t.x, t.y, t.z, q.x, q.y, q.z, q.w))
    if allow_tilt:
        _, _, _, x, y, z, w = rigid
        tf_yaw = math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))
    else:
        tf_yaw = yaw_of(q)
    context = MapContext(map_session_id, 'map', info.width, info.height,
                         info.resolution, info.origin.position.x, info.origin.position.y,
                         t.x, t.y, tf_yaw, rigid[2] if allow_tilt else 0.,
                         rigid[3:] if allow_tilt else ())
    if not context.valid():
        raise ValueError('INVALID_MAP_CONTEXT')
    return context


def parse_observation(raw, now, expected_frame,
                      expected_kind='verified_short_segment_observation'):
    if len(raw) > 8192:
        raise ValueError('EVIDENCE_TOO_LARGE')
    proof = json.loads(raw)
    keys = ('probe_x', 'probe_y', 'ground_z', 'yaw', 'travel_length',
            'robot_length', 'robot_width', 'robot_height', 'resolution',
            'measured_at', 'oldest_observation', 'newest_observation', 'valid_until')
    if (not isinstance(proof, dict) or proof.get('schema') != 1 or
            proof.get('kind') != expected_kind or
            proof.get('frame') != expected_frame or
            type(proof.get('confirmed_frames')) is not int or
            proof['confirmed_frames'] < 3 or
            type(proof.get('checked_voxels')) is not int or
            proof['checked_voxels'] <= 0 or
            any(type(proof.get(k)) not in (int, float) or
                not math.isfinite(proof[k]) for k in keys)):
        raise ValueError('INVALID_EVIDENCE_SCHEMA')
    if (min(proof['travel_length'], proof['robot_length'], proof['robot_width'],
            proof['robot_height'], proof['resolution'], proof['oldest_observation']) <= 0 or
            proof['newest_observation'] < proof['oldest_observation'] or
            proof['measured_at'] < proof['newest_observation'] or
            not 0 <= now - proof['measured_at'] <= 2.0 or
            not proof['measured_at'] <= proof['valid_until'] <= proof['measured_at'] + 1.0 or
            now > proof['valid_until']):
        raise ValueError('STALE_OR_INCONSISTENT_EVIDENCE')
    body_times = ('oldest_body_observation', 'newest_body_observation')
    if any(key in proof for key in body_times):
        if (not all(type(proof.get(key)) in (int, float) and
                    math.isfinite(proof[key]) for key in body_times) or
                not proof['oldest_observation'] <= proof['oldest_body_observation'] <=
                    proof['newest_body_observation'] <= proof['newest_observation']):
            raise ValueError('INVALID_BODY_OBSERVATION_TIMES')
    if 'oldest_support_observation' in proof:
        if (type(proof['oldest_support_observation']) not in (int, float) or
                not math.isfinite(proof['oldest_support_observation']) or
                not proof['oldest_observation'] <= proof['oldest_support_observation'] <=
                    proof['newest_observation']):
            raise ValueError('INVALID_SUPPORT_OBSERVATION_TIMES')
    if 'newest_support_observation' in proof:
        if ('oldest_support_observation' not in proof or
                type(proof['newest_support_observation']) not in (int, float) or
                not math.isfinite(proof['newest_support_observation']) or
                not proof['oldest_support_observation'] <= proof['newest_support_observation'] <=
                    proof['newest_observation']):
            raise ValueError('INVALID_SUPPORT_OBSERVATION_TIMES')
    coverage_keys = ('recent_body_voxels', 'body_voxels',
                     'entry_recent_window', 'recent_body_by_slice')
    if any(key in proof for key in coverage_keys):
        bins = proof.get('recent_body_by_slice')
        if (type(proof.get('recent_body_voxels')) is not int or
                type(proof.get('body_voxels')) is not int or
                type(proof.get('entry_recent_window')) not in (int, float) or
                not math.isfinite(proof['entry_recent_window']) or
                not 0 < proof['entry_recent_window'] <= 5 or
                not isinstance(bins, list) or len(bins) != 8 or
                any(not isinstance(pair, list) or len(pair) != 2 or
                    any(type(v) is not int for v in pair) or
                    not 0 <= pair[0] <= pair[1] for pair in bins) or
                sum(pair[0] for pair in bins) != proof['recent_body_voxels'] or
                sum(pair[1] for pair in bins) != proof['body_voxels'] or
                not 0 <= proof['recent_body_voxels'] <= proof['body_voxels']):
            raise ValueError('INVALID_BODY_FRESHNESS_COVERAGE')
    support_bins = proof.get('recent_support_by_slice')
    if support_bins is not None:
        if (not isinstance(support_bins, list) or len(support_bins) != 4 or
                any(not isinstance(pair, list) or len(pair) != 2 or
                    any(type(v) is not int for v in pair) or
                    not 0 <= pair[0] <= pair[1] for pair in support_bins)):
            raise ValueError('INVALID_SUPPORT_FRESHNESS_COVERAGE')
    entry_times = ('oldest_entry_observation', 'newest_entry_observation')
    if any(key in proof for key in entry_times):
        if (not all(type(proof.get(key)) in (int, float) and
                    math.isfinite(proof[key]) for key in entry_times) or
                type(proof.get('recent_entry_edge_voxels')) is not int or
                type(proof.get('entry_edge_voxels')) is not int or
                not 0 < proof['recent_entry_edge_voxels'] <= proof['entry_edge_voxels'] or
                not proof['oldest_observation'] <= proof['oldest_entry_observation'] <=
                    proof['newest_entry_observation'] <= proof['newest_observation']):
            raise ValueError('INVALID_ENTRY_EDGE_FRESHNESS')
        body_bins = proof.get('recent_body_by_slice')
        if body_bins is not None and support_bins is not None and (
                proof['recent_entry_edge_voxels'] !=
                    sum(pair[0] for pair in body_bins[6:8]) + support_bins[3][0] or
                proof['entry_edge_voxels'] !=
                    sum(pair[1] for pair in body_bins[6:8]) + support_bins[3][1]):
            raise ValueError('INCONSISTENT_ENTRY_EDGE_FRESHNESS')
    return proof


def _complete(pair):
    return (isinstance(pair, list) and len(pair) == 2 and
            type(pair[0]) is int and type(pair[1]) is int and
            pair[1] > 0 and pair[0] == pair[1])


def entry_edge_status(proof):
    """Check only the next forward slice, not the fully occluded tunnel interior."""
    body = proof.get('recent_body_by_slice')
    support = proof.get('recent_support_by_slice')
    if not isinstance(body, list) or len(body) != 8 or not isinstance(support, list) or len(support) != 4:
        return 'ENTRY_EDGE_NOT_MEASURED'
    # travel-axis bin 3 is the advancing edge in the probe's +yaw direction;
    # body bins interleave lower/upper height bands.
    if not _complete(body[6]) or not _complete(body[7]):
        return 'ENTRY_EDGE_BODY_NOT_RECENTLY_OBSERVED'
    if not _complete(support[3]):
        return 'ENTRY_EDGE_GROUND_SUPPORT_NOT_RECENTLY_OBSERVED'
    return 'ENTRY_EDGE_CURRENTLY_OBSERVED'


def entry_permit_window(proof, now, max_oldest_age):
    """Issue a brief permit from fresh forward-edge evidence, not a full-tunnel scan.

    Historical interior geometry may be assembled from controlled viewpoints.
    Motion is still incremental: only the leading body slice and its ground
    support can authorize the next short advance, while live obstacles veto it.
    """
    oldest_entry = proof.get('oldest_entry_observation')
    newest_entry = proof.get('newest_entry_observation')
    if (not math.isfinite(now) or not math.isfinite(max_oldest_age) or
            not 0 < max_oldest_age <= 5.0 or
            oldest_entry is None or newest_entry is None or
            not math.isfinite(oldest_entry) or not math.isfinite(newest_entry) or
            entry_edge_status(proof) != 'ENTRY_EDGE_CURRENTLY_OBSERVED' or
            type(proof.get('recent_entry_edge_voxels')) is not int or
            proof.get('recent_entry_edge_voxels') != proof.get('entry_edge_voxels') or
            not 0 <= now-newest_entry <= .5 or
            not 0 <= now-oldest_entry <= max_oldest_age):
        return None
    until = min(proof['valid_until'], oldest_entry+max_oldest_age,
                newest_entry+.5, proof['measured_at']+1.0)
    return until if until > now else None


def entry_recency_reason(proof, now, max_oldest_age):
    if entry_permit_window(proof, now, max_oldest_age) is not None:
        return 'FRESH_FORWARD_BODY_AND_GROUND_SUPPORT'
    edge_reason = entry_edge_status(proof)
    if edge_reason != 'ENTRY_EDGE_CURRENTLY_OBSERVED':
        return edge_reason
    newest = proof.get('newest_entry_observation')
    oldest = proof.get('oldest_entry_observation')
    if (newest is None or oldest is None or not math.isfinite(newest) or
            not math.isfinite(oldest) or not 0 <= now-newest <= .5 or
            not 0 <= now-oldest <= max_oldest_age):
        return 'ENTRY_WINDOW_EXPIRED'
    return 'ENTRY_EDGE_EVIDENCE_INCOMPLETE'


def passage_bbox(proof, transform, context):
    """Conservative *candidate* bounds; NEVER a clearing mask."""
    t = transform.transform.translation
    tf_yaw = yaw_of(transform.transform.rotation)
    center_x = t.x + math.cos(tf_yaw)*proof['probe_x'] - math.sin(tf_yaw)*proof['probe_y']
    center_y = t.y + math.sin(tf_yaw)*proof['probe_x'] + math.cos(tf_yaw)*proof['probe_y']
    yaw = tf_yaw + proof['yaw']
    half_length = (proof['travel_length'] + proof['robot_length']) / 2
    half_width = proof['robot_width'] / 2
    corners = [(center_x + math.cos(yaw)*dx - math.sin(yaw)*dy,
                center_y + math.sin(yaw)*dx + math.cos(yaw)*dy)
               for dx in (-half_length, half_length)
               for dy in (-half_width, half_width)]
    left = math.floor((min(p[0] for p in corners) - context.origin_x) / context.resolution)
    right = math.floor((max(p[0] for p in corners) - context.origin_x) / context.resolution) + 1
    bottom = math.floor((min(p[1] for p in corners) - context.origin_y) / context.resolution)
    top = math.floor((max(p[1] for p in corners) - context.origin_y) / context.resolution) + 1
    if not 0 <= left < right <= context.width or not 0 <= bottom < top <= context.height:
        raise ValueError('EVIDENCE_OUTSIDE_MAP')
    return left, right, bottom, top


def patch_hash(grid, passage):
    """Bind a saved candidate to the static-map cells it would touch."""
    width = grid.info.width
    if (not 0 <= passage.left < passage.right <= width or
            not 0 <= passage.bottom < passage.top <= grid.info.height):
        raise ValueError('INVALID_CANDIDATE_BOUNDS')
    values = [grid.data[y*width+x] for y in range(passage.bottom, passage.top)
              for x in range(passage.left, passage.right)]
    if any(v < -1 or v > 100 for v in values):
        raise ValueError('INVALID_MAP_CELL')
    return hashlib.sha256(bytes(v+1 for v in values)).hexdigest()


def association_stable(observed, current, max_shift=.25, max_yaw=.15):
    if (observed.epoch != current.epoch or
            observed.geometry() != current.geometry()):
        return False
    return transforms_close(context_transform(observed), context_transform(current),
                            max_shift, max_yaw)


class LiveMemoryShadow(Node):
    def __init__(self, runtime=False):
        super().__init__('live_passage_memory_shadow')
        self.runtime = runtime
        self.continuous = self.declare_parameter('continuous_structure_memory', runtime).value
        if self.continuous and not runtime:
            raise ValueError('Continuous memory requires the formal runtime')
        self.memory_type = ContinuousPassageMemory if self.continuous else PassageMemoryV2
        self.pending_terrain = None
        self.terrain_age = None
        if runtime and self.get_namespace() != '/passage':
            raise ValueError('Runtime memory requires /passage namespace')
        map_session_id = self.declare_parameter('map_session_id', '').value
        record_path = self.declare_parameter('record_path', '').value
        self.expected_frame = self.declare_parameter('evidence_frame', 'odom').value
        self.publish_test_mask = self.declare_parameter('publish_test_mask', False).value
        self.accept_virtual_preview = self.declare_parameter(
            'accept_virtual_preview', False).value
        if runtime and (self.accept_virtual_preview or self.publish_test_mask):
            raise ValueError('Runtime cannot accept virtual or test evidence')
        self.entry_max_observation_age = self.declare_parameter(
            'entry_max_observation_age', 2.0).value
        if self.publish_test_mask and self.get_namespace() != '/passage_fusion_test':
            raise ValueError('TEST MASK restricted to /passage_fusion_test namespace')
        if self.accept_virtual_preview and (
                self.get_namespace() != '/passage_fusion_test' or
                not self.publish_test_mask):
            raise ValueError('VIRTUAL PREVIEW requires isolated test mask mode')
        if not isinstance(map_session_id, str) or not map_session_id.strip() or (
                not isinstance(record_path, str) or not os.path.isabs(record_path) or
                not isinstance(self.expected_frame, str) or not self.expected_frame or
                not isinstance(self.entry_max_observation_age, (int, float)) or
                not math.isfinite(self.entry_max_observation_age) or
                not 0 < self.entry_max_observation_age <= 5.0):
            raise ValueError('Require nonempty map_session_id, absolute record_path and evidence_frame')
        self.map_session_id = map_session_id
        self.record_path = record_path
        if runtime:
            os.makedirs(os.path.dirname(record_path), mode=0o700, exist_ok=True)
        self.grid = None
        self.pending = None
        self.pending_structure = None
        self.last_structure_stamp = 0.0
        self.memory = self.memory_type()
        self.restored = False
        self.last_invalidation_time = 0.0
        self.current_map_associated = False
        self.reason = 'WAIT_MAP_AND_TF'
        self.last_entry_observation_reason = 'NO_PROOF'
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.on_map, latched)
        if runtime:
            self.structure_sub = self.create_subscription(
                PointCloud2, '/terrain/passage/structure', self.on_structure, 1)
            if self.continuous:
                self.terrain_sub = self.create_subscription(
                    OccupancyGrid, '/terrain/grid', self.on_terrain, 1)
        else:
            self.proof_sub = self.create_subscription(
                String, '/terrain/clearance/preview_segment' if self.accept_virtual_preview
                else '/terrain/clearance/verified_segment', self.on_evidence, 10)
        self.status = self.create_publisher(String, '/passage/memory_status' if runtime
                                            else '/passage/memory_shadow_status', 10)
        self.mask_pub = (self.create_publisher(
            OccupancyGrid, '/passage/verified_static_clearing' if runtime else
            '/passage_fusion_test/preview_static_clearing'
            if self.accept_virtual_preview else '/passage_fusion_test/test_evidence', 10)
            if self.publish_test_mask or runtime else None)
        self.permit_pub = (self.create_publisher(
            String, '/passage/fresh_permit' if runtime else '/passage_fusion_test/fresh_permit', 10)
            if not runtime and self.publish_test_mask and not self.accept_virtual_preview else None)
        self.create_timer(.2, self.tick)
        self.get_logger().warn(
            'RUNTIME: ROGMap hollow structure observations; no clearance proof or entry permit'
            if runtime else 'VIRTUAL PREVIEW: isolated correction mask only, no permit or motion output'
            if self.accept_virtual_preview else
            'ISOLATED TEST: publishes correction mask and fresh permits; no motion output'
            if self.publish_test_mask else
            'SHADOW ONLY: no Costmap correction, cmd_vel or entry permission')

    def on_map(self, msg):
        self.grid = msg

    def on_evidence(self, msg):
        # Keep only the newest observation. Exact-time TF lookup happens in timer
        # after TF subscriptions have had a chance to run in this executor.
        self.pending = msg.data

    def on_terrain(self, msg):
        self.pending_terrain = msg

    def on_structure(self, msg):
        self.pending_structure = msg

    def accept_structure(self, current, now):
        if self.pending_structure is None:
            return
        msg = self.pending_structure
        measured_at = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        try:
            if measured_at <= self.last_structure_stamp:
                self.pending_structure = None
                return
            if msg.width * msg.height == 0:
                self.reason = 'NO_HOLLOW_COLUMNS_IN_CURRENT_VIEW'
                self.pending_structure = None
                return
            observation = parse_structure(msg, now, self.expected_frame)
            transform = self.lookup(measured_at)
            observed_context = map_context(self.grid, transform, self.map_session_id, True)
            if not association_stable(observed_context, current):
                raise ValueError('MAP_ODOM_MOVED_SINCE_OBSERVATION')
            if measured_at <= self.last_invalidation_time:
                raise ValueError('STRUCTURE_PREDATES_MAP_INVALIDATION')
            t = transform.transform.translation
            q = transform.transform.rotation
            rigid = normalize_transform((t.x, t.y, t.z, q.x, q.y, q.z, q.w))
            cells = structure_map_cells(observation, current, rigid)
            if self.memory.state == 'SUSPENDED':
                if getattr(self.memory, 'continuous', False) and self.memory.reason == 'RECORD_WRITE_FAILED':
                    raise ValueError('MEMORY_STORAGE_RECOVERY_REQUIRED')
                self.memory = type(self.memory)()
            if cells:
                added = self.memory.add_structure(
                    context=observed_context, grid=list(self.grid.data),
                    observation=observation, cells=cells,
                    map_transform=rigid, now=now)
                if added is not None and not getattr(self.memory, 'continuous', False):
                    self.memory.save(self.record_path)
                self.reason = 'CLASSIFIED_HOLLOW_STRUCTURE_RECEIVED'
            else:
                self.reason = 'NO_STRUCTURE_CENTRES_IN_MAP'
            self.last_structure_stamp = measured_at
            self.pending_structure = None
        except (TransformException, ValueError, TypeError, OSError, OverflowError) as exc:
            self.reason = str(exc)
            if isinstance(exc, OSError):
                self.memory.suspend('RECORD_WRITE_FAILED')
            if not isinstance(exc, TransformException) or now-measured_at > 2:
                self.pending_structure = None

    def lookup(self, stamp):
        return self.buffer.lookup_transform(
            'map', self.expected_frame, Time(nanoseconds=int(stamp * 1e9)),
            timeout=Duration(seconds=0.0))

    def tick(self):
        now = self.get_clock().now().nanoseconds / 1e9
        self.current_map_associated = False
        if now <= 0 or self.grid is None:
            mask = self.publish_mask(None)
            self.publish_status('WAIT_MAP_OR_CLOCK', mask)
            return
        try:
            current_tf = self.lookup(0.0)
            current = map_context(self.grid, current_tf, self.map_session_id, self.runtime)
            self.current_map_associated = True
        except (TransformException, ValueError, OverflowError) as exc:
            self.memory.suspend('MAP_OR_TF_UNAVAILABLE')
            self.last_invalidation_time = now
            mask = self.publish_mask(None)
            self.publish_status(str(exc), mask)
            return
        if not self.restored:
            self.restored = True
            if not self.accept_virtual_preview and os.path.exists(self.record_path):
                self.memory = self.memory_type.restore(self.record_path, current,
                                                      list(self.grid.data))
                self.reason = self.memory.reason
        if self.memory.state == 'KNOWN_PASSAGE':
            try:
                before = len(self.memory.segments)+len(self.memory.structures)
                self.memory.check(current, list(self.grid.data))
                if self.memory.state == 'KNOWN_PASSAGE' and self.memory.needs_save:
                    # Persist a resized canvas even when the tunnel is out of view.
                    self.memory.save(self.record_path)
                    self.reason = self.memory.reason
                if (len(self.memory.segments)+len(self.memory.structures) < before or
                        self.memory.state == 'SUSPENDED'):
                    self.last_invalidation_time = now
            except (ValueError, OSError) as exc:
                self.memory.suspend('RECORD_WRITE_FAILED' if isinstance(exc, OSError)
                                    else 'INVALID_MAP_PATCH')
                self.last_invalidation_time = now
        if self.runtime:
            self.accept_structure(current, now)
        if self.pending is not None and not self.runtime:
            raw = self.pending
            try:
                proof = parse_observation(raw, now, self.expected_frame,
                    'virtual_short_segment_preview' if self.accept_virtual_preview
                    else 'verified_short_segment_observation')
                evidence_tf = self.lookup(proof['measured_at'])
                evidence_context = map_context(self.grid, evidence_tf, self.map_session_id)
                # If TF moved between observation and now, do not use an old
                # association. A new observation after correction is required.
                if not association_stable(evidence_context, current):
                    raise ValueError('MAP_ODOM_MOVED_SINCE_OBSERVATION')
                if proof['measured_at'] <= self.last_invalidation_time:
                    raise ValueError('EVIDENCE_PREDATES_MAP_INVALIDATION')
                # A map correction invalidates all old geometry. A genuinely
                # new, timestamped 3D observation may start a new association;
                # it must not resurrect any old segments or old entry permits.
                if self.memory.state == 'SUSPENDED':
                    self.memory = type(self.memory)()
                t = evidence_tf.transform.translation
                tf_yaw = yaw_of(evidence_tf.transform.rotation)
                center_x = t.x + math.cos(tf_yaw)*proof['probe_x'] - math.sin(tf_yaw)*proof['probe_y']
                center_y = t.y + math.sin(tf_yaw)*proof['probe_x'] + math.cos(tf_yaw)*proof['probe_y']
                segment_id = self.memory.add(context=evidence_context, grid=list(self.grid.data),
                                center_x=center_x, center_y=center_y,
                                yaw=tf_yaw+proof['yaw'],
                                travel_length=proof['travel_length'],
                                robot_length=proof['robot_length'],
                                robot_width=proof['robot_width'],
                                ground_z=t.z+proof['ground_z'],
                                robot_height=proof['robot_height'],
                                confirmed_frames=proof['confirmed_frames'],
                                measured_at=proof['measured_at'], now=now)
                if not self.accept_virtual_preview:
                    self.memory.save(self.record_path)
                permit_until = entry_permit_window(
                    proof, now, self.entry_max_observation_age)
                self.last_entry_observation_reason = entry_recency_reason(
                    proof, now, self.entry_max_observation_age)
                if self.permit_pub is not None and permit_until is not None:
                    self.permit_pub.publish(String(data=json.dumps({
                        'segment_id': segment_id,
                        'measured_at': proof['measured_at'],
                        'valid_until': permit_until,
                        'confirmed_frames': proof['confirmed_frames']})))
                self.pending = None
                self.reason = 'SAVED_HISTORICAL_CANDIDATE_ONLY'
            except (TransformException, ValueError, TypeError, OSError, json.JSONDecodeError) as exc:
                # TF may arrive after the message; retry only while the proof is
                # live. Any other failure does not create or extend a record.
                self.reason = str(exc)
                if isinstance(exc, OSError):
                    self.memory.suspend('RECORD_WRITE_FAILED')
                try:
                    if now > json.loads(raw).get('valid_until', 0):
                        self.pending = None
                except (ValueError, TypeError, AttributeError):
                    self.pending = None
        if getattr(self, 'continuous', False):
            self.refine_continuous(current, now)
        mask = self.publish_mask(current)
        self.publish_status(self.reason if self.memory.state == 'KNOWN_PASSAGE'
                            else self.memory.reason, mask)

    def refine_continuous(self, current, now):
        if not self.continuous:
            return
        try:
            terrain = self.pending_terrain
            if terrain is not None:
                at = terrain.header.stamp.sec + terrain.header.stamp.nanosec * 1e-9
                self.terrain_age = now-at
                if (terrain.header.frame_id != self.expected_frame or
                        abs(yaw_of(terrain.info.origin.orientation)) > .001):
                    raise ValueError('INVALID_TERRAIN_FRAME')
                if 0 <= self.terrain_age <= .7:
                    tf = self.lookup(at)
                    observed = map_context(self.grid, tf, self.map_session_id, True)
                    if not association_stable(observed, current):
                        raise ValueError('MAP_ODOM_MOVED_SINCE_TERRAIN')
                    t, q = tf.transform.translation, tf.transform.rotation
                    rigid = normalize_transform((t.x,t.y,t.z,q.x,q.y,q.z,q.w))
                    self.memory.refine(current, list(self.grid.data), terrain, rigid)
            if self.memory.state == 'KNOWN_PASSAGE' and self.memory.needs_save:
                self.memory.save(self.record_path)
            self.reason = self.memory.reason
        except (TransformException, ValueError, TypeError, OSError, OverflowError) as exc:
            self.reason = str(exc)

    def publish_mask(self, current):
        if self.mask_pub is None or self.grid is None:
            return None
        mask = OccupancyGrid()
        mask.header.frame_id = 'map'
        mask.header.stamp = self.get_clock().now().to_msg()
        mask.info = copy.deepcopy(self.grid.info)
        mask.data = [0] * len(self.grid.data)
        if self.memory.state == 'KNOWN_PASSAGE':
            try:
                for cell in self.memory.planning_cells(current, list(self.grid.data)):
                    mask.data[cell] = 100
            except ValueError:
                self.memory.suspend('INVALID_MAP_PATCH')
        self.mask_pub.publish(mask)
        return mask

    def publish_status(self, reason, mask=None):
        data = {'state': self.memory.state, 'reason': reason,
                'memory_reason': self.memory.reason,
                'last_entry_observation_reason': self.last_entry_observation_reason,
                'map_session_id': self.map_session_id,
                'map_associated': self.current_map_associated,
                'historical_candidate': self.memory.state == 'KNOWN_PASSAGE',
                'entry_allowed': False, 'costmap_override': False}
        data['virtual_preview'] = self.accept_virtual_preview
        data['observation_source'] = 'rog_map_hollow_structure' if self.runtime else 'clearance_diagnostic'
        data['structure_count'] = len(self.memory.structures)
        data['structure_cell_count'] = len({cell for record in self.memory.structures.values()
                                          for cell in record.cells})
        if getattr(self.memory, 'continuous', False):
            data.update(self.memory.diagnostics())
            data['terrain_age'] = self.terrain_age
        data['entry_gate_migration_pending'] = self.runtime
        if (self.accept_virtual_preview or self.runtime) and mask is not None:
            data['costmap_override'] = any(value == 100 for value in mask.data)
        if self.memory.state == 'KNOWN_PASSAGE':
            data['segment_ids'] = list(self.memory.segments)
            data['segment_count'] = len(self.memory.segments)
            data['segments'] = [{
                'segment_id': s.segment_id,
                'center_x': s.center_x, 'center_y': s.center_y,
                'yaw': s.yaw, 'travel_length': s.travel_length,
                'robot_length': s.robot_length,
                'robot_width': s.robot_width,
                'verified_width': s.robot_width,
                'ground_z': s.ground_z, 'robot_height': s.robot_height}
                for s in self.memory.segments.values()]
        if self.grid is not None and mask is not None:
            try:
                data['map_fingerprint'] = grid_fingerprint(self.grid)
                data['mask_fingerprint'] = grid_fingerprint(mask)
            except ValueError:
                pass  # The gate refuses a status without both identities.
        self.status.publish(String(data=json.dumps(data, sort_keys=True)))


def main():
    rclpy.init()
    node = None
    try:
        node = LiveMemoryShadow()
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
