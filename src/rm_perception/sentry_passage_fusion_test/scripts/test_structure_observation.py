#!/usr/bin/env python3
"""Classified columns -> runtime intake -> persistent static cells, without proofs."""
import json
import math
import tempfile
import os
import sys
import time
import unittest
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace

from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import PointField
from sensor_msgs_py.point_cloud2 import create_cloud
from std_msgs.msg import Header

from global_passage_memory import MapContext
from live_memory_shadow import LiveMemoryShadow
from passage_memory_v2 import PassageMemoryV2
from passage_record_handover import export_record
from structure_observation import parse_structure, structure_map_cells


def cloud(columns=None):
    header = Header(frame_id='odom')
    header.stamp.sec = 10
    fields = [PointField(name=name, offset=i*4, datatype=PointField.FLOAT32, count=1)
              for i, name in enumerate(('x', 'y', 'z', 'ceiling_z', 'resolution'))]
    if columns is None:
        columns = [(.025+x*.05, .025+y*.05, -.2, .2, .05)
                   for x in range(4) for y in range(4)]
    return create_cloud(header, fields, columns)


class StructureObservationTests(unittest.TestCase):
    def setUp(self):
        self.context = MapContext('mapping-test', 'map', 10, 10, .05, 0., 0., 0., 0., 0.)
        self.grid = [100]*100
        self.tf = TransformStamped()
        self.tf.transform.rotation.w = 1.

    def test_runtime_intake_does_not_need_body_proof_and_restores(self):
        with tempfile.TemporaryDirectory() as directory:
            record_path = str(Path(directory)/'structure.json')
            adapter = SimpleNamespace(
                pending_structure=cloud(), last_structure_stamp=0.,
                last_invalidation_time=0., expected_frame='odom',
                map_session_id='mapping-test', record_path=record_path,
                memory=PassageMemoryV2(), grid=SimpleNamespace(data=self.grid),
                lookup=lambda stamp: self.tf)
            # Exact runtime intake method, without a ROS transport or any
            # ClearanceObserver, probe, checked_voxels or confirmations.
            import live_memory_shadow
            original = live_memory_shadow.map_context
            live_memory_shadow.map_context = lambda *args: self.context
            try:
                LiveMemoryShadow.accept_structure(adapter, self.context, 10.1)
                self.assertEqual(adapter.memory.state, 'KNOWN_PASSAGE', adapter.reason)
                self.assertFalse(adapter.memory.segments)
                self.assertEqual(len(adapter.memory.structures), 1)
                expected = {y*10+x for y in range(4) for x in range(4)}
                self.assertEqual(adapter.memory.planning_cells(self.context, self.grid), expected)
                wrapper = json.loads(Path(record_path).read_text())
                self.assertEqual(wrapper['body']['version'], 3)
                self.assertNotIn('robot_height', wrapper['body']['structures'][0])
                restored = PassageMemoryV2.restore(record_path, self.context, self.grid)
                self.assertEqual(restored.planning_cells(self.context, self.grid), expected)
                # Repeated observations neither grow records nor fabricate permits.
                adapter.pending_structure = cloud()
                adapter.pending_structure.header.stamp.nanosec = 100_000_000
                LiveMemoryShadow.accept_structure(adapter, self.context, 10.2)
                self.assertEqual(len(adapter.memory.structures), 1)
                changed = self.grid.copy()
                changed[0] = 0
                self.assertEqual(restored.planning_cells(self.context, changed), expected-{0})
                self.assertEqual(restored.planning_cells(self.context, self.grid), expected)
                target = 'saved-map-'+'a'*64
                output = Path(directory)/'handover.json'
                self.assertEqual(export_record(record_path, output, target), 1)
                target_context = replace(self.context, epoch=target)
                migrated = PassageMemoryV2.restore(output, target_context, self.grid)
                self.assertEqual(migrated.planning_cells(target_context, self.grid), expected)
            finally:
                live_memory_shadow.map_context = original

    def test_missing_column_is_not_filled_and_unknown_map_is_not_cleared(self):
        msg = cloud([(.025+x*.05, .025+y*.05, -.2, .2, .05)
                     for x in range(4) for y in range(4) if (x, y) != (1, 1)])
        observation = parse_structure(msg, 10.1, 'odom')
        cells = structure_map_cells(observation, self.context, (0., 0., 0., 0.))
        self.assertNotIn(11, cells)
        grid = self.grid.copy()
        grid[0] = -1
        grid[1] = 0
        memory = PassageMemoryV2()
        memory.add_structure(context=self.context, grid=grid, observation=observation,
                             cells=cells, map_transform=(0., 0., 0., 0.), now=10.1)
        cleared = memory.planning_cells(self.context, grid)
        self.assertNotIn(0, cleared)
        self.assertNotIn(1, cleared)
        self.assertNotIn(11, cleared)

    def test_rejects_stale_wrong_frame_and_nonfinite_columns(self):
        for msg, now, frame in ((cloud(), 13., 'odom'), (cloud(), 10.1, 'map'),
                               (cloud([(0., 0., math.nan, .2, .05)]), 10.1, 'odom')):
            with self.assertRaises(ValueError):
                parse_structure(msg, now, frame)

    def stored_structure(self):
        observation = parse_structure(cloud(), 10.1, 'odom')
        cells = structure_map_cells(observation, self.context, (0., 0., 0., 0.))
        memory = PassageMemoryV2()
        memory.add_structure(context=self.context, grid=self.grid, observation=observation,
                             cells=cells, map_transform=(0., 0., 0., 0.), now=10.1)
        return memory, observation

    def test_expansion_reprojects_noninteger_origin_and_keeps_static_intersection(self):
        memory, observation = self.stored_structure()
        grown = replace(self.context, width=12, height=12, origin_x=-.025, origin_y=-.025)
        grid = [100]*144
        grid[13], grid[14] = -1, 0
        expected = {y*12+x for y in range(1, 5) for x in range(1, 5)} - {13, 14}
        ids = set(memory.structures)
        self.assertEqual(memory.planning_cells(grown, grid), expected)
        self.assertEqual(memory.reason, 'MAPPING_EXPANSION_REPROJECTED')
        self.assertTrue(memory.needs_save)
        self.assertEqual(set(memory.structures), ids)
        for _ in range(3):
            self.assertEqual(memory.planning_cells(grown, grid), expected)
        self.assertEqual(len(memory.structures), 1)
        # Reobserving identical geometry must not create a new record or permit.
        memory.add_structure(context=grown, grid=grid, observation=observation,
            cells=structure_map_cells(observation, grown, (0.,0.,0.,0.)),
            map_transform=(0.,0.,0.,0.), now=10.2)
        self.assertEqual(len(memory.structures), 1)
        with tempfile.TemporaryDirectory() as directory:
            record = str(Path(directory)/'grown.json')
            memory.save(record)
            self.assertFalse(memory.needs_save)
            restored = PassageMemoryV2.restore(record, grown, grid)
            self.assertEqual(restored.planning_cells(grown, grid), expected)

    def test_old_structure_record_restores_on_same_session_expanded_map(self):
        memory, _ = self.stored_structure()
        grown = replace(self.context, width=12, height=12, origin_x=-.025, origin_y=-.025)
        with tempfile.TemporaryDirectory() as directory:
            record = str(Path(directory)/'old.json')
            memory.save(record)
            restored = PassageMemoryV2.restore(record, grown, [100]*144)
            self.assertEqual(restored.state, 'KNOWN_PASSAGE')
            self.assertTrue(restored.needs_save)
            restored.save(record)
            self.assertFalse(restored.needs_save)
            self.assertEqual(json.loads(Path(record).read_text())['body']['context']['width'], 12)
            self.assertEqual(restored.planning_cells(grown, [100]*144),
                             {y*12+x for y in range(1,5) for x in range(1,5)})

    def test_expansion_retains_reload_resolution_tf_and_suspended_guards(self):
        grown = replace(self.context, width=12, height=12, origin_x=-.025, origin_y=-.025)
        for context in (replace(grown, epoch='mapping-new'),
                        replace(grown, resolution=.051),
                        replace(grown, width=9),
                        replace(grown, origin_x=.01),  # shifted canvas loses old coverage
                        replace(grown, map_to_odom_x=.3),
                        replace(grown, map_to_odom_yaw=.2),
                        replace(self.context, origin_x=-.025)):
            memory, _ = self.stored_structure()
            self.assertFalse(memory.check(context, [100]*(context.width*context.height)))
            self.assertEqual(memory.state, 'SUSPENDED')
        memory, _ = self.stored_structure()
        memory.suspend('MAP_OR_TF_UNAVAILABLE')
        self.assertFalse(memory.check(grown, [100]*144))
        self.assertEqual(memory.reason, 'MAP_OR_TF_UNAVAILABLE')
        # Immutable saved maps cannot acquire new geometry under the same ID.
        memory, _ = self.stored_structure()
        memory.context = replace(self.context, epoch='saved-map-'+'a'*64)
        self.assertFalse(memory.check(replace(grown, epoch=memory.context.epoch), [100]*144))

    def test_expansion_never_migrates_legacy_body_segments(self):
        memory, _ = self.stored_structure()
        memory.add(context=self.context, grid=self.grid, center_x=.125, center_y=.125,
                   yaw=0., travel_length=.1, robot_length=.1, robot_width=.1,
                   confirmed_frames=3, measured_at=10., now=10.1)
        self.assertTrue(memory.segments)
        grown = replace(self.context, width=12, height=12, origin_x=-.025, origin_y=-.025)
        self.assertTrue(memory.check(grown, [100]*144))
        self.assertFalse(memory.segments)
        self.assertEqual(len(memory.structures), 1)
        memory = PassageMemoryV2()
        memory.add(context=self.context, grid=self.grid, center_x=.125, center_y=.125,
                   yaw=0., travel_length=.1, robot_length=.1, robot_width=.1,
                   confirmed_frames=3, measured_at=10., now=10.1)
        self.assertFalse(memory.check(grown, [100]*144))

    def test_expansion_save_failure_suspends_runtime_output(self):
        from unittest.mock import patch
        memory, _ = self.stored_structure()
        grown = replace(self.context, width=12, height=12, origin_x=-.025, origin_y=-.025)
        adapter = SimpleNamespace(
            get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=10_200_000_000)),
            memory=memory, grid=SimpleNamespace(data=[100]*144), lookup=lambda stamp: self.tf,
            map_session_id='mapping-test', runtime=True, restored=True, pending=None,
            record_path='/not-written.json', reason='',
            accept_structure=lambda current, now: None,
            publish_mask=lambda current: None, publish_status=lambda *args: None)
        with patch('live_memory_shadow.map_context', return_value=grown), \
                patch.object(memory, 'save', side_effect=OSError('disk unavailable')):
            LiveMemoryShadow.tick(adapter)
        self.assertEqual(memory.state, 'SUSPENDED')
        self.assertEqual(memory.reason, 'RECORD_WRITE_FAILED')
        self.assertFalse(memory.planning_cells(grown, [100]*144))

    def test_expansion_to_no_static_intersection_cannot_clear_free_or_unknown(self):
        memory, _ = self.stored_structure()
        grown = replace(self.context, width=12, height=12, origin_x=-.025, origin_y=-.025)
        self.assertTrue(memory.check(grown, [-1]*144))
        self.assertEqual(memory.state, 'KNOWN_PASSAGE')
        self.assertEqual(len(memory.structures), 1)
        self.assertFalse(memory.planning_cells(grown, [0]*144))
        self.assertEqual(len(memory.structures), 1)
        self.assertEqual(len(memory.planning_cells(grown, [100]*144)), 16)

    def test_content_updates_keep_history_without_new_observations(self):
        memory, _ = self.stored_structure()
        expected = set(memory.planning_cells(self.context, self.grid))
        for value in (0, -1, 100, 0, 100):
            grid = self.grid.copy(); grid[0] = value
            cells = memory.planning_cells(self.context, grid)
            self.assertEqual(cells, expected if value == 100 else expected-{0})
            self.assertEqual(len(memory.structures), 1)
            self.assertTrue(memory.needs_save)
        with tempfile.TemporaryDirectory() as directory:
            path = str(Path(directory)/'empty.json')
            self.assertFalse(memory.planning_cells(self.context, [0]*100))
            memory.save(path)
            restored = PassageMemoryV2.restore(path, self.context, [0]*100)
            self.assertEqual(len(restored.structures), 1)
            self.assertEqual(restored.planning_cells(self.context, self.grid), expected)

    def test_saved_map_patch_change_still_revokes_structure(self):
        memory, _ = self.stored_structure()
        saved = replace(self.context, epoch='saved-map-'+'a'*64)
        memory.context = saved
        changed = self.grid.copy(); changed[0] = 0
        self.assertFalse(memory.planning_cells(saved, changed))
        self.assertEqual(memory.state, 'SUSPENDED')



def run_ros_intake(tilted=False):
    """Exercise the real runtime subscription, timer, record and mask publication."""
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import QoSProfile, DurabilityPolicy
    from nav_msgs.msg import OccupancyGrid
    from tf2_ros import StaticTransformBroadcaster
    domain = '232' if tilted else '231'
    expected_cells = 16
    if os.environ.get('ROS_DOMAIN_ID') != domain or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
        raise RuntimeError(f'Require isolated local domain {domain}')
    with tempfile.TemporaryDirectory() as directory:
        record = str(Path(directory)/'runtime.json')
        rclpy.init(args=['--ros-args', '-r', '__ns:=/passage',
                        '-p', 'map_session_id:=mapping-test', '-p', f'record_path:={record}',
                        '-p', 'continuous_structure_memory:=false'])
        memory = LiveMemoryShadow(runtime=True)
        peer = rclpy.create_node('structure_intake_test')
        executor = SingleThreadedExecutor()
        executor.add_node(memory)
        executor.add_node(peer)
        try:
            assert memory.permit_pub is None
            assert not any('/terrain/clearance/' in sub.topic_name for sub in memory.subscriptions)
            map_pub = peer.create_publisher(OccupancyGrid, '/map',
                QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
            structure_pub = peer.create_publisher(type(cloud()), '/terrain/passage/structure', 1)
            masks = []
            peer.create_subscription(OccupancyGrid, '/passage/verified_static_clearing',
                                     masks.append, 10)
            grid = OccupancyGrid()
            grid.header.frame_id = 'map'
            grid.info.width = grid.info.height = 10
            grid.info.resolution = .05
            grid.info.origin.orientation.w = 1.
            grid.data = [100]*100
            transform = TransformStamped()
            transform.header.frame_id = 'map'
            transform.child_frame_id = 'odom'
            transform.transform.rotation.w = 1.
            if tilted:
                half_angle = math.radians(12.66)/2
                transform.transform.rotation.x = math.sin(half_angle)
                transform.transform.rotation.w = math.cos(half_angle)
            broadcaster = StaticTransformBroadcaster(peer)
            transform.header.stamp = peer.get_clock().now().to_msg()
            broadcaster.sendTransform(transform)
            grid.header.stamp = peer.get_clock().now().to_msg()
            map_pub.publish(grid)
            deadline = time.monotonic()+8
            while time.monotonic() < deadline:
                msg = cloud()
                msg.header.stamp = peer.get_clock().now().to_msg()
                structure_pub.publish(msg)
                executor.spin_once(timeout_sec=.02)
                if masks and sum(v == 100 for v in masks[-1].data) == expected_cells:
                    break
            else:
                raise AssertionError(f'No structure mask: {memory.reason}, {memory.memory.reason}')
            assert Path(record).exists()
            assert not memory.memory.segments and len(memory.memory.structures) == 1
            assert memory.last_entry_observation_reason == 'NO_PROOF'
            # Expansion while the observed tunnel is no longer producing messages.
            # This must update both the mask and disk record without fresh proof/structure.
            grown = OccupancyGrid()
            grown.header.frame_id = 'map'
            grown.header.stamp = peer.get_clock().now().to_msg()
            grown.info.width = grown.info.height = 12
            grown.info.resolution = .05
            grown.info.origin.position.x = grown.info.origin.position.y = -.025
            grown.info.origin.orientation.w = 1.
            grown.data = [100]*144
            old_geometry = memory.memory.context.geometry()
            map_pub.publish(grown)
            deadline = time.monotonic()+4
            while time.monotonic() < deadline:
                executor.spin_once(timeout_sec=.02)
                wrapper = json.loads(Path(record).read_text())
                if (masks and masks[-1].info.width == 12 and
                        sum(v == 100 for v in masks[-1].data) == expected_cells and
                        wrapper['body']['context']['width'] == 12):
                    break
            else:
                raise AssertionError('No persisted out-of-view expansion mask: '+memory.reason)
            assert memory.memory.context.geometry() != old_geometry
            assert memory.pending_structure is None
            assert len(memory.memory.structures) == 1 and not memory.memory.segments
            # No more structure messages: normal SLAM cell changes must not erase history.
            tracked = next(i for i, v in enumerate(masks[-1].data) if v == 100)
            for value in (0, -1, 100):
                changed = list(grown.data); changed[tracked] = value; grown.data = changed
                grown.header.stamp = peer.get_clock().now().to_msg(); map_pub.publish(grown)
                deadline = time.monotonic()+2
                target_count = expected_cells if value == 100 else expected_cells-1
                while time.monotonic() < deadline:
                    executor.spin_once(timeout_sec=.02)
                    saved = json.loads(Path(record).read_text())
                    count = len(saved['body']['structures'][0]['cells'])
                    if masks and sum(v == 100 for v in masks[-1].data) == target_count and count == target_count:
                        break
                else:
                    raise AssertionError('SLAM update lost persisted structure: '+memory.reason)
                assert len(memory.memory.structures) == 1 and memory.pending_structure is None

            print(f'PASS: runtime roll={12.66 if tilted else 0} degrees, '
                  f'{expected_cells} static correction cells before/after canvas expansion; '
                  'no clearance message, probe or fresh permit')
        finally:
            executor.shutdown()
            memory.destroy_node()
            peer.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    if '--ros' in sys.argv:
        run_ros_intake('--tilted' in sys.argv)
    else:
        unittest.main()
