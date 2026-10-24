#!/usr/bin/env python3
"""Independent analytic footprints and live context checks for tilted TF."""
from dataclasses import replace
import math
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import OccupancyGrid

from global_passage_memory import MapContext
from live_memory_shadow import LiveMemoryShadow, association_stable, map_context
from passage_memory_v2 import PassageMemoryV2
from rigid_transform import normalize_transform
from structure_observation import structure_map_cells
from test_structure_observation import cloud


def rotation(axis, angle, translation=(0., 0., 0.)):
    q = [0., 0., 0., math.cos(angle/2)]
    q[axis] = math.sin(angle/2)
    return (*translation, *q)


def observation(lower=.2, count=8):
    return dict(frame='odom', measured_at=10., resolution=.05,
                columns=[(.025+x*.05, .025+y*.05, lower, lower+.4)
                         for x in range(count) for y in range(count)])


class StructureTFTests(unittest.TestCase):
    def setUp(self):
        self.context = MapContext('mapping-test', 'map', 20, 20, .05, 0., 0., 0., 0., 0.)

    def test_roll_uses_height_and_projects_actual_footprint(self):
        transform = rotation(0, math.pi/6, (.1, .3, .5))
        cells = set(structure_map_cells(observation(), self.context, transform))
        # x = .1 + source_x; y = .3 + cos(30)*source_y - sin(30)*.2.
        expected = {y*20+x for x in range(2, 10) for y in range(4, 11)}
        self.assertEqual(cells, expected)
        at_zero = set(structure_map_cells(observation(0.), self.context, transform))
        self.assertEqual(at_zero, {cell+40 for cell in expected})

    def test_pitch_uses_height(self):
        transform = rotation(1, math.pi/6, (.1, .1, .5))
        # x = .1 + cos(30)*source_x + sin(30)*.2; y = .1 + source_y.
        self.assertEqual(set(structure_map_cells(observation(), self.context, transform)),
                         {y*20+x for x in range(4, 11) for y in range(2, 10)})

    def test_yaw_maps_observed_centres_to_target_grid(self):
        angle = math.pi/4
        transform = rotation(2, angle, (.5, .1, 0.))
        expected = set()
        # Independently transform each observed centre; absent columns add no cells.
        for y in range(8):
            for x in range(8):
                sx, sy = (x+.5)*.05, (y+.5)*.05
                mx = math.floor((.5+math.cos(angle)*sx-math.sin(angle)*sy)/.05+1e-9)
                my = math.floor((.1+math.sin(angle)*sx+math.cos(angle)*sy)/.05+1e-9)
                expected.add(my*20+mx)
        self.assertGreater(len(expected), 20)
        self.assertEqual(set(structure_map_cells(observation(), self.context, transform)), expected)

    def test_overlapping_projected_centres_are_not_duplicated(self):
        angle = math.pi/3
        obs = dict(frame='odom', measured_at=10., resolution=.05,
                   columns=[(.025, .025, 0., .4),
                            (.025, .075, .05/math.tan(angle), .4)])
        # Both observed centres land in one target cell, without dilation.
        self.assertEqual(structure_map_cells(obs, self.context, rotation(0, angle)), (0,))

    def test_combined_rotation_matches_independent_axis_angle_footprint(self):
        axis = [v/math.sqrt(14) for v in (1., 2., 3.)]
        angle = .4
        c, s = math.cos(angle), math.sin(angle)
        # Rodrigues' axis-angle construction is independent of quaternion_matrix.
        cross = ((0., -axis[2], axis[1]), (axis[2], 0., -axis[0]),
                 (-axis[1], axis[0], 0.))
        matrix = [[c*(i == j)+(1-c)*axis[i]*axis[j]+s*cross[i][j]
                   for j in range(3)] for i in range(3)]
        tx, ty, lower = .2, .1, .2
        bx, by = tx+matrix[0][2]*lower, ty+matrix[1][2]*lower
        expected = set()
        for y in range(8):
            for x in range(8):
                sx, sy = (x+.5)*.05, (y+.5)*.05
                mx = math.floor((bx+matrix[0][0]*sx+matrix[0][1]*sy)/.05+1e-9)
                my = math.floor((by+matrix[1][0]*sx+matrix[1][1]*sy)/.05+1e-9)
                expected.add(my*20+mx)
        transform = (tx, ty, .5, *(v*math.sin(angle/2) for v in axis), math.cos(angle/2))
        self.assertEqual(set(structure_map_cells(observation(), self.context, transform)), expected)

    def test_tilted_context_accepted_only_for_structure_runtime(self):
        grid = OccupancyGrid()
        grid.header.frame_id = 'map'
        grid.info.width = grid.info.height = 20
        grid.info.resolution = .05
        grid.info.origin.orientation.w = 1.
        grid.data = [100]*400
        tf = TransformStamped()
        tf.transform.translation.x, tf.transform.translation.y = .1, .3
        tf.transform.rotation.x = math.sin(math.pi/12)
        tf.transform.rotation.w = math.cos(math.pi/12)
        context = map_context(grid, tf, 'mapping-test', True)
        self.assertTrue(context.valid())
        self.assertTrue(association_stable(context, replace(context,
            map_to_odom_quaternion=tuple(-v for v in context.map_to_odom_quaternion))))
        self.assertFalse(association_stable(context, replace(context, map_to_odom_z=.3)))
        changed = replace(context, map_to_odom_quaternion=rotation(0, math.pi/3)[3:])
        self.assertFalse(association_stable(context, changed))
        with self.assertRaises(ValueError):
            map_context(grid, tf, 'mapping-test')  # Legacy planar diagnostic contract.
        grid.info.origin.orientation.x = math.sin(math.pi/12)
        grid.info.origin.orientation.w = math.cos(math.pi/12)
        with self.assertRaises(ValueError):
            map_context(grid, tf, 'mapping-test', True)  # Map origin must still be planar.

    def test_runtime_uses_observation_time_and_persists_full_pose(self):
        grid = OccupancyGrid()
        grid.header.frame_id = 'map'
        grid.info.width = grid.info.height = 20
        grid.info.resolution = .05
        grid.info.origin.orientation.w = 1.
        grid.data = [100]*400
        tf = TransformStamped()
        tf.transform.translation.x, tf.transform.translation.y = .1, .3
        tf.transform.rotation.x = math.sin(math.pi/12)
        tf.transform.rotation.w = math.cos(math.pi/12)
        context = map_context(grid, tf, 'mapping-test', True)
        calls = []
        def lookup(stamp):
            calls.append(stamp)
            return tf
        with tempfile.TemporaryDirectory() as directory:
            adapter = SimpleNamespace(pending_structure=cloud(
                [(*row, .05) for row in observation()['columns']]),
                last_structure_stamp=0., last_invalidation_time=0., expected_frame='odom',
                map_session_id='mapping-test', record_path=str(Path(directory)/'memory.json'),
                memory=PassageMemoryV2(), grid=grid, lookup=lookup)
            LiveMemoryShadow.accept_structure(adapter, context, 10.1)
            self.assertEqual(calls, [10.])
            self.assertEqual(adapter.memory.state, 'KNOWN_PASSAGE', adapter.reason)
            record = next(iter(adapter.memory.structures.values()))
            self.assertEqual(len(record.map_transform), 7)
            self.assertAlmostEqual(record.map_transform[3], math.sin(math.pi/12))
            restored = PassageMemoryV2.restore(adapter.record_path, context, list(grid.data))
            self.assertEqual(restored.planning_cells(context, list(grid.data)), set(record.cells))
            changed = replace(context, map_to_odom_quaternion=rotation(0, math.pi/3)[3:])
            self.assertFalse(restored.planning_cells(changed, list(grid.data)))
            self.assertEqual(restored.reason, 'MAP_ODOM_ASSOCIATION_MOVED')

    def test_single_column_survives_subcell_grid_offset(self):
        obs = dict(frame='odom', measured_at=10., resolution=.05,
                   columns=[(.025, .025, .2, .6)])
        self.assertEqual(structure_map_cells(obs, self.context, (.012, .013, 0., 0.)), (0,))

    def test_invalid_quaternion_rejected_and_legacy_transform_supported(self):
        for values in ((0., 0., 0., 0., 0., 0., 0.), (0., 0., 0., 0., 0., 0., math.nan)):
            with self.assertRaises(ValueError):
                normalize_transform(values)
        legacy = (0., 0., 0., .2)
        self.assertEqual(structure_map_cells(observation(), self.context, legacy),
                         structure_map_cells(observation(), self.context, normalize_transform(legacy)))


if __name__ == '__main__':
    unittest.main()
