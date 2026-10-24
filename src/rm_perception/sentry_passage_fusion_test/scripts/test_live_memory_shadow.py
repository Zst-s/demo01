#!/usr/bin/env python3
"""Replay one actual ROGMap short-segment message through the map adapter."""
import json
from dataclasses import replace
from pathlib import Path
from tempfile import TemporaryDirectory

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import OccupancyGrid

from global_passage_memory import ConfirmedPassage, GlobalPassageMemory
from live_memory_shadow import (association_stable, entry_permit_window,
                                entry_edge_status, entry_recency_reason,
                                map_context, parse_observation,
                                passage_bbox, patch_hash)


def main():
    # Captured from the headless lidar -> ROGMap multiview run at the entrance;
    # this is a short-segment observation, not a whole-tunnel permission.
    raw = json.dumps({
        'schema': 1, 'kind': 'verified_short_segment_observation', 'frame': 'world',
        'probe_x': 1.7, 'probe_y': 0, 'ground_z': -.025, 'yaw': 0,
        'travel_length': .2, 'robot_length': .45, 'robot_width': .45,
        'robot_height': .7, 'resolution': .1, 'revision': 1,
        'confirmed_frames': 3, 'measured_at': 27.9,
        'oldest_observation': 20, 'newest_observation': 27.9,
        'oldest_body_observation': 20, 'newest_body_observation': 27.9,
        'oldest_support_observation': 20, 'newest_support_observation': 27.9,
        'oldest_entry_observation': 27.7, 'newest_entry_observation': 27.9,
        'recent_entry_edge_voxels': 128, 'entry_edge_voxels': 128,
        'recent_body_voxels': 96, 'body_voxels': 384,
        'entry_recent_window': 2.0,
        'recent_body_by_slice': [[0, 48], [0, 48], [0, 48], [0, 48],
                                 [0, 48], [0, 48], [48, 48], [48, 48]],
        'recent_support_by_slice': [[0, 32], [0, 32], [0, 32], [32, 32]],
        'valid_until': 28.9, 'checked_voxels': 432})
    grid = OccupancyGrid()
    grid.header.frame_id = 'map'
    grid.info.width = grid.info.height = 80
    grid.info.resolution = .1
    grid.info.origin.position.x = grid.info.origin.position.y = -4.
    grid.info.origin.orientation.w = 1.
    grid.data = [0] * 6400
    transform = TransformStamped()
    transform.header.frame_id = 'map'
    transform.child_frame_id = 'world'
    transform.transform.rotation.w = 1.
    context = map_context(grid, transform, 'verified-map-session')
    proof = parse_observation(raw, 28., 'world')
    virtual_raw = json.dumps(dict(proof, kind='virtual_short_segment_preview'))
    assert parse_observation(virtual_raw, 28., 'world',
                             'virtual_short_segment_preview') == json.loads(virtual_raw)
    try:
        parse_observation(virtual_raw, 28., 'world')
        raise AssertionError('Virtual-body proof was accepted as real evidence')
    except ValueError:
        pass
    # Old interior cells remain useful as structural memory, but only a fully
    # fresh leading body slice plus its measured floor can authorize the next
    # short move. This reflects the moving, occluded sensor in the live test.
    assert entry_edge_status(proof) == 'ENTRY_EDGE_CURRENTLY_OBSERVED'
    assert entry_permit_window(proof, 28., 2.0) == 28.4
    assert entry_permit_window(proof, 28.5, 2.0) is None
    stale_front_body = dict(proof, recent_body_by_slice=proof['recent_body_by_slice'][:-2] +
                            [[47, 48], [48, 48]], recent_body_voxels=95,
                            recent_entry_edge_voxels=127)
    parsed = parse_observation(json.dumps(stale_front_body), 28., 'world')
    assert entry_permit_window(parsed, 28., 2.0) is None
    assert entry_recency_reason(parsed, 28., 2.0) == (
        'ENTRY_EDGE_BODY_NOT_RECENTLY_OBSERVED')
    stale_front_ground = dict(proof, recent_support_by_slice=[
        [0, 32], [0, 32], [0, 32], [31, 32]], recent_entry_edge_voxels=127)
    parsed = parse_observation(json.dumps(stale_front_ground), 28., 'world')
    assert entry_permit_window(parsed, 28., 2.0) is None
    assert entry_recency_reason(parsed, 28., 2.0) == (
        'ENTRY_EDGE_GROUND_SUPPORT_NOT_RECENTLY_OBSERVED')
    legacy_without_support_timing = dict(proof)
    legacy_without_support_timing.pop('newest_support_observation')
    legacy_without_support_timing.pop('recent_support_by_slice')
    legacy_partial = parse_observation(
        json.dumps(legacy_without_support_timing), 28., 'world')
    assert entry_permit_window(legacy_partial, 28., 2.0) is None
    no_support_timing = dict(proof)
    no_support_timing.pop('oldest_support_observation')
    no_support_timing.pop('newest_support_observation')
    no_support_timing.pop('recent_support_by_slice')
    legacy = parse_observation(json.dumps(no_support_timing), 28., 'world')
    assert entry_permit_window(legacy, 28., 2.0) is None
    upper_blind = dict(proof, recent_body_voxels=48, body_voxels=384,
                       entry_recent_window=2.0,
                       recent_entry_edge_voxels=80,
                       recent_body_by_slice=[[0, 48], [0, 48]]*3 +
                                             [[48, 48], [0, 48]])
    upper_blind = parse_observation(json.dumps(upper_blind), 28., 'world')
    assert entry_permit_window(upper_blind, 28., 2.0) is None
    assert entry_recency_reason(upper_blind, 28., 2.0) == 'ENTRY_EDGE_BODY_NOT_RECENTLY_OBSERVED'
    malformed_coverage = dict(upper_blind, recent_body_voxels=193)
    try:
        parse_observation(json.dumps(malformed_coverage), 28., 'world')
        raise AssertionError('Inconsistent freshness coverage was accepted')
    except ValueError:
        pass
    invalid_body = dict(proof, oldest_body_observation=19.0)
    try:
        parse_observation(json.dumps(invalid_body), 28., 'world')
        raise AssertionError('Inconsistent body timestamps were accepted')
    except ValueError:
        pass
    bounds = passage_bbox(proof, transform, context)
    assert bounds == (53, 61, 37, 43), bounds
    assert not association_stable(context, replace(context, map_to_odom_x=.3))
    memory = GlobalPassageMemory()
    unbound = ConfirmedPassage(*bounds, 'map', 3, 27.9, 'real-rogmap-proof')
    segment = replace(unbound, map_patch_hash=patch_hash(grid, unbound))
    assert memory.observe(segment, context, 28.)
    with TemporaryDirectory(prefix='live-passage-') as directory:
        file = Path(directory) / 'memory.json'
        memory.save(file)
        reloaded = GlobalPassageMemory.restore(file, context)
        assert reloaded.state == 'KNOWN_PASSAGE'
        assert reloaded.passage.map_patch_hash == patch_hash(grid, reloaded.passage)
        grid.data[40 * grid.info.width + 55] = 100
        assert reloaded.passage.map_patch_hash != patch_hash(grid, reloaded.passage)
        assert GlobalPassageMemory.restore(file, replace(context, epoch='other')).state == 'SUSPENDED'
    try:
        parse_observation(raw, 29., 'world')
        raise AssertionError('expired proof was accepted')
    except ValueError:
        pass
    print('PASS: captured ROGMap proof -> map candidate -> disk -> fail-closed reload')


if __name__ == '__main__':
    main()
