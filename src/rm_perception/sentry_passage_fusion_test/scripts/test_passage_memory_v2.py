#!/usr/bin/env python3
"""Multi-segment history must survive distance/restart but fail closed on changes."""
from dataclasses import replace
from pathlib import Path
from tempfile import TemporaryDirectory

from global_passage_memory import MapContext
from passage_memory_v2 import PassageMemoryV2
from passage_record_handover import export_record


def run():
    context = MapContext('mapping-test-session', 'map', 80, 80, .1, -4., -4., 0., 0., 0.)
    grid = [0] * 6400
    for y in range(37, 43):
        for x in range(43, 68):
            grid[y*80+x] = 100
    memory = PassageMemoryV2()
    common = dict(context=context, grid=grid, center_y=0., yaw=0.,
                  travel_length=.2, robot_length=.45, robot_width=.45,
                  confirmed_frames=3, measured_at=9.9, now=10.)
    first = memory.add(center_x=1., **common)
    second = memory.add(center_x=2., **common)
    assert first != second and len(memory.segments) == 2
    assert memory.add(center_x=1., **common) == first
    cells = memory.planning_cells(context, grid)
    assert cells and all(grid[i] == 100 for i in cells)
    assert all(i in cells for i in memory.segments[first].cells)
    # The robot has moved far outside the sensor's local window. History remains.
    assert memory.check(context, grid) and len(memory.segments) == 2

    # Two physically different sweeps can have identical raster cells. Their
    # permits must not share an ID; vertical geometry also travels with history.
    precise = PassageMemoryV2()
    one = precise.add(center_x=1., ground_z=0., robot_height=.7, **common)
    shifted = precise.add(center_x=1.001, ground_z=0., robot_height=.7, **common)
    assert precise.segments[one].cells == precise.segments[shifted].cells
    assert one != shifted
    low = precise.add(center_x=1., ground_z=0., robot_height=.5, **common)
    assert low != one

    with TemporaryDirectory(prefix='multi-passage-') as directory:
        file = Path(directory) / 'segments.json'
        memory.save(file)
        restored = PassageMemoryV2.restore(file, context, grid)
        assert restored.state == 'KNOWN_PASSAGE' and len(restored.segments) == 2
        assert restored.planning_cells(context, grid) == cells
        handover = Path(directory) / 'saved-map-record.json'
        target = 'saved-map-' + 'a' * 64
        assert export_record(file, handover, target) == 2
        assert PassageMemoryV2.restore(file, context, grid).state == 'KNOWN_PASSAGE'
        target_context = replace(context, epoch=target)
        migrated = PassageMemoryV2.restore(handover, target_context, grid)
        assert migrated.state == 'KNOWN_PASSAGE' and migrated.planning_cells(target_context, grid) == cells
        assert PassageMemoryV2.restore(handover, replace(target_context, epoch='saved-map-' +
                                      'b' * 64), grid).state == 'SUSPENDED'
        assert PassageMemoryV2.restore(handover, replace(target_context, origin_x=-3.9),
                                       grid).state == 'SUSPENDED'
        # A restarted odom origin is expected for an immutable saved map.
        restarted = replace(target_context, map_to_odom_x=.3)
        rebased = PassageMemoryV2.restore(handover, restarted, grid)
        assert rebased.state == 'KNOWN_PASSAGE'
        assert not rebased.check(replace(restarted, map_to_odom_x=.6), grid)
        assert rebased.state == 'SUSPENDED'
        changed_target = grid.copy()
        changed_target[memory.segments[first].cells[0]] = 0
        assert first not in PassageMemoryV2.restore(handover, target_context,
                                                    changed_target).segments
        with_vertical = Path(directory)/'vertical.json'
        precise.save(with_vertical)
        loaded = PassageMemoryV2.restore(with_vertical, context, grid)
        assert loaded.segments[one].ground_z == 0.
        assert loaded.segments[one].robot_height == .7
        assert PassageMemoryV2.restore(file, replace(context, epoch='reloaded'), grid).state == 'SUSPENDED'
        assert PassageMemoryV2.restore(file, replace(context, map_to_odom_x=.3), grid).state == 'SUSPENDED'
        outside = grid.copy()
        outside[0] = 100
        assert PassageMemoryV2.restore(file, context, outside).state == 'KNOWN_PASSAGE'
        changed = grid.copy()
        changed[memory.segments[first].cells[0]] = 0
        partial = PassageMemoryV2.restore(file, context, changed)
        assert first not in partial.segments and second in partial.segments
        assert partial.state == 'KNOWN_PASSAGE'
        file.write_text('invalid', encoding='utf-8')
        assert PassageMemoryV2.restore(file, context, grid).state == 'SUSPENDED'
    print('Multi-segment memory: away, restart, localized invalidation, map/TF fail-close passed')


if __name__ == '__main__':
    run()
