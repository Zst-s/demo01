#!/usr/bin/env python3
"""Focused state/association tests, independent of ROS discovery."""
from dataclasses import replace
from pathlib import Path
from tempfile import TemporaryDirectory
from global_passage_memory import MapContext, ConfirmedPassage, GlobalPassageMemory


def main():
    c = MapContext('session-A', 'map', 80, 80, .1, -4., -4., 0., 0., 0.)
    p = ConfirmedPassage(38, 42, 30, 50, 'map', 3, 10., 'proof-A')
    m = GlobalPassageMemory()
    assert not m.observe(replace(p, confirmed_frames=2), c, 10.1)
    assert not m.planning_cells(c)
    assert not m.observe(replace(p, measured_at=6.), c, 10.1)
    assert m.observe(p, c, 10.1)
    assert len(m.planning_cells(c)) == 80
    # Local observation disappears: the structure remains available to planning.
    assert len(m.planning_cells(c)) == 80
    m.obstacle_conflict()
    assert not m.planning_cells(c)
    assert not m.observe(p, c, 10.2), 'Consumed evidence must not revive record'
    assert m.observe(replace(p, evidence_id='proof-B', measured_at=10.3), c, 10.4)
    assert not m.planning_cells(replace(c, map_to_odom_x=.3))
    assert m.reason == 'MAP_ODOM_CORRECTION_UNASSOCIATED'
    assert not m.planning_cells(c), 'Correction must not revive old record'
    assert m.observe(replace(p, evidence_id='proof-C', measured_at=10.5), c, 10.6)
    assert not m.planning_cells(replace(c, epoch='session-B'))
    assert m.reason == 'MAP_RELOADED_OR_GEOMETRY_CHANGED'
    assert m.observe(replace(p, evidence_id='proof-D', measured_at=10.7), c, 10.8)
    assert not m.planning_cells(replace(c, origin_x=-3.5))
    assert m.observe(replace(p, evidence_id='proof-E', measured_at=10.9), c, 11.)
    assert not m.planning_cells(replace(c, map_to_odom_yaw=.2))
    # 真正的磁盘往返：进程重启后仅恢复历史结构；地图不匹配/损坏一律停用。
    assert m.observe(replace(p, evidence_id='proof-F', measured_at=11.1), c, 11.2)
    with TemporaryDirectory(prefix='passage-memory-') as directory:
        record = Path(directory) / 'passage.json'
        m.save(record)
        restored = GlobalPassageMemory.restore(record, c)
        assert restored.state == 'KNOWN_PASSAGE'
        assert len(restored.planning_cells(c)) == 80
        assert not restored.observe(replace(p, evidence_id='proof-F'), c, 11.3)
        assert not GlobalPassageMemory.restore(record, replace(c, epoch='other-map')).planning_cells(c)
        assert not GlobalPassageMemory.restore(record, replace(c, map_to_odom_x=.5)).planning_cells(c)
        record.write_text('{"body":{},"sha256":"wrong"}', encoding='utf-8')
        assert not GlobalPassageMemory.restore(record, c).planning_cells(c)
    print('PASS: historical candidate retained outside local view; conflict/reload/pose correction fail closed')


if __name__ == '__main__':
    main()
