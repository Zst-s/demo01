#!/usr/bin/env python3
"""A map update or mask update must not reuse the previous catalog identity."""
from copy import deepcopy
from types import SimpleNamespace

from nav_msgs.msg import OccupancyGrid
from grid_fingerprint import grid_fingerprint
from isolated_entry_gate import IsolatedEntryGate


def run():
    grid = OccupancyGrid()
    grid.header.frame_id = 'map'
    grid.info.width = grid.info.height = 2
    grid.info.resolution = .1
    grid.info.origin.orientation.w = 1.
    grid.info.map_load_time.sec = 10
    grid.data = [0, 100, -1, 0]
    original = grid_fingerprint(grid)
    copy = deepcopy(grid)
    assert grid_fingerprint(copy) == original
    copy.data[1] = 0
    assert grid_fingerprint(copy) != original
    copy = deepcopy(grid)
    copy.info.map_load_time.sec = 11
    assert grid_fingerprint(copy) != original
    copy = deepcopy(grid)
    copy.info.origin.position.x = .1
    assert grid_fingerprint(copy) != original
    copy = deepcopy(grid)
    copy.data[2] = 101
    try:
        grid_fingerprint(copy)
    except ValueError:
        pass
    else:
        raise AssertionError('Invalid map data was accepted')
    mask = deepcopy(grid)
    mask.data = [0, 100, 0, 0]
    gate = SimpleNamespace(static_map=grid, correction_mask=mask,
                           mask_received=10.,
                           catalog=('KNOWN_PASSAGE', [], True,
                                    grid_fingerprint(grid), grid_fingerprint(mask)))
    assert IsolatedEntryGate.valid_mask(gate, 10.1)
    changed_map = deepcopy(grid)
    changed_map.data[0] = 100
    gate.static_map = changed_map
    assert not IsolatedEntryGate.valid_mask(gate, 10.1)
    gate.static_map = grid
    changed_mask = deepcopy(mask)
    changed_mask.data[0] = 100
    gate.correction_mask = changed_mask
    assert not IsolatedEntryGate.valid_mask(gate, 10.1)
    gate.correction_mask = mask
    assert not IsolatedEntryGate.valid_mask(gate, 10.6)
    gate.static_hash = original
    IsolatedEntryGate.on_map(gate, deepcopy(grid))
    assert gate.correction_mask is mask
    IsolatedEntryGate.on_map(gate, changed_map)
    assert gate.correction_mask is None
    print('PASS: static map, geometry and correction-mask identity is content-bound')


if __name__ == '__main__':
    run()
