"""Stable content identity for paired static-map and correction-mask messages."""
import hashlib
import json
from math import isfinite


def grid_fingerprint(grid):
    info = grid.info
    floats = (info.resolution, info.origin.position.x, info.origin.position.y,
              info.origin.position.z, info.origin.orientation.x,
              info.origin.orientation.y, info.origin.orientation.z,
              info.origin.orientation.w)
    if (grid.header.frame_id != 'map' or info.width <= 0 or info.height <= 0 or
            info.width*info.height != len(grid.data) or
            not all(isfinite(v) for v in floats) or info.resolution <= 0 or
            any(type(value) is not int or not -1 <= value <= 100
                for value in grid.data)):
        raise ValueError('INVALID_GRID_FOR_FINGERPRINT')
    meta = (grid.header.frame_id, info.width, info.height, *floats,
            info.map_load_time.sec, info.map_load_time.nanosec)
    digest = hashlib.sha256(json.dumps(meta, separators=(',', ':'),
                                       allow_nan=False).encode())
    digest.update(bytes(value+1 for value in grid.data))
    return digest.hexdigest()
