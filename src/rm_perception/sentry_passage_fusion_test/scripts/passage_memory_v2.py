"""Classified hollow structures and legacy short segments for one SLAM session.

Historical records are candidates for global planning only. They never grant
entry permission. A mapping-canvas expansion reprojects saved structures; a
map/TF association change suspends output. Within one mapping session, occupancy updates reproject measured structures
without discarding their geometry. Saved-map patch changes still invalidate
the affected record. No unknown map cell is cleared.
"""
from dataclasses import asdict, dataclass, replace
from math import cos, floor, isfinite, sin
import hashlib
import json
import os

from passage_record_io import atomic_write_record
from global_passage_memory import MapContext
from structure_observation import validate_columns, structure_map_cells
from rigid_transform import context_transform, normalize_transform, transforms_close


@dataclass(frozen=True)
class StoredSegment:
    segment_id: str
    center_x: float
    center_y: float
    yaw: float
    travel_length: float
    robot_length: float
    robot_width: float
    cells: tuple
    map_patch_hash: str
    ground_z: float = None
    robot_height: float = None


@dataclass(frozen=True)
class StoredStructure:
    structure_id: str
    cells: tuple
    map_patch_hash: str
    source_frame: str
    resolution: float
    columns: tuple
    map_transform: tuple


def _cell_hash(grid, cells, *, allow_empty=False):
    if (not cells and not allow_empty) or any(type(i) is not int or i < 0 or i >= len(grid) for i in cells):
        raise ValueError('INVALID_SEGMENT_CELLS')
    values = [grid[i] for i in cells]
    if any(type(v) is not int or v < -1 or v > 100 for v in values):
        raise ValueError('INVALID_MAP_CELL')
    return hashlib.sha256(bytes(v+1 for v in values)).hexdigest()


def raster_swept_cells(context, center_x, center_y, yaw, travel_length,
                       robot_length, robot_width):
    """Only cells fully contained by the observed, oriented swept body volume."""
    values = (center_x, center_y, yaw, travel_length, robot_length, robot_width)
    if (not context.valid() or not all(isfinite(v) for v in values) or
            min(travel_length, robot_length, robot_width) <= 0):
        raise ValueError('INVALID_SWEEP')
    c, s = cos(yaw), sin(yaw)
    half_u = (travel_length+robot_length)/2
    half_v = robot_width/2
    span_x = abs(c)*half_u+abs(s)*half_v
    span_y = abs(s)*half_u+abs(c)*half_v
    x0 = max(0, floor((center_x-span_x-context.origin_x)/context.resolution))
    x1 = min(context.width, floor((center_x+span_x-context.origin_x)/context.resolution)+1)
    y0 = max(0, floor((center_y-span_y-context.origin_y)/context.resolution))
    y1 = min(context.height, floor((center_y+span_y-context.origin_y)/context.resolution)+1)
    cells = []
    half_cell = context.resolution/2
    # Every corner of a chosen map cell must lie inside the observed volume.
    # This deliberately under-approximates a corridor, never enlarges it.
    for y in range(y0, y1):
        py = context.origin_y+(y+.5)*context.resolution-center_y
        for x in range(x0, x1):
            px = context.origin_x+(x+.5)*context.resolution-center_x
            if all(abs(c*(px+dx)+s*(py+dy)) <= half_u+1e-9 and
                   abs(-s*(px+dx)+c*(py+dy)) <= half_v+1e-9
                   for dx in (-half_cell, half_cell)
                   for dy in (-half_cell, half_cell)):
                cells.append(y*context.width+x)
    if not cells:
        raise ValueError('NO_FULLY_VERIFIED_MAP_CELLS')
    return tuple(cells)


def _is_mapping_expansion(previous, current, *, max_edge_retraction=0.0):
    """A larger canvas in the same mapping session, not a reload or sliding map."""
    if (not previous.epoch.startswith('mapping-') or
            current.epoch != previous.epoch or current.frame != previous.frame or
            current.resolution != previous.resolution or
            current.width < previous.width or current.height < previous.height or
            (current.width == previous.width and current.height == previous.height)):
        return False
    if not 0 <= max_edge_retraction <= current.resolution:
        return False
    epsilon = current.resolution * 1e-6 + max_edge_retraction
    return (current.origin_x <= previous.origin_x + epsilon and
            current.origin_y <= previous.origin_y + epsilon and
            current.origin_x + current.width*current.resolution >=
                previous.origin_x + previous.width*previous.resolution - epsilon and
            current.origin_y + current.height*current.resolution >=
                previous.origin_y + previous.height*previous.resolution - epsilon)


class PassageMemoryV2:
    def __init__(self, max_records=128, max_pose_shift=.25, max_yaw_shift=.15):
        self.max_records = max_records
        self.max_pose_shift = max_pose_shift
        self.max_yaw_shift = max_yaw_shift
        self.context = None
        self.segments = {}
        self.structures = {}
        self.state = 'UNKNOWN'
        self.reason = 'NO_VERIFIED_SEGMENTS'
        self.needs_save = False
        self._structure_grid = None

    def suspend(self, reason):
        self.state, self.reason = 'SUSPENDED', reason

    def check(self, context, grid):
        if (not context.valid() or len(grid) != context.width*context.height or
                self.context is None):
            self.suspend('MAP_UNAVAILABLE')
            return False
        previous = self.context
        if context.epoch != previous.epoch:
            self.suspend('MAP_RELOAD_OR_GEOMETRY_CHANGED')
            return False
        if not transforms_close(context_transform(context), context_transform(previous),
                                self.max_pose_shift, self.max_yaw_shift):
            self.suspend('MAP_ODOM_ASSOCIATION_MOVED')
            return False
        if self.state == 'SUSPENDED':
            return False
        if context.geometry() != previous.geometry():
            if not self.structures or not _is_mapping_expansion(previous, context):
                self.suspend('MAP_RELOAD_OR_GEOMETRY_CHANGED')
                return False
            return self._reproject_structures(context, grid)
        invalid = [key for key, segment in self.segments.items()
                   if _cell_hash(grid, segment.cells) != segment.map_patch_hash]
        for key in invalid:
            del self.segments[key]
        # SLAM occupancy updates do not revoke a measured hollow structure.
        # Recompute only its current static-wall intersection; keep its 3D history,
        # including when that intersection is temporarily empty.
        if (self.structures and context.epoch.startswith('mapping-') and
                tuple(grid) != self._structure_grid):
            return self._reproject_structures(context, grid, expansion=False)
        invalid_structures = [key for key, structure in self.structures.items()
                              if _cell_hash(grid, structure.cells, allow_empty=True) != structure.map_patch_hash]
        for key in invalid_structures:
            del self.structures[key]
        invalid += invalid_structures
        if invalid:
            self.reason = 'MAP_PATCH_CHANGED_SEGMENT_REMOVED'
        if not self.segments and not self.structures:
            if invalid:
                self.suspend('MAP_PATCH_CHANGED_ALL_SEGMENTS_REMOVED')
            else:
                self.state = 'UNKNOWN'
            return False
        self.state = 'KNOWN_PASSAGE'
        return True

    def _reproject_structures(self, context, grid, expansion=True):
        # The original map transform anchors each measured structure in map.
        # Never follow the latest TF or shift old linear indices into a new canvas.
        projected = {}
        try:
            for key, structure in self.structures.items():
                observation = dict(columns=structure.columns, resolution=structure.resolution)
                cells = tuple(i for i in structure_map_cells(
                    observation, context, structure.map_transform) if grid[i] == 100)
                projected[key] = replace(structure, cells=cells,
                    map_patch_hash=_cell_hash(grid, cells, allow_empty=True))
        except (ValueError, TypeError, OverflowError):
            self.suspend('INVALID_STRUCTURE_REPROJECTION')
            return False
        # Commit only a complete reprojection; legacy body proofs cannot migrate.
        self.structures = projected
        if expansion:
            self.segments.clear()
            self.context = context
        self._structure_grid = tuple(grid)
        self.needs_save = bool(projected)
        self.state = 'KNOWN_PASSAGE' if projected else 'UNKNOWN'
        self.reason = ('MAPPING_EXPANSION_REPROJECTED' if expansion else
                       'MAPPING_CONTENT_REPROJECTED')
        return bool(projected)

    def add(self, *, context, grid, center_x, center_y, yaw, travel_length,
            robot_length, robot_width, confirmed_frames, measured_at, now,
            ground_z=None, robot_height=None):
        if (confirmed_frames < 3 or not isfinite(measured_at) or
                not 0 <= now-measured_at <= 2 or
                len(grid) != context.width*context.height):
            raise ValueError('STALE_OR_INVALID_PROOF')
        if not ((ground_z is None and robot_height is None) or
                (ground_z is not None and robot_height is not None and
                 isfinite(ground_z) and isfinite(robot_height) and robot_height > 0)):
            raise ValueError('INVALID_VERTICAL_GEOMETRY')
        if self.context is not None and self.state == 'KNOWN_PASSAGE' and (
                not self.check(context, grid)):
            raise ValueError(self.reason)
        if self.state == 'SUSPENDED':
            raise ValueError('NEW_MAP_ASSOCIATION_REQUIRED')
        cells = raster_swept_cells(context, center_x, center_y, yaw,
                                   travel_length, robot_length, robot_width)
        # Cell indices alone are not a proof identity: distinct sub-cell poses,
        # headings or clearances can rasterize to the same cells. Never renew a
        # permit for an older, geometrically different sweep via such a collision.
        signature = json.dumps((cells, center_x, center_y, yaw, travel_length,
                                robot_length, robot_width, ground_z, robot_height),
                               separators=(',', ':')).encode()
        segment_id = hashlib.sha256(signature).hexdigest()[:20]
        if segment_id in self.segments:
            return segment_id  # Refreshing a view does not create a new tunnel.
        if len(self.segments)+len(self.structures) >= self.max_records:
            raise ValueError('RECORD_LIMIT_REACHED')
        segment = StoredSegment(segment_id, center_x, center_y, yaw,
                                travel_length, robot_length, robot_width, cells,
                                _cell_hash(grid, cells), ground_z, robot_height)
        self.context = context
        self.segments[segment_id] = segment
        self.state, self.reason = 'KNOWN_PASSAGE', 'HISTORICAL_STRUCTURE_ONLY'
        return segment_id

    def add_structure(self, *, context, grid, observation, cells, map_transform, now):
        """Record classified columns directly, without creating a body proof/permit."""
        validate_columns(observation['columns'], observation['resolution'])
        map_transform = normalize_transform(map_transform)
        if (not context.valid() or len(grid) != context.width*context.height or
                not isinstance(observation['frame'], str) or not observation['frame'] or
                not isfinite(observation['measured_at']) or
                not 0 <= now-observation['measured_at'] <= 2):
            raise ValueError('INVALID_STRUCTURE_OBSERVATION')
        if self.context is not None and self.state == 'KNOWN_PASSAGE' and not self.check(context, grid):
            raise ValueError(self.reason)
        if self.state == 'SUSPENDED':
            raise ValueError('NEW_MAP_ASSOCIATION_REQUIRED')
        _cell_hash(grid, cells)
        known = {cell for record in self.structures.values() for cell in record.cells}
        # This consumer corrects known static walls only. Repeated observations
        # of the same cells do not create an unbounded series of records.
        added = tuple(sorted({cell for cell in cells if grid[cell] == 100} - known))
        if not added:
            return None
        if len(self.segments)+len(self.structures) >= self.max_records:
            raise ValueError('RECORD_LIMIT_REACHED')
        structure_id = hashlib.sha256(json.dumps(added).encode()).hexdigest()[:20]
        self.structures[structure_id] = StoredStructure(
            structure_id, added, _cell_hash(grid, added), observation['frame'],
            observation['resolution'], tuple(tuple(row) for row in observation['columns']),
            tuple(map_transform))
        self.context = context
        self._structure_grid = tuple(grid)
        self.state, self.reason = 'KNOWN_PASSAGE', 'CLASSIFIED_HOLLOW_STRUCTURE'
        return structure_id

    def planning_cells(self, context, grid):
        if not self.check(context, grid):
            return set()
        return {cell for segment in (*self.segments.values(), *self.structures.values()) for cell in segment.cells
                if grid[cell] == 100}

    def save(self, path):
        if self.state != 'KNOWN_PASSAGE' or self.context is None or not (self.segments or self.structures):
            raise ValueError('NO_ACTIVE_RECORDS')
        body = {'version': 2, 'context': asdict(self.context),
                'segments': [asdict(s) for s in self.segments.values()]}
        if self.structures:
            body['version'] = 3
            body['structures'] = [asdict(s) for s in self.structures.values()]
        atomic_write_record(path, body)
        self.needs_save = False

    @classmethod
    def restore(cls, path, context, grid):
        memory = cls()
        try:
            if os.path.getsize(path) > 4_100_000:
                raise ValueError('RECORD_TOO_LARGE')
            with open(path, encoding='utf-8') as stream:
                wrapper = json.load(stream)
            body = wrapper['body']
            packed = json.dumps(body, sort_keys=True, separators=(',', ':'), allow_nan=False).encode()
            if (body['version'] not in (2, 3) or
                    hashlib.sha256(packed).hexdigest() != wrapper['sha256']):
                raise ValueError('INVALID_RECORD_INTEGRITY')
            saved_context = MapContext(**body['context'])
            structures = body.get('structures', []) if body['version'] == 3 else []
            if (not saved_context.valid() or not isinstance(body['segments'], list) or
                    not isinstance(structures, list) or
                    not 0 < len(body['segments'])+len(structures) <= memory.max_records):
                raise ValueError('INVALID_RECORD_CONTEXT')
            memory.context = saved_context
            for item in body['segments']:
                item['cells'] = tuple(item['cells'])
                segment = StoredSegment(**item)
                if (not segment.segment_id or not segment.cells or
                        any(type(i) is not int for i in segment.cells) or
                        len(set(segment.cells)) != len(segment.cells) or
                        not all(isfinite(v) for v in (segment.center_x, segment.center_y,
                            segment.yaw, segment.travel_length, segment.robot_length,
                            segment.robot_width)) or
                        min(segment.travel_length, segment.robot_length,
                            segment.robot_width) <= 0):
                    raise ValueError('INVALID_SAVED_SEGMENT')
                if not ((segment.ground_z is None and segment.robot_height is None) or
                        (segment.ground_z is not None and segment.robot_height is not None and
                         isfinite(segment.ground_z) and isfinite(segment.robot_height) and
                         segment.robot_height > 0)):
                    raise ValueError('INVALID_SAVED_VERTICAL_GEOMETRY')
                memory.segments[segment.segment_id] = segment
            for item in structures:
                item = dict(item)
                item['cells'] = tuple(item['cells'])
                item['columns'] = tuple(tuple(row) for row in item['columns'])
                item['map_transform'] = normalize_transform(item['map_transform'])
                structure = StoredStructure(**item)
                validate_columns(structure.columns, structure.resolution)
                if (not isinstance(structure.structure_id, str) or not structure.structure_id or
                        structure.structure_id in memory.structures or
                        not isinstance(structure.source_frame, str) or not structure.source_frame or
                        len(set(structure.cells)) != len(structure.cells)):
                    raise ValueError('INVALID_SAVED_STRUCTURE')
                # Old indices belong to the saved canvas. check() either compares
                # its patch hash or rerasterizes the measured structure on expansion.
                if any(type(i) is not int or i < 0 or
                       i >= saved_context.width*saved_context.height for i in structure.cells):
                    raise ValueError('INVALID_SAVED_STRUCTURE_CELLS')
                memory.structures[structure.structure_id] = structure
            memory.state, memory.reason = 'KNOWN_PASSAGE', 'RESTORED_STRUCTURE_ONLY'
            # A new navigation process normally has a new odom origin. The
            # immutable saved-map identity plus map geometry and segment-cell
            # hashes anchor historical geometry in map coordinates; the old
            # map->odom numerical value is not a cross-restart invariant.
            # Rebase only a saved-map record, then reject future TF jumps in
            # this process via check(). Never restore a short entry permit.
            if (saved_context.epoch.startswith('saved-map-') and
                    saved_context.epoch == context.epoch and
                    saved_context.geometry() == context.geometry()):
                memory.context = context
                memory.reason = 'RESTORED_SAVED_MAP_STRUCTURE_ONLY'
            if (saved_context.geometry() == context.geometry() and
                    all(_cell_hash(grid, record.cells, allow_empty=True) == record.map_patch_hash
                        for record in memory.structures.values())):
                memory._structure_grid = tuple(grid)
            memory.check(context, grid)
        except (OSError, ValueError, TypeError, KeyError, AttributeError):
            memory.segments.clear()
            memory.structures.clear()
            memory.suspend('RECORD_UNAVAILABLE_OR_INVALID')
        return memory
