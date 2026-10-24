"""Phase 3 laboratory model: verified structure memory, never entry permission.

The caller must provide a map-session identifier. If a real mapping launch cannot
identify reloads and pose corrections, it must not use this model to clear cells.
"""
from dataclasses import dataclass, asdict
from math import atan2, cos, hypot, isfinite, pi, sin
import hashlib
import json
import os
import tempfile
from rigid_transform import context_transform


@dataclass(frozen=True)
class MapContext:
    epoch: str
    frame: str
    width: int
    height: int
    resolution: float
    origin_x: float
    origin_y: float
    map_to_odom_x: float
    map_to_odom_y: float
    map_to_odom_yaw: float
    map_to_odom_z: float = 0.0
    map_to_odom_quaternion: tuple = ()

    def valid(self):
        try:
            context_transform(self)
        except (ValueError, TypeError, AttributeError):
            return False
        values = (self.resolution, self.origin_x, self.origin_y,
                  self.map_to_odom_x, self.map_to_odom_y, self.map_to_odom_yaw)
        return (bool(self.epoch) and self.frame == 'map' and self.width > 0 and
                self.height > 0 and isfinite(self.resolution) and self.resolution > 0 and
                all(isfinite(v) for v in values))

    def geometry(self):
        return (self.frame, self.width, self.height, self.resolution,
                self.origin_x, self.origin_y)


@dataclass(frozen=True)
class ConfirmedPassage:
    # Half-open map-cell rectangle. In production this must come from a real
    # three-dimensional validator and keep keyframe/posegraph provenance.
    left: int
    right: int
    bottom: int
    top: int
    frame: str
    confirmed_frames: int
    measured_at: float
    evidence_id: str
    map_patch_hash: str = ''  # Optional in the old lab model; required by live shadow adapter.


class GlobalPassageMemory:
    """In-session candidate for global planning; does not authorize crossing."""

    def __init__(self, max_pose_shift=.25, max_yaw_shift=.15, evidence_age=2.0):
        if not all(isfinite(v) and v > 0 for v in (
                max_pose_shift, max_yaw_shift, evidence_age)):
            raise ValueError('Invalid memory thresholds')
        self.max_pose_shift = max_pose_shift
        self.max_yaw_shift = max_yaw_shift
        self.evidence_age = evidence_age
        self.context = None
        self.passage = None
        self.used_evidence_ids = set()
        self.state = 'UNKNOWN'
        self.reason = 'NO_VERIFIED_PASSAGE'

    def observe(self, passage, context, now):
        if not context.valid() or not isfinite(now) or not isfinite(passage.measured_at) or (
                passage.frame != context.frame or passage.confirmed_frames < 3 or
                not passage.evidence_id or passage.evidence_id in self.used_evidence_ids or
                passage.measured_at <= 0 or
                not 0 <= now-passage.measured_at <= self.evidence_age or
                not 0 <= passage.left < passage.right <= context.width or
                not 0 <= passage.bottom < passage.top <= context.height):
            self.suspend('INVALID_OR_STALE_VERIFICATION')
            return False
        # A fresh validator event is required after a conflict or map correction.
        self.context, self.passage = context, passage
        self.used_evidence_ids.add(passage.evidence_id)
        self.state, self.reason = 'KNOWN_PASSAGE', 'HISTORICAL_STRUCTURE_ONLY'
        return True

    def check_context(self, context):
        if not context.valid():
            self.suspend('INVALID_MAP_CONTEXT')
        elif self.context is None:
            self.suspend('NO_VERIFIED_PASSAGE')
        elif context.epoch != self.context.epoch or context.geometry() != self.context.geometry():
            self.suspend('MAP_RELOADED_OR_GEOMETRY_CHANGED')
        else:
            dx = context.map_to_odom_x-self.context.map_to_odom_x
            dy = context.map_to_odom_y-self.context.map_to_odom_y
            da = atan2(sin(context.map_to_odom_yaw-self.context.map_to_odom_yaw),
                       cos(context.map_to_odom_yaw-self.context.map_to_odom_yaw))
            if hypot(dx, dy) > self.max_pose_shift or abs(da) > self.max_yaw_shift:
                self.suspend('MAP_ODOM_CORRECTION_UNASSOCIATED')
        return self.state == 'KNOWN_PASSAGE'

    def obstacle_conflict(self):
        self.suspend('NEW_OBSTACLE_CONFLICT')

    def suspend(self, reason):
        self.state, self.reason = 'SUSPENDED', reason

    def planning_cells(self, context):
        if not self.check_context(context):
            return set()
        p = self.passage
        return {y*context.width+x for y in range(p.bottom, p.top)
                for x in range(p.left, p.right)}

    def save(self, path):
        """Atomically persist only an active historical structure record."""
        if self.state != 'KNOWN_PASSAGE' or self.context is None or self.passage is None:
            raise ValueError('No active verified passage to save')
        body = {'version': 1, 'context': asdict(self.context),
                'passage': asdict(self.passage)}
        packed = json.dumps(body, sort_keys=True, separators=(',', ':'), allow_nan=False).encode()
        wrapper = {'body': body, 'sha256': hashlib.sha256(packed).hexdigest()}
        directory = os.path.dirname(os.path.abspath(path))
        fd, temporary = tempfile.mkstemp(prefix='.passage-', dir=directory)
        try:
            with os.fdopen(fd, 'w', encoding='utf-8') as out:
                os.fchmod(out.fileno(), 0o600)
                json.dump(wrapper, out, sort_keys=True, allow_nan=False)
                out.flush()
                os.fsync(out.fileno())
            os.replace(temporary, path)
            folder = os.open(directory, os.O_RDONLY)
            try:
                os.fsync(folder)
            finally:
                os.close(folder)
        finally:
            if os.path.exists(temporary):
                os.unlink(temporary)

    @classmethod
    def restore(cls, path, current_context):
        """Reload structure only when the caller proves the same map session."""
        result = cls()
        try:
            if os.path.getsize(path) > 65536:
                raise ValueError('Record too large')
            with open(path, encoding='utf-8') as stream:
                wrapper = json.load(stream)
            body = wrapper['body']
            packed = json.dumps(body, sort_keys=True, separators=(',', ':'), allow_nan=False).encode()
            if hashlib.sha256(packed).hexdigest() != wrapper['sha256'] or body['version'] != 1:
                raise ValueError('Record integrity/version mismatch')
            saved_context = MapContext(**body['context'])
            passage = ConfirmedPassage(**body['passage'])
            if (not saved_context.valid() or passage.frame != saved_context.frame or
                    passage.confirmed_frames < 3 or not passage.evidence_id or
                    not isfinite(passage.measured_at) or passage.measured_at <= 0 or
                    not 0 <= passage.left < passage.right <= saved_context.width or
                    not 0 <= passage.bottom < passage.top <= saved_context.height):
                raise ValueError('Invalid saved passage')
            result.context = saved_context
            result.passage = passage
            result.used_evidence_ids.add(passage.evidence_id)
            result.state, result.reason = 'KNOWN_PASSAGE', 'RESTORED_STRUCTURE_ONLY'
            result.check_context(current_context)
        except (OSError, AttributeError, KeyError, TypeError, ValueError,
                json.JSONDecodeError):
            result.suspend('RECORD_UNAVAILABLE_OR_INVALID')
        return result
