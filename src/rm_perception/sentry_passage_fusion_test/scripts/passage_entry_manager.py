"""Short-segment entry interlock core; never treats historical memory as permission.

The caller must ensure this is the only route from Nav2's desired velocity to the
chassis and must provide fresh pose, path and observed obstacle state. Until that
wiring is proven, use this class only in the isolated experiment.
"""
from dataclasses import dataclass
import hashlib
import json
from math import cos, hypot, isfinite, sin, sqrt


@dataclass(frozen=True)
class ControlledSegment:
    segment_id: str
    center_x: float
    center_y: float
    yaw: float
    travel_length: float
    robot_length: float
    robot_width: float
    verified_width: float
    ground_z: float = None
    robot_height: float = None

    def valid(self):
        return (bool(self.segment_id) and all(isfinite(v) for v in (
            self.center_x, self.center_y, self.yaw, self.travel_length,
            self.robot_length, self.robot_width, self.verified_width)) and
            min(self.travel_length, self.robot_length, self.robot_width) > 0 and
            self.verified_width >= self.robot_width and
            ((self.ground_z is None and self.robot_height is None) or
             (self.ground_z is not None and self.robot_height is not None and
              isfinite(self.ground_z) and isfinite(self.robot_height) and self.robot_height > 0)))


@dataclass(frozen=True)
class FreshPermit:
    segment_id: str
    measured_at: float
    valid_until: float
    confirmed_frames: int

    def valid_for(self, segment, now):
        return (self.segment_id == segment.segment_id and
                self.confirmed_frames >= 3 and
                all(isfinite(v) for v in (now, self.measured_at, self.valid_until)) and
                0 <= now - self.measured_at <= 1.0 and
                self.measured_at <= now <= self.valid_until <= self.measured_at + 1.0)


@dataclass(frozen=True)
class GateDecision:
    state: str
    reason: str
    stop: bool
    speed_limit: float
    distance_to_entry: float


class PassageEntryManager:
    def __init__(self, minimum_deceleration=.8, latency=.2, margin=.1,
                 path_start_tolerance=.5, path_max_age=.5):
        values = (minimum_deceleration, latency, margin, path_start_tolerance, path_max_age)
        if not all(isfinite(v) and v > 0 for v in values):
            raise ValueError('Invalid braking or path parameters')
        self.a = minimum_deceleration
        self.latency = latency
        self.margin = margin
        self.path_start_tolerance = path_start_tolerance
        self.path_max_age = path_max_age
        self.last_now = 0.0
        self.clock_fault = False

    def reset_after_clock_change(self):
        """The caller must discard all old proofs and paths before resetting."""
        self.last_now = 0.0
        self.clock_fault = False

    @staticmethod
    def _local(point, segment):
        dx, dy = point[0]-segment.center_x, point[1]-segment.center_y
        c, s = cos(segment.yaw), sin(segment.yaw)
        return c*dx+s*dy, -s*dx+c*dy

    @staticmethod
    def _clip(a, b, half_travel, half_width):
        """Return first/last parameters within a rectangle, or None."""
        du, dv = b[0]-a[0], b[1]-a[1]
        lo, hi = 0.0, 1.0
        for p, q in ((-du, a[0]+half_travel), (du, half_travel-a[0]),
                     (-dv, a[1]+half_width), (dv, half_width-a[1])):
            if p == 0:
                if q < 0:
                    return None
            elif p < 0:
                lo = max(lo, q/p)
            else:
                hi = min(hi, q/p)
        return (lo, hi) if lo <= hi else None

    @classmethod
    def lateral_groups(cls, segments, *, fresh=False):
        """Union only overlapping, co-oriented, equal-depth measured rectangles.

        Never fill a gap or enlarge a longitudinal sweep. Exact grouping is
        deliberately conservative: differing headings, depths, body dimensions
        or longitudinal centres remain separate gates. Membership is retained
        so historic side observations cannot become a fresh entry permission.
        """
        buckets = {}
        for segment in segments:
            if not segment.valid():
                raise ValueError('INVALID_SEGMENT')
            c, s = cos(segment.yaw), sin(segment.yaw)
            u = c*segment.center_x+s*segment.center_y
            v = -s*segment.center_x+c*segment.center_y
            key = (segment.yaw, segment.travel_length, segment.robot_length,
                   segment.robot_width, u, segment.ground_z, segment.robot_height,
                   segment.segment_id if fresh and segment.ground_z is None else '')
            buckets.setdefault(key, []).append((v-segment.verified_width/2,
                                                v+segment.verified_width/2, segment))
        groups = []
        for key, intervals in sorted(buckets.items(), key=lambda item: repr(item[0])):
            components = []
            for lo, hi, segment in sorted(intervals, key=lambda item: (item[0], item[2].segment_id)):
                if not components or lo > components[-1][1]:
                    components.append([lo, hi, [segment]])
                else:
                    components[-1][1] = max(components[-1][1], hi)
                    components[-1][2].append(segment)
            yaw, travel, length, width, u, ground, height, _ = key
            for lo, hi, members in components:
                if len(members) == 1:
                    groups.append((members[0], tuple(members)))
                    continue
                # The result is precisely the union rectangle, not its inflated
                # costmap or a bounding box over disconnected observations.
                v = (lo+hi)/2
                identity = json.dumps(sorted(member.segment_id for member in members))
                merged = ControlledSegment(
                    'lateral-union:'+hashlib.sha256(identity.encode()).hexdigest(),
                    cos(yaw)*u-sin(yaw)*v, sin(yaw)*u+cos(yaw)*v,
                    yaw, travel, length, width, hi-lo, ground, height)
                groups.append((merged, tuple(members)))
        return groups

    def evaluate_catalog(self, *, segments, permits, **inputs):
        """Evaluate a historical union, with fresh coverage checked separately.

        A temporary group permit is derived only if currently valid ORIGINAL
        member permits cover the complete path/body overlap. It is local to
        this evaluation, path-dependent, and never published or persisted.
        """
        groups = self.lateral_groups(segments)
        decisions = []
        for combined, members in groups:
            fresh = [member for member in members
                     if permits.get(member.segment_id) is not None and
                     permits[member.segment_id].valid_for(member, inputs['now'])]
            # Older records without vertical metadata may still help global
            # planning, but cannot jointly authorize a new composite volume.
            fresh_groups = [part for part, _ in self.lateral_groups(fresh, fresh=True)]
            permission = None
            if fresh_groups and self.peripheral_overlap_covered(
                    inputs['path'], combined, fresh_groups):
                proofs = [permits[member.segment_id] for member in fresh]
                permission = FreshPermit(
                    combined.segment_id, min(p.measured_at for p in proofs),
                    min(p.valid_until for p in proofs), min(p.confirmed_frames for p in proofs))
            decisions.append(self.evaluate(segment=combined, permit=permission, **inputs))

        # Separate longitudinal/heading groups can still have peripheral body
        # envelopes touching the same path. Ignore that warning only when a
        # valid independently evaluated group covers the entire overlap.
        result = []
        for index, decision in enumerate(decisions):
            if decision.reason == 'PATH_OVERLAPS_UNVERIFIED_CORRIDOR_EDGE':
                covering = [group for other, (group, _) in enumerate(groups)
                            if other != index and decisions[other].state in
                            ('WAIT_VERIFY', 'ALLOWED', 'BLOCKED') and
                            decisions[other].reason not in
                            ('INVALID_OR_STALE_PATH', 'PATH_OVERLAPS_UNVERIFIED_CORRIDOR_EDGE')]
                if self.peripheral_overlap_covered(inputs['path'], groups[index][0], covering):
                    continue
            result.append(decision)
        return result

    @classmethod
    def peripheral_overlap_covered(cls, path, side_segment, other_segments):
        """Only disregard a side-strip edge warning if another body-safe strip covers it.

        Interval union is checked on every path edge, not at sparse sample points.
        This does not grant entry: the covering strip is still independently
        evaluated and needs its own fresh permit at the entry boundary.
        """
        if (not side_segment.valid() or len(path) < 2 or
                not all(isfinite(v) for point in path for v in point)):
            return False
        found = False
        for start, end in zip(path, path[1:]):
            a = cls._local(start, side_segment)
            b = cls._local(end, side_segment)
            outer = cls._clip(a, b,
                              (side_segment.travel_length+side_segment.robot_length)/2,
                              (side_segment.verified_width+side_segment.robot_width)/2)
            if outer is None:
                continue
            found = True
            cover = []
            for alternative in other_segments:
                if not alternative.valid():
                    continue
                c = cls._local(start, alternative)
                d = cls._local(end, alternative)
                interval = cls._clip(c, d,
                                     (alternative.travel_length+alternative.robot_length)/2,
                                     (alternative.verified_width-alternative.robot_width)/2+1e-6)
                if interval is not None:
                    cover.append(interval)
            if outer[1]-outer[0] <= 1e-9:
                if not any(lo-1e-9 <= outer[0] <= hi+1e-9
                           for lo, hi in cover):
                    return False
                continue
            cursor = outer[0]
            for lo, hi in sorted(cover):
                if hi < cursor:
                    continue
                if lo > cursor+1e-9:
                    return False
                cursor = max(cursor, hi)
                if cursor >= outer[1]-1e-9:
                    break
            if cursor < outer[1]-1e-9:
                return False
        return found

    def _distance_to_entry(self, path, pose, segment):
        """Distance along a map-frame path to the first controlled rectangle."""
        if len(path) < 2 or hypot(path[0][0]-pose[0], path[0][1]-pose[1]) > self.path_start_tolerance:
            raise ValueError('PATH_NOT_ANCHORED_AT_ROBOT')
        local = [self._local(point, segment) for point in path]
        if not all(isfinite(v) for pair in local for v in pair):
            raise ValueError('INVALID_PATH')
        start_u, start_v = local[0]
        half_travel = segment.travel_length / 2
        outer_half_travel = half_travel + segment.robot_length / 2
        # A validated body-width corridor does not permit arbitrary lateral
        # displacement of the robot centre. Only the spare width does.
        half_width = (segment.verified_width-segment.robot_width) / 2 + 1e-6
        outer_half_width = (segment.verified_width+segment.robot_width) / 2
        if abs(start_u) <= outer_half_travel and abs(start_v) <= outer_half_width:
            if abs(start_v) > half_width:
                raise ValueError('ROBOT_ALREADY_OVERLAPS_UNVERIFIED_EDGE')
            return 0.0  # Already in the controlled segment.
        travelled = 0.0
        for a, b in zip(local, local[1:]):
            du, dv = b[0]-a[0], b[1]-a[1]
            length = hypot(du, dv)
            if length == 0:
                continue
            # Detect even a centreline outside the verified channel when the
            # robot body would overlap the passage. It must not bypass the gate.
            outer = self._clip(a, b, outer_half_travel, outer_half_width)
            if outer is not None:
                outer_entry_v = a[1] + outer[0]*dv
                outer_exit_v = a[1] + outer[1]*dv
                if (abs(outer_entry_v) > half_width or
                        abs(outer_exit_v) > half_width):
                    raise ValueError('PATH_OVERLAPS_UNVERIFIED_CORRIDOR_EDGE')
                # A dense planner path usually crosses the outer braking
                # envelope on earlier edges than the actual entry plane. Its
                # first outer-only edge is not a lateral safety violation.
                inner = self._clip(a, b, half_travel, half_width)
                if inner is None:
                    travelled += length
                    continue
                lo = inner[0]
                entry_u, entry_v = a[0]+lo*du, a[1]+lo*dv
                if (abs(abs(entry_u)-half_travel) > 1e-7 or
                        abs(entry_v) > half_width or
                        du*entry_u >= 0):
                    raise ValueError('PATH_ENTERS_PASSAGE_FROM_UNVERIFIED_SIDE')
                return travelled + length*lo
            travelled += length
        return None

    def evaluate(self, *, path, path_stamp, pose, pose_stamp, speed, now,
                 segment, permit=None, obstacle=False, obstacle_stamp=None,
                 map_associated=False):
        if not isfinite(now) or now <= 0 or now < self.last_now:
            self.clock_fault = True
            return GateDecision('BLOCKED', 'CLOCK_INVALID_OR_REWOUND', True, 0.0, 0.0)
        if self.clock_fault:
            return GateDecision('BLOCKED', 'CLOCK_RESET_REQUIRED', True, 0.0, 0.0)
        self.last_now = now
        if path_stamp is None or not isfinite(path_stamp) or not 0 <= now-path_stamp <= self.path_max_age:
            return GateDecision('BLOCKED', 'STALE_PATH_POSE_OR_OBSTACLE', True, 0.0, 0.0)
        stamps = (pose_stamp, obstacle_stamp)
        if not all(v is not None and isfinite(v) and 0 <= now-v <= .5
                   for v in stamps):
            return GateDecision('BLOCKED', 'STALE_PATH_POSE_OR_OBSTACLE', True, 0.0, 0.0)
        if (not map_associated or segment is None or not segment.valid() or
                not all(isfinite(v) for v in (*pose, speed)) or speed < 0):
            return GateDecision('BLOCKED', 'MISSING_CONTEXT_OR_ODOMETRY', True, 0.0, 0.0)
        try:
            distance = self._distance_to_entry(path, pose, segment)
        except (ValueError, IndexError, TypeError) as error:
            detail = str(error)
            reason = {
                'PATH_OVERLAPS_UNVERIFIED_CORRIDOR_EDGE':
                    'PATH_OVERLAPS_UNVERIFIED_CORRIDOR_EDGE',
                'ROBOT_ALREADY_OVERLAPS_UNVERIFIED_EDGE':
                    'ROBOT_IN_UNVERIFIED_PASSAGE_EDGE',
                'PATH_ENTERS_PASSAGE_FROM_UNVERIFIED_SIDE':
                    'PATH_ENTERS_UNVERIFIED_PASSAGE_SIDE',
                'PATH_NOT_ANCHORED_AT_ROBOT': 'INVALID_OR_STALE_PATH',
                'INVALID_PATH': 'INVALID_OR_STALE_PATH',
            }.get(detail, 'INVALID_OR_STALE_PATH')
            return GateDecision('BLOCKED', reason, True, 0.0, 0.0)
        if distance is None:
            return GateDecision('NOT_ON_PATH', 'ROUTE_DOES_NOT_ENTER_THIS_SEGMENT',
                                False, float('inf'), float('inf'))
        if obstacle:
            return GateDecision('BLOCKED', 'LIVE_OBSTACLE_VETO', True, 0.0, distance)
        if permit is not None and permit.valid_for(segment, now):
            return GateDecision('ALLOWED', 'FRESH_SEGMENT_VERIFIED',
                                False, float('inf'), distance)
        safe_distance = distance - segment.robot_length/2 - self.margin
        stopping_distance = speed*speed/(2*self.a) + speed*self.latency
        if safe_distance <= stopping_distance:
            return GateDecision('BLOCKED', 'STOP_BEFORE_UNVERIFIED_ENTRY',
                                True, 0.0, distance)
        # Limit the approach speed so the next cycle can still stop before entry.
        limit = max(0.0, sqrt((self.a*self.latency)**2 + 2*self.a*safe_distance)
                    - self.a*self.latency)
        return GateDecision('WAIT_VERIFY', 'APPROACH_WITH_BRAKING_RESERVE',
                            False, limit, distance)
