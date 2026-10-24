#!/usr/bin/env python3
"""Entry-gate edge cases: history never grants crossing by itself."""
from passage_entry_manager import ControlledSegment, FreshPermit, PassageEntryManager
from dataclasses import replace


def run():
    segment = ControlledSegment('probe-A', 2., 0., 0., .2, .45, .45, .55)
    permit = FreshPermit('probe-A', 9.5, 10.2, 3)

    def evaluate(gate, path, pose, speed=.2, now=10., proof=None, **overrides):
        args = dict(path=path, path_stamp=now, pose=pose, pose_stamp=now,
                    speed=speed, now=now, segment=segment, permit=proof,
                    obstacle=False, obstacle_stamp=now, map_associated=True)
        args.update(overrides)
        return gate.evaluate(**args)

    gate = PassageEntryManager()
    approach = [(0., 0.), (3., 0.)]
    d = evaluate(gate, approach, (0., 0.))
    assert d.state == 'WAIT_VERIFY' and 0 < d.speed_limit < 2
    assert abs(d.distance_to_entry - 1.9) < 1e-6
    d = evaluate(gate, [(1.6, 0.), (3., 0.)], (1.6, 0.))
    assert d.state == 'BLOCKED' and d.stop
    d = evaluate(gate, [(1.6, 0.), (3., 0.)], (1.6, 0.), proof=permit)
    assert d.state == 'ALLOWED' and not d.stop

    near_path = [(1.6, 0.), (3., 0.)]
    assert evaluate(gate, near_path, (1.6, 0.), proof=FreshPermit('other', 9.5, 10.2, 3)).stop
    assert evaluate(gate, near_path, (1.6, 0.), proof=FreshPermit('probe-A', 9.5, 10.2, 2)).stop
    assert evaluate(gate, approach, (0., 0.), proof=permit, obstacle=True).reason == 'LIVE_OBSTACLE_VETO'
    assert evaluate(gate, approach, (0., 0.), proof=permit, map_associated=False).stop
    assert evaluate(gate, approach, (0., 0.), proof=permit, path_stamp=9.).stop
    assert evaluate(gate, approach, (0., 0.), proof=permit, pose_stamp=9.).stop
    assert evaluate(gate, approach, (0., 0.), proof=permit, obstacle_stamp=9.).stop

    assert evaluate(gate, [(2., -1.), (2., 1.)], (2., -1.), proof=permit).stop
    assert evaluate(gate, [(0., .1), (3., .1)], (0., .1), proof=permit).stop
    assert evaluate(gate, [(0., 1.), (3., 1.)], (0., 1.)).state == 'NOT_ON_PATH'
    assert evaluate(gate, approach, (1., 0.), proof=permit).stop  # stale path anchor
    assert evaluate(gate, [(2., 0.), (3., 0.)], (2., 0.)).stop
    assert evaluate(gate, [(2.2, 0.), (3., 0.)], (2.2, 0.)).stop  # rear still overlaps

    d = evaluate(gate, approach, (0., 0.), now=9.)
    assert d.reason == 'CLOCK_INVALID_OR_REWOUND'
    assert evaluate(gate, approach, (0., 0.), now=11.).reason == 'CLOCK_RESET_REQUIRED'
    gate.reset_after_clock_change()
    assert evaluate(gate, approach, (0., 0.), now=11.).state == 'WAIT_VERIFY'
    assert evaluate(gate, approach, (0., 0.), now=11., proof=permit).state == 'WAIT_VERIFY'
    # A planner may need adjacent verified strips to create enough inflated
    # width. Their peripheral overlaps must not veto a centre strip, but an
    # uncovered side approach remains blocked.
    centre = ControlledSegment('centre', -.05, .05, 0., .2, .45, .45, .45)
    side = ControlledSegment('side', -.05, -.25, 0., .2, .45, .45, .45)
    common_path = [(-2., .05), (2., .05)]
    assert gate.peripheral_overlap_covered(common_path, side, [centre])
    assert not gate.peripheral_overlap_covered(common_path, side, [])
    shifted = [(-2., -.05), (2., -.05)]
    assert not gate.peripheral_overlap_covered(shifted, side, [centre])
    assert not gate.peripheral_overlap_covered(
        [(-1., .05), (-.375, .05)], side, [])  # tangent alone is not coverage
    side_decision = evaluate(gate, common_path, (-2., .05), now=12., segment=side)
    assert side_decision.reason == 'PATH_OVERLAPS_UNVERIFIED_CORRIDOR_EDGE'
    centre_decision = evaluate(gate, common_path, (-2., .05), now=12., segment=centre)
    assert centre_decision.state == 'WAIT_VERIFY'
    dense_path = [(-2.+i*.05, .05) for i in range(81)]
    assert evaluate(gate, dense_path, (-2., .05), now=12., segment=centre).state == 'WAIT_VERIFY'
    dense_edge = [(-2.+i*.05, .10) for i in range(81)]
    assert evaluate(gate, dense_edge, (-2., .10), now=12., segment=centre).stop

    strips = [ControlledSegment(f'strip-{y}', 1.7, y, 0., .2, .45, .45, .45, 0., .7)
              for y in (-.25, 0., .25)]
    groups = gate.lateral_groups(strips)
    assert len(groups) == 1 and len(groups[0][1]) == 3
    assert abs(groups[0][0].verified_width-.95) < 1e-9
    assert groups == gate.lateral_groups(list(reversed(strips)))
    assert len(gate.lateral_groups([strips[0], strips[2]])) == 2  # 5 cm unknown gap
    assert len(gate.lateral_groups([strips[0], replace(strips[1], center_x=1.71)])) == 2
    assert len(gate.lateral_groups([strips[0], replace(strips[1], yaw=.01)])) == 2
    assert len(gate.lateral_groups([strips[0], replace(strips[1], travel_length=.3)])) == 2
    assert len(gate.lateral_groups([strips[0], replace(strips[1], ground_z=.2)])) == 2
    assert len(gate.lateral_groups([strips[0], replace(strips[1], robot_height=.5)])) == 2

    def catalog(y=0., x=0., proofs=None, now=20., obstacle=False, selected=None, path=None):
        path = path or [(x, y), (2.2, y)]
        return PassageEntryManager().evaluate_catalog(
            segments=strips if selected is None else selected, permits=proofs or {},
            path=path, path_stamp=now, pose=path[0], pose_stamp=now,
            speed=.2, now=now, obstacle=obstacle, obstacle_stamp=now, map_associated=True)

    fresh = {s.segment_id: FreshPermit(s.segment_id, 19.8, 20.8, 3) for s in strips}
    central = {strips[1].segment_id: fresh[strips[1].segment_id]}
    lower_and_centre = {s.segment_id: fresh[s.segment_id] for s in strips[:2]}
    assert catalog(y=-.14)[0].state == 'WAIT_VERIFY'
    assert catalog(y=-.14, x=1.3)[0].reason == 'STOP_BEFORE_UNVERIFIED_ENTRY'
    assert catalog(x=1.3, proofs=central)[0].state == 'ALLOWED'
    assert catalog(y=-.14, x=1.3, proofs=central)[0].stop  # old side strip cannot grant entry
    assert catalog(y=-.14, x=1.3, proofs=lower_and_centre)[0].state == 'ALLOWED'
    legacy_strips = [replace(s, ground_z=None, robot_height=None) for s in strips]
    assert catalog(y=-.14, x=1.3, proofs=fresh, selected=legacy_strips)[0].stop
    assert catalog(y=.14, x=1.3, proofs=lower_and_centre)[0].stop
    assert catalog(y=.14, x=1.3, proofs=fresh)[0].state == 'ALLOWED'
    assert catalog(y=.26, x=1.3, proofs=fresh)[0].stop  # outside body-safe union
    assert catalog(y=-.14, x=1.3, proofs=lower_and_centre, now=20.81)[0].stop
    assert catalog(y=-.14, x=1.3, proofs=fresh, obstacle=True)[0].reason == 'LIVE_OBSTACLE_VETO'
    assert all(d.stop for d in catalog(x=1.3, proofs=fresh, selected=[strips[0], strips[2]]))
    # Cover the entire path/body overlap, not only its centre or first point.
    assert catalog(x=1.3, proofs=central,
                   path=[(1.3, 0.), (1.7, 0.), (1.9, .10), (2.2, .10)])[0].stop
    forward = replace(strips[1], segment_id='next', center_x=2.6)
    assert len(gate.lateral_groups(strips+[forward])) == 2
    second = catalog(x=1.3, proofs=fresh, selected=strips+[forward],
                     path=[(1.3, 0.), (3., 0.)])
    assert [d.state for d in second] == ['ALLOWED', 'WAIT_VERIFY']
    print('Entry gate: approach, braking, fresh proof, obstacle, clock, stale inputs and side-entry passed')


if __name__ == '__main__':
    run()
