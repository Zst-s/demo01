#!/usr/bin/env python3
"""Real Nav2 layers consume remembered short ROGMap evidence in an isolated domain."""
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import time

import rclpy
from geometry_msgs.msg import TransformStamped, TwistStamped, PoseStamped
from nav_msgs.msg import Odometry, Path as NavPath
from std_msgs.msg import String, Bool
from tf2_ros import StaticTransformBroadcaster

from fusion_scenario import Scenario, WIDTH


def run():
    if os.environ.get('ROS_DOMAIN_ID') != '221' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
        raise RuntimeError('Isolated ROS domain 221 required')
    rclpy.init()
    scenario = Scenario()
    proof_pub = scenario.node.create_publisher(
        String, '/terrain/clearance/verified_segment', 10)
    received = {'status': None}
    gate_status = {'value': None, 'command': None}
    status_sub = scenario.node.create_subscription(
        String, '/passage/memory_shadow_status',
        lambda msg: received.update(status=json.loads(msg.data)), 10)
    gate_status_sub = scenario.node.create_subscription(
        String, '/passage_fusion_test/entry_gate_status',
        lambda msg: gate_status.update(value=json.loads(msg.data)), 10)
    authorized_sub = scenario.node.create_subscription(
        TwistStamped, '/passage_fusion_test/cmd_vel_authorized',
        lambda msg: gate_status.update(command=msg), 10)
    path_pub = scenario.node.create_publisher(
        NavPath, '/passage_fusion_test/planned_path', 10)
    odom_pub = scenario.node.create_publisher(
        Odometry, '/passage_fusion_test/odom_map', 10)
    obstacle_pub = scenario.node.create_publisher(
        Bool, '/passage_fusion_test/live_obstacle', 10)
    desired_pub = scenario.node.create_publisher(
        TwistStamped, '/passage_fusion_test/cmd_vel_desired', 10)
    goal_pub = scenario.node.create_publisher(
        PoseStamped, '/passage_fusion_test/desired_goal', 10)
    probes = []
    probe_status = {'value': None}
    probe_sub = scenario.node.create_subscription(
        PoseStamped, '/terrain/clearance/probe', lambda msg: probes.append(msg), 10)
    probe_status_sub = scenario.node.create_subscription(
        String, '/passage_fusion_test/probe_status',
        lambda msg: probe_status.update(value=json.loads(msg.data)), 10)
    broadcaster = StaticTransformBroadcaster(scenario.node)
    transform = TransformStamped()
    transform.header.frame_id = 'map'
    transform.child_frame_id = 'odom'
    transform.transform.rotation.w = 1.
    harness = shadow = gate = coordinator = None
    robot_x = [-1.]
    goal_x = [1.]

    def send_gate_inputs():
        stamp = scenario.node.get_clock().now().to_msg()
        path = NavPath()
        path.header.frame_id = 'map'
        path.header.stamp = stamp
        for x in (robot_x[0], goal_x[0]):
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = x
            pose.pose.position.y = .05
            pose.pose.orientation.w = 1.
            path.poses.append(pose)
        path_pub.publish(path)
        odom = Odometry()
        odom.header.frame_id = 'map'
        odom.header.stamp = stamp
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = robot_x[0]
        odom.pose.pose.position.y = .05
        odom.pose.pose.orientation.w = 1.
        odom.twist.twist.linear.x = .2
        odom_pub.publish(odom)
        obstacle_pub.publish(Bool(data=scenario.dynamic))
        desired = TwistStamped()
        desired.header.frame_id = 'base_link'
        desired.header.stamp = stamp
        desired.twist.linear.x = .2
        desired_pub.publish(desired)
        goal = PoseStamped()
        goal.header = path.header
        goal.pose.position.x = goal_x[0]
        goal.pose.position.y = .05
        goal.pose.orientation.w = 1.
        goal_pub.publish(goal)

    def wait_for(predicate, label, seconds=6):
        end = time.monotonic()+seconds
        while time.monotonic() < end:
            send_gate_inputs()
            scenario.tick()
            if predicate():
                return
        probe_log_path = Path(directory)/'probe.log'
        probe_log = (probe_log_path.read_text(encoding='utf-8')
                     if probe_log_path.exists() else '')
        raise AssertionError(f'{label}: cost={None if scenario.raw is None else scenario.cost(40,40)} '
                             f'status={received["status"]} '
                             f'gate={gate_status["value"]} robot_x={robot_x[0]} '
                             f'probe_status={probe_status["value"]} probe_log={probe_log[-1000:]}')

    def proof(old_oldest=False):
        stamp = scenario.node.get_clock().now().nanoseconds/1e9
        assert probes, 'No automatic 2D candidate probe was emitted'
        selected = probes[-1]
        # The integration fixture represents a partly occluded tunnel: old
        # interior observations can establish history, while only the newly
        # observed leading body/floor slice can issue an entry permit.
        body_bins = [[0, 48] for _ in range(6)] + [
            [48, 48], [47 if old_oldest else 48, 48]]
        support_bins = [[0, 12] for _ in range(3)] + [[12, 12]]
        recent_edge = body_bins[6][0] + body_bins[7][0] + support_bins[3][0]
        return String(data=json.dumps({
            'schema': 1, 'kind': 'verified_short_segment_observation',
            'frame': 'odom', 'probe_x': selected.pose.position.x,
            'probe_y': selected.pose.position.y, 'ground_z': -.025,
            'yaw': 0., 'travel_length': .2, 'robot_length': .45,
            'robot_width': .45, 'robot_height': .7, 'resolution': .1,
            'revision': 1, 'confirmed_frames': 3, 'measured_at': stamp,
            'oldest_observation': stamp-(8. if old_oldest else .1),
            'newest_observation': stamp,
            'oldest_body_observation': stamp-(8. if old_oldest else .1),
            'newest_body_observation': stamp,
            'oldest_support_observation': stamp-(8. if old_oldest else .1),
            'newest_support_observation': stamp,
            'oldest_entry_observation': stamp-(8. if old_oldest else .1),
            'newest_entry_observation': stamp,
            'recent_body_voxels': sum(pair[0] for pair in body_bins),
            'body_voxels': sum(pair[1] for pair in body_bins),
            'entry_recent_window': 2., 'recent_body_by_slice': body_bins,
            'recent_support_by_slice': support_bins,
            'recent_entry_edge_voxels': recent_edge,
            'entry_edge_voxels': sum(pair[1] for pair in body_bins[6:8]) +
                support_bins[3][1],
            'valid_until': stamp+.8, 'checked_voxels': 432}))

    try:
        with tempfile.TemporaryDirectory(prefix='memory-fusion-wire-') as directory:
            root = Path(__file__).resolve().parents[1]
            record = Path(directory)/'passages.json'
            with open(Path(directory)/'harness.log', 'w', encoding='utf-8') as out:
                harness = subprocess.Popen([
                    sys.argv[1], '--ros-args', '--params-file',
                    str(root/'config/fusion_test.yaml')], stdout=out,
                    stderr=subprocess.STDOUT)
            wait_for(lambda: scenario.map_pub.get_subscription_count() >= 2,
                     'Nav2 test layers did not subscribe')
            scenario.map_pub.publish(scenario.map)
            broadcaster.sendTransform(transform)
            wait_for(lambda: scenario.raw is not None and scenario.cost(40,40) == 254,
                     'Baseline wall')
            baseline = list(scenario.raw.data)

            def start_shadow():
                output = open(Path(directory)/'shadow.log', 'a', encoding='utf-8')
                proc = subprocess.Popen([
                    sys.executable, str(root/'scripts/live_memory_shadow.py'),
                    '--ros-args', '-r', '__ns:=/passage_fusion_test',
                    '-r', '/map:=/passage_fusion_test/map',
                    '-p', 'map_session_id:=integration-session',
                    '-p', f'record_path:={record}', '-p', 'evidence_frame:=odom',
                    '-p', 'publish_test_mask:=true'], stdout=output,
                    stderr=subprocess.STDOUT)
                output.close()
                return proc

            shadow = start_shadow()
            gate_log = open(Path(directory)/'gate.log', 'w', encoding='utf-8')
            gate = subprocess.Popen([
                sys.executable, str(root/'scripts/isolated_entry_gate.py'),
                '--ros-args', '-r', '__ns:=/passage_fusion_test'],
                stdout=gate_log, stderr=subprocess.STDOUT)
            gate_log.close()
            probe_log = open(Path(directory)/'probe.log', 'w', encoding='utf-8')
            coordinator = subprocess.Popen([
                sys.executable, str(root/'scripts/progressive_probe_coordinator.py'),
                '--ros-args', '-r', '__ns:=/passage_fusion_test',
                '-p', 'evidence_frame:=odom'],
                stdout=probe_log, stderr=subprocess.STDOUT)
            probe_log.close()
            wait_for(lambda: proof_pub.get_subscription_count() > 0,
                     'Memory proof subscriber')
            wait_for(lambda: scenario.map_pub.get_subscription_count() >= 4,
                     'All Nav2 and probe map subscribers')
            scenario.map_pub.publish(scenario.map)
            wait_for(lambda: len(probes) > 0, 'Automatic static-conflict probe')
            # A proof with real ROGMap's short-segment schema drives the *actual*
            # Nav2 master, not an in-process arithmetic model.
            end = time.monotonic()+6
            while time.monotonic() < end and not (
                    received['status'] and received['status']['state'] == 'KNOWN_PASSAGE' and
                    scenario.cost(40,40) < 254):
                send_gate_inputs()
                proof_pub.publish(proof(old_oldest=True))
                scenario.pump(.1)
            assert scenario.cost(40,40) < 254 and scenario.route_exists(), (
                'Remembered segment did not open a global route')
            assert record.exists() and received['status']['segment_count'] == 1
            wait_for(lambda: gate_status['value'] and
                     gate_status['value']['state'] == 'WAIT_VERIFY',
                     'Old constituent voxels may establish history, never entry')
            end = time.monotonic()+6
            while time.monotonic() < end and not (
                    gate_status['value'] and gate_status['value']['state'] == 'ALLOWED' and
                    gate_status['command'] and gate_status['command'].twist.linear.x > 0):
                send_gate_inputs()
                proof_pub.publish(proof())
                scenario.pump(.1)
            assert gate_status['value']['state'] == 'ALLOWED', (
                f'Fresh current segment proof did not allow isolated motion: '
                f'{gate_status["value"]}')

            # Leave the observed area. No new proof is received, but the
            # historical global route must remain available for a return trip.
            robot_x[0] = -2.
            until = time.monotonic()+1.25
            while time.monotonic() < until:
                send_gate_inputs()
                scenario.tick()
            wait_for(lambda: scenario.cost(40,40) < 254 and
                     gate_status['value'] and gate_status['value']['state'] == 'WAIT_VERIFY',
                     'Remote historical route must persist without entry permission')

            shadow.terminate()
            shadow.wait(timeout=3)
            shadow = None
            wait_for(lambda: scenario.cost(40,40) == 254,
                     'Evidence publisher loss must restore wall', seconds=3)
            received['status'] = None
            shadow = start_shadow()
            wait_for(lambda: received['status'] and
                     received['status']['state'] == 'KNOWN_PASSAGE' and
                     scenario.cost(40,40) < 254,
                     'Restarted memory must reopen verified structure')
            # A simple kinematic chassis consumes ONLY the authorized command.
            # It must approach and stop outside the entry before fresh proof.
            previous = time.monotonic()
            approach_deadline = previous+12
            while time.monotonic() < approach_deadline:
                send_gate_inputs()
                scenario.tick()
                current = time.monotonic()
                if (gate_status['value'] and gate_status['value']['state'] == 'WAIT_VERIFY' and
                        gate_status['command']):
                    robot_x[0] += max(0., gate_status['command'].twist.linear.x)*(current-previous)
                previous = current
                if gate_status['value'] and gate_status['value']['reason'] == 'STOP_BEFORE_UNVERIFIED_ENTRY':
                    break
            assert robot_x[0] <= -.35, f'Approach crossed its braking boundary: {robot_x[0]}'
            wait_for(lambda: gate_status['value'] and
                     gate_status['value']['reason'] == 'STOP_BEFORE_UNVERIFIED_ENTRY' and
                     gate_status['command'] and
                     gate_status['command'].twist.linear.x == 0,
                     'History alone must not permit entry after restart')
            proof_pub.publish(proof())
            wait_for(lambda: gate_status['value'] and
                     gate_status['value']['state'] == 'ALLOWED' and
                     gate_status['command'] and
                     gate_status['command'].twist.linear.x > 0,
                     'Re-observation must issue new short-segment permit')

            scenario.dynamic = True
            wait_for(lambda: scenario.cost(40,40) == 254,
                     'Live obstacle must veto historical channel')
            wait_for(lambda: gate_status['value'] and
                     gate_status['value']['state'] == 'BLOCKED' and
                     gate_status['command'].twist.linear.x == 0,
                     'Entry gate must also veto current obstacle')
            scenario.dynamic = False
            wait_for(lambda: scenario.cost(40,40) < 254,
                     'Historical channel returns after transient obstacle clears')

            # Continued short-segment re-observation is required while the
            # vehicle crosses. No old historical record alone drives it.
            proof_pub.publish(proof())
            previous = last_proof = time.monotonic()
            crossing_deadline = previous+12
            while robot_x[0] < .45 and time.monotonic() < crossing_deadline:
                if time.monotonic()-last_proof > .2:
                    proof_pub.publish(proof())
                    last_proof = time.monotonic()
                send_gate_inputs()
                scenario.tick()
                current = time.monotonic()
                if (gate_status['value'] and gate_status['value']['state'] == 'BLOCKED' and
                        gate_status['value']['reason'] != 'STOP_BEFORE_UNVERIFIED_ENTRY'):
                    raise AssertionError(f'Unexpected crossing stop at {robot_x[0]}: '
                                         f'{gate_status["value"]}')
                if gate_status['command']:
                    robot_x[0] += max(0., gate_status['command'].twist.linear.x)*(current-previous)
                previous = current
            assert robot_x[0] >= .45, f'Verified short-segment crossing failed: {robot_x[0]}'

            # Map changes on one recorded cell revoke the associated history.
            scenario.map.data[40*WIDTH+39] = 0
            scenario.map_pub.publish(scenario.map)
            wait_for(lambda: received['status'] and
                     received['status']['state'] == 'SUSPENDED' and
                     scenario.cost(40,40) == 254,
                     'Changed static map patch must restore remaining wall')
            wait_for(lambda: gate_status['value'] and
                     gate_status['value']['reason'] == 'MAP_ASSOCIATION_SUSPENDED' and
                     gate_status['command'].twist.linear.x == 0,
                     'Changed map must stop isolated output')
            assert list(scenario.raw.data) != baseline  # The underlying /map really changed.
            proof_pub.publish(proof())
            wait_for(lambda: received['status'] and
                     received['status']['state'] == 'KNOWN_PASSAGE' and
                     scenario.cost(40,40) < 254,
                     'Fresh proof after map change must establish a new association')
            # A second wall appears beyond the verified entrance. A first
            # segment proof must not be interpreted as whole-tunnel clearance.
            goal_x[0] = 2.
            scenario.map.data[40*WIDTH+50] = 100
            scenario.map.data[40*WIDTH+51] = 100
            scenario.map_pub.publish(scenario.map)
            wait_for(lambda: scenario.cost(50,40) == 254 and
                     gate_status['value'] and
                     gate_status['value']['state'] == 'WAIT_VERIFY',
                     'Second wall must constrain the route before another proof')
            previous = time.monotonic()
            deadline = previous+8
            while time.monotonic() < deadline:
                send_gate_inputs()
                scenario.tick()
                current = time.monotonic()
                if gate_status['command']:
                    robot_x[0] += max(0., gate_status['command'].twist.linear.x)*(current-previous)
                previous = current
                if gate_status['value'] and gate_status['value']['reason'] == (
                        'STOP_BEFORE_UNVERIFIED_STATIC_CELL'):
                    break
            assert robot_x[0] < .8, f'Robot entered the second unverified wall: {robot_x[0]}'
            wait_for(lambda: gate_status['value'] and gate_status['value']['reason'] ==
                     'STOP_BEFORE_UNVERIFIED_STATIC_CELL' and
                     gate_status['command'].twist.linear.x == 0,
                     'Second wall must stop the simulated vehicle')
            wait_for(lambda: probes and probes[-1].pose.position.x > .7,
                     'Progressive observer did not select the second wall')
            end = time.monotonic()+6
            while time.monotonic() < end and not (
                    received['status'] and received['status'].get('segment_count') == 2 and
                    scenario.cost(50,40) < 254):
                send_gate_inputs()
                proof_pub.publish(proof())
                scenario.pump(.1)
            assert received['status']['segment_count'] == 2
            assert scenario.cost(50,40) < 254
            end = time.monotonic()+6
            while time.monotonic() < end and not (
                    gate_status['value'] and gate_status['value']['state'] == 'ALLOWED' and
                    gate_status['command'] and gate_status['command'].twist.linear.x > 0):
                send_gate_inputs()
                proof_pub.publish(proof())
                scenario.pump(.1)
            assert gate_status['value']['state'] == 'ALLOWED', (
                f'Second short proof did not permit travel: {gate_status["value"]}')
            print('PASS: discover -> leave -> restart -> return/stop -> reverify -> '
                  'cross -> stop at second wall -> second proof; map-change revoke/reconfirm, '
                  'real Nav2 costmap and isolated gate',
                  flush=True)
    finally:
        for proc in (shadow, gate, coordinator, harness):
            if proc is not None and proc.poll() is None:
                proc.terminate()
                try:
                    proc.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    proc.kill()
                    proc.wait(timeout=3)
        del status_sub, gate_status_sub, authorized_sub, probe_sub, probe_status_sub, broadcaster
        scenario.node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    run()
