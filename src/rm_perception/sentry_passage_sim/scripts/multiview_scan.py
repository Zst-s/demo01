#!/usr/bin/env python3
"""Explicit opt-in sensor-rig repositioning in isolated lab, never robot control."""
import json
import argparse
import os
import statistics
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from gazebo_msgs.srv import SetEntityState
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--both-sides', action='store_true',
                        help='Also inspect from behind by teleporting the diagnostic sensor rig')
    parser.add_argument('--interior-diagnostic', action='store_true',
                        help='Move only the simulated sensor inside the tunnel to investigate occlusion')
    parser.add_argument('--probe-x', type=float, default=2.6,
                        help='X of fixed straight-volume probe (world coordinates)')
    parser.add_argument('--probe-y', type=float, default=0.0,
                        help='Y of full-body short-volume probe (world coordinates)')
    parser.add_argument('--require-verified', action='store_true',
                        help='Require ROGMap to emit a confirmed short-segment observation')
    options = parser.parse_args()
    domain_id = os.environ.get('ROS_DOMAIN_ID')
    if domain_id not in {str(value) for value in range(219, 240)} or \
            os.environ.get('ROS_LOCALHOST_ONLY') != '1':
        raise SystemExit('Refusing: use isolated ROS_DOMAIN_ID 219..239 and ROS_LOCALHOST_ONLY=1')
    rclpy.init()
    node = Node('passage_multiview_scan', parameter_overrides=[
        rclpy.parameter.Parameter('use_sim_time', value=True)])
    data = {}
    evidence = []
    subscriptions = [
        node.create_subscription(Odometry, '/passage_sim/sensor_odom',
                                 lambda m: data.update(odom=m), qos_profile_sensor_data),
        node.create_subscription(String, '/terrain/clearance/status',
                                 lambda m: data.update(status=dict(
                                     w.split('=', 1) for w in m.data.split() if '=' in w)), 1),
        node.create_subscription(String, '/terrain/clearance/verified_segment',
                                 lambda m: evidence.append(json.loads(m.data)), 10)]
    publisher = node.create_publisher(PoseStamped, '/terrain/clearance/probe', 1)
    client = node.create_client(SetEntityState, '/passage_sim/set_entity_state')

    def wait(predicate, seconds=30):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.05)
            if predicate():
                return
        raise RuntimeError(f'Timed out waiting for lab data; status={data.get("status")}')

    def move(x, y):
        req = SetEntityState.Request()
        req.state.name = 'sensor_rig'
        req.state.reference_frame = 'world'
        req.state.pose.position.x = float(x)
        req.state.pose.position.y = float(y)
        req.state.pose.orientation.w = 1.0
        future = client.call_async(req)
        wait(future.done, 10)
        if not future.result().success:
            raise RuntimeError('Gazebo rejected sensor_rig reposition')
        wait(lambda: abs(data['odom'].pose.pose.position.x-x) < .02 and
             abs(data['odom'].pose.pose.position.y-y) < .02, 10)

    original = None
    try:
        wait(lambda: 'odom' in data and 'status' in data and
             publisher.get_subscription_count() > 0 and client.service_is_ready() and
             node.get_clock().now().nanoseconds > 0)
        if any('cmd_vel' in name for name, _ in node.get_topic_names_and_types()):
            raise RuntimeError('Unexpected motion topic in lab domain; refusing to move sensor')
        odom = data['odom']
        q = odom.pose.pose.orientation
        if odom.header.frame_id != 'world' or abs(odom.pose.pose.position.z-.55) > .02 or (
                abs(q.x)+abs(q.y)+abs(q.z) > .01):
            raise RuntimeError('Unexpected rig pose; requires original upright laboratory rig')
        original = (odom.pose.pose.position.x, odom.pose.pose.position.y)
        probe = PoseStamped()
        probe.header.frame_id = 'world'
        probe.header.stamp = node.get_clock().now().to_msg()
        probe.pose.position.x = options.probe_x
        probe.pose.position.y = options.probe_y
        # Calibrated height of direct floor returns in the lab's voxel map.
        probe.pose.position.z = -.025
        probe.pose.orientation.w = 1.0
        previous_revision = int(data['status'].get('revision', '0'))
        publisher.publish(probe)
        wait(lambda: abs(float(data['status'].get('probe_x', 'nan'))-options.probe_x) < .001 and
             abs(float(data['status'].get('probe_y', 'nan'))-options.probe_y) < .001 and
             int(data['status'].get('revision', '0')) > previous_revision and
             'body_unknown' in data['status'])
        revision = data['status']['revision']
        results = []
        fields = ('body_unknown', 'support_missing', 'support_unknown', 'support_free',
                  'support_stale', 'support_outside', 'blocked', 'stale', 'unknown')
        # All viewpoints remain outside the entrance x=2.0. No crossing or chassis motion.
        views = [('far', 0., 0.), ('near', 1.2, 0.),
                 ('left', 1.2, .45), ('right', 1.2, -.45)]
        if options.interior_diagnostic:
            views += [('inside_front', 2.2, 0.), ('inside_center', 2.6, 0.),
                      ('inside_rear', 3.0, 0.)]
        if options.both_sides:
            views += [('far_side', 3.8, 0.), ('far_side_left', 3.8, .45),
                      ('far_side_right', 3.8, -.45)]
        views.append(('return', 0., 0.))
        for name, x, y in views:
            move(x, y)
            start = node.get_clock().now().nanoseconds / 1e9
            samples = []
            seen = set()
            deadline = time.monotonic() + 45
            while (node.get_clock().now().nanoseconds/1e9 - start < 4.0 or len(samples) < 5):
                if time.monotonic() > deadline:
                    raise RuntimeError(f'Insufficient fresh observations at {name}: {data.get("status")}')
                rclpy.spin_once(node, timeout_sec=.05)
                s = data['status']
                if s.get('revision') != revision:
                    raise RuntimeError('Probe changed during comparison; do not click Publish Point')
                stamp = float(s.get('stamp', '0'))
                # Discard transition frames: require two seconds settled, recent input, unique samples.
                if stamp >= start+2 and stamp not in seen and int(s.get('checked', '0')) > 0 and (
                        0 <= float(s.get('input_age', 'inf')) <= .5):
                    seen.add(stamp)
                    samples.append({f: int(s[f]) for f in fields})
            if len(samples) < 3:
                raise RuntimeError(f'{name}: insufficient fresh samples ({len(samples)})')
            row = dict(view=name, x=x, y=y, samples=len(samples),
                       median={f: statistics.median(s[f] for s in samples) for f in fields})
            results.append(row)
            print(json.dumps(row), flush=True)
        baseline = results[0]['median']
        if options.require_verified:
            confirmed = [item for item in evidence if item.get('schema') == 1 and
                         item.get('kind') == 'verified_short_segment_observation' and
                         item.get('frame') == 'world' and
                         abs(item.get('probe_x', -100) - options.probe_x) < .001 and
                         abs(item.get('probe_y', -100) - options.probe_y) < .001 and
                         item.get('confirmed_frames', 0) >= 3 and
                         item.get('valid_until', 0) >= item.get('measured_at', 1)]
            if not confirmed:
                raise RuntimeError('No verified short-segment observation from live ROGMap')
            print(json.dumps(dict(verified_segment=confirmed[-1])), flush=True)
        print(json.dumps(dict(comparison=[dict(view=r['view'],
            body_unknown_delta=r['median']['body_unknown']-baseline['body_unknown'],
            support_missing_delta=r['median']['support_missing']-baseline['support_missing'])
            for r in results], note='Current fresh evidence, not permanent accumulation; no entry permission'),
            indent=2), flush=True)
    finally:
        if original is not None and rclpy.ok():
            try:
                move(*original)
                print('Sensor rig restored to original XY', flush=True)
            except Exception as exc:
                print(f'WARNING: could not restore simulated sensor rig: {exc}', flush=True)
        del subscriptions
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
