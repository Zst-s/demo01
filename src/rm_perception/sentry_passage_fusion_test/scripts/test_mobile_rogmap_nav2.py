#!/usr/bin/env python3
"""One isolated run: moving Gazebo lidar -> ROGMap -> Nav2 -> entry gate.

No real chassis, serial port, production /cmd_vel, or synthetic passage proof.
The only artificial input is a deliberately wrong static wall in the test map.
Failures are retained with logs and a stage result for user inspection.
"""
import argparse
from collections import Counter
import copy
import json
import math
import os
from pathlib import Path
import statistics
import signal
import subprocess
import sys
import time

import rclpy
import yaml
from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist, TwistStamped
from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState
from nav2_msgs.action import ComputePathToPose, FollowPath
from nav2_msgs.msg import Costmap
from nav_msgs.msg import OccupancyGrid, Odometry, Path as NavPath
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String
from visualization_msgs.msg import MarkerArray
from tf2_ros import StaticTransformBroadcaster


def stop(process, interrupt=False):
    if process is None or process.poll() is not None:
        return
    process.send_signal(signal.SIGINT if interrupt else signal.SIGTERM)
    try:
        process.wait(timeout=8)
    except subprocess.TimeoutExpired:
        process.kill()
        process.wait(timeout=3)


def run():
    domain_id = os.environ.get('ROS_DOMAIN_ID')
    if (domain_id not in {str(value) for value in range(219, 240)} or
            os.environ.get('ROS_LOCALHOST_ONLY') != '1'):
        raise RuntimeError('Isolated ROS_DOMAIN_ID in 219..239 and '
                           'ROS_LOCALHOST_ONLY=1 required')
    parser = argparse.ArgumentParser()
    parser.add_argument('--artifacts-dir', required=True, type=Path)
    parser.add_argument('--approach-timeout', type=float, default=120.)
    args = parser.parse_args()
    args.artifacts_dir.mkdir(parents=True, exist_ok=False)
    directory = args.artifacts_dir
    root = Path(__file__).resolve().parents[1]
    lab_share = Path(get_package_share_directory('sentry_passage_sim'))
    processes = []
    stage = 'START'
    rclpy.init()
    node = Node('mobile_rogmap_nav2_check', parameter_overrides=[
        Parameter('use_sim_time', value=True)])
    latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
    map_pub = node.create_publisher(OccupancyGrid, '/passage_fusion_test/map', latched)
    local_pub = node.create_publisher(
        OccupancyGrid, '/passage_fusion_test/controller_fixture_map', latched)
    obstacle_pub = node.create_publisher(Bool, '/passage_fusion_test/live_obstacle', 10)
    path_pub = node.create_publisher(NavPath, '/passage_fusion_test/planned_path', 10)
    probe_pub = node.create_publisher(PoseStamped, '/terrain/clearance/probe', 10)
    state = dict(truth=None, odom=None, memory=None, gate=None, cost=None,
                 terrain=None, proof=[], permit=[], auth=None, raw=None,
                 clearance_status=None, clearance_samples=[], drive=None,
                 clearance_markers=None,
                 view_probe_diagnostics=[],
                 gate_reasons=Counter(), raw_nonzero=0,
                 auth_nonzero=0, drive_nonzero=0)
    def on_gate(msg):
        status = json.loads(msg.data)
        state['gate'] = status
        state['gate_reasons'][status['reason']] += 1
    def on_raw(msg):
        state['raw'] = msg
        state['raw_nonzero'] += abs(msg.linear.x) > .01
    def on_auth(msg):
        state['auth'] = msg
        state['auth_nonzero'] += abs(msg.twist.linear.x) > .01
    def on_drive(msg):
        state['drive'] = msg
        state['drive_nonzero'] += abs(msg.linear.x) > .01
    subscriptions = [
        node.create_subscription(Odometry, '/passage_sim/sensor_odom',
                                 lambda msg: state.update(truth=msg), 10),
        node.create_subscription(Odometry, '/passage_fusion_test/odom_map',
                                 lambda msg: state.update(odom=msg), 10),
        node.create_subscription(String, '/passage/memory_shadow_status',
                                 lambda msg: state.update(memory=json.loads(msg.data)), 10),
        node.create_subscription(String, '/passage_fusion_test/entry_gate_status',
                                 on_gate, 10),
        node.create_subscription(Costmap, '/passage_fusion_test/global_costmap/costmap_raw',
                                 lambda msg: state.update(cost=msg), 10),
        node.create_subscription(OccupancyGrid, '/terrain/grid',
                                 lambda msg: state.update(terrain=msg), 10),
        node.create_subscription(String, '/terrain/clearance/status',
                                 lambda msg: (state.update(clearance_status=msg.data),
                                              state['clearance_samples'].append(msg.data)), 10),
        node.create_subscription(MarkerArray, '/terrain/clearance/markers',
                                 lambda msg: state.update(clearance_markers=msg), 10),
        node.create_subscription(String, '/terrain/clearance/verified_segment',
                                 lambda msg: state['proof'].append(json.loads(msg.data)), 10),
        node.create_subscription(String, '/passage_fusion_test/fresh_permit',
                                 lambda msg: state['permit'].append(json.loads(msg.data)), 10),
        node.create_subscription(TwistStamped, '/passage_fusion_test/cmd_vel_authorized',
                                 on_auth, 10),
        node.create_subscription(Twist, '/passage_fusion_test/controller_cmd_vel_raw',
                                 on_raw, 10),
        node.create_subscription(Twist, '/passage_sim/drive_cmd_vel', on_drive, 10)]
    static_tf = StaticTransformBroadcaster(node)
    plan_action = ActionClient(node, ComputePathToPose,
                               '/passage_fusion_test/compute_path_to_pose')
    follow_action = ActionClient(node, FollowPath, '/passage_fusion_test/follow_path')
    planner_lifecycle = node.create_client(
        ChangeState, '/passage_fusion_test/planner_server/change_state')
    controller_lifecycle = node.create_client(
        ChangeState, '/passage_fusion_test/controller_server/change_state')
    grid = OccupancyGrid()
    grid.header.frame_id = 'map'
    grid.info.width = grid.info.height = 80
    grid.info.resolution = .1
    grid.info.origin.position.x = grid.info.origin.position.y = -4.
    grid.info.origin.orientation.w = 1.
    grid.data = [0]*6400
    for row in range(80):
        for col in (57, 58, 66, 67):
            grid.data[row*80+col] = 100
    local_map = copy.deepcopy(grid)
    local_map.data = [0]*6400
    route = None
    obstacle = False
    last_pub = 0.
    last_diag = 0.

    def start(name, command):
        log_path = directory/f'{name}.log'
        with log_path.open('w', encoding='utf-8') as stream:
            process = subprocess.Popen(command, stdout=stream, stderr=subprocess.STDOUT)
        processes.append((name, process, log_path))
        return process

    def fail_context():
        costs = None
        if state['cost'] is not None and len(state['cost'].data) == 6400:
            costs = list(state['cost'].data[40*80+38:40*80+59])
        terrain_samples = None
        terrain = state['terrain']
        if terrain is not None:
            terrain_samples = {}
            for x in (0., .5, 1.2, 1.7, 2.2):
                ix = int((x-terrain.info.origin.position.x)/terrain.info.resolution)
                iy = int((0.-terrain.info.origin.position.y)/terrain.info.resolution)
                terrain_samples[str(x)] = (terrain.data[iy*terrain.info.width+ix]
                    if 0 <= ix < terrain.info.width and 0 <= iy < terrain.info.height
                    else 'OUTSIDE')
        return dict(stage=stage,
                    pose=None if state['odom'] is None else [
                        state['odom'].pose.pose.position.x, state['odom'].pose.pose.position.y,
                        state['odom'].pose.pose.position.z],
                    truth_pose=None if state['truth'] is None else [
                        state['truth'].pose.pose.position.x,
                        state['truth'].pose.pose.position.y,
                        state['truth'].pose.pose.position.z],
                    odom_age=None if state['odom'] is None else
                        node.get_clock().now().nanoseconds/1e9 - (
                            state['odom'].header.stamp.sec+
                            state['odom'].header.stamp.nanosec*1e-9),
                    memory=state['memory'], gate=state['gate'],
                    clearance_status=state['clearance_status'],
                    support_missing_positions=[] if state['clearance_markers'] is None else [
                        [round(point.x, 3), round(point.y, 3), round(point.z, 3)]
                        for marker in state['clearance_markers'].markers
                        if marker.ns == 'support_missing_magenta' and marker.action == 0
                        for point in marker.points],
                    proof_count=len(state['proof']), permit_count=len(state['permit']),
                    view_probe_diagnostics=state['view_probe_diagnostics'],
                    wall_cost=None if state['cost'] is None else [
                        state['cost'].data[40*80+57], state['cost'].data[40*80+66]],
                    terrain_frame=None if state['terrain'] is None else
                        state['terrain'].header.frame_id,
                    x_costs_from_zero_to_first_wall=costs,
                    terrain_samples=terrain_samples,
                    raw=None if state['raw'] is None else [state['raw'].linear.x,
                        state['raw'].linear.y, state['raw'].angular.z],
                    auth=None if state['auth'] is None else [state['auth'].twist.linear.x,
                        state['auth'].twist.linear.y, state['auth'].twist.angular.z],
                    drive=None if state['drive'] is None else [state['drive'].linear.x,
                        state['drive'].linear.y, state['drive'].angular.z],
                    gate_reasons=dict(state['gate_reasons']),
                    nonzero_counts={key:state[key] for key in (
                        'raw_nonzero','auth_nonzero','drive_nonzero')},
                    process_exits={name: p.returncode for name, p, _ in processes
                                   if p.poll() is not None})

    def tick():
        nonlocal last_pub, last_diag
        for name, process, log_path in processes:
            if process.poll() is not None:
                tail = log_path.read_text(encoding='utf-8')[-2000:]
                raise RuntimeError(f'{name} exited {process.returncode}: {tail}')
        now = time.monotonic()
        if now-last_pub > .08 and node.get_clock().now().nanoseconds > 0:
            last_pub = now
            obstacle_pub.publish(Bool(data=obstacle))
            if route is not None and state['odom'] is not None:
                anchor = state['odom'].pose.pose.position
                path = NavPath()
                path.header.frame_id = 'map'
                path.header.stamp = node.get_clock().now().to_msg()
                first = PoseStamped()
                first.header = path.header
                first.pose.position.x = anchor.x
                first.pose.position.y = anchor.y
                first.pose.orientation.w = 1.
                path.poses.append(first)
                # The Nav2 action path is anchored at the pose where planning
                # began. Re-publishing it verbatim after the robot moves makes
                # the safety gate see a path that doubles back behind the
                # current pose (and eventually trips PATH_START / stale-path
                # rejection). Keep only the untraversed suffix, starting at
                # the route pose nearest the live odometry pose.
                if route.poses:
                    nearest = min(
                        range(len(route.poses)),
                        key=lambda i: math.hypot(
                            route.poses[i].pose.position.x-anchor.x,
                            route.poses[i].pose.position.y-anchor.y))
                    path.poses.extend(copy.deepcopy(route.poses[nearest:]))
                path_pub.publish(path)
        rclpy.spin_once(node, timeout_sec=.04)
        if now-last_diag > 8. and state['odom'] is not None:
            last_diag = now
            x = state['odom'].pose.pose.position.x
            print(f'DIAG stage={stage} x={x:.3f} truth={fail_context()["truth_pose"]} '
                  f'odom_age={fail_context()["odom_age"]:.2f} raw={fail_context()["raw"]} '
                  f'auth={fail_context()["auth"]} drive={fail_context()["drive"]} '
                  f'gate={state["gate"]}', flush=True)

    def wait(predicate, label, seconds=20.):
        end = time.monotonic()+seconds
        while time.monotonic() < end:
            tick()
            if predicate():
                print(f'PASS {label}', flush=True)
                return
        raise AssertionError(f'{label}: {json.dumps(fail_context(), default=str)}')

    def transition(client, code):
        req = ChangeState.Request()
        req.transition.id = code
        future = client.call_async(req)
        wait(future.done, f'lifecycle {code}', 20.)
        if not future.result().success:
            raise RuntimeError(f'lifecycle {code} returned failure')

    def plan(x, y=0.):
        start_pose = state['odom'].pose.pose.position
        goal = ComputePathToPose.Goal()
        goal.use_start = True
        for pose, px, py in ((goal.start, start_pose.x, start_pose.y),
                             (goal.goal, x, y)):
            pose.header.frame_id = 'map'
            pose.header.stamp = node.get_clock().now().to_msg()
            pose.pose.position.x = px
            pose.pose.position.y = py
            pose.pose.orientation.w = 1.
        goal.planner_id = 'GridBased'
        sent = plan_action.send_goal_async(goal)
        wait(sent.done, 'planner accepted request', 15.)
        handle = sent.result()
        if not handle.accepted:
            raise RuntimeError('planner rejected request')
        result = handle.get_result_async()
        wait(result.done, 'planner responded', 20.)
        return result.result().status, result.result().result.path

    def follow(path, x, y=0., timeout=45.):
        nonlocal route
        route = path
        goal = FollowPath.Goal()
        goal.path = path
        goal.controller_id = 'FollowPath'
        goal.goal_checker_id = 'goal_checker'
        sent = follow_action.send_goal_async(goal)
        wait(sent.done, 'controller accepted path', 15.)
        handle = sent.result()
        if not handle.accepted:
            raise RuntimeError('controller rejected path')
        outcome = handle.get_result_async()
        try:
            def arrived():
                if outcome.done():
                    raise RuntimeError(f'FollowPath ended before arrival: '
                                       f'{outcome.result().status}')
                return state['odom'] is not None and math.hypot(
                    state['odom'].pose.pose.position.x-x,
                    state['odom'].pose.pose.position.y-y) < .12
            wait(arrived, f'actual vehicle arrived ({x:.2f},{y:.2f})', timeout)
        finally:
            cancellation = handle.cancel_goal_async()
            wait(cancellation.done, 'controller goal cancelled', 6.)
            route = None

    def status_fields(raw):
        return dict(part.split('=', 1) for part in raw.split() if '=' in part)

    def observe_view(label, x, y):
        status, view_path = plan(x, y)
        if status != GoalStatus.STATUS_SUCCEEDED:
            raise RuntimeError(f'Cannot navigate to {label} view: planner status={status}')
        points = [(round(p.pose.position.x, 2), round(p.pose.position.y, 2))
                  for p in view_path.poses]
        print(f'VIEW_PATH {label} count={len(points)} '
              f'points={points[:8]} ... {points[-4:]}', flush=True)
        follow(view_path, x, y, timeout=35.)
        state['clearance_samples'].clear()
        deadline = time.monotonic()+6.
        while time.monotonic() < deadline:
            tick()
        selected_probe = status_fields(state['clearance_status'])
        probe_x = float(selected_probe.get('probe_x', 'nan'))
        probe_y = float(selected_probe.get('probe_y', 'nan'))
        samples = []
        for raw in state['clearance_samples']:
            fields = status_fields(raw)
            try:
                matches_probe = (abs(float(fields.get('probe_x', 'nan'))-probe_x) < 1e-6 and
                                 abs(float(fields.get('probe_y', 'nan'))-probe_y) < 1e-6)
            except ValueError:
                matches_probe = False
            if matches_probe:
                try:
                    samples.append({key: int(fields[key].split('/')[0]) for key in (
                        'body_unknown', 'body_recent', 'support_missing',
                        'support_unknown', 'support_free', 'support_stale',
                                'support_outside', 'checked', 'confirmed')} | {
                                key: float(fields[key]) for key in (
                                    'support_oldest_age', 'support_newest_age')})
                except (KeyError, ValueError):
                    continue
        if len(samples) < 2:
            raise RuntimeError(f'{label}: no repeated current clearance observations: '
                               f'{state["clearance_status"]}')
        summary = {key: statistics.median(sample[key] for sample in samples)
                   for key in samples[0]} | {'samples': len(samples), 'x': x, 'y': y}
        if state['proof']:
            latest = state['proof'][-1]
            summary['body_slices'] = latest.get('recent_body_by_slice')
            summary['support_slices'] = latest.get('recent_support_by_slice')
            summary['proof_probe'] = [latest.get('probe_x'), latest.get('probe_y')]
        return summary

    def probe_segment(x, y):
        previous_revision = int(status_fields(state['clearance_status']).get('revision', '0'))
        probe = PoseStamped()
        probe.header.frame_id = 'world'
        probe.header.stamp = node.get_clock().now().to_msg()
        probe.pose.position.x = x
        probe.pose.position.y = y
        # In this Gazebo ray-sensor pipeline, measured floor returns land in
        # the z=-1 voxel at 0.1 m resolution; keep the calibrated lab height.
        probe.pose.position.z = -.025
        probe.pose.orientation.w = 1.
        probe_pub.publish(probe)
        wait(lambda: state['clearance_status'] is not None and
             abs(float(status_fields(state['clearance_status']).get('probe_x', 'nan'))-x) < 1e-6 and
             abs(float(status_fields(state['clearance_status']).get('probe_y', 'nan'))-y) < 1e-6 and
             int(status_fields(state['clearance_status']).get('revision', '0')) >
             previous_revision,
             f'ROGMap observes strip ({x:.2f},{y:.2f})', 12.)
        return state['clearance_status']

    def verify_segment(x, y, timeout=10.):
        previous = max((p.get('measured_at', 0.) for p in state['proof']
                        if abs(p.get('probe_x', -100.)-x) < 1e-6 and
                        abs(p.get('probe_y', -100.)-y) < 1e-6), default=0.)
        probe_segment(x, y)
        wait(lambda: any(
            abs(p.get('probe_x', -100.)-x) < 1e-6 and
            abs(p.get('probe_y', -100.)-y) < 1e-6 and
            p.get('measured_at', 0.) > previous for p in state['proof']),
            f'ROGMap confirms fresh swept volume ({x:.2f},{y:.2f})', timeout)

    def diagnose_probe_from_current_view(label, x, y, seconds=4.):
        previous = max((p.get('measured_at', 0.) for p in state['proof']
                        if abs(p.get('probe_x', -100.)-x) < 1e-6 and
                        abs(p.get('probe_y', -100.)-y) < 1e-6), default=0.)
        previous_permits = len(state['permit'])
        probe_segment(x, y)
        deadline = time.monotonic()+seconds
        while time.monotonic() < deadline:
            tick()
        matching = [p for p in state['proof']
                    if abs(p.get('probe_x', -100.)-x) < 1e-6 and
                    abs(p.get('probe_y', -100.)-y) < 1e-6 and
                    p.get('measured_at', 0.) > previous]
        fields = status_fields(state['clearance_status'] or '')
        result = {'view': label, 'probe': [x, y], 'proof_created': bool(matching),
                  'permit_created': len(state['permit']) > previous_permits,
                  'reason': fields.get('reason'),
                  'body_unknown': int(fields.get('body_unknown', 0)),
                  'support_missing': int(fields.get('support_missing', 0)),
                  'support_unknown': int(fields.get('support_unknown', 0)),
                  'support_free': int(fields.get('support_free', 0)),
                  'support_stale': int(fields.get('support_stale', 0)),
                  'support_oldest_age': float(fields.get('support_oldest_age', 'inf')),
                  'support_newest_age': float(fields.get('support_newest_age', 'inf'))}
        if matching:
            result['body_recent_by_slice'] = matching[-1].get('recent_body_by_slice')
            result['support_recent_by_slice'] = matching[-1].get('recent_support_by_slice')
        state['view_probe_diagnostics'].append(result)
        print(f'VIEW_PROBE {json.dumps(result, sort_keys=True)}', flush=True)
        return result

    try:
        stage = 'LIVE_PERCEPTION_START'
        start('lab', ['ros2', 'launch', 'sentry_passage_sim',
                      'perception_lab.launch.py', 'gui:=false', 'rviz:=false',
                      'mobile:=true',
                      f'world_file:={lab_share}/worlds/passages_mobile_dense.world',
                      f'config_file:={lab_share}/config/rog_map_dense_lab.yaml'])
        wait(lambda: node.get_clock().now().nanoseconds > 0 and state['truth'] is not None
             and state['terrain'] is not None, 'mobile Gazebo + actual terrain', 80.)
        start('bridge', [sys.executable, str(root/'scripts/mobile_lab_bridge.py'),
                         '--ros-args', '-r', '__ns:=/passage_fusion_test',
                         '-p', 'use_sim_time:=true'])
        wait(lambda: state['odom'] is not None, 'Gazebo truth to gated odom/TF', 15.)
        grid.header.stamp = node.get_clock().now().to_msg()
        grid.info.map_load_time = grid.header.stamp
        local_map.header.stamp = grid.header.stamp
        local_map.info.map_load_time = grid.info.map_load_time
        transforms = []
        for child in ('world', 'odom'):
            transform = TransformStamped()
            transform.header.frame_id = 'map'
            transform.header.stamp = grid.header.stamp
            transform.child_frame_id = child
            transform.transform.rotation.w = 1.
            transforms.append(transform)
        static_tf.sendTransform(transforms)
        # These generated parameters belong only to this retained experiment.
        planner_params = yaml.safe_load((root/'config/isolated_planner.yaml').read_text())
        for value in planner_params.values():
            value['ros__parameters']['use_sim_time'] = True
        planner_params['/passage_fusion_test/global_costmap/global_costmap'][
            'ros__parameters']['terrain_layer']['topic'] = '/terrain/grid'
        planner_yaml = directory/'mobile_planner.yaml'
        planner_yaml.write_text(yaml.safe_dump(planner_params))
        controller_params = yaml.safe_load((root/'config/isolated_controller.yaml').read_text())
        for value in controller_params.values():
            value['ros__parameters']['use_sim_time'] = True
        controller_yaml = directory/'mobile_controller.yaml'
        controller_yaml.write_text(yaml.safe_dump(controller_params))
        start('planner', ['ros2', 'run', 'nav2_planner', 'planner_server',
                          '--ros-args', '-r', '__ns:=/passage_fusion_test',
                          '--params-file', str(planner_yaml)])
        start('controller', ['ros2', 'run', 'nav2_controller', 'controller_server',
                             '--ros-args', '-r', '__ns:=/passage_fusion_test',
                             '-r', 'cmd_vel:=/passage_fusion_test/controller_cmd_vel_raw',
                             '--params-file', str(controller_yaml)])
        start('memory', [sys.executable, str(root/'scripts/live_memory_shadow.py'),
                         '--ros-args', '-r', '__ns:=/passage_fusion_test',
                         '-r', '/map:=/passage_fusion_test/map',
                         '-p', 'use_sim_time:=true',
                         '-p', 'map_session_id:=mobile-rogmap-lab',
                         '-p', f'record_path:={directory}/memory.json',
                         '-p', 'evidence_frame:=world', '-p', 'publish_test_mask:=true'])
        start('adapter', [sys.executable, str(root/'scripts/controller_command_adapter.py'),
                          '--ros-args', '-r', '__ns:=/passage_fusion_test',
                          '-p', 'use_sim_time:=true'])
        start('gate', [sys.executable, str(root/'scripts/isolated_entry_gate.py'),
                       '--ros-args', '-r', '__ns:=/passage_fusion_test',
                       '-p', 'use_sim_time:=true'])
        wait(lambda: planner_lifecycle.service_is_ready() and
             controller_lifecycle.service_is_ready(), 'real Nav2 servers started', 30.)
        transition(planner_lifecycle, Transition.TRANSITION_CONFIGURE)
        transition(controller_lifecycle, Transition.TRANSITION_CONFIGURE)
        map_pub.publish(grid)
        local_pub.publish(local_map)
        transition(planner_lifecycle, Transition.TRANSITION_ACTIVATE)
        transition(controller_lifecycle, Transition.TRANSITION_ACTIVATE)
        wait(lambda: plan_action.server_is_ready() and follow_action.server_is_ready() and
             state['memory'] is not None and state['memory']['map_associated'] and
             state['cost'] is not None and state['gate'] is not None,
             'planning + memory + gate all live', 30.)
        if any(name == '/cmd_vel' for name, _ in node.get_topic_names_and_types()):
            raise RuntimeError('Unexpected production /cmd_vel in isolated domain')
        stage = 'APPROACH_UNDISCOVERED'
        status, approach = plan(1.2)
        if status != GoalStatus.STATUS_SUCCEEDED:
            raise AssertionError(f'Cannot plan initial free-space approach: status={status}')
        follow(approach, 1.2, timeout=args.approach_timeout)
        if state['memory']['state'] != 'UNKNOWN':
            raise AssertionError('Passage discovered before controlled probe')
        stage = 'ACTUAL_ROGMAP_PROOF'
        probe = PoseStamped()
        probe.header.frame_id = 'world'
        probe.header.stamp = node.get_clock().now().to_msg()
        probe.pose.position.x = 1.7
        probe.pose.position.y = 0.
        probe.pose.position.z = -.025
        probe.pose.orientation.w = 1.
        probe_pub.publish(probe)
        wait(lambda: state['clearance_status'] is not None and
             status_fields(state['clearance_status']).get('probe_x') == '1.7',
             'ROGMap begins observing selected short segment', 15.)
        views = {'center': observe_view('center', 1.1, 0.)}
        print(f'VIEW center {views["center"]}', flush=True)
        print(f'PROBE center {probe_segment(1.7, 0.0)}', flush=True)
        # Keep both side-view poses fully outside the passage's swept vehicle
        # envelope. The entry manager must be free to stop a lateral path that
        # cuts across the side of the tunnel, even when that path is intended
        # only for sensing. Move back from the opening before shifting laterally.
        views['left'] = observe_view('left', 1.0, .60)
        print(f'VIEW left {views["left"]}', flush=True)
        print(f'PROBE left_strip {probe_segment(1.7, .20)}', flush=True)
        views['right'] = observe_view('right', 1.0, -.60)
        print(f'VIEW right {views["right"]}', flush=True)
        print(f'PROBE right_strip {probe_segment(1.7, -.20)}', flush=True)
        # Extra distant scans increase lower-body coverage without driving the
        # chassis into the candidate passage's swept envelope.
        views['far_left'] = observe_view('far_left', .50, .60)
        print(f'VIEW far_left {views["far_left"]}', flush=True)
        diagnose_probe_from_current_view('far_left', 1.7, .20)
        views['far_right'] = observe_view('far_right', .50, -.60)
        print(f'VIEW far_right {views["far_right"]}', flush=True)
        diagnose_probe_from_current_view('far_right', 1.7, -.20)
        # A straight-on far stand-off view complements side views without
        # driving into the tunnel. Real sensors often have asymmetric
        # occlusion; only the measured leading slices may renew entry.
        views['far_center'] = observe_view('far_center', .50, 0.0)
        print(f'VIEW far_center {views["far_center"]}', flush=True)
        diagnose_probe_from_current_view('far_center', 1.7, .20)
        diagnose_probe_from_current_view('far_center', 1.7, -.20)
        # A second, farther observation changes the lidar depression angle to
        # the floor near the tunnel lip; this tests the real near-field blind
        # zone without granting any motion through the opening.
        views['far_back'] = observe_view('far_back', 0.0, 0.0)
        print(f'VIEW far_back {views["far_back"]}', flush=True)
        diagnose_probe_from_current_view('far_back', 1.7, .20)
        diagnose_probe_from_current_view('far_back', 1.7, -.20)
        views['far_back_left'] = observe_view('far_back_left', 0.0, .40)
        print(f'VIEW far_back_left {views["far_back_left"]}', flush=True)
        diagnose_probe_from_current_view('far_back_left', 1.7, .20)
        views['far_back_right'] = observe_view('far_back_right', 0.0, -.40)
        print(f'VIEW far_back_right {views["far_back_right"]}', flush=True)
        diagnose_probe_from_current_view('far_back_right', 1.7, -.20)
        # Two overlapping body-width probes form a measured corridor. Their
        # union is derived from two real ROGMap validations, not an inflated
        # width parameter. Refresh both from the close pre-entry pose; the
        # sensor may still fail the strict test if its lower-body/support
        # evidence is occluded, in which case the gate must remain closed.
        views['center_refresh'] = observe_view('center_refresh', 1.2, 0.)
        print(f'VIEW center_refresh {views["center_refresh"]}', flush=True)
        verify_segment(1.7, .20)
        verify_segment(1.7, -.20)
        wait(lambda: state['memory'] is not None and
             state['memory'].get('segment_count', 0) >= 2,
             'two overlapping independently observed body-width strips are remembered', 8.)
        if state['memory']['state'] != 'KNOWN_PASSAGE':
            raise AssertionError('No verified ROGMap segment after real multi-view scan; '
                                 f'views={views}; status={state["clearance_status"]}')
        stage = 'PLANNER_USES_REAL_PROOF'
        wait(lambda: state['cost'].data[40*80+57] < 253 and
             state['cost'].data[40*80+58] < 253,
             'overlapping real proof strips clear the static wall in Nav2 costmap', 12.)
        status, crossing = plan(2.2)
        if status != GoalStatus.STATUS_SUCCEEDED:
            raise AssertionError(f'Nav2 cannot plan discovered short crossing: {status}')
        stage = 'ENTRY_GATE_WITH_REAL_PROOF'
        follow(crossing, 2.2, timeout=45.)
        result = dict(passed=True, stage='FIRST_ACTUAL_CROSSING',
                      pose=fail_context()['pose'], actual_proofs=len(state['proof']),
                      actual_permits=len(state['permit']))
        (directory/'result.json').write_text(json.dumps(result, indent=2))
        print('PASS actual Gazebo robot crossed first verified segment via real Nav2 and gate',
              flush=True)
    except BaseException as exc:
        result = dict(passed=False, error=str(exc), **fail_context())
        (directory/'result.json').write_text(json.dumps(result, indent=2, default=str))
        raise
    finally:
        for name, process, _ in reversed(processes):
            stop(process, interrupt=name == 'lab')
        del subscriptions, static_tf, plan_action, follow_action
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    run()
