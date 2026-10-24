#!/usr/bin/env python3
"""Replay measured lab ROGMap memory into Nav2 and its isolated entry gate.

The input must be a retained successful test_live_rogmap_fusion lateral run.
This creates no verified_segment or fresh_permit publisher and never actuates
a robot. The static map is the exact fixed two-wall fixture from that run.
"""
import argparse
import copy
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, TransformStamped, TwistStamped
from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState
from nav2_msgs.action import ComputePathToPose
from nav2_msgs.msg import Costmap
from nav_msgs.msg import OccupancyGrid, Odometry, Path as NavPath
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String
from tf2_ros import StaticTransformBroadcaster

from global_passage_memory import MapContext
from passage_memory_v2 import PassageMemoryV2


def run():
    parser = argparse.ArgumentParser()
    parser.add_argument('record_directory', type=Path)
    args = parser.parse_args()
    if os.environ.get('ROS_DOMAIN_ID') != '224' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
        raise RuntimeError('Requires isolated ROS_DOMAIN_ID=224 ROS_LOCALHOST_ONLY=1')
    source_result = json.loads((args.record_directory/'result.json').read_text())
    record = json.loads((args.record_directory/'memory.json').read_text())
    context = MapContext(**record['body']['context'])
    if (not source_result.get('passed') or
            not all(source_result.get('lateral_diagnostics', {}).get(str(y), {}).get('verified')
                    for y in (-.25, .25)) or
            context.epoch != 'live-rogmap-lab-session' or context.width != 80 or
            context.height != 80 or context.origin_x != -4. or context.origin_y != -4.):
        raise RuntimeError('A successful measured lateral run of the fixed lab fixture is required')

    rclpy.init()
    node = Node('recorded_rogmap_planner_check')
    latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
    map_pub = node.create_publisher(OccupancyGrid, '/passage_fusion_test/map', latched)
    terrain_pub = node.create_publisher(OccupancyGrid, '/passage_fusion_test/terrain/grid', 10)
    path_pub = node.create_publisher(NavPath, '/passage_fusion_test/planned_path', 10)
    odom_pub = node.create_publisher(Odometry, '/passage_fusion_test/odom_map', 10)
    obstacle_pub = node.create_publisher(Bool, '/passage_fusion_test/live_obstacle', 10)
    command_pub = node.create_publisher(TwistStamped, '/passage_fusion_test/cmd_vel_desired', 10)
    state = {'cost': None, 'memory': None, 'gate': None, 'permits': []}
    subscriptions = [
        node.create_subscription(Costmap, '/passage_fusion_test/global_costmap/costmap_raw',
                                 lambda m: state.update(cost=m), 10),
        node.create_subscription(String, '/passage/memory_shadow_status',
                                 lambda m: state.update(memory=json.loads(m.data)), 10),
        node.create_subscription(String, '/passage_fusion_test/entry_gate_status',
                                 lambda m: state.update(gate=json.loads(m.data)), 10),
        node.create_subscription(String, '/passage_fusion_test/fresh_permit',
                                 lambda m: state['permits'].append(m.data), 10)]
    tf = StaticTransformBroadcaster(node)
    action = ActionClient(node, ComputePathToPose, '/passage_fusion_test/compute_path_to_pose')
    lifecycle = node.create_client(ChangeState, '/passage_fusion_test/planner_server/change_state')
    grid = OccupancyGrid()
    grid.header.frame_id = 'map'
    grid.header.stamp = node.get_clock().now().to_msg()
    grid.info.map_load_time = grid.header.stamp
    grid.info.width = grid.info.height = 80
    grid.info.resolution = context.resolution
    grid.info.origin.position.x = grid.info.origin.position.y = -4.
    grid.info.origin.orientation.w = 1.
    grid.data = [0]*6400
    for y in range(80):
        for x in (57, 58, 66, 67):
            grid.data[y*80+x] = 100
    restored = PassageMemoryV2.restore(str(args.record_directory/'memory.json'), context,
                                       list(grid.data))
    if restored.state != 'KNOWN_PASSAGE' or len(restored.segments) < 4:
        raise RuntimeError('Saved measured memory failed map/geometry/hash validation')
    transforms = []
    for child in ('world', 'base_link'):
        transform = TransformStamped()
        transform.header = grid.header
        transform.child_frame_id = child
        transform.transform.rotation.w = 1.
        transforms.append(transform)
    processes = []
    route = None
    robot_x = 0.
    last_publish = 0.

    def tick(obstacle=False):
        nonlocal last_publish
        now = time.monotonic()
        if now-last_publish > .06:
            stamp = node.get_clock().now().to_msg()
            terrain = OccupancyGrid()
            terrain.header.frame_id = 'map'
            terrain.header.stamp = stamp
            terrain.info = grid.info
            terrain.data = [0]*6400
            if obstacle:
                for y in range(30, 51):
                    for x in (57, 58):
                        terrain.data[y*80+x] = 100
            terrain_pub.publish(terrain)
            if route is not None:
                path = NavPath()
                path.header = terrain.header
                first = PoseStamped()
                first.header = path.header
                first.pose.position.x = robot_x
                first.pose.orientation.w = 1.
                path.poses = [first] + [copy.deepcopy(p) for p in route.poses
                                       if p.pose.position.x > robot_x+.02]
                path_pub.publish(path)
                odom = Odometry()
                odom.header = path.header
                odom.child_frame_id = 'base_link'
                odom.pose.pose = first.pose
                odom.twist.twist.linear.x = .2
                odom_pub.publish(odom)
                obstacle_pub.publish(Bool(data=obstacle))
                cmd = TwistStamped()
                cmd.header.frame_id = 'base_link'
                cmd.header.stamp = stamp
                cmd.twist.linear.x = .2
                command_pub.publish(cmd)
            last_publish = now
        rclpy.spin_once(node, timeout_sec=.03)

    def wait(predicate, label, obstacle=False, seconds=10):
        deadline = time.monotonic()+seconds
        while time.monotonic() < deadline:
            for process, log_path in processes:
                if process.poll() is not None:
                    raise AssertionError(f'{label}: child exited: {log_path.read_text()[-1800:]}')
            tick(obstacle)
            if predicate():
                return
        logs = {str(p): p.read_text()[-1000:] for _, p in processes}
        raise AssertionError(f'{label}: gate={state["gate"]} memory={state["memory"]} logs={logs}')

    def transition(value):
        request = ChangeState.Request()
        request.transition.id = value
        future = lifecycle.call_async(request)
        wait(future.done, 'planner lifecycle')
        assert future.result().success

    def plan(obstacle=False):
        goal = ComputePathToPose.Goal()
        goal.use_start = True
        goal.planner_id = 'GridBased'
        for pose, x in ((goal.start, 0.), (goal.goal, 2.1)):
            pose.header.frame_id = 'map'
            pose.header.stamp = node.get_clock().now().to_msg()
            pose.pose.position.x = x
            pose.pose.orientation.w = 1.
        sent = action.send_goal_async(goal)
        wait(sent.done, 'planner accepts request', obstacle)
        assert sent.result().accepted
        result = sent.result().get_result_async()
        wait(result.done, 'planner returns route', obstacle)
        return result.result().status, result.result().result.path

    try:
        with tempfile.TemporaryDirectory(prefix='recorded-rogmap-planner-') as directory:
            root = Path(__file__).resolve().parents[1]
            def start(name, command):
                log_path = Path(directory)/f'{name}.log'
                with open(log_path, 'w', encoding='utf-8') as log:
                    process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT)
                processes.append((process, log_path))
            start('planner', ['ros2', 'run', 'nav2_planner', 'planner_server', '--ros-args',
                             '-r', '__ns:=/passage_fusion_test', '--params-file',
                             str(root/'config/isolated_planner.yaml')])
            wait(lifecycle.service_is_ready, 'planner lifecycle service')
            transition(Transition.TRANSITION_CONFIGURE)
            map_pub.publish(grid)
            tf.sendTransform(transforms)
            transition(Transition.TRANSITION_ACTIVATE)
            wait(action.server_is_ready, 'planner action service')
            wait(lambda: state['cost'] is not None and state['cost'].data[40*80+57] == 254,
                 'original static wall')
            assert plan()[0] != GoalStatus.STATUS_SUCCEEDED
            copied_record = Path(directory)/'memory.json'
            shutil.copyfile(args.record_directory/'memory.json', copied_record)
            start('memory', [sys.executable, str(root/'scripts/live_memory_shadow.py'),
                             '--ros-args', '-r', '__ns:=/passage_fusion_test',
                             '-r', '/map:=/passage_fusion_test/map',
                             '-p', 'map_session_id:=live-rogmap-lab-session',
                             '-p', f'record_path:={copied_record}',
                             '-p', 'evidence_frame:=world', '-p', 'publish_test_mask:=true'])
            wait(lambda: state['cost'].data[40*80+57] < 253 and state['memory'] and
                 state['memory']['state'] == 'KNOWN_PASSAGE', 'restore measured passage')
            status, route = plan()
            assert status == GoalStatus.STATUS_SUCCEEDED and len(route.poses) > 2
            assert any(1.65 < p.pose.position.x < 1.95 for p in route.poses)
            (args.record_directory/'planner_path.json').write_text(json.dumps(
                [[p.pose.position.x, p.pose.position.y] for p in route.poses]))
            start('gate', [sys.executable, str(root/'scripts/isolated_entry_gate.py'),
                           '--ros-args', '-r', '__ns:=/passage_fusion_test'])
            wait(lambda: state['gate'] and state['gate']['state'] == 'WAIT_VERIFY' and
                 state['gate']['linear_x'] > 0, 'controlled approach to measured passage')
            robot_x = 1.3
            wait(lambda: state['gate'] and state['gate']['reason'] ==
                 'STOP_BEFORE_UNVERIFIED_ENTRY' and state['gate']['linear_x'] == 0,
                 'historical measured passage grants no current entry')
            assert not state['permits']
            assert node.count_publishers('/terrain/clearance/verified_segment') == 0
            wait(lambda: state['cost'].data[40*80+57] == 254,
                 'real-time obstacle overrides remembered passage', obstacle=True)
            assert plan(obstacle=True)[0] != GoalStatus.STATUS_SUCCEEDED
            report = {'passed': True, 'source_memory_sha256': record['sha256'],
                      'restored_segments': len(restored.segments),
                      'planner_path_poses': len(route.poses),
                      'current_entry_permits': len(state['permits']),
                      'unverified_entry_stopped': True, 'dynamic_obstacle_veto': True}
            (args.record_directory/'planner_replay.json').write_text(json.dumps(report, indent=2))
            print('PASS: measured ROGMap memory after restart → actual Nav2 path → '
                  'controlled approach and entry stop; dynamic obstacle veto', flush=True)
    finally:
        for process, _ in reversed(processes):
            if process.poll() is None:
                process.terminate()
                try:
                    process.wait(timeout=4)
                except subprocess.TimeoutExpired:
                    process.kill(); process.wait(timeout=3)
        del subscriptions, action, lifecycle, tf
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    run()
