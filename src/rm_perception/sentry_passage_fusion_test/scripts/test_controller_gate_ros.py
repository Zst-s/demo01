#!/usr/bin/env python3
"""Actual Nav2 FollowPath -> adapter -> gate -> watchdog kinematic test plant.

Fixture geometry and permissions are synthetic. This tests controller wiring,
not ROGMap detection, Gazebo dynamics or real vehicle braking. No /cmd_vel.
"""
import copy
import json
import math
import os
import struct
from pathlib import Path
import subprocess
import sys
import tempfile
import time

import rclpy
from ament_index_python.packages import get_package_prefix
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist, TwistStamped
from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState
from nav2_msgs.action import FollowPath
from nav_msgs.msg import OccupancyGrid, Odometry, Path as NavPath
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String
from tf2_ros import TransformBroadcaster

from global_passage_memory import MapContext
from passage_memory_v2 import PassageMemoryV2


def run():
    if os.environ.get('ROS_DOMAIN_ID') != '225' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
        raise RuntimeError('Requires isolated domain 225 and ROS_LOCALHOST_ONLY=1')
    rclpy.init()
    node = Node('controller_gate_check', namespace='/passage_fusion_test')
    root = Path(__file__).resolve().parents[1]
    latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
    map_pub = node.create_publisher(OccupancyGrid, '/passage_fusion_test/map', latched)
    free_pub = node.create_publisher(OccupancyGrid, '/passage_fusion_test/controller_fixture_map', latched)
    odom_pub = node.create_publisher(Odometry, '/passage_fusion_test/odom_map', 10)
    path_pub = node.create_publisher(NavPath, '/passage_fusion_test/planned_path', 10)
    obstacle_pub = node.create_publisher(Bool, '/passage_fusion_test/live_obstacle', 10)
    permit_pub = node.create_publisher(String, '/passage_fusion_test/fresh_permit', 10)
    state = {'raw': 0., 'raw_at': 0., 'authorized': None, 'auth_at': 0.,
             'gate': None, 'memory': None}
    def raw(msg):
        state.update(raw=msg.linear.x, raw_at=time.monotonic())
    def authorized(msg):
        state.update(authorized=msg, auth_at=time.monotonic())
    subscriptions = [
        node.create_subscription(Twist, '/passage_fusion_test/controller_cmd_vel_raw', raw, 10),
        node.create_subscription(TwistStamped, '/passage_fusion_test/cmd_vel_authorized', authorized, 10),
        node.create_subscription(String, '/passage_fusion_test/entry_gate_status',
                                 lambda m: state.update(gate=json.loads(m.data)), 10),
        node.create_subscription(String, '/passage/memory_shadow_status',
                                 lambda m: state.update(memory=json.loads(m.data)), 10)]
    tf = TransformBroadcaster(node)
    action = ActionClient(node, FollowPath, '/passage_fusion_test/follow_path')
    lifecycle = node.create_client(ChangeState, '/passage_fusion_test/controller_server/change_state')
    grid = OccupancyGrid()
    grid.header.frame_id = 'map'
    grid.header.stamp = node.get_clock().now().to_msg()
    grid.info.map_load_time = grid.header.stamp
    grid.info.width = grid.info.height = 100
    grid.info.resolution = struct.unpack('f', struct.pack('f', .1))[0]
    grid.info.origin.position.x = grid.info.origin.position.y = -4.
    grid.info.origin.orientation.w = 1.
    grid.data = [0]*10000
    free_map = copy.deepcopy(grid)
    for y in range(100):
        grid.data[y*100+60] = 100
    context = MapContext('controller-fixture', 'map', 100, 100, grid.info.resolution,
                         -4., -4., 0., 0., 0.)
    memory = PassageMemoryV2()
    for y in (-.25, 0., .25):
        memory.add(context=context, grid=list(grid.data), center_x=2., center_y=y,
                   yaw=0., travel_length=1., robot_length=.45, robot_width=.45,
                   ground_z=0., robot_height=.7, confirmed_frames=3, measured_at=10., now=10.)
    x = y = yaw = speed = 0.
    obstacle = permits = False
    odom_enabled = True
    last_tick = time.monotonic()
    processes = []
    handle = None

    def path_from(start):
        path = NavPath()
        path.header.frame_id = 'map'
        path.header.stamp = node.get_clock().now().to_msg()
        for index in range(81):
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = start+(3.5-start)*index/80
            pose.pose.orientation.w = 1.
            path.poses.append(pose)
        return path

    def tick():
        nonlocal x, y, yaw, speed, last_tick
        now = time.monotonic()
        dt = min(now-last_tick, .05)
        last_tick = now
        command = state['authorized']
        # Only authorized output moves the plant; gate death also stops it.
        speed = command.twist.linear.x if command and now-state['auth_at'] <= .2 else 0.
        omega = command.twist.angular.z if command and now-state['auth_at'] <= .2 else 0.
        x += speed*math.cos(yaw)*dt
        y += speed*math.sin(yaw)*dt
        yaw += omega*dt
        stamp = node.get_clock().now().to_msg()
        odom = Odometry()
        odom.header.frame_id = 'map'
        odom.header.stamp = stamp
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x, odom.pose.pose.position.y = x, y
        odom.pose.pose.orientation.z = math.sin(yaw/2)
        odom.pose.pose.orientation.w = math.cos(yaw/2)
        odom.twist.twist.linear.x = speed
        odom.twist.twist.angular.z = omega
        if odom_enabled:
            odom_pub.publish(odom)
        transform = TransformStamped()
        transform.header = odom.header
        transform.child_frame_id = 'base_link'
        transform.transform.translation.x, transform.transform.translation.y = x, y
        transform.transform.rotation = odom.pose.pose.orientation
        identity = TransformStamped()
        identity.header = odom.header
        identity.child_frame_id = 'odom'
        identity.transform.rotation.w = 1.
        tf.sendTransform([transform, identity])
        path_pub.publish(path_from(x))
        obstacle_pub.publish(Bool(data=obstacle))
        if permits and state['memory']:
            current = node.get_clock().now().nanoseconds/1e9
            for segment in state['memory'].get('segments', []):
                permit_pub.publish(String(data=json.dumps(dict(
                    segment_id=segment['segment_id'], measured_at=current,
                    valid_until=current+.3, confirmed_frames=3))))
        rclpy.spin_once(node, timeout_sec=.02)

    def wait(predicate, label, seconds=10.):
        deadline = time.monotonic()+seconds
        while time.monotonic() < deadline:
            tick()
            if predicate():
                print('PASS: '+label, flush=True)
                return
        logs = {name: log.read_text()[-1500:] for name, _, log in processes}
        raise AssertionError(f'{label}: x={x}, state={state}, logs={logs}')

    def hold(seconds):
        deadline = time.monotonic()+seconds
        while time.monotonic() < deadline:
            tick()

    def transition(value):
        request = ChangeState.Request()
        request.transition.id = value
        future = lifecycle.call_async(request)
        wait(future.done, 'controller lifecycle transition')
        assert future.result().success

    try:
        with tempfile.TemporaryDirectory(prefix='controller-gate-') as directory:
            record = Path(directory)/'memory.json'
            memory.save(record)
            def start(name, command):
                log = Path(directory)/(name+'.log')
                with log.open('w') as out:
                    process = subprocess.Popen(command, stdout=out, stderr=subprocess.STDOUT)
                processes.append((name, process, log))
                return process
            controller = start('controller', [str(Path(get_package_prefix('nav2_controller')) /
                'lib/nav2_controller/controller_server'),
                '--ros-args', '-r', '__ns:=/passage_fusion_test',
                '-r', 'cmd_vel:=/passage_fusion_test/controller_cmd_vel_raw',
                '--params-file', str(root/'config/isolated_controller.yaml')])
            start('memory', [sys.executable, str(root/'scripts/live_memory_shadow.py'), '--ros-args',
                '-r', '__ns:=/passage_fusion_test', '-r', '/map:=/passage_fusion_test/map',
                '-p', 'map_session_id:=controller-fixture', '-p', f'record_path:={record}',
                '-p', 'publish_test_mask:=true'])
            start('adapter', [sys.executable, str(root/'scripts/controller_command_adapter.py'),
                              '--ros-args', '-r', '__ns:=/passage_fusion_test'])
            gate = start('gate', [sys.executable, str(root/'scripts/isolated_entry_gate.py'),
                                  '--ros-args', '-r', '__ns:=/passage_fusion_test'])
            map_pub.publish(grid)
            free_pub.publish(free_map)
            wait(lifecycle.service_is_ready, 'actual ControllerServer started')
            transition(Transition.TRANSITION_CONFIGURE)
            transition(Transition.TRANSITION_ACTIVATE)
            wait(action.server_is_ready, 'FollowPath action available')
            goal = FollowPath.Goal()
            goal.path = path_from(0.)
            goal.controller_id, goal.goal_checker_id = 'FollowPath', 'goal_checker'
            sent = action.send_goal_async(goal)
            wait(sent.done, 'FollowPath request accepted')
            handle = sent.result()
            assert handle.accepted
            wait(lambda: x > .1 and state['raw'] > .1, 'actual controller drives plant through gate')
            odom_enabled = False
            wait(lambda: state['gate'] and state['gate']['reason'] ==
                 'STALE_OBSTACLE_OR_COMMAND' and speed == 0.,
                 'stale localization stops controller output')
            odom_enabled = True
            wait(lambda: speed > .1, 'fresh localization resumes control')
            # Check endpoint ownership, not just topic names.
            expected = {'/passage_fusion_test/controller_cmd_vel_raw': 'controller_server',
                        '/passage_fusion_test/cmd_vel_desired': 'controller_command_adapter',
                        '/passage_fusion_test/cmd_vel_authorized': 'isolated_entry_gate'}
            for topic, owner in expected.items():
                endpoints = node.get_publishers_info_by_topic(topic)
                assert len(endpoints) == 1 and endpoints[0].node_name == owner, (topic, endpoints)
            assert not any(name == '/cmd_vel' for name, _ in node.get_topic_names_and_types())
            wait(lambda: state['gate'] and state['gate']['reason'] == 'STOP_BEFORE_UNVERIFIED_ENTRY'
                 and speed == 0., 'history permits approach but not entry', seconds=15.)
            stopped = x
            assert stopped < 1.5-.225
            permits = True
            wait(lambda: x > stopped+.08 and state['gate']['state'] == 'ALLOWED',
                 'fresh test permission releases real controller output')
            permits = False
            wait(lambda: state['gate']['state'] == 'BLOCKED' and speed == 0.,
                 'permission expiry stops plant')
            stopped = x
            hold(.3)
            assert abs(x-stopped) < .005 and state['raw'] > .05
            permits = True
            wait(lambda: speed > .1, 'fresh permission resumes tracking')
            obstacle = True
            wait(lambda: state['gate']['reason'] == 'LIVE_OBSTACLE_VETO' and speed == 0.,
                 'live obstacle overrides controller and permission')
            obstacle = False
            wait(lambda: speed > .1, 'obstacle removal resumes tracking')
            # Cancel the action: the controller must produce/leave a stop, and
            # receipt timestamps must not be renewed by the command adapter.
            cancelled = handle.cancel_goal_async()
            wait(cancelled.done, 'FollowPath cancelled')
            wait(lambda: speed == 0., 'upstream action stop stops plant')
            hold(.5)
            assert state['gate']['state'] == 'BLOCKED'
            goal.path = path_from(x)
            sent = action.send_goal_async(goal)
            wait(sent.done, 'new FollowPath after cancellation')
            handle = sent.result()
            assert handle.accepted
            wait(lambda: speed > .1, 'tracking resumed before gate failure')
            before_loss = x
            gate.terminate()
            gate.wait(timeout=4)
            hold(.3)
            assert speed == 0.
            assert x-before_loss < .06
            print('PASS: moving plant stops on gate-process loss via downstream watchdog', flush=True)
            gate = start('gate_restart', [sys.executable, str(root/'scripts/isolated_entry_gate.py'),
                                         '--ros-args', '-r', '__ns:=/passage_fusion_test'])
            wait(lambda: speed > .1, 'gate rechecks current inputs before resuming')
            controller.kill()
            controller.wait(timeout=4)
            wait(lambda: speed == 0. and state['gate']['reason'] == 'STALE_OBSTACLE_OR_COMMAND',
                 'controller-process loss expires commands and stops plant')
            stopped = x
            hold(.3)
            assert abs(x-stopped) < .005
            print('ALL PASS: real controller/gate wiring; fixture permissions and kinematic plant only', flush=True)
    finally:
        for _, process, _ in reversed(processes):
            if process.poll() is None:
                process.terminate()
                try:
                    process.wait(timeout=4)
                except subprocess.TimeoutExpired:
                    process.kill(); process.wait(timeout=3)
        del subscriptions, tf, action, lifecycle
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    run()
