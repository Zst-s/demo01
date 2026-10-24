#!/usr/bin/env python3
"""Historical short passage reaches a real isolated Nav2 PlannerServer.

No controller or chassis topic is started. A successful global path is only a
candidate; the independent entry gate is intentionally not bypassed here.
"""
import json
import os
import copy
from pathlib import Path
import subprocess
import sys
import tempfile
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import TransformStamped, TwistStamped, PoseStamped
from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState
from nav2_msgs.action import ComputePathToPose
from nav2_msgs.msg import Costmap
from nav_msgs.msg import OccupancyGrid, Odometry, Path as NavPath
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String, Bool
from tf2_ros import StaticTransformBroadcaster


def run():
    if os.environ.get('ROS_DOMAIN_ID') != '223' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
        raise RuntimeError('Isolated local ROS_DOMAIN_ID=223 required')
    rclpy.init()
    node = Node('planner_history_wire')
    latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
    map_pub = node.create_publisher(OccupancyGrid, '/passage_fusion_test/map', latched)
    terrain_pub = node.create_publisher(OccupancyGrid,
                                        '/passage_fusion_test/terrain/grid', 10)
    proof_pub = node.create_publisher(String, '/terrain/clearance/verified_segment', 10)
    gate_path_pub = node.create_publisher(NavPath, '/passage_fusion_test/planned_path', 10)
    gate_odom_pub = node.create_publisher(Odometry, '/passage_fusion_test/odom_map', 10)
    gate_obstacle_pub = node.create_publisher(Bool, '/passage_fusion_test/live_obstacle', 10)
    gate_desired_pub = node.create_publisher(
        TwistStamped, '/passage_fusion_test/cmd_vel_desired', 10)
    state = {'cost': None, 'memory': None, 'gate': None}
    cost_sub = node.create_subscription(
        Costmap, '/passage_fusion_test/global_costmap/costmap_raw',
        lambda msg: state.update(cost=msg), 10)
    memory_sub = node.create_subscription(
        String, '/passage/memory_shadow_status',
        lambda msg: state.update(memory=json.loads(msg.data)), 10)
    gate_sub = node.create_subscription(
        String, '/passage_fusion_test/entry_gate_status',
        lambda msg: state.update(gate=json.loads(msg.data)), 10)
    tf = StaticTransformBroadcaster(node)
    planner_action = ActionClient(node, ComputePathToPose,
                                  '/passage_fusion_test/compute_path_to_pose')
    lifecycle = node.create_client(ChangeState,
                                   '/passage_fusion_test/planner_server/change_state')
    grid = OccupancyGrid()
    grid.header.frame_id = 'map'
    grid.header.stamp = node.get_clock().now().to_msg()
    grid.info.map_load_time = grid.header.stamp
    grid.info.width = grid.info.height = 80
    grid.info.resolution = .1
    grid.info.origin.position.x = grid.info.origin.position.y = -4.
    grid.info.origin.orientation.w = 1.
    grid.data = [0]*6400
    for y in range(80):
        for x in (39, 40):
            grid.data[y*80+x] = 100
    transforms = []
    for child, x, y in [('odom', 0., 0.), ('base_link', -2., 0.)]:
        t = TransformStamped()
        t.header.frame_id = 'map'
        t.header.stamp = grid.header.stamp
        t.child_frame_id = child
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.rotation.w = 1.
        transforms.append(t)
    process = shadow = gate = None
    planner_log_path = None
    last_terrain = 0.
    gate_path = None
    gate_pose_x = -2.
    gate_last_inputs = 0.

    def tick(dynamic=False):
        nonlocal last_terrain, gate_last_inputs
        now = time.monotonic()
        if now-last_terrain > .1:
            terrain = OccupancyGrid()
            terrain.header.frame_id = 'map'
            terrain.header.stamp = node.get_clock().now().to_msg()
            terrain.info = grid.info
            terrain.data = [0]*6400
            if dynamic:
                for y in range(30, 51):
                    for x in (39, 40):
                        terrain.data[y*80+x] = 100
            terrain_pub.publish(terrain)
            last_terrain = now
        if gate_path is not None and now-gate_last_inputs > .06:
            stamp = node.get_clock().now().to_msg()
            route = NavPath()
            route.header.frame_id = 'map'
            route.header.stamp = stamp
            anchor = PoseStamped()
            anchor.header = route.header
            anchor.pose.position.x = gate_pose_x
            anchor.pose.position.y = 0.
            anchor.pose.orientation.w = 1.
            route.poses.append(anchor)
            for point in gate_path.poses:
                if point.pose.position.x > gate_pose_x+.05:
                    route.poses.append(copy.deepcopy(point))
            gate_path_pub.publish(route)
            odom = Odometry()
            odom.header = route.header
            odom.child_frame_id = 'base_link'
            odom.pose.pose.position.x = gate_pose_x
            odom.pose.pose.position.y = 0.
            odom.pose.pose.orientation.w = 1.
            odom.twist.twist.linear.x = .2
            gate_odom_pub.publish(odom)
            gate_obstacle_pub.publish(Bool(data=dynamic))
            desired = TwistStamped()
            desired.header.frame_id = 'base_link'
            desired.header.stamp = stamp
            desired.twist.linear.x = .2
            gate_desired_pub.publish(desired)
            gate_last_inputs = now
        rclpy.spin_once(node, timeout_sec=.03)

    def wait(predicate, label, limit=10., dynamic=False):
        end = time.monotonic()+limit
        while time.monotonic() < end:
            tick(dynamic)
            if predicate():
                return
        log_tail = (planner_log_path.read_text(encoding='utf-8')[-2500:]
                    if planner_log_path and planner_log_path.exists() else '')
        gate_log_path = Path(directory)/'gate.log' if 'directory' in locals() else None
        gate_tail = (gate_log_path.read_text(encoding='utf-8')[-1000:]
                     if gate_log_path and gate_log_path.exists() else '')
        raise AssertionError(f'{label}: memory={state["memory"]} '
                             f'gate={state["gate"]} '
                             f'passage_path={[(p.pose.position.x,p.pose.position.y) for p in gate_path.poses if abs(p.pose.position.x)<.4] if gate_path else None} '
                             f'cost={None if state["cost"] is None else state["cost"].data[40*80+40]} '
                             f'gate_log={gate_tail} planner_log={log_tail}')

    def transition(which):
        request = ChangeState.Request()
        request.transition.id = which
        future = lifecycle.call_async(request)
        wait(future.done, f'lifecycle transition {which}')
        assert future.result().success, f'planner lifecycle transition {which} failed'

    def plan(dynamic=False):
        goal = ComputePathToPose.Goal()
        goal.use_start = True
        for pose, x in ((goal.start, -2.), (goal.goal, 2.)):
            pose.header.frame_id = 'map'
            pose.header.stamp = node.get_clock().now().to_msg()
            pose.pose.position.x = x
            pose.pose.position.y = 0.
            pose.pose.orientation.w = 1.
        goal.planner_id = 'GridBased'
        future = planner_action.send_goal_async(goal)
        wait(future.done, 'planner action goal response', 12, dynamic)
        handle = future.result()
        assert handle.accepted, 'Nav2 planner rejected goal'
        outcome = handle.get_result_async()
        wait(outcome.done, 'planner action result', 12, dynamic)
        return outcome.result().status, outcome.result().result.path

    try:
        with tempfile.TemporaryDirectory(prefix='planner-history-') as directory:
            root = Path(__file__).resolve().parents[1]
            planner_log_path = Path(directory)/'planner.log'
            log = open(planner_log_path, 'w', encoding='utf-8')
            process = subprocess.Popen([
                'ros2', 'run', 'nav2_planner', 'planner_server',
                '--ros-args', '-r', '__ns:=/passage_fusion_test',
                '--params-file', str(root/'config/isolated_planner.yaml')],
                stdout=log, stderr=subprocess.STDOUT)
            log.close()
            wait(lambda: lifecycle.service_is_ready(), 'planner lifecycle service')
            transition(Transition.TRANSITION_CONFIGURE)
            map_pub.publish(grid)
            tf.sendTransform(transforms)
            transition(Transition.TRANSITION_ACTIVATE)
            wait(lambda: planner_action.server_is_ready(), 'planner action server')
            wait(lambda: state['cost'] is not None and
                 state['cost'].data[40*80+40] == 254, 'original wall in planner master')
            status, _ = plan()
            assert status != GoalStatus.STATUS_SUCCEEDED, 'Unverified wall was planned through'
            output = open(Path(directory)/'shadow.log', 'w', encoding='utf-8')
            shadow = subprocess.Popen([
                sys.executable, str(root/'scripts/live_memory_shadow.py'),
                '--ros-args', '-r', '__ns:=/passage_fusion_test',
                '-r', '/map:=/passage_fusion_test/map',
                '-p', 'map_session_id:=planner-history-session',
                '-p', f'record_path:={directory}/memory.json',
                '-p', 'evidence_frame:=odom', '-p', 'publish_test_mask:=true'],
                stdout=output, stderr=subprocess.STDOUT)
            output.close()
            wait(lambda: proof_pub.get_subscription_count() > 0 and
                 state['memory'] and state['memory']['map_associated'],
                 'memory subscriber and map association')
            # One robot-width strip is correctly rejected after inflation. Three
            # independently verified adjacent strips make a wider corridor.
            for count, center_y in enumerate((-.3, 0., .3), start=1):
                end = time.monotonic()+6
                while time.monotonic() < end and not (state['memory'] and
                        state['memory']['state'] == 'KNOWN_PASSAGE' and
                        state['memory'].get('segment_count', 0) >= count):
                    stamp = node.get_clock().now().nanoseconds/1e9
                    proof_pub.publish(String(data=json.dumps({
                        'schema': 1, 'kind': 'verified_short_segment_observation',
                        'frame': 'odom', 'probe_x': -.05, 'probe_y': center_y,
                        'ground_z': -.025, 'yaw': 0., 'travel_length': .2,
                        'robot_length': .45, 'robot_width': .45,
                        'robot_height': .7, 'resolution': .1, 'revision': 1,
                        'confirmed_frames': 3, 'measured_at': stamp,
                        'oldest_observation': stamp-8.,
                        'newest_observation': stamp, 'valid_until': stamp+.8,
                        'checked_voxels': 432})))
                    tick()
                assert state['memory'].get('segment_count', 0) >= count
            wait(lambda: state['cost'].data[40*80+40] < 253,
                 'inflation-aware historical gap in planner master')
            status, path = plan()
            assert status == GoalStatus.STATUS_SUCCEEDED and len(path.poses) > 2, (
                f'Historical passage was not used by Nav2: status={status}; '
                f'gap_costs={[list(state["cost"].data[y*80+38:y*80+42]) for y in range(36,45)]}; '
                f'planner_log={planner_log_path.read_text(encoding="utf-8")[-3500:]}')
            assert any(abs(p.pose.position.x) < .3 for p in path.poses)
            # No current entry permit was issued: this is only a global route.
            assert state['memory']['entry_allowed'] is False
            with open(Path(directory)/'gate.log', 'w', encoding='utf-8') as output:
                gate = subprocess.Popen([
                    sys.executable, str(root/'scripts/isolated_entry_gate.py'),
                    '--ros-args', '-r', '__ns:=/passage_fusion_test'],
                    stdout=output, stderr=subprocess.STDOUT)
            gate_path = path  # The gate consumes Nav2's actual planned route.
            wait(lambda: state['gate'] and state['gate']['state'] == 'WAIT_VERIFY' and
                 state['gate']['linear_x'] > 0,
                 'actual Nav2 path permits controlled approach', limit=5)
            gate_pose_x = -.45
            wait(lambda: state['gate'] and state['gate']['state'] == 'BLOCKED' and
                 state['gate']['linear_x'] == 0 and
                 state['gate']['reason'] == 'STOP_BEFORE_UNVERIFIED_ENTRY',
                 'actual Nav2 path stops before entry without fresh permit', limit=5)
            end = time.monotonic()+3
            while time.monotonic() < end and not (
                    state['gate'] and state['gate']['state'] == 'ALLOWED' and
                    state['gate']['linear_x'] > 0):
                stamp = node.get_clock().now().nanoseconds/1e9
                proof_pub.publish(String(data=json.dumps({
                    'schema': 1, 'kind': 'verified_short_segment_observation',
                    'frame': 'odom', 'probe_x': -.05, 'probe_y': 0.,
                    'ground_z': -.025, 'yaw': 0., 'travel_length': .2,
                    'robot_length': .45, 'robot_width': .45,
                    'robot_height': .7, 'resolution': .1, 'revision': 1,
                    'confirmed_frames': 3, 'measured_at': stamp,
                    'oldest_observation': stamp-.1,
                    'newest_observation': stamp, 'valid_until': stamp+.8,
                    'checked_voxels': 432})))
                tick()
            assert state['gate'] and state['gate']['state'] == 'ALLOWED', (
                f'Fresh centre-body proof did not authorize actual Nav2 path: {state["gate"]}')
            wait(lambda: state['gate'] and state['gate']['state'] == 'BLOCKED' and
                 state['gate']['reason'] == 'LIVE_OBSTACLE_VETO' and
                 state['gate']['linear_x'] == 0,
                 'live obstacle vetoes actual planner path at gate', dynamic=True)
            wait(lambda: state['cost'].data[40*80+40] == 254,
                 'dynamic obstacle vetoes remembered gap', dynamic=True)
            status, _ = plan(dynamic=True)
            assert status != GoalStatus.STATUS_SUCCEEDED, (
                'Planner crossed a dynamically blocked historical passage')
            wait(lambda: state['cost'].data[40*80+40] < 253,
                 'clear dynamic obstacle restores historical route')
            status, _ = plan()
            assert status == GoalStatus.STATUS_SUCCEEDED
            wait(lambda: state['gate'] and state['gate']['state'] == 'BLOCKED' and
                 state['gate']['reason'] == 'STOP_BEFORE_UNVERIFIED_ENTRY' and
                 state['gate']['linear_x'] == 0,
                 'expired fresh permit restores entry stop', limit=4)
            shadow.terminate()
            shadow.wait(timeout=4)
            wait(lambda: state['cost'].data[40*80+40] == 254,
                 'expired evidence restores static wall', limit=5)
            with open(Path(directory)/'shadow-restarted.log', 'w', encoding='utf-8') as output:
                shadow = subprocess.Popen([
                    sys.executable, str(root/'scripts/live_memory_shadow.py'),
                    '--ros-args', '-r', '__ns:=/passage_fusion_test',
                    '-r', '/map:=/passage_fusion_test/map',
                    '-p', 'map_session_id:=planner-history-session',
                    '-p', f'record_path:={directory}/memory.json',
                    '-p', 'evidence_frame:=odom', '-p', 'publish_test_mask:=true'],
                    stdout=output, stderr=subprocess.STDOUT)
            wait(lambda: state['cost'].data[40*80+40] < 253,
                 'saved passage restored after memory restart')
            status, _ = plan()
            assert status == GoalStatus.STATUS_SUCCEEDED
            assert state['memory']['entry_allowed'] is False
            print('PASS: real Nav2 planner rejects wall, uses persistent historical '
                  'route after restart; actual path approaches, stops without permit, '
                  'allows one freshly verified short segment, then stops on obstacle '
                  'or permit expiry', flush=True)
    finally:
        for p in (gate, shadow, process):
            if p is not None and p.poll() is None:
                p.terminate()
                try:
                    p.wait(timeout=4)
                except subprocess.TimeoutExpired:
                    p.kill(); p.wait(timeout=3)
        del cost_sub, memory_sub, gate_sub, tf, planner_action, lifecycle
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    run()
