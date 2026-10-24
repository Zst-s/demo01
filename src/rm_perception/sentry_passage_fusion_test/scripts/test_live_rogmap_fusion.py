#!/usr/bin/env python3
"""Headless Gazebo lidar → ROGMap proof → memory → real Nav2 test master.

This checks two distinct 0.2 m segments, not whole-tunnel traversability.
No base or chassis controller is started. The diagnostic sensor rig is moved
only by the existing lab scanner and restored by that scanner on exit.
"""
import json
import argparse
from contextlib import nullcontext
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time

import rclpy
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from geometry_msgs.msg import TransformStamped
from nav2_msgs.msg import Costmap
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from tf2_ros import StaticTransformBroadcaster


def stop(process, interrupt=False):
    if process is None or process.poll() is not None:
        return
    process.send_signal(signal.SIGINT if interrupt else signal.SIGTERM)
    try:
        process.wait(timeout=8)
    except subprocess.TimeoutExpired:
        process.terminate()
        try:
            process.wait(timeout=4)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=3)


def run():
    domain_id = os.environ.get('ROS_DOMAIN_ID')
    if domain_id not in {str(value) for value in range(219, 240)} or \
            os.environ.get('ROS_LOCALHOST_ONLY') != '1':
        raise RuntimeError('Requires isolated ROS_DOMAIN_ID 219..239 and ROS_LOCALHOST_ONLY=1')
    parser = argparse.ArgumentParser()
    parser.add_argument('harness')
    parser.add_argument('--lateral-diagnostic', action='store_true')
    parser.add_argument('--artifacts-dir', type=Path,
                        help='New directory retaining logs and result after exit/interruption')
    args = parser.parse_args()
    lateral_diagnostic = args.lateral_diagnostic
    if args.artifacts_dir is not None:
        args.artifacts_dir.mkdir(parents=True, exist_ok=False)
        artifacts = nullcontext(str(args.artifacts_dir))
    else:
        artifacts = tempfile.TemporaryDirectory(prefix='live-rogmap-fusion-')
    rclpy.init()
    node = Node('live_rogmap_nav2_wire', parameter_overrides=[
        Parameter('use_sim_time', value=True)])
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
    latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
    map_pub = node.create_publisher(OccupancyGrid, '/passage_fusion_test/map', latched)
    terrain_pub = node.create_publisher(OccupancyGrid, '/passage_fusion_test/terrain/grid', qos)
    events = {'raw': None, 'proofs': [], 'permits': [], 'status': None}
    raw_sub = node.create_subscription(
        Costmap, '/passage_fusion_test/costmap_raw',
        lambda msg: events.update(raw=msg), qos)
    proof_sub = node.create_subscription(
        String, '/terrain/clearance/verified_segment',
        lambda msg: events['proofs'].append(json.loads(msg.data)), 10)
    status_sub = node.create_subscription(
        String, '/passage/memory_shadow_status',
        lambda msg: events.update(status=json.loads(msg.data)), 10)
    permit_sub = node.create_subscription(
        String, '/passage_fusion_test/fresh_permit',
        lambda msg: events['permits'].append(json.loads(msg.data)), 10)
    broadcaster = StaticTransformBroadcaster(node)
    lab = harness = shadow = scan = None

    def cost(x=57):
        raw = events['raw']
        return None if raw is None else raw.data[40*80+x]

    def tick(grid=None, obstacle=False):
        if grid is not None and node.get_clock().now().nanoseconds > 0:
            terrain = OccupancyGrid()
            terrain.header.frame_id = 'map'
            terrain.header.stamp = node.get_clock().now().to_msg()
            terrain.info = grid.info
            terrain.data = [0]*6400
            if obstacle:
                terrain.data[40*80+57] = 100
            terrain_pub.publish(terrain)
        rclpy.spin_once(node, timeout_sec=.05)

    def wait_for(predicate, label, seconds=20, grid=None, obstacle=False):
        deadline = time.monotonic()+seconds
        while time.monotonic() < deadline:
            for process, log_name in ((lab, 'perception.log'), (harness, 'harness.log'),
                                      (shadow, 'shadow.log')):
                if process is not None and process.poll() is not None:
                    log_path = Path(directory)/log_name
                    detail = log_path.read_text(encoding='utf-8')[-1800:] if log_path.exists() else ''
                    raise AssertionError(f'{label}: {log_name} process exited '
                                         f'{process.returncode}: {detail}')
            tick(grid, obstacle)
            if predicate():
                return
        raise AssertionError(f'{label}: raw_costs=({cost(57)}, {cost(66)}) '
                             f'memory={events["status"]} proofs={events["proofs"][-2:]}')

    try:
        with artifacts as directory:
            print(f'Diagnostic artifacts: {directory}', flush=True)
            package = Path(__file__).resolve().parents[1]
            lab_share = Path(get_package_share_directory('sentry_passage_sim'))
            lab_log = open(Path(directory)/'perception.log', 'w', encoding='utf-8')
            lab = subprocess.Popen([
                'ros2', 'launch', 'sentry_passage_sim', 'perception_lab.launch.py',
                'gui:=false', 'rviz:=false',
                f'world_file:={lab_share}/worlds/passages_dense.world',
                f'config_file:={lab_share}/config/rog_map_dense_lab.yaml'],
                stdout=lab_log, stderr=subprocess.STDOUT)
            lab_log.close()
            wait_for(lambda: node.get_clock().now().nanoseconds > 0,
                     'Gazebo clock', seconds=40)
            harness_log = open(Path(directory)/'harness.log', 'w', encoding='utf-8')
            harness = subprocess.Popen([
                args.harness, '--ros-args', '--params-file',
                str(package/'config/fusion_test.yaml'), '-p', 'use_sim_time:=true'],
                stdout=harness_log, stderr=subprocess.STDOUT)
            harness_log.close()
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
                for x in (57, 58, 66, 67):
                    grid.data[y*80+x] = 100
            tf = TransformStamped()
            tf.header.frame_id = 'map'
            tf.child_frame_id = 'world'
            tf.header.stamp = node.get_clock().now().to_msg()
            tf.transform.rotation.w = 1.
            broadcaster.sendTransform(tf)
            wait_for(lambda: map_pub.get_subscription_count() >= 2,
                     'Nav2 static/correction subscriptions', grid=grid)
            map_pub.publish(grid)
            wait_for(lambda: cost(57) == 254 and cost(66) == 254,
                     'Original static walls', grid=grid)
            shadow_log = open(Path(directory)/'shadow.log', 'w', encoding='utf-8')
            shadow = subprocess.Popen([
                sys.executable, str(package/'scripts/live_memory_shadow.py'),
                '--ros-args', '-r', '__ns:=/passage_fusion_test',
                '-r', '/map:=/passage_fusion_test/map',
                '-p', 'use_sim_time:=true',
                '-p', 'map_session_id:=live-rogmap-lab-session',
                '-p', f'record_path:={directory}/memory.json',
                '-p', 'evidence_frame:=world', '-p', 'publish_test_mask:=true'],
                stdout=shadow_log, stderr=subprocess.STDOUT)
            shadow_log.close()
            wait_for(lambda: map_pub.get_subscription_count() >= 3,
                     'Live memory map subscription', grid=grid)
            map_pub.publish(grid)
            wait_for(lambda: events['status'] and events['status']['map_associated'],
                     'Live memory map/world association', grid=grid)
            scan_log = open(Path(directory)/'scan.log', 'w', encoding='utf-8')
            scan = subprocess.Popen([
                sys.executable,
                str(Path(get_package_prefix('sentry_passage_sim'))/
                    'lib/sentry_passage_sim/multiview_scan.py'),
                '--probe-x', '1.7', '--require-verified'],
                stdout=scan_log, stderr=subprocess.STDOUT)
            scan_log.close()
            wait_for(lambda: any(abs(p.get('probe_x', -100)-1.7) < .001
                                 for p in events['proofs']) and
                     events['status'] and events['status']['state'] == 'KNOWN_PASSAGE' and
                     cost(57) is not None and cost(57) < 254 and cost(66) == 254,
                     'First actual ROGMap proof must open only the first Nav2 wall',
                     seconds=120, grid=grid)
            assert any(p['checked_voxels'] > 0 for p in events['proofs'])
            assert events['status']['segment_count'] >= 1
            wait_for(lambda: scan.poll() is not None, 'Multiview scanner restore',
                     seconds=120, grid=grid)
            if scan.returncode != 0:
                raise AssertionError('Multiview scan failed')
            print('Centre entrance proof received; sensor restored', flush=True)
            lateral_results = {}
            if lateral_diagnostic:
                for y in (-.25, .25):
                    side_log_path = Path(directory)/f'lateral_{y:+.2f}.log'
                    with open(side_log_path, 'w', encoding='utf-8') as side_log:
                        scan = subprocess.Popen([
                            sys.executable,
                            str(Path(get_package_prefix('sentry_passage_sim'))/
                                'lib/sentry_passage_sim/multiview_scan.py'),
                            '--probe-x', '1.7', '--probe-y', str(y)],
                            stdout=side_log, stderr=subprocess.STDOUT)
                    wait_for(lambda: scan.poll() is not None,
                             f'Lateral scan {y:+.2f} restores diagnostic sensor',
                             seconds=120, grid=grid)
                    if scan.returncode != 0:
                        raise AssertionError(
                            f'Lateral scan {y:+.2f} failed: '
                            f'{side_log_path.read_text(encoding="utf-8")[-1800:]}')
                    rows = []
                    for line in side_log_path.read_text(encoding='utf-8').splitlines():
                        try:
                            item = json.loads(line)
                            if 'view' in item:
                                rows.append(item)
                        except json.JSONDecodeError:
                            pass
                    proof = next((p for p in reversed(events['proofs']) if
                                  abs(p.get('probe_x', -100)-1.7) < .001 and
                                  abs(p.get('probe_y', -100)-y) < .001), None)
                    lateral_results[str(y)] = {
                        'verified': proof is not None,
                        'best_body_unknown': min(
                            (row['median']['body_unknown'] for row in rows),
                            default=None),
                        'best_support_missing': min(
                            (row['median']['support_missing'] for row in rows),
                            default=None),
                        'recent_body_voxels': (
                            proof.get('recent_body_voxels') if proof else None),
                        'body_voxels': proof.get('body_voxels') if proof else None}
                    print(f'Lateral {y:+.2f}: '
                          f'{json.dumps(lateral_results[str(y)], sort_keys=True)}', flush=True)
            # A second short proof is made from additional sensor viewpoints,
            # including interior *diagnostic rig* positions. No chassis moves.
            second_log = open(Path(directory)/'second_scan.log', 'w', encoding='utf-8')
            scan = subprocess.Popen([
                sys.executable,
                str(Path(get_package_prefix('sentry_passage_sim'))/
                    'lib/sentry_passage_sim/multiview_scan.py'),
                '--probe-x', '2.6', '--interior-diagnostic', '--require-verified'],
                stdout=second_log, stderr=subprocess.STDOUT)
            second_log.close()
            wait_for(lambda: any(abs(p.get('probe_x', -100)-2.6) < .001
                                 for p in events['proofs']) and
                     events['status'] and events['status'].get('segment_count', 0) >= 2 and
                     cost(66) is not None and cost(66) < 254,
                     'Second actual ROGMap proof must open second Nav2 wall',
                     seconds=180, grid=grid)
            wait_for(lambda: scan.poll() is not None, 'Second scanner restore',
                     seconds=120, grid=grid)
            if scan.returncode != 0:
                raise AssertionError('Second multiview scan failed')
            wait_for(lambda: cost() == 254, 'Actual live obstacle veto',
                     grid=grid, obstacle=True)
            wait_for(lambda: cost() < 254, 'Historical route after obstacle clears',
                     grid=grid)
            for permit in events['permits']:
                source = next((p for p in events['proofs'] if
                               abs(p['measured_at']-permit['measured_at']) < 1e-5), None)
                assert source is not None, 'Entry permit has no actual ROGMap source proof'
                assert source['measured_at']-source.get(
                    'oldest_body_observation', source['oldest_observation']) <= 2.0
                assert source['measured_at']-source.get(
                    'newest_body_observation', source['newest_observation']) <= .5
            coverage = {}
            for center in (1.7, 2.6):
                source = next((p for p in reversed(events['proofs']) if
                               abs(p.get('probe_x', -100)-center) < .001 and
                               abs(p.get('probe_y', -100)) < .001), None)
                assert source and source['body_voxels'] > 0
                coverage[str(center)] = {
                    'recent': source['recent_body_voxels'],
                    'body': source['body_voxels'],
                    'axial_low_high': source['recent_body_by_slice'],
                    'oldest_body_age': round(source['measured_at']-
                                             source['oldest_body_observation'], 2)}
            with open(Path(directory)/'result.json', 'w', encoding='utf-8') as result_file:
                json.dump({'passed': True, 'fresh_entry_permits': len(events['permits']),
                           'lateral_diagnostics': lateral_results,
                           'body_coverage': coverage}, result_file, indent=2)
            print('PASS: Gazebo lidar → two distinct actual ROGMap 3D proofs → '
                  'disk memory → real Nav2 test master; live obstacle veto; '
                  f'fresh_entry_permits={len(events["permits"])} '
                  f'lateral_diagnostics={json.dumps(lateral_results, sort_keys=True)} '
                  f'body_coverage={json.dumps(coverage, sort_keys=True)}', flush=True)
    finally:
        stop(scan, interrupt=True)
        stop(shadow)
        stop(harness)
        stop(lab, interrupt=True)
        del raw_sub, proof_sub, status_sub, permit_sub, broadcaster
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    run()
