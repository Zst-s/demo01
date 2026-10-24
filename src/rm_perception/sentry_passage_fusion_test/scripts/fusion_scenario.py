#!/usr/bin/env python3
"""第一阶段：合成输入驱动真实 Nav2 图层，断言原始代价；不发送速度。"""
import argparse
import copy
import os
import signal
import subprocess
import sys
import time
import tempfile
from pathlib import Path

import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.msg import Costmap
from std_msgs.msg import String
from visualization_msgs.msg import Marker
from global_passage_memory import MapContext, ConfirmedPassage, GlobalPassageMemory
from collections import deque

NS = '/passage_fusion_test'
WIDTH = 80


class Scenario:
    def __init__(self, pause=0.0, rviz_process=None):
        self.node = rclpy.create_node('fusion_scenario', namespace=NS)
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.map_pub = self.node.create_publisher(OccupancyGrid, 'map', latched)
        self.terrain_pub = self.node.create_publisher(OccupancyGrid, 'terrain/grid', qos)
        self.evidence_pub = self.node.create_publisher(OccupancyGrid, 'test_evidence', qos)
        # 显示通道独立构造消息，禁止向 RViz 转发原始故障注入数据。
        self.preview_pub = self.node.create_publisher(OccupancyGrid, 'evidence_preview', qos)
        self.status_pub = self.node.create_publisher(Marker, 'status_marker', qos)
        self.preview_count = 0
        self.preview_sub = self.node.create_subscription(
            OccupancyGrid, 'evidence_preview', self.check_preview, qos)
        self.command_pub = self.node.create_publisher(String, 'test_command', 10)
        self.raw = None
        self.raw_count = 0
        self.sub = self.node.create_subscription(Costmap, 'costmap_raw', self.receive, qos)
        self.map = self.new_grid()
        # x=39、40 两列模拟静态实心墙，原始 /map 从不因测试证据开洞。
        for y in range(WIDTH):
            for x in (39, 40):
                self.map.data[y * WIDTH + x] = 100
        for y in range(60, 65):
            for x in range(10, 15):
                self.map.data[y * WIDTH + x] = -1
        self.active = None
        self.dynamic = False
        self.last_send = 0.0
        self.pause = pause
        self.passed = []
        self.phase = 'Waiting for map'
        self.rviz_process = rviz_process

    def check_preview(self, msg):
        # 每帧验证安全显示接口；非法消息仍完整送入修正插件接受测试。
        assert msg.header.frame_id == 'map'
        assert msg.header.stamp.sec >= 0 and 0 <= msg.header.stamp.nanosec < 1000000000
        assert msg.info.map_load_time.sec >= 0
        assert 0 <= msg.info.map_load_time.nanosec < 1000000000
        assert msg.info.width == WIDTH and msg.info.height == WIDTH
        assert len(msg.data) == WIDTH * WIDTH and all(v in (0, 100) for v in msg.data)
        assert msg.info.resolution > 0 and msg.info.origin.orientation.w == 1.0
        self.preview_count += 1

    def publish_visualization(self):
        # 预览只表示当前持续发送的人工输入，不表示修正层已批准通行。
        preview = self.new_grid()
        preview.info.origin = copy.deepcopy(self.map.info.origin)
        if self.active is not None and len(self.active.data) == WIDTH * WIDTH:
            preview.data = [100 if value == 100 else 0 for value in self.active.data]
        self.preview_pub.publish(preview)
        if self.raw is None:
            return
        status = Marker()
        status.header.frame_id = 'map'
        status.header.stamp = self.node.get_clock().now().to_msg()
        status.ns = 'fusion_test_status'
        status.id = 0
        status.type = Marker.TEXT_VIEW_FACING
        status.action = Marker.ADD
        status.pose.orientation.w = 1.0
        status.pose.position.y = -4.6
        status.pose.position.z = 0.1
        status.scale.z = 0.16
        status.color.r = status.color.g = status.color.b = 1.0
        status.color.a = 1.0
        status.text = (f'{self.phase}\nA raw cost={self.cost(40, 40)} | '
                       f'B raw cost={self.cost(40, 20)}\n'
                       '0=FREE  254=OBSTACLE  255=UNKNOWN | TEST ONLY')
        self.status_pub.publish(status)

    def new_grid(self):
        grid = OccupancyGrid()
        grid.header.frame_id = 'map'
        grid.header.stamp = self.node.get_clock().now().to_msg()
        grid.info.map_load_time = grid.header.stamp
        grid.info.width = grid.info.height = WIDTH
        grid.info.resolution = 0.1
        grid.info.origin.position.x = grid.info.origin.position.y = -4.0
        grid.info.origin.orientation.w = 1.0
        grid.data = [0] * (WIDTH * WIDTH)
        return grid

    def receive(self, msg):
        self.raw = msg
        self.raw_count += 1

    def mask(self, region='A'):
        grid = copy.deepcopy(self.map)
        grid.data = [0] * len(grid.data)
        ys = range(30, 50) if region == 'A' else range(12, 28)
        for y in ys:
            for x in range(38, 42):
                grid.data[y * WIDTH + x] = 100
        return grid

    def publish_evidence(self, mask, refresh=True):
        msg = copy.deepcopy(mask)
        if refresh:
            msg.header.stamp = self.node.get_clock().now().to_msg()
        self.evidence_pub.publish(msg)
        return msg

    def tick(self):
        if self.rviz_process is not None and self.rviz_process.poll() is not None:
            raise AssertionError(f'RViz exited early: {self.rviz_process.returncode}')
        now = time.monotonic()
        if now - self.last_send > 0.10:
            terrain = copy.deepcopy(self.map)
            terrain.header.stamp = self.node.get_clock().now().to_msg()
            terrain.data = [-1 if x == -1 else 0 for x in self.map.data]
            if self.dynamic:
                terrain.data[40 * WIDTH + 40] = 100
            self.terrain_pub.publish(terrain)
            if self.active is not None:
                self.publish_evidence(self.active)
            self.publish_visualization()
            self.last_send = now
        rclpy.spin_once(self.node, timeout_sec=0.02)

    def pump(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.tick()

    def cost(self, x, y):
        assert self.raw is not None
        assert self.raw.metadata.size_x == WIDTH and self.raw.metadata.size_y == WIDTH
        return self.raw.data[y * WIDTH + x]

    def route_exists(self, start=(20, 40), goal=(60, 40)):
        """Check the actual published master Costmap, not a synthetic mask."""
        queue = deque([start])
        seen = {start}
        while queue:
            x, y = queue.popleft()
            if (x, y) == goal:
                return True
            for q in ((x-1,y), (x+1,y), (x,y-1), (x,y+1)):
                if (0 <= q[0] < WIDTH and 0 <= q[1] < WIDTH and
                        q not in seen and self.cost(*q) < 254):
                    seen.add(q)
                    queue.append(q)
        return False

    def test_memory_on_global_costmap(self, baseline):
        ctx = MapContext('isolated-session-A', 'map', WIDTH, WIDTH, .1, -4., -4., 0., 0., 0.)
        memory = GlobalPassageMemory()
        proof = ConfirmedPassage(38, 42, 30, 50, 'map', 3,
                                 time.monotonic(), 'synthetic-confirmation-A')
        assert memory.observe(proof, ctx, time.monotonic())
        self.active = self.mask()
        self.active.data = [100 if i in memory.planning_cells(ctx) else 0
                            for i in range(WIDTH*WIDTH)]
        self.check('historical candidate opens actual global costmap',
                   lambda: self.cost(40, 40) == 0 and self.route_exists())
        # No new local verification arrives; the memory keeps its structure.
        self.pump(1.4)
        self.check('global route persists after local observation and layer TTL',
                   lambda: self.cost(40, 40) == 0 and self.route_exists())
        self.dynamic = True
        self.check('live obstacle vetoes remembered passage cell',
                   lambda: self.cost(40, 40) == 254)
        self.dynamic = False
        memory.obstacle_conflict()
        self.active = None
        assert not memory.planning_cells(ctx)
        self.check('conflict suspends memory and restores static wall',
                   lambda: list(self.raw.data) == baseline, settle=1.3)
        assert not memory.observe(proof, ctx, time.monotonic())
        second = ConfirmedPassage(38, 42, 30, 50, 'map', 3,
                                  time.monotonic(), 'synthetic-confirmation-B')
        assert memory.observe(second, ctx, time.monotonic())
        assert memory.planning_cells(ctx)
        assert not memory.planning_cells(MapContext('new-map-session', 'map',
            WIDTH, WIDTH, .1, -4., -4., 0., 0., 0.))
        self.check('map reload invalidates remembered passage',
                   lambda: list(self.raw.data) == baseline)

    def check(self, name, predicate, timeout=4.0, settle=0.3):
        self.phase = name
        # 至少等待新更新；不能拿命令前的旧 Costmap 假装撤销成功。
        before = self.raw_count
        self.pump(settle)
        end = time.monotonic() + timeout
        good = 0
        last_seen = self.raw_count
        while time.monotonic() < end:
            self.tick()
            if self.raw_count > before and self.raw_count != last_seen:
                last_seen = self.raw_count
                good = good + 1 if predicate() else 0
                if good >= 3:
                    self.passed.append(name)
                    print('PASS:', name, flush=True)
                    if self.pause > 0:
                        self.pump(self.pause)
                    return
        summary = None if self.raw is None else {'A': self.cost(40, 40), 'B': self.cost(40, 20)}
        raise AssertionError(f'{name}; latest costs={summary}')

    def command(self, value):
        self.command_pub.publish(String(data=value))

    def run(self):
        # 等待两个地图订阅者（StaticLayer + CorrectionLayer）及地形、证据订阅者。
        end = time.monotonic() + 12
        while time.monotonic() < end:
            self.tick()
            if (self.map_pub.get_subscription_count() >= 2 and
                    self.evidence_pub.get_subscription_count() >= 1 and
                    self.terrain_pub.get_subscription_count() >= 1):
                break
        else:
            raise AssertionError('Test harness discovery timed out')
        self.map_pub.publish(self.map)
        self.check('baseline static wall', lambda: self.cost(40, 40) == 254)
        baseline = list(self.raw.data)
        self.test_memory_on_global_costmap(baseline)
        self.active = self.mask('A')
        self.check('open A only; static wall elsewhere preserved',
                   lambda: self.cost(40, 40) == 0 and self.cost(40, 20) == 254)

        self.dynamic = True
        self.check('live obstacle veto and inflation',
                   lambda: self.cost(40, 40) == 254 and 0 < self.cost(41, 40) < 255)
        self.dynamic = False
        self.check('obstacle removed: fresh permission opens A again', lambda: self.cost(40, 40) == 0)

        self.active = self.mask('B')
        self.check('move evidence: restore A and open B',
                   lambda: self.cost(40, 40) == 254 and self.cost(40, 20) == 0)
        self.active = self.mask()
        self.active.data = [0] * len(self.active.data)
        self.check('explicit revoke restores full original costmap', lambda: list(self.raw.data) == baseline)

        self.active = self.mask()
        self.check('reopen before expiry test', lambda: self.cost(40, 40) == 0)
        self.active = None
        self.check('publisher stops: TTL restores full map including inflation',
                   lambda: list(self.raw.data) == baseline, settle=1.3)

        self.active = self.mask()
        self.check('reopen before reset', lambda: self.cost(40, 40) == 0)
        self.active = None
        self.command('reset')
        self.check('reset restores wall without new /map', lambda: list(self.raw.data) == baseline)

        self.active = self.mask()
        self.check('reopen before disable', lambda: self.cost(40, 40) == 0)
        self.active = None
        self.command('disable')
        self.check('disable restores wall', lambda: list(self.raw.data) == baseline)
        self.command('enable')
        self.check('enable does not resurrect old permission', lambda: list(self.raw.data) == baseline)

        # 无效消息均拒绝，且不能保持上一个有效许可。
        for kind in ('wrong_frame', 'wrong_origin', 'bad_size', 'bad_value',
                     'negative_stamp', 'invalid_nanosecond', 'expired', 'future', 'wrong_epoch'):
            self.active = self.mask()
            self.check(f'reopen before {kind}', lambda: self.cost(40, 40) == 0)
            self.active = None
            invalid = self.mask()
            invalid.header.stamp = self.node.get_clock().now().to_msg()
            if kind == 'wrong_frame':
                invalid.header.frame_id = 'odom'
            elif kind == 'wrong_origin':
                invalid.info.origin.position.x += 1.0
            elif kind == 'bad_size':
                invalid.data = invalid.data[:-1]
            elif kind == 'bad_value':
                invalid.data[0] = 50
            elif kind == 'negative_stamp':
                invalid.header.stamp.sec = -1
            elif kind == 'invalid_nanosecond':
                invalid.header.stamp.nanosec = 1000000000
            elif kind == 'expired':
                invalid.header.stamp.sec -= 5
            elif kind == 'future':
                invalid.header.stamp.sec += 5
            elif kind == 'wrong_epoch':
                invalid.info.map_load_time.sec -= 1
            self.publish_evidence(invalid, refresh=False)
            self.check(f'reject {kind} and restore', lambda: list(self.raw.data) == baseline,
                       timeout=0.5, settle=0.15)

        self.active = self.mask()
        self.check('open before replay test', lambda: self.cost(40, 40) == 0)
        self.active = None
        first = self.publish_evidence(self.mask())
        self.pump(0.10)
        self.publish_evidence(first, refresh=False)
        self.check('reject duplicate stamp', lambda: list(self.raw.data) == baseline,
                   timeout=0.5, settle=0.15)

        self.active = self.mask()
        for y in range(60, 65):
            for x in range(10, 15):
                self.active.data[y * WIDTH + x] = 100
        self.check('test evidence never clears unknown',
                   lambda: self.cost(40, 40) == 0 and self.cost(12, 62) == 255)

        self.active = None
        self.map.info.map_load_time = self.node.get_clock().now().to_msg()
        self.map.header.stamp = self.map.info.map_load_time
        self.map_pub.publish(self.map)
        self.check('map version change revokes permission', lambda: list(self.raw.data) == baseline)

        self.command('wrong_order')
        self.active = self.mask()
        self.check('incorrect plugin order refuses clearing', lambda: list(self.raw.data) == baseline,
                   settle=0.5)
        self.command('restore_order')
        self.check('restored order with fresh evidence works', lambda: self.cost(40, 40) == 0)
        self.active = None
        self.check('final timeout leaves no hole', lambda: list(self.raw.data) == baseline, settle=1.3)

        self.active = self.mask()
        self.check('open before new static contents', lambda: self.cost(40, 40) == 0)
        self.active = None
        self.map.data[20 * WIDTH + 20] = 100
        self.map.info.map_load_time = self.node.get_clock().now().to_msg()
        self.map.header.stamp = self.map.info.map_load_time
        self.map_pub.publish(self.map)
        self.check('restore CURRENT map, not cached old contents',
                   lambda: self.cost(40, 40) == 254 and self.cost(20, 20) == 254)
        new_baseline = list(self.raw.data)
        self.active = self.mask()
        self.check('fresh evidence works with new map contents',
                   lambda: self.cost(40, 40) == 0 and self.cost(20, 20) == 254)
        self.active = None
        self.check('new map retained after evidence timeout',
                   lambda: list(self.raw.data) == new_baseline, settle=1.3)

        self.active = self.mask()
        self.check('open before map origin moves', lambda: self.cost(40, 40) == 0)
        self.active = None
        old_geometry = self.mask()
        self.map.info.origin.position.x += 0.5
        self.map.info.map_load_time = self.node.get_clock().now().to_msg()
        self.map.header.stamp = self.map.info.map_load_time
        self.map_pub.publish(self.map)
        self.check('map origin move revokes old permission',
                   lambda: self.raw.metadata.origin.position.x == -3.5 and self.cost(40, 40) == 254)
        self.publish_evidence(old_geometry)
        self.check('old geometry cannot open shifted map', lambda: self.cost(40, 40) == 254)

        assert self.map_pub.get_subscription_count() >= 2
        assert self.preview_count > 0, 'No safe preview received'
        if self.rviz_process is not None:
            assert self.rviz_process.poll() is None, 'RViz failed during fault injection'
            assert self.preview_pub.get_subscription_count() >= 2, 'RViz did not subscribe to safe preview'
            assert self.status_pub.get_subscription_count() >= 1, 'RViz did not subscribe to cost labels'
        assert self.node.count_publishers(NS + '/map') == 1, '/map was unexpectedly republished'
        topics = self.node.get_topic_names_and_types()
        # RViz 自身会订阅 TF；发现话题不等于存在发布者。仍禁止任何控制/TF 发布者。
        assert not any(name.endswith(('/cmd_vel', '/cmd_vel_smoothed', '/tf', '/tf_static'))
                       and self.node.count_publishers(name) > 0
                       for name, _ in topics), 'Unexpected control/TF publisher in test domain'
        print(f'ALL PASS: {len(self.passed)} checkpoints; raw map unchanged; no motion control', flush=True)
        print(f'PASS: {self.preview_count} safe preview messages validated', flush=True)
        if self.rviz_process is not None:
            print('PASS: RViz survived full fault injection with preview and cost labels enabled', flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--harness', help='自动启动独立图层宿主的可执行文件')
    parser.add_argument('--config', help='测试宿主 YAML')
    parser.add_argument('--pause', type=float, default=0.0, help='检查间暂停秒数，便于 RViz 观察')
    parser.add_argument('--rviz-config', help='启动 RViz 完整回归；强制打开安全预览验证隔离')
    args, ros_args = parser.parse_known_args()
    process = None
    log = None
    scenario = None
    rviz_process = None
    rviz_log = None
    rviz_config = None
    # 自运行测试用独立域，启动文件方式由 launch 设置；不继承实车域。
    if args.harness:
        os.environ['ROS_DOMAIN_ID'] = '218'
        os.environ['ROS_LOCALHOST_ONLY'] = '1'
        # 验证误启动防护：没有显式授权、或进入其他命名空间时，宿主必须拒绝启动。
        guards = [([], 'Explicit test opt-in'),
                  (['--ros-args', '-r', '__ns:=/not_the_test_namespace'], 'only run in')]
        for extra, expected in guards:
            result = subprocess.run([args.harness] + extra, capture_output=True,
                                    text=True, timeout=8)
            if result.returncode == 0 or expected not in result.stdout + result.stderr:
                print('FAIL: startup isolation guard', extra, result.stderr, file=sys.stderr)
                return 1
            print('PASS: startup isolation guard', expected, flush=True)
        log = open(Path.cwd() / 'fusion_harness_test.log', 'w', encoding='utf-8')
        process = subprocess.Popen([args.harness, '--ros-args', '--params-file', args.config],
                                   stdout=log, stderr=subprocess.STDOUT)
    rclpy.init(args=ros_args)
    try:
        if args.rviz_config:
            import yaml
            from ament_index_python.packages import get_package_prefix
            with open(args.rviz_config, encoding='utf-8') as stream:
                config = yaml.safe_load(stream)
            for display in config['Visualization Manager']['Displays']:
                topic = display.get('Topic', {}).get('Value', '')
                assert topic != NS + '/test_evidence', 'RViz must not subscribe to malformed input'
                if topic == NS + '/evidence_preview':
                    display['Enabled'] = display['Value'] = True
            rviz_config = tempfile.NamedTemporaryFile(mode='w', suffix='.rviz', encoding='utf-8')
            yaml.safe_dump(config, rviz_config)
            rviz_config.flush()
            rviz_log = open(Path.cwd() / 'fusion_rviz_test.log', 'w', encoding='utf-8')
            rviz_process = subprocess.Popen(
                [str(Path(get_package_prefix('rviz2')) / 'lib/rviz2/rviz2'), '-d', rviz_config.name],
                stdout=rviz_log, stderr=subprocess.STDOUT)
        scenario = Scenario(pause=args.pause, rviz_process=rviz_process)
        if rviz_process is not None:
            deadline = time.monotonic() + 20
            while scenario.preview_pub.get_subscription_count() < 2:
                if time.monotonic() > deadline:
                    raise AssertionError('RViz preview discovery timed out; see fusion_rviz_test.log')
                scenario.tick()
        scenario.run()
        return 0
    except Exception as error:
        print(f'FAIL: {error}', file=sys.stderr, flush=True)
        return 1
    finally:
        if rviz_process:
            if rviz_process.poll() is None:
                rviz_process.send_signal(signal.SIGINT)
                try:
                    rviz_process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    rviz_process.terminate()
                    rviz_process.wait(timeout=5)
            rviz_log.close()
        if rviz_config:
            rviz_config.close()
        if scenario:
            scenario.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        if process:
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    process.terminate()
                    process.wait(timeout=5)
            log.close()


if __name__ == '__main__':
    sys.exit(main())
