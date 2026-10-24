#!/usr/bin/env python3
"""Runtime wiring, distinct from artificial proof/virtual-body demonstrations.

No node in this file opens hardware. The optional launch-owned serial process
must be the sole consumer of chassis_cmd_vel. MCU command-loss protection is
still required: host software cannot stop hardware after a power loss.
"""
import copy
import math
import sys
import time

import rclpy
from action_msgs.msg import GoalStatusArray
from geometry_msgs.msg import PointStamped, PoseStamped, Twist, TwistStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.time import Time
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformException, TransformListener

from isolated_entry_gate import IsolatedEntryGate, seconds
from live_memory_shadow import LiveMemoryShadow
from progressive_probe_coordinator import ProgressiveProbeCoordinator


def planar(command):
    return (all(math.isfinite(v) for v in (command.linear.x, command.linear.y,
             command.linear.z, command.angular.x, command.angular.y, command.angular.z))
            and command.linear.z == command.angular.x == command.angular.y == 0)


def corridor_blocked(grid, pose, command, radius, horizon=1.0, *, length=None, width=None,
                     padding=0.01, center_x=0.0, center_y=0.0):
    """Sweep the actual rectangle through translation/turns; retain legacy circle API.

    Rectangle mode checks lethal source cells, not the inscribed inflation band:
    applying a complete footprint to that band would count the body size twice.
    """
    if (grid.header.frame_id != 'map' or grid.info.resolution <= 0 or
            len(grid.data) != grid.info.width*grid.info.height or not grid.data or
            not planar(command) or not math.isfinite(horizon) or horizon <= 0):
        return True
    q = grid.info.origin.orientation
    if abs(q.x)+abs(q.y)+abs(q.z) > 1e-5 or abs(abs(q.w)-1) > 1e-5:
        return True
    p, q = pose.position, pose.orientation
    if not all(math.isfinite(v) for v in (p.x, p.y, q.x, q.y, q.z, q.w)):
        return True
    rectangle = length is not None or width is not None
    if rectangle:
        if (not all(isinstance(v, (int, float)) and math.isfinite(v)
                    for v in (length, width, padding, center_x, center_y)) or
                min(length, width) <= 0 or padding < 0):
            return True
        hx, hy = length/2+padding, width/2+padding
        radius = math.hypot(hx+abs(center_x), hy+abs(center_y))
    elif not math.isfinite(radius) or radius <= 0:
        return True
    yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
    c0, s0 = math.cos(yaw), math.sin(yaw)
    vx, vy, omega = command.linear.x, command.linear.y, command.angular.z
    r, ox, oy = grid.info.resolution, grid.info.origin.position.x, grid.info.origin.position.y
    speed = math.hypot(vx, vy)+(abs(omega)*radius if rectangle else 0.)
    steps = max(1, math.ceil(speed*horizon/(r/2)))
    for step in range(steps+1):
        dt = horizon*step/steps
        theta = omega*dt
        if abs(omega) < 1e-8:
            lx, ly = vx*dt, vy*dt
        else:
            lx = (vx*math.sin(theta)+vy*(math.cos(theta)-1))/omega
            ly = (vx*(1-math.cos(theta))+vy*math.sin(theta))/omega
        x, y = p.x+c0*lx-s0*ly, p.y+s0*lx+c0*ly
        c, s = math.cos(yaw+theta), math.sin(yaw+theta)
        if rectangle:
            x, y = x+c*center_x-s*center_y, y+s*center_x+c*center_y
            span_x = abs(c)*hx+abs(s)*hy
            span_y = abs(s)*hx+abs(c)*hy
        else:
            span_x = span_y = radius+r*math.sqrt(2)/2
        for iy in range(math.floor((y-span_y-oy)/r), math.floor((y+span_y-oy)/r)+1):
            for ix in range(math.floor((x-span_x-ox)/r), math.floor((x+span_x-ox)/r)+1):
                dx, dy = ox+(ix+.5)*r-x, oy+(iy+.5)*r-y
                if rectangle:
                    # Separating-axis test for an oriented rectangle and square cell.
                    half = r/2
                    if (abs(dx) > span_x+half or abs(dy) > span_y+half or
                            abs(c*dx+s*dy) > hx+half*(abs(c)+abs(s)) or
                            abs(-s*dx+c*dy) > hy+half*(abs(c)+abs(s))):
                        continue
                elif math.hypot(dx, dy) > span_x:
                    continue
                if not (0 <= ix < grid.info.width and 0 <= iy < grid.info.height):
                    return True
                value = grid.data[iy*grid.info.width+ix]
                if value < 0 or value >= (100 if rectangle else 99):
                    return True
    return False


class RuntimeIO(Node):
    def __init__(self):
        super().__init__('passage_runtime_io')
        if self.get_namespace() != '/passage':
            raise ValueError('Runtime IO requires /passage namespace')
        self.motion = self.declare_parameter('enable_motion', False).value
        length = self.declare_parameter('robot_length', .62).value
        width = self.declare_parameter('robot_width', .58).value
        self.padding = self.declare_parameter('footprint_padding', .01).value
        self.center_x = self.declare_parameter('footprint_center_x', -.01).value
        self.center_y = self.declare_parameter('footprint_center_y', 0.).value
        if not all(math.isfinite(v) and abs(v) < 1 for v in (self.center_x, self.center_y)):
            raise ValueError('Invalid footprint center')
        if not all(math.isfinite(v) and 0 < v < 3 for v in (length, width)):
            raise ValueError('Invalid body dimensions')
        self.radius = math.hypot(length, width)/2
        if not math.isfinite(self.padding) or not 0 <= self.padding <= .1:
            raise ValueError('Invalid footprint padding')
        self.length, self.width = length, width
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.grid = self.odom = self.command = self.authorized = None
        self.measured_speed = None
        self.grid_at = self.odom_at = self.command_at = self.authorized_at = 0.
        self.active = False
        self.action_at = 0.
        self.desired_pub = self.create_publisher(TwistStamped, '/passage/cmd_vel_desired', 1)
        self.pose_pub = self.create_publisher(Odometry, '/passage/odom_map', 1)
        self.obstacle_pub = self.create_publisher(Bool, '/passage/live_obstacle', 1)
        self.output = self.create_publisher(Twist, '/passage/chassis_cmd_vel', 1)
        self.path_pub = self.create_publisher(Path, '/passage/planned_path', 1)
        self.goal_pub = self.create_publisher(PoseStamped, '/passage/desired_goal', 1)
        self.create_subscription(Twist, '/passage/nav_cmd_vel', self.on_command, 1)
        self.create_subscription(TwistStamped, '/passage/cmd_vel_authorized', self.on_authorized, 1)
        self.create_subscription(OccupancyGrid, '/local_costmap/costmap', self.on_grid, 1)
        self.create_subscription(Odometry, '/Odometry', self.on_odom, 1)
        self.create_subscription(Path, '/plan', self.on_path, 1)
        self.create_subscription(PoseStamped, '/goal_pose', self.goal_pub.publish, 1)
        self.create_subscription(PointStamped, '/clicked_point', self.on_probe_point, 1)
        self.create_subscription(GoalStatusArray, '/navigate_to_pose/_action/status', self.on_action, 1)
        self.create_timer(.05, self.tick)

    def on_action(self, msg):
        self.active = any(s.status in (1, 2) for s in msg.status_list)
        # Status is event driven, NOT treated as a periodic heartbeat. Command,
        # path, pose and obstacle deadlines remain mandatory independently.
        if not self.active:
            self.command = self.authorized = None

    def on_grid(self, msg):
        self.grid, self.grid_at = msg, time.monotonic()

    def on_odom(self, msg):
        self.measured_speed = None
        if self.odom is not None and msg.header.frame_id == self.odom.header.frame_id:
            dt = seconds(msg.header.stamp)-seconds(self.odom.header.stamp)
            a, b = msg.pose.pose.position, self.odom.pose.pose.position
            if 0 < dt <= .5:
                speed = math.hypot(a.x-b.x, a.y-b.y)/dt
                reported = math.hypot(msg.twist.twist.linear.x, msg.twist.twist.linear.y)
                # Some LIO versions leave Twist zero. Derive a conservative
                # speed from stamped odometry instead of assuming stationary.
                if math.isfinite(speed) and math.isfinite(reported) and max(speed, reported) <= 2.:
                    self.measured_speed = max(speed, reported)
        self.odom, self.odom_at = msg, time.monotonic()

    def on_path(self, msg):
        self.path_pub.publish(msg)  # preserve original stamp; never renew an old path
        if msg.header.frame_id == 'map' and msg.poses:
            goal = copy.deepcopy(msg.poses[-1])
            goal.header = msg.header
            self.goal_pub.publish(goal)

    def on_probe_point(self, msg):
        """RViz Publish Point selects a read-only candidate when no path exists."""
        if msg.header.frame_id != 'map' or not all(math.isfinite(v) for v in
                (msg.point.x, msg.point.y)):
            return
        try:
            tf = self.buffer.lookup_transform('map', 'base_link', Time())
            now = self.get_clock().now().nanoseconds/1e9
            if not 0 <= now-seconds(tf.header.stamp) <= .2:
                return
        except TransformException:
            return
        p = tf.transform.translation
        direction = math.atan2(msg.point.y-p.y, msg.point.x-p.x)
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = msg.point.x
        goal.pose.position.y = msg.point.y
        goal.pose.orientation.z = math.sin(direction/2)
        goal.pose.orientation.w = math.cos(direction/2)
        self.goal_pub.publish(goal)

    def on_command(self, msg):
        output = TwistStamped()
        output.header.frame_id = 'base_link'
        output.header.stamp = self.get_clock().now().to_msg()
        if planar(msg) and self.active:
            output.twist = copy.deepcopy(msg)
            norm = math.hypot(msg.linear.x, msg.linear.y)
            if norm > .2:
                output.twist.linear.x *= .2/norm
                output.twist.linear.y *= .2/norm
            output.twist.angular.z = max(-.4, min(.4, msg.angular.z))
        self.command, self.command_at = output.twist, time.monotonic()
        self.desired_pub.publish(output)

    def on_authorized(self, msg):
        self.authorized, self.authorized_at = msg, time.monotonic()

    def tick(self):
        steady = time.monotonic()
        now = self.get_clock().now().nanoseconds/1e9
        blocked = True
        try:
            tf = self.buffer.lookup_transform('map', 'base_link', Time())
            if not 0 <= now-seconds(tf.header.stamp) <= .2:
                raise ValueError('Stale TF')
            if (self.odom is None or self.measured_speed is None or steady-self.odom_at > .2 or
                    not 0 <= now-seconds(self.odom.header.stamp) <= .2):
                raise ValueError('Stale odometry')
            pose = Odometry()
            pose.header = tf.header
            pose.child_frame_id = 'base_link'
            pose.pose.pose.position.x = tf.transform.translation.x
            pose.pose.pose.position.y = tf.transform.translation.y
            pose.pose.pose.position.z = tf.transform.translation.z
            pose.pose.pose.orientation = tf.transform.rotation
            pose.twist = copy.deepcopy(self.odom.twist)
            pose.twist.twist.linear.x = self.measured_speed
            pose.twist.twist.linear.y = 0.
            self.pose_pub.publish(pose)
            if (self.grid is not None and steady-self.grid_at <= .5 and
                    0 <= now-seconds(self.grid.header.stamp) <= .5 and
                    self.command is not None and steady-self.command_at <= .2):
                blocked = corridor_blocked(self.grid, pose.pose.pose, self.command, self.radius,
                    length=self.length, width=self.width, padding=self.padding,
                    center_x=self.center_x, center_y=self.center_y)
        except (TransformException, ValueError, OverflowError):
            pass
        self.obstacle_pub.publish(Bool(data=blocked))
        output = Twist()
        if (self.motion and self.active and not blocked and self.authorized is not None
                and steady-self.authorized_at <= .15 and steady-self.command_at <= .2
                and self.authorized.header.frame_id == 'base_link'
                and 0 <= now-seconds(self.authorized.header.stamp) <= .15
                and planar(self.authorized.twist)):
            output = self.authorized.twist
        self.output.publish(output)


def main():
    component = sys.argv[1]
    rclpy.init(args=sys.argv[2:])
    factories = {'memory': lambda: LiveMemoryShadow(runtime=True),
                 'gate': lambda: IsolatedEntryGate(runtime=True),
                 'probe': lambda: ProgressiveProbeCoordinator(runtime=True), 'io': RuntimeIO}
    node = factories[component]()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if isinstance(node, RuntimeIO) and rclpy.ok():
            node.output.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
