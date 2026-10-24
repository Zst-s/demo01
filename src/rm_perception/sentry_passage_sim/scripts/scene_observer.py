#!/usr/bin/env python3
"""Visualization and manual diagnostic probes only; never publishes motion."""
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PointStamped, PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import TransformBroadcaster


class SceneObserver(Node):
    def __init__(self):
        super().__init__('passage_scene_observer')
        self.tf = TransformBroadcaster(self)
        self.odom_sub = self.create_subscription(Odometry, '/passage_sim/sensor_odom', self.odom, qos_profile_sensor_data)
        self.click_sub = self.create_subscription(PointStamped, '/passage_sim/clicked_point', self.probe, 1)
        self.probe_pub = self.create_publisher(PoseStamped, '/terrain/clearance/probe', 1)
        self.scene_pub = self.create_publisher(MarkerArray, '/passage_sim/scene_labels', 1)
        self.timer = self.create_timer(1.0, self.labels)
        self.get_logger().info('隔离感知实验：Publish Point 点击地面可检查直线体积；不发布速度或通行许可。')

    def odom(self, msg):
        if msg.header.frame_id != 'world':
            return
        tf = TransformStamped()
        tf.header = msg.header
        tf.child_frame_id = 'passage_lidar'
        tf.transform.translation.x = msg.pose.pose.position.x
        tf.transform.translation.y = msg.pose.pose.position.y
        tf.transform.translation.z = msg.pose.pose.position.z
        tf.transform.rotation = msg.pose.pose.orientation
        self.tf.sendTransform(tf)

    def probe(self, msg):
        if msg.header.frame_id != 'world':
            self.get_logger().warning('探针必须使用 world 坐标系')
            return
        probe = PoseStamped()
        probe.header.frame_id = 'world'
        probe.header.stamp = self.get_clock().now().to_msg()
        probe.pose.position.x = msg.point.x
        probe.pose.position.y = msg.point.y
        # 合成地板顶面 z=0；选择顶面下方体素。不是实车自动地面估计。
        # Calibrated height of direct floor returns in the lab's voxel map.
        probe.pose.position.z = -0.025
        probe.pose.orientation.w = 1.0
        self.probe_pub.publish(probe)

    def labels(self):
        out = MarkerArray()
        descriptions = [(0.0, 'WIDE: 1.20 m x 0.95 m'),
                        (2.0, 'NARROW: 0.30 m x 0.95 m'),
                        (-2.0, 'LOW: 1.20 m x 0.50 m'),
                        (4.0, 'SOLID WALL')]
        for index, (y, label) in enumerate(descriptions):
            marker = Marker()
            marker.header.frame_id = 'world'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'SCENE_TRUTH_NOT_PERCEPTION'
            marker.id = index
            marker.type = Marker.TEXT_VIEW_FACING
            marker.pose.position.x = 2.6
            marker.pose.position.y = y
            marker.pose.position.z = 1.5
            marker.pose.orientation.w = 1.0
            marker.scale.z = 0.10
            marker.color.r = marker.color.g = marker.color.b = marker.color.a = 1.0
            marker.text = label.replace(': ', '\n') + '\nTRUTH'
            marker.lifetime.sec = 2
            out.markers.append(marker)
        self.scene_pub.publish(out)


def main():
    rclpy.init()
    node = SceneObserver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
