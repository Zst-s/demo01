
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, Point, Quaternion
import sys
import time
import os
import subprocess

class InitialPosePublisher(Node):

    def __init__(self):
        super().__init__('initial_pose_publisher')
        self.publisher_ = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.start_time = self.get_clock().now()
        self.declare_parameter('x', 0.0)
        self.declare_parameter('y', 0.0)
        self.declare_parameter('z', 0.0)
        self.declare_parameter('qx', 0.0)
        self.declare_parameter('qy', 0.0)
        self.declare_parameter('qz', 0.0)
        self.declare_parameter('qw', 1.0)
        self.declare_parameter('frame_id', 'map')

        # 读取文件中的参数值
        self.read_parameters_from_file()

    def read_parameters_from_file(self):
        translation_path = "/home/doublegun/translation.txt"
        rotation_path = "/home/doublegun/rotation.txt"

        # 检查文件是否为空
        if not os.path.getsize(translation_path) > 0:
            self.get_logger().info(f"{translation_path} is empty. Using default values.")
            self.set_parameters([
                rclpy.parameter.Parameter('x', rclpy.Parameter.Type.DOUBLE, 0.0),
                rclpy.parameter.Parameter('y', rclpy.Parameter.Type.DOUBLE, 0.0),
                rclpy.parameter.Parameter('z', rclpy.Parameter.Type.DOUBLE, 0.0),
            ])
        else:
            with open(translation_path, 'r') as trans_file:
                trans_data = trans_file.readline().strip().split(',')
                self.set_parameters([
                    rclpy.parameter.Parameter('x', rclpy.Parameter.Type.DOUBLE, float(trans_data[0].split('=')[1])),
                    rclpy.parameter.Parameter('y', rclpy.Parameter.Type.DOUBLE, float(trans_data[1].split('=')[1])),
                    rclpy.parameter.Parameter('z', rclpy.Parameter.Type.DOUBLE, float(trans_data[2].split('=')[1])),
                ])

        # 检查文件是否为空
        if not os.path.getsize(rotation_path) > 0:
            self.get_logger().info(f"{rotation_path} is empty. Using default values.")
            self.set_parameters([
                rclpy.parameter.Parameter('qx', rclpy.Parameter.Type.DOUBLE, 0.0),
                rclpy.parameter.Parameter('qy', rclpy.Parameter.Type.DOUBLE, 0.0),
                rclpy.parameter.Parameter('qz', rclpy.Parameter.Type.DOUBLE, 0.0),
                rclpy.parameter.Parameter('qw', rclpy.Parameter.Type.DOUBLE, 1.0),
            ])
        else:
            with open(rotation_path, 'r') as rot_file:
                rot_data = rot_file.readline().strip().split(',')
                self.set_parameters([
                    rclpy.parameter.Parameter('qx', rclpy.Parameter.Type.DOUBLE, float(rot_data[0].split('=')[1])),
                    rclpy.parameter.Parameter('qy', rclpy.Parameter.Type.DOUBLE, float(rot_data[1].split('=')[1])),
                    rclpy.parameter.Parameter('qz', rclpy.Parameter.Type.DOUBLE, float(rot_data[2].split('=')[1])),
                    rclpy.parameter.Parameter('qw', rclpy.Parameter.Type.DOUBLE, float(rot_data[3].split('=')[1])),
                ])

    def timer_callback(self):
        current_time = self.get_clock().now()
        elapsed_time = (current_time - self.start_time).nanoseconds / 1e9

        if elapsed_time > 20.0:
            self.timer.cancel()
            self.get_logger().info('Stopping publisher and killing nav processes after 20 seconds')
            self.kill_nav_processes()
            rclpy.shutdown()
            sys.exit(0)

        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value

        msg.pose.pose.position = Point()
        msg.pose.pose.position.x = self.get_parameter('x').get_parameter_value().double_value
        msg.pose.pose.position.y = self.get_parameter('y').get_parameter_value().double_value
        msg.pose.pose.position.z = self.get_parameter('z').get_parameter_value().double_value

        msg.pose.pose.orientation = Quaternion()
        msg.pose.pose.orientation.x = self.get_parameter('qx').get_parameter_value().double_value
        msg.pose.pose.orientation.y = self.get_parameter('qy').get_parameter_value().double_value
        msg.pose.pose.orientation.z = self.get_parameter('qz').get_parameter_value().double_value
        msg.pose.pose.orientation.w = self.get_parameter('qw').get_parameter_value().double_value

        # Set covariance (example: identity matrix)
        msg.pose.covariance = [0.0] * 36
        msg.pose.covariance[0] = 0.1
        msg.pose.covariance[7] = 0.1
        msg.pose.covariance[14] = 0.1
        msg.pose.covariance[21] = 0.1
        msg.pose.covariance[28] = 0.1
        msg.pose.covariance[35] = 0.1

        self.publisher_.publish(msg)
        self.get_logger().info('Publishing initial pose')

    def kill_nav_processes(self):
        # 使用 ps aux | grep 'nav' 查找包含 'nav' 的进程 ID
        result = subprocess.Popen(['ps', 'aux'], stdout=subprocess.PIPE)
        grep_result = subprocess.Popen(['grep', 'py_tools'], stdin=result.stdout, stdout=subprocess.PIPE)
        result.stdout.close()
        output, _ = grep_result.communicate()

        # 解析输出并获取进程 ID
        for line in output.splitlines():
            parts = line.split()
            pid = parts[1]
            try:
                subprocess.run(['kill', '-9', pid])
                self.get_logger().info(f'Killed process {pid}')
            except Exception as e:
                self.get_logger().error(f'Failed to kill process {pid}: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = InitialPosePublisher()

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()