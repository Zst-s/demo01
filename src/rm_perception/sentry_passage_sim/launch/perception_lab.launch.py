"""Isolated perception lab. Ground-truth pose, not a LIO/navigation test."""
import os
import socket
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable, OpaqueFunction, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def ensure_lab_port_unused(context):
    with socket.socket() as sock:
        sock.settimeout(0.5)
        if sock.connect_ex(('127.0.0.1', 11349)) == 0:
            raise RuntimeError('实验端口 11349 已占用。请先在原实验终端 Ctrl+C，勿重复启动同一场景。')
    return []


def generate_launch_description():
    share = get_package_share_directory('sentry_passage_sim')
    readiness = Node(package='sentry_passage_sim', executable='check_lab.py',
                     arguments=['--startup-only', '--mobile', LaunchConfiguration('mobile')],
                     output='screen')
    def readiness_exited(event, context):
        if event.returncode != 0:
            return [EmitEvent(event=Shutdown(reason='感知实验启动检查失败；请查看 missing 和 last_status，不能把无数据当作正常。'))]
        return []
    return LaunchDescription([
        OpaqueFunction(function=ensure_lab_port_unused),
        # Keep every child (including Gazebo's ROS plugins) in the caller's
        # isolated test domain. The default remains 219 for the normal lab
        # commands, while integration tests may choose a fresh domain.
        SetEnvironmentVariable('ROS_DOMAIN_ID', os.environ.get('ROS_DOMAIN_ID', '219')),
        SetEnvironmentVariable('ROS_LOCALHOST_ONLY', '1'),
        SetEnvironmentVariable('GAZEBO_MASTER_URI', 'http://127.0.0.1:11349'),
        SetEnvironmentVariable('GAZEBO_IP', '127.0.0.1'),
        SetEnvironmentVariable('GAZEBO_MODEL_DATABASE_URI', ''),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('mobile', default_value='false'),
        DeclareLaunchArgument('world_file', default_value=os.path.join(share, 'worlds', 'passages.world')),
        DeclareLaunchArgument('config_file', default_value=os.path.join(share, 'config', 'rog_map_lab.yaml')),
        RegisterEventHandler(OnProcessExit(target_action=readiness, on_exit=readiness_exited)),
        ExecuteProcess(cmd=['gzserver', '--verbose', '-s', 'libgazebo_ros_init.so',
                            '-s', 'libgazebo_ros_factory.so',
                            LaunchConfiguration('world_file')], output='screen'),
        ExecuteProcess(cmd=['gzclient'], condition=IfCondition(LaunchConfiguration('gui'))),
        Node(package='sentry_terrain_bridge', executable='terrain_node', name='sentry_terrain',
             parameters=[LaunchConfiguration('config_file')], output='screen'),
        Node(package='sentry_passage_sim', executable='scene_observer.py',
             parameters=[{'use_sim_time': True}], output='screen'),
        readiness,
        Node(package='rviz2', executable='rviz2', parameters=[{'use_sim_time': True}],
             arguments=['-d', os.path.join(share, 'rviz', 'perception_lab.rviz')],
             condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
