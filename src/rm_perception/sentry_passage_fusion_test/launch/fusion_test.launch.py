"""独立合成实验；不 include 原 bringup，不启动导航或底盘。"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, SetEnvironmentVariable
from launch.actions import EmitEvent
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    share = FindPackageShare('sentry_passage_fusion_test')
    harness = Node(
        package='sentry_passage_fusion_test', executable='fusion_harness',
        parameters=[PathJoinSubstitution([share, 'config', 'fusion_test.yaml'])],
        output='screen')
    scenario = Node(
        package='sentry_passage_fusion_test', executable='fusion_scenario.py',
        arguments=['--pause', LaunchConfiguration('pause')], output='screen')
    rviz = Node(
        package='rviz2', executable='rviz2',
        arguments=['-d', PathJoinSubstitution([share, 'rviz', 'fusion_test.rviz'])],
        condition=IfCondition(LaunchConfiguration('rviz')), output='screen')
    # 仅修改本 launch 子进程环境；Domain 218 用于隔离测试，请勿与实车共用。
    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('pause', default_value='1.0'),
        SetEnvironmentVariable('ROS_DOMAIN_ID', '218'),
        SetEnvironmentVariable('ROS_LOCALHOST_ONLY', '1'),
        harness, scenario, rviz,
        RegisterEventHandler(OnProcessExit(
            target_action=scenario,
            on_exit=[EmitEvent(event=Shutdown(reason='Synthetic scenario finished; inspect ALL PASS / FAIL'))])),
        RegisterEventHandler(OnProcessExit(
            target_action=harness,
            on_exit=[EmitEvent(event=Shutdown(reason='Test harness stopped'))])),
        RegisterEventHandler(OnProcessExit(
            target_action=rviz,
            on_exit=[EmitEvent(event=Shutdown(
                reason='RViz exited; without ALL PASS the demonstration is incomplete'))])),
    ])
