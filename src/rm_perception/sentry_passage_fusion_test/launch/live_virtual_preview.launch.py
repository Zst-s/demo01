"""Overlay live /map with virtual-body passage evidence; never launch motion control."""
import uuid

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    share = FindPackageShare('sentry_passage_fusion_test')
    sim_time = LaunchConfiguration('use_sim_time')
    harness = Node(
        package='sentry_passage_fusion_test', executable='fusion_harness',
        parameters=[PathJoinSubstitution([share, 'config', 'production_preview.yaml']),
                    {'use_sim_time': sim_time}], output='screen')
    memory = Node(
        package='sentry_passage_fusion_test', executable='live_memory_shadow.py',
        namespace='passage_fusion_test',
        parameters=[{'use_sim_time': sim_time,
                     'map_session_id': 'virtual-preview-' + uuid.uuid4().hex,
                     'record_path': '/tmp/zzs_virtual_preview_not_saved.json',
                     'evidence_frame': 'odom',
                     'publish_test_mask': True,
                     'accept_virtual_preview': True}], output='screen')
    rviz = Node(
        package='rviz2', executable='rviz2',
        arguments=['-d', PathJoinSubstitution([share, 'rviz', 'live_virtual_preview.rviz'])],
        condition=IfCondition(LaunchConfiguration('rviz')), output='screen')
    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        harness, memory, rviz,
        RegisterEventHandler(OnProcessExit(
            target_action=harness,
            on_exit=[EmitEvent(event=Shutdown(reason='Preview costmap host stopped'))])),
        RegisterEventHandler(OnProcessExit(
            target_action=memory,
            on_exit=[EmitEvent(event=Shutdown(reason='Preview evidence memory stopped'))])),
    ])
