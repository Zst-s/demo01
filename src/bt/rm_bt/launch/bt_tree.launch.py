import os

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description() -> LaunchDescription:
    ld = LaunchDescription()

    package_path = get_package_share_directory('rm_bt')
    # 新决策树：基于独立计时器的时间导航（behavior_tree/simple_timer_navigation.xml）
    config_path = os.path.join(package_path, 'config', 'simple_timer_nav.yaml')

    tree_exec_node = Node(
        package='rm_bt',
        executable='rm_bt',
        name='tree_exec_node',
        output='screen',
        parameters=[config_path],
    )

    bridge = Node(
        package='rm_bt',
        executable='goal_pose_bridge',
        name='goal_pose_bridge',
        parameters=[{'input_topic': 'goal_pose', 'action_name': 'navigate_to_pose'}],
    )

    ld.add_action(tree_exec_node)
    ld.add_action(bridge)
    return ld
