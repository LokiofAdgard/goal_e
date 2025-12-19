from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    target_finder_node = Node(
        package='goal_e',
        executable='target_finder',
        name='target_finder',
        output='screen'
    )

    pos_commander_node = Node(
        package='goal_e',
        executable='pos_commander',
        name='pos_commander',
        output='screen'
    )

    return LaunchDescription([
        target_finder_node,
        pos_commander_node
    ])
