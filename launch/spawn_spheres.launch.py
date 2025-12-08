from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Bridge for the spawn service
    spawn_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/world/assessment_world/create@ros_gz_interfaces/srv/SpawnEntity'
        ],
        output='screen'
    )
    
    # Node to spawn spheres
    spawn_spheres_node = Node(
        package='goal_e',
        executable='spawn_spheres',
        name='sphere_spawner',
        output='screen'
    )
    
    # Add a delay to ensure bridge is ready
    delayed_spawn = TimerAction(
        period=2.0,
        actions=[spawn_spheres_node]
    )

    return LaunchDescription([
        spawn_bridge,
        delayed_spawn
    ])
