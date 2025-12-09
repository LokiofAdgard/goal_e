from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Arguments
    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Whether to launch RViz'
    )

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )

    slam_params = os.path.join(
        get_package_share_directory('goal_e'),
        'config', 'mapper_params_online_async.yaml'
    )

    declare_slam_params_file = DeclareLaunchArgument(
        'slam_params_file',
        default_value=slam_params,
        description='Path to the slam_toolbox parameters file'
    )

    nav2_params = os.path.join(
        get_package_share_directory('goal_e'),
        'config',
        'nav2_params.yaml'
    )

    declare_nav2_params_file = DeclareLaunchArgument(
        'nav2_params_file',
        default_value=nav2_params,
        description='Path to the Nav2 parameters file'
    )

    # Launch Nav2
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                os.getenv('ROS_DISTRO_PATH', '/opt/ros/jazzy/share/nav2_bringup/launch'),
                'navigation_launch.py'
            )
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'params_file': LaunchConfiguration('nav2_params_file')
        }.items()
    )

    # Launch SLAM Toolbox
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                os.getenv('ROS_DISTRO_PATH', '/opt/ros/jazzy/share/slam_toolbox/launch'),
                'online_async_launch.py'
            )
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'params_file': LaunchConfiguration('slam_params_file')
        }.items()
    )

    # RViz (conditional)
    rviz_launch = ExecuteProcess(
        condition=IfCondition(LaunchConfiguration('use_rviz')),
        cmd=[
            'ros2', 'run', 'rviz2', 'rviz2',
            '-d', '/opt/ros/jazzy/share/nav2_bringup/rviz/nav2_default_view.rviz'
        ],
        output='screen'
    )


    return LaunchDescription([
        declare_use_rviz,
        declare_use_sim_time,
        declare_slam_params_file,
        declare_nav2_params_file,
        nav2_launch,
        slam_launch,
        rviz_launch
    ])
