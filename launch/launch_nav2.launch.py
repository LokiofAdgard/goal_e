from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    # -------------------------
    # Launch Arguments
    # -------------------------
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

    # Map file
    map_yaml = os.path.join(
        get_package_share_directory('goal_e'),
        'map',
        'goal_e_map.yaml'
    )

    declare_map_file = DeclareLaunchArgument(
        'map',
        default_value=map_yaml,
        description='Full path to map YAML file for localization'
    )

    # Nav2 parameters
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

    # -------------------------
    # Launch Nav2 Localization
    # -------------------------
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                os.getenv('ROS_DISTRO_PATH', '/opt/ros/jazzy/share/nav2_bringup/launch'),
                'localization_launch.py'
            )
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'params_file': LaunchConfiguration('nav2_params_file'),
            'map': LaunchConfiguration('map')
        }.items()
    )

    # -------------------------
    # Launch Nav2 Full Navigation
    # -------------------------
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                os.getenv('ROS_DISTRO_PATH', '/opt/ros/jazzy/share/nav2_bringup/launch'),
                'navigation_launch.py'
            )
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'map_subscribe_transient_local': 'True',  # ensure map topic is transient_local
            'params_file': LaunchConfiguration('nav2_params_file')
        }.items()
    )

    # -------------------------
    # Launch RViz (optional)
    # -------------------------
    rviz_launch = ExecuteProcess(
        condition=IfCondition(LaunchConfiguration('use_rviz')),
        cmd=[
            'ros2', 'run', 'rviz2', 'rviz2',
            '-d', PathJoinSubstitution([
                FindPackageShare('goal_e'),
                'config',
                'goal_e.rviz'
            ])
        ],
        output='screen'
    )

    # -------------------------
    # Initial Pose Publisher Node
    # -------------------------
    initial_pose_node = Node(
        package='goal_e',  # your package
        executable='initial_pose_publisher',  # Python script you create
        name='initial_pose_publisher',
        output='screen',
        parameters=[{
            'x': 0.03839767351746559,
            'y': -0.021558823063969612,
            'yaw': -0.0004  # radians
        }]
    )

    # -------------------------
    # Return Launch Description
    # -------------------------
    return LaunchDescription([
        declare_use_rviz,
        declare_use_sim_time,
        declare_map_file,
        declare_nav2_params_file,
        localization_launch,
        initial_pose_node,
        navigation_launch,
        rviz_launch
    ])
