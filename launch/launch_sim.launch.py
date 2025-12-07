import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import ExecuteProcess

from launch_ros.actions import Node



def generate_launch_description():

    package_name='goal_e'

    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'True'}.items()
    )

    # world_file = os.path.join(
    #     get_package_share_directory('goal_e'),
    #     'world',
    #     'test_world.sdf'
    # )

    world_file = os.path.join(
        get_package_share_directory('goal_e'),
        'world',
        'oct_world.world'
    )

    # world_file = os.path.join(
    #     get_package_share_directory('goal_e'),
    #     'world',
    #     'turtlebot3_world.world'
    # )

    gz_sim = ExecuteProcess(
        cmd=['gz', 'sim', '-v', '4', '-r', world_file],
        output='screen'
    )

    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'my_bot',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.3'
        ],
        output='screen'
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            # '/model/my_bot/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/lidar1/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
            '/lidar1/scan/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked',
        ],
        output='screen'
    )

    twist_to_stamped = Node(
        package='goal_e',
        executable='twistToStamped',
        name='twist_to_stamped',
        output='screen',
    )

    lidar_frame_fix = Node(
        package='goal_e',
        executable='lidar_frame_fix',
        name='lidar_fix',
        output='screen',
    )

    diff_drive_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_cont', "--controller-manager", "/controller_manager"]
    )

    joint_broad_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', "--controller-manager", "/controller_manager"]
    )

    return LaunchDescription([
        rsp,
        gz_sim,
        spawn_entity,
        bridge,
        twist_to_stamped,
        lidar_frame_fix,
        diff_drive_spawner,
        joint_broad_spawner
    ])