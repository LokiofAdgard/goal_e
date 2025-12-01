import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import ExecuteProcess

from launch_ros.actions import Node



def generate_launch_description():


    # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled
    # !!! MAKE SURE YOU SET THE PACKAGE NAME CORRECTLY !!!

    package_name='goal_e' #<--- CHANGE ME

    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    # Start Ignition Gazebo
    gz_sim = ExecuteProcess(
        cmd=['gz', 'sim', '-v', '4', '-r', 'empty.sdf'],
        output='screen'
    )


    # Spawn robot from /robot_description into Gazebo (Ignition)
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'my_bot',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.3'     # <- raise the robot
        ],
        output='screen'
    )

    # Bridge ROS2 <-> Ignition
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # ROS2 -> Ignition
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',

            # Ignition -> ROS2
            '/model/my_bot/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry',

            # Clock bridge (Ignition -> ROS2)
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        ],
        output='screen'
    )

    twist_to_stamped = Node(
        package='goal_e',
        executable='twistToStamped',
        name='twist_to_stamped',
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

    # Launch them all!
    return LaunchDescription([
        rsp,
        gz_sim,
        spawn_entity,
        bridge,
        twist_to_stamped,
        diff_drive_spawner,
        joint_broad_spawner
    ])