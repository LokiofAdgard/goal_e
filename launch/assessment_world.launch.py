from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Path to the world file
    world_file = PathJoinSubstitution([
        FindPackageShare('goal_e'),
        'world',
        'assessment.sdf'
    ])

    # Include Gazebo launch file
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            ])
        ]),
        launch_arguments={
            'gz_args': ['-r --render-engine ogre ', world_file]
        }.items()
    )

    return LaunchDescription([
        gz_sim
    ])
