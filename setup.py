import os
from glob import glob
from setuptools import setup
from setuptools import find_packages

package_name = 'goal_e'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'world'), glob('world/*')),
        (os.path.join('share', package_name, 'map'), glob('map/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='admin',
    maintainer_email='dulnathvanderbona@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'state_publisher = goal_e.state_publisher:main',
            'twistToStamped = goal_e.twistToStamped:main',
            'lidar_frame_fix = goal_e.lidar_frame_fix:main',
            'spawn_spheres = goal_e.spawn_spheres:main',
            'initial_pose_publisher = goal_e.initial_pose_publisher:main',
        ],
    },
)
