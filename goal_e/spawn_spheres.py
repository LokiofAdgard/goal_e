#!/usr/bin/env python3

import random
import rclpy
from rclpy.node import Node
from ros_gz_interfaces.srv import SpawnEntity
from ros_gz_interfaces.msg import EntityFactory
from geometry_msgs.msg import Pose, Point, Quaternion


class SphereSpawner(Node):
    def __init__(self):
        super().__init__('sphere_spawner')
        
        # Create client for spawn service
        self.spawn_client = self.create_client(SpawnEntity, '/world/assessment_world/create')
        
        # Wait for service to be available with timeout
        self.get_logger().info('Waiting for spawn service...')
        timeout_counter = 0
        while not self.spawn_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for spawn service...')
            timeout_counter += 1
            if timeout_counter > 30:
                self.get_logger().error('Spawn service not available after 30 seconds. Make sure Gazebo is running!')
                return
        
        self.get_logger().info('Spawn service available, spawning spheres...')
        
        # Define sphere configurations: (name, radius, mass, color)
        spheres = [
            ('small_sphere', 0.1, 0.3, '1 0 0 1'),    # Red - lightest
            ('medium_sphere', 0.2, 0.6, '0 1 0 1'),  # Green - medium
            ('large_sphere', 0.3, 1.0, '0 0 1 1')    # Blue - heaviest
        ]
        
        # Spawn each sphere
        for name, radius, mass, color in spheres:
            self.spawn_sphere(name, radius, mass, color)
        
        self.get_logger().info('All spheres spawned successfully!')
    
    def get_random_position(self):
        """Generate random position within the 8m x 8m enclosure, avoiding walls and pens"""
        # Enclosure bounds: -4m to 4m in both x and y
        # Add some margin from walls (0.5m) and avoid pen areas at top
        x = random.uniform(-3.5, 3.5)
        
        # Avoid the pen areas in the top corners (y > 2.7)
        # Keep spheres in the main area
        y = random.uniform(-3.5, 2.5)
        
        # Drop from height to avoid spawning inside obstacles
        # Spheres will fall and settle naturally
        z = 2.5
        
        return x, y, z
    
    def generate_sphere_sdf(self, name, radius, mass, color):
        """Generate SDF string for a sphere (without pose - handled by EntityFactory)"""
        # Calculate moment of inertia for a solid sphere: I = (2/5) * m * r^2
        inertia = (2.0 / 5.0) * mass * (radius ** 2)
        
        sdf = f'''<?xml version="1.0" ?>
<sdf version="1.9">
  <model name="{name}">
    <link name="link">
      <inertial>
        <mass>{mass}</mass>
        <inertia>
          <ixx>{inertia}</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>{inertia}</iyy>
          <iyz>0</iyz>
          <izz>{inertia}</izz>
        </inertia>
      </inertial>
      <collision name="collision">
        <geometry>
          <sphere>
            <radius>{radius}</radius>
          </sphere>
        </geometry>
        <surface>
          <friction>
            <ode>
              <mu>1.0</mu>
              <mu2>1.0</mu2>
            </ode>
          </friction>
          <contact>
            <ode>
              <kp>1e6</kp>
              <kd>100</kd>
            </ode>
          </contact>
        </surface>
      </collision>
      <visual name="visual">
        <geometry>
          <sphere>
            <radius>{radius}</radius>
          </sphere>
        </geometry>
        <material>
          <ambient>{color}</ambient>
          <diffuse>{color}</diffuse>
          <specular>0.5 0.5 0.5 1</specular>
        </material>
      </visual>
    </link>
  </model>
</sdf>'''
        return sdf
    
    def spawn_sphere(self, name, radius, mass, color):
        """Spawn a sphere in the Gazebo world"""
        # Get random position
        x, y, z = self.get_random_position()
        
        # Create request
        request = SpawnEntity.Request()
        request.entity_factory = EntityFactory()
        request.entity_factory.name = name
        request.entity_factory.sdf = self.generate_sphere_sdf(name, radius, mass, color)
        request.entity_factory.allow_renaming = False
        
        # Set pose
        request.entity_factory.pose = Pose()
        request.entity_factory.pose.position = Point(x=x, y=y, z=z)
        request.entity_factory.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        
        future = self.spawn_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None and future.result().success:
            self.get_logger().info(f'Successfully spawned {name} (mass: {mass}kg) at ({x:.2f}, {y:.2f}, {z:.2f})')
        else:
            self.get_logger().error(f'Failed to spawn {name}')


def main(args=None):
    rclpy.init(args=args)
    spawner = SphereSpawner()
    
    # Keep node alive briefly to ensure all callbacks complete
    rclpy.spin_once(spawner, timeout_sec=1.0)
    
    spawner.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
