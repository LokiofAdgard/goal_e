#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from math import sin, cos

class InitialPosePublisher(Node):
    def __init__(self):
        super().__init__('initial_pose_publisher')
        self.pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)

        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.frame_id = 'map'
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.pose.pose.position.x = 0.03839767351746559
        pose_msg.pose.pose.position.y = -0.021558823063969612
        pose_msg.pose.pose.position.z = 0.0

        yaw = -0.0004
        pose_msg.pose.pose.orientation.x = 0.0
        pose_msg.pose.pose.orientation.y = 0.0
        pose_msg.pose.pose.orientation.z = sin(yaw/2)
        pose_msg.pose.pose.orientation.w = cos(yaw/2)

        pose_msg.pose.covariance = [0.25, 0, 0, 0, 0, 0,
                                    0, 0.25, 0, 0, 0, 0,
                                    0, 0, 0, 0, 0, 0,
                                    0, 0, 0, 0, 0, 0,
                                    0, 0, 0, 0, 0, 0,
                                    0, 0, 0, 0, 0, 0.06853891909122467]

        self.pub.publish(pose_msg)
        self.get_logger().info('Initial pose published!')
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = InitialPosePublisher()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
