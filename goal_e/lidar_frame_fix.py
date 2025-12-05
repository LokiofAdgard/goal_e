import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2

class LidarFrameFix(Node):
    def __init__(self):
        super().__init__('lidar_frame_fix')

        self.correct_frame = "lidar1"

        # LaserScan
        self.sub_scan = self.create_subscription(
            LaserScan,
            '/lidar1/scan',
            self.scan_callback,
            10
        )
        self.pub_scan = self.create_publisher(
            LaserScan,
            '/scan',
            10
        )

        # PointCloud2
        self.sub_points = self.create_subscription(
            PointCloud2,
            '/lidar1/scan/points',
            self.points_callback,
            10
        )
        self.pub_points = self.create_publisher(
            PointCloud2,
            '/scan/points',
            10
        )

    def scan_callback(self, msg: LaserScan):
        msg.header.frame_id = self.correct_frame
        self.pub_scan.publish(msg)

    def points_callback(self, msg: PointCloud2):
        msg.header.frame_id = self.correct_frame
        self.pub_points.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = LidarFrameFix()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
