import math
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PointStamped

import tf2_ros
import tf2_geometry_msgs

from rclpy.qos import (
    qos_profile_sensor_data,
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy
)

# ============================
# Hardcoded tuning parameters
# ============================
MIN_DISTANCE_FROM_MAP = 0.10   # meters
CLUSTER_DIST = 0.15            # meters
MIN_CLUSTER_POINTS = 6

MAX_RADIUS_STD = 0.05          # meters
MIN_ARC_RADIUS = 0.05
MAX_ARC_RADIUS = 2.0


class ScanMapDifferencer(Node):

    def __init__(self):
        super().__init__('scan_map_differencer')

        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            map_qos
        )

        self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            qos_profile_sensor_data
        )

        self.marker_pub = self.create_publisher(
            MarkerArray,
            '/unmapped_scan_points',
            10
        )

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.map = None
        self.get_logger().info('Scan–Map differencer with arc detection started')

    # ============================
    def map_callback(self, msg):
        self.map = msg

    # ============================
    def scan_callback(self, scan):
        if self.map is None:
            return

        try:
            transform = self.tf_buffer.lookup_transform(
                self.map.header.frame_id,
                scan.header.frame_id,
                rclpy.time.Time()
            )
        except Exception:
            return

        base_marker = Marker()
        base_marker.header.frame_id = self.map.header.frame_id
        base_marker.header.stamp = self.get_clock().now().to_msg()
        base_marker.ns = 'unmapped_points'
        base_marker.id = 0
        base_marker.type = Marker.POINTS
        base_marker.action = Marker.ADD
        base_marker.scale.x = 0.05
        base_marker.scale.y = 0.05
        base_marker.color.r = 1.0
        base_marker.color.a = 1.0

        angle = scan.angle_min

        for r in scan.ranges:
            if not math.isfinite(r):
                angle += scan.angle_increment
                continue

            p = PointStamped()
            p.header.frame_id = scan.header.frame_id
            p.point.x = r * math.cos(angle)
            p.point.y = r * math.sin(angle)

            try:
                p_map = tf2_geometry_msgs.do_transform_point(p, transform)
            except Exception:
                angle += scan.angle_increment
                continue

            if self.is_unmapped(p_map.point.x, p_map.point.y):
                base_marker.points.append(p_map.point)

            angle += scan.angle_increment

        self.get_logger().info(
            f"Unmapped points: {len(base_marker.points)}",
            throttle_duration_sec=1.0
        )

        if not base_marker.points:
            return

        clusters = self.cluster_points(base_marker.points)

        markers = MarkerArray()
        markers.markers.append(base_marker)

        mid = 1
        for cluster in clusters:
            fit = self.fit_circle(cluster)
            if fit is None:
                continue

            cx, cy, r, r_std = fit

            if (
                r_std < MAX_RADIUS_STD and
                MIN_ARC_RADIUS < r < MAX_ARC_RADIUS
            ):
                m = Marker()
                m.header = base_marker.header
                m.ns = 'arc_centers'
                m.id = mid
                mid += 1
                m.type = Marker.SPHERE
                m.scale.x = 0.12
                m.scale.y = 0.12
                m.scale.z = 0.12
                m.color.g = 1.0
                m.color.a = 1.0
                m.pose.position.x = cx
                m.pose.position.y = cy

                markers.markers.append(m)

        self.marker_pub.publish(markers)

    # ============================
    def is_unmapped(self, x, y):
        info = self.map.info
        res = info.resolution

        mx = int((x - info.origin.position.x) / res)
        my = int((y - info.origin.position.y) / res)

        if mx < 0 or my < 0 or mx >= info.width or my >= info.height:
            return False

        idx = my * info.width + mx
        if self.map.data[idx] != 0:
            return False

        radius = int(math.ceil(MIN_DISTANCE_FROM_MAP / res))

        for dx in range(-radius, radius + 1):
            for dy in range(-radius, radius + 1):
                nx = mx + dx
                ny = my + dy
                if 0 <= nx < info.width and 0 <= ny < info.height:
                    if self.map.data[ny * info.width + nx] > 50:
                        return False

        return True

    # ============================
    def cluster_points(self, points):
        clusters = []
        used = [False] * len(points)

        for i, p in enumerate(points):
            if used[i]:
                continue

            cluster = [p]
            used[i] = True

            queue = [i]
            while queue:
                idx = queue.pop()
                for j, q in enumerate(points):
                    if used[j]:
                        continue
                    if math.hypot(
                        points[idx].x - q.x,
                        points[idx].y - q.y
                    ) < CLUSTER_DIST:
                        used[j] = True
                        queue.append(j)
                        cluster.append(q)

            if len(cluster) >= MIN_CLUSTER_POINTS:
                clusters.append(cluster)

        return clusters

    # ============================
    def fit_circle(self, cluster):
        xs = [p.x for p in cluster]
        ys = [p.y for p in cluster]

        xm = sum(xs) / len(xs)
        ym = sum(ys) / len(ys)

        Suu = Suv = Svv = Suuu = Svvv = Suvv = Svuu = 0.0

        for x, y in zip(xs, ys):
            u = x - xm
            v = y - ym
            Suu += u * u
            Svv += v * v
            Suv += u * v
            Suuu += u * u * u
            Svvv += v * v * v
            Suvv += u * v * v
            Svuu += v * u * u

        det = 2 * (Suu * Svv - Suv * Suv)
        if abs(det) < 1e-6:
            return None

        cx = xm + (Svv * (Suuu + Suvv) - Suv * (Svvv + Svuu)) / det
        cy = ym + (Suu * (Svvv + Svuu) - Suv * (Suuu + Suvv)) / det

        radii = [math.hypot(p.x - cx, p.y - cy) for p in cluster]
        r_mean = sum(radii) / len(radii)
        r_std = math.sqrt(sum((r - r_mean) ** 2 for r in radii) / len(radii))

        return cx, cy, r_mean, r_std


def main():
    rclpy.init()
    node = ScanMapDifferencer()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
