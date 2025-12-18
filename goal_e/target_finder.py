#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import (
    PointStamped,
    Point,
    PoseArray,
    Pose,
    Quaternion
)

import tf2_ros
import tf2_geometry_msgs

from rclpy.qos import (
    qos_profile_sensor_data,
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy
)

# ============================
# Tuning parameters
# ============================
MIN_DISTANCE_FROM_MAP = 0.20
CLUSTER_DIST = 0.15
MIN_CLUSTER_POINTS = 6

MAX_RADIUS_STD = 0.05

BALL_UPDATE_TOL = 0.45
APPROACH_DIST = 1.40
ANGLE_STEP = math.radians(5)

APPROACH_COST_RADIUS = 0.30  # meters
PICKUP_CLEARANCE = 0.41

# ============================
# Ball classification
# ============================
BALL_RADII = {
    "red": 0.07,
    "green": 0.20,
    "blue": 0.26
}

RADIUS_TOL = 0.03

# ============================
# Weighted scoring parameters
# ============================
WEIGHT_COST = 0.07
WEIGHT_DIST = 0.93


class ScanMapDifferencer(Node):

    def __init__(self):
        super().__init__('scan_map_differencer')

        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, map_qos
        )

        self.create_subscription(
            OccupancyGrid,
            '/global_costmap/costmap',
            self.costmap_callback,
            map_qos
        )

        self.create_subscription(
            LaserScan, '/scan', self.scan_callback, qos_profile_sensor_data
        )

        self.marker_pub = self.create_publisher(
            MarkerArray, '/unmapped_scan_points', 10
        )

        self.pose_pub = self.create_publisher(
            PoseArray, '/ball_targets', 10
        )

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.map = None
        self.costmap = None

        # Ball centers
        self.red_ball = None
        self.green_ball = None
        self.blue_ball = None

        # Persistent approach points (Option B)
        self.red_approach = None
        self.green_approach = None
        self.blue_approach = None

        # Pickup offsets
        self.red_offset = None
        self.green_offset = None
        self.blue_offset = None

        self.get_logger().info('Scan–Map differencer started')

    # ============================
    # Map / Costmap callbacks
    # ============================
    def map_callback(self, msg):
        self.map = msg

    def costmap_callback(self, msg):
        self.costmap = msg

    # ============================
    # Ball state helpers
    # ============================
    def update_ball(self, current, new):
        """Return (updated_center, changed_flag)."""
        if current is None:
            return new, True

        if math.hypot(current[0] - new[0], current[1] - new[1]) <= BALL_UPDATE_TOL:
            return new, True

        # Ball moved too far → do NOT update center (Option B)
        return current, False

    # ============================
    # Pose helpers
    # ============================
    def yaw_to_quaternion(self, yaw):
        q = Quaternion()
        q.w = math.cos(yaw / 2)
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(yaw / 2)
        return q

    def make_pose(self, xy, yaw=None):
        pose = Pose()
        pose.position.x = xy[0]
        pose.position.y = xy[1]
        pose.position.z = 0.0
        if yaw is not None:
            pose.orientation = self.yaw_to_quaternion(yaw)
        else:
            pose.orientation.w = 1.0
        return pose

    # ============================
    # Main scan callback
    # ============================
    def scan_callback(self, scan):
        if self.map is None or self.costmap is None:
            return

        try:
            transform = self.tf_buffer.lookup_transform(
                self.map.header.frame_id,
                scan.header.frame_id,
                rclpy.time.Time()
            )
        except Exception:
            return

        points = []
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
                points.append(p_map.point)

            angle += scan.angle_increment

        clusters = self.cluster_points(points)

        # ============================
        # Circle fitting & ball classification
        # ============================
        for cluster in clusters:
            fit = self.fit_circle(cluster)
            if fit is None:
                continue

            cx, cy, r, r_std = fit
            if r_std > MAX_RADIUS_STD:
                continue

            for color, target_r in BALL_RADII.items():
                if abs(r - target_r) > RADIUS_TOL:
                    continue

                if color == "red":
                    self.red_ball, changed = self.update_ball(self.red_ball, (cx, cy))
                    if changed:
                        if self.red_approach is None:
                            self.red_approach = self.find_best_approach_point(self.red_ball)
                        if self.red_approach:
                            self.red_offset = self.offset_towards(
                                self.red_ball, self.red_approach, PICKUP_CLEARANCE
                            )

                elif color == "green":
                    self.green_ball, changed = self.update_ball(self.green_ball, (cx, cy))
                    if changed:
                        if self.green_approach is None:
                            self.green_approach = self.find_best_approach_point(self.green_ball)
                        if self.green_approach:
                            self.green_offset = self.offset_towards(
                                self.green_ball, self.green_approach, PICKUP_CLEARANCE
                            )

                elif color == "blue":
                    self.blue_ball, changed = self.update_ball(self.blue_ball, (cx, cy))
                    if changed:
                        if self.blue_approach is None:
                            self.blue_approach = self.find_best_approach_point(self.blue_ball)
                        if self.blue_approach:
                            self.blue_offset = self.offset_towards(
                                self.blue_ball, self.blue_approach, PICKUP_CLEARANCE
                            )

        # ============================
        # Markers (with DELETEALL)
        # ============================
        markers = MarkerArray()

        clear = Marker()
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)

        mid = 0

        def marker_point(ns, color, pos):
            nonlocal mid
            m = Marker()
            m.header.frame_id = self.map.header.frame_id
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = ns
            m.id = mid
            mid += 1
            m.type = Marker.POINTS
            m.action = Marker.ADD
            m.scale.x = 0.12
            m.scale.y = 0.12
            m.color.a = 1.0

            if color == "red":
                m.color.r = 1.0
            elif color == "green":
                m.color.g = 1.0
            elif color == "blue":
                m.color.b = 1.0
            elif color == "black":
                m.color.r = 0.0
                m.color.g = 0.0
                m.color.b = 0.0

            m.points.append(Point(x=pos[0], y=pos[1], z=0.0))
            return m

        # Add markers for each ball
        for name, ball, approach, offset in [
            ("red", self.red_ball, self.red_approach, self.red_offset),
            ("green", self.green_ball, self.green_approach, self.green_offset),
            ("blue", self.blue_ball, self.blue_approach, self.blue_offset),
        ]:
            if ball:
                markers.markers.append(marker_point(f"{name}_ball", name, ball))
            if approach:
                markers.markers.append(marker_point(f"{name}_approach", "black", approach))
            if offset:
                markers.markers.append(marker_point(f"{name}_offset", "black", offset))

        self.marker_pub.publish(markers)

        # ============================
        # PoseArray (approach + offset)
        # ============================
        pose_array = PoseArray()
        pose_array.header.frame_id = self.map.header.frame_id
        pose_array.header.stamp = self.get_clock().now().to_msg()

        targets = []
        for name, ball, approach, offset in [
            ("red", self.red_ball, self.red_approach, self.red_offset),
            ("green", self.green_ball, self.green_approach, self.green_offset),
            ("blue", self.blue_ball, self.blue_approach, self.blue_offset),
        ]:
            if not ball or not approach:
                targets.append({
                    "ball": None,
                    "approach": None,
                    "offset": None,
                    "distance": float('inf'),
                })
                continue

            dist = math.hypot(approach[0] - 0.0, approach[1] - 0.3)

            targets.append({
                "ball": ball,
                "approach": approach,
                "offset": offset,
                "distance": dist,
            })

        targets.sort(key=lambda t: t["distance"])

        for t in targets:
            ball = t["ball"]
            approach = t["approach"]
            offset = t["offset"]

            if ball and approach:
                yaw = math.atan2(ball[1] - approach[1], ball[0] - approach[0])
                pose_array.poses.append(self.make_pose(approach, yaw))
            else:
                pose_array.poses.append(Pose())

            if ball and approach and offset:
                yaw = math.atan2(ball[1] - approach[1], ball[0] - approach[0])
                pose_array.poses.append(self.make_pose(offset, yaw))
            else:
                pose_array.poses.append(Pose())

        self.pose_pub.publish(pose_array)

    # ============================
    # Cost helpers
    # ============================
    def cost_along_line(self, start, end, radius):
        info = self.costmap.info
        res = info.resolution

        dist = math.hypot(end[0] - start[0], end[1] - start[1])
        steps = max(3, int(dist / res))

        total_cost = 0.0
        valid = 0

        for i in range(steps + 1):
            t = i / steps
            x = start[0] * (1 - t) + end[0] * t
            y = start[1] * (1 - t) + end[1] * t

            c = self.cost_circle_at(x, y, radius)
            if c is None:
                return None
            total_cost += c
            valid += 1

        if valid == 0:
            return None

        return total_cost / valid

    # ============================
    # Weighted scoring
    # ============================
    def find_best_approach_point(self, center):
        candidates = []

        for a in self.frange(0, 2 * math.pi, ANGLE_STEP):
            x = center[0] + APPROACH_DIST * math.cos(a)
            y = center[1] + APPROACH_DIST * math.sin(a)

            cost = self.cost_along_line((x, y), center, APPROACH_COST_RADIUS)
            if cost is None:
                continue

            dist = math.hypot(x, y)

            score = WEIGHT_COST * cost + WEIGHT_DIST * dist

            candidates.append((x, y, score))

        if not candidates:
            return None

        best_x, best_y, _ = min(candidates, key=lambda c: c[2])
        return (best_x, best_y)

    # ============================
    # Offset point helper
    # ============================
    def offset_towards(self, src, dst, dist):
        dx = dst[0] - src[0]
        dy = dst[1] - src[1]
        L = math.hypot(dx, dy)
        if L < 1e-6:
            return src
        scale = dist / L
        return (src[0] + dx * scale, src[1] + dy * scale)

    # ============================
    # Costmap sampling
    # ============================
    def cost_circle_at(self, x, y, radius):
        info = self.costmap.info
        res = info.resolution
        r_cells = int(radius / res)

        cx = int((x - info.origin.position.x) / res)
        cy = int((y - info.origin.position.y) / res)

        if cx < 0 or cy < 0 or cx >= info.width or cy >= info.height:
            return None

        total_cost = 0.0
        valid_cells = 0
        bad_cells = 0

        for dx in range(-r_cells, r_cells + 1):
            for dy in range(-r_cells, r_cells + 1):
                if dx*dx + dy*dy > r_cells*r_cells:
                    continue

                mx = cx + dx
                my = cy + dy

                if mx < 0 or my < 0 or mx >= info.width or my >= info.height:
                    bad_cells += 1
                    continue

                cost = self.costmap.data[my * info.width + mx]

                if cost < 0 or cost >= 253:
                    bad_cells += 1
                    continue

                total_cost += cost
                valid_cells += 1

        if valid_cells == 0:
            return None

        if bad_cells > valid_cells:
            return None

        return total_cost / valid_cells

    # ============================
    # Misc helpers
    # ============================
    def frange(self, start, stop, step):
        while start < stop:
            yield start
            start += step

    def is_unmapped(self, x, y):
        info = self.map.info
        res = info.resolution

        mx = int((x - info.origin.position.x) / res)
        my = int((y - info.origin.position.y) / res)

        if mx < 0 or my < 0 or mx >= info.width or my >= info.height:
            return False

        if self.map.data[my * info.width + mx] != 0:
            return False

        radius = int(MIN_DISTANCE_FROM_MAP / res)
        for dx in range(-radius, radius + 1):
            for dy in range(-radius, radius + 1):
                nx, ny = mx + dx, my + dy
                if 0 <= nx < info.width and 0 <= ny < info.height:
                    if self.map.data[ny * info.width + nx] > 50:
                        return False

        return True

    def cluster_points(self, points):
        clusters = []
        used = [False] * len(points)

        for i in range(len(points)):
            if used[i]:
                continue

            cluster = [points[i]]
            used[i] = True
            q = [i]

            while q:
                idx = q.pop()
                for j in range(len(points)):
                    if used[j]:
                        continue
                    if math.hypot(
                        points[idx].x - points[j].x,
                        points[idx].y - points[j].y
                    ) < CLUSTER_DIST:
                        used[j] = True
                        q.append(j)
                        cluster.append(points[j])

            if len(cluster) >= MIN_CLUSTER_POINTS:
                clusters.append(cluster)

        return clusters

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
        r = sum(radii) / len(radii)
        r_std = math.sqrt(
            sum((ri - r) ** 2 for ri in radii) / len(radii)
        )

        return cx, cy, r, r_std


def main():
    rclpy.init()
    rclpy.spin(ScanMapDifferencer())
    rclpy.shutdown()


if __name__ == '__main__':
    main()

