#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, PoseArray, Pose
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class SequentialNav2Goals(Node):

    def __init__(self):
        super().__init__('sequential_nav2_goals')

        # ---------------- Action Client ----------------
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # ---------------- Arm Publisher ----------------
        self.arm_pub = self.create_publisher(
            JointTrajectory,
            '/arm_controller/joint_trajectory',
            10
        )

        # ---------------- Internal State ----------------
        self.pick_targets = []
        self.latest_targets = []
        self.pick_index = 0
        self.place_index = 0
        self.state = None
        self.cycle_running = False
        self.ready_for_cycles = False
        self.delay_timer = None
        self.refresh_timer = None
        self.initial_pose_sent = False

        # ---------------- Fixed Place Goals ----------------
        self.place_goals = [
            self.make_goal(-0.2, 1.2, 0.7071, 0.7071),
            self.make_goal(-0.15, 1.8, 0.7071, 0.7071)
        ]

        # ---------------- Nav2 Server ----------------
        self.get_logger().info('Waiting for Nav2 action server...')
        self.nav_client.wait_for_server()
        self.get_logger().info('Nav2 action server available')

        # ---------------- Subscriptions ----------------
        self.create_subscription(
            PoseArray,
            '/ball_targets',
            self.ball_targets_cb,
            10
        )

        # ---------------- Delay before sending initial pose ----------------
        self.get_logger().info('Waiting for AMCL and costmaps to initialize...')
        self.create_timer(3.0, self.send_initial_pose_once)

    # ==================================================
    # Send initial start pose ONCE after delay
    # ==================================================
    def send_initial_pose_once(self):
        if self.initial_pose_sent:
            return

        self.initial_pose_sent = True
        self.get_logger().info('Navigating to start pose...')

        self.state = 'INIT_START_POSE'
        start_pose = self.make_goal(-1.0, 0.0, 0.0, 1.0)
        self.send_nav_goal(start_pose)

    # ==================================================
    # Ball target callback (updates immediately)
    # ==================================================
    def ball_targets_cb(self, msg: PoseArray):
        updated = []

        for i in range(0, len(msg.poses), 2):
            if i + 1 >= len(msg.poses):
                break

            approach = msg.poses[i]
            target = msg.poses[i + 1]

            if self.is_zero_pose(approach) or self.is_zero_pose(target):
                continue

            updated.append((approach, target))

        self.latest_targets = updated

        # Execute only when system is fully ready
        if (
            self.ready_for_cycles and
            not self.cycle_running and
            self.latest_targets
        ):
            self.pick_targets = list(self.latest_targets)
            self.pick_index = 0
            self.cycle_running = True
            self.start_cycle()

    # ==================================================
    # High-level cycle control
    # ==================================================
    def start_cycle(self):
        if self.pick_index >= len(self.pick_targets):
            self.get_logger().info('All pick & place cycles completed')
            self.cycle_running = False
            return

        approach, target = self.pick_targets[self.pick_index]
        self.pick_approach_pose = approach
        self.pick_target_pose = target

        self.state = 'PICK_GO_APPROACH'
        self.send_nav_goal(self.pick_approach_pose)

    # ==================================================
    # Navigation
    # ==================================================
    def send_nav_goal(self, pose: Pose):
        goal = NavigateToPose.Goal()

        ps = PoseStamped()
        ps.header.frame_id = 'map'
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose = pose

        goal.pose = ps
        self.nav_client.send_goal_async(goal).add_done_callback(
            self.goal_response_cb
        )

    def goal_response_cb(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error('Navigation goal rejected')
            self.cycle_running = False
            return

        handle.get_result_async().add_done_callback(self.result_cb)

    def result_cb(self, future):
        result = future.result()
        if result.status != 4:  # SUCCEEDED
            self.get_logger().error('Navigation failed')
            self.cycle_running = False
            return

        # ---------------- INITIAL START POSE ----------------
        if self.state == 'INIT_START_POSE':
            self.get_logger().info('Initial pose reached. System ready.')
            self.state = None
            self.ready_for_cycles = True

            # Start cycle immediately if targets already exist
            if self.latest_targets:
                self.pick_targets = list(self.latest_targets)
                self.pick_index = 0
                self.cycle_running = True
                self.start_cycle()
            return

        # ---------------- PICK ----------------
        if self.state == 'PICK_GO_APPROACH':
            self.open_arms()
            self.state = 'PICK_GO_TARGET'
            self.send_nav_goal(self.pick_target_pose)

        elif self.state == 'PICK_GO_TARGET':
            self.close_arms()
            self.start_delay(self.after_pick_delay)

        # ---------------- PLACE ----------------
        elif self.state == 'PLACE_GOAL':
            if self.place_index == 0:
                self.open_arms()
                self.place_index = 1
                self.send_nav_goal(self.place_goals[1])
            else:
                self.close_arms()
                self.start_delay(self.after_place_delay)

    # ==================================================
    # Delay handling
    # ==================================================
    def start_delay(self, callback):
        if self.delay_timer:
            self.delay_timer.cancel()

        self.delay_timer = self.create_timer(
            7.0,
            lambda: self._delay_done(callback)
        )

    def _delay_done(self, callback):
        self.delay_timer.cancel()
        self.delay_timer = None
        callback()

    def after_pick_delay(self):
        self.place_index = 0
        self.state = 'PLACE_GOAL'
        self.send_nav_goal(self.place_goals[0])

    def after_place_delay(self):
        self.pick_index += 1
        self.start_cycle()

    # ==================================================
    # Arm control
    # ==================================================
    def open_arms(self):
        traj = JointTrajectory()
        traj.joint_names = ['chassis_to_arm1', 'chassis_to_arm2']

        point = JointTrajectoryPoint()
        point.positions = [-2.8, 2.8]
        point.time_from_start.sec = 2

        traj.points.append(point)
        self.arm_pub.publish(traj)

    def close_arms(self):
        traj = JointTrajectory()
        traj.joint_names = ['chassis_to_arm1', 'chassis_to_arm2']

        point = JointTrajectoryPoint()
        point.positions = [0.0, 0.0]
        point.time_from_start.sec = 2

        traj.points.append(point)
        self.arm_pub.publish(traj)

    # ==================================================
    # Helpers
    # ==================================================
    def make_goal(self, x, y, z, w):
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.orientation.z = z
        pose.orientation.w = w
        return pose

    def is_zero_pose(self, pose: Pose):
        return pose.position.x == 0.0 and pose.position.y == 0.0


def main():
    rclpy.init()
    node = SequentialNav2Goals()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
