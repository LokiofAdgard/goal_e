import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class SequentialNav2Goals(Node):

    def __init__(self):
        super().__init__('sequential_nav2_goals')

        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose'
        )

        self.arm_pub = self.create_publisher(
            JointTrajectory,
            '/arm_controller/joint_trajectory',
            10
        )

        # --------------------
        # PLACE-IN-GOAL (UNCHANGED)
        # --------------------
        self.place_goals = [
            self.make_goal(
                x=-0.016622602939605713,
                y=1.385196328163147,
                z=0.7109481310557748,
                w=0.7032444489281809
            ),
            self.make_goal(
                x=0.013612210750579834,
                y=2.2709434032440186,
                z=0.714937117227181,
                w=0.6991887573544701
            )
        ]

        # --------------------
        # PICK TARGETS (RANDOM / ARBITRARY)
        # --------------------
        self.pick_targets = [
            (
                self.make_goal(-1.0, 0.5, 0.0, 1.0),   # approach
                self.make_goal(-0.1, 0.5, 0.0, 1.0)    # target
            ),
            (
                self.make_goal(-1.2, 1.0, 0.0, 1.0),
                self.make_goal(-1.0, 1.0, 0.0, 1.0)
            ),
            (
                self.make_goal(-1.4, 1.5, 0.0, 1.0),
                self.make_goal(-1.2, 1.5, 0.0, 1.0)
            )
        ]

        self.pick_index = 0
        self.place_index = 0

        self.state = None
        self.current_nav_goal = None

        self.nav_client.wait_for_server()

        self.start_cycle()

    # ==========================================================
    # High-level execution
    # ==========================================================

    def start_cycle(self):
        if self.pick_index >= len(self.pick_targets):
            self.get_logger().info('All pick & place cycles completed')
            return

        self.get_logger().info(f'Starting cycle {self.pick_index + 1}')

        approach, target = self.pick_targets[self.pick_index]
        self.pick_target(approach, target)

    # ==========================================================
    # PICK TARGET (NEW)
    # ==========================================================

    def pick_target(self, approach_pose, target_pose):
        self.pick_approach = approach_pose
        self.pick_target_pose = target_pose

        self.state = 'PICK_GO_APPROACH'
        self.send_nav_goal(self.pick_approach)

    # ==========================================================
    # PLACE IN GOAL (UNCHANGED LOGIC)
    # ==========================================================

    def place_in_goal(self):
        self.place_index = 0
        self.state = 'PLACE_GOAL'
        self.send_nav_goal(self.place_goals[self.place_index])

    # ==========================================================
    # Navigation handling
    # ==========================================================

    def send_nav_goal(self, pose):
        goal = NavigateToPose.Goal()
        goal.pose = pose
        future = self.nav_client.send_goal_async(goal)
        future.add_done_callback(self.goal_response_cb)

    def goal_response_cb(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error('Goal rejected')
            return

        handle.get_result_async().add_done_callback(self.result_cb)

    def result_cb(self, future):
        if future.result().status != 4:
            self.get_logger().error('Navigation failed')
            return

        # ---------------- PICK TARGET ----------------
        if self.state == 'PICK_GO_APPROACH':
            self.open_arms()
            self.state = 'PICK_GO_TARGET'
            self.send_nav_goal(self.pick_target_pose)

        elif self.state == 'PICK_GO_TARGET':
            self.close_arms()
            self.state = 'PLACE_GOAL'
            self.place_in_goal()

        # ---------------- PLACE IN GOAL ----------------
        elif self.state == 'PLACE_GOAL':
            if self.place_index == 0:
                self.open_arms()
                self.place_index = 1
                self.send_nav_goal(self.place_goals[1])
            else:
                self.close_arms()
                self.pick_index += 1
                self.start_cycle()

    # ==========================================================
    # Arm control (unchanged)
    # ==========================================================

    def open_arms(self):
        traj = JointTrajectory()
        traj.joint_names = ['chassis_to_arm1', 'chassis_to_arm2']
        point = JointTrajectoryPoint()
        point.positions = [-2.8, 2.8]
        point.time_from_start.sec = 2
        traj.points.append(point)
        self.arm_pub.publish(traj)
        self.get_logger().info("Arms Open")

    def close_arms(self):
        traj = JointTrajectory()
        traj.joint_names = ['chassis_to_arm1', 'chassis_to_arm2']
        point = JointTrajectoryPoint()
        point.positions = [0.0, 0.0]
        point.time_from_start.sec = 2
        traj.points.append(point)
        self.arm_pub.publish(traj)
        self.get_logger().info("Arms Closed")

    # ==========================================================
    # Helpers
    # ==========================================================

    def make_goal(self, x, y, z, w):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.z = z
        pose.pose.orientation.w = w
        return pose


def main():
    rclpy.init()
    node = SequentialNav2Goals()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
