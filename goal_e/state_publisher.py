from math import sin, cos, pi
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster, TransformStamped

class StatePublisher(Node):

    def __init__(self):
        rclpy.init()
        super().__init__('state_publisher')

        qos_profile = QoSProfile(depth=10)
        self.joint_pub = self.create_publisher(JointState, 'joint_states', qos_profile)
        self.broadcaster = TransformBroadcaster(self, qos=qos_profile)
        self.nodeName = self.get_name()
        self.get_logger().info("{0} started".format(self.nodeName))

        degree = pi / 180.0
        loop_rate = self.create_rate(30)

        # Initialize wheel positions
        rf_wheel = 0.0
        rb_wheel = 0.0
        lf_wheel = 0.0
        lb_wheel = 0.0
        wheel_increment = 0.05  # radians per loop iteration

        # message declarations
        joint_state = JointState()
        odom_trans = TransformStamped()  # if you still want a base transform
        odom_trans.header.frame_id = 'odom'
        odom_trans.child_frame_id = 'base_link'

        try:
            while rclpy.ok():
                rclpy.spin_once(self)

                # update joint_state
                now = self.get_clock().now()
                joint_state.header.stamp = now.to_msg()
                joint_state.name = ['chassis_to_rf_wheel', 'chassis_to_rb_wheel', 'chassis_to_lf_wheel', 'chassis_to_lb_wheel']
                joint_state.position = [rf_wheel, rb_wheel, lf_wheel, lb_wheel]

                # publish joint state
                self.joint_pub.publish(joint_state)

                # increment wheels
                rf_wheel -= wheel_increment
                rb_wheel -= wheel_increment
                lf_wheel += wheel_increment
                lb_wheel += wheel_increment

                loop_rate.sleep()

        except KeyboardInterrupt:
            pass

def euler_to_quaternion(roll, pitch, yaw):
    qx = sin(roll/2) * cos(pitch/2) * cos(yaw/2) - cos(roll/2) * sin(pitch/2) * sin(yaw/2)
    qy = cos(roll/2) * sin(pitch/2) * cos(yaw/2) + sin(roll/2) * cos(pitch/2) * sin(yaw/2)
    qz = cos(roll/2) * cos(pitch/2) * sin(yaw/2) - sin(roll/2) * sin(pitch/2) * cos(yaw/2)
    qw = cos(roll/2) * cos(pitch/2) * cos(yaw/2) + sin(roll/2) * sin(pitch/2) * sin(yaw/2)
    return Quaternion(x=qx, y=qy, z=qz, w=qw)

def main():
    node = StatePublisher()

if __name__ == '__main__':
    main()