# Jada Ruzek
# ROS 2 subscriber that consumes normalized hand position from gesture_publisher,
# solves 2-DOF planar inverse kinematics, and publishes joint commands.

# Subscribes:
#   - /hand_position (geometry_msgs/Point) - normalized [0,1] x, y

# Publishes:
#   - /joint_states (sensor_msgs/JointState) - shoulder + elbow angles in radians

import math

import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from sensor_msgs.msg import JointState

L1 = 0.5
L2 = 0.5

X_MIN, X_MAX = -0.7, 0.7
Y_MIN, Y_MAX = 0.1, 0.9

# Joint limits (radians); a final safety clamp after IK
SHOULDER_MIN, SHOULDER_MAX = -math.pi, math.pi
ELBOW_MIN, ELBOW_MAX = -math.pi, math.pi

JOINT_NAMES = ['shoulder_joint', 'elbow_joint']


class ArmController(Node):
    def __init__(self):
        super().__init__('arm_controller')

        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.position_sub = self.create_subscription(
            Point, '/hand_position', self.position_callback, 10
        )

        self.get_logger().info(
            'arm_controller running. L1=%.2f L2=%.2f' % (L1, L2)
        )

    def position_callback(self, msg: Point):
        target_x = X_MIN + msg.x * (X_MAX - X_MIN)
        target_y = Y_MIN + (1.0 - msg.y) * (Y_MAX - Y_MIN)

        shoulder, elbow = self.solve_ik(target_x, target_y)

        # Final safety clamp to declared joint limits
        shoulder = max(SHOULDER_MIN, min(SHOULDER_MAX, shoulder))
        elbow = max(ELBOW_MIN, min(ELBOW_MAX, elbow))

        out = JointState()
        out.header.stamp = self.get_clock().now().to_msg()
        out.name = JOINT_NAMES
        out.position = [shoulder, elbow]
        self.joint_pub.publish(out)

    def solve_ik(self, x: float, y: float):
        r = math.hypot(x, y)
        r_min = abs(L1 - L2) + 1e-3
        r_max = (L1 + L2) - 1e-3

        if r < r_min:
            scale = r_min / max(r, 1e-6)
            x, y = x * scale, y * scale
            r = r_min
        elif r > r_max:
            scale = r_max / r
            x, y = x * scale, y * scale
            r = r_max

        cos_elbow = (r * r - L1 * L1 - L2 * L2) / (2.0 * L1 * L2)
        cos_elbow = max(-1.0, min(1.0, cos_elbow))
        theta2 = math.acos(cos_elbow)

        k1 = L1 + L2 * math.cos(theta2)
        k2 = L2 * math.sin(theta2)
        theta1 = math.atan2(y, x) - math.atan2(k2, k1)

        return theta1, theta2


def main(args=None):
    rclpy.init(args=args)
    node = ArmController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
