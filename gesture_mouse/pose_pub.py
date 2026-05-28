import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
import math


def quaternion_to_yaw(q):
    """Convert a quaternion to yaw angle (rotation around Z axis) in degrees."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    yaw_rad = math.atan2(siny_cosp, cosy_cosp)
    return math.degrees(yaw_rad)


class PosePublisher(Node):
    def __init__(self):
        super().__init__('turtlebot_pose_publisher')
        # subscribes to odometry
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        # Republish as PoseStamped
        self.publisher_ = self.create_publisher(PoseStamped, '/turtlebot_pose', 10)
        self.get_logger().info('TurtleBot Pose Publisher node started.')
        self.get_logger().info('Listening to /odom -> republishing to /turtlebot_pose ...')

    def odom_callback(self, msg):
        # builds a PoseStamped from the odometry message
        pose_msg = PoseStamped()
        pose_msg.header = msg.header
        pose_msg.pose = msg.pose.pose

        self.publisher_.publish(pose_msg)

        # log pose info
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        yaw = quaternion_to_yaw(msg.pose.pose.orientation)
        self.get_logger().info(f'POSE -> x: {x:.4f}  y: {y:.4f}  yaw: {yaw:.2f} deg')


def main(args=None):
    rclpy.init(args=args)
    node = PosePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
