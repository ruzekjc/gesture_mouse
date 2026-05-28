#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
import random

class RandomVelPublisher(Node):
    def __init__(self):
        super().__init__('random_vel_publisher')
        self.publisher_ = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.timer = self.create_timer(1.5, self.timer_callback)
        self.get_logger().info('Random Velocity Publisher started.')

    def timer_callback(self):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.twist.linear.x = random.uniform(-0.1, 0.3)
        msg.twist.angular.z = random.uniform(-1.0, 1.0)
        self.publisher_.publish(msg)
        self.get_logger().info(
            f'CMD_VEL -> linear.x: {msg.twist.linear.x:.3f}  angular.z: {msg.twist.angular.z:.3f}'
        )

def main(args=None):
    rclpy.init(args=args)
    node = RandomVelPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
