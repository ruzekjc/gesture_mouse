import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class Talker(Node):
    def __init__(self):
        super().__init__('talker')
        self.pub = self.create_publisher(String, '/chatter', 10)
        self.create_timer(1.0, self.publishing)

    def publishing(self):
        msg = String()
        msg.data = 'Hi this is my first project demo'
        self.pub.publish(msg)
        self.get_logger().info('Publishing: ' + msg.data)

rclpy.init()
rclpy.spin(Talker())