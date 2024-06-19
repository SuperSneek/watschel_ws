import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Joy

class JoySubscriber(Node):
    def __init__(self):
        super().__init__('joy_subscriber')
        self.subscription = self.create_subscription(
            Joy,
            '/joy',
            self.listener_callback,
            10)
        self.subscription 

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.buttons)
