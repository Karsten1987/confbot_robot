import rclpy
from rclpy.clock import Clock
from rclpy.node import Node

from geometry_msgs.msg import Twist
from visualization_msgs.msg import Marker


class PathPublisher(Node):

    def __init__(self):
        super().__init__('path_publisher')
        self.i = 0
        self.pub = self.create_publisher(Marker, 'path')
        self.pub_vel = self.create_publisher(Twist, 'cmd_vel')
        timer_period = 1.0
        self.tmr = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = Marker()
        msg.header.stamp = Clock().now().to_msg()
        msg.id = 1
        msg.type = 3
        msg.color.r = 0.1
        msg.color.g = 0.7
        msg.color.b = 0.1
        msg.color.a = 0.7
        msg.header.frame_id = 'odom'
        msg.scale.x = 5.5
        msg.scale.y = 5.5
        msg.scale.z = 0.01

        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    node = PathPublisher()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
