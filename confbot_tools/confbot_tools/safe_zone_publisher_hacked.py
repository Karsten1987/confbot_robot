# Copyright 2018 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import math

from geometry_msgs.msg import Twist

import rclpy
from rclpy.clock import Clock
from rclpy.node import Node

from visualization_msgs.msg import Marker


class SafeZonePublisher(Node):

    def __init__(self):
        super().__init__('safe_zone_publisher')

        self.pub = self.create_publisher(Marker, 'safe_zone')
        self.hacked_pub = self.create_publisher(Twist, 'cmd_vel')
        self.shark_pub = self.create_publisher(Marker, 'danger_zone')
        timer_period = 1.0
        self.tmr = self.create_timer(timer_period, self.timer_callback)

        self.marker = Marker()
        self.marker.id = 1
        self.marker.type = 3
        self.marker.color.r = 0.1
        self.marker.color.g = 0.7
        self.marker.color.b = 0.1
        self.marker.color.a = 0.7
        self.marker.header.frame_id = 'odom'
        self.marker.scale.x = 5.5
        self.marker.scale.y = 5.5
        self.marker.scale.z = 0.01

        self.shark_heading = math.pi / 2
        self.shark_x = 0.0
        self.shark_y = -3.5

        self.shark = Marker()
        self.shark.id = 2
        self.shark.header.frame_id = 'odom'
        self.shark.pose.position.z = -1.0
        self.shark.scale.x = 0.05
        self.shark.scale.y = 0.05
        self.shark.scale.z = 0.05
        self.shark.color.r = 1.0
        self.shark.color.g = 0.0
        self.shark.color.b = 0.0
        self.shark.color.a = 1.0
        self.shark.type = 10
        # https://www.yobi3d.com/
        # https://www.yobi3d.com/v/tivZ0KYsrZ/Shark_t.stl
        # mesh is licensed under CC-BY
        self.shark.mesh_resource = 'file:///Users/karsten/workspace/osrf/roscon2018_ws/src/roscon2018/confbot_description/meshes/Shark_t.stl'  # noqa E501

    def timer_callback(self):

        self.marker.header.stamp = Clock().now().to_msg()
        self.shark.header.stamp = Clock().now().to_msg()

        self.shark_heading += 0.3
        self.shark_x += math.sin(self.shark_heading)
        self.shark_y -= math.cos(self.shark_heading)
        cy = math.cos(self.shark_heading * 0.5)
        sy = math.sin(self.shark_heading * 0.5)
        cr = math.cos(0 * 0.5)
        sr = math.sin(0 * 0.5)
        cp = math.cos(0 * 0.5)
        sp = math.sin(0 * 0.5)

        self.shark.pose.position.x = self.shark_x
        self.shark.pose.position.y = self.shark_y
        self.shark.pose.orientation.w = cy * cr * cp + sy * sr * sp
        self.shark.pose.orientation.x = cy * sr * cp - sy * cr * sp
        self.shark.pose.orientation.y = cy * cr * sp + sy * sr * cp
        self.shark.pose.orientation.z = sy * cr * cp - cy * sr * sp

        self.pub.publish(self.marker)
        self.shark_pub.publish(self.shark)


def main(args=None):
    rclpy.init(args=args)

    node = SafeZonePublisher()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
