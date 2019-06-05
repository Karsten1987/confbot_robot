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

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription

from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    urdf = os.path.join(get_package_share_directory('confbot_description'),
                        'urdf', 'confbot.urdf')

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            node_executable='robot_state_publisher',
            output='screen', arguments=[urdf]),
        Node(
            package='confbot_tools',
            node_executable='safe_zone_publisher',
            output='screen'),
        ComposableNodeContainer(
            node_name='confbot_driver_container',
            node_namespace='',
            package='rclcpp_components',
            node_executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='confbot_driver',
                    node_plugin='confbot_driver::nodes::ConfbotDriver',
                    node_name='confbot_driver'),
                ComposableNode(
                    package='confbot_driver',
                    node_plugin='confbot_driver::nodes::TwistPublisher',
                    node_name='twist_publisher')
            ],
            output='screen',),
        ComposableNodeContainer(
            node_name='confbot_sensors_container',
            node_namespace='',
            package='rclcpp_components',
            node_executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='confbot_sensors',
                    node_plugin='confbot_sensors::nodes::ConfbotLaser',
                    node_name='confbot_laser'),
            ],
            output='screen',)
    ])
