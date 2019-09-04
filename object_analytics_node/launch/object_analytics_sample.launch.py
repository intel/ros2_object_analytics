# Copyright (c) 2018 Intel Corporation
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
import launch_ros.actions


def generate_launch_description():
    default_rviz = os.path.join(get_package_share_directory('object_analytics_node'), 'launch',
                                'rviz/default.rviz')
    return LaunchDescription([
        # object_analytics_node
        launch_ros.actions.Node(
            package='object_analytics_node', node_executable='object_analytics_node',
            arguments=['--tracking', '--localization'],
            remappings=[
                ('/object_analytics/detected_objects', '/ros2_openvino_toolkit/detected_objects'),
                ('/object_analytics/rgb', '/camera/color/image_raw'),
                ('/object_analytics/pointcloud', '/camera/pointcloud')],
            output='screen'),

        # object_analytics_rviz
        launch_ros.actions.Node(
            package='object_analytics_rviz', node_executable='image_publisher',
            remappings=[
                ('/object_analytics/rgb', '/camera/color/image_raw')],
            output='screen'),
        launch_ros.actions.Node(
            package='object_analytics_rviz', node_executable='marker_publisher', output='screen'),

        # rviz
        launch_ros.actions.Node(
            package='rviz2', node_executable='rviz2', output='screen',
            arguments=['--display-config', default_rviz]),
    ])
