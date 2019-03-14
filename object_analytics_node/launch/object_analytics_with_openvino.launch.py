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
    # depth_image_proc = 'depth_image_proc'
    # depth_image_proc_plugin = 'depth_image_proc::PointCloudXyzrgbNode'
    default_yaml = os.path.join(get_package_share_directory('dynamic_vino_sample'), 'param',
                                'pipeline_object_oss_topic.yaml')

    default_rviz = os.path.join(get_package_share_directory('object_analytics_node'), 'launch',
                                'rviz/default.rviz')
    default_rsconfig = os.path.join(get_package_share_directory(
        'object_analytics_node'), 'launch', 'rs_param.yaml')
    return LaunchDescription([
        # Realsense
        launch_ros.actions.Node(
            package='realsense_ros2_camera', node_executable='realsense_ros2_camera',
            arguments=['__params:=' + default_rsconfig],
            output='screen'),

        # api_composition
        # launch_ros.actions.Node(
        #     package='composition', node_executable='api_composition', output='screen',
        #     remappings=[('rgb/camera_info', '/camera/color/camera_info'),
        #                 ('rgb/image_rect_color', '/camera/color/image_raw'),
        #                 ('depth_registered/image_rect',
        #                  '/camera/aligned_depth_to_color/image_raw'),
        #                 ('points', '/camera/depth/color/points')]),

        # depth_image_proc
        # TODO: enable depth_image_proc when ros2 image_pipeline is ready
        # launch_ros.actions.Node(
        #     package='composition', node_executable='api_composition_cli', output='screen',
        #     arguments=[depth_image_proc, depth_image_proc_plugin]),

        # Openvino Detection
        launch_ros.actions.Node(
            package='dynamic_vino_sample', node_executable='pipeline_with_params',
            arguments=['-config', default_yaml],
            remappings=[
                ('/openvino_toolkit/image_raw', '/camera/color/image_raw'),
                ('/openvino_toolkit/detected_objects', '/ros2_openvino_toolkit/detected_objects'),
                ('/openvino_toolkit/images', '/ros2_openvino_toolkit/image_rviz')],
            output='screen'),

        # object_analytics_node
        launch_ros.actions.Node(
            package='object_analytics_node', node_executable='object_analytics_node',
            arguments=['--tracking', '--localization'],
            remappings=[
                ('/object_analytics/detected_objects', '/ros2_openvino_toolkit/detected_objects'),
                ('/object_analytics/rgb', '/camera/color/image_raw'),
                ('/object_analytics/pointcloud', '/camera/aligned_depth_to_color/color/points')],
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
