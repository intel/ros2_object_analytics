# Copyright 2018 Intel Corporation
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

from launch.legacy.exit_handler import default_exit_handler
from ros2run.api import get_executable_path


def launch(launch_descriptor, argv):
    ld = launch_descriptor

    package = 'object_analytics_rviz'
    ld.add_process(
        cmd=[
            get_executable_path(package_name=package, executable_name='marker_publisher'),
        ],
        name='marker_publisher',
        exit_handler=default_exit_handler,
    )
    package = 'object_analytics_rviz'
    ld.add_process(
        cmd=[
            get_executable_path(package_name=package, executable_name='image_publisher'),
        ],
        name='image_publisher',
        exit_handler=default_exit_handler,
    )
    package = 'rviz2'
    default_rviz = os.path.join(get_package_share_directory('object_analytics_launch'),
                                'launch', 'default.rviz')
    ld.add_process(
        cmd=[
            get_executable_path(package_name=package, executable_name='rviz2'),
            '--display-config', default_rviz,
        ],
        name='object_rviz',
        exit_handler=default_exit_handler,
    )
    return ld
