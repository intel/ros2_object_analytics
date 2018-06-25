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

from launch.exit_handler import default_exit_handler, restart_exit_handler
from ros2run.api import get_executable_path


def launch(launch_descriptor, argv):
    ld = launch_descriptor
    package = 'object_analytics_visualization'
#    ld.add_process(
#        cmd=[
#            get_executable_path(package_name=package, executable_name='image_publisher'),
#        ],
#        name='visualization2d',
#        exit_handler=default_exit_handler,
#    )
    package = 'object_analytics_visualization'
    ld.add_process(
        cmd=[
            get_executable_path(package_name=package, executable_name='marker_publisher'),
        ],
        name='visualization3d',
        exit_handler=default_exit_handler,
    )
    return ld
