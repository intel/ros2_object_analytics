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

from ros2run.api import get_executable_path


file_path = os.path.dirname(os.path.realpath(__file__))


def launch(launch_descriptor, argv):
    ld = launch_descriptor

    package = 'object_analytics_node'
    ld.add_process(
        cmd=[
            get_executable_path(package_name=package, executable_name='composition'),
            '--localization',                     # enable localization feature, optional
            '--tracking',                         # enable tracking feature, optional
            '--detect-module',                    # detection backend module name, must
            'movidius_ncs_stream',
            '--detect-class',                     # detection backend class name, must
            'movidius_ncs_stream::NCSComposition'
        ],
    )
    return ld
