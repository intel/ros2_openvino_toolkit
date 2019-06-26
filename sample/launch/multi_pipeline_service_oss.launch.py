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

"""Launch face detection and rviz."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
import launch_ros.actions


def generate_launch_description():
    default_yaml = os.path.join(get_package_share_directory('dynamic_vino_sample'), 'param',
                                'multi_pipleine_service_object_oss.yaml')
    # default_rviz = os.path.join(get_package_share_directory('dynamic_vino_sample'), 'launch',
    #                             'rviz/default.rviz')
    return LaunchDescription([
        # Openvino detection
        launch_ros.actions.Node(
            package='dynamic_vino_sample', node_executable='pipeline_with_params',
            arguments=['-config', default_yaml],
            output='screen'),
    ])
