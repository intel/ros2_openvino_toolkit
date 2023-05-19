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

from launch.substitutions import LaunchConfiguration, PythonExpression
import launch

def generate_launch_description():
    #default_yaml = os.path.join(get_package_share_directory('dynamic_vino_sample'), 'param',
                                #'pipeline_video.yaml')
    default_rviz = os.path.join(get_package_share_directory('openvino_node'), 'launch',
                                'rviz/default.rviz')
    return LaunchDescription([
    	launch.actions.DeclareLaunchArgument(name='yaml_path', default_value = 
                                             os.path.join(get_package_share_directory('openvino_node'), 'param','pipeline_human_pose.yaml')),
        # Openvino detection
        launch_ros.actions.Node(
            package='openvino_node',
            executable='pipeline_with_params',
            arguments=['-config', LaunchConfiguration('yaml_path')],
            remappings=[
                ('/openvino_toolkit/people/images', '/ros2_openvino_toolkit/image_rviz')],
            output='screen'),

        # Rviz
        launch_ros.actions.Node(
            package='rviz2',
            executable='rviz2', output='screen',
            arguments=['--display-config', default_rviz]),
    ])
