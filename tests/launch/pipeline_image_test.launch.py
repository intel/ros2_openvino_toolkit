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

"""
ROS 2 Test Launch File: Image Pipeline Test

Launches the OpenVINO people analytics pipeline on static images for automated testing.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for image pipeline test."""

    # Get test package share directory
    test_package_dir = get_package_share_directory('openvino_test')
    default_yaml = os.path.join(test_package_dir, 'param', 'pipeline_image_test.yaml')

    return LaunchDescription([
        # Declare launch argument for YAML configuration
        DeclareLaunchArgument(
            name='yaml_path',
            default_value=default_yaml,
            description='Path to test YAML configuration file'
        ),

        # OpenVINO image pipeline test node
        Node(
            package='openvino_node',
            executable='pipeline_with_params',
            name='openvino_test_pipeline',
            arguments=['-config', LaunchConfiguration('yaml_path')],
            remappings=[
                ('/openvino_toolkit/people/detected_objects',
                 '/ros2_openvino_toolkit/face_detection'),
                ('/openvino_toolkit/people/emotions',
                 '/ros2_openvino_toolkit/emotions_recognition'),
                ('/openvino_toolkit/people/headposes',
                 '/ros2_openvino_toolkit/headposes_estimation'),
                ('/openvino_toolkit/people/age_genders',
                 '/ros2_openvino_toolkit/age_genders_Recognition'),
                ('/openvino_toolkit/images',
                 '/ros2_openvino_toolkit/image_rviz'),
            ],
            output='screen'
        ),
    ])
