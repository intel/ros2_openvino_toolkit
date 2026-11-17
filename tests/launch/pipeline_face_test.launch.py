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
ROS 2 test launch file: Face Detection Pipeline Test.

Launches the OpenVINO face detection with analytics pipeline for automated testing.
"""

import os
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

launch_dir = os.path.dirname(os.path.abspath(__file__))
if launch_dir not in sys.path:
    sys.path.insert(0, launch_dir)

import test_launch_helpers  # noqa: E402


def generate_launch_description():
    """Generate launch description for face detection test."""
    # Resolve path placeholders in the test YAML
    default_yaml = test_launch_helpers.resolve_test_yaml('pipeline_face_test.yaml')

    return LaunchDescription([
        # Declare launch argument for YAML configuration
        DeclareLaunchArgument(
            name='yaml_path',
            default_value=default_yaml,
            description='Path to test YAML configuration file'
        ),

        # OpenVINO face detection test node
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
