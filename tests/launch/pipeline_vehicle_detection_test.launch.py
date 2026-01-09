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
ROS 2 Test Launch File: Vehicle Detection Pipeline Test

Launches the OpenVINO vehicle detection pipeline for automated testing.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for vehicle detection test."""

    # Get test package share directory
    test_package_dir = get_package_share_directory('openvino_test')
    default_yaml = os.path.join(test_package_dir, 'param', 'pipeline_vehicle_detection_test.yaml')

    return LaunchDescription([
        # Declare launch argument for YAML configuration
        DeclareLaunchArgument(
            name='yaml_path',
            default_value=default_yaml,
            description='Path to test YAML configuration file'
        ),

        # OpenVINO vehicle detection test node
        Node(
            package='openvino_node',
            executable='pipeline_with_params',
            name='openvino_test_pipeline',
            arguments=['-config', LaunchConfiguration('yaml_path')],
            remappings=[
                ('/openvino_toolkit/object/detected_license_plates',
                 '/ros2_openvino_toolkit/detected_license_plates'),
                ('/openvino_toolkit/object/detected_vehicles_attribs',
                 '/ros2_openvino_toolkit/detected_vehicles_attribs'),
            ],
            output='screen'
        ),
    ])
