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
ROS 2 Launch File: Image People Analytics Service Server

Launches the OpenVINO people analytics service server for on-demand inference.
Provides ROS 2 service interface for face detection, age/gender, emotion,
and head pose estimation on static images.

This is a service-based interface (request/response) rather than continuous
stream processing.
"""

import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Add the launch directory to Python path to import helper
launch_dir = os.path.dirname(os.path.abspath(__file__))
if launch_dir not in sys.path:
    sys.path.insert(0, launch_dir)

import launch_helpers


def generate_launch_description():
    """Generate launch description for image people analytics service."""

    # Resolve YAML configuration paths
    resolved_yaml = launch_helpers.resolve_yaml_paths('image_people_server.yaml')

    return LaunchDescription([
        # Declare launch argument for YAML configuration
        DeclareLaunchArgument(
            name='yaml_path',
            default_value=resolved_yaml,
            description='Path to YAML configuration file for the OpenVINO service'
        ),

        # OpenVINO image people analytics service node
        Node(
            package='openvino_node',
            executable='image_people_server',
            name='image_people_server',
            arguments=['-config', LaunchConfiguration('yaml_path')],
            output='screen'
        ),
    ])
