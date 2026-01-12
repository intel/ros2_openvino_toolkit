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
ROS 2 Launch File: OpenVINO Object Analytics (Topic-based)

Launches the OpenVINO object detection pipeline for integration with
OpenVINO Object Analytics (OA) framework.

This is a lightweight launcher without visualization, suitable for
integration with other ROS 2 nodes or analytics pipelines.

Note: Launch your camera node (e.g., realsense2_camera) separately.

Topics:
- Subscribes to: /camera/color/image_raw
- Publishes: /ros2_openvino_toolkit/detected_objects
- Publishes: /ros2_openvino_toolkit/image_rviz
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
    """Generate launch description for OpenVINO Object Analytics."""

    # Resolve YAML configuration paths
    resolved_yaml = launch_helpers.resolve_yaml_paths('pipeline_object_topic.yaml')

    return LaunchDescription([
        # Declare launch argument for YAML configuration
        DeclareLaunchArgument(
            name='yaml_path',
            default_value=resolved_yaml,
            description='Path to YAML configuration file for the OpenVINO OA pipeline'
        ),

        # OpenVINO object detection node (for Object Analytics)
        Node(
            package='openvino_node',
            executable='pipeline_with_params',
            name='openvino_pipeline',
            arguments=['-config', LaunchConfiguration('yaml_path')],
            remappings=[
                ('/openvino_toolkit/image_raw',
                 '/camera/color/image_raw'),
                ('/openvino_toolkit/object/detected_objects',
                 '/ros2_openvino_toolkit/detected_objects'),
                ('/openvino_toolkit/object/images',
                 '/ros2_openvino_toolkit/image_rviz'),
            ],
            output='screen'
        ),
    ])
