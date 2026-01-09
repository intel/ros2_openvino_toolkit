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
ROS 2 Launch File: Multi-Pipeline Service

Launches multiple OpenVINO detection pipelines simultaneously.
Useful for running different models or configurations in parallel.

This configuration runs two object detection pipelines with separate topics:
- Pipeline 1: /ros2_openvino_toolkit/detected_objects1
- Pipeline 2: /ros2_openvino_toolkit/detected_objects2

Note: Requires sufficient compute resources (CPU/GPU) for multiple pipelines.
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
    """Generate launch description for multi-pipeline service with RViz2."""

    # Get package share directory
    package_share_dir = get_package_share_directory('openvino_node')

    # Resolve YAML configuration paths
    resolved_yaml = launch_helpers.resolve_yaml_paths('multi_pipeline_service.yaml')

    # RViz configuration for dual pipeline visualization
    dual_rviz = os.path.join(package_share_dir, 'launch', 'rviz', 'default2.rviz')

    return LaunchDescription([
        # Declare launch argument for YAML configuration
        DeclareLaunchArgument(
            name='yaml_path',
            default_value=resolved_yaml,
            description='Path to YAML configuration file for multi-pipeline service'
        ),

        # OpenVINO multi-pipeline node
        Node(
            package='openvino_node',
            executable='pipeline_with_params',
            name='openvino_multi_pipeline',
            arguments=['-config', LaunchConfiguration('yaml_path')],
            remappings=[
                ('/openvino_toolkit/object1/detected_objects',
                 '/ros2_openvino_toolkit/detected_objects1'),
                ('/openvino_toolkit/object2/detected_objects',
                 '/ros2_openvino_toolkit/detected_objects2'),
                ('/openvino_toolkit/object1/images',
                 '/ros2_openvino_toolkit/image_rviz1'),
                ('/openvino_toolkit/object2/images',
                 '/ros2_openvino_toolkit/image_rviz2'),
            ],
            output='screen'
        ),

        # RViz2 visualization with dual-view config
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['--display-config', dual_rviz],
            output='screen'
        ),
    ])
