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
ROS 2 Launch File: Standalone RViz2 for OpenVINO Toolkit

Launches RViz2 with the default configuration for visualizing
OpenVINO toolkit output. Use this when running the pipeline separately.

For integrated pipeline + visualization, use:
- pipeline_*_topic.launch.py files
- pipeline_object_with_rqt.launch.py (for rqt_image_view)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for standalone RViz2."""

    # Get package share directory
    package_share_dir = get_package_share_directory('openvino_node')

    # Default RViz configuration
    default_rviz_config = os.path.join(
        package_share_dir,
        'launch',
        'rviz',
        'default.rviz'
    )

    return LaunchDescription([
        # Declare launch argument for custom RViz config
        DeclareLaunchArgument(
            name='rviz_config',
            default_value=default_rviz_config,
            description='Path to RViz2 configuration file'
        ),

        # RViz2 node
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['--display-config', LaunchConfiguration('rviz_config')],
            output='screen'
        ),
    ])
