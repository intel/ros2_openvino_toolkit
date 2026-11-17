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
ROS 2 Launch File: Composable Object Detection Pipeline

Launches OpenVINO object detection pipeline using ROS 2 composable nodes
for zero-copy intra-process communication. This approach improves performance
by avoiding message serialization overhead.

Composable nodes:
- RealSense camera node (optional, if available)
- OpenVINO composable pipeline

Benefits:
- Reduced latency through intra-process communication
- Lower CPU overhead
- Better real-time performance

Topics published:
- /ros2_openvino_toolkit/detected_objects
- /ros2_openvino_toolkit/image_rviz
"""

import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

# Add the launch directory to Python path to import helper
launch_dir = os.path.dirname(os.path.abspath(__file__))
if launch_dir not in sys.path:
    sys.path.insert(0, launch_dir)

import launch_helpers


def generate_launch_description():
    """Generate launch description for composable object detection pipeline."""

    # Get package share directory
    try:
        realsense_config_dir = get_package_share_directory('realsense_examples')
        realsense_config = os.path.join(realsense_config_dir, 'config', 'd435i.yaml')
    except:
        # Fallback if realsense_examples not installed
        realsense_config = ''

    # Resolve YAML configuration paths
    resolved_yaml = launch_helpers.resolve_yaml_paths('pipeline_composite_object_topic.yaml')

    return LaunchDescription([
        # Declare launch argument for YAML configuration
        DeclareLaunchArgument(
            name='yaml_path',
            default_value=resolved_yaml,
            description='Path to YAML configuration file for the composable pipeline'
        ),

        # Composable node container with OpenVINO pipeline
        # Note: RealSense node is commented out - uncomment if using RealSense camera
        ComposableNodeContainer(
            name='vision_pipeline_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                # Uncomment to include RealSense camera as composable node:
                # ComposableNode(
                #     package='realsense_ros',
                #     plugin='realsense::RealSenseNodeFactory',
                #     name='realsense',
                #     parameters=[realsense_config] if realsense_config else [],
                #     extra_arguments=[{'use_intra_process_comms': True}]
                # ),

                # OpenVINO composable pipeline node
                ComposableNode(
                    package='openvino_node',
                    plugin='ComposablePipeline',
                    name='composable_pipeline',
                    parameters=[{'config': LaunchConfiguration('yaml_path')}],
                    remappings=[
                        ('/openvino_toolkit/object/detected_objects',
                         '/ros2_openvino_toolkit/detected_objects'),
                        ('/openvino_toolkit/object/images',
                         '/ros2_openvino_toolkit/image_rviz'),
                    ],
                    extra_arguments=[{'use_intra_process_comms': True}]
                ),
            ],
            output='screen',
        ),
    ])
