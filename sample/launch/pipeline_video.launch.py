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
ROS 2 Launch File: Video Segmentation Pipeline with Optional Visualization

Launches the OpenVINO semantic segmentation pipeline for video files.
Reads video files and performs pixel-wise classification.

Visualization options:
- viewer:=rqt    -> Launches rqt_image_view in standalone mode
- viewer:=rviz2  -> Launches RViz2 with custom config
- viewer:=none   -> No visualization

Topics published:
- /ros2_openvino_toolkit/segmented_objects
- /ros2_openvino_toolkit/image_rviz
"""

import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

# Add the launch directory to Python path to import helper
launch_dir = os.path.dirname(os.path.abspath(__file__))
if launch_dir not in sys.path:
    sys.path.insert(0, launch_dir)

import launch_helpers


def generate_launch_description():
    """Generate launch description for video segmentation pipeline."""

    # Get package share directory
    package_share_dir = get_package_share_directory('openvino_node')

    # Resolve YAML configuration paths
    # Note: video_path will be replaced after LaunchConfiguration is evaluated
    resolved_yaml = launch_helpers.resolve_yaml_paths('pipeline_video.yaml')

    # RViz configuration file
    default_rviz = os.path.join(package_share_dir, 'launch', 'rviz', 'people.rviz')

    # Function to replace video path in YAML
    def replace_video_path(context):
        video_path_value = context.launch_configurations.get('video_path', '')
        if video_path_value:
            # Expand ~ to home directory
            expanded_path = os.path.expanduser(video_path_value)
            # Read the resolved YAML
            with open(resolved_yaml, 'r') as f:
                yaml_content = f.read()
            # Replace the placeholder
            yaml_content = yaml_content.replace('to/be/set/video_path', expanded_path)
            # Write to a new temporary file
            import tempfile
            temp_fd, temp_path = tempfile.mkstemp(suffix='.yaml', prefix='pipeline_video_')
            with os.fdopen(temp_fd, 'w') as f:
                f.write(yaml_content)
            return temp_path
        return resolved_yaml

    from launch.substitutions import LaunchConfiguration
    from launch.actions import OpaqueFunction

    def launch_openvino_node(context):
        yaml_path = replace_video_path(context)
        return [
            Node(
                package='openvino_node',
                executable='pipeline_with_params',
                name='openvino_pipeline',
                arguments=['-config', yaml_path],
                remappings=[
                    ('/openvino_toolkit/segmentation/segmented_objects',
                     '/ros2_openvino_toolkit/segmented_objects'),
                    ('/openvino_toolkit/segmentation/images',
                     '/ros2_openvino_toolkit/image_rviz'),
                ],
                output='screen'
            )
        ]

    return LaunchDescription([
        # Declare launch argument for YAML configuration
        DeclareLaunchArgument(
            name='yaml_path',
            default_value=resolved_yaml,
            description='Path to YAML configuration file for the video segmentation pipeline'
        ),

        # Declare visualization viewer selection
        DeclareLaunchArgument(
            name='viewer',
            default_value='rviz2',
            description='Visualization viewer: "rviz2", "rqt", or "none"'
        ),

        # Declare RViz config path
        DeclareLaunchArgument(
            name='rviz_config',
            default_value=default_rviz,
            description='Path to RViz configuration file'
        ),

        # Declare image topic for rqt
        DeclareLaunchArgument(
            name='image_topic',
            default_value='/ros2_openvino_toolkit/image_rviz',
            description='Image topic for rqt_image_view'
        ),

        # Declare viewer startup delay
        DeclareLaunchArgument(
            name='viewer_delay',
            default_value='2.0',
            description='Delay before launching viewer (seconds)'
        ),

        # Declare video file path
        DeclareLaunchArgument(
            name='video_path',
            default_value='',
            description='Path to video file for processing (required for Video input type)'
        ),

        # Launch OpenVINO node with video path replacement
        OpaqueFunction(function=launch_openvino_node),

        # RViz2 visualization (conditional)
        TimerAction(
            period=LaunchConfiguration('viewer_delay'),
            actions=[
                Node(
                    package='rviz2',
                    executable='rviz2',
                    name='rviz2',
                    arguments=['--display-config', LaunchConfiguration('rviz_config')],
                    output='screen',
                    condition=IfCondition(
                        PythonExpression(["'", LaunchConfiguration('viewer'), "' == 'rviz2'"])
                    )
                )
            ]
        ),

        # rqt_image_view (conditional)
        TimerAction(
            period=LaunchConfiguration('viewer_delay'),
            actions=[
                ExecuteProcess(
                    cmd=['rqt', '--standalone', 'rqt_image_view', '--args', LaunchConfiguration('image_topic')],
                    output='screen',
                    shell=False,
                    condition=IfCondition(
                        PythonExpression(["'", LaunchConfiguration('viewer'), "' == 'rqt'"])
                    )
                )
            ]
        ),
    ])
