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
ROS 2 Launch File: Object Detection Pipeline with RViz2

Launches the OpenVINO object detection pipeline with RViz2 visualization.
This version subscribes to image topics (e.g., from RealSense camera).



Visualization options:
- viewer:=rqt    -> Launches rqt_image_view in standalone mode (default)
- viewer:=rviz2  -> Launches RViz2 with custom config
- viewer:=none   -> No visualization

Note: This launch file does NOT start the camera node. Launch your camera
      separately (e.g., realsense2_camera) before running this.

Topics:
- Subscribes to: /camera/color/image_raw (or configured input)
- Publishes: /ros2_openvino_toolkit/detected_objects
- Publishes: /ros2_openvino_toolkit/image_rviz
"""

import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

# Add the launch directory to Python path to import helper
launch_dir = os.path.dirname(os.path.abspath(__file__))
if launch_dir not in sys.path:
    sys.path.insert(0, launch_dir)

import launch_helpers


def generate_launch_description():
    """Generate launch description for object detection with RViz2."""

    # Get package share directory
    package_share_dir = get_package_share_directory('openvino_node')

    # Resolve YAML configuration paths
    resolved_yaml = launch_helpers.resolve_yaml_paths('pipeline_object_topic.yaml')

    # RViz configuration file
    default_rviz = os.path.join(package_share_dir, 'launch', 'rviz', 'people.rviz')

    return LaunchDescription([
        # Declare launch argument for YAML configuration
        DeclareLaunchArgument(
            name='yaml_path',
            default_value=resolved_yaml,
            description='Path to YAML configuration file for the OpenVINO pipeline'
        ),


        # Declare visualization viewer selection
        DeclareLaunchArgument(
            name='viewer',
            default_value='rqt',
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

        # OpenVINO object detection node
        Node(
            package='openvino_node',
            executable='pipeline_with_params',
            name='openvino_pipeline',
            arguments=['-config', LaunchConfiguration('yaml_path')],
            remappings=[
                ('/openvino_toolkit/object/detected_objects',
                 '/ros2_openvino_toolkit/detected_objects'),
                ('/openvino_toolkit/object/images',
                 '/ros2_openvino_toolkit/image_rviz'),
            ],
            output='screen'
        ),
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
