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

This is a lightweight launcher without visualization by default (viewer:=rqt or
viewer:=rviz2 to add one), suitable for integration with other ROS 2 nodes or
analytics pipelines.

Note: The input is StandardCamera: the first /dev/video* device that is not a RealSense
      camera (e.g. a laptop or USB webcam), opened directly without a camera node.

Topics:
- Publishes: /ros2_openvino_toolkit/detected_objects
- Publishes: /ros2_openvino_toolkit/image_rviz
"""

import os
import sys

# Add the launch directory to Python path to import helper
launch_dir = os.path.dirname(os.path.abspath(__file__))
if launch_dir not in sys.path:
    sys.path.insert(0, launch_dir)

import launch_helpers  # noqa: E402  # pylint: disable=wrong-import-position


def generate_launch_description():
    """Generate launch description for OpenVINO Object Analytics."""
    return launch_helpers.pipeline_launch_description(
        'pipeline_object_topic.yaml', 'object', ['detected_objects'], default_viewer='none')
