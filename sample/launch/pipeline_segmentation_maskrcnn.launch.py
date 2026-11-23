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
ROS 2 Launch File: Instance Segmentation Pipeline (Mask R-CNN)

Launches the OpenVINO instance segmentation pipeline using Mask R-CNN.
Performs object detection with pixel-perfect segmentation masks.



Visualization options:
- viewer:=rqt    -> Launches rqt_image_view in standalone mode (default)
- viewer:=rviz2  -> Launches RViz2 with custom config
- viewer:=none   -> No visualization

Note: The input is StandardCamera: the first /dev/video* device that is not a RealSense
      camera (e.g. a laptop or USB webcam), opened directly without a camera node.
      RViz2 visualization is disabled by default for this pipeline.

Topics:
- Publishes: /ros2_openvino_toolkit/segmented_objects
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
    """Generate launch description for Mask R-CNN instance segmentation."""
    return launch_helpers.pipeline_launch_description(
        'pipeline_segmentation_maskrcnn.yaml', 'segmentation', ['segmented_objects'])
