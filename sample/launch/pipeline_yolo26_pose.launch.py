# Copyright (c) 2026 Intel Corporation
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
ROS 2 Launch File: YOLO26 Keypoint Detection (Pose) Pipeline

Detects persons and their body keypoints in the frames of a camera with an Ultralytics
YOLO26 model.
Export the model first: ros2 run openvino_node export_yolo26.sh --tasks pose

Visualization options:
- viewer:=rqt    -> Launches rqt_image_view in standalone mode (default)
- viewer:=rviz2  -> Launches RViz2 with custom config
- viewer:=none   -> No visualization

Topics published:
- /ros2_openvino_toolkit/detected_keypoints
- /ros2_openvino_toolkit/image_rviz
"""

import os
import sys

# Add the launch directory to Python path to import helper
launch_dir = os.path.dirname(os.path.abspath(__file__))
if launch_dir not in sys.path:
    sys.path.insert(0, launch_dir)

import launch_helpers  # noqa: E402  # pylint: disable=wrong-import-position


def generate_launch_description():
    """Generate launch description for YOLO26 keypoint detection."""
    return launch_helpers.pipeline_launch_description(
        'pipeline_yolo26_pose.yaml', 'pose', ['detected_keypoints'])
