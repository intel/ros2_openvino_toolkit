# Copyright (c) 2018 Intel Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# This message can represent a 3D detection object with 2D region of interest and 3D information (min & max)
Object object             # detected object
sensor_msgs/RegionOfInterest roi      # region of interest
geometry_msgs/Point32 min             # min and max locate the diagonal of a bounding-box of the detected object whose
geometry_msgs/Point32 max             # x, y and z axis parellel to the axises correspondingly in camera coordinates
