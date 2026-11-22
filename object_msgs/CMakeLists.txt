# Copyright (c) 2017 Intel Corporation
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

cmake_minimum_required(VERSION 3.10)
project(object_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

set(msg_files
  msg/AgeGender.msg
  msg/AgeGenderStamped.msg
  msg/Emotion.msg
  msg/EmotionsStamped.msg
  msg/HeadPose.msg
  msg/HeadPoseStamped.msg
  msg/Landmark.msg
  msg/LandmarkStamped.msg
  msg/LicensePlate.msg
  msg/LicensePlateStamped.msg
  msg/Object.msg
  msg/Objects.msg
  msg/ObjectInBox.msg
  msg/ObjectsInBoxes.msg
  msg/ObjectInBox3D.msg
  msg/ObjectsInBoxes3D.msg
  msg/ObjectInMask.msg
  msg/ObjectsInMasks.msg
  msg/PerfValue.msg
  msg/PersonAttribute.msg
  msg/PersonAttributeStamped.msg
  msg/PersonsStamped.msg
  msg/Reidentification.msg
  msg/ReidentificationStamped.msg
  msg/VehicleAttribs.msg
  msg/VehicleAttribsStamped.msg
)

set(srv_files
  srv/ClassifyObject.srv
  srv/DetectObject.srv
  srv/AgeGenderSrv.srv
  srv/EmotionSrv.srv
  srv/HeadPoseSrv.srv
  srv/People.srv

)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES sensor_msgs std_msgs geometry_msgs)

ament_package()
