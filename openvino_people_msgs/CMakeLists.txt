# Copyright (c) 2018-2022 Intel Corporation
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

cmake_minimum_required(VERSION 3.5)

project(openvino_people_msgs)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(object_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Emotion.msg"
  "msg/EmotionsStamped.msg"
  "msg/AgeGender.msg"
  "msg/AgeGenderStamped.msg"
  "msg/HeadPose.msg"
  "msg/HeadPoseStamped.msg"
  "msg/PersonsStamped.msg"
  "msg/ObjectInMask.msg"
  "msg/ObjectsInMasks.msg"
  "msg/Reidentification.msg"
  "msg/ReidentificationStamped.msg"
  "msg/PersonAttribute.msg"
  "msg/PersonAttributeStamped.msg"
  "msg/Landmark.msg"
  "msg/LandmarkStamped.msg"
  "msg/VehicleAttribs.msg"
  "msg/VehicleAttribsStamped.msg"
  "msg/LicensePlate.msg"
  "msg/LicensePlateStamped.msg"
  "srv/AgeGenderSrv.srv"
  "srv/EmotionSrv.srv"
  "srv/HeadPoseSrv.srv"
  "srv/People.srv"
  DEPENDENCIES builtin_interfaces std_msgs sensor_msgs object_msgs geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
