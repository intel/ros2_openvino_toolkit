cmake_minimum_required(VERSION 3.5)
project(openvino_ros)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rmw REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(class_loader REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(message_filters REQUIRED)
find_package(rdk_interfaces REQUIRED)

set(InferenceEngine_DIR /opt/intel/computer_vision_sdk/deployment_tools/inference_engine/share)
find_package(InferenceEngine 1.1)
if(NOT InferenceEngine_FOUND)
    message(FATAL_ERROR "\n\n Intel OpenVINO toolkit is missing, please install it from https://software.intel.com/en-us/openvino-toolkit\n\n")
endif()

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${InferenceEngine_INCLUDE_DIRS}
  ${rdk_interfaces_INCLUDE_DIRS}
  ${message_filters_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} SHARED
  src/openvino_factory.cpp
  src/openvino_base.cpp
  src/object_detection_ssd.cpp
  src/object_detection_yolov2.cpp
  src/reidentification.cpp
  src/object_segmentation.cpp
  src/tracker.cpp
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_components
  rmw_implementation
  cv_bridge
  sensor_msgs
  std_msgs
  tf2_ros
  tf2
  message_filters
  rdk_interfaces
  message_filters
)

target_link_libraries(${PROJECT_NAME}
  ${InferenceEngine_LIBRARIES}
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

ament_package()

