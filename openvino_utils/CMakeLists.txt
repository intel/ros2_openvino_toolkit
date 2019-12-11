cmake_minimum_required(VERSION 3.5)
project(openvino_utils)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(message_filters REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(object_msgs REQUIRED)

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

add_executable(${PROJECT_NAME}
  src/visualization.cpp
  src/viz_detection.cpp
  src/viz_factory.cpp
  src/viz_segmentation.cpp
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  message_filters
  cv_bridge
  std_msgs
  sensor_msgs
  object_msgs
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_libraries(${PROJECT_NAME})

ament_package()
