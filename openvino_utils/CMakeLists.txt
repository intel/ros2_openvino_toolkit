cmake_minimum_required(VERSION 3.5)
project(openvino_utils)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

add_library(${PROJECT_NAME} SHARED
  src/visualization.cpp
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_components
)

rclcpp_components_register_nodes(${PROJECT_NAME} "openvino::OpenVINOFactory")
set(node_plugins "")
set(node_plugins "${node_plugins}openvino::OpenVINOFactory;$<TARGET_FILE:${PROJECT_NAME}>\n")

file(GENERATE
  OUTPUT
  "${CMAKE_CURRENT_BINARY_DIR}/test_ament_index/$<CONFIG>/share/ament_index/resource_index/node_plugin/${PROJECT_NAME}"
  CONTENT "${node_plugins}")

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_libraries(${PROJECT_NAME})

ament_package()