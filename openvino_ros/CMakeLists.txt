cmake_minimum_required(VERSION 3.5)
project(openvino_ros)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if("${CMAKE_BUILD_TYPE}" STREQUAL "")
    message(STATUS "CMAKE_BUILD_TYPE not defined, 'Release' will be used")
    set(CMAKE_BUILD_TYPE "Release")
endif()

# Compiler Defense Flags
if(UNIX OR APPLE)
  # Linker flags.
  if(${CMAKE_CXX_COMPILER_ID} STREQUAL "GNU" OR ${CMAKE_CXX_COMPILER_ID} STREQUAL "Intel")
    # GCC specific flags. ICC is compatible with them.
    set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -z noexecstack -z relro -z now")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -z noexecstack -z relro -z now")
  elseif(${CMAKE_CXX_COMPILER_ID} STREQUAL "Clang")
    # In Clang, -z flags are not compatible, they need to be passed to linker via -Wl.
    set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -Wl,-z,noexecstack -Wl,-z,relro -Wl,-z,now")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,-z,noexecstack -Wl,-z,relro -Wl,-z,now")
  endif()
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rmw REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(class_loader REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(message_filters REQUIRED)
find_package(object_msgs REQUIRED)

#find_package(OpenCV 4 REQUIRED)
find_package(OpenCV 3 REQUIRED)
if(NOT OpenCV_FOUND)
    message(FATAL_ERROR "\n\n OpenCV4 is not found\n\n")
endif()

set(InferenceEngine_DIRS /opt/intel/openvino/deployment_tools/inference_engine)
find_package(InferenceEngine)
if(NOT InferenceEngine_FOUND)
    message(FATAL_ERROR "\n\n Intel OpenVINO toolkit is missing, please install it from https://software.intel.com/en-us/openvino-toolkit\n\n")
endif()

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${InferenceEngine_DIRS}/src
  ${InferenceEngine_INCLUDE_DIRS}
  ${object_msgs_INCLUDE_DIRS}
  ${message_filters_INCLUDE_DIRS}
)

set(CPU_EXTENSION_LIB /opt/intel/openvino/deployment_tools/inference_engine/samples/build/intel64/Release/lib/libcpu_extension.so)
add_library(cpu_extension SHARED IMPORTED)
set_target_properties(cpu_extension PROPERTIES
  IMPORTED_LOCATION ${CPU_EXTENSION_LIB})

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
  sensor_msgs
  std_msgs
  tf2_ros
  tf2
  message_filters
  object_msgs
  message_filters
  OpenCV
)

target_link_libraries(${PROJECT_NAME}
  cpu_extension
  ${InferenceEngine_LIBRARIES}
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

install(
  DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

ament_package()
