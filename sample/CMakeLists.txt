# Copyright (c) 2018 Intel Corporation
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#http://www.apache.org/licenses/LICENSE-2.0
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

cmake_minimum_required(VERSION 3.5)
project(dynamic_vino_sample)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(CMAKE_BUILD_TYPE EQUAL "RELEASE")
  message(STATUS "Create Release Build.")
  set(CMAKE_CXX_FLAGS "-O2 ${CMAKE_CXX_FLAGS}")
else()
  message(STATUS "Create Debug Build.")
endif()

set(CMAKE_CXX_FLAGS "-fPIE -fPIC -D_FORTIFY_SOURCE=2 -fstack-protector -Wformat -Wformat-security -Wall ${CMAKE_CXX_FLAGS}")

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcutils)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(InferenceEngine REQUIRED)
find_package(dynamic_vino_lib REQUIRED)
find_package(object_msgs REQUIRED)
find_package(people_msgs REQUIRED)
find_package(pipeline_srv_msgs REQUIRED)
find_package(vino_param_lib REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(realsense2 REQUIRED)
find_package(rclcpp_components)

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

#remove some build warnings
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-unused-parameter")

if(OpenCV_FOUND)
    message(STATUS "OPENCV is enabled")
    message(STATUS "OpenCV_INCLUDE_DIRS=${OpenCV_INCLUDE_DIRS}")
    message(STATUS "OpenCV_LIBS=${OpenCV_LIBS}")
else()
    message(STATUS "OPENCV is disabled or not found, " ${PROJECT_NAME} " skiped")
    return()
endif()

include_directories(include
  ${yaml_cpp_vendor_DIR}/../../../opt/yaml_cpp_vendor/include)

if( BUILD_SAMPLE_NAME AND NOT ${BUILD_SAMPLE_NAME} STREQUAL ${PROJECT_NAME} )
    message(STATUS "SAMPLE ${PROJECT_NAME} SKIPPED")
    return()
endif()

file(GLOB MAIN_SRC
  ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp
)

file(GLOB MAIN_HEADERS
  ${CMAKE_CURRENT_SOURCE_DIR}/*.h
  ${CMAKE_CURRENT_SOURCE_DIR}/include/*.h
)

# Create named folders for the sources within the .vcproj
# Empty name lists them directly under the .vcproj
source_group("src" FILES ${MAIN_SRC})
source_group("include" FILES ${MAIN_HEADERS})

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${dynamic_vino_lib_INCLUDE_DIRS})
include_directories(${vino_param_lib_INCLUDE_DIRS})
include_directories(${InferenceEngine_INCLUDE_DIRS})
include_directories(${InferenceEngine_INCLUDE_DIRS}/../samples)
include_directories(${InferenceEngine_INCLUDE_DIRS}/../samples/extension)
include_directories(${InferenceEngine_INCLUDE_DIRS}/../src)
#include_directories(${InferenceEngine_INCLUDE_DIRS}/../samples/build/thirdparty/gflags/include)
#include_directories(${InferenceEngine_INCLUDE_DIRS}/../build/samples/thirdparty/gflags/include)

include_directories(${realsense2_INCLUDE_DIRS})
#include_directories (/opt/ros2_openvino/include)

# Create library file from sources.
#add_executable(${PROJECT_NAME} ${MAIN_SRC} ${MAIN_HEADERS})

if(UNIX)
    set(LIB_DL dl)
endif()

add_executable(vino_param_sample
  src/parameters.cpp
)
target_link_libraries(vino_param_sample
  dl
  )

ament_target_dependencies(vino_param_sample
  "vino_param_lib"
  "dynamic_vino_lib"
  "yaml_cpp_vendor"
)

add_executable(pipeline_with_params
  src/pipeline_with_params.cpp
)
target_link_libraries(pipeline_with_params
  dl
  )
ament_target_dependencies(pipeline_with_params
  "rclcpp"
  "rmw_implementation"
  "std_msgs"
  "object_msgs"
  "ament_index_cpp"
  "class_loader"
  "dynamic_vino_lib"
  "InferenceEngine"
  "people_msgs"
  "pipeline_srv_msgs"
  "vino_param_lib"
  "OpenCV"
  "yaml_cpp_vendor"
  "realsense2"
)

# Add Pipeline Composition version
add_library(composable_pipeline SHARED
  src/pipeline_composite.cpp)
target_link_libraries(composable_pipeline
  dl
  )
ament_target_dependencies(composable_pipeline
  "rclcpp"
  "rmw_implementation"
  "rclcpp_components"
  "std_msgs"
  "object_msgs"
  "ament_index_cpp"
  "class_loader"
  "dynamic_vino_lib"
  "InferenceEngine"
  "people_msgs"
  "pipeline_srv_msgs"
  "vino_param_lib"
  "OpenCV"
  "yaml_cpp_vendor"
  "realsense2"
)
rclcpp_components_register_nodes(composable_pipeline "ComposablePipeline")
#set(node_plugins "")
#set(node_plugins "${node_plugins}ComposablePipeline;$<TARGET_FILE:composable_pipeline>\n")


add_executable(image_object_server
  src/image_object_server.cpp
)
target_link_libraries(image_object_server
  dl
  )
ament_target_dependencies(image_object_server
  "rclcpp"
  "rmw_implementation"
  "std_msgs"
  "object_msgs"
  "ament_index_cpp"
  "class_loader"
  "dynamic_vino_lib"
  "InferenceEngine"
  "people_msgs"
  "pipeline_srv_msgs"
  "vino_param_lib"
  "OpenCV"
)

add_executable(image_people_server
  src/image_people_server.cpp
)
target_link_libraries(image_people_server
  dl
  )
ament_target_dependencies(image_people_server
  "rclcpp"
  "rmw_implementation"
  "std_msgs"
  "object_msgs"
  "ament_index_cpp"
  "class_loader"
  "dynamic_vino_lib"
  "InferenceEngine"
  "people_msgs"
  "pipeline_srv_msgs"
  "vino_param_lib"
  "OpenCV"
)

add_executable(image_object_client
  src/image_object_client.cpp
)
target_link_libraries(image_object_client
  dl
  )
ament_target_dependencies(image_object_client
  "rclcpp"
  "rmw_implementation"
  "std_msgs"
  "object_msgs"
  "ament_index_cpp"
  "class_loader"
  "dynamic_vino_lib"
  "InferenceEngine"
  "people_msgs"
  "pipeline_srv_msgs"
  "vino_param_lib"
  "OpenCV"
)

add_executable(image_people_client
  src/image_people_client.cpp
)
target_link_libraries(image_people_client
  dl
  )
ament_target_dependencies(image_people_client
  "rclcpp"
  "rmw_implementation"
  "std_msgs"
  "object_msgs"
  "ament_index_cpp"
  "class_loader"
  "dynamic_vino_lib"
  "InferenceEngine"
  "people_msgs"
  "pipeline_srv_msgs"
  "vino_param_lib"
  "OpenCV"
)

install(TARGETS vino_param_sample
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS pipeline_with_params
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS image_object_server
  RUNTIME DESTINATION bin
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS image_people_server
  RUNTIME DESTINATION bin
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS image_object_client
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS image_people_client
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS composable_pipeline
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install param files.
install(DIRECTORY
  param
  DESTINATION share/${PROJECT_NAME}/
)

# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_libraries(composable_pipeline)
ament_package()
