# Copyright (c) 2018 Intel Corporation

# Licensed under the Apache License, Version 2.0 (the "License"); # you may not use this file except in compliance with the License.  # You may obtain a copy of the License at #      http://www.apache.org/licenses/LICENSE-2.0

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
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if (CMAKE_BUILD_TYPE EQUAL "RELEASE")
  message(STATUS "Create Release Build.")
  set(CMAKE_CXX_FLAGS "-O2 ${CMAKE_CXX_FLAGS}")
else()
  message(STATUS "Create Debug Build.")
endif()

set(CMAKE_CXX_FLAGS "-fPIE -fPIC -D_FORTIFY_SOURCE=2 -fstack-protector -Wformat -Wformat-security -Wall ${CMAKE_CXX_FLAGS}")

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
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcutils)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(InferenceEngine REQUIRED)
find_package(dynamic_vino_lib REQUIRED)
find_package(object_msgs REQUIRED)
find_package(people_msgs REQUIRED)
find_package(vino_param_lib REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(librealsense2 REQUIRED)

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

set (CpuExtension_lib $ENV{CPU_EXTENSION_LIB})

add_library(cpu_extension SHARED IMPORTED)
set_target_properties(cpu_extension PROPERTIES
  IMPORTED_LOCATION $ENV{CPU_EXTENSION_LIB})

set (Gflags_lib $ENV{GFLAGS_LIB})

add_library(gflags STATIC IMPORTED)
set_target_properties(gflags PROPERTIES
  IMPORTED_LOCATION $ENV{GFLAGS_LIB})

file (GLOB MAIN_SRC
        ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp
        )

file (GLOB MAIN_HEADERS
        ${CMAKE_CURRENT_SOURCE_DIR}/*.h
        ${CMAKE_CURRENT_SOURCE_DIR}/include/*.h
        )

# Create named folders for the sources within the .vcproj
# Empty name lists them directly under the .vcproj
source_group("src" FILES ${MAIN_SRC})
source_group("include" FILES ${MAIN_HEADERS})

include_directories (${OpenCV_INCLUDE_DIRS})
include_directories (${PROJECT_SOURCE_DIR}/include)
include_directories (${dynamic_vino_lib_INCLUDE_DIRS})
include_directories (${vino_param_lib_INCLUDE_DIRS})
include_directories (${InferenceEngine_INCLUDE_DIRS})
include_directories (${InferenceEngine_INCLUDE_DIRS}/../samples)
include_directories (${InferenceEngine_INCLUDE_DIRS}/../samples/extension)
include_directories (${InferenceEngine_INCLUDE_DIRS}/../src)
include_directories (${InferenceEngine_INCLUDE_DIRS}/../samples/build/thirdparty/gflags/include)
include_directories (${InferenceEngine_INCLUDE_DIRS}/../build/samples/thirdparty/gflags/include)

include_directories (${librealsense2_INCLUDE_DIRS})
#include_directories (/opt/ros2_openvino/include)

# Create library file from sources.
#add_executable(${PROJECT_NAME} ${MAIN_SRC} ${MAIN_HEADERS})

add_executable(${PROJECT_NAME}
  main.cpp
)
target_link_libraries(${PROJECT_NAME}
  dl
  cpu_extension
  gflags)
ament_target_dependencies(${PROJECT_NAME}
  "rclcpp"
  "rmw_implementation"
  "std_msgs"
  "object_msgs"
  "ament_index_cpp"
  "class_loader"
  "dynamic_vino_lib"
  "InferenceEngine"
  "people_msgs"
  "vino_param_lib"
  "yaml_cpp_vendor"
)


add_executable(vino_param_sample
  parameters.cpp
)
target_link_libraries(vino_param_sample
  dl
  gflags)

ament_target_dependencies(vino_param_sample
  "vino_param_lib"
  "dynamic_vino_lib"
  "rviz_yaml_cpp_vendor"

)

add_executable(pipeline_with_params
  pipeline_with_params.cpp
)
target_link_libraries(pipeline_with_params
  dl
  cpu_extension
  gflags)
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
  "vino_param_lib"
  "OpenCV"
  "rviz_yaml_cpp_vendor"
  "librealsense2"
)

add_executable(object_detection_with_params
  object_detection_with_params.cpp
)
target_link_libraries(object_detection_with_params
  dl
  cpu_extension
  gflags)
ament_target_dependencies(object_detection_with_params
  "rclcpp"
  "rmw_implementation"
  "std_msgs"
  "object_msgs"
  "ament_index_cpp"
  "class_loader"
  "dynamic_vino_lib"
  "InferenceEngine"
  "people_msgs"
  "vino_param_lib"
  "OpenCV"
  "librealsense2"
) 

if(UNIX)
    target_link_libraries( ${PROJECT_NAME} ${LIB_DL} pthread ${OpenCV_LIBRARIES})
endif()
set_target_properties(${PROJECT_NAME} PROPERTIES "CMAKE_CXX_FLAGS" "${CMAKE_CXX_FLAGS} -fPIE"
        COMPILE_PDB_NAME ${PROJECT_NAME})

install(DIRECTORY param
        DESTINATION share/${PROJECT_NAME}/
)
install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS vino_param_sample
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS pipeline_with_params
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS object_detection_with_params
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
