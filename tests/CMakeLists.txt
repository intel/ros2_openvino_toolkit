# Copyright (c) 2018-2022 Intel Corporation

# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at

#      http://www.apache.org/licenses/LICENSE-2.0

# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

cmake_minimum_required(VERSION 3.5)

project(openvino_test)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake)

####################################
## to use C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "-std=c++17 ${CMAKE_CXX_FLAGS}")
####################################

find_package(OpenVINO)
if(NOT OpenVINO_FOUND)
    message(FATAL_ERROR "")
endif()
set(OpenVINO_LIBRARIES openvino::runtime)

# Find OpenCV libray if exists
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
if(OpenCV_FOUND)
    add_definitions(-DUSE_OPENCV)
endif()

find_package(realsense2)
if(NOT realsense2_FOUND)
  message(FATAL_ERROR "\n\n Intel RealSense SDK 2.0 is missing, please install it from https://github.com/IntelRealSense/librealsense/releases\n\n")
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcutils)
find_package(rmw REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(object_msgs REQUIRED)
find_package(openvino_msgs REQUIRED)
find_package(class_loader REQUIRED)
find_package(openvino_param_lib REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(openvino_wrapper_lib REQUIRED)

set(CMAKE_BUILD_TYPE "Release")
if("${CMAKE_BUILD_TYPE}" STREQUAL "")
    message(STATUS "CMAKE_BUILD_TYPE not defined, 'Release' will be used")
    set(CMAKE_BUILD_TYPE "Release")
endif()

if(NOT(BIN_FOLDER))
    if("${CMAKE_SIZEOF_VOID_P}" EQUAL "8")
        set(ARCH intel64)
    else()
        set(ARCH ia32)
    endif()

    set(BIN_FOLDER ${ARCH})
endif()

if(WIN32)
    if(NOT "${CMAKE_SIZEOF_VOID_P}" EQUAL "8")
        message(FATAL_ERROR "Only 64-bit supported on Windows")
    endif()

    set_property(DIRECTORY APPEND PROPERTY COMPILE_DEFINITIONS _CRT_SECURE_NO_WARNINGS)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -D_SCL_SECURE_NO_WARNINGS -DNOMINMAX")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /EHsc") #no asynchronous structured exception handling
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} /LARGEADDRESSAWARE")
    if(ENABLE_OMP)
        find_package(OpenMP)
        if(OPENMP_FOUND)
            set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
            set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
        endif()
    endif()
else()
    if(APPLE)
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-error=unused-command-line-argument")
    elseif(UNIX)
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wuninitialized -Winit-self -Wmaybe-uninitialized")
    endif()
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

#remove some build warnings
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-unused-parameter -Wno-deprecated-declarations -Wno-unused-result ")

# Properties->C/C++->General->Additional Include Directories
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/common/format_reader
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${OpenVINO_DIRS}
    ${realsense2_INCLUDE_DIRS}
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

  macro(custom_gtest target)
    ament_add_gtest(${target} ${ARGN})
    if(TARGET ${target})
      ament_target_dependencies(${target}
        "rclcpp"
        "openvino_param_lib"
        "object_msgs"
        "openvino_msgs"
	      "OpenVINO"
	      "OpenCV"
	      "realsense2"
        "ament_index_cpp"
        "yaml_cpp_vendor"
        "class_loader"
	      "openvino_wrapper_lib")
    endif()
  endmacro()

  custom_gtest(unittest_createPipelineCheck
    "src/lib/unittest_createPipelineCheck.cpp"
    TIMEOUT 300)
  custom_gtest(unittest_faceDetection
    "src/topic/unittest_faceDetectionCheck.cpp"
    TIMEOUT 100)
  custom_gtest(unittest_objectDetection
    "src/topic/unittest_objectDetectionCheck.cpp"
    TIMEOUT 100)
  custom_gtest(unittest_imageDetection
    "src/topic/unittest_imageCheck.cpp"
    TIMEOUT 100)
  custom_gtest(unittest_segmentation
    "src/topic/unittest_segmentationCheck.cpp"
    TIMEOUT 100)
  custom_gtest(unittest_reidentification
    "src/topic/unittest_reidentification.cpp"
    TIMEOUT 100)
  custom_gtest(unittest_face_reidentification
    "src/topic/unittest_face_reidentification.cpp"
    TIMEOUT 100)
  custom_gtest(unittest_vehicleDetectionCheck
    "src/topic/unittest_vehicleDetectionCheck.cpp"
    TIMEOUT 100)
  custom_gtest(unittest_objectService
    "src/service/unittest_objectService.cpp"
    TIMEOUT 100)
  custom_gtest(unittest_peopleService
    "src/service/unittest_peopleService.cpp"
    TIMEOUT 100)
endif()

ament_package()
