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

project(openvino_wrapper_lib)

####################################
## to use C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "-std=c++17 ${CMAKE_CXX_FLAGS}")
####################################

####################################
## by default, new OpenVINO API (ov::core) is used.
####################################

####################################
## to get verbose log,
## then, uncomment below line
add_definitions(-DLOG_LEVEL_DEBUG)
####################################

# environment variable OpenVINO_DIR can be use instead of relaive path to specify location of configuration file

find_package(OpenVINO REQUIRED)
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

find_package(realsense2 QUIET)
if(NOT (realsense2_FOUND))
  message(STATUS "\n\n Intel RealSense SDK 2.0 is missing, some features depending on it won't work. \
    please install it from https://github.com/IntelRealSense/librealsense/releases\n\n")
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
find_package(cv_bridge REQUIRED)
find_package(openvino_param_lib REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)

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

if(NOT (IE_MAIN_SOURCE_DIR))
    set(NEED_EXTENSIONS TRUE)
    if(WIN32)
        set(IE_MAIN_SOURCE_DIR ${CMAKE_SOURCE_DIR}/../bin/)
    else()
        set(IE_MAIN_SOURCE_DIR ${CMAKE_CURRENT_BINARY_DIR})
    endif()
endif()

if(NOT(UNIX))
    set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${IE_MAIN_SOURCE_DIR}/${BIN_FOLDER})
    set(CMAKE_LIBRARY_PATH ${IE_MAIN_SOURCE_DIR}/${BIN_FOLDER})
    set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${IE_MAIN_SOURCE_DIR}/${BIN_FOLDER})
    set(CMAKE_COMPILE_PDB_OUTPUT_DIRECTORY ${IE_MAIN_SOURCE_DIR}/${BIN_FOLDER})
    set(CMAKE_PDB_OUTPUT_DIRECTORY ${IE_MAIN_SOURCE_DIR}/${BIN_FOLDER})
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${IE_MAIN_SOURCE_DIR}/${BIN_FOLDER})
    set(LIBRARY_OUTPUT_DIRECTORY ${IE_MAIN_SOURCE_DIR}/${BIN_FOLDER})
    set(LIBRARY_OUTPUT_PATH ${LIBRARY_OUTPUT_DIRECTORY}) # compatibility issue: linux uses LIBRARY_OUTPUT_PATH, windows uses LIBRARY_OUTPUT_DIRECTORY
else()
    set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${IE_MAIN_SOURCE_DIR}/${BIN_FOLDER}/${CMAKE_BUILD_TYPE}/lib)
    set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${IE_MAIN_SOURCE_DIR}/${BIN_FOLDER}/${CMAKE_BUILD_TYPE}/lib)
    set(CMAKE_COMPILE_PDB_OUTPUT_DIRECTORY ${IE_MAIN_SOURCE_DIR}/${BIN_FOLDER}/${CMAKE_BUILD_TYPE})
    set(CMAKE_PDB_OUTPUT_DIRECTORY ${IE_MAIN_SOURCE_DIR}/${BIN_FOLDER}/${CMAKE_BUILD_TYPE})
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${IE_MAIN_SOURCE_DIR}/${BIN_FOLDER}/${CMAKE_BUILD_TYPE})
    set(LIBRARY_OUTPUT_DIRECTORY ${IE_MAIN_SOURCE_DIR}/${BIN_FOLDER}/${CMAKE_BUILD_TYPE}/lib)
    set(LIBRARY_OUTPUT_PATH ${LIBRARY_OUTPUT_DIRECTORY}/lib)
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

#The below command is commented in order to ignore "colcon test" errors.
#This maybe impacts CPU's inference tuning, although we tested on Core i7-6700 and got the similar performance.
#If you concern the inference performence, you can try to get the resource from the follow url and enable the below line.
#https://github.com/openvinotoolkit/openvino/tree/2018/inference-engine/src/extension/cmake
#include(feature_defs OPTIONAL)

# Properties->C/C++->General->Additional Include Directories
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${OpenVINO_INCLUDE_DIRS}
)

if(UNIX)
    set(LIB_DL dl)
endif()

set(DEPENDENCIES ${realsense2_LIBRARY} ${OpenCV_LIBS} openvino::runtime)

add_library(${PROJECT_NAME} SHARED
        src/services/pipeline_processing_server.cpp
        src/services/frame_processing_server.cpp
        src/pipeline.cpp
        src/pipeline_params.cpp
        src/pipeline_manager.cpp
        src/engines/engine.cpp
        src/engines/engine_manager.cpp
        src/inferences/base_filter.cpp
        src/inferences/base_inference.cpp
        src/inferences/base_reidentification.cpp
        src/inferences/emotions_detection.cpp
        src/inferences/age_gender_detection.cpp
        src/inferences/face_detection.cpp
        src/inferences/object_detection.cpp
        src/inferences/head_pose_detection.cpp
        src/inferences/object_segmentation.cpp
        src/inferences/object_segmentation_maskrcnn.cpp
        src/inferences/person_reidentification.cpp
        src/inferences/person_attribs_detection.cpp
        #src/inferences/landmarks_detection.cpp
        #src/inferences/face_reidentification.cpp
        src/inferences/vehicle_attribs_detection.cpp
        src/inferences/license_plate_detection.cpp
        src/inferences/openpose_decoder.cpp
        src/inferences/human_pose_estimation.cpp
        src/inputs/realsense_camera.cpp
        src/inputs/image_topic.cpp
        src/inputs/standard_camera.cpp
        src/inputs/ip_camera.cpp
        src/inputs/video_input.cpp
        src/inputs/image_input.cpp
        src/models/base_model.cpp
        src/models/attributes/ssd_model_attr.cpp
        src/models/emotion_detection_model.cpp
        src/models/age_gender_detection_model.cpp
        src/models/face_detection_model.cpp
        src/models/head_pose_detection_model.cpp
        src/models/object_segmentation_model.cpp
        src/models/object_segmentation_maskrcnn_model.cpp
        src/models/person_reidentification_model.cpp
        src/models/person_attribs_detection_model.cpp
        #src/models/landmarks_detection_model.cpp
        #src/models/face_reidentification_model.cpp
        src/models/vehicle_attribs_detection_model.cpp
        src/models/license_plate_detection_model.cpp
        src/models/object_detection_ssd_model.cpp
        src/models/object_detection_yolov5_model.cpp
        src/models/human_pose_estimation_model.cpp
        src/outputs/image_window_output.cpp
        src/outputs/ros_topic_output.cpp
        src/outputs/rviz_output.cpp
        src/outputs/base_output.cpp
        src/outputs/ros_service_output.cpp
)

target_link_libraries(${PROJECT_NAME} ${DEPENDENCIES})

ament_target_dependencies(${PROJECT_NAME}
  "rclcpp"
  "rmw_implementation"
  "std_msgs"
  "sensor_msgs"
  "object_msgs"
  "openvino_msgs"
  "ament_index_cpp"
  "class_loader"
  "realsense2"
  "cv_bridge"
  "openvino_param_lib"
  "yaml_cpp_vendor"
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

install(DIRECTORY include/
        DESTINATION include/
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_package()
