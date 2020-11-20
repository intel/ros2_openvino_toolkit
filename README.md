# ros2_openvino_toolkit - branch ov-2020.3
# IN DEVELOPMENT

This branch is now under development of supporting OpenVINO V2020.3 version. 

Current Status:

ROS2 Version:

* [x] ROS2 Dashing
* [x] ROS2 Eloquent
* [ ] ROS2 Foxy

Inference Features:

* [x] Object Detection
* [x] Face Detection
* [x] Age-Gender Recognition
* [x] Emotion Recognition
* [x] Head Pose Estimation
* [x] Object Segmentation
* [x] Person Re-Identification
* [x] Vehicle Attribute Detection
* [x] Vehicle License Plate Detection

## Introduction

The OpenVINO™ (Open visual inference and neural network optimization) toolkit provides a ROS-adaptered runtime framework of neural network which quickly deploys applications and solutions for vision inference. By leveraging Intel® OpenVINO™ toolkit and corresponding libraries, this ROS2 runtime framework extends  workloads across Intel® hardware (including accelerators) and maximizes performance.

See more from [here](https://github.com/openvinotoolkit/openvino) for Intel OpenVINO™ introduction.

## Prerequisite

* Process: A platform with Intel processors assembled. (see [here](https://software.intel.com/content/www/us/en/develop/articles/openvino-2020-3-lts-relnotes.html) for the full list of Intel processors supported.)
* OS: Ubuntu 18.04
* ROS2: Foxy Fitzroy
* OpenVINO: V2020.3, see [the release notes](https://software.intel.com/content/www/us/en/develop/articles/openvino-2020-3-lts-relnotes.html) for more info.
* [Optional] RealSense D400 Series Camera
* [Optional] Intel NCS2 Stick
## Tables of contents
* [Design Architecture and Logic Flow](https://github.com/intel/ros2_openvino_toolkit/blob/devel/doc/tables_of_contents/Design_Architecture_and_logic_flow.md)
* [Supported Features](https://github.com/intel/ros2_openvino_toolkit/blob/devel/doc/tables_of_contents/supported_features/Supported_features.md)
* Tutorials
  - [How to install ros2 openvinot toolkit based on ROS2 Dashing](https://github.com/intel/ros2_openvino_toolkit/blob/devel/doc/installation/BINARY_INSTALLATION.md)
  - [How to install ros2 openvinot toolkit based on ROS2 Foxy](https://github.com/intel/ros2_openvino_toolkit/blob/devel/doc/installation/BINARY_INSTALLATION.md)
  - [How to configure a inference pipeline?](https://github.com/intel/ros2_openvino_toolkit/blob/devel/doc/tables_of_contents/tutorials/configuration_file_customization.md)
  - [How to create multiple pipelines in a process?](https://github.com/intel/ros2_openvino_toolkit/blob/devel/doc/tables_of_contents/tutorials/Multiple_Pipelines.md)

## Installation
See [Installation Page](https://github.com/intel/ros2_openvino_toolkit/blob/devel/doc/installation/installation.md) for detailed installation instructions.

## Launching
See [Launching Page](https://github.com/intel/ros2_openvino_toolkit/blob/devel/doc/launching/launch.md) for detailed launching instructions.


# TODO Features
* Support **result filtering** for inference process, so that the inference results can be filtered to different subsidiary inference. For example, given an image, firstly we do Object Detection on it, secondly we pass cars to vehicle brand recognition and pass license plate to license number recognition.
* Design **resource manager** to better use such resources as models, engines, and other external plugins.
* Develop GUI based **configuration and management tools** (and monitoring and diagnose tools), in order to provide easy entry for end users to simplify their operation. 

# More Information
* ROS2 OpenVINO discription writen in Chinese: https://mp.weixin.qq.com/s/BgG3RGauv5pmHzV_hkVAdw 

###### *Any security issue should be reported using process at https://01.org/security*

