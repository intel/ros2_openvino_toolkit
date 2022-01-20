# ros2_openvino_toolkit

ROS2 Version supported:

* [x] ROS2 Dashing
* [x] ROS2 Eloquent
* [x] ROS2 Foxy
* [x] ROS2 Galactic

Inference Features supported:

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

* Processor: A platform with Intel processors assembled. (see [here](https://software.intel.com/content/www/us/en/develop/articles/openvino-2021-4-lts-relnotes.html) for the full list of Intel processors supported.)
* OS: Ubuntu 20.04
* ROS2: Galactic Geochelone
* OpenVINO: V2021.4, see [the release notes](https://software.intel.com/content/www/us/en/develop/articles/openvino-relnotes.html)  for more info.
* [Optional] RealSense D400 Series Camera
* [Optional] Intel NCS2 Stick
## Tables of contents
* [Design Architecture and Logic Flow](./doc/tables_of_contents/Design_Architecture_and_logic_flow.md)
* [Supported Features](./doc/tables_of_contents/supported_features/Supported_features.md)
* Tutorials
  - [How to configure a inference pipeline?](./doc/tables_of_contents/tutorials/configuration_file_customization.md)
  - [How to create multiple pipelines in a process?](./doc/tables_of_contents/tutorials/Multiple_Pipelines.md)

## Installation & Launching
See Getting Start Pages for [ROS2 Dashing](./doc/getting_started_with_Dashing.md) or [ROS2 Foxy](./doc/getting_started_with_Foxy_Ubuntu20.04.md) or [ROS2 Galactic](./doc/getting_started_with_Galactic_Ubuntu20.04.md) for detailed installation & lauching instructions.

# More Information
* ROS2 OpenVINO discription writen in Chinese: https://mp.weixin.qq.com/s/BgG3RGauv5pmHzV_hkVAdw

###### *Any security issue should be reported using process at https://01.org/security*
