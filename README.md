# ros2_openvino_toolkit

# Table of Contents
* [➤ Overview](#overview)
	* [ROS2 Version Supported](#ros2-version-supported)
	* [Inference Features Supported](#inference-features-supported)
* [➤ Prerequisite](#prerequisite)	
* [➤ Introduction](#introduction)
	* [Design Architecture](#design-architecture)
	* [Logic Flow](#logic-flow)
* [➤ Supported Features](#supported-features)
	* [Multiple Input Components](#multiple-input-components)
	* [Inference Implementations](#inference-implementations)
	* [ROS Interfaces and Outputs](#ros-interfaces-and-outputs)
	* [Demo Result Snapshots](#demo-result-snapshots)
* [➤ Installation & Launching](#installation-and-launching)
	* [Deploy in Local Environment](#deploy-in-local-environment)
	* [Deploy in Docker](#deploy-in-docker)
	* [Install from Debian Packages](#install-from-debian-packages)
	* [Test in a Container](#test-in-a-container)
* [➤ Reference](#reference)
* [➤ FAQ](#faq)
* [➤ Feedback](#feedback)
* [➤ More Information](#more-information)

# Overview
## ROS2 Version Supported

Pick the branch that matches your ROS 2 distribution. Each branch carries its own build and run instructions.

|Branch|ROS 2|Ubuntu|OpenVINO|Install method|
|---|---|---|---|---|
|[ros2_lyrical](https://github.com/intel/ros2_openvino_toolkit/tree/ros2_lyrical)|Lyrical|26.04|2026.4.1|Intel APT|
|[ros2_jazzy](https://github.com/intel/ros2_openvino_toolkit/tree/ros2_jazzy)|Jazzy|24.04|2025.4.0|Intel APT|
|[ros2_humble](https://github.com/intel/ros2_openvino_toolkit/tree/ros2_humble)|Humble|22.04|2025.4.0|Intel APT|

Only the active ROS 2 distributions above are maintained. Releases for end-of-life distributions (Dashing, Eloquent, Foxy, Galactic) are kept as `archive/*` tags, for example `git checkout archive/foxy-ov2021.4`.

## Inference Features Supported
* [x] Object Detection
* [x] Face Detection
* [x] Age Gender Recognition
* [x] Emotion Recognition
* [x] Head Pose Estimation
* [x] Object Segmentation (Semantic & Instance)
* [x] Person Re-Identification
* [x] Vehicle Attribute Detection
* [x] Vehicle License Plate Detection
* [x] **Intel NPU (Neural Processing Unit) Support** - Hardware acceleration for Intel Core Ultra processors

# Prerequisite

|Prerequisite|Mandatory?|Description|
|-----------------------|-----------------------|--------------------------------|
|**Processor**|Mandatory|A platform with Intel processors assembled. (Refer to [here](https://software.intel.com/content/www/us/en/develop/articles/openvino-2020-3-lts-relnotes.html) for the full list of Intel processors supported.)|
|**OS**|Mandatory|Only Ubuntu is tested. Use the Ubuntu release that matches your ROS 2 distribution. **For example: Ubuntu 22.04 for Humble, Ubuntu 24.04 for Jazzy, Ubuntu 26.04 for Lyrical.**|
|**ROS2**|Mandatory|Choose the distribution matching your OS and check out the corresponding branch from the table in section [**ROS2 Version Supported**](#ros2-version-supported).|
|**OpenVINO**|Mandatory|The version of OpenVINO toolkit is decided by the OS and ROS2 distros you use. See the table above in Section [**ROS2 Version Supported**](#ros2-version-supported).|
|**RealSense Camera**|Optional|RealSense Camera is optional, you may choose these alternatives as the input: Standard Camera, ROS Image Topic, Video/Image File or RTSP camera.|

# Introduction
## Design Architecture
<p><details><summary>Architecture Design</summary>
From the view of hierarchical architecture design, the package is divided into different functional components, as shown in the picture below. 

![OpenVINO_Architecture](./data/images/design_arch.PNG "OpenVINO RunTime Architecture")

<p>
<details>
<summary>Intel® OpenVINO™ toolkit</summary>

- **Intel® OpenVINO™ toolkit** provides a ROS-adapted runtime framework of neural network which quickly deploys applications and solutions for vision inference. By leveraging Intel® OpenVINO™ toolkit and corresponding libraries, this ROS2 runtime framework extends workloads across Intel® hardware (including accelerators) and maximizes performance.
   - Increase deep learning workload performance up to 19x1 with computer vision accelerators from Intel.
   - Unleash convolutional neural network (CNN)-based deep learning inference using a common API.
   - Speed development using optimized OpenCV* and OpenVX* functions.
See more from [here](https://github.com/openvinotoolkit/openvino) for Intel OpenVINO™ introduction.
</details>
</p>

<p>
<details>
<summary>ROS OpenVINO Runtime Framework</summary>

- **ROS OpenVINO Runtime Framework** is the main body of this repo. It provides key logic implementation for pipeline lifecycle management, resource management and ROS system adapter, which extends Intel OpenVINO toolkit and libraries. Furthermore, this runtime framework provides ways to simplify launching, configuration, data analysis and re-use.
</details>
</p>

<p>
<details>
<summary>ROS Input & Output</summary>

- **Diverse Input resources** are data resources to be inferred and analyzed with the OpenVINO framework.
- **ROS interfaces and outputs** currently include _Topic_ and _service_. Natively, RViz output and CV image window output are also supported by refactoring topic message and inference results.
</details>
</p>

<p>
<details>
<summary>Optimized Models</summary>

- **Optimized Models** provided by Model Optimizer component of Intel® OpenVINO™ toolkit. Imports trained models from various frameworks (Caffe*, TensorFlow*, MxNet*, ONNX*, Kaldi*) and converts them to a unified intermediate representation file. It also optimizes topologies through node merging, horizontal fusion, eliminating batch normalization, and quantization. It also supports graph freeze and graph summarize along with dynamic input freezing.
</details>
</p>
</details></p>

## Logic Flow
<p><details><summary> Logic Flow</summary>
From the view of logic implementation, the package introduces the definitions of parameter manager, pipeline and pipeline manager. The following picture depicts how these entities co-work together when the corresponding program is launched.

![Logic_Flow](./data/images/impletation_logic.PNG "OpenVINO RunTime Logic Flow")

Once a corresponding program is launched with a specified .yaml config file passed in the .launch file or via command line, _**parameter manager**_ analyzes the configurations about pipeline and the whole framework, then shares the parsed configuration information with pipeline procedure. A _**pipeline instance**_ is created by following the configuration info and is added into _**pipeline manager**_ for lifecycle control and inference action triggering.

The contents in **.yaml config file** should be well structured and follow the supported rules and entity names. Please see [yaml configuration guidance](./doc/quick_start/yaml_configuration_guide.md) for how to create or edit the config files.

<p>
<details>
<summary>Pipeline</summary>

**Pipeline** fulfills the whole data handling process: initializing Input Component for image data gathering and formatting; building up the structured inference network and passing the formatted data through the inference network; transferring the inference results and handling output, etc.
</details>
</p>

<p>
<details>
<summary>Pipeline manager</summary>

**Pipeline manager** manages all the created pipelines according to the inference requests or external demands (say, system exception, resource limitation, or end user's operation). Because of co-working with resource management and being aware of the whole framework, it covers the ability of performance optimization by sharing system resource between pipelines and reducing the burden of data copy.
</details>
</p>
</details></p>

# Supported Features
## Multiple Input Components
Currently, the package supports several input resources for acquiring image data. The following tables are listed:

<p>
<details>
<summary>Input Resource Table</summary>

|Input Resource|Description|
|--------------------|------------------------------------------------------------------|
|StandardCamera|Any RGB camera with USB port supporting. Currently only the first USB camera if many are connected.|
|RealSenseCamera| Intel RealSense RGB-D Camera, directly calling RealSense Camera via librealsense plugin of openCV.|
|ImageTopic| Any ROS topic which is structured in image message.|
|Image| Any image file which can be parsed by openCV, such as .png, .jpeg.|
|Video| Any video file which can be parsed by openCV.|
|IpCamera| Any RTSP server which can push video stream.|
</details>
</p>

## Inference Implementations
Currently, the corresponding relation of supported inference features, models used and yaml configurations are listed as follows:

<p>
<details>
<summary>Inference Feature Correspondence Table</summary>

|Inference|Description|YAML Configuration|Model Used|
|-----------------------|------------------------------------------------------------------|----------------------|----------------------|
|Face Detection| Object Detection task applied to face recognition using a sequence of neural networks.|[pipeline_image.yaml](./sample/param/pipeline_image.yaml)<br>[pipeline_image_video.yaml](./sample/param/pipeline_image_video.yaml)<br>[pipeline_people.yaml](./sample/param/pipeline_people.yaml)<br>[pipeline_people_ip.yaml](./sample/param/pipeline_people_ip.yaml)|[face-detection-adas-0001](https://github.com/openvinotoolkit/open_model_zoo/tree/releases/2022/3/models/intel/face-detection-adas-0001)<br>[age-gender-recognition-retail-0013](https://github.com/openvinotoolkit/open_model_zoo/tree/releases/2022/3/models/intel/age-gender-recognition-retail-0013)<br>[emotions-recognition-retail-0003](https://github.com/openvinotoolkit/open_model_zoo/tree/releases/2022/3/models/intel/emotions-recognition-retail-0003)<br>[head-pose-estimation-adas-0001](https://github.com/openvinotoolkit/open_model_zoo/tree/releases/2022/3/models/intel/head-pose-estimation-adas-0001)|
|Emotion Recognition| Emotion recognition based on detected face image.|[pipeline_image.yaml](./sample/param/pipeline_image.yaml)<br>[pipeline_image_video.yaml](./sample/param/pipeline_image_video.yaml)<br>[pipeline_people.yaml](./sample/param/pipeline_people.yaml)<br>[pipeline_people_ip.yaml](./sample/param/pipeline_people_ip.yaml)|[emotions-recognition-retail-0003](https://github.com/openvinotoolkit/open_model_zoo/tree/releases/2022/3/models/intel/emotions-recognition-retail-0003)|
|Age & Gender Recognition| Age and gender recognition based on detected face image.|[pipeline_image.yaml](./sample/param/pipeline_image.yaml)<br>[pipeline_image_video.yaml](./sample/param/pipeline_image_video.yaml)<br>[pipeline_people.yaml](./sample/param/pipeline_people.yaml)<br>[pipeline_people_ip.yaml](./sample/param/pipeline_people_ip.yaml)|[age-gender-recognition-retail-0013](https://github.com/openvinotoolkit/open_model_zoo/tree/releases/2022/3/models/intel/age-gender-recognition-retail-0013)|
|Head Pose Estimation| Head pose estimation based on detected face image.|[pipeline_image.yaml](./sample/param/pipeline_image.yaml)<br>[pipeline_image_video.yaml](./sample/param/pipeline_image_video.yaml)<br>[pipeline_people.yaml](./sample/param/pipeline_people.yaml)<br>[pipeline_people_ip.yaml](./sample/param/pipeline_people_ip.yaml)|[head-pose-estimation-adas-0001](https://github.com/openvinotoolkit/open_model_zoo/tree/releases/2022/3/models/intel/head-pose-estimation-adas-0001)|
|Object Detection| Object detection based on SSD-based trained models.|[pipeline_object.yaml](./sample/param/pipeline_object.yaml)<br>[pipeline_object_topic.yaml](./sample/param/pipeline_object_topic.yaml)|[mobilenet-ssd](https://github.com/openvinotoolkit/open_model_zoo/tree/releases/2022/3/models/public/mobilenet-ssd)<br>[yolov5](https://github.com/openvinotoolkit/openvino_notebooks/tree/main/notebooks/111-yolov5-quantization-migration)<br>[yolov7](https://github.com/openvinotoolkit/openvino_notebooks/tree/main/notebooks/226-yolov7-optimization)<br>[yolov8](https://github.com/openvinotoolkit/openvino_notebooks/tree/main/notebooks/230-yolov8-optimization)<br>[ssdlite_mobilenet_v2](https://github.com/openvinotoolkit/open_model_zoo/tree/2024.6.0/models/public/ssdlite_mobilenet_v2)|
|Vehicle and License Detection| Vehicle and license detection based on Intel models.|[pipeline_vehicle_detection.yaml](./sample/param/pipeline_vehicle_detection.yaml)|[vehicle-license-plate-detection-barrier-0106](https://github.com/openvinotoolkit/open_model_zoo/tree/releases/2022/3/models/intel/vehicle-license-plate-detection-barrier-0106)<br>[vehicle-attributes-recognition-barrier-0039](https://github.com/openvinotoolkit/open_model_zoo/tree/releases/2022/3/models/intel/vehicle-attributes-recognition-barrier-0039)<br>[license-plate-recognition-barrier-0001](https://github.com/openvinotoolkit/open_model_zoo/tree/releases/2022/3/models/intel/license-plate-recognition-barrier-0001)|
|Object Segmentation - Semantic| semantic segmentation, assign a class label to each pixel in an image. |[pipeline_segmentation.yaml](./sample/param/pipeline_segmentation.yaml)<br>[pipeline_segmentation_image.yaml](./sample/param/pipeline_segmentation_image.yaml)<br>[pipeline_video.yaml](./sample/param/pipeline_video.yaml)|[semantic-segmentation-adas-0001](https://github.com/openvinotoolkit/open_model_zoo/tree/releases/2022/3/models/intel/semantic-segmentation-adas-0001)<br>[deeplabv3](https://github.com/openvinotoolkit/open_model_zoo/tree/releases/2022/3/models/public/deeplabv3)|
| Object Segmentation - Instance | Instance Segmentation, combination of semantic segmentation & object detection. | [pipeline_segmentation_instance.yaml](./sample/param/pipeline_segmentation_instance.yaml) | [yolov8-seg](https://github.com/openvinotoolkit/openvino_notebooks/tree/main/notebooks/230-yolov8-optimization)<br>[mask_rcnn_inception_v2_coco_2018_01_28](https://github.com/openvinotoolkit/open_model_zoo/tree/releases/2022/3/models/public/mask_rcnn_inception_resnet_v2_atrous_coco)|
|Person Attributes| Person attributes based on object detection.|[pipeline_person_attributes.yaml](./sample/param/pipeline_person_attributes.yaml)|[person-attributes-recognition-crossroad-0230](https://github.com/openvinotoolkit/open_model_zoo/tree/releases/2022/3/models/intel/person-attributes-recognition-crossroad-0230)<br>[person-detection-retail-0013](https://github.com/openvinotoolkit/open_model_zoo/tree/releases/2022/3/models/intel/person-detection-retail-0013)|
|Person Reidentification|Person reidentification based on object detection.|[pipeline_reidentification.yaml](./sample/param/pipeline_reidentification.yaml)|[person-detection-retail-0013](https://github.com/openvinotoolkit/open_model_zoo/tree/releases/2022/3/models/intel/person-detection-retail-0013)<br>[person-reidentification-retail-0277](https://github.com/openvinotoolkit/open_model_zoo/tree/releases/2022/3/models/intel/person-reidentification-retail-0277)|
|Object Segmentation Maskrcnn| Object segmentation and detection based on maskrcnn model.[_Deprecated, it is recommended to use `object segmentation - instance` for first try._]|[pipeline_segmentation_maskrcnn.yaml](./sample/param/pipeline_segmentation_maskrcnn.yaml)|[mask_rcnn_inception_v2_coco_2018_01_28](https://github.com/openvinotoolkit/open_model_zoo/tree/releases/2022/3/models/public/mask_rcnn_inception_resnet_v2_atrous_coco)|
</details>
</p>

## ROS interfaces and outputs
The inference results can be output in several types. One or more types can be enabled for any inference pipeline.
### Topic
Specific topic(s) can be generated and published according to the given inference functionalities.

<p>
<details>
<summary>Published Topic Correspondence Table</summary>

|Inference|Published Topic|
|---|---|
|People Detection|```/ros2_openvino_toolkit/face_detection```([object_msgs:msg:ObjectsInBoxes](./object_msgs/msg/ObjectsInBoxes.msg))|
|Emotion Recognition|```/ros2_openvino_toolkit/emotions_recognition```([object_msgs:msg:EmotionsStamped](./object_msgs/msg/EmotionsStamped.msg))|
|Age and Gender Recognition|```/ros2_openvino_toolkit/age_genders_Recognition```([object_msgs:msg:AgeGenderStamped](./object_msgs/msg/AgeGenderStamped.msg))|
|Head Pose Estimation|```/ros2_openvino_toolkit/headposes_estimation```([object_msgs:msg:HeadPoseStamped](./object_msgs/msg/HeadPoseStamped.msg))|
|Object Detection|```/ros2_openvino_toolkit/detected_objects```([object_msgs::msg::ObjectsInBoxes](./object_msgs/msg/ObjectsInBoxes.msg))|
|Object Segmentation|```/ros2_openvino_toolkit/segmented_objects```([object_msgs::msg::ObjectsInMasks](./object_msgs/msg/ObjectsInMasks.msg))|
|Object Segmentation Maskrcnn|```/ros2_openvino_toolkit/segmented_objects```([object_msgs::msg::ObjectsInMasks](./object_msgs/msg/ObjectsInMasks.msg))|
|Person Reidentification|```/ros2_openvino_toolkit/reidentified_persons```([object_msgs::msg::ReidentificationStamped](./object_msgs/msg/ReidentificationStamped.msg))|
|Vehicle Detection|```/ros2_openvino_toolkit/detected_vehicles_attribs```([object_msgs::msg::VehicleAttribsStamped](./object_msgs/msg/VehicleAttribsStamped.msg))|
|Vehicle License Detection|```/ros2_openvino_toolkit/detected_license_plates```([object_msgs::msg::LicensePlateStamped](./object_msgs/msg/LicensePlateStamped.msg))|
</details>
</p>

### Service
Several ROS2 Services are created, expecting to be used in client/server mode, especially when synchronously getting inference results for a given image frame or when managing inference pipeline's lifecycle.</br>

<p>
<details>
<summary>Service Correspondence Table</summary>

|Inference|Service|
|---|---|
|Object Detection Service|```/detect_object```([object_msgs::srv::DetectObject](./object_msgs/srv/DetectObject.srv))|
|Face Detection Service|```/detect_face```([object_msgs::srv::DetectObject](./object_msgs/srv/DetectObject.srv))|
|Age Gender Detection Service|```/detect_age_gender```([object_msgs::srv::AgeGender](./object_msgs/srv/AgeGenderSrv.srv))|
|Headpose Detection Service|```/detect_head_pose```([object_msgs::srv::HeadPose](./object_msgs/srv/HeadPoseSrv.srv))|
|Emotion Detection Service|```/detect_emotion```([object_msgs::srv::Emotion](./object_msgs/srv/EmotionSrv.srv))|
</details>
</p>

### RViz
RViz display is also supported by the composited topic of original image frame with inference result.
To show in RViz tool, add an image marker with the composited topic:
```/ros2_openvino_toolkit/image_rviz```([sensor_msgs::Image](https://docs.ros.org/en/api/sensor_msgs/html/msg/Image.html))

### Image Window
OpenCV based image window is natively supported by the package.
To enable window, Image Window output should be added into the output choices in .yaml config file. Refer to [the config file guidance](./doc/quick_start/yaml_configuration_guide.md) for more information about checking/adding this feature in your launching.

## Demo Result Snapshots
<p><details><summary>Demo Snapshots</summary>
For the snapshot of demo results, refer to the following picture.

* Face detection input from standard camera
![face_detection_demo_image](./data/images/face_detection.png "face detection demo image")

* Object detection input from realsense camera
![object_detection_demo_realsense](./data/images/object_detection.gif "object detection demo realsense")

* Object segmentation input from video
![object_segmentation_demo_video](./data/images/object_segmentation.gif "object segmentation demo video")

* Person reidentification input from standard camera
![person_reidentification_demo_video](./data/images/person-reidentification.gif "person reidentification demo video")
</details></p>

# Installation and Launching

## Quick Start
Commands below are for the `ros2_lyrical` branch (Ubuntu 26.04). The full walkthrough, including prerequisites and troubleshooting, is in the [getting started guide](./doc/quick_start/getting_started_with_ros2_ov2.0.md).

```bash
# 1. Get the sources and install OpenVINO from the Intel APT repository
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone https://github.com/intel/ros2_openvino_toolkit -b ros2_lyrical
sudo ./ros2_openvino_toolkit/script/packaging/setup_openvino_apt.sh
sudo apt install openvino-libraries-dev-2026.4.1

# 2. Build
cd ~/ros2_ws
source /opt/ros/lyrical/setup.bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash

# 3. Download the models and run a demo on a sample image
ros2 run openvino_node download_models.sh
ros2 launch openvino_node pipeline_image.launch.py
```

### Choose the inference device
Every sample launch file accepts a `device` argument that overrides the `engine:` value of all models in the YAML file. Without it, the YAML values are used.

```bash
ros2 launch openvino_node pipeline_image.launch.py device:=GPU   # CPU, GPU, NPU, AUTO, ...
```

The device each model actually runs on is printed at startup (`Engine created: requested device=GPU, executing on=GPU.0`) and drawn on the output image next to the FPS (`FPS` is only shown for camera and video inputs).

### Where models and data are located
|What|Location|
|---|---|
|Models|`$OPENVINO_TOOLKIT_DIR/models`, else `~/.local/share/openvino_toolkit/models`, else `/opt/openvino_toolkit/models`|
|Labels and sample images|Installed with the package (`share/openvino_node/data`)|

The YAML files use the placeholders `<OPENVINO_NODE_SHARE>` (the models location above) and `<OPENVINO_DATA>` (labels and images). The launch files resolve them at start-up, so no path needs to be edited.

## Deploy in Local Environment
* Refer to the quick start document for [getting_started_with_ros2](./doc/quick_start/getting_started_with_ros2_ov2.0.md) for detailed installation & launching instructions.
* Refer to the quick start document for [yaml configuration guidance](./doc/quick_start/yaml_configuration_guide.md) for detailed configuration guidance.

## Deploy in Docker
* Refer to the [Lyrical docker instructions](./docker/ros2_ov2026/ros2_lyrical/docker_instructions.md) for building the image and launching the demos. For Jazzy and Humble, see the `ros2_jazzy` and `ros2_humble` branches.
* Refer to the quick start document for [yaml configuration guidance](./doc/quick_start/yaml_configuration_guide.md) for detailed configuration guidance.

## Install from Debian Packages
The toolkit can be packaged for ROS 2 Lyrical on Ubuntu 26.04 (amd64). The packages are built in a container, so Docker is the only build requirement.

|Package|Content|
|-----------------------|--------------------------------|
|`ros-lyrical-openvino-node`|Pipeline and service nodes, launch files, YAML configurations, labels, sample images and `download_models.sh`. Depends on all packages below.|
|`ros-lyrical-openvino-wrapper-lib`|OpenVINO pipeline library and headers|
|`ros-lyrical-openvino-param-lib`|YAML parameter library and headers|
|`ros-lyrical-openvino-msgs`|Pipeline messages and services|
|`ros-lyrical-object-msgs`|Object messages and services, built from the [object_msgs](./object_msgs) copy in this repository|

* Build the packages. They are written to `dist/`.
```bash
make package
```
* Install the packages on a system with ROS 2 Lyrical. OpenVINO 2026.4.1 is installed as a dependency from the Intel APT repository, which the first command enables.
```bash
sudo ./script/packaging/setup_openvino_apt.sh
sudo apt install ./dist/*.deb
source /opt/ros/lyrical/setup.bash
```
* Each `lyrical/<version>` release on GitHub also provides the packages as `ros-lyrical-openvino-toolkit_<version>_amd64.tar.gz`. Extract the archive and install the `.deb` files in the same way.
* The OpenVINO models are not part of the packages. Download them with `ros2 run openvino_node download_models.sh`.
* `make test` builds the workspace, downloads the models to `.models/` and runs the unit tests in a container. Its colcon output goes to `.colcon-docker/`, separate from the `build/`, `install/` and `log/` of a local `colcon build`. Run `make help` for the other targets, for example `make test-package`, `make build` and `make lint`.

## Test in a Container
`make test-functional` and `make shell` run the Debian packages in a `ros:lyrical-ros-base` container, so no local ROS 2 installation is needed. Both install the packages from `dist/` (run `make package` first) and use the model cache `.models/` of `make test`.

|Target|Purpose|
|-----------------------|--------------------------------|
|`make test-functional`|Runs the CI pipelines (`sample/launch/*_ci_test.py`) of the downloadable models on the sample images. A pipeline passes when it is created, reports no error and publishes on its topics. The OpenVINO device is `CPU`; select another one with, for example, `make test-functional DEVICE=GPU`.|
|`make shell`|Opens a shell for manual tests. The X11 display, the GPU (`/dev/dri`) and the NPU (`/dev/accel`) of the host are passed to the container, and `intel-opencl-icd` and `rqt_image_view` are installed in addition (variable `SHELL_PACKAGES`).|

```bash
make shell
# inside the container
ros2 run openvino_node download_models.sh
ros2 launch openvino_node pipeline_image.launch.py device:=CPU
```
* `GPU` needs a driver in the container that supports the GPU of the host. The `intel-opencl-icd` package of Ubuntu 26.04 is installed for that purpose.
* `NPU` needs the [Intel NPU driver](https://github.com/intel/linux-npu-driver) inside the container. It is not installed by these targets.

# Reference
* Open_model_zoo: Refer to the OpenVINO document for [open_model_zoo](https://github.com/openvinotoolkit/open_model_zoo/tree/releases/2022/3) for detailed model structure and demo samples.
* OpenVINO api 2.0: Refer to the OpenVINO document for [OpenVINO_api_2.0](https://docs.openvino.ai/latest/openvino_2_0_transition_guide.html) for latest api 2.0 transition guide.

# FAQ
* How to get the IR file for [yolov5](./doc/quick_start/tutorial_for_yolov5_converted.md) | [yolov7](./doc/quick_start/tutorial_for_yolov7_converted.md) | [yolov8](./doc/quick_start/tutorial_for_yolov8_converted.md) ?
* [How to build OpenVINO by source?](https://github.com/openvinotoolkit/openvino/wiki#how-to-build)
* [How to build RealSense by source?](https://github.com/IntelRealSense/librealsense/blob/master/doc/installation.md)
* [What is the basic command of Docker CLI?](https://docs.docker.com/engine/reference/commandline/docker/)
* [What is the canonical C++ API for interacting with ROS?](https://docs.ros2.org/latest/api/rclcpp/)
<p><details><summary> How to change logging level?</summary>
  This project provides two logging levels: *DEBUG* & *INFO*.<br>
  You may follow the steps to change logging level:<br>

  - Update ./openvino_wrapper_lib/CMakeLists.txt by uncommenting (for DEBUG level) or commenting (for INFO level) this line:
    ```code
    #add_definitions(-DLOG_LEVEL_DEBUG)
    ```
  - Rebuild project<br>
    Refer to the corresponding quick-start documents to rebuild this project. e.g.:<br>
    ```code
    source /opt/ros/<ros-distro>/setup.bash
    colcon build --symlink-install
    ```
  - Launch OpenVINO Node<br>
    You will see the logging is changed.
</details></p>

# Feedback
* Report questions, issues and suggestions, using: [issue](https://github.com/intel/ros2_openvino_toolkit/issues).

# More Information
* ROS2 OpenVINO description written in Chinese: https://mp.weixin.qq.com/s/BgG3RGauv5pmHzV_hkVAdw 

###### *Any security issue should be reported using process at https://01.org/security*

