# ros2_openvino_toolkit

# Table of Contents
* [➤ Introduction](#introduction)
	* [ROS2 Version Supported](#ros2-version-supported)
	* [Inference Features Supported](#inference-features-supported)
	* [Prerequisite](#prerequisite)
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
* [➤ Reference](#reference)
* [➤ More Information](#more-information)

# Introduction
## ROS2 Version Supported
* [x] ROS2 Dashing
* [x] ROS2 Galactic
* [x] ROS2 Foxy
* [x] ROS2 Humble

|Branch Name|ROS2 Version Supported|Openvino Version|Corresponding Branch Link|OS Version|
|-----------------------|-----------------------|--------------------------------|----------------------|----------------------|
|dashing|Dashing|V2022.1, V2022.2|[dashing branch](https://github.com/intel/ros2_openvino_toolkit/tree/dashing)|Ubuntu 18.04|
|ros2|Galactic, Foxy, Humble|V2022.1, V2022.2|[ros2 branch](https://github.com/intel/ros2_openvino_toolkit/tree/ros2)|Ubuntu 20.04, Ubuntu 22.04|
|foxy|Foxy|V2021.4|[foxy branch](https://github.com/intel/ros2_openvino_toolkit/tree/foxy)|Ubuntu 20.04|
|galactic-ov2021.4|Galactic|V2021.4|[galactic branch](https://github.com/intel/ros2_openvino_toolkit/tree/galactic-ov2021.4)|Ubuntu 20.04|

## Inference Features Supported
* [x] Object Detection
* [x] Face Detection
* [x] Age Gender Recognition
* [x] Emotion Recognition
* [x] Head Pose Estimation
* [x] Object Segmentation
* [x] Person Re-Identification
* [x] Vehicle Attribute Detection
* [x] Vehicle License Plate Detection

## Prerequisite
* Processor: A platform with Intel processors assembled. (see [here](https://software.intel.com/content/www/us/en/develop/articles/openvino-2020-3-lts-relnotes.html) for the full list of Intel processors supported.)
* OS: Ubuntu 20.04, Ubuntu 22.04
* ROS2: Foxy, Galactic, Humble
* OpenVINO: V2022.1, V2022.2
* Python: 3.6, 3.7, 3.8, 3.9
* [Optional] RealSense D400 Series Camera
* [Optional] Intel NCS2 Stick

## Design Architecture
From the view of hirarchical architecture design, the package is divided into different functional components, as shown in below picture. 

![OpenVINO_Architecture](./data/images/design_arch.PNG "OpenVINO RunTime Architecture")

<p>
<details>
<summary>Intel® OpenVINO™ toolkit</summary>

- **Intel® OpenVINO™ toolkit** provides a ROS-adapted runtime framework of neural network which quickly deploys applications and solutions for vision inference. By leveraging Intel® OpenVINO™ toolkit and corresponding libraries, this ROS2 runtime framework extends  workloads across Intel® hardware (including accelerators) and maximizes performance.
   - Increase deep learning workload performance up to 19x1 with computer vision accelerators from Intel.
   - Unleash convolutional neural network (CNN)-based deep learning inference using a common API.
   - Speed development using optimized OpenCV* and OpenVX* functions.
See more from [here](https://github.com/openvinotoolkit/openvino) for Intel OpenVINO™ introduction.
</details>
</p>

<p>
<details>
<summary>ROS OpenVINO Runtime Framework</summary>

- **ROS OpenVINO Runtime Framework** is the main body of this repo. it provides key logic implementation for pipeline lifecycle management, resource management and ROS system adapter, which extends Intel OpenVINO toolkit and libraries. Furthermore, this runtime framework provides ways to ease launching, configuration and data analytics and re-use.
</details>
</p>

<p>
<details>
<summary>ROS Input & Output</summary>

- **Diversal Input resources** are the data resources to be infered and analyzed with the OpenVINO framework.
- **ROS interfaces and outputs** currently include _Topic_ and _service_. Natively, RViz output and CV image window output are also supported by refactoring topic message and inferrence results.
</details>
</p>

<p>
<details>
<summary>Optimized Models</summary>

- **Optimized Models** provides by Model Optimizer component of Intel® OpenVINO™ toolkit. Imports trained models from various frameworks (Caffe*, Tensorflow*, MxNet*, ONNX*, Kaldi*) and converts them to a unified intermediate representation file. It also optimizes topologies through node merging, horizontal fusion, eliminating batch normalization, and quantization.It also supports graph freeze and graph summarize along with dynamic input freezing.
</details>
</p>

## Logic Flow
From the view of logic implementation, the package introduces the definitions of parameter manager, pipeline and pipeline manager. The below picture depicts how these entities co-work together when the corresponding program is launched.

![Logic_Flow](./data/images/impletation_logic.PNG "OpenVINO RunTime Logic Flow")

Once a corresponding program is launched with a specified .yaml config file passed in the .launch file or via commandline, _**parameter manager**_ analyzes the configurations about pipeline and the whole framework, then shares the parsed configuration information with pipeline procedure. A _**pipeline instance**_ is created by following the configuration info and is added into _**pipeline manager**_ for lifecycle control and inference action triggering.

The contents in **.yaml config file** should be well structured and follow the supported rules and entity names. Please see [yaml configuration guidance](./doc/quick_start/yaml_configuration_guide.md) for how to create or edit the config files.

<p>
<details>
<summary>Pipeline</summary>

**Pipeline** fulfills the whole data handling process: initiliazing Input Component for image data gathering and formating; building up the structured inference network and passing the formatted data through the inference network; transfering the inference results and handling output, etc.
</details>
</p>

<p>
<details>
<summary>Pipeline manager</summary>

**Pipeline manager** manages all the created pipelines according to the inference requests or external demands (say, system exception, resource limitation, or end user's operation). Because of co-working with resource management and being aware of the whole framework, it covers the ability of performance optimization by sharing system resource between pipelines and reducing the burden of data copy.
</details>
</p>

# Supported Features
## Multiple Input Components
Currently, the package support several kinds of input resources of gaining image data:

|Input Resource|Description|
|--------------------|------------------------------------------------------------------|
|StandardCamera|Any RGB camera with USB port supporting. Currently only the first USB camera if many are connected.|
|RealSenseCamera| Intel RealSense RGB-D Camera, directly calling RealSense Camera via librealsense plugin of openCV.|
|RealSenseCameraTopic| Any ROS topic which is structured in image message.|
|Image| Any image file which can be parsed by openCV, such as .png, .jpeg.|
|Video| Any video file which can be parsed by openCV.|
|IpCamera| Any RTSP server which can push video stream.|

## Inference Implementations
Currently, the inference feature list is supported:

|Inference|Description|YAML Configuration|Model Used|
|-----------------------|------------------------------------------------------------------|----------------------|----------------------|
|Face Detection| Object Detection task applied to face recognition using a sequence of neural networks.|[Face Detection YAML](./sample/param/pipeline_image.yaml)|[face-detection-adas-0001](https://github.com/openvinotoolkit/open_model_zoo/tree/master/models/intel/face-detection-adas-0001)  [age-gender-recognition-retail-0013](https://github.com/openvinotoolkit/open_model_zoo/tree/master/models/intel/age-gender-recognition-retail-0013)  [emotions-recognition-retail-0003](https://github.com/openvinotoolkit/open_model_zoo/tree/master/models/intel/emotions-recognition-retail-0003)  [head-pose-estimation-adas-0001](https://github.com/openvinotoolkit/open_model_zoo/tree/master/models/intel/head-pose-estimation-adas-0001)|
|Emotion Recognition| Emotion recognition based on detected face image.|[Emotion Detection YAML](./sample/param/pipeline_image.yaml)|[emotions-recognition-retail-0003](https://github.com/openvinotoolkit/open_model_zoo/tree/master/models/intel/emotions-recognition-retail-0003)|
|Age & Gender Recognition| Age and gender recognition based on detected face image.|[Age Gender Detection YAML](./sample/param/pipeline_image.yaml)|[age-gender-recognition-retail-0013](https://github.com/openvinotoolkit/open_model_zoo/tree/master/models/intel/age-gender-recognition-retail-0013)|
|Head Pose Estimation| Head pose estimation based on detected face image.|[Head Pose Detection YAML](./sample/param/pipeline_image.yaml)|[head-pose-estimation-adas-0001](https://github.com/openvinotoolkit/open_model_zoo/tree/master/models/intel/head-pose-estimation-adas-0001)|
|Object Detection| Object detection based on SSD-based trained models.|[Object Detection YAML](./sample/param/pipeline_object.yaml)|[mobilenet-ssd](https://github.com/openvinotoolkit/open_model_zoo/tree/master/models/public/mobilenet-ssd)|
|Vehicle and License Detection| Vehicle and license detection based on Intel models.|[Vehicle & License Detection YAML](./sample/param/pipeline_vehicle_detection.yaml)|[vehicle-license-plate-detection-barrier-0106](https://github.com/openvinotoolkit/open_model_zoo/tree/master/models/intel/vehicle-license-plate-detection-barrier-0106)|
|Object Segmentation| Object segmentation.|[Object Segmentation YAML](./sample/param/pipeline_segmentation.yaml)|[semantic-segmentation-adas-0001](https://github.com/openvinotoolkit/open_model_zoo/tree/master/models/intel/semantic-segmentation-adas-0001)|
|Person Reidentification| Person Reidentification based on object detection.|[Person Reidentification YAML](./sample/param/pipeline_person_attributes.yaml)|[person-attributes-recognition-crossroad-0230](https://github.com/openvinotoolkit/open_model_zoo/tree/master/models/intel/person-attributes-recognition-crossroad-0230)|

## ROS interfaces and outputs
### Topic

<p>
<details>
<summary>Subscribed Topic</summary>

- Image topic:
```/camera/color/image_raw```([sensor_msgs::Image](https://docs.ros.org/en/api/sensor_msgs/html/msg/Image.html))
</details>
</p>

<p>
<details>
<summary>Published Topic</summary>

- Face Detection:
```/ros2_openvino_toolkit/face_detection```([object_msgs::ObjectsInBoxes](https://github.com/intel/object_msgs/blob/master/msg/ObjectsInBoxes.msg))
- Emotion Recognition:
```/ros2_openvino_toolkit/emotion_detection```([people_msgs::EmotionsStamped](./people_msgs/msg/EmotionsStamped.msg))
- Age and Gender Recognition:
```/ros2_openvino_toolkit/age_gender_detection```([people_msgs::AgeGenderStamped](./people_msgs/msg/AgeGenderStamped.msg))
- Head Pose Estimation:
```/ros2_openvino_toolkit/head_pose_detection```([people_msgs::HeadPoseStamped](./people_msgs/msg/HeadPoseStamped.msg))
- Object Detection:
```/ros2_openvino_toolkit/detected_objects```([object_msgs::ObjectsInBoxes](https://github.com/intel/object_msgs/blob/master/msg/ObjectsInBoxes.msg))
- Object Segmentation:
```/ros2_openvino_toolkit/segmented_objects```([people_msgs::ObjectsInMasks](./people_msgs/msg/ObjectsInMasks.msg))
- Person Reidentification:
```/ros2_openvino_toolkit/reidentified_persons```([people_msgs::ReidentificationStamped](./people_msgs/msg/ReidentificationStamped.msg))
- Vehicle Detection:
```/ros2_openvino_toolkit/detected_license_plates```([people_msgs::VehicleAttribsStamped](./people_msgs/msg/VehicleAttribsStamped.msg)
- Vehicle License Detection:
```/ros2_openvino_toolkit/detected_license_plates```([people_msgs::LicensePlateStamped](./people_msgs/msg/LicensePlateStamped.msg)
- Rviz Output:
```/ros2_openvino_toolkit/image_rviz```([sensor_msgs::Image](https://docs.ros.org/en/api/sensor_msgs/html/msg/Image.html))
</details>
</p>

### Service
- Object Detection Service:
```/detect_object``` ([object_msgs::DetectObject](https://github.com/intel/object_msgs/blob/master/srv/DetectObject.srv))
- Face Detection Service:
```/detect_face``` ([object_msgs::DetectObject](https://github.com/intel/object_msgs/blob/master/srv/DetectObject.srv))
- Age & Gender Detection Service:
```/detect_age_gender``` ([people_msgs::AgeGender](./people_msgs/srv/AgeGenderSrv.srv))
- Headpose Detection Service:
```/detect_head_pose``` ([people_msgs::HeadPose](./people_msgs/srv/HeadPoseSrv.srv))
- Emotion Detection Service:
```/detect_emotion``` ([people_msgs::Emotion](./people_msgs/srv/EmotionSrv.srv))

### RViz
RViz dispaly is also supported by the composited topic of original image frame with inference result.
To show in RViz tool, add an image marker with the composited topic:
```/ros2_openvino_toolkit/image_rviz```([sensor_msgs::Image](https://docs.ros.org/en/api/sensor_msgs/html/msg/Image.html))

### Image Window
OpenCV based image window is natively supported by the package.
To enable window, Image Window output should be added into the output choices in .yaml config file. see [the config file guidance](./doc/quick_start/yaml_configuration_guide.md) for checking/adding this feature in your launching.

## Demo Result Snapshots
See below pictures for the demo result snapshots.
* Face detection input from standard camera
![face_detection_demo_image](./data/images/face_detection.png "face detection demo image")

* Object detection input from realsense camera
![object_detection_demo_realsense](./data/images/object_detection.gif "object detection demo realsense")

* Object segmentation input from video
![object_segmentation_demo_video](./data/images/object_segmentation.gif "object segmentation demo video")

* Person Reidentification input from standard camera
![person_reidentification_demo_video](./data/images/person-reidentification.gif "person reidentification demo video")

# Installation and Launching
## Deploy in local environment
* Refer to the quick start document for [getting_started_with_ros2](./doc/quick_start/getting_started_with_ros2_ov2.0.md) for detailed installation & lauching instructions.
* Refer to the quick start document for [yaml configuration guidance](./doc/quick_start/yaml_configuration_guide.md) for detailed configuration guidance.

## Deploy in docker
* Refer to the docker instruction for [docker_instructions](./docker/docker_instructions_ov2.0.md) for detailed information about building docker image and launching.
* Refer to the quick start document for [yaml configuration guidance](./doc/quick_start/yaml_configuration_guide.md) for detailed configuration guidance.

# Reference
* Open_model_zoo: Refer to the OpenVINO document for [open_model_zoo](https://github.com/openvinotoolkit/open_model_zoo/tree/master) for detailed model structure and demo samples.
* OpenVINO api 2.0: Refer to the OpenVINO document for [OpenVINO_api_2.0](https://docs.openvino.ai/latest/openvino_2_0_transition_guide.html) for latest api 2.0 transition guide.

# More Information
* ROS2 OpenVINO discription writen in Chinese: https://mp.weixin.qq.com/s/BgG3RGauv5pmHzV_hkVAdw 

###### *Any security issue should be reported using process at https://01.org/security*

