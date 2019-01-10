# Introduction

The contents in .yaml config file should be well structured and follow the supported rules and entity names.

# Sample
## [pipeline_people.yaml](https://github.com/intel/ros2_openvino_toolkit/blob/devel/sample/param/pipeline_people.yaml)
```bash
Pipelines:
- name: people
  inputs: [StandardCamera]
  infers:
    - name: FaceDetection
      model: /opt/intel/computer_vision_sdk/deployment_tools/intel_models/face-detection-adas-0001/FP32/face-detection-adas-0001.xml
      engine: CPU
      label: /opt/intel/computer_vision_sdk/deployment_tools/intel_models/face-detection-adas-0001/FP32/face-detection-adas-0001.labels
      batch: 1
    - name: AgeGenderRecognition
      model: /opt/intel/computer_vision_sdk/deployment_tools/intel_models/age-gender-recognition-retail-0013/FP32/age-gender-recognition-retail-0013.xml
      engine: CPU
      label: to/be/set/xxx.labels
      batch: 16
    - name: EmotionRecognition
      model: /opt/intel/computer_vision_sdk/deployment_tools/intel_models/emotions-recognition-retail-0003/FP32/emotions-recognition-retail-0003.xml
      engine: CPU
      label: /opt/intel/computer_vision_sdk/deployment_tools/intel_models/emotions-recognition-retail-0003/FP32/emotions-recognition-retail-0003.labels
      batch: 16
    - name: HeadPoseEstimation
      model: /opt/intel/computer_vision_sdk/deployment_tools/intel_models/head-pose-estimation-adas-0001/FP32/head-pose-estimation-adas-0001.xml
      engine: CPU
      label: to/be/set/xxx.labels
      batch: 16
  outputs: [ImageWindow, RosTopic, RViz]
  confidence_threshold: 0.2
  connects:
    - left: StandardCamera
      right: [FaceDetection]
    - left: FaceDetection
      right: [AgeGenderRecognition, EmotionRecognition, HeadPoseEstimation, ImageWindow, RosTopic, RViz]
    - left: AgeGenderRecognition
      right: [ImageWindow, RosTopic, RViz]
    - left: EmotionRecognition
      right: [ImageWindow, RosTopic, RViz]
    - left: HeadPoseEstimation
      right: [ImageWindow, RosTopic, RViz]

Common:
```
## Interface Description

### name
The name of this pipeline, its value can be any value other than empty.

### inputs
**Note**:The value of the input parametar can only have one.</br>
Currently, options for inputs are:

|option|Description|
|--------------------|------------------------------------------------------------------|
|StandardCamera|Any RGB camera with USB port supporting. Currently only the first USB camera if many are connected.|
|RealSenseCamera| Intel RealSense RGB-D Camera, directly calling RealSense Camera via librealsense plugin of openCV.|
|RealSenseCameraTopic| any ROS topic which is structured in image message.|
|Image| Any image file which can be parsed by openCV, such as .png, .jpeg.|
|Video| Any video file which can be parsed by openCV.|

### input_path
When input is Image or Video, need to use input_path to specify the path of the input file.

### infers
The Inference Engine is a set of C++ classes to provides an API to read the Intermediate Representation, set the input and output formats, and execute the model on devices.

#### name
The name of the inference engine. Currently, the inference feature list is supported:

|Inference|Description|
|-----------------------|------------------------------------------------------------------|
|FaceDetection|Object Detection task applied to face recognition using a sequence of neural networks.|
|EmotionRecognition| Emotion recognition based on detected face image.|
|AgeGenderRecognition| Age and gener recognition based on detected face image.|
|HeadPoseEstimation| Head pose estimation based on detected face image.|
|ObjectDetection| object detection based on SSD-based trained models.|
|VehicleDetection| Vehicle and passenger detection based on Intel models.|
|ObjectSegmentation| object detection and segmentation.|

#### model
The path of the model. The scheme below illustrates the typical workflow for deploying a trained deep learning model.
![trained deep learning model](https://github.com/intel/ros2_openvino_toolkit/blob/devel/doc/CVSDK_Flow.png "trained deep learning model")

#### engine
**Note**:Currently, only supports CPU and GPU.</br>
options for target device are:

|target device|
|-----------------------|
|CPU|
|Intel® Integrated Graphics|
|FPGA|
|Intel® Movidius™ Neural Compute Stick|

#### label
Currently, This parameter does not work.

#### batch
Enable dynamic batch size for inference engine net. 

### outputs
**Note**:The value of the output parameter can be selected one or more.</br>
Currently, options for outputs are:

|option|Description|
|--------------------|------------------------------------------------------------------|
|ImageWindow| window showing results|
|RosTopic| output the topic|
|RViz| display the result in rviz|

### confidence_threshold
Probability threshold for detections.

### connects
The topology of the pipeline, left can only have one value, right can have multiple values.
