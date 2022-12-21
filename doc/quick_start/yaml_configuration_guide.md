# Introduction

The contents in .yaml config file should be well structured and follow the supported rules and entity names.

# Sample
## [pipeline_people.yaml](../../sample/param/pipeline_people.yaml)
```bash
Pipelines:
- name: people
  inputs: [StandardCamera]
  infers: 
    - name: FaceDetection
      model: /opt/openvino_toolkit/models/intel/face-detection-adas-0001/FP16/face-detection-adas-0001.xml
      engine: CPU
      label: /opt/openvino_toolkit/models/intel/face-detection-adas-0001/FP16/face-detection-adas-0001.labels
      batch: 1
      confidence_threshold: 0.5
      enable_roi_constraint: true # set enable_roi_constraint to false if you don't want to make the inferred ROI (region of interest) constrained into the camera frame
    - name: AgeGenderRecognition
      model: /opt/openvino_toolkit/models/intel/age-gender-recognition-retail-0013/FP32/age-gender-recognition-retail-0013.xml
      engine: CPU
      label: to/be/set/xxx.labels
      batch: 16
    - name: EmotionRecognition
      model: /opt/openvino_toolkit/models/intel/emotions-recognition-retail-0003/FP32/emotions-recognition-retail-0003.xml
      engine: CPU
      label: /opt/openvino_toolkit/models/intel/emotions-recognition-retail-0003/FP32/emotions-recognition-retail-0003.labels
      batch: 16
    - name: HeadPoseEstimation
      model: /opt/openvino_toolkit/models/intel/head-pose-estimation-adas-0001/FP32/head-pose-estimation-adas-0001.xml
      engine: CPU
      label: to/be/set/xxx.labels
      batch: 16
  outputs: [ImageWindow, RosTopic, RViz]
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

### Specify pipeline name
The name value of this pipeline can be anyone other than null.

### Specify inputs
**Note:** The input parameter can only have one value.</br>
Currently, options for inputs are:

|Input Option|Description|Configuration|
|--------------------|------------------------------------------------------------------|-----------------------------------------|
|StandardCamera|Any RGB camera with USB port supporting. Currently only the first USB camera if many are connected.|```inputs: [StandardCamera]```|
|RealSenseCamera| Intel RealSense RGB-D Camera, directly calling RealSense Camera via librealsense plugin of openCV.|```inputs: [RealSenseCamera]```|
|RealSenseCameraTopic| Any ROS topic which is structured in image message.|```inputs: [RealSenseCameraTopic]```|
|Image| Any image file which can be parsed by openCV, such as .png, .jpeg.|```inputs: [Image]```|
|Video| Any video file which can be parsed by openCV.|```inputs: [Video]```|
|IpCamera| Any RTSP server which can push video stream.|```inputs: [IpCamera]```|

**Note:** Please refer to this opensource repo [RTSP_server_install_guide](https://github.com/EasyDarwin/EasyDarwin) to install RTSP server for IpCamera input.

### Specify input_path
The input_path need to be specified when input is Image, Video and Ipcamera. 

|Input Option|Configuration|
|--------------------|------------------------------------------------------------------|
|Image|```input_path: to/be/set/image_path```|
|Video|```input_path: to/be/set/video_path```|
|IpCamera|```input_path: "rtsp://localhost/test"```|

### Specify infers
The Inference Engine is a set of C++ classes to provides an API to read the Intermediate Representation, set the input and output formats, and execute the model on devices.

* #### name
The name of inference engine need to be specified here. Currently, the inference feature list is supported:

|Inference|Description|
|-----------------------|------------------------------------------------------------------|
|FaceDetection|Object Detection task applied to face recognition using a sequence of neural networks.|
|EmotionRecognition| Emotion recognition based on detected face image.|
|AgeGenderRecognition| Age and gener recognition based on detected face image.|
|HeadPoseEstimation| Head pose estimation based on detected face image.|
|ObjectDetection| object detection based on SSD-based trained models.|
|VehicleDetection| Vehicle and passenger detection based on Intel models.|
|ObjectSegmentation| object detection and segmentation.|
|ObjectSegmentationMaskrcnn| object segmentation based on Maskrcnn model.|

* #### model
The path of model need to be specified here. The scheme below illustrates the typical workflow for deploying a trained deep learning model.
![trained deep learning model](../../data/images/CVSDK_Flow.png "trained deep learning model")

* #### engine
**Note:** Currently, only CPU and GPU are supported.</br>
Target device options are:

|Target Device|
|-----------------------|
|CPU|
|Intel® Integrated Graphics|
|FPGA|
|Intel® Movidius™ Neural Compute Stick|

* #### label
Currently, this parameter does not work.

* #### batch
Enable dynamic batch size for the inference engine net. 

### Specify outputs
**Note:** The output parameter can be one or more.</br>
Currently, the output options are:

|Option|Description|Configuration|
|--------------------|-----------------------------------------------------|---------------------------------------------|
|ImageWindow| Window showing results|```outputs: [ImageWindow, RosTopic, RViz]```|
|RosTopic| Output the topic|```outputs: [ImageWindow, RosTopic, RViz]```|
|RViz| Display the result in rviz|```outputs: [ImageWindow, RosTopic, RViz]```|

### Specify confidence_threshold
Set the threshold of detection probability.

### Specify connects
The topology of a pipe can only have one value on the left and multiple values on the right. The value of the first left node should be the same as the specified **inputs**.
