# ros2_openvino_toolkit

## 1. Introduction
The [OpenVINO™](https://software.intel.com/en-us/openvino-toolkit) toolkit quickly deploys applications and solutions that emulate human vision. Based on Convolutional Neural Networks (CNN), the Toolkit extends computer vision (CV) workloads across Intel® hardware, maximizing performance.

This project is a ROS2 wrapper for CV API of [OpenVINO™](https://software.intel.com/en-us/openvino-toolkit), providing the following features:
* Support CPU and GPU platforms
* Support standard USB camera and Intel® RealSense™ camera
* Support Video or Image file as detection source
* Face detection
* Emotion recognition
* Age and gender recognition
* Head pose recognition
* Demo application to show above detection and recognitions

## 2. Prerequisite
- An x86_64 computer running Ubuntu 16.04. Below processors are supported:
	* 6th-8th Generation Intel® Core™
	* Intel® Xeon® v5 family
	* Intel®  Xeon® v6 family
- ROS2 [Bouncy](https://github.com/ros2/ros2/wiki)
- [OpenVINO™ Toolkit](https://software.intel.com/en-us/openvino-toolkit)
- RGB Camera, e.g. RealSense D400 Series or standard USB camera or Video/Image File
- Graphics are required only if you use a GPU. The official system requirements for GPU are:
  * 6th to 8th generation Intel® Core™ processors with Iris® Pro graphics and Intel® HD Graphics
  * 6th to 8th generation Intel® Xeon® processors with Iris Pro graphics and Intel HD Graphics (excluding the e5 product family, which does not have graphics)
  * Intel® Pentium® processors N4200/5, N3350/5, N3450/5 with Intel HD Graphics

  Use one of the following methods to determine the GPU on your hardware:
  1. [lspci] command: GPU info may lie in the [VGA compatible controller] line.
  2. Ubuntu system: Menu [System Settings] --> [Details] may help you find the graphics information.
  3. Openvino: Download the install package, install_GUI.sh inside will check the GPU information before installation.

## 3. Environment Setup
- Install ROS2 [Bouncy](https://github.com/ros2/ros2/wiki) ([guide](https://github.com/ros2/ros2/wiki/Linux-Development-Setup))
- Install Intel® RealSense™ SDK 2.0 [(tag v2.14.1)](https://github.com/IntelRealSense/librealsense/tree/v2.14.1)
	* [Install from source code](https://github.com/IntelRealSense/librealsense/blob/v2.14.1/doc/installation.md)(Recommended)
	* [Install from package](https://github.com/IntelRealSense/librealsense/blob/v2.14.1/doc/distribution_linux.md)
- Install [OpenVINO™ Toolkit](https://software.intel.com/en-us/openvino-toolkit) ([guide](https://software.intel.com/en-us/articles/OpenVINO-Install-Linux))

	**Note**: Please use  *root privileges* to run the installer when installing the core components.
- Install OpenCL Driver for GPU
```bash
cd /opt/intel/computer_vision_sdk/install_dependencies
sudo ./install_NEO_OCL_driver.sh
```
- Other Dependencies
```bash
# numpy
pip3 install numpy
# libboost
sudo apt-get install -y --no-install-recommends libboost-all-dev
cd /usr/lib/x86_64-linux-gnu
sudo ln -s libboost_python-py35.so libboost_python3.so
```

## 4. Building and Installation
- Build sample code under openvino toolkit
```bash
# root is required instead of sudo
source /opt/intel/computer_vision_sdk/bin/setupvars.sh
cd /opt/intel/computer_vision_sdk/deployment_tools/inference_engine/samples/
mdkir build
cd build
cmake ..
make
```

- Install ROS2_OpenVINO packages
```bash
mkdir -p ~/ros2_overlay_ws/src
cd ~/ros2_overlay_ws/src
git clone https://github.com/intel/ros2_openvino_toolkit
git clone https://github.com/intel/ros2_object_msgs
git clone https://github.com/ros-perception/vision_opencv -b ros2
```

- Build package
```
source ~/ros2_ws/install/local_setup.bash
cd ~/ros2_overlay_ws
colcon build --symlink-install
```

## 5. Running the Demo
1.  Preparation
	-  copy label files (excute _once_)
	```bash
	sudo cp ~/ros2_overlay_ws/src/ros2_openvino_toolkit/data/labels/emotions-recognition/FP32/emotions-recognition-retail-0003.labels /opt/intel/computer_vision_sdk/deployment_tools/intel_models/emotions-recognition-retail-0003/FP32
	```
	- set OpenVINO toolkit ENV
	```bash
	source /opt/intel/computer_vision_sdk/bin/setupvars.sh
	```
	- set ENV LD_LIBRARY_PATH
	```bash
	export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/opt/intel/computer_vision_sdk/deployment_tools/inference_engine/samples/build/intel64/Release/lib
	```

2.  run sample code with command-line arguments for <b>image or video</b> detection
```bash
ros2 run dynamic_vino_sample dynamic_vino_sample -m <model_path_for_face_detection> -m_em <model_path_for_emotion_detection> -m_ag <model_path_for_age_and_gender_detection> -m-hp <model_path_for_headpose_detection> -i <input_type> -i_path <file_directory> -d <device> -d_em <device> -d_ag <device> -d_hp <device>
```

Options for &lt;input_type&gt;: _Image_ or _Video_.

Options for &lt;device&gt;: _CPU_ or _GPU_. Default is _CPU_. 
    
For example,
```bash
ros2 run dynamic_vino_sample dynamic_vino_sample -m /opt/intel/computer_vision_sdk/deployment_tools/intel_models/face-detection-adas-0001/FP32/face-detection-adas-0001.xml -m_em /opt/intel/computer_vision_sdk/deployment_tools/intel_models/emotions-recognition-retail-0003/FP32/emotions-recognition-retail-0003.xml -m_ag /opt/intel/computer_vision_sdk/deployment_tools/intel_models/age-gender-recognition-retail-0013/FP32/age-gender-recognition-retail-0013.xml -m-hp /opt/intel/computer_vision_sdk/deployment_tools/intel_models/head-pose-estimation-adas-0001/FP32/head-pose-estimation-adas-0001.xml -i Image -i_path <file_directory> -d CPU -d_em CPU -d_ag GPU -d_hp GPU
```

3. run sample code with command-line arguments for <b>video stream from camera</b> detection
```bash
ros2 run dynamic_vino_sample dynamic_vino_sample -m <model_path_for_face_detection> -m_em <model_path_for_emotion_detection> -m_ag <model_path_for_age_and_gender_detection> -m-hp <model_path_for_headpose_detection> -i <input_type> -d <device> -d_em <device> -d_ag <device> -d_hp <device>
```

Options for &lt;input_type&gt;: _StandardCamera_ or _RealSenseCamera_. Default is _StandardCamera_.

Options for &lt;device&gt;: _CPU_ or _GPU_. Default is _CPU_. 
    
For example, 
```bash
ros2 run dynamic_vino_sample dynamic_vino_sample -m /opt/intel/computer_vision_sdk/deployment_tools/intel_models/face-detection-adas-0001/FP32/face-detection-adas-0001.xml -m_em /opt/intel/computer_vision_sdk/deployment_tools/intel_models/emotions-recognition-retail-0003/FP32/emotions-recognition-retail-0003.xml -m_ag /opt/intel/computer_vision_sdk/deployment_tools/intel_models/age-gender-recognition-retail-0013/FP32/age-gender-recognition-retail-0013.xml -m-hp /opt/intel/computer_vision_sdk/deployment_tools/intel_models/head-pose-estimation-adas-0001/FP32/head-pose-estimation-adas-0001.xml -i StandardCamera -d CPU -d_em CPU -d_ag GPU -d_hp GPU
```
4. run sample code with parameters extracted from [yaml](https://github.com/intel/ros2_openvino_toolkit/blob/master/vino_param_lib/param/pipeline.yaml).
```bash
ros2 run dynamic_vino_sample pipeline_with_params
```

## 6. Interfaces
### 6.1 Topic
- Face Detection:
```/openvino_toolkit/faces```([object_msgs:msg:ObjectsInBoxes](https://github.com/intel/ros2_object_msgs/blob/master/msg/ObjectsInBoxes.msg))
- Emotion Detection:
```/openvino_toolkit/emotions```([people_msgs:msg:EmotionsStamped](https://github.com/intel/ros2_openvino_toolkit/blob/master/people_msgs/msg/EmotionsStamped.msg))
- Age and Gender Detection:
```/openvino_toolkit/age_genders```([people_msgs:msg:AgeGenderStamped](https://github.com/intel/ros2_openvino_toolkit/blob/master/people_msgs/msg/AgeGenderStamped.msg))
- Head Pose:
```/openvino_toolkit/headposes```([people_msgs:msg:HeadPoseStamped](https://github.com/intel/ros2_openvino_toolkit/blob/master/people_msgs/msg/HeadPoseStamped.msg))

## 7. Known Issues
- Parameters "-m_ag, -m_hp, -m_em" should be optional, but samples throw exception without them.
- Parameters "-n_ag, -n_hp, -n_em" doesn't work. The maximum number of face/age/headpose/emotion is always 16.
- Standard USB camera can be unexpected launched with input parameter "-i RealSenseCamera". 

###### *Any security issue should be reported using process at https://01.org/security*
