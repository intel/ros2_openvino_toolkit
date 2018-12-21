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
* Object detection
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

- Use one of the following methods to determine the GPU on your hardware:
	* [lspci] command: GPU info may lie in the [VGA compatible controller] line.
	* Ubuntu system: Menu [System Settings] --> [Details] may help you find the graphics information.
	* Openvino: Download the install package, install_GUI.sh inside will check the GPU information before installation.

## 3. Environment Setup
* Install ROS2 [Bouncy](https://github.com/ros2/ros2/wiki) ([guide](https://github.com/ros2/ros2/wiki/Linux-Development-Setup))<br>
* Install [OpenVINO™ Toolkit](https://software.intel.com/en-us/openvino-toolkit) ([guide](https://software.intel.com/en-us/articles/OpenVINO-Install-Linux))<br>
    	**Note**: Please use  *root privileges* to run the installer when installing the core components.
* Install OpenCL Driver for GPU
	```bash
	cd /opt/intel/computer_vision_sdk/install_dependencies
	sudo ./install_NEO_OCL_driver.sh
	```
- Other Dependencies
	```bash
	#librealsense dependency
	sudo apt-get install -y libssl-dev libusb-1.0-0-dev pkg-config libgtk-3-dev
	sudo apt-get install -y libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev
	# numpy
	pip3 install numpy
	# libboost
	sudo apt-get install -y --no-install-recommends libboost-all-dev
	cd /usr/lib/x86_64-linux-gnu
	sudo ln -s libboost_python-py35.so libboost_python3.so
	```
## 4. Building and Installation
**Note**:You can choose to build the environment using *./environment_setup.sh* script in the script subfolder.
```bash
./environment_setup_binary.sh
```
**Note**:You can also choose to follow the steps below to build the environment step by step.
* Build sample code under openvino toolkit
	```bash
	# root is required instead of sudo
	source /opt/intel/computer_vision_sdk/bin/setupvars.sh
	cd /opt/intel/computer_vision_sdk/deployment_tools/inference_engine/samples/
	mkdir build
	cd build
	cmake ..
	make
	```
* set ENV CPU_EXTENSION_LIB and GFLAGS_LIB
	```bash
	export CPU_EXTENSION_LIB=/opt/intel/computer_vision_sdk/deployment_tools/inference_engine/samples/build/intel64/Release/lib/libcpu_extension.so
	export GFLAGS_LIB=/opt/intel/computer_vision_sdk/deployment_tools/inference_engine/samples/build/intel64/Release/lib/libgflags_nothreads.a
	```
* Install ROS2_OpenVINO packages
	```bash
	mkdir -p ~/ros2_overlay_ws/src
	cd ~/ros2_overlay_ws/src
	git clone https://github.com/intel/ros2_openvino_toolkit
	git clone https://github.com/intel/ros2_object_msgs
	git clone https://github.com/ros-perception/vision_opencv -b ros2
	git clone https://github.com/ros2/message_filters.git
	git clone https://github.com/ros-perception/image_common.git -b ros2
	git clone https://github.com/IntelRealSense/librealsense.git -b ros2debian
	git clone https://github.com/intel/ros2_intel_realsense.git
	cd ~/ros2_overlay_ws/src/librealsense
	sudo cp ./config/99-realsense-libusb.rules /etc/udev/rules.d/
	sudo udevadm control --reload-rules
	udevadm trigger
	```

* Build package
	```
	source ~/ros2_ws/install/local_setup.bash
	source /opt/intel/computer_vision_sdk/bin/setupvars.sh
	cd ~/ros2_overlay_ws
	colcon build --symlink-install
	source ./install/local_setup.bash
 	sudo mkdir -p /opt/openvino_toolkit
 	sudo ln -s ~/ros2_overlay_ws/src/ros2_openvino_toolkit /opt/openvino_toolkit/ros2_openvino_toolkit
	```
	
## 5. Running the Demo
* Preparation
	*  copy label files (excute _once_)
		```bash
		sudo cp /opt/openvino_toolkit/ros2_openvino_toolkit/data/labels/emotions-recognition/FP32/emotions-recognition-retail-0003.labels /opt/intel/computer_vision_sdk/deployment_tools/intel_models/emotions-recognition-retail-0003/FP32
		```
	* set OpenVINO toolkit ENV
		```bash
		source /opt/intel/computer_vision_sdk/bin/setupvars.sh
		```
	* set ENV LD_LIBRARY_PATH
		```bash
		export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/opt/intel/computer_vision_sdk/deployment_tools/inference_engine/samples/build/intel64/Release/lib
		```
**Note**:In [pipeline_people.yaml](https://github.com/intel/ros2_openvino_toolkit/blob/master/sample/param/pipeline_people.yaml) and [pipeline_object.yaml](https://github.com/intel/ros2_openvino_toolkit/blob/master/sample/param/pipeline_object.yaml) yaml file,
> Options for inputs parameter: StandardCamera, RealSenseCamera, Image, Video or RealSenseCameraTopic. Default is StandardCamera. If RealSenseCameraTopic is chosen in the yaml file, you should start the realsense node firstly. 

> Options for outputs parameter: ImageWindow, RosTopic and RViz. If RViz is chosen in the yaml file, the output results can be shown in RVIZ tool.
* Start the realsense camera node.To start the camera node in ROS2, plug in the realsense camera, then type the following command:
	```bash
	source ~/ros2_ws/install/local_setup.bash
	source ~/ros2_overlay_ws/install/local_setup.bash
	# To launch with "ros2 run"
	ros2 run realsense_ros2_camera realsense_ros2_camera
	# OR, to invoke the executable directly
	realsense_ros2_camera
	```
* run face detection sample code with parameters extracted from [yaml](https://github.com/intel/ros2_openvino_toolkit/blob/master/sample/param/pipeline_people.yaml).
	```bash
	ros2 run dynamic_vino_sample pipeline_with_params -config /opt/openvino_toolkit/ros2_openvino_toolkit/sample/param/pipeline_people.yaml
	```
* run object detection sample code with paramters extracted from [yaml](https://github.com/intel/ros2_openvino_toolkit/blob/master/sample/param/pipeline_object.yaml).
	```bash
	ros2 run dynamic_vino_sample pipeline_with_params -config /opt/openvino_toolkit/ros2_openvino_toolkit/sample/param/pipeline_object.yaml
	```
* View the output result in RVIZ
	```bash
	#launch rviz2
	source ~/ros2_overlay_ws/install/local_setup.bash
	ros2 run rviz2 rviz2
	# add image and select /openvino_toolkit/images topic in rviz.
	```

## 6. Interfaces
### 6.1 Topic
- Face Detection:
```/openvino_toolkit/faces```([object_msgs:msg:ObjectsInBoxes](https://github.com/intel/ros2_object_msgs/blob/master/msg/ObjectsInBoxes.msg))
- Emotion Recognition:
```/openvino_toolkit/emotions```([people_msgs:msg:EmotionsStamped](https://github.com/intel/ros2_openvino_toolkit/blob/master/people_msgs/msg/EmotionsStamped.msg))
- Age and Gender Recognition:
```/openvino_toolkit/age_genders```([people_msgs:msg:AgeGenderStamped](https://github.com/intel/ros2_openvino_toolkit/blob/master/people_msgs/msg/AgeGenderStamped.msg))
- Head Pose Estimation:
```/openvino_toolkit/headposes```([people_msgs:msg:HeadPoseStamped](https://github.com/intel/ros2_openvino_toolkit/blob/master/people_msgs/msg/HeadPoseStamped.msg))
- Object Detection:
```/openvino_toolkit/objects```([object_msgs::msg::ObjectsInBoxes](https://github.com/intel/ros2_object_msgs/blob/master/msg/ObjectsInBoxes.msg))
- Rviz Output:
```/openvino_toolkit/images```([sensor_msgs::msg::Image](https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/msg/Image.msg))

## 7. Known Issues
- In the yaml file,inputs parameters "Image, Video" are not yet supported.

###### *Any security issue should be reported using process at https://01.org/security*


