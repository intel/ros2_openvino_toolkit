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
- An x86_64 computer running Ubuntu 16.04 and Ubuntu 18.04. Below processors are supported:
	* 6th-8th Generation Intel® Core™
	* Intel® Xeon® v5 family
	* Intel®  Xeon® v6 family
- ROS2 [Bouncy](https://github.com/ros2/ros2/wiki)

- OpenVINO™ Toolkit Open Source<br>
  	* The [Deep Learning Deployment Toolkit](https://github.com/opencv/dldt) that helps to enable fast, heterogeneous deep learning inferencing for Intel® processors (CPU and GPU/Intel® Processor Graphics), and supports more than 100 public and custom models.<br>
	* [Open Model Zoo](https://github.com/opencv/open_model_zoo) includes 20+ pre-trained deep learning models to expedite development and improve deep learning inference on Intel® processors (CPU, Intel Processor Graphics, FPGA, VPU), along with many samples to easily get started.
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
**Note**:You can choose to build the environment using *./environment_setup.sh* script in the script subfolder.
```bash
./environment_setup.sh
```
**Note**:You can also choose to follow the steps below to build the environment step by step.
* Install ROS2 [Bouncy](https://github.com/ros2/ros2/wiki) ([guide](https://github.com/ros2/ros2/wiki/Linux-Development-Setup))<br>
* Install OpenVINO™ Toolkit Open Source<br>
	* Install [OpenCV 3.3 or later](https://docs.opencv.org/master/d9/df8/tutorial_root.html)([guide](https://docs.opencv.org/master/d7/d9f/tutorial_linux_install.html))
		```bash
		[compiler] sudo apt-get install build-essential
		[required] sudo apt-get install cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev
		[optional] sudo apt-get install python-dev python-numpy libtbb2 libtbb-dev libjpeg-dev libpng-dev libtiff-dev libjasper-dev libdc1394-22-dev 
		cd ~/code
		git clone https://github.com/opencv/opencv.git
		git clone https://github.com/opencv/opencv_contrib.git
		cd opencv && git checkout 3.4.0 && cd ..
		cd opencv_contrib && git checkout 3.4.0 && cd ..
		cd opencv
		mkdir build && cd build
		cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local -D OPENCV_EXTRA_MODULES_PATH=/home/<hostname>/code/opencv_contrib/modules/ ..
		make -j8
		sudo make install
		```
		* Additional steps are required on ubuntu 18.04
			```bash
			sudo add-apt-repository "deb http://security.ubuntu.com/ubuntu xenial-security main"
			sudo apt update
			sudo apt install libjasper1 libjasper-dev
			```
	* Install OpenCL Driver for GPU([guide](http://registrationcenter-download.intel.com/akdlm/irc_nas/11396/intel-opencl-4.1-installation.pdf))<br>
		```bash
		cd ~/Downloads
		wget http://registrationcenter-download.intel.com/akdlm/irc_nas/11396/SRB5.0_linux64.zip
		unzip SRB5.0_linux64.zip -d SRB5.0_linux64
		cd SRB5.0_linux64
		sudo apt-get install xz-utils
		mkdir intel-opencl
		tar -C intel-opencl -Jxf intel-opencl-r5.0-63503.x86_64.tar.xz
		tar -C intel-opencl -Jxf intel-opencl-devel-r5.0-63503.x86_64.tar.xz
		tar -C intel-opencl -Jxf intel-opencl-cpu-r5.0-63503.x86_64.tar.xz
		sudo cp -R intel-opencl/* /
		sudo ldconfig
		```
	* Install [Deep Learning Deployment Toolkit](https://github.com/opencv/dldt)([guide](https://github.com/opencv/dldt/tree/2018/inference-engine))<br>
		```bash
		mkdir ~/code && cd ~/code
		git clone https://github.com/opencv/dldt.git
		cd dldt/inference-engine/
		git checkout 2018_R4
		git submodule init
		git submodule update --recursive
		./install_dependencies.sh
		mkdir build && cd build
		cmake -DCMAKE_BUILD_TYPE=Release ..
		make -j8
		sudo mkdir -p /opt/openvino_toolkit
		sudo ln -s ~/code/dldt /opt/openvino_toolkit/dldt
		```
	* Install [Open Model Zoo](https://github.com/opencv/open_model_zoo)([guide](https://github.com/opencv/open_model_zoo/tree/2018/demos))<br>
		```bash
		cd ~/code
		git clone https://github.com/opencv/open_model_zoo.git
		cd open_model_zoo/demos/
		git checkout 2018_R4
		mkdir build && cd build
		cmake -DCMAKE_BUILD_TYPE=Release /opt/openvino_toolkit/dldt/inference-engine
		make -j8
		sudo mkdir -p /opt/openvino_toolkit
		sudo ln -s ~/code/open_model_zoo /opt/openvino_toolkit/open_model_zoo
		```
- Other Dependencies
	```bash
	#librealsense dependency
	sudo apt-get install -y libssl-dev libusb-1.0-0-dev pkg-config libgtk-3-dev
	sudo apt-get install -y libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev
	# numpy
	pip3 install numpy
	```
	* Ubuntu 16.04
		```bash
		# libboost
		sudo apt-get install -y --no-install-recommends libboost-all-dev
		cd /usr/lib/x86_64-linux-gnu
		sudo ln -s libboost_python-py35.so libboost_python3.so
		```
	* Ubuntu 18.04
		```bash
		#libboost
		sudo apt-get install -y --no-install-recommends libboost-all-dev
		sudo apt install libboost-python1.62.0
		```
## 4. Building and Installation

* set ENV InferenceEngine_DIR, CPU_EXTENSION_LIB and GFLAGS_LIB
	```bash
	export InferenceEngine_DIR=/opt/openvino_toolkit/dldt/inference-engine/build/
	export CPU_EXTENSION_LIB=/opt/openvino_toolkit/dldt/inference-engine/bin/intel64/Release/lib/libcpu_extension.so
	export GFLAGS_LIB=/opt/openvino_toolkit/dldt/inference-engine/bin/intel64/Release/lib/libgflags_nothreads.a
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
	cd ~/ros2_overlay_ws
	colcon build --symlink-install
	source ./install/local_setup.bash
	sudo mkdir -p /opt/openvino_toolkit
	sudo ln -s ~/ros2_overlay_ws/src/ros2_openvino_toolkit /opt/openvino_toolkit/ros2_openvino_toolkit
	```
	
## 5. Running the Demo
* Preparation
	* download model file (excute _once_)<br>
		```bash
		cd /opt/openvino_toolkit/open_model_zoo/model_downloader
		python3 downloader.py --name face-detection-adas-0001
		python3 downloader.py --name age-gender-recognition-retail-0013
		python3 downloader.py --name emotions-recognition-retail-0003
		python3 downloader.py --name head-pose-estimation-adas-0001
		python3 downloader.py --name person-vehicle-bike-detection-crossroad-0078
		```
	* copy label files (excute _once_)<br>
		```bash
		sudo cp /opt/openvino_toolkit/ros2_openvino_toolkit/data/labels/emotions-recognition/FP32/emotions-recognition-retail-0003.labels /opt/openvino_toolkit/open_model_zoo/model_downloader/Retail/object_attributes/emotions_recognition/0003/dldt
		```
	* set ENV LD_LIBRARY_PATH<br>
		```bash
		export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/opt/openvino_toolkit/dldt/inference-engine/bin/intel64/Release/lib
		```
**Note**:In [pipeline_people_oss.yaml](https://github.com/intel/ros2_openvino_toolkit/blob/master/sample/param/pipeline_people_oss.yaml) and [pipeline_object_oss.yaml](https://github.com/intel/ros2_openvino_toolkit/blob/master/sample/param/pipeline_object_oss.yaml) yaml file:<br>
> Options for inputs parameter: StandardCamera, RealSenseCamera, Image, Video or RealSenseCameraTopic. Default is StandardCamera. If RealSenseCameraTopic is chosen in the yaml file, you should start the realsense node firstly.</br>

> Options for outputs parameter: ImageWindow, RosTopic and RViz. If RViz is chosen in the yaml file, the output results can be shown in RVIZ tool.</br>
* Start the realsense camera node.To start the camera node in ROS2, plug in the realsense camera, then type the following command:
	```bash
	source ~/ros2_ws/install/local_setup.bash
	source ~/ros2_overlay_ws/install/local_setup.bash
	# To launch with "ros2 run"
	ros2 run realsense_ros2_camera realsense_ros2_camera
	# OR, to invoke the executable directly
	realsense_ros2_camera
	```
* run face detection sample code with parameters extracted from [yaml](https://github.com/intel/ros2_openvino_toolkit/blob/master/sample/param/pipeline_people_oss.yaml).
	```bash
	ros2 run dynamic_vino_sample pipeline_with_params -config /opt/openvino_toolkit/ros2_openvino_toolkit/sample/param/pipeline_people_oss.yaml
	```
* run object detection sample code with paramters extracted from [yaml](https://github.com/intel/ros2_openvino_toolkit/blob/master/sample/param/pipeline_object_oss.yaml).
	```bash
	ros2 run dynamic_vino_sample pipeline_with_params -config /opt/openvino_toolkit/ros2_openvino_toolkit/sample/param/pipeline_object_oss.yaml
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


