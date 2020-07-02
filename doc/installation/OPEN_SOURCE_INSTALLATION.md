# ros2_openvino_toolkit

## 1. Prerequisite
- An x86_64 computer running Ubuntu 18.04. Below processors are supported:
	* 6th-8th Generation Intel® Core™
	* Intel® Xeon® v5 family
	* Intel®  Xeon® v6 family
- ROS2 [Dashing](https://github.com/ros2/ros2/wiki)

- OpenVINO™ Toolkit Open Source<br>
  	* The [Deep Learning Deployment Toolkit](https://github.com/openvinotoolkit/openvino) that helps to enable fast, heterogeneous deep learning inferencing for Intel® processors (CPU and GPU/Intel® Processor Graphics), and supports more than 100 public and custom models.<br>
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

## 2. Environment Setup
**Note**:You can choose to build the environment using *./environment_setup_binary.sh* script in the script subfolder.The *modules.conf* file in the same directory as the .sh file is the configuration file that controls the installation process.You can modify the *modules.conf* to customize your installation process.
```bash
./environment_setup.sh
```
**Note**:You can also choose to follow the steps below to build the environment step by step.
* Install ROS2 [Dashing](https://github.com/ros2/ros2/wiki) ([guide](https://index.ros.org/doc/ros2/Installation/Dashing/Linux-Development-Setup/))<br>
* Install OpenVINO™ Toolkit Open Source<br>
	* Install OpenCL Driver for GPU<br>
		```bash
		cd ~/Downloads
		wget https://github.com/intel/compute-runtime/releases/download/19.04.12237/intel-gmmlib_18.4.1_amd64.deb
		wget https://github.com/intel/compute-runtime/releases/download/19.04.12237/intel-igc-core_18.50.1270_amd64.deb
		wget https://github.com/intel/compute-runtime/releases/download/19.04.12237/intel-igc-opencl_18.50.1270_amd64.deb
		wget https://github.com/intel/compute-runtime/releases/download/19.04.12237/intel-opencl_19.04.12237_amd64.deb
		wget https://github.com/intel/compute-runtime/releases/download/19.04.12237/intel-ocloc_19.04.12237_amd64.deb
		sudo dpkg -i *.deb
		```
	* Install [Deep Learning Deployment Toolkit](https://github.com/openvinotoolkit/openvino)([tag 2019_R3.1](https://github.com/openvinotoolkit/openvino/blob/2019_R3.1/inference-engine/README.md))<br>
	* Install [Open Model Zoo](https://github.com/opencv/open_model_zoo)([tag 2019_R3.1](https://github.com/opencv/open_model_zoo/blob/2019_R3.1/demos/README.md))<br>
	
- Install Intel® RealSense™ SDK 2.0 [(tag v2.30.0)](https://github.com/IntelRealSense/librealsense/tree/v2.30.0)<br>
	* [Install from package](https://github.com/IntelRealSense/librealsense/blob/v2.30.0/doc/distribution_linux.md)<br>

## 3. Building and Installation

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
	git clone https://github.com/ros-perception/image_common.git -b dashing
	git clone https://github.com/intel/ros2_intel_realsense.git -b refactor
	```

* Build package
	```
	source ~/ros2_ws/install/local_setup.bash
	cd ~/ros2_overlay_ws
	colcon build --symlink-install
	source ./install/local_setup.bash
	sudo mkdir -p /opt/openvino_toolkit
	sudo ln -sf ~/ros2_overlay_ws/src/ros2_openvino_toolkit /opt/openvino_toolkit/
	```





