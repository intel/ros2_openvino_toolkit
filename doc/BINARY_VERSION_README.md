# ros2_openvino_toolkit

## 1. Introduction
The [OpenVINO™](https://software.intel.com/en-us/openvino-toolkit) toolkit quickly deploys applications and solutions that emulate human vision. Based on Convolutional Neural Networks (CNN), the Toolkit extends computer vision (CV) workloads across Intel® hardware, maximizing performance.

This project is a ROS2 wrapper for CV API of [OpenVINO™](https://software.intel.com/en-us/openvino-toolkit), providing the following features:
* Support CPU, GPU and Intel® Neural Compute Stick 2 platforms
* Support standard USB camera and Intel® RealSense™ camera
* Support Video or Image file as detection source
* Face detection
* Emotion recognition
* Age and gender recognition
* Head pose recognition
* Object detection
* Demo application to show above detection and recognitions

## 2. Prerequisite
- An x86_64 computer running Ubuntu 18.04. Below processors are supported:
	* 6th-8th Generation Intel® Core™
	* Intel® Xeon® v5 family
	* Intel®  Xeon® v6 family
- ROS2 [Crystal](https://github.com/ros2/ros2/wiki)
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
**Note**:You can choose to build the environment using *./environment_setup_binary.sh* script in the script subfolder.The *modules.conf* file in the same directory as the .sh file is the configuration file that controls the installation process.You can modify the *modules.conf* to customize your installation process.
```bash
./environment_setup_binary.sh
```
**Note**:You can also choose to follow the steps below to build the environment step by step.
* Install ROS2 [Crystal](https://github.com/ros2/ros2/wiki) ([guide](https://index.ros.org/doc/ros2/Installation/Linux-Development-Setup/))<br>
* Install [OpenVINO™ Toolkit](https://software.intel.com/en-us/openvino-toolkit) ([guide](https://software.intel.com/en-us/articles/OpenVINO-Install-Linux))<br>
    	**Note**: Please use  *root privileges* to run the installer when installing the core components.
* Install OpenCL Driver for GPU
	```bash
	cd /opt/intel/openvino/install_dependencies
	sudo ./install_NEO_OCL_driver.sh
	```
* Install [OpenCV 3.4 or later](https://docs.opencv.org/master/d9/df8/tutorial_root.html)([guide](https://docs.opencv.org/master/d7/d9f/tutorial_linux_install.html))
	```bash
	[compiler] sudo apt-get install build-essential
	[required] sudo apt-get install cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev
	[optional] sudo apt-get install python-dev python-numpy libtbb2 libtbb-dev libjpeg-dev libpng-dev libtiff-dev libjasper-dev libdc1394-22-dev 
	mkdir -p ~/code && cd ~/code
	git clone https://github.com/opencv/opencv.git
	git clone https://github.com/opencv/opencv_contrib.git
	cd opencv && git checkout 3.4.2 && cd ..
	cd opencv_contrib && git checkout 3.4.2 && cd ..
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
- Install Intel® RealSense™ SDK 2.0 [(tag v2.17.1)](https://github.com/IntelRealSense/librealsense/tree/v2.17.1)<br>
	* [Install from source code](https://github.com/IntelRealSense/librealsense/blob/v2.17.1/doc/installation.md)(Recommended)<br>
		```bash
		 sudo apt-get install git libssl-dev libusb-1.0-0-dev pkg-config libgtk-3-dev
		 sudo apt-get install libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev
		 mkdir -p ~/code && cd ~/code
		 git clone https://github.com/IntelRealSense/librealsense
		 cd ~/code/librealsense
		 git checkout v2.17.1
		 mkdir build && cd build
		 cmake ../ -DBUILD_EXAMPLES=true
		 sudo make uninstall 
		 make clean
		 make
		 sudo make install
		 cd ..
		 sudo cp config/99-realsense-libusb.rules /etc/udev/rules.d/
		 sudo udevadm control --reload-rules
		 udevadm trigger
		```
	* [Install from package](https://github.com/IntelRealSense/librealsense/blob/v2.17.1/doc/distribution_linux.md)<br>
- Other Dependencies
	```bash
	#librealsense dependency
	sudo apt-get install -y libssl-dev libusb-1.0-0-dev pkg-config libgtk-3-dev
	sudo apt-get install -y libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev
	# numpy and networkx
	pip3 install numpy
	pip3 install networkx
	# libboost
	sudo apt-get install -y --no-install-recommends libboost-all-dev
	cd /usr/lib/x86_64-linux-gnu
	sudo ln -sf libboost_python-py35.so libboost_python3.so
	```
## 4. Building and Installation
* Build sample code under openvino toolkit
	```bash
	# root is required instead of sudo
	source /opt/intel/openvino/bin/setupvars.sh
	cd /opt/intel/openvino/deployment_tools/inference_engine/samples/
	mkdir build
	cd build
	cmake ..
	make
	```
* set ENV CPU_EXTENSION_LIB and GFLAGS_LIB
	```bash
	export CPU_EXTENSION_LIB=/opt/intel/openvino/deployment_tools/inference_engine/samples/build/intel64/Release/lib/libcpu_extension.so
	export GFLAGS_LIB=/opt/intel/openvino/deployment_tools/inference_engine/samples/build/intel64/Release/lib/libgflags_nothreads.a
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
	git clone https://github.com/intel/ros2_intel_realsense.git
	```

* Build package
	```
	source ~/ros2_ws/install/local_setup.bash
	source /opt/intel/openvino/bin/setupvars.sh
	export OpenCV_DIR=$HOME/code/opencv/build
	cd ~/ros2_overlay_ws
	colcon build --symlink-install
	source ./install/local_setup.bash
 	sudo mkdir -p /opt/openvino_toolkit
 	sudo ln -sf ~/ros2_overlay_ws/src/ros2_openvino_toolkit /opt/openvino_toolkit/
	```
	
## 5. Running the Demo
* Preparation
	* Configure the Neural Compute Stick USB Driver
	```bash
	cd ~/Downloads
	cat <<EOF > 97-usbboot.rules
	SUBSYSTEM=="usb", ATTRS{idProduct}=="2150", ATTRS{idVendor}=="03e7", GROUP="users", MODE="0666", ENV{ID_MM_DEVICE_IGNORE}="1"
	SUBSYSTEM=="usb", ATTRS{idProduct}=="2485", ATTRS{idVendor}=="03e7", GROUP="users", MODE="0666", ENV{ID_MM_DEVICE_IGNORE}="1"
	SUBSYSTEM=="usb", ATTRS{idProduct}=="f63b", ATTRS{idVendor}=="03e7", GROUP="users", MODE="0666", ENV{ID_MM_DEVICE_IGNORE}="1"
	EOF
	sudo cp 97-usbboot.rules /etc/udev/rules.d/
	sudo udevadm control --reload-rules
	sudo udevadm trigger
	sudo ldconfig
	rm 97-usbboot.rules
	```
	* download and convert a trained model to produce an optimized Intermediate Representation (IR) of the model 
	```bash
	#object segmentation model
	cd /opt/intel/openvino/deployment_tools/model_optimizer/install_prerequisites
	sudo ./install_prerequisites.sh
	mkdir -p ~/Downloads/models
	cd ~/Downloads/models
	wget http://download.tensorflow.org/models/object_detection/mask_rcnn_inception_v2_coco_2018_01_28.tar.gz
	tar -zxvf mask_rcnn_inception_v2_coco_2018_01_28.tar.gz
	cd mask_rcnn_inception_v2_coco_2018_01_28
	python3 /opt/intel/openvino/deployment_tools/model_optimizer/mo_tf.py --input_model frozen_inference_graph.pb --tensorflow_use_custom_operations_config /opt/intel/openvino/deployment_tools/model_optimizer/extensions/front/tf/mask_rcnn_support.json --tensorflow_object_detection_api_pipeline_config pipeline.config --reverse_input_channels --output_dir ./output/
	sudo mkdir -p /opt/models
	sudo ln -sf ~/Downloads/models/mask_rcnn_inception_v2_coco_2018_01_28 /opt/models/
	#object detection model
	cd /opt/intel/openvino/deployment_tools/tools/model_downloader
	sudo python3 ./downloader.py --name mobilenet-ssd
	#FP32 precision model
	sudo python3 /opt/intel/openvino/deployment_tools/model_optimizer/mo.py --input_model /opt/intel/openvino/deployment_tools/tools/model_downloader/object_detection/common/mobilenet-ssd/caffe/mobilenet-ssd.caffemodel --output_dir /opt/intel/openvino/deployment_tools/tools/model_downloader/object_detection/common/mobilenet-ssd/caffe/output/FP32 --mean_values [127.5,127.5,127.5] --scale_values [127.5]
	#FP16 precision model
	sudo python3 /opt/intel/openvino/deployment_tools/model_optimizer/mo.py --input_model /opt/intel/openvino/deployment_tools/tools/model_downloader/object_detection/common/mobilenet-ssd/caffe/mobilenet-ssd.caffemodel --output_dir /opt/intel/openvino/deployment_tools/tools/model_downloader/object_detection/common/mobilenet-ssd/caffe/output/FP16 --data_type=FP16 --mean_values [127.5,127.5,127.5] --scale_values [127.5]
	```
	* download the optimized Intermediate Representation (IR) of model (excute once)
	```bash
	sudo python3 downloader.py --name face-detection-adas-0001
	sudo python3 downloader.py --name face-detection-adas-0001-fp16
	sudo python3 downloader.py --name age-gender-recognition-retail-0013
	sudo python3 downloader.py --name emotions-recognition-retail-0003
	sudo python3 downloader.py --name head-pose-estimation-adas-0001
	sudo python3 downloader.py --name person-detection-retail-0013
	sudo python3 downloader.py --name person-reidentification-retail-0076
	sudo python3 downloader.py --name landmarks-regression-retail-0009
	sudo python3 downloader.py --name face-reidentification-retail-0095
	sudo python3 downloader.py --name vehicle-license-plate-detection-barrier-0106
	sudo python3 downloader.py --name vehicle-attributes-recognition-barrier-0039
	sudo python3 downloader.py --name license-plate-recognition-barrier-0001
	```
		
	* copy label files (excute _once_)<br>
	```bash
	sudo cp /opt/openvino_toolkit/ros2_openvino_toolkit/data/labels/emotions-recognition/FP32/emotions-recognition-retail-0003.labels /opt/intel/openvino/deployment_tools/tools/model_downloader/Retail/object_attributes/emotions_recognition/0003/dldt
	sudo cp /opt/openvino_toolkit/ros2_openvino_toolkit/data/labels/face_detection/face-detection-adas-0001.labels /opt/intel/openvino/deployment_tools/tools/model_downloader/Transportation/object_detection/face/pruned_mobilenet_reduced_ssd_shared_weights/dldt
	sudo cp /opt/openvino_toolkit/ros2_openvino_toolkit/data/labels/face_detection/face-detection-adas-0001-fp16.labels /opt/intel/openvino/deployment_tools/tools/model_downloader/Transportation/object_detection/face/pruned_mobilenet_reduced_ssd_shared_weights/dldt
	sudo cp /opt/openvino_toolkit/ros2_openvino_toolkit/data/labels/object_segmentation/frozen_inference_graph.labels ~/Downloads/models/mask_rcnn_inception_v2_coco_2018_01_28/output
	sudo cp /opt/openvino_toolkit/ros2_openvino_toolkit/data/labels/object_detection/mobilenet-ssd.labels /opt/intel/openvino/deployment_tools/tools/model_downloader/object_detection/common/mobilenet-ssd/caffe/output/FP32
	sudo cp /opt/openvino_toolkit/ros2_openvino_toolkit/data/labels/object_detection/mobilenet-ssd.labels /opt/intel/openvino/deployment_tools/tools/model_downloader/object_detection/common/mobilenet-ssd/caffe/output/FP16
	sudo cp /opt/openvino_toolkit/ros2_openvino_toolkit/data/labels/object_detection/vehicle-license-plate-detection-barrier-0106.labels /opt/intel/openvino/deployment_tools/tools/model_downloader/Security/object_detection/barrier/0106/dldt
	```
	* set ENV LD_LIBRARY_PATH and environment
	```bash
	source /opt/intel/openvino/bin/setupvars.sh
	export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/opt/intel/openvino/deployment_tools/inference_engine/samples/build/intel64/Release/lib
	```
* run face detection sample code input from StandardCamera.(connect Intel® Neural Compute Stick 2)
	```bash
	ros2 launch dynamic_vino_sample pipeline_people_myriad.launch.py
	```
* run face detection sample code input from Image.
	```bash
	ros2 launch dynamic_vino_sample pipeline_image.launch.py
	```
* run object detection sample code input from RealSenseCamera.(connect Intel® Neural Compute Stick 2)
	```bash
	ros2 launch dynamic_vino_sample pipeline_object.launch.py
	```
* run object detection sample code input from RealSenseCameraTopic.(connect Intel® Neural Compute Stick 2)
	```bash
	ros2 launch dynamic_vino_sample pipeline_object_topic.launch.py
	```
* run object segmentation sample code input from RealSenseCameraTopic.(connect Intel® Neural Compute Stick 2)
	```bash
	ros2 launch dynamic_vino_sample pipeline_segmentation.launch.py
	```
* run object segmentation sample code input from Video.
	```bash
	ros2 launch dynamic_vino_sample pipeline_video.launch.py
	```
* run person reidentification sample code input from StandardCamera.
	```bash
	ros2 launch dynamic_vino_sample pipeline_reidentification.launch.py
	```
* run vehicle detection sample code input from StandardCamera.
	```bash
	ros2 launch dynamic_vino_sample pipeline_vehicle_detection.launch.py
	```
* run object detection service sample code input from Image  
  Run image processing service:
	```bash
	ros2 launch dynamic_vino_sample image_object_server.launch.py
	```
  Run example application with an absolute path of an image on another console:
	```bash
	ros2 run dynamic_vino_sample image_object_client ~/Pictures/car.png
	```
* run people detection service sample code input from Image  
  Run image processing service:
	```bash
	ros2 launch dynamic_vino_sample image_people_server.launch.py
	```
  Run example application with an absolute path of an image on another console:
	```bash
	ros2 run dynamic_vino_sample image_people_client ~/Pictures/face.png
	```


## 6.Known Issues
* Possible problems
	* When running sample with Intel® Neural Compute Stick 2 occurred error:
		```bash
		E: [ncAPI] [         0] ncDeviceOpen:668	failed to find device
		# or
		E: [ncAPI] [         0] ncDeviceCreate:324      global mutex initialization failed
		```
	> solution - Please refer to the [guide](https://software.intel.com/en-us/neural-compute-stick/get-started) to set up the environment.


