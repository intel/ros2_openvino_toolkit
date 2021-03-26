# ROS2_FOXY_OpenVINO_Toolkit

**NOTE:** 
Below steps have been tested on **Ubuntu 20.04**.

## 1. Environment Setup
* Install ROS2 Foxy [(guide)](https://index.ros.org/doc/ros2/Installation/Foxy/Linux-Development-Setup/)
* Install OpenVINO™ Toolkit Version: 2020.3 [(guide)](https://software.intel.com/content/www/us/en/develop/tools/openvino-toolkit/download.html)
**Note:** Please use root privileges to run the installer when installing the core components.
* Install Intel® RealSense™ SDK 2.0 [for Linux Distribution](https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md)

## 2. Building and Installation

* Install ROS2_OpenVINO and dependent packages
```bash
mkdir -p ~/my_ros2_ws/src
cd ~/my_ros2_ws/src
git clone https://github.com/intel/ros2_openvino_toolkit
git clone https://github.com/intel/ros2_object_msgs
```
* Build package
```bash
source ~/ros2_foxy/install/local_setup.bash
source /opt/intel/<INSTALL_DIR>/bin/setupvars.sh
cd ~/my_ros2_ws/src
colcon build --symlink-install
source ./install/local_setup.bash
```

## 4. Running the Demo
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
	* Download the optimized Intermediate Representation (IR) of model (execute once)

	```bash
	cd /opt/intel/<INSTALL_DIR>/deployment_tools/open_model_zoo/tools/downloader
	sudo python3 downloader.py --name face-detection-adas-0001 --output_dir /opt/openvino_toolkit/models/face_detection/output
	sudo python3 downloader.py --name age-gender-recognition-retail-0013 --output_dir /opt/openvino_toolkit/models/age-gender-recognition/output
	sudo python3 downloader.py --name emotions-recognition-retail-0003 --output_dir /opt/openvino_toolkit/models/emotions-recognition/output
	sudo python3 downloader.py --name head-pose-estimation-adas-0001 --output_dir /opt/openvino_toolkit/models/head-pose-estimation/output
	sudo python3 downloader.py --name person-detection-retail-0013 --output_dir /opt/openvino_toolkit/models/person-detection/output
	sudo python3 downloader.py --name person-reidentification-retail-0031 --output_dir /opt/openvino_toolkit/models/person-reidentification/output
	sudo python3 downloader.py --name vehicle-license-plate-detection-barrier-0106 --output_dir /opt/openvino_toolkit/models/vehicle-license-plate-detection/output
	sudo python3 downloader.py --name vehicle-attributes-recognition-barrier-0039 --output_dir /opt/openvino_toolkit/models/vehicle-attributes-recongnition/output
	sudo python3 downloader.py --name license-plate-recognition-barrier-0001 --output_dir /opt/openvino_toolkit/models/license-plate-recognition/output
	sudo python3 downloader.py --name road-segmentation-adas-0001 --output_dir /opt/openvino_toolkit/models/road-segmentation/output
	sudo python3 downloader.py --name person-attributes-recognition-crossroad-0230 --output_dir /opt/openvino_toolkit/models/person-attributes/output
	```
	* copy label files (execute once)
	```bash
	sudo cp ~/my_ros2_ws/src/ros2_openvino_toolkit/data/labels/emotions-recognition/FP32/emotions-recognition-retail-0003.labels /opt/openvino_toolkit/models/emotions-recognition/output/intel/emotions-recognition-retail-0003/FP32/
	sudo cp ~/my_ros2_ws/src/ros2_openvino_toolkit/data/labels/face_detection/face-detection-adas-0001.labels /opt/openvino_toolkit/models/face_detection/output/intel/face-detection-adas-0001/FP32/
	sudo cp ~/my_ros2_ws/src/ros2_openvino_toolkit/data/labels/face_detection/face-detection-adas-0001.labels /opt/openvino_toolkit/models/face_detection/output/intel/face-detection-adas-0001/FP16/
	sudo cp ~/my_ros2_ws/src/ros2_openvino_toolkit/data/labels/object_segmentation/frozen_inference_graph.labels /opt/openvino_toolkit/models/semantic-segmentation/output/FP32/
	sudo cp ~/my_ros2_ws/src/ros2_openvino_toolkit/data/labels/object_segmentation/frozen_inference_graph.labels /opt/openvino_toolkit/models/semantic-segmentation/output/FP16/
	sudo cp ~/my_ros2_ws/src/ros2_openvino_toolkit/data/labels/object_detection/vehicle-license-plate-detection-barrier-0106.labels /opt/openvino_toolkit/models/vehicle-license-plate-detection/output/intel/vehicle-license-plate-detection-barrier-0106/FP32
	sudo cp ~/my_ros2_ws/src/ros2_openvino_toolkit/data/labels/face_detection/face-detection-adas-0001.labels /opt/openvino_toolkit/models/face_detection/output/intel/face-detection-adas-0001/FP32/
	sudo cp ~/my_ros2_ws/src/ros2_openvino_toolkit/data/labels/face_detection/face-detection-adas-0001.labels /opt/openvino_toolkit/models/face_detection/output/intel/face-detection-adas-0001/FP16/
	```
	* If the model (tensorflow, caffe, MXNet, ONNX, Kaldi)need to be converted to intermediate representation (For example the model for object detection)
	 ```bash
	 sudo python3 downloader.py --name mobilenet-ssd --output_dir /opt/openvino_toolkit/models/object_detection/mobilenet_ssd/caffe/output
	 cd /opt/intel/<INSTALL_DIR>/deployment_tools/model_optimizer
	 sudo python3 mo.py --input_model /opt/openvino_toolkit/models/object_detection/mobilenet_ssd/caffe/output/public/mobilenet-ssd/mobilenet-ssd.caffemodel --output_dir /opt/openvino_toolkit/models/object_detection/mobilenet_ssd/caffe/output
	 ```
	 * Before launch, check the parameter configuration in ros2_openvino_toolkit/sample/param/xxxx.yaml, make sure the paramter like model path, label path, inputs are right.

* run face detection sample code input from StandardCamera. 
```bash
ros2 launch dynamic_vino_sample pipeline_people.launch.py
```
* run face detection sample code input from Image. 
```bash
ros2 launch dynamic_vino_sample pipeline_image.launch.py
```
* run object segmentation sample code input from RealSenseCameraTopic.
```bash
ros2 launch dynamic_vino_sample pipeline_segmentation.launch.py
```
* run object segmentation sample code input from Image.
```bash
ros2 launch dynamic_vino_sample pipeline_segmentation_image.launch.py
```
* run vehicle detection sample code input from StandardCamera. 
```bash
ros2 launch dynamic_vino_sample pipeline_vehicle_detection.launch.py
```
* run person attributes sample code input from StandardCamera.
```bash
ros2 launch dynamic_vino_sample pipeline_person_attributes.launch.py
```
* run person reidentification sample code input from StandardCamera.
```bash
ros2 launch dynamic_vino_sample pipeline_reidentification.launch.py
```



