# ROS2_FOXY_OpenVINO_Toolkit

**NOTE:** 
Below steps have been tested on **Ubuntu 20.04**.

## 1. Environment Setup
* Install ROS2 Foxy (see [here](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html))
* Install Intel® OpenVINO™ Toolkit Version: 2021.3 [guide](https://docs.openvinotoolkit.org/latest/openvino_docs_install_guides_installing_openvino_linux.html)
* Install Intel®  RealSense ™ SDK [guide](https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md)

## 2. Building and Installation
* Build demo code in OpenVINO toolkit
```
 # root is required instead of sudo
 source /opt/intel/<INSTALL_DIR>/bin/setupvars.sh
 cd /opt/intel/<INSTALL_DIR>/deployment_tools/open_model_zoo/demos
 source build_demos.sh
```
* Install ROS2_OpenVINO packages
```
mkdir -p ~/my_ros2_ws/src
cd ~/my_ros2_ws/src
git clone https://github.com/intel/ros2_openvino_toolkit -b dev-ov.2021.3
git clone https://github.com/intel/ros2_object_msgs
git clone https://github.com/intel/ros2_intel_realsense.git -b refactor
```
* Build package
```
source ~/ros2_foxy/install/local_setup.bash
source /opt/intel/<INSTALL_DIR>/bin/setupvars.sh
cd ~/my_ros2_ws/src
colcon build --symlink-install
source ./install/local_setup.bash
```

## 3. Running the Demo
* Preparation
	* Configure the Neural Compute Stick USB Driver (if needed)
```
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

* See all available models
```
cd /opt/intel/<INSTALL_DIR>/deployment_tools/open_model_zoo/tools/downloader
sudo python3 downloader.py --print_all
```

* Download the optimized Intermediate Representation (IR) of model (execute once), for example:
```
cd /opt/intel/<INSTALL_DIR>/deployment_tools/open_model_zoo/tools/downloader
sudo python3 downloader.py --name face-detection-adas-0001 --output_dir /opt/openvino_toolkit/models/face_detection/output
```

* copy label files (execute once)
```
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
```
 sudo python3 downloader.py --name mobilenet-ssd --output_dir /opt/openvino_toolkit/models/object_detection/mobilenet_ssd/caffe/output
 cd /opt/intel/<INSTALL_DIR>/deployment_tools/model_optimizer
 sudo python3 mo.py --input_model /opt/openvino_toolkit/models/object_detection/mobilenet_ssd/caffe/output/public/mobilenet-ssd/mobilenet-ssd.caffemodel --output_dir /opt/openvino_toolkit/models/object_detection/mobilenet_ssd/caffe/output
```

* Before launch, check the parameter configuration in ros2_openvino_toolkit/sample/param/xxxx.yaml, make sure the paramter like model path, label path, inputs are right.
* run face detection sample code input from StandardCamera.
```
ros2 launch dynamic_vino_sample pipeline_people.launch.py
```
* run face detection sample code input from Image.
```
ros2 launch dynamic_vino_sample pipeline_image.launch.py
```
* run object segmentation sample code input from RealSenseCameraTopic.
```
ros2 launch dynamic_vino_sample pipeline_segmentation.launch.py
```
* run object segmentation sample code input from Image.
```
ros2 launch dynamic_vino_sample pipeline_segmentation_image.launch.py
```
* run vehicle detection sample code input from StandardCamera.
```
ros2 launch dynamic_vino_sample pipeline_vehicle_detection.launch.py
```
* run person attributes sample code input from StandardCamera.
```
ros2 launch dynamic_vino_sample pipeline_person_attributes.launch.py
```
* run person reidentification sample code input from StandardCamera.
```
ros2 launch dynamic_vino_sample pipeline_reidentification.launch.py
```

# More Information
* ROS2 OpenVINO discription writen in Chinese: https://mp.weixin.qq.com/s/BgG3RGauv5pmHzV_hkVAdw 

###### *Any security issue should be reported using process at https://01.org/security*
