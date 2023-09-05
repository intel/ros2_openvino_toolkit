# ROS2_OpenVINO_Toolkit

**NOTE:**
Below steps have been tested on **Ubuntu 20.04** and **Ubuntu 22.04**.
Supported ROS2 versions include foxy,galactic and humble.

## 1. Environment Setup
For ROS2 foxy and galactic and humble on ubuntu 20.04/ubuntu22.04:
  * Install ROS2.</br>
  Refer to: [ROS_foxy_install_guide](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html) & [ROS_galactic_install_guide](https://docs.ros.org/en/galactic/Installation/Ubuntu-Install-Debians.html) & [ROS_humble_install_guide](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)

`
Note that it is recommended to install ROS2-Humble for Ubuntu22.04, and ROS2-Foxy for Ubuntu 20.04.
`

  * Install Intel® OpenVINO™ Toolkit Version: 2023.0.</br>
  Refer to: [OpenVINO_install_guide](https://docs.openvino.ai/2023.0/openvino_docs_install_guides_installing_openvino_apt.html#doxid-openvino-docs-install-guides-installing-openvino-apt)

  * Install Intel® RealSense™ SDK.</br>
  Refer to: [RealSense_install_guide](https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md)


## 2. Building and Installation
* Install ROS2_OpenVINO_Toolkit packages
```shell
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone https://github.com/intel/ros2_openvino_toolkit -b ros2
git clone https://github.com/intel/ros2_object_msgs
```
* Install dependencies
```shell
sudo apt-get install ros-<ROS2_VERSION>-diagnostic-updater
sudo apt install python3-colcon-common-extensions
```
* Build package
```bash
source /opt/ros/<ROS2_VERSION>/setup.bash 
source <OpenVINO_INSTALL_DIR>/setupvars.sh
cd ~/catkin_ws
colcon build --symlink-install
source ./install/local_setup.bash
```

## 3 Prepare Models (Execute ONCE)
OMZ tools are provided for downloading and converting models of open_model_zoo.</br>

*  Refer [OMZtool_guide](https://pypi.org/project/openvino-dev/) to prepare Openvino-Development PIP oenvironment
```bash
python3 -m venv openvino_env
source openvino_env/bin/activate
python -m pip install --upgrade pip
pip install openvino-dev[extras]
pip install openvino-dev[tensorflow2,onnx]
```

* [Optional] See all available models
```bash
omz_downloader --print_all
```

* Download the optimized Intermediate Representation (IR) of model.
```bash
cd ~/catkin_ws/src/ros2_openvino_toolkit/data/model_list
omz_downloader --list download_model.lst -o /opt/openvino_toolkit/models/
```

* Convert the public models to OpenVINO mode (intermediate representation):
```bash
cd ~/catkin_ws/src/ros2_openvino_toolkit/data/model_list
omz_converter --list convert_model.lst -d /opt/openvino_toolkit/models/ -o /opt/openvino_toolkit/models/convert
```
Especially for Yolov8 models, please execute below command to get converted models:
```sh
mkdir -p ~/yolov8 && cd ~/yolov8
pip install ultralytics
yolo export model=yolov8n.pt format=onnx opset=10
mo --input_model yolov8n.onnx --use_legacy_frontend
cd yolov8n_openvino_model
mkdir -p  /opt/openvino_toolkit/models/convert/public/FP32/yolov8n
sudo cp yolov8* /opt/openvino_toolkit/models/convert/public/FP32/yolov8n
```
* Copy label files
**Note**:Need to make label_dirs if skip steps for set output_dirs above.
```shell
sudo mkdir -p /opt/openvino_toolkit/models/intel/face-detection-adas-0001/FP32/
sudo cp ~/catkin_ws/src/ros2_openvino_toolkit/data/labels/face_detection/face-detection-adas-0001.labels /opt/openvino_toolkit/models/intel/face-detection-adas-0001/FP32/
sudo mkdir -p /opt/openvino_toolkit/models/intel/face-detection-adas-0001/FP16/
sudo cp ~/catkin_ws/src/ros2_openvino_toolkit/data/labels/face_detection/face-detection-adas-0001.labels /opt/openvino_toolkit/models/intel/face-detection-adas-0001/FP16/
sudo mkdir -p /opt/openvino_toolkit/models/intel/emotions-recognition-retail-0003/FP32/
sudo cp ~/catkin_ws/src/ros2_openvino_toolkit/data/labels/emotions-recognition/FP32/emotions-recognition-retail-0003.labels /opt/openvino_toolkit/models/intel/emotions-recognition-retail-0003/FP32/
sudo mkdir -p /opt/openvino_toolkit/models/intel/semantic-segmentation-adas-0001/FP32/
sudo cp ~/catkin_ws/src/ros2_openvino_toolkit/data/labels/object_segmentation/frozen_inference_graph.labels /opt/openvino_toolkit/models/intel/semantic-segmentation-adas-0001/FP32/
sudo mkdir -p /opt/openvino_toolkit/models/intel/semantic-segmentation-adas-0001/FP16/
sudo cp ~/catkin_ws/src/ros2_openvino_toolkit/data/labels/object_segmentation/frozen_inference_graph.labels /opt/openvino_toolkit/models/intel/semantic-segmentation-adas-0001/FP16/
sudo mkdir -p /opt/openvino_toolkit/models/intel/vehicle-license-plate-detection-barrier-0106/FP32
sudo cp ~/catkin_ws/src/ros2_openvino_toolkit/data/labels/object_detection/vehicle-license-plate-detection-barrier-0106.labels /opt/openvino_toolkit/models/intel/vehicle-license-plate-detection-barrier-0106/FP32
```

## 4. Launching OpenVINO Samples
**NOTE:** Check the parameter configuration in ros2_openvino_toolkit/sample/param/xxxx.yaml before lauching, make sure parameters such as model_path, label_path and input_path are set correctly. Please refer to the quick start document for [yaml configuration guidance](./yaml_configuration_guide.md) for detailed configuration guidance.

  * run face detection sample code input from StandardCamera.
  ```
  ros2 launch openvino_node pipeline_people.launch.py
  ```
  * run person reidentification sample code input from StandardCamera.
  ```
  ros2 launch openvino_node pipeline_reidentification.launch.py
  ```
  * run face detection sample code input from Image.
  ```
  ros2 launch openvino_node pipeline_image.launch.py
  ```
  * run object segmentation sample code input from RealSenseCameraTopic.
  ```
  ros2 launch openvino_node pipeline_segmentation.launch.py
  ```
  * run vehicle detection sample code input from StandardCamera.
  ```
  ros2 launch openvino_node pipeline_vehicle_detection.launch.py
  ```
  * run person attributes sample code input from StandardCamera.
  ```
  ros2 launch openvino_node pipeline_person_attributes.launch.py
  ```

# More Information
* ROS2 OpenVINO discription writen in Chinese: https://mp.weixin.qq.com/s/BgG3RGauv5pmHzV_hkVAdw

###### *Any security issue should be reported using process at https://01.org/security*

