# ROS2_OpenVINO_Toolkit

**NOTE:**
Below steps have been tested on **Ubuntu 20.04** and **Ubuntu 22.04**.
Supported ROS2 versions include foxy, galactic and humble.

## 1. Environment Setup
For ROS2 foxy and galactic on ubuntu 20.04:
  * Install ROS2.</br>
  Refer to: [ROS_foxy_install_guide](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html) & [ROS_galactic_install_guide](https://docs.ros.org/en/galactic/Installation/Ubuntu-Install-Debians.html)

  * Install Intel® OpenVINO™ Toolkit Version: 2022.3.</br>
  Refer to: [OpenVINO_install_guide](https://docs.openvino.ai/2022.3/openvino_docs_install_guides_installing_openvino_apt.html#doxid-openvino-docs-install-guides-installing-openvino-apt)
    * Install from an archive file. Both runtime and development tool are needed, `pip` is recommended for installing the development tool.</br>
    Refer to: [OpenVINO_devtool_install_guide](https://www.intel.com/content/www/us/en/developer/tools/openvino-toolkit/download.html)

  * Install Intel® RealSense™ SDK.</br>
  Refer to: [RealSense_install_guide](https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md)

For ROS2 humble on ubuntu 22.04:
  * Install ROS2.</br>
  Refer to: [ROS_humble_install_guide](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)

  * Install Intel® OpenVINO™ Toolkit Latest Version by Source.</br>
  Refer to: [OpenVINO_install_guide](https://github.com/openvinotoolkit/openvino/wiki/BuildingCode)

  * Install Intel®  RealSense™ SDK by Source.</br>
  Refer to: [RealSense_install_guide](https://github.com/IntelRealSense/librealsense/blob/master/doc/installation.md)

## 2. Building and Installation
* Install ROS2_OpenVINO_Toolkit packages
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone https://github.com/intel/ros2_openvino_toolkit -b ros2
git clone https://github.com/intel/ros2_object_msgs
git clone https://github.com/IntelRealSense/realsense-ros.git -b ros2-development
git clone https://github.com/ros-perception/vision_opencv.git -b <ROS2_VERSION>
```
* Install dependencies
```
sudo apt-get install ros-<ROS2_VERSION>-diagnostic-updater
sudo apt install python3-colcon-common-extensions
```
* Build package
```
source /opt/ros/<ROS2_VERSION>/setup.bash 
source <OpenVINO_INSTALL_DIR>/setupvars.sh
cd ~/catkin_ws
colcon build --symlink-install
source ./install/local_setup.bash
```

## 3. Running the Demo
### Install OpenVINO 2022.3 by PIP
OMZ tools are provided for downloading and converting models of open_model_zoo in ov2022.</br>
Refer to: [OMZtool_guide](https://pypi.org/project/openvino-dev/)

* See all available models
```
omz_downloader --print_all
```

* Download the optimized Intermediate Representation (IR) of model (execute once), for example:
```
cd ~/catkin_ws/src/ros2_openvino_toolkit/data/model_list
omz_downloader --list download_model.lst -o /opt/openvino_toolkit/models/
```

* If the model (tensorflow, caffe, MXNet, ONNX, Kaldi) need to be converted to intermediate representation (such as the model for object detection):
```
cd ~/catkin_ws/src/ros2_openvino_toolkit/data/model_list
omz_converter --list convert_model.lst -d /opt/openvino_toolkit/models/ -o /opt/openvino_toolkit/models/convert
```
### Install OpenVINO 2022.3 by source code
* See all available models
```
cd ~/openvino/thirdparty/open_model_zoo/tools/model_tools
sudo python3 downloader.py --print_all
```

* Download the optimized Intermediate Representation (IR) of models (execute once), for example:
```
cd ~/openvino/thirdparty/open_model_zoo/tools/model_tools
sudo python3 downloader.py --list download_model.lst -o /opt/openvino_toolkit/models/
```

* If the model (tensorflow, caffe, MXNet, ONNX, Kaldi) need to be converted to Intermediate Representation (such as the model for object detection):
```
cd ~/openvino/thirdparty/open_model_zoo/tools/model_tools
sudo python3 converter.py --list convert_model.lst -d /opt/openvino_toolkit/models/ -o /opt/openvino_toolkit/models/convert
```

* Copy label files (execute once)
**Note**:Need to make label_dirs if skip steps for set output_dirs above.
```
sudo cp ~/catkin_ws/src/ros2_openvino_toolkit/data/labels/face_detection/face-detection-adas-0001.labels /opt/openvino_toolkit/models/intel/face-detection-adas-0001/FP32/
sudo cp ~/catkin_ws/src/ros2_openvino_toolkit/data/labels/face_detection/face-detection-adas-0001.labels /opt/openvino_toolkit/models/intel/face-detection-adas-0001/FP16/
sudo cp ~/catkin_ws/src/ros2_openvino_toolkit/data/labels/emotions-recognition/FP32/emotions-recognition-retail-0003.labels /opt/openvino_toolkit/models/intel/emotions-recognition-retail-0003/FP32/
sudo cp ~/catkin_ws/src/ros2_openvino_toolkit/data/labels/object_segmentation/frozen_inference_graph.labels /opt/openvino_toolkit/models/intel/semantic-segmentation-adas-0001/FP32/
sudo cp ~/catkin_ws/src/ros2_openvino_toolkit/data/labels/object_segmentation/frozen_inference_graph.labels /opt/openvino_toolkit/models/intel/semantic-segmentation-adas-0001/FP16/
sudo cp ~/catkin_ws/src/ros2_openvino_toolkit/data/labels/object_detection/vehicle-license-plate-detection-barrier-0106.labels /opt/openvino_toolkit/models/intel/vehicle-license-plate-detection-barrier-0106/FP32
```

* Check the parameter configuration in ros2_openvino_toolkit/sample/param/xxxx.yaml before launching, make sure parameters such as model_path, label_path and input_path are set correctly. Please refer to the quick start document for [yaml configuration guidance](./yaml_configuration_guide.md) for detailed configuration guidance.
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
* ROS2 OpenVINO description written in Chinese: https://mp.weixin.qq.com/s/BgG3RGauv5pmHzV_hkVAdw

###### *Any security issue should be reported using process at https://01.org/security*

