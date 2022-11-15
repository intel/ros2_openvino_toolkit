# ROS2_GALACTIC_OpenVINO_Toolkit

**NOTE:**
Below steps have been tested on **Ubuntu 20.04**.

## 1. Environment Setup
* Install ROS2 Galactic ([guide](https://docs.ros.org/en/galactic/Installation/Ubuntu-Install-Debians.html))

* Install Intel® OpenVINO™ Toolkit Version: 2022.1 ([guide](https://docs.openvino.ai/2022.1/openvino_docs_install_guides_installing_openvino_linux.html)). 
  * Install from an achive file.([guide](https://www.intel.com/content/www/us/en/developer/tools/openvino-toolkit/download.html))
  Tips: Both runtime and development tool are needed, `pip` is recommended for installing the development tool [guide](https://docs.openvino.ai/latest/openvino_docs_install_guides_install_dev_tools.html).
  * Install from source code.([guide](https://github.com/openvinotoolkit/openvino/wiki/BuildingForLinux))

* Install Intel®  RealSense ™ SDK ([guide](https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md))

## 2. Building and Installation
* Install ROS2_OpenVINO_Toolkit packages
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone https://github.com/intel/ros2_openvino_toolkit -b galactic
git clone https://github.com/intel/ros2_object_msgs
git clone https://github.com/IntelRealSense/realsense-ros.git -b ros2
git clone https://github.com/ros-perception/vision_opencv.git -b galactic
```
* Install dependencies
```
sudo apt-get install ros-galactic-diagnostic-updater
```
* Build package
```
source /opt/ros/galactic/setup.bash 
source <OpenVINO_INSTALL_DIR>/setupvars.sh
cd ~/catkin_ws
colcon build --symlink-install
source ./install/local_setup.bash
```

## 3. Running the Demo
### Install OpenVINO 2022.1 by source code
* See all available models
```
cd ~/openvino/thirdparty/open_model_zoo/tools/model_tools
sudo python3 downloader.py --print_all
```

* Download the optimized Intermediate Representation (IR) of models (execute once), for example:
```
cd ~/openvino/thirdparty/open_model_zoo/tools/model_tools
sudo python3 downloader.py --name face-detection-adas-0001 --output_dir /opt/openvino_toolkit/models/face_detection/output
sudo python3 downloader.py --name age-gender-recognition-retail-0013 --output_dir /opt/openvino_toolkit/models/age-gender-recognition/output
sudo python3 downloader.py --name emotions-recognition-retail-0003 --output_dir /opt/openvino_toolkit/models/emotions-recognition/output
sudo python3 downloader.py --name head-pose-estimation-adas-0001 --output_dir /opt/openvino_toolkit/models/head-pose-estimation/output
sudo python3 downloader.py --name person-detection-retail-0013 --output_dir /opt/openvino_toolkit/models/person-detection/output
sudo python3 downloader.py --name person-reidentification-retail-0277 --output_dir /opt/openvino_toolkit/models/person-reidentification/output
sudo python3 downloader.py --name landmarks-regression-retail-0009 --output_dir /opt/openvino_toolkit/models/landmarks-regression/output
sudo python3 downloader.py --name semantic-segmentation-adas-0001 --output_dir /opt/openvino_toolkit/models/semantic-segmentation/output
sudo python3 downloader.py --name vehicle-license-plate-detection-barrier-0106 --output_dir /opt/openvino_toolkit/models/vehicle-license-plate-detection/output
sudo python3 downloader.py --name vehicle-attributes-recognition-barrier-0039 --output_dir /opt/openvino_toolkit/models/vehicle-attributes-recognition/output
sudo python3 downloader.py --name license-plate-recognition-barrier-0001 --output_dir /opt/openvino_toolkit/models/license-plate-recognition/output
sudo python3 downloader.py --name person-attributes-recognition-crossroad-0230 --output_dir /opt/openvino_toolkit/models/person-attributes/output
```

* Copy the label file to model output_dir (execute once)
  
  **Note** Need to make label dirs if skip steps for set output dirs above.
  ```
  mkdir -p /opt/openvino_toolkit/models/face_detection/output/intel/face-detection-adas-0001/FP32/
  mkdir -p /opt/openvino_toolkit/models/face_detection/output/intel/face-detection-adas-0001/FP16/
  mkdir -p /opt/openvino_toolkit/models/emotions-recognition/output/intel/emotions-recognition-retail-0003/FP32/
  mkdir -p /opt/openvino_toolkit/models/semantic-segmentation/output/FP32/
  mkdir -p /opt/openvino_toolkit/models/semantic-segmentation/output/FP16/
  mkdir -p /opt/openvino_toolkit/models/vehicle-license-plate-detection/output/intel/vehicle-license-plate-detection-barrier-0106/FP32

  ```

```
 sudo cp ~/catkin_ws/src/ros2_openvino_toolkit/data/labels/face_detection/face-detection-adas-0001.labels /opt/openvino_toolkit/models/face_detection/output/intel/face-detection-adas-0001/FP32/
 sudo cp ~/catkin_ws/src/ros2_openvino_toolkit/data/labels/face_detection/face-detection-adas-0001.labels /opt/openvino_toolkit/models/face_detection/output/intel/face-detection-adas-0001/FP16/
 sudo cp ~/catkin_ws/src/ros2_openvino_toolkit/data/labels/emotions-recognition/FP32/emotions-recognition-retail-0003.labels /opt/openvino_toolkit/models/emotions-recognition/output/intel/emotions-recognition-retail-0003/FP32/
 sudo cp ~/catkin_ws/src/ros2_openvino_toolkit/data/labels/object_segmentation/frozen_inference_graph.labels /opt/openvino_toolkit/models/semantic-segmentation/output/FP32/
 sudo cp ~/catkin_ws/src/ros2_openvino_toolkit/data/labels/object_segmentation/frozen_inference_graph.labels /opt/openvino_toolkit/models/semantic-segmentation/output/FP16/
 sudo cp ~/catkin_ws/src/ros2_openvino_toolkit/data/labels/object_detection/vehicle-license-plate-detection-barrier-0106.labels /opt/openvino_toolkit/models/vehicle-license-plate-detection/output/intel/vehicle-license-plate-detection-barrier-0106/FP32
```

* If the model (tensorflow, caffe, MXNet, ONNX, Kaldi) need to be converted to Intermediate Representation (such as the model for object detection):
  * mobilenet-ssd
    ```
    cd ~/openvino/thirdparty/open_model_zoo/tools/model_tools
    sudo python3 downloader.py --name mobilenet-ssd --output_dir /opt/openvino_toolkit/models/object_detection/mobilenet_ssd/output
    cd ~/openvino/tools/mo/openvino/tools/mo
    sudo python3 mo.py --input_model /opt/openvino_toolkit/models/object_detection/mobilenet_ssd/output/public/mobilenet-ssd/mobilenet-ssd.caffemodel --output_dir /opt/openvino_toolkit/models/object_detection/mobilenet_ssd/output
    ```
  * deeplabv3
    ```
    cd ~/openvino/thirdparty/open_model_zoo/tools/model_tools
    sudo python3 downloader.py --name deeplabv3 --output_dir /opt/openvino_toolkit/models/deeplabv3/output
    sudo python3 converter.py --name=deeplabv3 --mo ~/openvino/tools/mo/openvino/tools/mo.py 
    ```

### Install OpenVINO 2022.1 by PIP
* OMZ tools are provided for downloading and converting OMZ models in ov2022.([guide](https://pypi.org/project/openvino-dev/))

* See all available models
```
omz_downloader --print_all
```

* Download the optimized Intermediate Representation (IR) of model (execute once), for example:
```
omz_downloader --name face-detection-adas-0001 --output_dir /opt/openvino_toolkit/models/face_detection/output
omz_downloader --name age-gender-recognition-retail-0013 --output_dir /opt/openvino_toolkit/models/age-gender-recognition/output
omz_downloader --name emotions-recognition-retail-0003 --output_dir /opt/openvino_toolkit/models/emotions-recognition/output
omz_downloader --name head-pose-estimation-adas-0001 --output_dir /opt/openvino_toolkit/models/head-pose-estimation/output
omz_downloader --name person-detection-retail-0013 --output_dir /opt/openvino_toolkit/models/person-detection/output
omz_downloader --name person-reidentification-retail-0277 --output_dir /opt/openvino_toolkit/models/person-reidentification/output
omz_downloader --name landmarks-regression-retail-0009 --output_dir /opt/openvino_toolkit/models/landmarks-regression/output
omz_downloader --name semantic-segmentation-adas-0001 --output_dir /opt/openvino_toolkit/models/semantic-segmentation/output
omz_downloader --name vehicle-license-plate-detection-barrier-0106 --output_dir /opt/openvino_toolkit/models/vehicle-license-plate-detection/output
omz_downloader --name vehicle-attributes-recognition-barrier-0039 --output_dir /opt/openvino_toolkit/models/vehicle-attributes-recognition/output
omz_downloader --name license-plate-recognition-barrier-0001 --output_dir /opt/openvino_toolkit/models/license-plate-recognition/output
omz_downloader --name person-attributes-recognition-crossroad-0230 --output_dir /opt/openvino_toolkit/models/person-attributes/output
```
* Copy label files (execute once)
```
 sudo cp ~/catkin_ws/src/ros2_openvino_toolkit/data/labels/face_detection/face-detection-adas-0001.labels /opt/openvino_toolkit/models/face_detection/output/intel/face-detection-adas-0001/FP32/
 sudo cp ~/catkin_ws/src/ros2_openvino_toolkit/data/labels/face_detection/face-detection-adas-0001.labels /opt/openvino_toolkit/models/face_detection/output/intel/face-detection-adas-0001/FP16/
 sudo cp ~/catkin_ws/src/ros2_openvino_toolkit/data/labels/emotions-recognition/FP32/emotions-recognition-retail-0003.labels /opt/openvino_toolkit/models/emotions-recognition/output/intel/emotions-recognition-retail-0003/FP32/
 sudo cp ~/catkin_ws/src/ros2_openvino_toolkit/data/labels/object_segmentation/frozen_inference_graph.labels /opt/openvino_toolkit/models/semantic-segmentation/output/FP32/
 sudo cp ~/catkin_ws/src/ros2_openvino_toolkit/data/labels/object_segmentation/frozen_inference_graph.labels /opt/openvino_toolkit/models/semantic-segmentation/output/FP16/
 sudo cp ~/catkin_ws/src/ros2_openvino_toolkit/data/labels/object_detection/vehicle-license-plate-detection-barrier-0106.labels /opt/openvino_toolkit/models/vehicle-license-plate-detection/output/intel/vehicle-license-plate-detection-barrier-0106/FP32
```

* If the model (tensorflow, caffe, MXNet, ONNX, Kaldi) need to be converted to intermediate representation (such as the model for object detection):
  * mobilenet-ssd
    ```
    omz_downloader --name mobilenet-ssd --output_dir /opt/openvino_toolkit/models/object_detection/mobilenet_ssd/output
    omz_converter --name mobilenet-ssd -d /opt/openvino_toolkit/models/object_detection/mobilenet_ssd/output -o /opt/openvino_toolkit/models/object_detection/mobilenet_ssd/output/convert
    ```
  * deeplabv3
    ```
    omz_downloader --name deeplabv3 --output_dir /opt/openvino_toolkit/models/deeplabv3/output
    omz_converter --name deeplabv3 -d /opt/openvino_toolkit/models/deeplabv3/output -o /opt/openvino_toolkit/models/deeplabv3/output/convert
    ```

* Please check the parameter configuration in ros2_openvino_toolkit/sample/param/xxxx.yaml before lauching, make sure parameters such as model_path, label_path and input_path are set correctly.

  * run face detection sample code input from StandardCamera.
  ```
  ros2 launch dynamic_vino_sample pipeline_people.launch.py
  ```
  * run person reidentification sample code input from StandardCamera.
  ```
  ros2 launch dynamic_vino_sample pipeline_reidentification.launch.py
  ```
  * run person face reidentification sample code input from RealSenseCamera.
  ```
  ros2 launch dynamic_vino_sample pipeline_face_reidentification.launch.py
  ```
  * run face detection sample code input from Image.
  ```
  ros2 launch dynamic_vino_sample pipeline_image.launch.py
  ```
  * run object segmentation sample code input from RealSenseCameraTopic.
  ```
  ros2 launch dynamic_vino_sample pipeline_segmentation.launch.py
  ```
  * run vehicle detection sample code input from StandardCamera.
  ```
  ros2 launch dynamic_vino_sample pipeline_vehicle_detection.launch.py
  ```
  * run person attributes sample code input from StandardCamera.
  ```
  ros2 launch dynamic_vino_sample pipeline_person_attributes.launch.py
  ```

# More Information
* ROS2 OpenVINO discription writen in Chinese: https://mp.weixin.qq.com/s/BgG3RGauv5pmHzV_hkVAdw

###### *Any security issue should be reported using process at https://01.org/security*

