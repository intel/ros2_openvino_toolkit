# ROS2_OpenVINO_Toolkit

**NOTE:** 
Below steps have been tested on **Ubuntu 18.04**.

## 1. Environment Setup
* Install ROS2 Dashing [(guide)](https://docs.ros.org/en/dashing/Installation/Ubuntu-Install-Debians.html)
* Install OpenVINO™ Toolkit Version: 2021.4 [(guide)](https://docs.openvino.ai/2021.4/openvino_docs_install_guides_installing_openvino_linux.html)
* Install Intel® RealSense™ SDK (https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md).

## 2. Building and Installation
* Install ROS2_OpenVINO packages
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone https://github.com/intel/ros2_openvino_toolkit -b dashing
git clone https://github.com/intel/ros2_object_msgs
git clone https://github.com/IntelRealSense/realsense-ros.git -b ros2
git clone https://github.com/ros-perception/vision_opencv -b ros2
```
* Install dependencies
```
sudo apt-get install ros-dashing-diagnostic-updater
```
* Build package
```
source /opt/ros/dashing/setup.bash
source /opt/intel/openvino_2021/bin/setupvars.sh
cd ~/catkin_ws
colcon build --symlink-install
source ./install/local_setup.bash
```

## 3. Running the Demo
* Preparation
```
source /opt/intel/openvino_2021/bin/setupvars.sh
sudo mkdir -p /opt/openvino_toolkit
sudo ln -s /opt/intel/openvino_2021/deployment_tools/open_model_zoo/tools/downloader /opt/openvino_toolkit/models
sudo chmod 777 -R /opt/openvino_toolkit/models
```

* See all available models
```
cd /opt/intel/openvino_2021/deployment_tools/open_model_zoo/tools/downloader
sudo python3 downloader.py --print_all
```

* Download the optimized Intermediate Representation (IR) of model (execute once)
```
cd /opt/intel/openvino_2021/deployment_tools/open_model_zoo/tools/downloader
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

* copy label files (execute once)
* Before launch, copy label files to the same model path, make sure the model path and label path match the ros_openvino_toolkit/vino_launch/param/xxxx.yaml.
```
sudo cp ~/catkin_ws/src/ros2_openvino_toolkit/data/labels/face_detection/face-detection-adas-0001.labels /opt/openvino_toolkit/models/face_detection/output/intel/face-detection-adas-0001/FP32/
sudo cp ~/catkin_ws/src/ros2_openvino_toolkit/data/labels/face_detection/face-detection-adas-0001.labels /opt/openvino_toolkit/models/face_detection/output/intel/face-detection-adas-0001/FP16/
sudo cp ~/catkin_ws/src/ros2_openvino_toolkit/data/labels/emotions-recognition/FP32/emotions-recognition-retail-0003.labels /opt/openvino_toolkit/models/emotions-recognition/output/intel/emotions-recognition-retail-0003/FP32/
sudo cp ~/catkin_ws/src/ros2_openvino_toolkit/data/labels/object_segmentation/frozen_inference_graph.labels /opt/openvino_toolkit/models/semantic-segmentation/output/FP32/
sudo cp ~/catkin_ws/src/ros2_openvino_toolkit/data/labels/object_segmentation/frozen_inference_graph.labels /opt/openvino_toolkit/models/semantic-segmentation/output/FP16/
sudo cp ~/catkin_ws/src/ros2_openvino_toolkit/data/labels/object_detection/vehicle-license-plate-detection-barrier-0106.labels /opt/openvino_toolkit/models/vehicle-license-plate-detection/output/intel/vehicle-license-plate-detection-barrier-0106/FP32	
```

* If the model (tensorflow, caffe, MXNet, ONNX, Kaldi)need to be converted to intermediate representation (For example the model for object detection)
* ssd_mobilenet_v2_coco
  ```
  cd /opt/openvino_toolkit/models/
  sudo python3 downloader/downloader.py --name ssd_mobilenet_v2_coco
  sudo python3 /opt/intel/openvino_2021/deployment_tools/open_model_zoo/tools/downloader/converter.py --name=ssd_mobilenet_v2_coco --mo /opt/intel/openvino_2021/deployment_tools/model_optimizer/mo.py
  ```
  * deeplabv3
  ```
  cd /opt/openvino_toolkit/models/
  sudo python3 downloader/downloader.py --name deeplabv3
  sudo python3 /opt/intel/openvino_2021/deployment_tools/open_model_zoo/tools/downloader/converter.py --name=deeplabv3 --mo /opt/intel/openvino_2021/deployment_tools/model_optimizer/mo.py
  ```
  * YOLOV2
  ```
  cd /opt/openvino_toolkit/models/
  sudo python3 downloader/downloader.py --name yolo-v2-tf
  sudo python3 /opt/intel/openvino_2021/deployment_tools/open_model_zoo/tools/downloader/converter.py --name=yolo-v2-tf --mo /opt/intel/openvino_2021/deployment_tools/model_optimizer/mo.py
  ```

* Before launch, check the parameter configuration in ros2_openvino_toolkit/sample/param/xxxx.yaml, make sure the parameters like model path, label path, inputs are right.
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
  * run object segmentation sample code input from RealSenseCamera.
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

# More Information
* ROS2 OpenVINO description written in Chinese: https://mp.weixin.qq.com/s/BgG3RGauv5pmHzV_hkVAdw

###### *Any security issue should be reported using process at https://01.org/security*




