# Run Docker Images For ROS2_OpenVINO_Toolkit

**NOTE:**
Below steps have been tested on **Ubuntu 20.04**.

## 1. Environment Setup
* Install docker ([guide](https://docs.docker.com/engine/install/ubuntu/))

## 2. Build docker image by dockerfile
```
cd ~/ros2_openvino_toolkit/docker/Dockerfile
vi ~/ros2_openvino_toolkit/docker/Dockerfile
docker build --build-arg ROS_VERSION=<EXPECT_ROS_BASE_IMAGE> --build-arg VERSION=<EXPECT_ROS_VERSION> --build-arg "HTTP_PROXY=set_your_proxy" -t ros2_openvino_202201 .
```
For example:
* Build image for ros_galactic
```
cd ~/ros2_openvino_toolkit/docker/Dockerfile
vi ~/ros2_openvino_toolkit/docker/Dockerfile
docker build --build-arg ROS_VERSION=galactic-desktop --build-arg VERSION=galactic --build-arg "HTTP_PROXY=set_your_proxy" -t ros2_galactic_openvino_202201 .
```
* Build image for ros_foxy
```
cd ~/ros2_openvino_toolkit/docker/Dockerfile
vi ~/ros2_openvino_toolkit/docker/Dockerfile
docker build --build-arg ROS_VERSION=foxy-desktop --build-arg VERSION=foxy --build-arg "HTTP_PROXY=set_your_proxy" -t ros2_foxy_openvino_202201 .
```

## 3. Download and load docker image
* Download docker image
```
 # ros2_openvino_202201 for demo
 cd ~/Downloads/
 wget <DOCKER_IMAGE_PATH>
```
* Load docker image
```
cd ~/Downloads/
docker load -i <DOCKER_IMAGE>
docker images
/// (show <DOCKER_IMAGE> in the list)
```

## 4. Running the Demos
* Install dependency
```
  sudo apt install x11-xserver-utils
  xhost +
```
* run docker image
```
  docker images
  docker run -itd  -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix -v /dev:/dev  --privileged=true --name <your_image_name> <IMAGE_ID>
```
* In Docker Container

* Preparation
```
source /opt/intel/openvino_2022/setupvars.sh
source /opt/ros/<ROS_VERSION>/setup.bash
cd ~/catkin_ws
source ./install/local_setup.bash
```

* See all available models
OMZ tools are provided for downloading and converting OMZ models in ov2022.([guide](https://pypi.org/project/openvino-dev/))

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
**Note**:Need to make label dirs if skip steps for set output dirs above.
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

* Before launch, check the parameter configuration in ros2_openvino_toolkit/sample/param/xxxx.yaml, make sure the paramter like model path, label path, inputs are right.
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
* ROS2 OpenVINO discription writen in Chinese: https://mp.weixin.qq.com/s/BgG3RGauv5pmHzV_hkVAdw

###### *Any security issue should be reported using process at https://01.org/security*

