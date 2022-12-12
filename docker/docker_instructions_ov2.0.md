# Run Docker Images For ROS2_OpenVINO_Toolkit

**NOTE:**
Below steps have been tested on **Ubuntu 18.04**.
Supported ROS2 versions: dashing.

## 1. Environment Setup
* Install docker ([guide](https://docs.docker.com/engine/install/ubuntu/))

## 2. Build docker image by dockerfile
```
cd ~/ros2_openvino_toolkit/docker/Dockerfile
vi ~/ros2_openvino_toolkit/docker/Dockerfile
docker build -t ros2_openvino_dashing_202201 .
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
// (show <DOCKER_IMAGE> in the list)
```

## 4. Running the Demos
* Install dependency
```
  sudo apt install x11-xserver-utils
  xhost +
```
* Run docker image
```
  docker images
  docker run -itd  -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix -v /dev:/dev  --privileged=true --name <your_image_name> <IMAGE_ID>
```
* In Docker Container

* Preparation
```
source /opt/intel/openvino_2022/setupvars.sh
source /opt/ros/dashing/setup.bash
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
cd ~/catkin_ws/src/ros2_openvino_toolkit/data/model_list
omz_downloader --list download_model.lst -o /opt/openvino_toolkit/models/
```

* If the model (tensorflow, caffe, MXNet, ONNX, Kaldi) need to be converted to intermediate representation (such as the model for object detection):
```
cd ~/catkin_ws/src/ros2_openvino_toolkit/data/model_list
omz_converter --list convert_model.lst -d /opt/openvino_toolkit/models/ -o /opt/openvino_toolkit/models/convert
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

