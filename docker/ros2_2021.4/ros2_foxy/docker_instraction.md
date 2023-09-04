# Run Docker Images For ROS2_OpenVINO_Toolkit

**NOTE:**
Below steps have been tested on **Ubuntu 20.04**.

## 1. Environment Setup
* System with Ubuntu20.04 or 22.04 installed  
* Docker installed ([guide](https://docs.docker.com/engine/install/ubuntu/))
* Realsense Camera inserted
* Dockerfile(docker/ros2_2021.4/ros2_foxy/Dockerfile)
* Converted models(Optional)

## 2. add proxy
```
a. sudo mkdir -p /etc/systemd/system/docker.service.d/
		 
b. sudo vim /etc/systemd/system/docker.service.d/proxy.conf
		 
cat /etc/systemd/system/docker.service.d/proxy.conf
[Service]
Environment="HTTPS_PROXY=http://child-prc.intel.com:913"
		 
c. sudo vim /etc/systemd/system/docker.service.d/http-proxy.conf
root@GNRD:/home/media/GNRD# cat /etc/systemd/system/docker.service.d/http-proxy.conf
[Service]
Environment="HTTP_PROXY=http://child-prc.intel.com:913" "HTTPS_PROXY=http://child-prc.intel.com:913"
		 
sudo systemctl daemon-reload
sudo systemctl restart docker
```
## 3. Build image
```
sudo docker build --build-arg ROS_PRE_INSTALLED_PKG=foxy-desktop --build-arg VERSION=foxy --build-arg "HTTP_PROXY=http://proxy-prc.intel.com:913" -t ros2_foxy_openvino_20230 .
```

## 4. Running the Demos
* Install dependency
```
  xhost +
```
* run docker image
```
  sudo docker images
  sudo docker run -itd --network=host --privileged -p 40080:80  -p 48080:8080 -p 8888:8888 -p 9000:9000 -p 9090:9090  --device /dev/dri -v /dev/:/dev/ -v /tmp/.X11-unix:/tmp/.X11-unix -e DISPLAY=unix:0 -e AMR_TYPE=symg --name ros2_foxy_ov_230 ros2_foxy_openvino_20230 bash
  sudo docker exec -it ros2_foxy_ov_230 bash
  ```
* In Docker Container

* Preparation
```
source /opt/ros/foxy/setup.bash
source install/setup.bash
mkdir -p  /opt/openvino_toolkit/models/convert/public/FP32/yolov8n
sudo ln -s /opt/intel/openvino_2021/deployment_tools/open_model_zoo/tools/downloader /opt/openvino_toolkit/models
sudo chmod 777 -R /opt/openvino_toolkit/models
```

* See all available models
```
cd /opt/intel/openvino_2021/deployment_tools/open_model_zoo/tools/downloader
sudo python3 downloader.py --print_all
```

* Download the optimized Intermediate Representation (IR) of model (execute once), for example:
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
* Before launch, copy label files to the same model path, make sure the model path and label path match the ros_openvino_toolkit/vino_launch/param/xxxx.yaml.Please refer to the quick start document for yaml configuration guidance for detailed configuration guidance.
```
 sudo cp /root/ros2_ws/src/ros2_openvino_toolkit/data/labels/face_detection/face-detection-adas-0001.labels /opt/openvino_toolkit/models/face_detection/output/intel/face-detection-adas-0001/FP32/
 sudo cp /root/ros2_ws/src/ros2_openvino_toolkit/data/labels/face_detection/face-detection-adas-0001.labels /opt/openvino_toolkit/models/face_detection/output/intel/face-detection-adas-0001/FP16/
 sudo cp /root/ros2_ws/src/ros2_openvino_toolkit/data/labels/emotions-recognition/FP32/emotions-recognition-retail-0003.labels /opt/openvino_toolkit/models/emotions-recognition/output/intel/emotions-recognition-retail-0003/FP32/
 sudo cp /root/ros2_ws/src/ros2_openvino_toolkit/data/labels/object_segmentation/frozen_inference_graph.labels /opt/openvino_toolkit/models/semantic-segmentation/output/FP32/
 sudo cp /root/ros2_ws/src/ros2_openvino_toolkit/data/labels/object_segmentation/frozen_inference_graph.labels /opt/openvino_toolkit/models/semantic-segmentation/output/FP16/
 sudo cp /root/ros2_ws/src/ros2_openvino_toolkit/data/labels/object_detection/vehicle-license-plate-detection-barrier-0106.labels /opt/openvino_toolkit/models/vehicle-license-plate-detection/output/intel/vehicle-license-plate-detection-barrier-0106/FP32
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
  * YOLOV8
  ```
  mkdir -p yolov8 && cd yolov8
  pip install ultralytics
  apt install python3.10-venv
  python3 -m venv openvino_env
  source openvino_env/bin/activate
  python -m pip install --upgrade pip
  pip install openvino-dev	
  pip install openvino-dev[extras]
  pip install openvino-dev[tensorflow2,onnx]
  #yolo export model=yolov8n.pt format=openvino
  yolo export model=yolov8n.pt format=onnx opset=10
  mo --input_model yolov8n.onnx --use_legacy_frontend
  cp yolov8* /opt/openvino_toolkit/models/convert/public/FP32/yolov8n/


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

