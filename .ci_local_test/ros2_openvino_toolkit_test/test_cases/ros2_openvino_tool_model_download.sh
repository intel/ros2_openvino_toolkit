#!/bin/bash

mkdir -p /opt/openvino_toolkit/models
#apt install -y python-pip
apt install -y python3.8-venv
cd ~ && python3 -m venv openvino_env && source openvino_env/bin/activate
python -m pip install --upgrade pip
pip install openvino-dev[tensorflow2,onnx]==2022.3


#Download the optimized Intermediate Representation (IR) of model (execute once)
cd ~/catkin_ws/src/ros2_openvino_toolkit/data/model_list && omz_downloader --list download_model.lst -o /opt/openvino_toolkit/models/

cd ~/catkin_ws/src/ros2_openvino_toolkit/data/model_list && omz_converter --list convert_model.lst -d /opt/openvino_toolkit/models/ -o /opt/openvino_toolkit/models/convert


#Copy label files (execute once)
cp ~/catkin_ws/src/ros2_openvino_toolkit/data/labels/face_detection/face-detection-adas-0001.labels /opt/openvino_toolkit/models/intel/face-detection-adas-0001/FP32/
cp ~/catkin_ws/src/ros2_openvino_toolkit/data/labels/face_detection/face-detection-adas-0001.labels /opt/openvino_toolkit/models/intel/face-detection-adas-0001/FP16/
cp ~/catkin_ws/src/ros2_openvino_toolkit/data/labels/emotions-recognition/FP32/emotions-recognition-retail-0003.labels /opt/openvino_toolkit/models/intel/emotions-recognition-retail-0003/FP32/
cp ~/catkin_ws/src/ros2_openvino_toolkit/data/labels/object_segmentation/frozen_inference_graph.labels /opt/openvino_toolkit/models/intel/semantic-segmentation-adas-0001/FP32/
cp ~/catkin_ws/src/ros2_openvino_toolkit/data/labels/object_segmentation/frozen_inference_graph.labels /opt/openvino_toolkit/models/intel/semantic-segmentation-adas-0001/FP16/
cp ~/catkin_ws/src/ros2_openvino_toolkit/data/labels/object_detection/vehicle-license-plate-detection-barrier-0106.labels /opt/openvino_toolkit/models/intel/vehicle-license-plate-detection-barrier-0106/FP32

mkdir -p /opt/openvino_toolkit/models/public/mask_rcnn_inception_resnet_v2_atrous_coco/FP16/
cp /opt/openvino_toolkit/models/convert/public/mask_rcnn_inception_resnet_v2_atrous_coco/FP16/* /opt/openvino_toolkit/models/public/mask_rcnn_inception_resnet_v2_atrous_coco/FP16/

cd /root/test_cases/ && ./yolov5_model_download.sh

