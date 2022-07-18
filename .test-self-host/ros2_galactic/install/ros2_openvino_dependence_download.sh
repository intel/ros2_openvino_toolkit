#!/bin/bash

export download_path="/opt/intel/openvino_2021/deployment_tools/open_model_zoo/tools/downloader"
export model_optimizer="/opt/intel/openvino_2021/deployment_tools/model_optimizer"


apt update &&  apt install python3-defusedxml
pip3 install tensorflow==2.4.1 
#See all available models
cd $download_path && python3 downloader.py --print_all

# Download the optimized Intermediate Representation (IR) of model (execute once), for example:
cd /opt/intel/openvino_2021/deployment_tools/open_model_zoo/tools/downloader
cd $download_path &&  python3 downloader.py --name face-detection-adas-0001 --output_dir /opt/openvino_toolkit/models/face_detection/output
cd $download_path && python3 downloader.py --name age-gender-recognition-retail-0013 --output_dir /opt/openvino_toolkit/models/age-gender-recognition/output
cd $download_path && python3 downloader.py --name emotions-recognition-retail-0003 --output_dir /opt/openvino_toolkit/models/emotions-recognition/output
cd $download_path && python3 downloader.py --name head-pose-estimation-adas-0001 --output_dir /opt/openvino_toolkit/models/head-pose-estimation/output
cd $download_path && python3 downloader.py --name person-detection-retail-0013 --output_dir /opt/openvino_toolkit/models/person-detection/output
cd $download_path && python3 downloader.py --name person-reidentification-retail-0277 --output_dir /opt/openvino_toolkit/models/person-reidentification/output
cd $download_path && python3 downloader.py --name landmarks-regression-retail-0009 --output_dir /opt/openvino_toolkit/models/landmarks-regression/output
cd $download_path && python3 downloader.py --name semantic-segmentation-adas-0001 --output_dir /opt/openvino_toolkit/models/semantic-segmentation/output
cd $download_path && python3 downloader.py --name vehicle-license-plate-detection-barrier-0106 --output_dir /opt/openvino_toolkit/models/vehicle-license-plate-detection/output
cd $download_path && python3 downloader.py --name vehicle-attributes-recognition-barrier-0039 --output_dir /opt/openvino_toolkit/models/vehicle-attributes-recognition/output
cd $download_path && python3 downloader.py --name license-plate-recognition-barrier-0001 --output_dir /opt/openvino_toolkit/models/license-plate-recognition/output
cd $download_path && python3 downloader.py --name person-attributes-recognition-crossroad-0230 --output_dir /opt/openvino_toolkit/models/person-attributes/output

# copy label files (execute once)
mkdir /opt/openvino_toolkit/models/semantic-segmentation/output/FP32
mkdir /opt/openvino_toolkit/models/semantic-segmentation/output/FP16
cp ~/catkin_ws/src/ros2_openvino_toolkit/data/labels/face_detection/face-detection-adas-0001.labels /opt/openvino_toolkit/models/face_detection/output/intel/face-detection-adas-0001/FP16/
cp ~/catkin_ws/src/ros2_openvino_toolkit/data/labels/emotions-recognition/FP32/emotions-recognition-retail-0003.labels /opt/openvino_toolkit/models/emotions-recognition/output/intel/emotions-recognition-retail-0003/FP32/
cp ~/catkin_ws/src/ros2_openvino_toolkit/data/labels/object_segmentation/frozen_inference_graph.labels /opt/openvino_toolkit/models/semantic-segmentation/output/FP32/
cp ~/catkin_ws/src/ros2_openvino_toolkit/data/labels/object_segmentation/frozen_inference_graph.labels /opt/openvino_toolkit/models/semantic-segmentation/output/FP16/
cp ~/catkin_ws/src/ros2_openvino_toolkit/data/labels/object_detection/vehicle-license-plate-detection-barrier-0106.labels /opt/openvino_toolkit/models/vehicle-license-plate-detection/output/intel/vehicle-license-plate-detection-barrier-0106/FP32

# mobilenet-ssd

cd $download_path && python3 downloader.py --name mobilenet-ssd --output_dir /opt/openvino_toolkit/models/object_detection/mobilenet_ssd/caffe/output

cp -f /root/output/mobilenet-ssd.bin /opt/openvino_toolkit/models/object_detection/mobilenet_ssd/caffe/output
cp -f /root/output/mobilenet-ssd.mapping /opt/openvino_toolkit/models/object_detection/mobilenet_ssd/caffe/output
cp -f /root/output/mobilenet-ssd.xml /opt/openvino_toolkit/models/object_detection/mobilenet_ssd/caffe/output

#cd $model_optimizer && python3 mo.py --input_model /opt/openvino_toolkit/models/object_detection/mobilenet_ssd/caffe/output/public/mobilenet-ssd/mobilenet-ssd.caffemodel --output_dir /opt/openvino_toolkit/models/object_detection/mobilenet_ssd/caffe/output

# deeplabv3
mkdir -p /opt/intel/openvino_2021.4.752/deployment_tools/open_model_zoo/tools/downloader/public/deeplabv3/deeplabv3_mnv2_pascal_train_aug
cp -r /opt/openvino_toolkit/models/deeplabv3/output/public/deeplabv3/deeplabv3_mnv2_pascal_train_aug/frozen_inference_graph.pb /opt/intel/openvino_2021.4.752/deployment_tools/open_model_zoo/tools/downloader/public/deeplabv3/deeplabv3_mnv2_pascal_train_aug

cd /opt/intel/openvino_2021/deployment_tools/open_model_zoo/tools/downloader
cd $download_path && python3 downloader.py --name deeplabv3 --output_dir /opt/openvino_toolkit/models/deeplabv3/output
#cd $download_path && python3 converter.py --name=deeplabv3 --mo /opt/intel/openvino_2021/deployment_tools/model_optimizer/mo.py

apt update && apt install -y tmux

mkdir -p /root/catkin_ws/log
