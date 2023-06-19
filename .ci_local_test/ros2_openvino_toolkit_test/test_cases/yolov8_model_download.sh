#!/bin/bash

#Pip install the ultralytics package including all requirements in a Python>=3.7 environment with PyTorch>=1.7.

mkdir -p yolov8 && cd yolov8
pip install ultralytics
apt install python3.8-venv
python3 -m venv openvino_env
source openvino_env/bin/activate


#Export a YOLOv8n model to a different format like ONNX, CoreML, etc.
# export official model
yolo export model=yolov8n.pt format=openvino


# Move to the Recommended Model Path
mkdir -p  /opt/openvino_toolkit/models/convert/public/FP32/yolov8n

cp yolov8n_openvino_model/* /opt/openvino_toolkit/models/convert/public/FP32/yolov8n

