#!/bin/bash

#1. Copy YOLOv5 Repository from GitHub
cd /root && git clone https://github.com/ultralytics/yolov5.git

#Set Environment for Installing YOLOv5

cd yolov5
python3 -m venv yolo_env            # Create a virtual python environment
source yolo_env/bin/activate        # Activate environment
pip install -r requirements.txt     # Install yolov5 prerequisites
pip install wheel
pip install onnx

# Download PyTorch Weights
mkdir -p /root/yolov5/model_convert && cd /root/yolov5/model_convert
wget https://github.com/ultralytics/yolov5/releases/download/v6.2/yolov5n.pt

cd /root/yolov5
python3 export.py --weights model_convert/yolov5n.pt --include onnx


#2. Convert ONNX files to IR files
cd /root/yolov5/
python3 -m venv ov_env                      # Create openVINO virtual environment
source ov_env/bin/activate                  # Activate environment
python -m pip install --upgrade pip         # Upgrade pip
pip install openvino[onnx]==2022.1.0        # Install OpenVINO for ONNX
pip install openvino-dev[onnx]==2022.1.0    # Install OpenVINO Dev Tool for ONNX


cd /root/yolov5/model_convert
mo --input_model yolov5n.onnx


mkdir -p  /opt/openvino_toolkit/models/convert/public/yolov5n/FP32/
sudo cp yolov5n.bin yolov5n.mapping yolov5n.xml /opt/openvino_toolkit/models/convert/public/yolov5n/FP32/

