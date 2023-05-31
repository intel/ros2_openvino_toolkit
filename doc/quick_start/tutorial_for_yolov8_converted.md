# Tutorial_For_yolov8_Converted

# Introduction
Ultralytics YOLOv8 is a cutting-edge, state-of-the-art (SOTA) model that builds upon the success of previous YOLO versions and introduces new features and improvements to further boost performance and flexibility.
YOLOv8 is designed to be fast, accurate, and easy to use, making it an excellent choice for a wide range of object detection and tracking, instance segmentation,
image classification and pose estimation tasks.
This document describes a method to convert YOLOv8 nano PyTorch weight files with the .pt extension to ONNX weight files, and a method to convert ONNX weight files to IR 
files using the OpenVINO model optimizer. This method can help OpenVINO users optimize YOLOv8 for deployment in practical applications.

## <div align="center">Documentation</div>

See below for a quickstart installation and usage example, and see the [YOLOv8 Docs](https://docs.ultralytics.com) for full documentation on training, validation, prediction and deployment.

<details open>
<summary>Install</summary>


Pip install the ultralytics package including all [requirements](https://github.com/ultralytics/ultralytics/blob/main/requirements.txt) in a [**Python>=3.7**](https://www.python.org/) environment with [**PyTorch>=1.7**](https://pytorch.org/get-started/locally/).

```bash
mkdir -p yolov8 && cd yolov8
pip install ultralytics
apt install python3.8-venv
python3 -m venv openvino_env
source openvino_env/bin/activate
```

 #### Train
Train YOLOv8n on the COCO128 dataset for 100 epochs at image size 640. For a full list of available arguments seethe Configuration page.
YOLOv8 may be used directly in the Command Line Interface (CLI) with a `yolo` command:

```CLI
# Build a new model from YAML and start training from scratch
yolo detect train data=coco128.yaml model=yolov8n.yaml epochs=100 imgsz=640

# Start training from a pretrained *.pt model
yolo detect train data=coco128.yaml model=yolov8n.pt epochs=100 imgsz=640
```


#### Val

Validate trained YOLOv8n model accuracy on the COCO128 dataset. No argument need to passed as the model retains it's training data and arguments as model attributes.
```CLI
# val official model
yolo detect val model=yolov8n.pt

```

#### Predict
Use a trained YOLOv8n model to run predictions on images.
``` CLI
# predict with official model
yolo detect predict model=yolov8n.pt source='https://ultralytics.com/images/bus.jpg'
```

#### Export
Export a YOLOv8n model to a different format like ONNX, CoreML, etc.
```
# export official model
yolo export model=yolov8n.pt format=openvino

```

# Move to the Recommended Model Path
```
cd yolov8n_openvino_model

mkdir -p  /opt/openvino_toolkit/models/convert/public/FP32/yolov8n

sudo cp yolov8* /opt/openvino_toolkit/models/convert/public/FP32/yolov8n

```

# yolov8n optimize to yolov8n-int8
```
The yolov8n optimize to yolov8n-int8 refer to the link:

https://github.com/openvinotoolkit/openvino_notebooks/blob/main/notebooks/230-yolov8-optimization/230-yolov8-optimization.ipynb

The installation guide
https://github.com/openvinotoolkit/openvino_notebooks/blob/main/README.md#-installation-guide

```

# FAQ

<p>
<details>
<summary>Reference link</summary>

```
https://github.com/ultralytics/ultralytics
https://docs.ultralytics.com/tasks/detect/#predict

```

</details>
</p>
