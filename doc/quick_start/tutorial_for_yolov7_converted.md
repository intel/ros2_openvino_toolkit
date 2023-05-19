# Tutorial_For_yolov7_Converted

# Introduction
This document describes a method to convert YOLOv7 nano PyTorch weight files with the .pt extension to ONNX weight files, and a method to convert ONNX weight files to IR 
files using the OpenVINO model optimizer. This method can help OpenVINO users optimize YOLOv7 for deployment in practical applications.

## Reference Phrase
|Term|Description|
|---|---|
|OpenVINO|Open Visual Inference & Neural Network Optimization|
|ONNX|Open Neural Network Exchange|
|YOLO|You Only Look Once|
|IR|Intermediate Representation|

## Reference Document
|Doc|Link|
|---|---|
|OpenVINO|[openvino_2_0_transition_guide](https://docs.openvino.ai/latest/openvino_2_0_transition_guide.html)|
|YOLOv7|[yolov7](https://github.com/WongKinYiu/yolov7)|

# Convert Weight File to ONNX
* Copy YOLOv7 Repository from GitHub
```
git clone https://github.com/WongKinYiu/yolov7.git
```

* Set Environment for Installing YOLOv7
```
cd yolov7
python3 -m venv yolo_env            // Create a virtual python environment
source yolo_env/bin/activate        // Activate environment
pip install -r requirements.txt     // Install yolov7 prerequisites
pip install onnx                    // Install ONNX
pip install nvidia-pyindex          // Add NVIDIA PIP index
pip install onnx-graphsurgeon       // Install GraphSurgeon
```

* Download PyTorch Weights
```
mkdir model_convert && cd model_convert
wget "https://github.com/WongKinYiu/yolov7/releases/download/v0.1/yolov7.pt"
```

* Convert PyTorch weights to ONNX weights
YOLOv7 repository provides export.py script, which can be used to convert PyTorch weight to ONNX weight.
```
cd ..
python3 export.py --weights model_convert/yolov7.pt --include onnx --grid
```

# Convert ONNX files to IR files
After obtaining the ONNX weight file from the previous section [Convert Weight File to ONNX](#convert-weight-file-to-onnx), we can use the model optimizer to convert it to an IR file.

* Install the OpenVINO Model Optimizer Environment
To use the model optimizer, you need to run the following command to install some necessary components (if you are still in the yolo_env virtual environment, you need to run the **deactivate** command to exit the environment or start a new terminal).
```
python3 -m venv ov_env                      // Create openVINO virtual environment
source ov_env/bin/activate                  // Activate environment
python -m pip install --upgrade pip         // Upgrade pip
pip install openvino[onnx]==2022.3.0        // Install OpenVINO for ONNX
pip install openvino-dev[onnx]==2022.3.0    // Install OpenVINO Dev Tool for ONNX
```

* Generate IR file
```
cd model_convert
mo --input_model yolov7.onnx
```
Then we will get three files: yolov7.xml, yolov7.bin, and yolov7.mapping under the model_convert folder.

# Move to the Recommended Model Path
```
cd ~/yolov7/model_convert
mkdir -p  /opt/openvino_toolkit/models/convert/public/yolov7/FP32/
sudo cp yolov7.bin yolov7.mapping yolov7.xml /opt/openvino_toolkit/models/convert/public/yolov7/FP32/
```

# FAQ

<p>
<details>
<summary>How to install the python3-venv package?</summary>

On Debian/Ubuntu systems, you need to install the python3-venv package using the following command.
```
apt-get update
apt-get install python3-venv
```
You may need to use sudo with that command. After installing, recreate your virtual environment.
</details>
</p>
