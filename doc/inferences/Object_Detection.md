# Object Detection
## Introduction
The section depict the kind of Object Detection, which produces object classification and its location based ROI. 
Two kinds of models are supported currently:
- SSD based Object Detection Models
    * SSD300-VGG16, SSD500-VGG16, Mobilenet-SSD (both caffe and tensorflow)
- YoloV2

## Demo Result Snapshots
* object detection input from realsense camera

![object_detection_demo_realsense](https://github.com/intel/ros2_openvino_toolkit/blob/doc-ov.2020.3/data/images/object_detection.gif "object detection demo realsense")

## Download Models
>> Before using the supported models, you need to first downloand and optimize them into OpenVINO mode. mobilenet-SSD caffe model is the default one used in the Object Detection configuration. 

#### mobilenet-ssd
* download and convert a trained model to produce an optimized Intermediate Representation (IR) of the model 
  ```bash
  cd $model_downloader
  sudo python3 ./downloader.py --name mobilenet-ssd
  #FP32 precision model
  sudo python3 $model_optimizer/mo.py --input_model $model_downloader/public/mobilenet-ssd/mobilenet-ssd.caffemodel --output_dir /opt/openvino_toolkit/models/object_detection/mobilenet-ssd/caffe/output/FP32 --mean_values [127.5,127.5,127.5] --scale_values [127.5]
  #FP16 precision model
  sudo python3 $model_optimizer/mo.py --input_model $model_downloader/public/mobilenet-ssd/mobilenet-ssd.caffemodel --output_dir /opt/openvino_toolkit/models/object_detection/mobilenet-ssd/caffe/output/FP16 --data_type=FP16 --mean_values [127.5,127.5,127.5] --scale_values [127.5]
  ```
* copy label files (excute _once_)<br>
  ```bash
  sudo cp $openvino_labels/object_detection/mobilenet-ssd.labels /opt/openvino_toolkit/models/object_detection/mobilenet-ssd/caffe/output/FP32
  sudo cp $openvino_labels/object_detection/mobilenet-ssd.labels /opt/openvino_toolkit/models/object_detection/mobilenet-ssd/caffe/output/FP16
  ```
#### YOLOv2-voc
* Darkflow to protobuf(.pb)
  - install [darkflow](https://github.com/thtrieu/darkflow)
    - install prerequsites
    ```bash
    pip3 install tensorflow opencv-python numpy networkx cython
    ```
    - Get darkflow and YOLO-OpenVINO
    ```bash
    mkdir -p ~/code && cd ~/code
    git clone https://github.com/thtrieu/darkflow
    git clone https://github.com/chaoli2/YOLO-OpenVINO
    sudo ln -sf ~/code/darkflow /opt/openvino_toolkit/
    ```
    - modify the line self.offset = 16 in the ./darkflow/utils/loader.py file and replace with self.offset = 20
    - Install darkflow
    ```bash
    cd ~/code/darkflow
    pip3 install .
    ```
  - Copy voc.names in YOLO-OpenVINO/common to labels.txt in darkflow.
    ```bash
    cp ~/code/YOLO-OpenVINO/common/voc.names ~/code/darkflow/labels.txt
    ```
  - Get yolov2 weights and cfg
    ```bash
    cd ~/code/darkflow
    mkdir -p models
    cd models
    wget -c https://pjreddie.com/media/files/yolov2-voc.weights
    wget https://raw.githubusercontent.com/pjreddie/darknet/master/cfg/yolov2-voc.cfg
    ```
  - Run convert script
    ```bash
    cd ~/code/darkflow
    flow --model models/yolov2-voc.cfg --load models/yolov2-voc.weights --savepb
    ```
* Convert YOLOv2-voc TensorFlow Model to the optimized Intermediate Representation (IR) of model
  ```bash
  cd ~/code/darkflow
  # FP32 precision model
  sudo python3 $model_optimizer/mo_tf.py \
  --input_model built_graph/yolov2-voc.pb \
  --batch 1 \
  --tensorflow_use_custom_operations_config $model_optimizer/extensions/front/tf/yolo_v2_voc.json \
  --data_type FP32 \
  --output_dir /opt/openvino_toolkit/models/object_detection/YOLOv2-voc/tf/output/FP32
  # FP16 precision model
  sudo python3 $model_optimizer/mo_tf.py \
  --input_model built_graph/yolov2-voc.pb \
  --batch 1 \
  --tensorflow_use_custom_operations_config $model_optimizer/extensions/front/tf/yolo_v2_voc.json \
  --data_type FP16 \
  --output_dir /opt/openvino_toolkit/models/object_detection/YOLOv2-voc/tf/output/FP16
  ```
* copy label files (excute _once_)<br>
  ```bash
  sudo cp $openvino_labels/object_detection/yolov2-voc.labels /opt/openvino_toolkit/models/object_detection/YOLOv2-voc/tf/output/FP32  
  sudo cp $openvino_labels/object_detection/yolov2-voc.labels /opt/openvino_toolkit/models/object_detection/YOLOv2-voc/tf/output/FP16
  ```
