# Prepare models
## MobileNetSSD
* copy from robot_devkit
```
mkdir -p ~/workspace/rdk_ws/data
cp -r /opt/openvino_toolkit/models/object_detection/mobilenet-ssd/caffe/output/* ~/workspace/rdk_ws/data
```

## Yolov2
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
  /opt/intel/openvino/deployment_tools/model_optimizer/mo_tf.py \
  --input_model built_graph/yolov2-voc.pb \
  --batch 1 \
  --tensorflow_use_custom_operations_config /opt/intel/openvino/deployment_tools/model_optimizer/extensions/front/tf/yolo_v1_v2.json \
  --data_type FP32 \
  --output_dir ./output/FP32
  # FP16 precision model
  /opt/intel/openvino/deployment_tools/model_optimizer/mo_tf.py \
  --input_model built_graph/yolov2-voc.pb \
  --batch 1 \
  --tensorflow_use_custom_operations_config /opt/intel/openvino/deployment_tools/model_optimizer/extensions/front/tf/yolo_v1_v2.json \
  --data_type FP16 \
  --output_dir ./output/FP16
  ```

## MaskRCNN
```
mkdir -p ~/workspace/rdk_ws/data
cp -r /opt/openvino_toolkit/models/segmentation/output/* ~/workspace/rdk_ws/data
```

## Face-reid and Person-reid
```
python3 /opt/intel/openvino/deployment_tools/tools/model_downloader/downloader.py --name person-detection-retail-0013-fp16 --output_dir /home/intel/workspace/rdk_ws/data/FP16
python3 /opt/intel/openvino/deployment_tools/tools/model_downloader/downloader.py --name person-reidentification-retail-0076-fp16 --output_dir /home/intel/workspace/rdk_ws/data/FP16
python3 /opt/intel/openvino/deployment_tools/tools/model_downloader/downloader.py --name face-detection-adas-0001-fp16 --output_dir /home/intel/workspace/rdk_ws/data/FP16
python3 /opt/intel/openvino/deployment_tools/tools/model_downloader/downloader.py --name face-reidentification-retail-0095-fp16 --output_dir /home/intel/workspace/rdk_ws/data/FP16
```

# How to launch examples
## Object Detection with RealSense Camera
Default model: mobilenet-ssd
Launch path: <openvino_root>/openvino_node/launch/rs_od.launch.py
Config path: <openvino_root>/openvino_node/config/object_detection.yaml
```
ros2 launch openvino_node rs_od.launch.py
```

## Object Segmentation with RealSense Camera
Default model: Mask-RCNN 
Launch path: <openvino_root>/openvino_node/launch/rs_osg.launch.py
Config path: <openvino_root>/openvino_node/config/object_segmentation.yaml
```
ros2 launch openvino_node rs_osg.launch.py
```

## Face Detection and Re-Identification with RealSense Camera
Launch path: <openvino_root>/openvino_node/launch/rs_face_reid.launch.py
Config path (Face Detection): <openvino_root>/openvino_node/config/face_detection.yaml
Config path (Face Re-Identification): <openvino_root>/openvino_node/config/face_reidentification.yaml
```
rs_face_reid.launch.py
```

## Person Detection and Re-Identification with RealSense Camera
Launch path: <openvino_root>/openvino_node/launch/rs_person_reid.launch.py
Config path (Person Detection): <openvino_root>/openvino_node/config/person_detection.yaml
Config path (Person Re-Identification): <openvino_root>/openvino_node/config/person_reidentification.yaml
```
rs_person_reid.launch.py

```
