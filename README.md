# Branch for Robot DevKit

# How to convert model
## MobileNetSSD
* copy from robot_devkit
```
mkdir -p ~/workspace/robot_sdk/data
cp -r /opt/openvino_toolkit/models/object_detection/mobilenet-ssd/caffe/output/* ~/workspace/robot_sdk/data
```

* convert from open model zoo
```
mkdir -p ~/workspace/robot_sdk/data
cd ~/workspace/robot_sdk/data
git clone https://github.com/opencv/open_model_zoo.git
cd open_model_zoo/demos/
git checkout 2019
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release /opt/intel/openvino/deployment_tools/inference_engine
make -j8

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
  --output_dir ./output/fp32
  # FP16 precision model
  /opt/intel/openvino/deployment_tools/model_optimizer/mo_tf.py \
  --input_model built_graph/yolov2-voc.pb \
  --batch 1 \
  --tensorflow_use_custom_operations_config /opt/intel/openvino/deployment_tools/model_optimizer/extensions/front/tf/yolo_v1_v2.json \
  --data_type FP16 \
  --output_dir ./output/fp16
  ```

## MaskRCNN
```
cd /opt/intel/openvino/deployment_tools/model_optimizer/install_prerequisites
sudo ./install_prerequisites.sh
mkdir -p ~/Downloads/models
cd ~/Downloads/models
wget http://download.tensorflow.org/models/object_detection/mask_rcnn_inception_v2_coco_2018_01_28.tar.gz
tar -zxvf mask_rcnn_inception_v2_coco_2018_01_28.tar.gz
cd mask_rcnn_inception_v2_coco_2018_01_28
python3 /opt/intel/openvino/deployment_tools/model_optimizer/mo_tf.py --input_model frozen_inference_graph.pb --tensorflow_use_custom_operations_config /opt/intel/openvino/deployment_tools/model_optimizer/extensions/front/tf/mask_rcnn_support.json --tensorflow_object_detection_api_pipeline_config pipeline.config --reverse_input_channels --data_type=FP16 --output_dir ~/workspace/robot_sdk/data/FP16/
```

## Face-reid and Person-reid
```
python3 /opt/intel/openvino/deployment_tools/tools/model_downloader/downloader.py --name person-detection-retail-0013-fp16 --output_dir /home/intel/workspace/robot_sdk/data/FP16
python3 /opt/intel/openvino/deployment_tools/tools/model_downloader/downloader.py --name person-reidentification-retail-0076-fp16 --output_dir /home/intel/workspace/robot_sdk/data/FP16
python3 /opt/intel/openvino/deployment_tools/tools/model_downloader/downloader.py --name face-detection-adas-0001-fp16 --output_dir /home/intel/workspace/robot_sdk/data/FP16
python3 /opt/intel/openvino/deployment_tools/tools/model_downloader/downloader.py --name face-reidentification-retail-0095-fp16 --output_dir /home/intel/workspace/robot_sdk/data/FP16
```
