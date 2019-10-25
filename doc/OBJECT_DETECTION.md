# Object Detection
## Launching
### OpenSource Version
#### mobilenet-ssd
* download and convert a trained model to produce an optimized Intermediate Representation (IR) of the model 
  ```bash
  cd /opt/openvino_toolkit/open_model_zoo/tools/downloader
  python3 ./downloader.py --name mobilenet-ssd
  #FP32 precision model
  sudo python3 /opt/openvino_toolkit/dldt/model-optimizer/mo.py --input_model /opt/openvino_toolkit/open_model_zoo/tools/downloader/public/mobilenet-ssd/mobilenet-ssd.caffemodel --output_dir /opt/openvino_toolkit/models/object_detection/mobilenet-ssd/caffe/output/FP32 --mean_values [127.5,127.5,127.5] --scale_values [127.5]
  #FP16 precision model
  sudo python3 /opt/openvino_toolkit/dldt/model-optimizer/mo.py --input_model /opt/openvino_toolkit/open_model_zoo/tools/downloader/public/mobilenet-ssd/mobilenet-ssd.caffemodel --output_dir /opt/openvino_toolkit/models/object_detection/mobilenet-ssd/caffe/output/FP16 --data_type=FP16 --mean_values [127.5,127.5,127.5] --scale_values [127.5]
  ```
* copy label files (excute _once_)<br>
  ```bash
  sudo cp /opt/openvino_toolkit/ros2_openvino_toolkit/data/labels/object_detection/mobilenet-ssd.labels /opt/openvino_toolkit/models/object_detection/mobilenet-ssd/caffe/output/FP32
  sudo cp /opt/openvino_toolkit/ros2_openvino_toolkit/data/labels/object_detection/mobilenet-ssd.labels /opt/openvino_toolkit/models/object_detection/mobilenet-ssd/caffe/output/FP16
  ```
* run object detection sample code input from RealSenseCamera.(connect Intel® Neural Compute Stick 2)
  ```bash
  ros2 launch dynamic_vino_sample pipeline_object.launch.py
  ```
* run object detection sample code input from RealSenseCameraTopic.(connect Intel® Neural Compute Stick 2)
  ```bash
  ros2 launch dynamic_vino_sample pipeline_object_topic.launch.py
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
  sudo python3 /opt/openvino_toolkit/dldt/model-optimizer/mo_tf.py \
  --input_model built_graph/yolov2-voc.pb \
  --batch 1 \
  --tensorflow_use_custom_operations_config /opt/openvino_toolkit/dldt/model-optimizer/extensions/front/tf/yolo_v2_voc.json \
  --data_type FP32 \
  --output_dir /opt/openvino_toolkit/models/object_detection/YOLOv2-voc/tf/output/FP32
  # FP16 precision model
  sudo python3 /opt/openvino_toolkit/dldt/model-optimizer/mo_tf.py \
  --input_model built_graph/yolov2-voc.pb \
  --batch 1 \
  --tensorflow_use_custom_operations_config /opt/openvino_toolkit/dldt/model-optimizer/extensions/front/tf/yolo_v2_voc.json \
  --data_type FP16 \
  --output_dir /opt/openvino_toolkit/models/object_detection/YOLOv2-voc/tf/output/FP16
  ```
* copy label files (excute _once_)<br>
  ```bash
  sudo cp /opt/openvino_toolkit/ros2_openvino_toolkit/data/labels/object_detection/yolov2-voc.labels /opt/openvino_toolkit/models/object_detection/YOLOv2-voc/tf/output/FP32  
  sudo cp /opt/openvino_toolkit/ros2_openvino_toolkit/data/labels/object_detection/yolov2-voc.labels /opt/openvino_toolkit/models/object_detection/YOLOv2-voc/tf/output/FP16
  ```
* run object detection sample code input from RealSenseCamera.(connect Intel® Neural Compute Stick 2)
  ```bash
  ros2 launch dynamic_vino_sample pipeline_object_yolo.launch.py
  ```
* run object detection sample code input from RealSenseCameraTopic.(connect Intel® Neural Compute Stick 2)
  ```bash
  ros2 launch dynamic_vino_sample pipeline_object_yolo_topic.launch.py
  ```
### Binary Version
#### mobilenet-ssd
* download and convert a trained model to produce an optimized Intermediate Representation (IR) of the model 
  ```bash
  cd /opt/intel/openvino/deployment_tools/tools/model_downloader
  sudo python3 ./downloader.py --name mobilenet-ssd
  #FP32 precision model
  sudo python3 /opt/intel/openvino/deployment_tools/model_optimizer/mo.py --input_model /opt/intel/openvino/deployment_tools/tools/model_downloader/object_detection/common/mobilenet-ssd/caffe/mobilenet-ssd.caffemodel --output_dir /opt/openvino_toolkit/models/object_detection/mobilenet-ssd/caffe/output/FP32 --mean_values [127.5,127.5,127.5] --scale_values [127.5]
  #FP16 precision model
  sudo python3 /opt/intel/openvino/deployment_tools/model_optimizer/mo.py --input_model /opt/intel/openvino/deployment_tools/tools/model_downloader/object_detection/common/mobilenet-ssd/caffe/mobilenet-ssd.caffemodel --output_dir /opt/openvino_toolkit/models/object_detection/mobilenet-ssd/caffe/output/FP16 --data_type=FP16 --mean_values [127.5,127.5,127.5] --scale_values [127.5]
  ```
* copy label files (excute _once_)<br>
  ```bash
  sudo cp /opt/openvino_toolkit/ros2_openvino_toolkit/data/labels/object_detection/mobilenet-ssd.labels /opt/openvino_toolkit/models/object_detection/mobilenet-ssd/caffe/output/FP32
  sudo cp /opt/openvino_toolkit/ros2_openvino_toolkit/data/labels/object_detection/mobilenet-ssd.labels /opt/openvino_toolkit/models/object_detection/mobilenet-ssd/caffe/output/FP16
  ```
* run object detection sample code input from RealSenseCamera.(connect Intel® Neural Compute Stick 2)
  ```bash
  ros2 launch dynamic_vino_sample pipeline_object.launch.py
  ```
* run object detection sample code input from RealSenseCameraTopic.(connect Intel® Neural Compute Stick 2)
  ```bash
  ros2 launch dynamic_vino_sample pipeline_object_topic.launch.py
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
  sudo python3 /opt/intel/openvino/deployment_tools/model_optimizer/mo_tf.py \
  --input_model built_graph/yolov2-voc.pb \
  --batch 1 \
  --tensorflow_use_custom_operations_config /opt/intel/openvino/deployment_tools/model_optimizer/extensions/front/tf/yolo_v2_voc.json \
  --data_type FP32 \
  --output_dir /opt/openvino_toolkit/models/object_detection/YOLOv2-voc/tf/output/FP32
  # FP16 precision model
  sudo python3 /opt/intel/openvino/deployment_tools/model_optimizer/mo_tf.py \
  --input_model built_graph/yolov2-voc.pb \
  --batch 1 \
  --tensorflow_use_custom_operations_config /opt/intel/openvino/deployment_tools/model_optimizer/extensions/front/tf/yolo_v2_voc.json \
  --data_type FP16 \
  --output_dir /opt/openvino_toolkit/models/object_detection/YOLOv2-voc/tf/output/FP16
  ```
* copy label files (excute _once_)<br>
  ```bash
  sudo cp /opt/openvino_toolkit/ros2_openvino_toolkit/data/labels/object_detection/yolov2-voc.labels /opt/openvino_toolkit/models/object_detection/YOLOv2-voc/tf/output/FP32
  sudo cp /opt/openvino_toolkit/ros2_openvino_toolkit/data/labels/object_detection/yolov2-voc.labels /opt/openvino_toolkit/models/object_detection/YOLOv2-voc/tf/output/FP16
  ```
* run object detection sample code input from RealSenseCamera.(connect Intel® Neural Compute Stick 2)
  ```bash
  ros2 launch dynamic_vino_sample pipeline_object_yolo.launch.py
  ```
* run object detection sample code input from RealSenseCameraTopic.(connect Intel® Neural Compute Stick 2)
  ```bash
  ros2 launch dynamic_vino_sample pipeline_object_yolo_topic.launch.py
  ```



