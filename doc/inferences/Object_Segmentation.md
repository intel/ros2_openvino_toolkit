# Object Segmentation
## Demo Result Snapshots
See below pictures for the demo result snapshots.
* object segmentation input from video
![object_segmentation_demo_video](https://github.com/intel/ros2_openvino_toolkit/blob/doc-ov.2020.3/data/images/object_segmentation.gif "object segmentation demo video")
## Download Models
* download and convert a trained model to produce an optimized Intermediate Representation (IR) of the model 
  ```bash
  #object segmentation model
  mkdir -p ~/Downloads/models
  cd ~/Downloads/models
  wget http://download.tensorflow.org/models/object_detection/mask_rcnn_inception_v2_coco_2018_01_28.tar.gz
  tar -zxvf mask_rcnn_inception_v2_coco_2018_01_28.tar.gz
  cd mask_rcnn_inception_v2_coco_2018_01_28
  #FP32
  sudo python3 $model_optimizer/mo_tf.py --input_model frozen_inference_graph.pb --tensorflow_use_custom_operations_config $model_optimizer/extensions/front/tf/mask_rcnn_support.json --tensorflow_object_detection_api_pipeline_config pipeline.config --reverse_input_channels --output_dir /opt/openvino_toolkit/models/segmentation/output/FP32
  #FP16
  sudo python3 $model_optimizer/mo_tf.py --input_model frozen_inference_graph.pb --tensorflow_use_custom_operations_config $model_optimizer/extensions/front/tf/mask_rcnn_support.json --tensorflow_object_detection_api_pipeline_config pipeline.config --reverse_input_channels --data_type=FP16 --output_dir /opt/openvino_toolkit/models/segmentation/output/FP16
  ```
* copy label files (excute _once_)<br>
	```bash
	sudo cp $openvino_labels/object_segmentation/frozen_inference_graph.labels /opt/openvino_toolkit/models/segmentation/output/FP32
	sudo cp $openvino_labels/object_segmentation/frozen_inference_graph.labels /opt/openvino_toolkit/models/segmentation/output/FP16
	```
