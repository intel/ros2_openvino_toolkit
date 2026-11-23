# YOLO26 pipelines

[Ultralytics YOLO26](https://docs.ultralytics.com/models/yolo26/) models run in the pipelines for the tasks below. The models are exported to OpenVINO IR on your machine, in the same way as in the [YOLO26 notebooks of OpenVINO](https://github.com/openvinotoolkit/openvino_notebooks/tree/latest/notebooks/yolov26-optimization).

|Task|`--tasks` value|Model|Launch file|Result topic|
|---|---|---|---|---|
|Object detection|`detect`|`yolo26n`|`pipeline_yolo26_detection.launch.py`|`/ros2_openvino_toolkit/detected_objects` ([ObjectsInBoxes](../../object_msgs/msg/ObjectsInBoxes.msg))|
|Instance segmentation|`segment`|`yolo26n-seg`|`pipeline_yolo26_segmentation.launch.py`|`/ros2_openvino_toolkit/segmented_objects` ([ObjectsInMasks](../../object_msgs/msg/ObjectsInMasks.msg))|
|Keypoint detection (pose)|`pose`|`yolo26n-pose`|`pipeline_yolo26_pose.launch.py`|`/ros2_openvino_toolkit/detected_keypoints` ([ObjectsWithKeypoints](../../object_msgs/msg/ObjectsWithKeypoints.msg))|
|Oriented bounding boxes (OBB)|`obb`|`yolo26n-obb`|`pipeline_yolo26_obb.launch.py`|`/ros2_openvino_toolkit/detected_oriented_objects` ([ObjectsInOrientedBoxes](../../object_msgs/msg/ObjectsInOrientedBoxes.msg))|
|Classification|`classify`|`yolo26n-cls`|`pipeline_yolo26_classification.launch.py`|`/ros2_openvino_toolkit/classified_objects` ([Objects](../../object_msgs/msg/Objects.msg))|
|Semantic segmentation|`semantic`|`yolo26n-sem`|`pipeline_yolo26_semantic.launch.py`|`/ros2_openvino_toolkit/segmented_objects` ([ObjectsInMasks](../../object_msgs/msg/ObjectsInMasks.msg))|
|Depth estimation|`depth`|`yolo26n-depth`|`pipeline_yolo26_depth.launch.py`|`/ros2_openvino_toolkit/depth_image` (`sensor_msgs/Image`, `32FC1`, meters)|

All pipelines also publish the annotated frame on `/ros2_openvino_toolkit/image_rviz`.

## 1. Export the models

```bash
ros2 run openvino_node setup_models.sh --all     # or: export_yolo26.sh --tasks all
```

`setup_models.sh` exports the YOLO26 models with `export_yolo26.sh`, which can also be run directly with these options (`setup_models.sh` passes `--size` and `--precision` on):
|Option|Values|Default|
|---|---|---|
|`--tasks`|Comma-separated list of the values in the table above, or `all`|`detect`|
|`--size`|`n`, `s`, `m`, `l`, `x`|`n`|
|`--precision`|`FP32`, `FP16`, `INT8`|`FP16`|

The first run creates a Python virtual environment with the `ultralytics` package in `~/.local/share/openvino_toolkit/venv/yolo26` (about 2 GB). The models are stored in `~/.local/share/openvino_toolkit/models/yolo26/<model>/<precision>/` together with a `<model>.labels` file that holds the class names of the model. Set `OPENVINO_TOOLKIT_DIR` to use another location, as for `download_models.sh`.

`INT8` models are calibrated on the small default dataset of Ultralytics. Check their accuracy before you use them.

**License:** the YOLO26 code and weights are provided by Ultralytics under [AGPL-3.0 or an Enterprise license](https://www.ultralytics.com/license). They are downloaded by the script and are not part of this repository or its Debian packages.

## 2. Run a pipeline

```bash
ros2 launch openvino_node pipeline_yolo26_detection.launch.py
ros2 launch openvino_node pipeline_yolo26_pose.launch.py device:=GPU
```

The pipelines read the first USB camera. The launch arguments `device`, `viewer` and `yaml_path` work as described in the [getting started guide](./getting_started_with_ros2_ov2.0.md).

The pretrained OBB model is trained on aerial images (DOTA) and the semantic segmentation model on street scenes (Cityscapes), so they give few results on other images.

## 3. Use a YOLO26 model in your own pipeline

Detection, instance segmentation and semantic segmentation use the existing inferences with `model_type: yolo26`. The other tasks have their own inference name. See the YAML files `sample/param/pipeline_yolo26_*.yaml`.

|Task|Inference `name`|`model_type`|
|---|---|---|
|Object detection|`ObjectDetection`|`yolo26`|
|Instance segmentation|`ObjectSegmentationInstance`|`yolo26`|
|Semantic segmentation|`ObjectSegmentation`|`yolo26`|
|Keypoint detection|`KeypointDetection`|-|
|Oriented bounding boxes|`OrientedObjectDetection`|-|
|Classification|`ImageClassification`|-|
|Depth estimation|`DepthEstimation`|-|

```yaml
infers:
  - name: ObjectDetection
    model: <OPENVINO_NODE_SHARE>/models/yolo26/yolo26s/FP16/yolo26s.xml
    model_type: yolo26
    engine: GPU
    label: <OPENVINO_NODE_SHARE>/models/yolo26/yolo26s/FP16/yolo26s.labels
    batch: 1
    confidence_threshold: 0.5
```

For classification, `confidence_threshold` selects the classes that are reported (at most five).

To use a model that you trained yourself, export it with `yolo export model=<file>.pt format=openvino nms=False` and write its class names to a label file, one per line. The detection, segmentation, pose and OBB pipelines need the end-to-end output of `nms=False`. A model with another output is rejected at start-up.

## 4. Test

`make test` and `make test-functional` include the YOLO26 pipelines when the models are in the model cache `.models/` of the repository, and skip them otherwise. Copy the exported models into the cache with:

```bash
mkdir -p .models/models && cp -r ~/.local/share/openvino_toolkit/models/yolo26 .models/models/
```
