# Multiple Pipelines
>> This is a way to run more than one pipeline in the same process.Having multiple pipelines in a single instance allows each pipeline to have custom configuration and different performance.

## prerequest
see [this guide](https://github.com/intel/ros2_openvino_toolkit/blob/doc-ov.2020.3/doc/tables_of_contents/tutorials/configuration_file_customization.md) to see how to customize a pipeline.

## A demo for multiple pipeline
```bash
1 Pipelines:
  2 - name: object1
  3   inputs: [StandardCamera]
  4   infers:
  5     - name: ObjectDetection
  6       model: /opt/openvino_toolkit/models/object_detection/mobilenet-ssd/caffe/output/FP32/mobilenet-ssd.xml
  7       engine: CPU
  8       label: to/be/set/xxx.labels
  9       batch: 1
 10       confidence_threshold: 0.5
 11       enable_roi_constraint: true # set enable_roi_constraint to false if you don't want to make the inferred ROI (region of interest) constrained into the camera frame
 12   outputs: [ImageWindow, RosTopic, RViz]
 13   connects:
 14     - left: StandardCamera
 15       right: [ObjectDetection]
 16     - left: ObjectDetection
 17       right: [ImageWindow]
 18     - left: ObjectDetection
 19       right: [RosTopic]
 20     - left: ObjectDetection
 21       right: [RViz]
 22 
 23 - name: object2
 24   inputs: [RealSenseCamera]
 25   infers:
 26     - name: ObjectDetection
 27       model: /opt/openvino_toolkit/models/object_detection/mobilenet-ssd/caffe/output/FP32/mobilenet-ssd.xml
 28       engine: CPU
 29       label: to/be/set/xxx.labels
 30       batch: 1
 31       confidence_threshold: 0.5
 32       enable_roi_constraint: true # set enable_roi_constraint to false if you don't want to make the inferred ROI (region of interest) constrained into the camera frame
 33   outputs: [ImageWindow, RosTopic, RViz]
 34   connects:
 35     - left: RealSenseCamera
 36       right: [ObjectDetection]
 37     - left: ObjectDetection
 38       right: [ImageWindow]
 39     - left: ObjectDetection
 40       right: [RosTopic]
 41     - left: ObjectDetection
 42       right: [RViz]
 43 
 44 OpenvinoCommon:

```
