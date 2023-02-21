#!/bin/bash

# modify pipeline_face_reidentification.yaml
sed -i 's#ImageWindow, RosTopic, RViz#RosTopic#g' $dynamic_vino_sample/param/pipeline_face_reidentification.yaml
sed -i '/# Rviz/,+4 s/^/#/' $dynamic_vino_sample/launch/pipeline_face_reidentification.launch.py

# modify pipeline_image.yaml
sed -i 's#input_path: to/be/set/image_path#input_path: /root/catkin_ws/src/ros2_openvino_toolkit/data/images/sample_faces.jpg#g' $dynamic_vino_sample/param/pipeline_image.yaml
sed -i 's#ImageWindow, RosTopic, RViz#RosTopic#g' $dynamic_vino_sample/param/pipeline_image.yaml
sed -i '/# Rviz/,+4 s/^/#/' $dynamic_vino_sample/launch/pipeline_image.launch.py

# modify pipeline_people.yaml
sed -i 's#ImageWindow, RosTopic, RViz#RosTopic#g' $dynamic_vino_sample/param/pipeline_people.yaml
sed -i '4 i input_path: /root/catkin_ws/src/ros2_openvino_toolkit/data/images/sample_faces.jpg' $dynamic_vino_sample/param/pipeline_people.yaml && sed -i 's#input_path#  input_path#g' $dynamic_vino_sample/param/pipeline_people.yaml
sed -i 's#StandardCamera#Image#g' $dynamic_vino_sample/param/pipeline_people.yaml
sed -i '/# Rviz/,+4 s/^/#/' $dynamic_vino_sample/launch/pipeline_people.launch.py

# modify pipeline_person_attributes.yaml
sed -i 's#ImageWindow, RViz, RosTopic#RosTopic#g' $dynamic_vino_sample/param/pipeline_person_attributes.yaml
sed -i 's#ImageWindow, RosTopic, RViz#RosTopic#g' $dynamic_vino_sample/param/pipeline_person_attributes.yaml
sed -i '4 i input_path: /root/catkin_ws/src/ros2_openvino_toolkit/data/images/sample_faces.jpg' $dynamic_vino_sample/param/pipeline_person_attributes.yaml && sed -i 's#input_path#  input_path#g' $dynamic_vino_sample/param/pipeline_person_attributes.yaml
sed -i 's#StandardCamera#Image#g' $dynamic_vino_sample/param/pipeline_person_attributes.yaml
sed -i '/# Rviz/,+4 s/^/#/' $dynamic_vino_sample/launch/pipeline_person_attributes.launch.py

# modify pipeline_redentification.yaml
sed -i 's#ImageWindow, RViz, RosTopic#RosTopic#g' $dynamic_vino_sample/param/pipeline_reidentification.yaml
sed -i '/# Rviz/,+4 s/^/#/' $dynamic_vino_sample/launch/pipeline_reidentification.launch.py
sed -i '4 i input_path: /root/catkin_ws/src/ros2_openvino_toolkit/data/images/sample_faces.jpg' $dynamic_vino_sample/param/pipeline_reidentification.yaml && sed -i 's#input_path#  input_path#g' $dynamic_vino_sample/param/pipeline_reidentification.yaml
sed -i 's#StandardCamera#Image#g' $dynamic_vino_sample/param/pipeline_reidentification.yaml

# modify pipeline_segmentation_image.yaml
sed -i 's#input_path: to/be/set/image_path#input_path: /root/catkin_ws/src/ros2_openvino_toolkit/data/images/sample_faces.jpg#g' $dynamic_vino_sample/param/pipeline_segmentation_image.yaml
sed -i 's#ImageWindow, RosTopic, RViz#RosTopic#g' $dynamic_vino_sample/param/pipeline_segmentation_image.yaml
sed -i 's#RViz##g' $dynamic_vino_sample/param/pipeline_segmentation_image.yaml
sed -i '/# Rviz/,+4 s/^/#/' $dynamic_vino_sample/launch/pipeline_segmentation_image.launch.py

# modify pipeline_segmentation.yaml
sed -i 's#ImageWindow, RosTopic, RViz#RosTopic#g' $dynamic_vino_sample/param/pipeline_segmentation.yaml
sed -i 's#RViz##g' $dynamic_vino_sample/param/pipeline_segmentation.yaml
sed -i 's#RealSenseCameraTopic#Image#g' $dynamic_vino_sample/param/pipeline_segmentation.yaml
sed -i '4 i input_path: /root/jpg/segmentation.jpg' $dynamic_vino_sample/param/pipeline_segmentation.yaml && sed -i 's#input_path#  input_path#g' $dynamic_vino_sample/param/pipeline_segmentation.yaml
sed -i '/# Rviz/,+4 s/^/#/' $dynamic_vino_sample/launch/pipeline_segmentation.launch.py

# modify pipeline_object_yolo.yaml
sed -i 's#input_path: to/be/set/image_path#input_path: /root/catkin_ws/src/ros2_openvino_toolkit/data/images/sample_faces.jpg#g' $dynamic_vino_sample/param/pipeline_object_yolo.yaml
sed -i 's#to/be/set/xxx.labels#/opt/openvino_toolkit/models/convert/public/yolov5n/FP32/coco.names#g' $dynamic_vino_sample/param/pipeline_object_yolo.yaml
sed -i '/# Rviz/,+4 s/^/#/' $dynamic_vino_sample/launch/pipeline_object_yolo.launch.py
sed -i 's#ImageWindow, RosTopic, RViz#RosTopic#g' $dynamic_vino_sample/param/pipeline_object_yolo.yaml


# modify pipeline_vehicle_detection.yaml
sed -i 's#ImageWindow, RosTopic, RViz#RosTopic#g' $dynamic_vino_sample/param/pipeline_vehicle_detection.yaml
sed -i 's#ImageWindow, RViz, RosTopic#RosTopic#g' $dynamic_vino_sample/param/pipeline_vehicle_detection.yaml
sed -i '/# Rviz/,+4 s/^/#/' $dynamic_vino_sample/launch/pipeline_vehicle_detection.launch.py
sed -i '4 i input_path: /root/jpg/car.jpg' $dynamic_vino_sample/param/pipeline_vehicle_detection.yaml && sed -i 's#input_path#  input_path#g' $dynamic_vino_sample/param/pipeline_vehicle_detection.yaml
sed -i 's#StandardCamera#Image#g' $dynamic_vino_sample/param/pipeline_vehicle_detection.yaml


