#!/bin/bash


mkdir -p /opt/openvino_toolkit/ros2_openvino_toolkit/data/images/
cp -f ~/catkin_ws/src/ros2_openvino_toolkit/data/images/sample_faces.jpg /opt/openvino_toolkit/ros2_openvino_toolkit/data/images/

sed -i ':a;N;$!ba;s#to/be/set/xxx.labels#/opt/openvino_toolkit/models/face_detection/output/intel/face-detection-adas-0001/FP16/face-detection-adas-0001.labels#1'  /root/catkin_ws/install/dynamic_vino_sample/share/dynamic_vino_sample/param/pipeline_image.yaml

sed -i ':a;N;$!ba;s#to/be/set/xxx.labels#/opt/openvino_toolkit/models/emotions-recognition/output/intel/emotions-recognition-retail-0003/FP32/emotions-recognition-retail-0003.labels#2'  /root/catkin_ws/install/dynamic_vino_sample/share/dynamic_vino_sample/param/pipeline_image.yaml

sed -i 's#ImageWindow, RosTopic, RViz##g' /root/catkin_ws/install/dynamic_vino_sample/share/dynamic_vino_sample/param/pipeline_image.yaml


tmux new -d -s tmux_pipeline_image
tmux send -t tmux_pipeline_image "cd ~/catkin_ws && ros2 launch dynamic_vino_sample pipeline_image.launch.py >> log/pipeline_image_test.log" ENTER

sleep 15

cd ~/catkin_ws/ && sed -i 's#\[ERROR\] \[rviz2-2\]#ignore it temporary#g' log/pipeline_image_test.log
cd ~/catkin_ws/ && cat log/pipeline_image_test.log

echo "Test pipeline_image Done"
tmux kill-session -t tmux_pipeline_image

