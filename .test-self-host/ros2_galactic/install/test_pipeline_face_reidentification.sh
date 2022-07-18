#!/bin/bash


sed -i 's#ImageWindow, RosTopic, RViz##g' /root/catkin_ws/install/dynamic_vino_sample/share/dynamic_vino_sample/param/pipeline_face_reidentification.yaml


tmux new -d -s tmux_pipeline_face_reidentification
tmux send -t tmux_pipeline_face_reidentification "cd ~/catkin_ws && ros2 launch dynamic_vino_sample pipeline_face_reidentification.launch.py >> log/pipeline_face_reidentification_test.log" ENTER

sleep 15
cd ~/catkin_ws/ &&sed -i 's#\[ERROR\] \[rviz2-2\]#ignore it temporary#g' log/pipeline_face_reidentification_test.log
cd ~/catkin_ws/ && cat log/pipeline_face_reidentification_test.log
echo "Test pipeline_face_reidentification Done"
tmux kill-session -t tmux_pipeline_face_reidentification
