#!/bin/bash


sed -i 's#ImageWindow, RViz, RosTopic##g' /root/catkin_ws/install/dynamic_vino_sample/share/dynamic_vino_sample/param/pipeline_person_attributes.yaml
sed -i 's#ImageWindow, RosTopic, RViz##g' /root/catkin_ws/install/dynamic_vino_sample/share/dynamic_vino_sample/param/pipeline_person_attributes.yaml


tmux new -d -s tmux_pipeline_person_attributes
tmux send -t tmux_pipeline_person_attributes "cd ~/catkin_ws && ros2 launch dynamic_vino_sample pipeline_person_attributes.launch.py >> log/pipeline_person_attributes_test.log" ENTER

sleep 15
cd ~/catkin_ws/ && sed -i 's#\[ERROR\] \[rviz2-2\]#ignore it temporary#g' log/pipeline_person_attributes_test.log
cd ~/catkin_ws/ && cat log/pipeline_person_attributes_test.log
echo "Test pipeline_person_attributes Done"
tmux kill-session -t tmux_pipeline_person_attributes
