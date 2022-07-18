#!/bin/bash


sed -i 's#ImageWindow, RosTopic, RViz##g' /root/catkin_ws/install/dynamic_vino_sample/share/dynamic_vino_sample/param/pipeline_segmentation.yaml
sed -i 's#RViz##g' /root/catkin_ws/install/dynamic_vino_sample/share/dynamic_vino_sample/param/pipeline_segmentation.yaml


tmux new -d -s tmux_pipeline_segmentation
tmux send -t tmux_pipeline_segmentation "cd ~/catkin_ws && ros2 launch dynamic_vino_sample pipeline_segmentation.launch.py >> log/pipeline_segmentation_test.log" ENTER

sleep 15

cd ~/catkin_ws/ && cat log/pipeline_segmentation_test.log
echo "Test pipeline_segmentation Done"
tmux kill-session -t tmux_pipeline_segmentation
