#!/bin/bash

apt update && apt install -y vim python3-pip && pip3 install networkx
apt-get install ros-galactic-diagnostic-updater
mkdir -p ~/catkin_ws/src

cd ~/catkin_ws/src && git clone $ros2_openvino_toolkit -b galactic
cd ~/catkin_ws/src && git clone $ros2_object_msgs
cd ~/catkin_ws/src && git clone $realsense_ros -b ros2
cd ~/catkin_ws/src && git clone $vision_opencv -b ros2
#cd ~/catkin_ws && rm -rf build  install  log
source /opt/ros/galactic/setup.bash && source /opt/intel/openvino_2021/bin/setupvars.sh && cd ~/catkin_ws && colcon build --symlink-install
cd ~/catkin_ws && source ./install/local_setup.bash




