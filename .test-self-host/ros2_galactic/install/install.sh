#!/bin/bash

cd ~/install && source config.sh 

cd ~/install && source install_librealsense2.sh

cd ~/install && source install_openvino.sh 

cd ~/install && source ros2_openvino_build.sh

cd ~/install && source ros2_openvino_dependence_download.sh

