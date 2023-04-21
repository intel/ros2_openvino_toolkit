#/bin/bash

if [[ $1 == '' ]]
then
    export ros2_branch=galactic
else
    export ros2_branch=$1
fi

export dynamic_vino_sample=/root/catkin_ws/install/openvino_node/share/openvino_node


source /opt/ros/$ros2_branch/setup.bash
