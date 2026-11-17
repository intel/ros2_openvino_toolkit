#/bin/bash

if [[ $1 == '' ]]
then
    export ros2_branch=galactic
else
    export ros2_branch=$1
fi

# Detect workspace directory based on ROS distro
if [[ "$ros2_branch" == "jazzy" ]] || [[ "$ros2_branch" == "humble" ]]; then
    export WORKSPACE_DIR=/root/ros2_ws
else
    export WORKSPACE_DIR=/root/catkin_ws
fi

export dynamic_vino_sample=${WORKSPACE_DIR}/install/openvino_node/share/openvino_node


source /opt/ros/$ros2_branch/setup.bash
