#!/bin/bash

export DISPLAY=:0
export work_dir=$PWD

function run_container() 
{ 
    if docker images -q ros2_openvino_docker:01 &>/dev/null; then
        echo "The container ros2_openvino_docker:01 image exists"
        docker rmi -f ros2_openvino_docker:01
    fi

    docker ps -a | grep ros2_openvino_container
    if docker ps -aq -f name=ros2_openvino_container; then
        echo "The container ros2_openvino_container exists. Removing the container..."
        docker rm -f ros2_openvino_container
    fi

    # Removing some docker image ..
    # Using jenkins server ros2_openvino_toolkit code instead of git clone code.
    cd "$work_dir" && sed -i '/RUN git clone -b ros2/d' Dockerfile
    # add the jpg for test.
    cd "$work_dir" && sed -i '$i COPY jpg /root/jpg' Dockerfile

    cd "$work_dir" && docker build --build-arg ROS_PRE_INSTALLED_PKG=galactic-desktop --build-arg VERSION=galactic  -t ros2_openvino_docker:01 .
    cd "$work_dir" && docker images
    docker run -i --privileged=true --device=/dev/dri -v "$work_dir"/ros2_openvino_toolkit:/root/catkin_ws/src/ros2_openvino_toolkit  -v "$HOME"/.Xauthority:/root/.Xauthority -e GDK_SCALE  -v "$work_dir"/test_cases:/root/test_cases --name ros2_openvino_container  ros2_openvino_docker:01 bash -c "cd /root/test_cases && ./run.sh galactic"
}

if ! run_container; then 
  echo "Test failed"
  exit 1
fi
