#!/bin/bash

export DISPLAY=:0

export work_dir=$PWD


function run_container() {

    docker images | grep ros2_openvino_docker

    if [ $? -eq 0 ]
    then
        echo "the image of ros2_openvino_docker:01 existence"
        docker rmi -f ros2_openvino_docker:01
    fi

    docker ps -a | grep ros2_openvino_container
    if [ $? -eq 0 ]
    then
        docker rm -f ros2_openvino_container
    fi
    # Using jenkins server ros2_openvino_toolkit code instead of git clone code.
    cd $work_dir && sed -i 's/RUN git clone -b ros2 https/#/g' Dockerfile
    cd $work_dir && docker build --build-arg ROS_VERSION=galactic-desktop --build-arg VERSION=galactic  -t ros2_openvino_docker:01 .
    docker run -i --privileged=true  -v $work_dir/ros2_openvino_toolkit:/root/catkin_ws/src/ros2_openvino_toolkit -v $work_dir/test_cases:/root/test_cases --name ros2_openvino_container  ros2_openvino_docker:01 bash -c "cd /root/test_cases && ./run.sh galactic"

}


run_container
if [ $? -ne 0 ]
then
  echo "Test fail"
  exit -1
fi


