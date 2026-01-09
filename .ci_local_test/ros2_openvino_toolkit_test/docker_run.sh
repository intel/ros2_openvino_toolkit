#!/bin/bash

export DISPLAY=:0
export work_dir=$PWD

function run_container() 
{ 
    if [ -n "$(docker images -q ros2_openvino_docker:01)" ]; then
        echo "The container ros2_openvino_docker:01 image exists"
        docker rmi -f ros2_openvino_docker:01
    fi

    docker ps -a | grep ros2_openvino_container
    if [ -n "$(docker ps -aq -f name=ros2_openvino_container)" ]; then
        echo "The container ros2_openvino_container exists. Removing the container..."
        docker rm -f ros2_openvino_container
    fi

    # Removing ros2_openvino_toolkit git clone from Dockerfile
    # We'll copy it from the host instead (to test local changes)
    cd "$work_dir" && sed -i '/RUN git clone.*ros2_openvino_toolkit/d' Dockerfile
    # add the jpg for test.
    cd "$work_dir" && sed -i '/^WORKDIR \/root\/ros2_ws$/a COPY jpg /root/jpg' Dockerfile || \
    cd "$work_dir" && sed -i '/^WORKDIR \/root\/catkin_ws$/a COPY jpg /root/jpg' Dockerfile

    # Detect ROS distro from Dockerfile
    if grep -q "ros:jazzy" Dockerfile; then
        ROS_DISTRO="jazzy"
        WORKSPACE_DIR="ros2_ws"
    elif grep -q "ros:humble" Dockerfile; then
        ROS_DISTRO="humble"
        WORKSPACE_DIR="ros2_ws"
    elif grep -q "ros:galactic" Dockerfile; then
        ROS_DISTRO="galactic"
        WORKSPACE_DIR="catkin_ws"
    else
        ROS_DISTRO="galactic"
        WORKSPACE_DIR="catkin_ws"
    fi

    cd "$work_dir" && docker build --build-arg ROS_PRE_INSTALLED_PKG=${ROS_DISTRO}-desktop --build-arg VERSION=${ROS_DISTRO}  -t ros2_openvino_docker:01 .
    cd "$work_dir" && docker images
    
    # Create container in detached mode
    docker create -i --privileged=true --device=/dev/dri \
        -v "$work_dir"/ros2_openvino_toolkit:/root/${WORKSPACE_DIR}/src/ros2_openvino_toolkit \
        -v "$HOME"/.Xauthority:/root/.Xauthority \
        -e GDK_SCALE \
        --name ros2_openvino_container \
        ros2_openvino_docker:01 \
        bash
    
    # Copy test files into container
    docker cp "$work_dir"/test_cases ros2_openvino_container:/root/
    
    # Start container and fix permissions inside (critical for GitHub Actions)
    docker start ros2_openvino_container
    docker exec ros2_openvino_container chmod -R a+rx /root/test_cases
    
    # Run tests
    docker exec -i ros2_openvino_container bash -c "cd /root/test_cases && ./run.sh ${ROS_DISTRO}"
    
    # Stop container (keeps it for debugging)
    docker stop ros2_openvino_container
}

if ! run_container; then 
  echo "Test failed"
  exit 1
fi
