# ros2 openvino toolkit
# from ubuntu:20.04
FROM osrf/ros:galactic-desktop

COPY install /root/install

# build ros2 openvino toolkit
WORKDIR /root

RUN apt update
COPY caffe/output /root/output

RUN cd /root/install && ./install.sh
# RUN cd ~/install && ./install_librealsense2.sh && ./install_openvino.sh  && ./ros2_openvino_install.sh





