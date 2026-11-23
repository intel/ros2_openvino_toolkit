# ros2 openvino toolkit env master f1b1ca4d914186a1881b87f103be9c6e910c9d80

FROM osrf/ros:humble-desktop

# Set non-interactive mode for APT
ENV DEBIAN_FRONTEND=noninteractive

# setting proxy env --option 
# If needed, enable the below ENV setting by correct proxies.
# ENV http_proxy=your_proxy
# ENV https_proxy=your_proxy

# maintainer information
LABEL maintainer="Jayabalaji Sathiyamoorthi <jayabalaji.sathiyamoorthi@intel.com>"

SHELL ["/bin/bash", "-c"]

# install OpenVINO and dependencies
# https://docs.openvino.ai/2025/get-started/install-openvino/install-openvino-apt.html
RUN apt update && \
    apt install -y --no-install-recommends wget curl gnupg2 lsb-release software-properties-common apt-transport-https nano && \ 
    wget -qO- https://apt.repos.intel.com/intel-gpg-keys/GPG-PUB-KEY-INTEL-SW-PRODUCTS.PUB | gpg --dearmor -o /etc/apt/trusted.gpg.d/intel.gpg && \
    echo "deb https://apt.repos.intel.com/openvino ubuntu22 main" > /etc/apt/sources.list.d/intel-openvino.list && \
    apt update && \
    apt-get install -y openvino-2025.4.0 && \
    rm -rf /var/lib/apt/lists/*

# RealSense ROS camera node, used with camera:=realsense
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    ros-humble-realsense2-camera \
    libgflags-dev \
    && rm -rf /var/lib/apt/lists/*

#Install sudo and setup ubuntu user and password to "ubuntu"
RUN apt-get update && apt-get install -y sudo && \
    echo "root:ubuntu" | chpasswd && \
    useradd -m -s /bin/bash ubuntu && \
    echo "ubuntu:ubuntu" | chpasswd && \
    usermod -aG sudo ubuntu && \
    rm -rf /var/lib/apt/lists/*

# Set working directory and Switch to user "ubuntu"
WORKDIR /home/ubuntu
USER ubuntu

# build ros2 openvino toolkit
RUN mkdir -p ros2_ws/src
WORKDIR /home/ubuntu/ros2_ws/src
RUN git clone -b ros2_humble https://github.com/intel/ros2_openvino_toolkit.git 
RUN git clone -b humble https://github.com/ros-perception/vision_opencv.git
WORKDIR /home/ubuntu/ros2_ws
RUN source /opt/ros/humble/setup.bash \
    && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
