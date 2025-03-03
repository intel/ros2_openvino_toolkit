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

# Define build argument for OpenVINO version (2025.0.0 , 2024.6.0, 2023.3.0)
ARG OPENVINO_VERSION=2025.0.0

# Extract the major version from the full version string
ENV OPENVINO_MAJOR_VERSION=${OPENVINO_VERSION%%.*}

# install OpenVINO and dependencies
# https://docs.openvinotoolkit.org/latest/openvino_docs_install_guides_installing_openvino_apt.html
RUN apt update && \
    apt install -y --no-install-recommends wget curl gnupg2 lsb-release software-properties-common apt-transport-https nano && \ 
    wget -qO- https://apt.repos.intel.com/intel-gpg-keys/GPG-PUB-KEY-INTEL-SW-PRODUCTS.PUB | gpg --dearmor -o /etc/apt/trusted.gpg.d/intel.gpg && \
    echo "deb https://apt.repos.intel.com/openvino/${OPENVINO_MAJOR_VERSION} ubuntu22 main" | tee /etc/apt/sources.list.d/intel-openvino-${OPENVINO_MAJOR_VERSION}.list && \
    apt update && \
    apt-get install -y openvino-${OPENVINO_VERSION} && \
    rm -rf /var/lib/apt/lists/*

# install librealsense2
# Reference : https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md
RUN mkdir -p /etc/apt/keyrings && \
    curl -sSf https://librealsense.intel.com/Debian/librealsense.pgp | tee /etc/apt/keyrings/librealsense.pgp > /dev/null && \
    echo "deb [signed-by=/etc/apt/keyrings/librealsense.pgp] https://librealsense.intel.com/Debian/apt-repo `lsb_release -cs` main" | \
    tee /etc/apt/sources.list.d/librealsense.list && \ 
    apt-get update && \ 
    apt-get install -y --no-install-recommends \
    librealsense2-utils \
    librealsense2-dev \
    librealsense2-dbg \
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
RUN git clone https://github.com/intel/ros2_object_msgs.git
#change the branch before merge
RUN git clone -b openvino_2025 https://github.com/jb-balaji/ros2_openvino_toolkit.git 
RUN git clone -b humble https://github.com/ros-perception/vision_opencv.git
WORKDIR /home/ubuntu/ros2_ws
RUN source /opt/ros/humble/setup.bash \
    && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
