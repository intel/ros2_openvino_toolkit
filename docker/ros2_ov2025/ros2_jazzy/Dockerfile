# ROS 2 OpenVINO Toolkit Environment (master f1b1ca4d914186a1881b87f103be9c6e910c9d80)

FROM osrf/ros:jazzy-desktop

# Set non-interactive mode for APT
ENV DEBIAN_FRONTEND=noninteractive

# setting proxy env --option 
# If needed, enable the below ENV setting by correct proxies.
# ENV http_proxy=your_proxy
# ENV https_proxy=your_proxy

# Maintainer information
LABEL maintainer="Jayabalaji Sathiyamoorthi <jayabalaji.sathiyamoorthi@intel.com>"

SHELL ["/bin/bash", "-c"]

# Install dependencies, OpenVINO, and librealsense2
RUN apt update && \
    apt install -y --no-install-recommends \
    curl wget gnupg2 lsb-release nano software-properties-common \
    apt-transport-https libyaml-cpp-dev ros-jazzy-libyaml-vendor \
    libssl-dev libusb-1.0-0-dev libudev-dev pkg-config \
    libgtk-3-dev git cmake build-essential nano && \
    # Add OpenVINO GPG key and repository
    wget -qO- https://apt.repos.intel.com/intel-gpg-keys/GPG-PUB-KEY-INTEL-SW-PRODUCTS.PUB | gpg --dearmor -o /etc/apt/trusted.gpg.d/intel.gpg && \
    echo "deb https://apt.repos.intel.com/openvino ubuntu24 main" | tee /etc/apt/sources.list.d/intel-openvino.list && \
    apt update && \
    apt-get install -y openvino-2025.2.0 && \
    rm -rf /var/lib/apt/lists/*

# Build librealsense2
WORKDIR /root/
RUN git clone --branch v2.55.1 --single-branch https://github.com/IntelRealSense/librealsense.git && \
    cd librealsense && \
    mkdir build && \
    cd build && \
    cmake ../ && \
    make uninstall && make clean && make -j7 && make install && \
    rm -rf /root/librealsense

# build ros2 openvino toolkit
WORKDIR /root
RUN mkdir -p ros2_ws/src
WORKDIR /root/ros2_ws/src
RUN git clone https://github.com/intel/ros2_object_msgs.git
#change the branch before merge
RUN git clone -b ros2_jazzy https://github.com/jb-balaji/ros2_openvino_toolkit.git
RUN git clone -b humble https://github.com/ros-perception/vision_opencv.git
WORKDIR /root/ros2_ws
RUN source /opt/ros/jazzy/setup.bash \
    && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
