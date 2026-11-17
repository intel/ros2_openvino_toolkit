# ROS 2 OpenVINO Toolkit environment for ROS 2 Lyrical (Ubuntu 26.04) and OpenVINO 2026.1
#
# Build from the repository root so that the checked-out sources are used:
#   docker build -f docker/ros2_ov2026/ros2_lyrical/Dockerfile -t ros2_openvino_lyrical .

FROM osrf/ros:lyrical-desktop

# Set non-interactive mode for APT
ENV DEBIAN_FRONTEND=noninteractive

# setting proxy env --option
# If needed, enable the below ENV setting by correct proxies.
# ENV http_proxy=your_proxy
# ENV https_proxy=your_proxy

LABEL maintainer="Jayabalaji Sathiyamoorthi <jayabalaji.sathiyamoorthi@intel.com>"

SHELL ["/bin/bash", "-c"]

# Build tools, ROS packages and GPU/NPU user-space runtimes
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    curl git nano cmake build-essential python3-venv \
    python3-colcon-common-extensions libopencv-dev \
    ros-lyrical-cv-bridge ros-lyrical-librealsense2 ros-lyrical-yaml-cpp-vendor \
    ros-lyrical-rqt-image-view \
    intel-opencl-icd libze-intel-gpu1 libze1 && \
    rm -rf /var/lib/apt/lists/*

# OpenVINO runtime from PyPI. The library path is embedded into the binaries at
# build time, so the virtual environment does not need to be activated at run time.
RUN python3 -m venv /opt/openvino_env && \
    /opt/openvino_env/bin/pip install --no-cache-dir openvino

# Models are stored here. Mount a host directory to keep them between runs.
ENV OPENVINO_TOOLKIT_DIR=/opt/openvino_toolkit
RUN mkdir -p ${OPENVINO_TOOLKIT_DIR}

# Build ros2 openvino toolkit from the local sources
WORKDIR /root/ros2_ws/src
RUN git clone https://github.com/intel/ros2_object_msgs.git
COPY . /root/ros2_ws/src/ros2_openvino_toolkit

WORKDIR /root/ros2_ws
RUN source /opt/ros/lyrical/setup.bash && \
    export OpenVINO_DIR=$(/opt/openvino_env/bin/python3 -c "import openvino, os; print(os.path.join(os.path.dirname(openvino.__file__), 'cmake'))") && \
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

# Source ROS and the workspace in every interactive shell
RUN echo "source /opt/ros/lyrical/setup.bash" >> /root/.bashrc && \
    echo "source /root/ros2_ws/install/setup.bash" >> /root/.bashrc

CMD ["/bin/bash"]
