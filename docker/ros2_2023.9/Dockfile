# ros2 openvino toolkit env master f1b1ca4d914186a1881b87f103be9c6e910c9d80

ARG ROS_PRE_INSTALLED_PKG
FROM osrf/ros:${ROS_PRE_INSTALLED_PKG}
#FROM osrf/ros:humble-desktop-full
ARG VERSION

# setting proxy env --option
# If needed, enable the below ENV setting by correct proxies.
ENV http_proxy="http://child-prc.intel.com:913"
ENV https_proxy="http://child-prc.intel.com:913"
ENV no_proxy="127.0.0.1, localhost"
#ENV ftp_proxy="http://child-prc.intel.com:913"
#ENV https_proxy="http://child-prc.intel.com:913"


# default shell type
SHELL ["/bin/bash", "-c"]

# ignore the warning
ARG DEBIAN_FRONTEND=noninteractive
ARG APT_KEY_DONT_WARN_ON_DANGEROUS_USAGE=1
RUN apt-get update && apt-get install --assume-yes apt-utils

# install openvino 2022.1
# https://docs.openvino.ai/2022.1/openvino_docs_install_guides_installing_openvino_apt.html
RUN apt-get update && apt-get install -y curl gnupg2 lsb-release
RUN curl -s https://apt.repos.intel.com/intel-gpg-keys/GPG-PUB-KEY-INTEL-SW-PRODUCTS.PUB | sudo apt-key add -
#RUN echo "deb https://apt.repos.intel.com/openvino/2022 $(lsb_release -cs) main" |tee /etc/apt/sources.list.d/intel-openvino-2022.list
RUN echo "deb https://apt.repos.intel.com/openvino/2023 ubuntu20 main" | sudo tee /etc/apt/sources.list.d/intel-openvino-2023.list
#RUN apt-get update && apt-cache search openvino-2022.1.0
#RUN apt-get install -y openvino-2022.1.0
#RUN ls -lh /opt/intel/openvino_2022
#RUN source /opt/intel/openvino_2022/setupvars.sh
#RUN apt update && apt-cache search openvino && apt install -y openvino-2022.3.1
RUN apt update && apt-cache search openvino && apt install -y openvino

# install librealsense2
RUN apt-get install -y --no-install-recommends \
software-properties-common
# https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md
# Make sure you set http-proxy in below commands if your environment needs.
# RUN apt-key adv --keyserver-options http-proxy=your_proxy --keyserver keys.gnupg.net --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || apt-key adv --keyserver-options http-proxy=your_proxy --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE
#RUN apt-key adv --keyserver keyserver.ubuntu.com --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE
RUN apt-key adv --keyserver-options http-proxy=$http_proxy --keyserver keys.gnupg.net --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || apt-key adv --keyserver-options http-proxy=$http_proxy --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE
RUN add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" -u \
&& apt-get install -y --no-install-recommends  \
librealsense2-dkms \
librealsense2-utils \
librealsense2-dev \
librealsense2-dbg \
libgflags-dev \
libboost-all-dev \
&& rm -rf /var/lib/apt/lists/*

# other dependencies
RUN apt-get update && apt-get install -y python3-pip && python3 -m pip install -U \
numpy \
networkx \
pyyaml \
requests \
&& apt-get install -y --no-install-recommends libboost-all-dev
WORKDIR /usr/lib/x86_64-linux-gnu
RUN pip install --upgrade pip
#RUN pip install openvino-dev[tensorflow2]==2022.1

# build ros2 openvino toolkit
WORKDIR /root
RUN mkdir -p catkin_ws/src
WORKDIR /root/catkin_ws/src
RUN git clone https://github.com/intel/ros2_object_msgs.git \
&& git clone -b ros2 https://github.com/intel/ros2_openvino_toolkit.git
RUN apt-get install ros-${VERSION}-diagnostic-updater
WORKDIR /root/catkin_ws
#RUN source /opt/ros/${VERSION}/setup.bash && source /opt/intel/openvino_2022/setupvars.sh && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
RUN source /opt/ros/${VERSION}/setup.bash  && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release


#install depends deb&&python  packages
RUN apt-get install -y \
  build-essential \
  cmake \
  git \
  libbullet-dev \
  python3-colcon-common-extensions \
  python3-flake8 \
  python3-pip \
  python3-pytest-cov \
  python3-rosdep \
  python3-setuptools \
  python3-vcstool \
  wget
RUN python3 -m pip install -U \
  argcomplete \
  flake8-blind-except \
  flake8-builtins \
  flake8-class-newline \
  flake8-comprehensions \
  flake8-deprecated \
  flake8-docstrings \
  flake8-import-order \
  flake8-quotes \
  pytest-repeat \
  pytest-rerunfailures \
  pytest

RUN apt-get update

#RUN  apt-get install ros-foxy-rosbag2-cpp ros-foxy-rosbag2-transport
#RUN apt-get install libopencv-dev python3-opencv
#RUN apt-get update

RUN wget -qO - https://repositories.intel.com/graphics/intel-graphics.key |  apt-key add -
RUN apt-add-repository   'deb [arch=amd64] https://repositories.intel.com/graphics/ubuntu focal main'
RUN apt-get update
RUN apt-get install -y  intel-opencl-icd   intel-level-zero-gpu level-zero   intel-media-va-driver-non-free libmfx1
RUN apt-get install -y  libigc-dev   intel-igc-cm   libigdfcl-dev   libigfxcmrt-dev   level-zero-dev

RUN RUN apt-get install -y ros-foxy-realsense2-camera

