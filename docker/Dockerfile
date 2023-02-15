# ros2 openvino toolkit env master f1b1ca4d914186a1881b87f103be9c6e910c9d80

ARG ROS_PRE_INSTALLED_PKG
FROM osrf/ros:${ROS_PRE_INSTALLED_PKG}
ARG VERSION

# setting proxy env --option 
# If needed, enable the below ENV setting by correct proxies.
# ENV HTTP_PROXY="your_proxy"
# ENV HTTPS_PROXY="your_proxy"
# ENV FTP_PROXY="your_proxy"

# author information
LABEL author="Jiawei Wu <jiawei.wu@intel.com>"

# default shell type
SHELL ["/bin/bash", "-c"]

# ignore the warning
ARG DEBIAN_FRONTEND=noninteractive
ARG APT_KEY_DONT_WARN_ON_DANGEROUS_USAGE=1
RUN apt-get update && apt-get install --assume-yes apt-utils

# install openvino 2022.3
# https://docs.openvino.ai/2022.3/openvino_docs_install_guides_installing_openvino_apt.html
RUN apt update && apt install --assume-yes curl wget gnupg2 lsb-release
RUN wget https://apt.repos.intel.com/intel-gpg-keys/GPG-PUB-KEY-INTEL-SW-PRODUCTS.PUB && \
apt-key add GPG-PUB-KEY-INTEL-SW-PRODUCTS.PUB && echo "deb https://apt.repos.intel.com/openvino/2022 focal main" | tee /etc/apt/sources.list.d/intel-openvino-2022.list
RUN apt update && apt-cache search openvino && apt install -y openvino-2022.3.0

# install librealsense2
RUN apt-get install -y --no-install-recommends \
software-properties-common 
# https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md
# Make sure you set http-proxy in below commands if your environment needs.
# RUN apt-key adv --keyserver-options http-proxy=your_proxy --keyserver keys.gnupg.net --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || apt-key adv --keyserver-options http-proxy=your_proxy --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE 
RUN apt-key adv --keyserver keyserver.ubuntu.com --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE
RUN add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" -u \ 
&& apt-get install -y --no-install-recommends \
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
RUN pip install openvino-dev[tensorflow2]==2022.1

# build ros2 openvino toolkit
WORKDIR /root
RUN mkdir -p catkin_ws/src
WORKDIR /root/catkin_ws/src
RUN git init && git clone https://github.com/intel/ros2_object_msgs.git \
&& git clone -b ros2 https://github.com/intel/ros2_openvino_toolkit.git
RUN apt-get install ros-${VERSION}-diagnostic-updater
WORKDIR /root/catkin_ws
RUN source /opt/ros/${VERSION}/setup.bash && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

