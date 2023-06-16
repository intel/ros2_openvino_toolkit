ARG ROS_PRE_INSTALLED_PKG
FROM osrf/ros:${ROS_PRE_INSTALLED_PKG}
ARG VERSION

SHELL ["/bin/bash", "-c"]

# ignore the warning
ARG DEBIAN_FRONTEND=noninteractive
ARG APT_KEY_DONT_WARN_ON_DANGEROUS_USAGE=1

# install openvino 2022.3
# https://docs.openvino.ai/2022.3/openvino_docs_install_guides_installing_openvino_apt.html
RUN apt update && apt install --assume-yes curl wget gnupg2 lsb-release \
&& wget https://apt.repos.intel.com/intel-gpg-keys/GPG-PUB-KEY-INTEL-SW-PRODUCTS.PUB && \
apt-key add GPG-PUB-KEY-INTEL-SW-PRODUCTS.PUB && echo "deb https://apt.repos.intel.com/openvino/2022 focal main" | tee /etc/apt/sources.list.d/intel-openvino-2022.list && \
apt update && apt-cache search openvino && apt install -y openvino-2022.3.0


# install librealsense2
RUN apt-get install -y --no-install-recommends \
software-properties-common \
&& apt-key adv --keyserver keyserver.ubuntu.com --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE \
&&  add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" -u \
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
&& apt-get install --assume-yes apt-utils \
&& apt-get install -y --no-install-recommends libboost-all-dev \
&& apt-get install ros-${VERSION}-diagnostic-updater \
&& pip install --upgrade pip
RUN cd /usr/lib/x86_64-linux-gnu && ln -sf libboost_python-py36.so libboost_python37.so
COPY jpg /root/jpg
# build ros2 openvino toolkit
RUN cd /root && mkdir -p catkin_ws/src && cd /root/catkin_ws/src \
&& git clone https://github.com/intel/ros2_object_msgs.git
WORKDIR /root/catkin_ws/src
RUN git clone -b ros2 https://github.com/intel/ros2_openvino_toolkit.git
WORKDIR /root/catkin_ws
RUN source /opt/ros/${VERSION}/setup.bash && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

