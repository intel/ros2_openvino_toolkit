# ros2 openvino toolkit env master f1b1ca4d914186a1881b87f103be9c6e910c9d80

from osrf/ros:foxy-desktop

MAINTAINER Cong Liu congliu0913@126.com

SHELL ["/bin/bash", "-c"]

# install openvino 2021.3
# https://docs.openvinotoolkit.org/latest/openvino_docs_install_guides_installing_openvino_apt.html
RUN apt update && apt install curl gnupg2 lsb-release
RUN curl -s https://apt.repos.intel.com/openvino/2021/GPG-PUB-KEY-INTEL-OPENVINO-2021 |apt-key add -
RUN echo "deb https://apt.repos.intel.com/openvino/2021 all main" | tee /etc/apt/sources.list.d/intel-openvino-2021.list
RUN apt update
RUN apt-cache search openvino
RUN apt-get install -y intel-openvino-dev-ubuntu20-2021.3.394 
RUN ls -lh /opt/intel/openvino_2021
RUN source /opt/intel/openvino_2021/bin/setupvars.sh 

# install librealsense2
# https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md
RUN apt-key adv --keyserver keys.gnupg.net --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE 
#RUN add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo focal main" -u
RUN echo 'deb https://librealsense.intel.com/Debian/apt-repo focal main' | sudo tee /etc/apt/sources.list.d/realsense-public.list
RUN apt-get update && apt-get install -y librealsense2-dev librealsense2
RUN dpkg -l |grep realsense

# build ros2 openvino toolkit
WORKDIR /root
RUN mkdir -p ros2_ws/src
WORKDIR /root/ros2_ws/src
RUN git clone https://github.com/intel/ros2_object_msgs.git
RUN git clone https://github.com/intel/ros2_openvino_toolkit.git
WORKDIR /root/ros2_ws
RUN source /opt/ros/foxy/setup.bash && source /opt/intel/openvino_2021/bin/setupvars.sh && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
