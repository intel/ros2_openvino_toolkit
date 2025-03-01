# ros2 openvino toolkit env master f1b1ca4d914186a1881b87f103be9c6e910c9d80

from osrf/ros:humble-desktop

# Set non-interactive mode for APT
ENV DEBIAN_FRONTEND=noninteractive

# setting proxy env --option 
# If needed, enable the below ENV setting by correct proxies.
# ENV http_proxy=your_proxy
# ENV https_proxy=your_proxy

# maintainer information
LABEL maintainer="Jayabalaji Sathiyamoorthi <jayabalaji.sathiyamoorthi@intel.com>"

SHELL ["/bin/bash", "-c"]

# install openvino 2021.4
# https://docs.openvinotoolkit.org/latest/openvino_docs_install_guides_installing_openvino_apt.html
RUN apt update && apt install wget curl gnupg2 lsb-release
RUN wget -qO- https://apt.repos.intel.com/intel-gpg-keys/GPG-PUB-KEY-INTEL-SW-PRODUCTS.PUB | sudo gpg --dearmor -o /etc/apt/trusted.gpg.d/intel.gpg
RUN echo "deb https://apt.repos.intel.com/openvino/2025 ubuntu22 main" | sudo tee /etc/apt/sources.list.d/intel-openvino-2025.list
RUN echo "deb https://apt.repos.intel.com/openvino/2024 ubuntu22 main" | sudo tee /etc/apt/sources.list.d/intel-openvino-2024.list
RUN echo "deb https://apt.repos.intel.com/openvino/2023 ubuntu22 main" | sudo tee /etc/apt/sources.list.d/intel-openvino-2023.list
RUN apt update

RUN apt update \
    && apt-cache search openvino \ 
    && apt-get install -y openvino-2025.0.0
#    && apt-get install -y openvino-2024.2.0
#    && apt-get install -y openvino-2023.3.0

# workaround to fetch librealsense2-dkms via ECI repo 
#RUN bash -c 'echo "deb [trusted=yes] http://wheeljack.ch.intel.com/apt-repos/ECI/$(lsb_release -sc) isar main" > /etc/apt/sources.list.d/eci.list'
#RUN bash -c 'echo "deb-src [trusted=yes] http://wheeljack.ch.intel.com/apt-repos/ECI/$(lsb_release -sc) isar main" >> /etc/apt/sources.list.d/eci.list'
#RUN bash -c 'echo -e "Package: *\nPin: origin wheeljack.ch.intel.com\nPin-Priority: 1000" > /etc/apt/preferences.d/isar'
#RUN bash -c 'echo "Acquire::http::Proxy \"http://proxy-dmz.intel.com:912\"; Acquire::https::Proxy \"http://proxy-dmz.intel.com:912\"; Acquire::http::Proxy::wheeljack.ch.intel.com DIRECT;" >> /etc/apt/apt.conf.d/proxy.conf'

#RUN apt-get update \
#    && apt-get install -y librealsense2-dkms 

# install librealsense2
# Reference : https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md
RUN apt-get install -y --no-install-recommends \
    software-properties-common \
    apt-transport-https

RUN mkdir -p /etc/apt/keyrings
#RUN curl -sSf https://librealsense.intel.com/Debian/librealsense.pgp | tee /etc/apt/keyrings/librealsense.pgp > /dev/null

#RUN echo "deb [signed-by=/etc/apt/keyrings/librealsense.pgp] https://librealsense.intel.com/Debian/apt-repo `lsb_release -cs` main" | \
#    tee /etc/apt/sources.list.d/librealsense.list
#RUN apt-get update
#RUN apt-get install -y --no-install-recommends \
#    librealsense2-dkms \ # Houston, this is not available in kernel 6.8  
#    librealsense2-utils \
#    librealsense2-dev \
#    librealsense2-dbg \
#    && rm -rf /var/lib/apt/lists/*

RUN curl -sSf https://librealsense.intel.com/Debian/librealsense.pgp | sudo tee /etc/apt/keyrings/librealsense.pgp > /dev/null && \
echo "deb [signed-by=/etc/apt/keyrings/librealsense.pgp] https://librealsense.intel.com/Debian/apt-repo `lsb_release -cs` main" | \
sudo tee /etc/apt/sources.list.d/librealsense.list &&  apt-get update &&  apt-get install -y --no-install-recommends \
librealsense2-utils \
librealsense2-dev \
librealsense2-dbg \
libgflags-dev \
&& rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y sudo

# Set root password to "ubuntu"
RUN echo "root:ubuntu" | chpasswd

# Create a new user "ubuntu" with password "ubuntu"
RUN useradd -m -s /bin/bash ubuntu && \
    echo "ubuntu:ubuntu" | chpasswd && \
    usermod -aG sudo ubuntu

# Set working directory
WORKDIR /home/ubuntu

# Switch to user "ubuntu"
USER ubuntu
# build ros2 openvino toolkit

# build ros2 openvino toolkit
WORKDIR /home/ubuntu
RUN mkdir -p ros2_ws/src
WORKDIR /home/ubuntu/ros2_ws/src
RUN git clone https://github.com/intel/ros2_object_msgs.git
RUN git clone -b ros2 https://github.com/jb-balaji/ros2_openvino_toolkit.git
RUN git clone -b humble https://github.com/ros-perception/vision_opencv.git
WORKDIR /home/ubuntu/ros2_ws
#RUN source /opt/ros/humble/setup.bash \
#    && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
    
