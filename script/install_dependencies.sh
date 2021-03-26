#!/bin/bash

#librealsense
sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --keyserver-options http-proxy=http://child-prc.intel.com:913 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE
sudo add-apt-repository "deb http://realsense-hw-public.s3.amazonaws.com/Debian/apt-repo focal main" -u
sudo apt-get install librealsense2-dev

#apt key for openvino
link: https://docs.openvinotoolkit.org/latest/openvino_docs_install_guides_installing_openvino_apt.html

curl -s https://apt.repos.intel.com/openvino/2021/GPG-PUB-KEY-INTEL-OPENVINO-2021 | sudo apt-key add -
echo "deb https://apt.repos.intel.com/openvino/2021 all main" | sudo tee /etc/apt/sources.list.d/intel-openvino-2021.list
sudo apt-cache search intel-openvino-dev-ubuntu20

