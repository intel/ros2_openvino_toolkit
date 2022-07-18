#!/bin/bash
# install openvino 2021.4
# https://docs.openvinotoolkit.org/latest/openvino_docs_install_guides_installing_openvino_apt.html
apt update && apt install -y curl gnupg2 lsb-release
curl -s https://apt.repos.intel.com/openvino/2021/GPG-PUB-KEY-INTEL-OPENVINO-2021 |apt-key add -
echo "deb https://apt.repos.intel.com/openvino/2021 all main" | tee /etc/apt/sources.list.d/intel-openvino-2021.list
apt update
apt-cache search intel-openvino-dev-ubuntu20
apt-get install -y intel-openvino-dev-ubuntu20-2021.4.752


echo "deb https://apt.repos.intel.com/openvino/2021 all main" | tee /etc/apt/sources.list.d/intel-openvino-2021.list
apt update
apt-cache search openvino
apt-get install -y intel-openvino-dev-ubuntu20-2021.4.752
ls -lh /opt/intel/openvino_2021
source /opt/intel/openvino_2021/bin/setupvars.sh

