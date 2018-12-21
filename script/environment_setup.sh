#!/bin/bash
set -euo pipefail

echo "Please Enter Your Password:"
stty -echo
read ROOT_PASSWD
stty echo

basedir=$PWD
echo "Begin Environment Setup"

system_ver=`cat /etc/lsb-release | grep -i "DISTRIB_RELEASE" | cut -d "=" -f2`

#Get Config Parameters
CLEAN=`cat modules.conf | grep 'clean'`
CLEAN=${CLEAN##*=}
echo "Set CLEAN to $CLEAN"

ROS2_SRC=`cat modules.conf | grep 'ros2_src'`
ROS2_SRC=${ROS2_SRC##*=}
echo "Set ROS2_SRC to $ROS2_SRC"

OPENCV=`cat modules.conf | grep 'opencv'`
OPENCV=${OPENCV##*=}
echo "Set OPENCV to $OPENCV"

OPENCL=`cat modules.conf | grep 'opencl'`
OPENCL=${OPENCL##*=}
echo "Set OPENCL to $OPENCL"

DLDT=`cat modules.conf | grep 'dldt'`
DLDT=${DLDT##*=}
echo "Set DLDT to $DLDT"

MODEL_ZOO=`cat modules.conf | grep 'model_zoo'`
MODEL_ZOO=${MODEL_ZOO##*=}
echo "Set MODEL_ZOO to $MODEL_ZOO"

OTHER_DEPENDENCY=`cat modules.conf | grep 'other_dependency'`
OTHER_DEPENDENCY=${OTHER_DEPENDENCY##*=}
echo "Set OTHER_DEPENDENCY to $OTHER_DEPENDENCY"


# Clean Existing Directories
if [ "$CLEAN" == "1" ]; then
  echo "===================Cleaning...===================================="
  
  rm -rf ~/code
  rm -rf ~/ros2_ws
  echo $ROOT_PASSWD | sudo -S rm -rf /opt/openvino_toolkit
  if [[ $system_ver = "16.04" && -L "/usr/lib/x86_64-linux-gnu/libboost_python3.so" ]]; then
    echo $ROOT_PASSWD | sudo -S rm /usr/lib/x86_64-linux-gnu/libboost_python3.so
  fi
fi

# Setup ROS2 from src
if [ "$ROS2_SRC" == "1" ]; then
  echo "===================Installing ROS2 from Source...======================="
  
  echo $ROOT_PASSWD | sudo -S locale-gen en_US en_US.UTF-8
  echo $ROOT_PASSWD | sudo -S update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
  export LANG=en_US.UTF-8

  echo $ROOT_PASSWD | sudo -S apt-get update && sudo apt-get install -y curl
  curl http://repo.ros2.org/repos.key | sudo apt-key add -
  echo $ROOT_PASSWD | sudo -S sh -c 'echo "deb [arch=amd64,arm64] http://repo.ros2.org/ubuntu/main `lsb_release -cs` main" > /etc/apt/sources.list.d/ros2-latest.list'
  echo $ROOT_PASSWD | sudo -S apt-get update && sudo apt-get install -y build-essential cmake git python3-colcon-common-extensions python3-pip python-rosdep python3-vcstool wget

# install some pip packages needed for testing
  echo $ROOT_PASSWD | sudo -S -H python3 -m pip install -U argcomplete flake8 flake8-blind-except flake8-builtins flake8-class-newline flake8-comprehensions flake8-deprecated flake8-docstrings flake8-import-order flake8-quotes pytest-repeat pytest-rerunfailures

  python3 -m pip install -U pytest pytest-cov pytest-runner setuptools
  echo $ROOT_PASSWD | sudo -S apt-get install --no-install-recommends -y libasio-dev libtinyxml2-dev

  mkdir -p ~/ros2_ws/src
  cd ~/ros2_ws
  wget https://raw.githubusercontent.com/ros2/ros2/master/ros2.repos
  vcs-import src < ros2.repos

  echo $ROOT_PASSWD | sudo -S apt install -y --no-install-recommends \
        libeigen3-dev \
        libtinyxml2-dev \
        qtbase5-dev \
        libfreetype6 \
        libfreetype6-dev \
        libyaml-dev \
        libconsole-bridge-dev \
        libcurl4-openssl-dev \
        curl \
        libxaw7-dev \
        libcppunit-dev \
        libpcre3-dev \
        cmake \
        clang-format \
        libgl1-mesa-dev \
        libglu1-mesa-dev \
        python3-flake8 \
        pyflakes3 \
        cppcheck \
        libxrandr-dev \
        libqt5core5a \
        libqt5widgets5 \
        python-mock \
        python3-pkg-resources \
        libxml2-utils \
        libopencv-dev \
        libtinyxml-dev \
        python3-yaml \
        uncrustify \
        libqt5opengl5 \
        python3-mock \
        python3-pytest \
        openssl \
        python3-pep8 \
        libassimp-dev \
        libpoco-dev \
        pydocstyle \
        zlib1g-dev \
        python3-empy \
        libx11-dev \
        libqt5gui5 \
        python3-setuptools \
        python3-catkin-pkg-modules \
        pkg-config
  colcon build --symlink-install
fi

# Setup OpenCV
if [ "$OPENCV" == "1" ]; then
  echo "===================Installing OpenCV3 from Source...======================="
  
  echo $ROOT_PASSWD | sudo -S apt-get install -y build-essential
  echo $ROOT_PASSWD | sudo -S apt-get install -y cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev
  echo $ROOT_PASSWD | sudo -S apt-get install -y python-dev python-numpy libtbb2 libtbb-dev libpng-dev libtiff-dev libjasper-dev libdc1394-22-dev

  if [ $system_ver = "18.04" ]; then
    echo $ROOT_PASSWD | sudo -S add-apt-repository "deb http://security.ubuntu.com/ubuntu xenial-security main"
    echo $ROOT_PASSWD | sudo apt update
    echo $ROOT_PASSWD | sudo apt install libjasper1 libjasper-dev
  else
    echo $ROOT_PASSWD | sudo -S apt-get install libjasper-dev 
  fi

  mkdir -p ~/code && cd ~/code
  echo "begin clone opencv"
  git clone https://github.com/opencv/opencv.git
  git clone https://github.com/opencv/opencv_contrib.git
  echo "finish clone opencv"

  cd ~/code/opencv
  git checkout 3.4.0
  cd ~/code/opencv_contrib
  git checkout 3.4.0

  cd ~/code/opencv
  mkdir build && cd build
  cmake -DOPENCV_EXTRA_MODULES_PATH=$HOME/code/opencv_contrib/modules -DCMAKE_INSTALL_PREFIX=/usr/local -DBUILD_opencv_cnn_3dobj=OFF ..
  make -j4
  echo $ROOT_PASSWD | sudo -S make install
  echo $ROOT_PASSWD | sudo -S ldconfig
  
  echo "==== END install OpenCV ===="
fi

# Setup OpenCL
if [ "$OPENCL" == "1" ]; then
  echo "===================Installing OpenCL Driver for GPU...======================="
  
  mkdir -p ~/code && cd ~/code
  wget http://registrationcenter-download.intel.com/akdlm/irc_nas/11396/SRB5.0_linux64.zip
  unzip SRB5.0_linux64.zip -d SRB5.0_linux64
  cd SRB5.0_linux64
  echo $ROOT_PASSWD | sudo -S apt-get install xz-utils
  mkdir intel-opencl
  tar -C intel-opencl -Jxf intel-opencl-r5.0-63503.x86_64.tar.xz
  tar -C intel-opencl -Jxf intel-opencl-devel-r5.0-63503.x86_64.tar.xz
  tar -C intel-opencl -Jxf intel-opencl-cpu-r5.0-63503.x86_64.tar.xz
  echo $ROOT_PASSWD | sudo -S cp -R intel-opencl/* /
  echo $ROOT_PASSWD | sudo -S ldconfig
  
  echo "==== END install OpenCL ===="
fi

# Setup DLDT
if [ "$DLDT" == "1" ]; then
  echo "===================Installing Deep Learning Deployment Toolkit...======================="
 
  if [[ -f /etc/lsb-release ]]; then
    echo $ROOT_PASSWD | sudo -S -E apt update
    echo $ROOT_PASSWD | sudo -S -E apt-get install -y \
            build-essential \
            cmake \
            curl \
            wget \
            libssl-dev \
            ca-certificates \
            git \
            libboost-regex-dev \
            gcc-multilib \
            g++-multilib \
            libgtk2.0-dev \
            pkg-config \
            unzip \
            automake \
            libtool \
            autoconf \
            libcairo2-dev \
            libpango1.0-dev \
            libglib2.0-dev \
            libgtk2.0-dev \
            libswscale-dev \
            libavcodec-dev \
            libavformat-dev \
            libgstreamer1.0-0 \
            gstreamer1.0-plugins-base \
            libusb-1.0-0-dev \
            libopenblas-dev
    if [ $system_ver = "18.04" ]; then
            echo $ROOT_PASSWD | sudo -S -E apt-get install -y libpng-dev
    else
            echo $ROOT_PASSWD | sudo -S -E apt-get install -y libpng12-dev
    fi
  fi
  mkdir -p  ~/code && cd ~/code
  git clone https://github.com/opencv/dldt.git
  cd dldt/inference-engine/
  git checkout 2018_R4
  git submodule init
  git submodule update --recursive
  mkdir build && cd build
  cmake -DCMAKE_BUILD_TYPE=Release ..
  make -j8
  echo $ROOT_PASSWD | sudo -S mkdir -p /opt/openvino_toolkit
  echo $ROOT_PASSWD | sudo -S ln -s ~/code/dldt /opt/openvino_toolkit/dldt
  
  echo "==== END install DLDT ===="
fi

# Setup open_model_zoo
if [ "$MODEL_ZOO" == "1" ]; then
  echo "===================Installing Open Model Zoo...======================="
  
  mkdir -p ~/code && cd ~/code
  git clone https://github.com/opencv/open_model_zoo.git
  cd open_model_zoo/demos/
  git checkout 2018_R4
  mkdir build && cd build
  cmake -DCMAKE_BUILD_TYPE=Release /opt/openvino_toolkit/dldt/inference-engine
  make -j8
  echo $ROOT_PASSWD | sudo -S mkdir -p /opt/openvino_toolkit
  echo $ROOT_PASSWD | sudo -S ln -s ~/code/open_model_zoo /opt/openvino_toolkit/open_model_zoo
  
  echo "==== END install open_model_zoo ===="
fi

# Setup other dependencies
if [ "$OTHER_DEPENDENCY" == "1" ]; then
  echo "===================Setting UP OTHER_DEPENDENCY DEPENDENCY...======================="
  
  echo $ROOT_PASSWD | sudo -S apt-get install -y libssl-dev libusb-1.0-0-dev pkg-config libgtk-3-dev
  echo $ROOT_PASSWD | sudo -S apt-get install -y libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev
  
  pip3 install numpy
  if [ $system_ver = "16.04" ]; then
     echo $ROOT_PASSWD | sudo -S apt-get install -y --no-install-recommends libboost-all-dev
     cd /usr/lib/x86_64-linux-gnu
     echo $ROOT_PASSWD | sudo -S ln -s libboost_python-py35.so libboost_python3.so
  elif [ $system_ver = "18.04" ]; then
     echo $ROOT_PASSWD | sudo -S apt-get install -y --no-install-recommends libboost-all-dev
     echo $ROOT_PASSWD | sudo -S apt install libboost-python1.62.0
   fi

   echo "==== END install other dependencies ===="
fi

echo "Environment setup successfully"
