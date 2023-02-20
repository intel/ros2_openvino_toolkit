#!/bin/bash

export ros2_branch=$1
if [[ $1 == '' ]]
then
    export ros2_branch=galactic
else
    export ros2_branch=$1
fi
source /root/test_cases/config.sh $ros2_branch

cd /root/catkin_ws && colcon build --symlink-install
cd /root/catkin_ws && source ./install/local_setup.bash

apt-get update
# apt-get install -y ros-$ros2_branch-diagnostic-updater
apt-get install python3-defusedxml
apt-get install -y python3-pip
pip3 install XTestRunner

cd /root/test_cases && ./ros2_openvino_tool_model_download.sh

cd /root/test_cases/unittest && ./adjust_test_yaml.sh
echo "===cat pipeline_people.yaml"
cat /root/catkin_ws/install/dynamic_vino_sample/share/dynamic_vino_sample/param/pipeline_people.yaml

echo "===cat pipeline_image.yaml"
cat /root/catkin_ws/install/dynamic_vino_sample/share/dynamic_vino_sample/param/pipeline_image.yaml

echo "===cat pipeline_person_attributes.yaml"
cat /root/catkin_ws/install/dynamic_vino_sample/share/dynamic_vino_sample/param/pipeline_person_attributes.yaml

echo "===cat pipeline_reidentification.yaml"
cat /root/catkin_ws/install/dynamic_vino_sample/share/dynamic_vino_sample/param/pipeline_reidentification.yaml


cd /root/test_cases/unittest && python3 run_all.py
if [ $? -ne 0 ]
then
        exit -1
fi

