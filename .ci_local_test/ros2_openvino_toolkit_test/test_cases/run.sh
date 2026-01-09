#!/bin/bash

export ros2_branch=$1
if [[ $1 == '' ]]
then
    export ros2_branch=galactic
else
    export ros2_branch=$1
fi
#shellcheck source=/dev/null
source /root/test_cases/config.sh "$ros2_branch"

# Clean build artifacts to avoid symlink conflicts
cd ${WORKSPACE_DIR} && rm -rf build install log
cd ${WORKSPACE_DIR} && colcon build
# shellcheck source=/dev/null 
source ./install/local_setup.bash

apt-get update
# apt-get install -y ros-$ros2_branch-diagnostic-updater
apt-get install -y python3-defusedxml
apt-get install -y python3-pip
pip3 install XTestRunner==1.5.0 --break-system-packages

cd /root/test_cases && ./ros2_openvino_tool_model_download.sh
mkdir -p /root/test_cases/log
echo "===cat pipeline_people_ci.yaml"
cat ${WORKSPACE_DIR}/install/openvino_node/share/openvino_node/param/pipeline_people_ci.yaml

cd /root/test_cases/unittest && python3 run_all.py
result=$?
#echo "cat segmentation maskrcnn"
#cat /root/test_cases/log/pipeline_segmentation_maskrcnn_test_ci.log

echo "Test ENV:" && df -h && free -g
if [ $result -ne 0 ]
then
        exit 1
fi
