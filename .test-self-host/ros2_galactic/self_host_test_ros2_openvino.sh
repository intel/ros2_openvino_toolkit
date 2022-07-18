#!/bin/bash

export DISPLAY=:0.0

cases_num=5

docker images | grep ros2_openvino
if [ $? -eq 0 ]
then
    echo "rm the ros2_openvino:01"
    docker rmi -f ros2_openvino:01
    docker rm -f ros2_openvino_for_log
fi

docker ps -a | grep ros2_openvino_for_log
if [ $? -eq 0 ]
then
    docker rm -f ros2_openvino_for_log
fi

cd /home/intel/ros2_openvino_toolkit && docker build -t ros2_openvino:01 .

docker run -i --privileged=true --device=/dev/dri -v $HOME/.Xauthority:$HOME/.Xauthority -e DISPLAY=$DISPLAY --name ros2_openvino_for_log  ros2_openvino:01 bash -c "cd ~/install && ./run.sh"


cd /home/intel/ros2_openvino_toolkit && rm -rf log && docker cp ros2_openvino_for_log:/root/catkin_ws/log .

cd /home/intel/ros2_openvino_toolkit/send_email && python3 sendEmail.py Ros2_Openvino_Test Ros2_Openvino
if [ $? -ne 0 ]
then
    echo "Sending email failed"
    exit -1
fi

docker rm -f ros2_openvino_for_log

cd /home/intel/ros2_openvino_toolkit && grep -rn "ERROR" log/

if [ $? -eq 0 ]
then
    echo "test ros2_openvino_toolkit failed"
    exit -1
fi

result_num=`cd /home/intel/ros2_openvino_toolkit && grep -rn -m 1 "Analyzing Detection results..." log/ | wc -l`
if [ $result_num -ne $cases_num ]
then
    echo "test ros2_openvino_toolkit failed"
    exit -1
fi

