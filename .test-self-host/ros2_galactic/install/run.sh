#!/bin/bash


cd ~/catkin_ws/ && source /opt/ros/galactic/setup.bash  && source /opt/intel/openvino_2021/bin/setupvars.sh && source ./install/local_setup.bash
# Run the test cases
cd ~/install && source test_pipeline_people.sh
cd ~/install && source test_pipeline_reidentification.sh
#cd ~/install && source test_pipeline_face_reidentification.sh
cd ~/install && source test_pipeline_image.sh
#cd ~/install && source test_pipeline_segmentation.sh
cd ~/install && source test_pipeline_vehicle_detection.sh
cd ~/install && source test_pipeline_person_attributes.launch.sh

# The ERROR caused by close rviz2 graphical ignore Temporarily
#cd ~/catkin_ws/log && sed -i 's#\[ERROR\] \[rviz2-2\]#ignore Temporarily#g' *
