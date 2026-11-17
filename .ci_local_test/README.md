

Jenkins file Introduction:

1. The Jenkins machine would scan the ROS2_OpenVINO project regularly.

It would trigger the tests when it detects a PR or other change.


2. The test machine is named robotics-testNUC11; it builds and tests the project as described at the link below.

    The Link: https://github.com/intel/ros2_openvino_toolkit/blob/ros2_jazzy/doc/quick_start

    getting_started_with_ros2_ov2.0.md

3. The test result would be sent by email.

