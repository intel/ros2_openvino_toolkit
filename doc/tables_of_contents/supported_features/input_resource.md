# Full list of supported Input Resources
|Input Resource Name|Description|
|---|-------------------------------------------|
|StandardCamera|Any RGB camera with USB port supporting. Currently only the first USB camera if many are connected.|
|RealSenseCamera| Intel RealSense RGB-D Camera,directly calling RealSense Camera via librealsense plugin of openCV.|
|RealSenseCameraTopic| any ROS topic which is structured in image message.The topic to be inputted must be remapped to name ```/openvino_toolkit/image_raw```(type [sensor_msgs::msg::Image](https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/msg/Image.msg))|
|Image| Any image file which can be parsed by openCV, such as .png, .jpeg.|
|Video| Any video file which can be parsed by openCV.|