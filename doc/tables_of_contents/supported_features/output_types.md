# Output Types
>> The inference results can be output in several types. One or more types can be enabled for any inference pipeline.
## Topic Publishing
>> Specific topic(s) can be generated and published according to the given inference functionalities.</br>

|Inference|Published Topic|
|---|---|
|People Detection|```/ros2_openvino_toolkit/face_detection```([object_msgs:msg:ObjectsInBoxes](https://github.com/intel/ros2_object_msgs/blob/master/msg/ObjectsInBoxes.msg))|
|Emotion Recognition|```/ros2_openvino_toolkit/emotions_recognition```([people_msgs:msg:EmotionsStamped](https://github.com/intel/ros2_openvino_toolkit/blob/master/people_msgs/msg/EmotionsStamped.msg))|/ros2_openvino_toolkit/face_detection(object_msgs:msg:ObjectsInBoxes)
|Age and Gender Recognition|```/ros2_openvino_toolkit/age_genders_Recognition```([people_msgs:msg:AgeGenderStamped](https://github.com/intel/ros2_openvino_toolkit/blob/master/people_msgs/msg/AgeGenderStamped.msg))|
|Head Pose Estimation|```/ros2_openvino_toolkit/headposes_estimation```([people_msgs:msg:HeadPoseStamped](https://github.com/intel/ros2_openvino_toolkit/blob/master/people_msgs/msg/HeadPoseStamped.msg))|
|Object Detection|```/ros2_openvino_toolkit/detected_objects```([object_msgs::msg::ObjectsInBoxes](https://github.com/intel/ros2_object_msgs/blob/master/msg/ObjectsInBoxes.msg))|
|Object Segmentation|```/ros2_openvino_toolkit/segmented_obejcts```([people_msgs::msg::ObjectsInMasks](https://github.com/intel/ros2_openvino_toolkit/blob/devel/people_msgs/msg/ObjectsInMasks.msg))|
|Person Reidentification|```/ros2_openvino_toolkit/reidentified_persons```([people_msgs::msg::ReidentificationStamped](https://github.com/intel/ros2_openvino_toolkit/blob/devel/people_msgs/msg/ReidentificationStamped.msg))|
|Face Reidenfication|```/ros2_openvino_toolkit/reidentified_faces```([people_msgs::msg::ReidentificationStamped](https://github.com/intel/ros2_openvino_toolkit/blob/devel/people_msgs/msg/ReidentificationStamped.msg))|
|Vehicle Detection|```/ros2_openvino_toolkit/detected_license_plates```([people_msgs::msg::VehicleAttribsStamped](https://github.com/intel/ros2_openvino_toolkit/blob/devel/people_msgs/msg/VehicleAttribsStamped.msg))|
|Vehicle License Detection|```/ros2_openvino_toolkit/detected_license_plates```([people_msgs::msg::LicensePlateStamped](https://github.com/intel/ros2_openvino_toolkit/blob/devel/people_msgs/msg/LicensePlateStamped.msg))|

## Image View Window
>> The original image and the inference results are rendered together and shown in a CV window.
## RViz Showing
>> The Rendered image (rendering inference results into the original image) was transformed into sensor_msgs::msg::Image topic, that can be shown in RViz application.
- RViz Published Topic
```/ros2_openvino_toolkit/image_rviz```([sensor_msgs::msg::Image](https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/msg/Image.msg))

## Service
>> Several ROS2 Services are created, expecting to be used in client/server mode, especially when synchronously getting inference results for a given image frame or when managing inference pipeline's lifecycle.</br>

- **Face Detection or Object Detection for a given Image file**

|Inference|Service|
|---|---|
|Object Detection Service|```/detect_object```([object_msgs::srv::DetectObject](https://github.com/intel/ros2_object_msgs/blob/master/srv/DetectObject.srv))|
|Face Detection Service|```/detect_face```([object_msgs::srv::DetectObject](https://github.com/intel/ros2_object_msgs/blob/master/srv/DetectObject.srv))|
|Age Gender Detection Service|```/detect_age_gender```([people_msgs::srv::AgeGender](https://github.com/intel/ros2_openvino_toolkit/blob/devel/people_msgs/srv/AgeGender.srv))|
|Headpose Detection Service|```/detect_head_pose```([people_msgs::srv::HeadPose](https://github.com/intel/ros2_openvino_toolkit/blob/devel/people_msgs/srv/HeadPose.srv))|
|Emotion Detection Service|```/detect_emotion```([people_msgs::srv::Emotion](https://github.com/intel/ros2_openvino_toolkit/blob/devel/people_msgs/srv/Emotion.srv))|

- **Inference Pipeline Lifecycle Management**
   - Create new pipeline
   - Start/Stop/Pause a pipeline
   - Get pipeline list or status

