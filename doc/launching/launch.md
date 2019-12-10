# Launching
## 1. Setup Environment
Please refer to this [guide](https://github.com/intel/ros2_openvino_toolkit/blob/devel/doc/launching/set_environment.md) for details.

**NOTE:** Configure *once* the Neural Compute Stick USB Driver by following between instructions, in case you have a NCS or NCS2 in hand. 
   ```bash
   cd ~/Downloads
   SUBSYSTEM=="usb", ATTRS{idProduct}=="2150", ATTRS{idVendor}=="03e7", GROUP="users", MODE="0666",   ENV{ID_MM_DEVICE_IGNORE}="1"
   SUBSYSTEM=="usb", ATTRS{idProduct}=="2485", ATTRS{idVendor}=="03e7", GROUP="users", MODE="0666", ENV{ID_MM_DEVICE_IGNORE}="1"
   SUBSYSTEM=="usb", ATTRS{idProduct}=="f63b", ATTRS{idVendor}=="03e7", GROUP="users", MODE="0666", ENV{ID_MM_DEVICE_IGNORE}="1"
   EOF
   sudo cp 97-usbboot.rules /etc/udev/rules.d/
   sudo udevadm control --reload-rules
   sudo udevadm trigger
   sudo ldconfig
   rm 97-usbboot.rules
   ```
## 2. Launch Program
### Topic
Each inference listed in [section Inference Implementations](https://github.com/intel/ros2_openvino_toolkit/blob/devel/doc/tables_of_contents/supported_features/Supported_features.md#inference-implementations) is created default launching configurations( xxx.launch.py) in OpenVINO Sample package. You can follow the utility of ROS2 launch instruction to launch them. For example:
   ```bash
   ros2 launch dynamic_vino_sample pipeline_object.launch.py
   ```

The full list of xxx.launch.py is shown in below tabel:

|Download Models|Launch File|Description|
|---|---|---|
|[Object Detection](https://github.com/intel/ros2_openvino_toolkit/blob/devel/doc/inferences/Object_Detection.md)|pipeline_object.launch.py|Launching file for **Object Detection**, by default mobilenet_ssd model and standard USB camera are used.|
|[Face Detection](https://github.com/intel/ros2_openvino_toolkit/blob/devel/doc/inferences/Face_Detection.md)|pipeline_people.launch.py|Launching file for **Face Detection**, also including **Age/Gender Recognition, HeadPose Estimation, and Emotion Recognition**.|
|[Object Segmentation](https://github.com/intel/ros2_openvino_toolkit/blob/devel/doc/inferences/Object_Segmentation.md)|pipeline_segmentation.launch.py|Launching file for **Object Segmentation**.| 
|[Person Re-Identification](https://github.com/intel/ros2_openvino_toolkit/blob/devel/doc/inferences/People_Reidentification.md)|pipeline_person_reid.launch.py|Launching file for **Person Re-Identification**.| 
|[Face Re-Identification](https://github.com/intel/ros2_openvino_toolkit/blob/devel/doc/inferences/Face_Reidentification.md)|pipeline_face_reid.launch.py|Launching file for **Face Segmentation**, in which **Face Landmark Detection** is included.| 
|[Vehicle Detection](https://github.com/intel/ros2_openvino_toolkit/blob/devel/doc/inferences/Vehicle_Detection.md)|pipeline_vehicle_detection.launch.py|Launching file for **vehicle detection**, in which **license plate recognition** is included.|

### Service
See [service Page](https://github.com/intel/ros2_openvino_toolkit/blob/devel/doc/launching/service.md) for detailed launching instructions.
