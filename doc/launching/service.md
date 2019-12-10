# Service
## Download Models
### Object Detection Service
* See [object detection download model](https://github.com/intel/ros2_openvino_toolkit/blob/devel/doc/inferences/Object_Detection.md#mobilenet-ssd) section for detailed instructions.

### People Detection Service
* See [People Detection download model](https://github.com/intel/ros2_openvino_toolkit/blob/devel/doc/inferences/Face_Detection.md#opensource-version) section for detaild instructions.

## Launching
* run object detection service sample code input from Image  
  Run image processing service:
	```bash
	ros2 launch dynamic_vino_sample image_object_server.launch.py
	```
  Run example application with an absolute path of an image on another console:
	```bash
	ros2 run dynamic_vino_sample image_object_client /opt/openvino_toolkit/ros2_openvino_toolkit/data/images/car.png
	```
* run face detection service sample code input from Image  
  Run image processing service:
	```bash
	ros2 launch dynamic_vino_sample image_people_server.launch.py
	```
  Run example application with an absolute path of an image on another console:
	```bash
	ros2 run dynamic_vino_sample image_people_client /opt/openvino_toolkit/ros2_openvino_toolkit/data/images/team.jpg
	```
