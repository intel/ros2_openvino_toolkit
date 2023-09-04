Prerequisites:
- Ubuntu 20.04 or Ubuntu 22.04 installed
- Docker tool installed
- Realsense Camera inserted
- Dockfile(intel@10.239.89.24:/home/intel/ros2_ov/Dockfile)
- Model:gearbot-model(include labels)(intel@10.239.89.24:/home/intel/ros2_ov/gearbot-model)
	
1. Install docker.
Refer to: Docker_install_guide
	 
2. docker add proxy：
	 
	a. sudo mkdir -p /etc/systemd/system/docker.service.d/
		 
	b. sudo vim /etc/systemd/system/docker.service.d/proxy.conf
		 
	cat /etc/systemd/system/docker.service.d/proxy.conf
	[Service]
	Environment="HTTPS_PROXY=http://child-prc.intel.com:913"
		 
	c. sudo vim /etc/systemd/system/docker.service.d/http-proxy.conf
	root@GNRD:/home/media/GNRD# cat /etc/systemd/system/docker.service.d/http-proxy.conf
	[Service]
	Environment="HTTP_PROXY=http://child-prc.intel.com:913" "HTTPS_PROXY=http://child-prc.intel.com:913"
		 
	sudo systemctl daemon-reload
	sudo systemctl restart docker


	3. sudo docker build --build-arg ROS_PRE_INSTALLED_PKG=foxy-desktop --build-arg VERSION=foxy --build-arg "HTTP_PROXY=http://proxy-prc.intel.com:913" -t ros2_foxy_openvino_202201 .

	4. xhost +
	5. sudo docker run -itd --network=host --privileged -p 40080:80  -p 48080:8080 -p 8888:8888 -p 9000:9000 -p 9090:9090  --device /dev/dri -v /dev/:/dev/ -v /tmp/.X11-unix:/tmp/.X11-unix  -e DISPLAY=unix:0 -e AMR_TYPE=symg --name ros2_foxy_ov_2201 ros2_foxy_openvino_202201 bash
	6. sudo docker exec -it ros2_foxy_openvino_202201 bash
	7. - in Docker Container
	mkdir -p  /opt/openvino_toolkit/models/convert/public/FP32/yolov8n
	
	8. - On Host Machine 
	#Model convert:
	mkdir -p yolov8 && cd yolov8
	pip install ultralytics
	apt install python3.10-venv
	python3 -m venv openvino_env
	source openvino_env/bin/activate
	python -m pip install --upgrade pip
	pip install openvino-dev
	pip install openvino-dev[extras]
	pip install openvino-dev[tensorflow2,onnx]
	#yolo export model=yolov8n.pt format=openvino
	yolo export model=yolov8n.pt format=onnx opset=10
	mo --input_model yolov8n.onnx --use_legacy_frontend
	
	cd ~/gearbolt-model
	yolo export model=best.pt format=onnx opset=10
	mo --input_model best.onnx --use_legacy_frontend
	sudo docker cp yolov8n/ ros2_foxy_openvino_202201:/opt/openvino_toolkit/models/convert/public/FP32/
	sudo docker cp gearbolt-model/ ros2_foxy_openvino_202201:/opt/openvino_toolkit/models/convert/public/
			
	9. - in Docker Container
	apt install vim
	apt install ros-foxy-realsense2-camera
	source /opt/ros/foxy/setup.bash
	source install/setup.bash
	vim /root/catkin_ws/install/openvino_node/share/openvino_node/param/pipeline_object_yolo.yml
		Inputs:[RealSenseCamera]
		model:/opt/openvino_toolkit/models/convert/public/FP32/yolov8n/yolov8n.xml
		Model_type:yolov8
		Left:RealSenseCamera
	ros2 launch openvino_node pipeline_object_yolo.launch.py
	vim /root/catkin_ws/install/openvino_node/share/openvino_node/param/pipeline_segmentation_instence.yml
		Input:[RealSenseCamera]
		model:/opt/openvino_toolkit/models/convert/public/gearbolt-model/best_openvino_model/best.xml
		label:/opt/openvino_toolkit/models/convert/public/gearbolt-model/configs/gearbolt.labels
		Left:RealSenseCamera
	ros2 launch openvino_node pipeline_segmentation_instence.launch.py

