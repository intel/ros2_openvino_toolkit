# ROS2 OpenVINO Toolkit Docker Image

This repository contains a Dockerfile for building a Docker image with ROS2 and the OpenVINO toolkit. The image is based on the `osrf/ros:jazzy-desktop` base image and includes additional tools and libraries for working with OpenVINO and librealsense.

## Features

- ROS2 Jazzy Desktop
- OpenVINO Toolkit version 2025.4.0
- librealsense2
- User setup with `sudo` privileges
  
## Prerequisites  

- Docker installed on your system
- Network connection & correct proxy settings for downloading base images and dependencies

## Building the Docker Image
  
To build the Docker image, use the following command. The image installs OpenVINO `2025.4.0`.

```bash
docker build -t  ros2_openvino_image  .

```

if you are behind a proxy server use the following command,

```bash
docker build --build-arg  "HTTP_PROXY=http://<your-proxy.com>:<your-port>" -t  ros2_openvino_image  .

```
## Download the Models
Run the download script inside the container (see [Running the Docker Container](#running-the-docker-container)). It stores the pre-converted Intel models in `$OPENVINO_TOOLKIT_DIR/models`, or in `~/.local/share/openvino_toolkit/models` when the variable is not set. Models that need conversion (for example YOLO) are not covered by the script, see the tutorials in [doc/quick_start](../../../doc/quick_start).

```bash
source /root/ros2_ws/install/setup.bash
ros2 run openvino_node download_models.sh        # models of the face / people demos
ros2 run openvino_node download_models.sh --all  # all Intel models supported by the script
``` 

## Running the Docker Container 

To run the Docker container, use the following command:
 
```bash
docker run -it --rm --name  ros2_openvino_container  ros2_openvino_image

``` 

To run the Docker container with GUI support, use the following command:

```bash
docker run -itd  -e  DISPLAY=$DISPLAY -v  /tmp/.X11-unix:/tmp/.X11-unix  -v  /dev:/dev  --privileged=true  --name  ros2_openvino_container  ros2_openvino_image

```

To keep the downloaded models on the host and to use your own images, mount the corresponding folders:

```bash
docker run -itd -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix -v /dev:/dev -v $HOME/openvino_models:/opt/openvino_toolkit -e OPENVINO_TOOLKIT_DIR=/opt/openvino_toolkit -v /data:/data --privileged=true --name ros2_openvino_container ros2_openvino_image
```

#### Explanation of Options

* -itd: Combines three flags:

        -i: Runs the container in interactive mode, keeping the standard input open.
		-t: Allocates a pseudo-TTY, which is useful for interactive applications.
		-d: Runs the container in detached mode, allowing it to run in the background.

* -e DISPLAY=$DISPLAY

    ###### Sets the DISPLAY environment variable inside the container to match the host's DISPLAY variable. This is       necessary for GUI applications to display on the host's screen.

* -v /tmp/.X11-unix:/tmp/.X11-unix

    ###### Mounts the X11 Unix socket from the host to the container. This is required for GUI applications to communicate with the X server on the host.

* -v /dev:/dev

    ###### Mounts the /dev directory from the host to the container, allowing the container to access hardware devices. This is often necessary for applications that interact with hardware, such as cameras or GPUs.

* -v $HOME/openvino_models:/opt/openvino_toolkit -e OPENVINO_TOOLKIT_DIR=/opt/openvino_toolkit

    ###### Mounts a host directory as the models directory of the toolkit, so that the models downloaded with download_models.sh are kept between runs.

* -v /data:/data

    ###### Mounts the /data directory from the host to the container, that contains the images and labels. Replace this with the folder of your images and labels.

* --privileged=true

    ###### Grants the container extended privileges, allowing it to access all devices on the host and perform operations that are typically restricted. This is necessary for certain applications that require direct hardware access.

* --name ros2_openvino_container

    ###### Assigns the name ros2_openvino_container to the running container, making it easier to reference in subsequent Docker commands.

* ros2_openvino_image

    ###### Specifies the name of the Docker image to run. Replace this with the actual name of your built image if it differs.
 
### GPU and NPU

`-v /dev:/dev --privileged=true` already exposes the Intel GPU (`/dev/dri`) and NPU (`/dev/accel`) of the host. To pass only these devices, replace both options with `--device /dev/dri --device /dev/accel`.

The image does not contain the user-space drivers of these devices. Install them inside the container:

* GPU: `apt install intel-opencl-icd`, or a newer driver as described in the [OpenVINO GPU configuration](https://docs.openvino.ai/2025/get-started/install-openvino/configurations/configurations-intel-gpu.html).
* NPU: the [Intel NPU driver](https://github.com/intel/linux-npu-driver).

Then select the device at launch, for example `ros2 launch openvino_node pipeline_image.launch.py device:=GPU`. The device in use is printed at startup.

### Notes

* Ensure that your host's X11 server is configured to allow connections from the Docker container. You may need to run xhost +local:docker on the host to permit this.

* The --privileged flag provides the container with elevated permissions, which can pose security risks. Use it only when necessary and understand the implications.
