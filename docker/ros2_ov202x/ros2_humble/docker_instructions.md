# ROS2 OpenVINO Toolkit Docker Image

This repository contains a Dockerfile for building a Docker image with ROS2 and the OpenVINO toolkit. The image is based on the `osrf/ros:humble-desktop` base image and includes additional tools and libraries for working with OpenVINO and librealsense.

## Features

- ROS2 Humble Desktop
- OpenVINO Toolkit (version configurable supports 2025.0.0, 2024.x, 2023.3)
- librealsense2
- User setup with `sudo` privileges
  
## Prerequisites  

- Docker installed on your system
- Network connection & correct proxy settings for downloading base images and dependencies

## Building the Docker Image
  
To build the Docker image, use the following command. You can specify the OpenVINO version using the `--build-arg` option. The default version is `2025.0.0`.

```bash
docker build --build-arg  OPENVINO_VERSION=2025.0.0  -t  ros2_openvino_image  .

```

Replace `2025.0.0` with the desired OPENVINO version if needed, for example to build `OpenVINO 2024.6`, use the following command,

```bash
docker build --build-arg  OPENVINO_VERSION=2024.6.0  -t  ros2_openvino_image  .

```  

if you are behind a proxy server use the following command,

```bash
docker build --build-arg  "HTTP_PROXY=http://<your-proxy.com>:<your-port>" --build-arg  OPENVINO_VERSION=2024.6.0  -t  ros2_openvino_image  .

```
## Download the Models from OpenVINO Model Zoo (OMZ)
OMZ tools are provided for downloading and converting OMZ models in OpenVINO 202x versions.</br>
Refer to: [OMZ-tool_guide](https://pypi.org/project/openvino-dev/)
Refer to: [OMZ Models](https://github.com/openvinotoolkit/open_model_zoo/tree/2024.6.0/models)

```bash
omz_downloader --print_all
```

* Download the optimized Intermediate Representation (IR) of model (execute once), for example:

```bash
omz_downloader --list <ros2-ws>/src/ros2_openvino_toolkit/data/model_list/download_model.lst -o /opt/openvino_toolkit/models/
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

To run the Docker container with the volumes containing models and images, use the following command:

```bash
docker run -itd -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix -v /dev:/dev -v /opt:/opt -v /data:/data --privileged=true --name ros2_openvino_container ros2_openvino_image
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

* -v /opt:/opt

    ###### Mounts the /opt directory from the host to the container, allowing the container to access the models installed from OpenVINO Model Zoo.

* -v /data:/data

    ###### Mounts the /data directory from the host to the container, that contains the images and labels. Replace this with the folder of your images and labels.

* --privileged=true

    ###### Grants the container extended privileges, allowing it to access all devices on the host and perform operations that are typically restricted. This is necessary for certain applications that require direct hardware access.

* --name ros2_openvino_container

    ###### Assigns the name ros2_openvino_container to the running container, making it easier to reference in subsequent Docker commands.

* ros2_openvino_image

    ###### Specifies the name of the Docker image to run. Replace this with the actual name of your built image if it differs.
 
### Notes

* Ensure that your host's X11 server is configured to allow connections from the Docker container. You may need to run xhost +local:docker on the host to permit this.

* The --privileged flag provides the container with elevated permissions, which can pose security risks. Use it only when necessary and understand the implications.
