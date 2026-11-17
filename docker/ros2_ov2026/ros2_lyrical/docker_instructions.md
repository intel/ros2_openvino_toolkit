# ROS 2 Lyrical OpenVINO Toolkit Docker Image

Dockerfile for ROS 2 Lyrical (Ubuntu 26.04) with OpenVINO 2026.1. The image is based on `osrf/ros:lyrical-desktop` and builds the toolkit from the sources of this repository.

## Features

- ROS 2 Lyrical Desktop
- OpenVINO 2026.1 runtime (pip, in `/opt/openvino_env`)
- librealsense2 (`ros-lyrical-librealsense2`)
- Intel GPU user-space drivers (OpenCL and Level Zero)

## Prerequisites

- Docker installed on your system
- Network connection and correct proxy settings for downloading base images and dependencies

## Build the image

Run the build from the **repository root**, so that your checked-out sources are copied into the image:

```bash
docker build -f docker/ros2_ov2026/ros2_lyrical/Dockerfile -t ros2_openvino_lyrical .
```

Behind a proxy:

```bash
docker build --build-arg "HTTP_PROXY=http://<your-proxy.com>:<your-port>" --build-arg "HTTPS_PROXY=http://<your-proxy.com>:<your-port>" \
  -f docker/ros2_ov2026/ros2_lyrical/Dockerfile -t ros2_openvino_lyrical .
```

## Run the container

Models are stored in `/opt/openvino_toolkit` (`OPENVINO_TOOLKIT_DIR`). Mount a host directory there to keep them between runs.

```bash
xhost +local:docker
docker run -it --rm --name ros2_openvino_container \
  -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v $HOME/openvino_models:/opt/openvino_toolkit \
  --device /dev/dri --group-add video --group-add render \
  ros2_openvino_lyrical
```

|Option|Purpose|
|---|---|
|`-e DISPLAY` and `-v /tmp/.X11-unix`|Show GUI windows (`rqt_image_view`, RViz) on the host|
|`-v $HOME/openvino_models:/opt/openvino_toolkit`|Persist the downloaded models on the host|
|`--device /dev/dri`|Give the container access to the Intel GPU|
|`--group-add video --group-add render`|Allow the container user to use the GPU device nodes|

For a RealSense or USB camera add `--device /dev/video0` (or `-v /dev:/dev --privileged`, which grants broad host access, so use it only when needed). The Intel NPU is available through `--device /dev/accel`.

## Download the models and run a demo

Inside the container:

```bash
ros2 run openvino_node download_models.sh
ros2 launch openvino_node pipeline_image.launch.py
```

Use `device:=GPU` or `device:=NPU` to select the OpenVINO device, for example `ros2 launch openvino_node pipeline_image.launch.py device:=GPU`. The device in use is printed at startup and drawn next to the FPS. See the [getting started guide](../../../doc/quick_start/getting_started_with_ros2_ov2.0.md) for all demos and launch arguments.

Models that need conversion (YOLO, SSD-lite, ...) are not downloaded by the script. See the YOLO tutorials in `doc/quick_start`.

## Notes

- `xhost +local:docker` allows the container to open windows on your X server. Revert it with `xhost -local:docker` when you are done.
- If no GPU is found inside the container, check that the host has the Intel GPU driver loaded and that `/dev/dri/renderD*` exists.
