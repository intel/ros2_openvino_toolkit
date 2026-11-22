# Getting Started (ROS 2 Humble, Ubuntu 22.04)

This guide builds the toolkit and runs a first demo. It targets the `ros2_humble` branch. For other ROS 2 distributions, use the matching branch listed in the [README](../../README.md#ros2-version-supported).

|Step|What you do|
|---|---|
|1|[Install the prerequisites](#1-install-the-prerequisites)|
|2|[Get the sources and install OpenVINO](#2-get-the-sources-and-install-openvino)|
|3|[Build](#3-build)|
|4|[Download the models](#4-download-the-models)|
|5|[Run a demo](#5-run-a-demo)|

To try the toolkit without a local ROS 2 installation, use the Debian packages in a container instead: see [Test in a Container](../../README.md#test-in-a-container).

## 1. Install the prerequisites

|Component|Install|
|---|---|
|ROS 2 Humble|[Install guide](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)|
|Build tools and ROS packages|`sudo apt install python3-colcon-common-extensions ros-humble-cv-bridge ros-humble-librealsense2 ros-humble-yaml-cpp-vendor ros-humble-rviz2 ros-humble-rqt-image-view libopencv-dev`|
|OpenVINO 2025.4|Installed in step 2|

Notes:
* `ros-humble-librealsense2` is needed to build even if you do not own a RealSense camera.
* `ros-humble-rviz2` and `ros-humble-rqt-image-view` are only needed for the visualization options (`viewer:=rviz2` or `viewer:=rqt`).
* The object messages are part of this repository ([object_msgs](../../object_msgs)). Do not clone `ros2_object_msgs` in addition.

## 2. Get the sources and install OpenVINO

```bash
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone https://github.com/intel/ros2_openvino_toolkit -b ros2_humble
sudo ./ros2_openvino_toolkit/script/packaging/setup_openvino_apt.sh
sudo apt install openvino-libraries-dev-2025.4.0
```

The script enables the [Intel APT repository of OpenVINO](https://docs.openvino.ai/2025/get-started/install-openvino/install-openvino-apt.html). OpenVINO 2025.4.0 is the release this branch is built and tested with.

## 3. Build

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

## 4. Download the models

```bash
ros2 run openvino_node download_models.sh
```

This downloads the pre-converted models used by the face / people demos (face detection, age and gender, emotion, head pose) to `~/.local/share/openvino_toolkit/models/intel/`.

|Command|Result|
|---|---|
|`ros2 run openvino_node download_models.sh --all`|All Intel models supported by the script|
|`ros2 run openvino_node download_models.sh face-detection-adas-0001/FP16`|Only the listed `<name>/<precision>` models|

To store the models somewhere else, set `OPENVINO_TOOLKIT_DIR` before downloading **and** before launching:

```bash
export OPENVINO_TOOLKIT_DIR=/data/openvino_toolkit
```

Models that need conversion (for example YOLO) are not covered by this script. See the [YOLOv5](./tutorial_for_yolov5_converted.md), [YOLOv7](./tutorial_for_yolov7_converted.md) and [YOLOv8](./tutorial_for_yolov8_converted.md) tutorials.

## 5. Run a demo

```bash
ros2 launch openvino_node pipeline_image.launch.py
```

This runs face detection, age / gender, emotion and head pose estimation on a sample image and opens `rqt_image_view` with the annotated result.

Useful launch arguments:

|Argument|Example|Description|
|---|---|---|
|`device`|`device:=GPU`|Run all models on this OpenVINO device (`CPU`, `GPU`, `NPU`, `AUTO`, ...). Defaults to the `engine:` values in the YAML file. Available on all sample launch files.|
|`viewer`|`viewer:=none`|`rqt` (default), `rviz2` or `none`. Only on launch files that open a viewer.|
|`yaml_path`|`yaml_path:=/path/to/my.yaml`|Use your own pipeline file. Use absolute paths in it, placeholders are not replaced.|

The device each model actually runs on is printed at startup (`Engine created: requested device=GPU, executing on=GPU.0`) and drawn on the output image next to the FPS.

`GPU` and `NPU` need the Intel user-space drivers of the device, see the OpenVINO configuration guides for [GPU](https://docs.openvino.ai/2025/get-started/install-openvino/configurations/configurations-intel-gpu.html) and [NPU](https://docs.openvino.ai/2025/get-started/install-openvino/configurations/configurations-intel-npu.html).

### Other demos

|Demo|Command|
|---|---|
|Face / people analytics, camera input|`ros2 launch openvino_node pipeline_people.launch.py`|
|Person re-identification, camera input|`ros2 launch openvino_node pipeline_reidentification.launch.py`|
|Object segmentation|`ros2 launch openvino_node pipeline_segmentation.launch.py`|
|Vehicle detection|`ros2 launch openvino_node pipeline_vehicle_detection.launch.py`|
|Person attributes|`ros2 launch openvino_node pipeline_person_attributes.launch.py`|

Each demo needs its own models. If one is missing, the launch prints `WARNING: model not found: <path>` with the exact location it expects. See the [YAML configuration guide](./yaml_configuration_guide.md) to adapt a pipeline.

## Troubleshooting

|Symptom|Cause and fix|
|---|---|
|`WARNING: model not found: ...` at launch|The models are not where the launch file looks. Run the download script, or set `OPENVINO_TOOLKIT_DIR` to the folder that contains `models/`.|
|CMake error `Could not find a package configuration file provided by "OpenVINO"`|The OpenVINO development files are not installed. See step 2.|
|CMake error about `realsense2`|Install `ros-humble-librealsense2`.|
|Symlink "File exists" errors, or packages built against the wrong ROS distribution|The shell still has an old workspace sourced. Open a new terminal, source only `/opt/ros/humble/setup.bash`, and remove the `build/`, `install/` and `log/` folders.|
|Device is shown as `GPU.0`, not `GPU`|Expected. OpenVINO reports the index of the GPU that is used.|
|The system monitor shows about 0% GPU load while running on GPU|OpenVINO uses the GPU compute engine, which many monitors do not show. Use `intel_gpu_top` and look at the Compute row.|
