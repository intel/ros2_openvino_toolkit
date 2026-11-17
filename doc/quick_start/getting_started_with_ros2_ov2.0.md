# Getting Started (ROS 2 Lyrical, Ubuntu 26.04)

This guide builds the toolkit and runs a first demo. It targets the `ros2_lyrical` branch. For other ROS 2 distributions, use the matching branch listed in the [README](../../README.md#ros2-version-supported).

|Step|What you do|
|---|---|
|1|[Install the prerequisites](#1-install-the-prerequisites)|
|2|[Get the sources](#2-get-the-sources)|
|3|[Build](#3-build)|
|4|[Download the models](#4-download-the-models)|
|5|[Run a demo](#5-run-a-demo)|

## 1. Install the prerequisites

|Component|Install|
|---|---|
|ROS 2 Lyrical|[Install guide](https://docs.ros.org/en/lyrical/Installation/Ubuntu-Install-Debs.html)|
|Build tools and ROS packages|`sudo apt install python3-colcon-common-extensions ros-lyrical-cv-bridge ros-lyrical-librealsense2 ros-lyrical-yaml-cpp-vendor ros-lyrical-rviz2 ros-lyrical-rqt-image-view libopencv-dev`|
|OpenVINO 2026.1 (runtime)|`python3 -m venv ~/openvino_env && ~/openvino_env/bin/pip install openvino`|

Notes:
* `ros-lyrical-librealsense2` is needed to build even if you do not own a RealSense camera.
* `ros-lyrical-rviz2` and `ros-lyrical-rqt-image-view` are only needed for the visualization options (`viewer:=rviz2` or `viewer:=rqt`).
* Do not activate the OpenVINO virtual environment while building. Its Python does not contain the ROS build tools. The build only needs the path to OpenVINO (see step 3).
* Any other OpenVINO installation works as well, as long as `OpenVINO_DIR` points to its CMake files (for the apt or archive packages this is `<install_dir>/runtime/cmake`).

## 2. Get the sources

```bash
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone https://github.com/intel/ros2_openvino_toolkit -b ros2_lyrical
git clone https://github.com/intel/ros2_object_msgs
```

## 3. Build

```bash
cd ~/ros2_ws
source /opt/ros/lyrical/setup.bash
export OpenVINO_DIR=$(~/openvino_env/bin/python3 -c "import openvino, os; print(os.path.join(os.path.dirname(openvino.__file__), 'cmake'))")
colcon build
source install/setup.bash
```

`OpenVINO_DIR` is only needed at build time. The OpenVINO library path is embedded in the binaries, so you do not need to activate the virtual environment to run the demos.

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
|CMake error `Could not find a package configuration file provided by "OpenVINO"`|`OpenVINO_DIR` is not set. See step 3.|
|CMake error about `realsense2`|Install `ros-lyrical-librealsense2`.|
|Symlink "File exists" errors, or packages built against the wrong ROS distribution|The shell still has an old workspace sourced. Open a new terminal, source only `/opt/ros/lyrical/setup.bash`, and remove the `build/`, `install/` and `log/` folders.|
|Device is shown as `GPU.0`, not `GPU`|Expected. OpenVINO reports the index of the GPU that is used.|
|The system monitor shows about 0% GPU load while running on GPU|OpenVINO uses the GPU compute engine, which many monitors do not show. Use `intel_gpu_top` and look at the Compute row.|
