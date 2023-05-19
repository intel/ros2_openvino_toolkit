# System requirements
Only Windows 10 is supported.

# Installing prerequisites
## Install Chocolatey
Chocolatey is a package manager for Windows, install it by following their installation instructions:
https://chocolatey.org/

You’ll use Chocolatey to install some other developer tools.

## Install Python
Open a Command Prompt and type the following to install Python via Chocolatey:
```
choco install -y python --version 3.8.3
```
Only python 3.8 is supported.

## Install Visual C++ Redistributables
Open a Command Prompt and type the following to install them via Chocolatey:
```
choco install -y vcredist2013 vcredist140
```

## Install OpenSSL
Download the ```Win64 OpenSSL v1.1.1n``` OpenSSL installer from [this page](https://slproweb.com/products/Win32OpenSSL.html). Scroll to the bottom of the page and download ```Win64 OpenSSL v1.1.1n```. Don’t download the Win32 or Light versions, or the v3.X.Y installers.

Run the installer with default parameters, as the following commands assume you used the default installation directory.

This command sets an environment variable that persists over sessions:
```
setx /m OPENSSL_CONF "C:\Program Files\OpenSSL-Win64\bin\openssl.cfg"
```

You will need to append the OpenSSL-Win64 bin folder to your PATH. You can do this by clicking the Windows icon, typing “Environment Variables”, then clicking on “Edit the system environment variables”. In the resulting dialog, click “Environment Variables”, then click “Path” on the bottom pane, finally click “Edit” and add the path below.
```
C:\Program Files\OpenSSL-Win64\bin\
```

## Install Visual Studio
Install Visual Studio 2019. Make sure that the Visual C++ features are installed.</br>
An easy way to make sure they’re installed is to select the Desktop development with C++ workflow during the install.</br>
Make sure that no C++ CMake tools are installed by unselecting them in the list of components to be installed.

## Install OpenCV
You should install [OpenCV against specific version of OpenVINO](https://github.com/opencv/opencv/wiki/BuildOpenCV4OpenVINO#approach-2-build-opencv-against-specific-version-of-openvino).</br>
Before  build OpenCV, make sure you have installed Microsoft Visual Studio, cmake, OpenVINO and Intel® Media SDK for Windows.
* Setup environment variables to detect OpenVINO before:
```
call "C:\Program Files (x86)\<OPENVINO_INSTALL_DIR>\bin\setupvars.bat"
```
* Copy OpenCV repository:
```
git clone --recurse-submodules https://github.com/opencv/opencv.git
```
* Create build directory and enter into it:
```
mkdir "build-opencv" && cd "build-opencv"
```
* Setup MSVC environment by running vcvars64.bat
Compile and install OpenCV:
OpenCV package is available at build-opencv/install directory.
To compile application that uses OpenCV, the following environment variables should be specified:
```
set "OpenCV_DIR=<OpenCV_INSTALL_DIR>\cmake"
set "PATH=<OpenCV_INSTALL_DIR>\bin;%PATH%"
set "PYTHONPATH=<OpenCV_INSTALL_DIR>\python;%PYTHONPATH%"
```
Assuming you install it to C:\opencv, type the following on a Command Prompt (requires Admin privileges):
```
setx /m OpenCV_DIR C:\opencv
```
Since you are using a precompiled ROS version, we have to tell it where to find the OpenCV libraries. You have to extend the PATH variable to ```C:\opencv\x64\vc16\bin```.

## Install dependencies
There are a few dependencies not available in the Chocolatey package database. In order to ease the manual installation process, we provide the necessary Chocolatey packages.

As some chocolatey packages rely on it, we start by installing CMake.
```
choco install -y cmake
```
You will need to append the CMake bin folder ```C:\Program Files\CMake\bin``` to your PATH.

Please download [these](https://github.com/ros2/choco-packages/releases/tag/2022-03-15) packages from this GitHub repository.
* asio.1.12.1.nupkg
* bullet.3.17.nupkg
* cunit.2.1.3.nupkg
* eigen-3.3.4.nupkg
* tinyxml-usestl.2.6.2.nupkg
* tinyxml2.6.0.0.nupkg

Once these packages are downloaded, open an administrative shell and execute the following command:
```
choco install -y -s <PATH\TO\DOWNLOADS\> asio cunit eigen tinyxml-usestl tinyxml2 bullet
```
Please replace ```<PATH\TO\DOWNLOADS>``` with the folder you downloaded the packages to.

First upgrade pip and setuptools:
```
python -m pip install -U pip setuptools==59.6.0
```
Now install some additional python dependencies:
```
python -m pip install -U catkin_pkg cryptography empy importlib-metadata lark==1.1.1 lxml matplotlib netifaces numpy opencv-python PyQt5 pillow psutil pycairo pydot pyparsing==2.4.7 pyyaml rosdistro
```
## Install Qt5
Download the [5.12.X offline installer](https://www.qt.io/offline-installers) from Qt’s website. Run the installer. Make sure to select the ```MSVC 2017 64-bit``` component under the ```Qt``` -> ```Qt 5.12.12``` tree.

Finally, in an administrator ```cmd.exe``` window set these environment variables. The commands below assume you installed it to the default location of ```C:\Qt```.
```
setx /m Qt5_DIR C:\Qt\Qt5.12.12\5.12.12\msvc2017_64
setx /m QT_QPA_PLATFORM_PLUGIN_PATH C:\Qt\Qt5.12.12\5.12.12\msvc2017_64\plugins\platforms
```
**Note:**  This path might change based on the installed MSVC version, the directory Qt was installed to, and the version of Qt installed.

* RQt dependencies
To run rqt_graph you need to [download](https://graphviz.org/download/) and install [Graphviz](https://graphviz.gitlab.io/). The installer will ask if to add graphviz to PATH, choose to either add it to the current user or all users.

## Install additional DDS implementations (optional)
If you would like to use another DDS or RTPS vendor besides the default, Fast DDS.

# Downloading ROS 2
Go to the releases page: https://github.com/ros2/ros2/releases

Download the latest package for Windows, e.g., ```ros2-humble-*-windows-release-amd64.zip```.

**Note:** There may be more than one binary download option which might cause the file name to differ.

Unpack the zip file somewhere (we’ll assume ```C:\dev\ros2_humble```).

## ROS2 Environment setup
* Start a command shell and source the ROS 2 setup file to set up the workspace:
```
call "C:\dev\ros2_humble\ros2-windows\local_setup.bat"
```
It is normal that the previous command, if nothing else went wrong, outputs “The system cannot find the path specified.” exactly once.

* Try some examples
In a command shell, set up the ROS 2 environment as described above and then run a C++ talker:
```
ros2 run demo_nodes_cpp talker
```
Start another command shell and run a Python listener:
```
ros2 run demo_nodes_py listener
```
You should see the talker saying that it’s Publishing messages and the listener saying I heard those messages. This verifies both the C++ and Python APIs are working properly. 

# Install OpenVINO 2022.3
## Download and Install OpenVINO Core Components
Create an Intel folder in the ```C:\Program Files (x86)\``` directory. Skip this step if the folder already exists.

You can also do this via command-lines. Open a new command prompt window as administrator by right-clicking Command Prompt from the Start menu and select Run as administrator, and then run the following command:
```
mkdir "C:\Program Files (x86)\Intel"
```
**Note:** ```C:\Program Files (x86)\Intel``` is the recommended folder. You may also use a different path if desired or if you don’t have administrator privileges on your computer.

Download the OpenVINO Runtime archive file for Windows to your local Downloads folder.

If you prefer using command-lines, run the following commands in the command prompt window you opened:
```
cd <user_home>/Downloads
curl -L https://storage.openvinotoolkit.org/repositories/openvino/packages/2022.3/windows/w_openvino_toolkit_windows_2022.3.0.9052.9752fafe8eb_x86_64.zip --output openvino_2022.3.0.zip
```
A ```.sha256``` file is provided together with the archive file to validate your download process. To do that, download the ```.sha256``` file from the same repository and run ```CertUtil -hashfile openvino_2022.3.0.zip SHA256```.

Use your favorite tool to extract the archive file, rename the extracted folder, and move it to the ```C:\Program Files (x86)\Intel``` directory.

To do this step using command-lines, run the following commands in the command prompt window you opened:
```
tar -xf openvino_2022.3.0.zip
ren w_openvino_toolkit_windows_2022.3.0.9052.9752fafe8eb_x86_64 openvino_2022.3.0
move openvino_2022.3.0 "C:\Program Files (x86)\Intel"
```
## Configure the Environment
You must update several environment variables before you can compile and run OpenVINO™ applications. Open the Command Prompt, and run the setupvars.bat batch file to temporarily set your environment variables. If your <INSTALL_DIR> is not C:\Program Files (x86)\Intel\openvino_2022.3.0, use the correct directory instead.
```
call "C:\Program Files (x86)\Intel\openvino_2022.3.0\setupvars.bat"
```
**Important:** The above command must be re-run every time a new Command Prompt window is opened.

**Note:**
If you see an error indicating Python is not installed, Python may not be added to the PATH environment variable (as described here). Check your system environment variables, and add Python if necessary.

The environment variables are set. Continue to the next section if you want to download any additional components.

## Download OpenVINO development tool
```
python -m pip install --upgrade pip
pip install openvino-dev[tensorflow2,onnx]
```

# Install Intel® RealSense™ SDK 2.0
Please see this [link](https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_windows.md) to install Intel® RealSense™ SDK 2.0 on Windows 10

#  Build and Install for ROS2_OpenVINO_Toolkit
## Install ROS2_OpenVINO_Toolkit packages
Run a Command Prompt by administrator.
```
md -p ~\catkin_ws\src
cd ~\catkin_ws\src
git clone https://github.com/intel/ros2_openvino_toolkit -b ros2
git clone https://github.com/intel/ros2_object_msgs
git clone https://github.com/IntelRealSense/realsense-ros.git -b ros2-development
git clone https://github.com/ros-perception/vision_opencv.git -b humble
```

## Install dependencies
* Install colcon-common-extensions
```
pip install -U colcon-common-extensions
```
* Install yaml-cpp
```
git clone https://github.com/jbeder/yaml-cpp
cd ~\yaml-cpp
md build &cd build
cmake ..
```
Open the ```YAML-CPP.sln``` on Visual Studio and click ```ALL_BUILD``` and ```Install``` to get ```yaml-cppd.lib```
You will need to append the folder such as ```C:\yaml_install\lib\cmake\yaml-cpp``` to your environment PATH.
Open the Cmakelist for ros2_openvino_tookit, update the yaml-cpp path:
```
find_package(yaml_cpp_vendor REQUIRED PATHS C:\\yaml_install\\lib\\cmake\\yaml-cpp)
```

* Install boost </br>

Downloaded ```boost_1_74_0.zip``` from the official ```boost.org``` website.</br>
Extracted to under the administrative C Drive folder ```Program Files```.</br>
Opened a Command Prompt.</br>
Navigated to the extracted folder for BOOST 1.74.0 and ran the following command:
```
cd C:\Program Files\boost\boost_1_74_0
bootstrap vc142
.\b2 address-model=64 architecture=x86
```
Append these folder to your PATH:
```
C:\Program Files\boost\boost_1_74_0\
C:\Program Files\boost\boost_1_74_0\libs\python
C:\Program Files\boost\boost_1_74_0\stage\lib
```

## Build package
Make sure your workspace under Visual Studio environment.
```
call "C:\Program Files\Microsoft Visual Studio\2019\Community\VC\Auxiliary\Build\vcvarsall.bat" x86_amd64
```
```
call "C:\dev\ros2_humble\ros2-windows\local_setup.bat"
call "C:\Program Files (x86)\Intel\openvino_2022.3.0\setupvars.bat"
cd ~\catkin_ws
colcon build --symlink-install
call ".\install\local_setup.bash"
```

## Running the Demo
* See all available models
```
omz_downloader --print_all
```

* Download the optimized Intermediate Representation (IR) of model (execute once), for example:
```
cd ~\catkin_ws\src\ros2_openvino_toolkit\data\model_list
omz_downloader --list download_model.lst -o "C:\Users\<USER_NAME>\Downloads\openvino_toolkit\models"
```

* If the model (tensorflow, caffe, MXNet, ONNX, Kaldi) need to be converted to intermediate representation (such as the model for object detection):
```
cd ~\catkin_ws\src\ros2_openvino_toolkit\data\model_list
omz_converter --list convert_model.lst -d "C:\Users\<USER_NAME>\Downloads\openvino_toolkit\models" -o "C:\Users\<USER_NAME>\Downloads\openvino_toolkit\models\convert"
```

* Copy label files (execute once)
**Note**:Need to make label_dirs if skip steps for set output_dirs above.
```
cp ~\catkin_ws\src\ros2_openvino_toolkit\data\labels\face_detection\face-detection-adas-0001.labels C:\Users\<USER_NAME>\Downloads\openvino_toolkit\models\intel\face-detection-adas-0001\FP32\
cp ~\catkin_ws\src\ros2_openvino_toolkit\data\labels\face_detection\face-detection-adas-0001.labels C:\Users\<USER_NAME>\Downloads\openvino_toolkit\models\intel\face-detection-adas-0001\FP16\
cp ~\catkin_ws\src\ros2_openvino_toolkit\data\labels\emotions-recognition\FP32\emotions-recognition-retail-0003.labels C:\Users\<USER_NAME>\Downloads\openvino_toolkit\models\intel\emotions-recognition-retail-0003\FP32\
cp ~\catkin_ws\src\ros2_openvino_toolkit\data\labels\object_segmentation\frozen_inference_graph.labels C:\Users\<USER_NAME>\Downloads\openvino_toolkit\models\intel\semantic-segmentation-adas-0001\FP32\
cp ~\catkin_ws\src\ros2_openvino_toolkit\data\labels\object_segmentation\frozen_inference_graph.labels C:\Users\<USER_NAME>\Downloads\openvino_toolkit\models\intel\semantic-segmentation-adas-0001\FP16\
cp ~\catkin_ws\src\ros2_openvino_toolkit\data\labels\object_detection\vehicle-license-plate-detection-barrier-0106.labels C:\Users\<USER_NAME>\Downloads\openvino_toolkit\models\intel\vehicle-license-plate-detection-barrier-0106\FP32
```

* Check the parameter configuration in ros2_openvino_toolkit/sample/param/xxxx.yaml before launching, make sure parameters such as model_path, label_path and input_path are set correctly. </br>
Please refer to the quick start document for [yaml configuration guidance](./yaml_configuration_guide.md) for detailed configuration guidance.
  * run face detection sample code input from StandardCamera.
  ```
  ros2 launch openvino_node pipeline_people.launch.py
  ```
  * run person reidentification sample code input from StandardCamera.
  ```
  ros2 launch openvino_node pipeline_reidentification.launch.py
  ```
  * run face detection sample code input from Image.
  ```
  ros2 launch openvino_node pipeline_image.launch.py
  ```
  * run object segmentation sample code input from RealSenseCameraTopic.
  ```
  ros2 launch openvino_node pipeline_segmentation.launch.py
  ```
  * run vehicle detection sample code input from StandardCamera.
  ```
  ros2 launch openvino_node pipeline_vehicle_detection.launch.py
  ```
  * run person attributes sample code input from StandardCamera.
  ```
  ros2 launch openvino_node pipeline_person_attributes.launch.py
  ```

# WIP
The deployment is still WIP due to the dependency problem of ros2 packages on Windows has not been resolved, such as cv_bridge.
Here are related links for reference:
* [Failed to Build boost for Windows 10](https://github.com/ros-perception/vision_opencv/issues/349)
* [Failed to Build vision_opencv for Windows 10](https://github.com/ms-iot/ROSOnWindows/issues/371)
* [ros2 packages for humble](https://github.com/ros2/ros2/blob/humble/ros2.repos)
* [Availability of ros2 humble packages on different OS](https://robostack.github.io/humble.html)
* [Chocolatey package source lists](https://community.chocolatey.org/packages)

# More Information
* ROS2 OpenVINO description written in Chinese: https://mp.weixin.qq.com/s/BgG3RGauv5pmHzV_hkVAdw

###### *Any security issue should be reported using process at https://01.org/security*
