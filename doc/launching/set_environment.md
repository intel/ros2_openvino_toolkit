# Set Environment
## OpenSource Version
* Set ENV LD_LIBRARY_PATH and openvino_version
  ```bash
  export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/opt/openvino_toolkit/dldt/inference-engine/bin/intel64/Release/lib
  export openvino_version=opensource
  ```
* Install prerequisites
  ```bash
  cd /opt/openvino_toolkit/dldt/model-optimizer/install_prerequisites
  sudo ./install_prerequisites.sh
  ```
* Set model tool variable
  ```bash
  source /opt/openvino_toolkit/ros2_openvino_toolkit/script/set_variable.sh
  ```
## Binary Version
* Set ENV LD_LIBRARY_PATH and openvino_version
  ```bash
  source /opt/intel/openvino/bin/setupvars.sh
  export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/opt/intel/openvino/deployment_tools/inference_engine/samples/build/intel64/Release/lib
  export openvino_version=binary
  ```
* Install prerequisites
  ```bash
  cd /opt/intel/openvino/deployment_tools/model_optimizer/install_prerequisites
  sudo ./install_prerequisites.sh
  ```
* Set model tool variable
  ```bash
  source /opt/openvino_toolkit/ros2_openvino_toolkit/script/set_variable.sh
  ```
