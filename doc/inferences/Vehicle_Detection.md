# Vehicle Detection
## Download Models
### OpenSource Version
* download the optimized Intermediate Representation (IR) of model (excute _once_)<br>
  ```bash
  cd $model_downloader
  sudo python3 downloader.py --name vehicle-license-plate-detection-barrier-0106 --output_dir /opt/openvino_toolkit/models/vehicle-license-plate-detection/output
  sudo python3 downloader.py --name vehicle-attributes-recognition-barrier-0039 --output_dir /opt/openvino_toolkit/models/vehicle-attributes-recongnition/output
  sudo python3 downloader.py --name license-plate-recognition-barrier-0001 --output_dir /opt/openvino_toolkit/models/license-plate-recognition/output
  ```
* copy label files (excute _once_)<br>
  ```bash
  sudo cp $openvino_labels/object_detection/vehicle-license-plate-detection-barrier-0106.labels /opt/openvino_toolkit/models/vehicle-license-plate-detection/output
  ```
