# Face Reidentification
## Download Models
* download the optimized Intermediate Representation (IR) of model (excute _once_)<br>
  ```bash
  cd $model_downloader
  sudo python3 downloader.py --name landmarks-regression-retail-0009 --output_dir /opt/openvino_toolkit/models/landmarks-regression/output
  sudo python3 downloader.py --name face-reidentification-retail-0095 --output_dir /opt/openvino_toolkit/models/face-reidentification/output
  ```


