# People Reidentification
## Demo Result Snapshots
See below pictures for the demo result snapshots.
* Person Reidentification input from standard camera
![person_reidentification_demo_video](https://github.com/intel/ros2_openvino_toolkit/blob/devel/data/images/person-reidentification.gif "person reidentification demo video")
## Download Models
* download the optimized Intermediate Representation (IR) of model (excute _once_)<br>
  ```bash
  cd $model_downloader
  sudo python3 downloader.py --name person-detection-retail-0013 --output_dir /opt/openvino_toolkit/models/person-detection/output
  sudo python3 downloader.py --name person-reidentification-retail-0076 --output_dir /opt/openvino_toolkit/models/person-reidentification/output
  ```

