pipeline {
  agent any
  stages {
    stage('pull code') {
      environment {
        Test_Server = '10.239.89.24'
        ROS2_OPENVINO_URL = 'https://github.com/intel/ros2_openvino_toolkit'
      }
      steps {
        sh '''pipeline {
    agent any
    environment {
        Test_Server = "10.239.89.24"
        ROS2_OPENVINO_URL = "https://github.com/intel/ros2_openvino_toolkit"
        ROS2_OBJECT_MSGS_URL = "https://github.com/intel/ros2_object_msgs"
        REALSENSE_ROS_URL = "https://github.com/IntelRealSense/realsense-ros.git -b ros2"
        VISION_OPENCV_URL = "https://github.com/ros-perception/vision_opencv.git -b ros2"
        Credentials_ID = "f6d78d9a-f737-4019-b64e-7bcb3a3313d0"
        WORKSPACE_PATH = "/home/intel/ros2_openvino_toolkit"
    }
    stages {
        stage(\'Pull Code\') {
            steps {
                dir ("ros2_openvino_dir") {
                    git  credentialsId: env.Credentials_ID, url: env.ROS2_OPENVINO_URL
                }
            }
        }
        stage(\'Test On Ros2 Galatic\') {
            steps {
                script {
                    def flag = sh script: "ssh intel@$Test_Server \'cd $WORKSPACE_PATH && docker images | grep ros2_openvino_test\'", returnStatus: true
                    if (flag == 0) {
                        docker rmi -f ros2_openvino_test
                    }
                    // sh "ssh intel@$Test_Server \'cd $WORKSPACE_PATH && docker rmi -f ros2_openvino_test\'"
                    // def test_result = sh script: "ssh intel@$Test_Server \'cd $WORKSPACE_PATH && docker build -t ros2_openvino_test .\'", returnStatus: true
                    def test_result = sh script: "ssh intel@$Test_Server \'cd $WORKSPACE_PATH && ./self_host_test_ros2_openvino.sh \'", returnStatus: true
                    if (test_result == 0) {
                        echo "test pass"
                    } else {
                        echo "test fail"
                    }
                
                }
            }
        }

    }

}'''
        }
      }

    }
  }