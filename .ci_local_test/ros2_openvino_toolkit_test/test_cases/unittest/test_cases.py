#from asyncio import sleep
from time import sleep
import unittest
import subprocess
import pdb
import os

class Test_Cases(unittest.TestCase):
    def test_pipeline(self, launch_file, log_file, topic_list=['/rosout']):
        print(f"{log_file} topic_list", topic_list)
        log_file = f"/root/test_cases/log/" + log_file
        kill_ros2_process()
        subprocess.Popen([f"ros2 launch openvino_node {launch_file} > {log_file} &"],stdout=subprocess.PIPE,shell=True).communicate()
        for topic in topic_list:
            name=topic.split('/', -1)[-1]
            sleep(2)
            subprocess.Popen([f"ros2 topic echo {topic} > {name}.log &"], stdout=subprocess.PIPE,shell=True).communicate()
        process_result = subprocess.Popen(["ps -ef | grep ros2 | grep -v 'grep' | awk '{print $2}'"],stdout=subprocess.PIPE,shell=True).communicate()
        kill_ros2_process()
        print(f"kill the test process done")
        with open(f"{log_file}") as handle:
            log = handle.read()
            check_log = log.split("user interrupted with ctrl-c (SIGINT)")[0]
            if f"pipeline_object_yolo" not in log_file:
                self.assertIn('Analyzing Detection results', check_log)
                self.assertNotIn('ERROR', check_log)
        for topic in topic_list:
            name = topic.split('/', -1)[-1]
            #os.system(f"cat /root/test_cases/unittest/{name}.log | head -n 5")
            with open(f"{name}.log") as topic_handle:
                topic_info = topic_handle.read()
                print("assertIn", self.assertIn("header", topic_info))
        print(f"check all done")


    def test_1_pipeline_people(self):
        topic_ls = ["/ros2_openvino_toolkit/age_genders_Recognition", \
                    "/ros2_openvino_toolkit/headposes_estimation", \
                    "/ros2_openvino_toolkit/face_detection", \
                    "/ros2_openvino_toolkit/emotions_recognition"]
        launch_file = f"pipeline_people.launch.py"
        log_file = f"pipeline_people_test.log"
        self.test_pipeline(launch_file, log_file, topic_list=topic_ls)

    def test_2_pipeline_reidentification(self):
        topic_ls = ["/ros2_openvino_toolkit/reidentified_persons",]
        launch_file = f"pipeline_reidentification.launch.py"
        log_file = f"pipeline_reidentification_test.log"
        self.test_pipeline(launch_file, log_file, topic_list=topic_ls)

    def test_3_pipeline_image(self):
        topic_ls = ["/ros2_openvino_toolkit/emotions_recognition", \
                    "/ros2_openvino_toolkit/headposes_estimation", \
                    "/ros2_openvino_toolkit/people/age_genders_Recognition"]
        launch_file = f"pipeline_image.launch.py"
        log_file = f"pipeline_image_test.log"
        self.test_pipeline(launch_file, log_file, topic_list=topic_ls)

    def test_4_pipeline_segmentation(self):
        topic_ls = ["/ros2_openvino_toolkit/segmented_obejcts"]
        launch_file = f"pipeline_segmentation.launch.py"
        log_file = f"pipeline_segmentation_test.log"
        self.test_pipeline(launch_file, log_file, topic_list=topic_ls)

    def test_5_pipeline_segmentation_image(self):
        topic_ls = ["/ros2_openvino_toolkit/segmented_obejcts"]
        launch_file = f"pipeline_segmentation_image.launch.py"
        log_file = f"pipeline_segmentation_image_test.log"
        self.test_pipeline(launch_file, log_file, topic_list=topic_ls)

    def test_6_pipeline_vehicle_detection(self):
        topic_ls = ["/ros2_openvino_toolkit/detected_license_plates",
                    "/ros2_openvino_toolkit/detected_vehicles_attribs"]
        launch_file = f"pipeline_vehicle_detection.launch.py"
        log_file = f"pipeline_vehicle_detection_test.log"
        self.test_pipeline(launch_file, log_file, topic_list=topic_ls)

    def test_7_pipeline_person_attributes(self):
        topic_ls = ["/ros2_openvino_toolkit/detected_objects", \
                   "/ros2_openvino_toolkit/person_attributes"]
        launch_file = f"pipeline_person_attributes.launch.py"
        log_file = f"pipeline_person_attributes_test.log"
        self.test_pipeline(launch_file, log_file, topic_list=topic_ls)

    def test_8_pipeline_object_yolo(self):
        topic_ls = ["/ros2_openvino_toolkit/detected_objects"]
        launch_file = f"pipeline_object_yolo.launch.py"
        log_file = f"pipeline_object_yolo_test.log"
        self.test_pipeline(launch_file, log_file, topic_list=topic_ls)


    def test_1_pipeline_people_ci(self):
        topic_ls = ["/ros2_openvino_toolkit/age_genders_Recognition", \
                    "/ros2_openvino_toolkit/headposes_estimation", \
                    "/ros2_openvino_toolkit/face_detection", \
                    "/ros2_openvino_toolkit/emotions_recognition"]
        launch_file = f"pipeline_people_ci_test.py"
        log_file = f"pipeline_people_test_ci.log"
        self.test_pipeline(launch_file, log_file, topic_list=topic_ls)

    def test_2_pipeline_reidentification_ci(self):
        topic_ls = ["/ros2_openvino_toolkit/reidentified_persons",]
        launch_file = f"pipeline_reidentification_ci_test.py"
        log_file = f"pipeline_reidentification_test_ci.log"
        self.test_pipeline(launch_file, log_file, topic_list=topic_ls)

    def test_3_pipeline_image_ci(self):
        topic_ls = ["/ros2_openvino_toolkit/emotions_recognition", \
                    "/ros2_openvino_toolkit/headposes_estimation", \
                    "/ros2_openvino_toolkit/people/age_genders_Recognition"]
        launch_file = f"pipeline_image_ci_test.py"
        log_file = f"pipeline_image_test_ci.log"
        self.test_pipeline(launch_file, log_file, topic_list=topic_ls)

    def test_4_pipeline_segmentation_ci(self):
        topic_ls = ["/ros2_openvino_toolkit/segmented_obejcts"]
        launch_file = f"pipeline_segmentation_ci_test.py"
        log_file = f"pipeline_segmentation_test_ci.log"
        self.test_pipeline(launch_file, log_file, topic_list=topic_ls)

    def test_5_pipeline_segmentation_image_ci(self):
        topic_ls = ["/ros2_openvino_toolkit/segmented_obejcts"]
        launch_file = f"pipeline_segmentation_image_ci_test.py"
        log_file = f"pipeline_segmentation_image_test_ci.log"
        self.test_pipeline(launch_file, log_file, topic_list=topic_ls)

    def test_6_pipeline_vehicle_detection_ci(self):
        topic_ls = ["/ros2_openvino_toolkit/detected_license_plates",
                    "/ros2_openvino_toolkit/detected_vehicles_attribs"]
        launch_file = f"pipeline_vehicle_detection_ci_test.py"
        log_file = f"pipeline_vehicle_detection_test_ci.log"
        self.test_pipeline(launch_file, log_file, topic_list=topic_ls)

    def test_7_pipeline_person_attributes_ci(self):
        topic_ls = ["/ros2_openvino_toolkit/detected_objects", \
                   "/ros2_openvino_toolkit/person_attributes"]
        launch_file = f"pipeline_person_attributes_ci_test.py"
        log_file = f"pipeline_person_attributes_test_ci.log"
        self.test_pipeline(launch_file, log_file, topic_list=topic_ls)

    def test_8_pipeline_object_yolo_ci(self):
        topic_ls = ["/ros2_openvino_toolkit/detected_objects"]
        launch_file = f"pipeline_object_yolo_ci_test.py"
        log_file = f"pipeline_object_yolo_test_ci.log"
        self.test_pipeline(launch_file, log_file, topic_list=topic_ls)





def kill_ros2_process():
    sleep(30)
    process_result = subprocess.Popen(["ps -ef | grep ros2 | grep -v 'grep' | awk '{print $2}'"],stdout=subprocess.PIPE,shell=True).communicate()
    print(process_result[0].decode('utf-8').replace('\n', ' '))
    kill_process = 'kill -9 ' + process_result[0].decode('utf-8').replace('\n', ' ')
    subprocess.Popen([kill_process], stdout=subprocess.PIPE ,shell=True).communicate()



