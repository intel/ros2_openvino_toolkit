#!/usr/bin/python
#_*_coding:utf-8_*_

"""
Author: jiafengx.huang@intel.com
Datetime: 2022/4/13
"""
import subprocess
import pdb
import sys
import re
import os

# ignore the case 'pipeline_segmentation': "pipeline_segmentation_test.log",
#ingnore the case 'pipeline_face_reidentification': "pipeline_face_reidentification_test.log",
test_case_log = {'pipeline_people': "pipeline_people_test.log",
                 'pipeline_reidentification': "pipeline_reidentification_test.log",
                 'pipeline_image': "pipeline_image_test.log",
                 'pipeline_vehicle_detection': "pipeline_vehicle_detection_test.log",
                 'pipeline_person_attributes': "pipeline_person_attributes_test.log"}

class Ros2_Openvino:
    def __init__(self, build_type, project_name, container_test = False):
        self.log_path = '/home/intel/ros2_openvino_toolkit/log/'
        self.build_type = build_type
        self.project_name = project_name
        self.container_test = container_test
        self.cases_result = {'pipeline_people': 'PASS', 'pipeline_reidentification': 'PASS', \
                        'pipeline_image': 'PASS', 'pipeline_vehicle_detection': 'PASS', 'pipeline_person_attributes': 'PASS'}
    def get_data(self, case_file):
        log_file = self.log_path + case_file
        # print("log_file", log_file)
        with open(log_file, 'r') as handle:
            content = handle.read()
        return content


    def get_the_test_result(self):
        for case, result in self.cases_result.items():
            try:
                log = self.get_data(test_case_log[case])
                log = log.replace("[ERROR] [rviz2-2]","There rviz2-2 is quit")
            except IndexError:
                print("ros2_openvino: get_the_test_result fail")
                log = '....'
            if "ERROR" in log or "Analyzing Detection results" not in log:
                print ("%s Fail", case)
                self.cases_result[case] = "Fail"
            else:
                self.cases_result[case] = "Pass"

    def clear_files(self):
        for name, fil in test_case_log.items():
            file_path = "/home/intel/ros2_openvino_toolkit/log/" + fil
            os.remove(file_path)

if __name__ == '__main__':

    # project name is pcl flann or opd.
    project_name = sys.argv[1]

    # Get the build type frameworks.industrial.robotics.vision.projects or others.
    build_type = sys.argv[2]

    ros2_openvino_obj = Ros2_Openvino(build_type, project_name)

    #Making the log files.
    ros2_openvino_obj.write_data_to_file()

    ros2_openvino_obj.get_the_branch()

    ros2_openvino_obj.get_the_test_result()

