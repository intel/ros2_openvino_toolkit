#!/usr/opt/python3
import unittest
from test_cases import Test_Cases
from XTestRunner import HTMLTestRunner

def main():

    suite = unittest.TestSuite()

    all_cases = [Test_Cases('test_1_pipeline_people_ci'),
                 Test_Cases('test_2_pipeline_reidentification_ci'),
                 Test_Cases('test_3_pipeline_image_ci'),
                 Test_Cases('test_4_pipeline_segmentation_ci'),
                 Test_Cases('test_5_pipeline_vehicle_detection_ci'),
                 Test_Cases('test_6_pipeline_person_attributes_ci'),
                 Test_Cases('test_7_pipeline_segmentation_image_ci'),
                 Test_Cases('test_8_pipeline_object_yolov5_ci'),
                 Test_Cases('test_9_pipeline_object_yolov8_ci')]
    suite.addTests(all_cases)

    with (open('./result.html', 'wb')) as fp:
        runner = HTMLTestRunner(
            stream=fp,
            title='ROS2 Openvino Test Report',
            description='Test ROS2-galactic openvino all cases',
            language='en',
        )
        result = runner.run(
            testlist=suite,
            rerun=1,
            save_last_run=False
        )

    failure_count = len(all_cases) - result.success_count
    print(f"all count: {len(all_cases)}")
    print(f"success count: {result.success_count}")
    print(f"failure count: {failure_count}")
    if result.success_count == len(all_cases) and failure_count == 0:
        print(f"Test ALL PASS")
    else:
        print(f"Test FAIL")
        exit(-1)

if __name__=="__main__":
    main()

