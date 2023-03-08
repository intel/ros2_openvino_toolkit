#!/usr/opt/python3
import unittest
from test_cases import Test_Cases
from XTestRunner import HTMLTestRunner

def main():

    suite = unittest.TestSuite()
    suite.addTest(Test_Cases('test_1_pipeline_people_ci'))
    suite.addTest(Test_Cases('test_2_pipeline_reidentification_ci'))
    suite.addTest(Test_Cases('test_3_pipeline_image_ci'))
    suite.addTest(Test_Cases('test_6_pipeline_vehicle_detection_ci'))
    suite.addTest(Test_Cases('test_7_pipeline_person_attributes_ci'))

    with (open('./result.html', 'wb')) as fp:
        runner = HTMLTestRunner(
            stream=fp,
            title='ROS2 Openvino Test Report',
            description='Test ROS2-galactic openvino all cases',
            language='en',
        )
        result = runner.run(
            testlist=suite,
            rerun=0,
            save_last_run=False
        )

    print(f"success count: {result.success_count}")
    print(f"failure count: {result.failure_count}")
    print(f"runtime:", result.case_end_time - result.case_start_time)
    if result.failure_count == 0 and result.error_count == 0:
        print(f"Test ALL PASS")
    else:
        print(f"Test FAIL")
        exit(-1)

if __name__=="__main__":
    main()

