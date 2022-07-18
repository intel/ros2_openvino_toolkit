#!/usr/bin/python
# -*- coding: UTF-8 -*-

"""
Author: jiafengx.huang@intel.com
Datetime: 2022/6/24
"""

import sys
import re
import smtplib
import data_collect
from Frame.host_config import email_config
from email.mime.text import MIMEText
from email.header import Header
from email.mime.application import MIMEApplication
from email.mime.multipart import MIMEMultipart

projectdic = {'ros2_openvino': "ros2_openvino Test", 'All_Test': "All Test"}

class Email_Info:
    message = MIMEMultipart()
    link_path = "https://github.com/huangjiafengx/ros2_openvino_toolkit/actions"
    def __init__(self):
        self.sender = email_config['sender']
        #self.receivers = email_config['receiver']
        self.receivers = email_config['owner']

    def add_attachment(self):
        # att_list = ['ros2_openvino_all.log', 'ros2_openvino_error.log', 'ros2_openvino_warning.log']
        for att, fil in data_collect.test_case_log.items():
            file_path = "/home/intel/ros2_openvino_toolkit/log/" + fil
            att_application = MIMEApplication(open(file_path,'rb').read())
            att_application["Content-Type"] = 'application/octet-stream'
            att_application.add_header('Content-Disposition', 'attachment', filename=(fil))
            self.message.attach(att_application)

    def run(self, build_type, project_name):

        ros2_openvino_obj = data_collect.Ros2_Openvino(build_type, project_name)
        ros2_openvino_obj.get_the_test_result()

        msg = """\
        <html xmlns="http://www.w3.org/1999/xhtml">
        <head>
        <body>
        <div id="container">
        <p><strong>""" +projectdic['All_Test'] +" Projects Include:" +"""</strong></p>
          <div id="content">
           <table width="600" border="2" bordercolor="green" cellspacing="2">
          <tr>
            <td><strong>Test Cases:</strong></td>
            <td><strong>Test Result:</strong></td>
          </tr>
          <tr>
            <td>""" + 'pipeline_people'  + """</td>
            <td>""" + ros2_openvino_obj.cases_result['pipeline_people'] + """</td>
          </tr>
          <tr>
            <td>""" + 'pipeline_reidentification'  + """</td>
            <td>""" + ros2_openvino_obj.cases_result['pipeline_reidentification'] + """</td>
          </tr>
          <tr>
            <td>""" + 'pipeline_image'  + """</td>
            <td>""" + ros2_openvino_obj.cases_result['pipeline_image'] + """</td>
          </tr>
          <tr>
            <td>""" + 'pipeline_vehicle_detection'  + """</td>
            <td>""" + ros2_openvino_obj.cases_result['pipeline_vehicle_detection'] + """</td>
          </tr>
          <tr>
            <td>""" + 'pipeline_person_attributes'  + """</td>
            <td>""" + ros2_openvino_obj.cases_result['pipeline_person_attributes'] + """</td>
          </tr>
        </table>
          </div>
        </div>
        <p><a href="%s">The detail information clicking the link:</a></p>
        </div>
        </body>
        </html>
        """ % (self.link_path)

        self.message.attach(MIMEText(msg,_subtype='html', _charset='utf-8'))
        self.message['From'] = Header(self.sender, 'utf-8')
        self.message['To'] = ','.join(self.receivers)
        subject = project_name
        self.message['Subject'] = Header(subject, 'utf-8')
        self.add_attachment()
        try:
            smtpObj = smtplib.SMTP('smtp.intel.com')
            smtpObj.sendmail(self.sender, self.receivers, self.message.as_string())
            print ("send success")
        except smtplib.SMTPException:
            print ("Error: send fail")
        #ros2_openvino_obj.clear_files()


if __name__ == '__main__':
    # project name is pcl flann or opd.
    project_name = sys.argv[1]
    build_type = sys.argv[2]
    
    project_email = Email_Info()
    # Send the Email
    project_email.run(build_type, project_name)


