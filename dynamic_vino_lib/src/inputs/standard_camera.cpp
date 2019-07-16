// Copyright (c) 2018 Intel Corporation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/**
 * @brief a header file with declaration of StandardCamera class
 * @file standard_camera.cpp
 */
#include "dynamic_vino_lib/inputs/standard_camera.hpp"

bool Input::StandardCamera::initialize()
{
  static int camera_count_ = -1;
  int fd; // A file descriptor to the video device
  struct v4l2_capability cap;
  char file[20];
  //if it is a realsense camera then skip it until we meet a standard camera
  do
  {
    camera_count_ ++;
    sprintf(file,"/dev/video%d",camera_count_);//format filename
    fd = open(file,O_RDWR);
    ioctl(fd, VIDIOC_QUERYCAP, &cap);
    close(fd);
    
    std::cout << "!!camera: "<< cap.card << std::endl;
  }
  while(!strcmp((char*)cap.card,"Intel(R) RealSense(TM) Depth Ca"));
  
  return initialize(camera_count_);
}

bool Input::StandardCamera::initialize(int camera_num)
{
  setInitStatus(cap.open(camera_num));
  setWidth((size_t)cap.get(cv::CAP_PROP_FRAME_WIDTH));
  setHeight((size_t)cap.get(cv::CAP_PROP_FRAME_HEIGHT));
  return isInit();
}

bool Input::StandardCamera::initialize(size_t width, size_t height)
{
  static int camera_count_ = 0;
  setWidth(width);
  setHeight(height);
  setInitStatus(cap.open(camera_count_));
  if (isInit()) {
    cap.set(cv::CAP_PROP_FRAME_WIDTH, width);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, height);
  }
  camera_count_++;
  return isInit();
}

bool Input::StandardCamera::read(cv::Mat * frame)
{
  if (!isInit()) {
    return false;
  }
  cap.grab();
  setHeader("standard_camera_frame");
  return cap.retrieve(*frame);
}
