// Copyright (c) 2018-2022 Intel Corporation
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
#include "openvino_wrapper_lib/inputs/standard_camera.hpp"

Input::StandardCamera::StandardCamera(const std::string & camera)
: device_path_(camera)
{
}

bool Input::StandardCamera::initialize()
{
  return initialize(640, 480);
}

bool Input::StandardCamera::initialize(size_t width, size_t height)
{
  bool init = false;
  if(!device_path_.empty()){
    init = cap.open(device_path_);
  }
  if (init == false){
    auto id = getCameraId();
    init = cap.open(id);
  }

  if(init){
    cap.set(cv::CAP_PROP_FRAME_WIDTH, width);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, height);
    setWidth(width);
    setHeight(height);
    setInitStatus(true);
  }
  return init;
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

int Input::StandardCamera::getCameraId()
{
  // In case this function is invoked more than once.
  if (camera_id_ >= 0){
    return camera_id_;
  }

  static int STANDARD_CAMERA_COUNT = -1;
  int fd; // A file descriptor to the video device
  struct v4l2_capability cap;
  char file[32];
  //if it is a realsense camera then skip it until we meet a standard camera
  do
  {
    STANDARD_CAMERA_COUNT ++;
    sprintf(file,"/dev/video%d",STANDARD_CAMERA_COUNT);//format filename
    fd = open(file,O_RDWR);
    ioctl(fd, VIDIOC_QUERYCAP, &cap);
    close(fd);
    std::cout << "!!camera: "<< cap.card << std::endl;
  }while(!strcmp((char*)cap.card,"Intel(R) RealSense(TM) Depth Ca"));

  camera_id_ = STANDARD_CAMERA_COUNT;
  return STANDARD_CAMERA_COUNT;
}
