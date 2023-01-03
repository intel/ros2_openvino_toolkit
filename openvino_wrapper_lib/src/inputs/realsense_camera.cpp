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
 * @brief a header file with declaration of RealSenseCamera class
 * @file realsense_camera.cpp
 */
#include "openvino_wrapper_lib/inputs/realsense_camera.hpp"
#include "openvino_wrapper_lib/slog.hpp"

// RealSenseCamera
bool Input::RealSenseCamera::initialize()
{
  return initialize(640, 480);
}

bool Input::RealSenseCamera::initialize(size_t width, size_t height)
{
  if (3 * width != 4 * height) {
    slog::err << "The aspect ratio must be 4:3 when using RealSense camera" << slog::endl;
    throw std::runtime_error("The aspect ratio must be 4:3 when using RealSense camera!");
    return false;
  }

  auto devSerialNumber = getCameraSN();
  slog::info << "RealSense Serial number : " << devSerialNumber << slog::endl;

  cfg_.enable_device(devSerialNumber);
  cfg_.enable_stream(RS2_STREAM_COLOR, static_cast<int>(width), static_cast<int>(height),
    RS2_FORMAT_BGR8, 30);

  setInitStatus(pipe_.start(cfg_));
  setWidth(width);
  setHeight(height);

  //bypass RealSense's bug: several captured frames after HW is inited are with wrong data.
  bypassFewFramesOnceInited();

  return isInit();
}

bool Input::RealSenseCamera::read(cv::Mat * frame)
{
  if (!isInit()) {
    return false;
  }

 try {
    rs2::frameset data = pipe_.wait_for_frames();  // Wait for next set of frames from the camera
    rs2::frame color_frame;
    color_frame = data.get_color_frame();

    cv::Mat(cv::Size(static_cast<int>(getWidth()), static_cast<int>(getHeight())), CV_8UC3,
      const_cast<void *>(color_frame.get_data()), cv::Mat::AUTO_STEP)
      .copyTo(*frame);
  } catch (...) {
    return false;
  }

  setHeader("realsense_camera_frame");
  return true;
}

std::string Input::RealSenseCamera::getCameraSN()
{
  static int rscamera_count = 0;
  // Get all devices connected
  rs2::context cxt;
  auto device = cxt.query_devices();
  size_t device_count = device.size();
  slog::info << "Find RealSense num:" << device_count << slog::endl;
  auto hardware = device[rscamera_count];
  auto devSerialNumber = hardware.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
  rscamera_count++;
  return devSerialNumber;
}

void Input::RealSenseCamera::bypassFewFramesOnceInited()
{
  if(!isInit() || !first_read_){
    return;
  }

  rs2::frameset frames;
  for (int i = 0; i < 30; i++) {
    frames = pipe_.wait_for_frames();
  }
  first_read_ = false;
}
