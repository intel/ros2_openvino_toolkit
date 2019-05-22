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
 * @brief a header file with declaration of RealSenseCamera class
 * @file realsense_camera.cpp
 */
#include "dynamic_vino_lib/inputs/realsense_camera.hpp"

#include "dynamic_vino_lib/slog.hpp"

// RealSenseCamera
bool Input::RealSenseCamera::initialize()
{
  return initialize(640,480);
}
bool Input::RealSenseCamera::initialize(size_t width, size_t height)
{
  static int rscamera_count = 0;
  // Get all devices connected
  rs2::context cxt;
  auto device = cxt.query_devices();
  size_t device_count = device.size();
  slog::info << "Find RealSense num:"<< device_count << slog::endl;
  auto hardware = device[rscamera_count];
  auto devSerialNumber = hardware.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
	//std::cout << "Camera " << rscamera_count << ": " << hardware.get_info(RS2_CAMERA_INFO_NAME) << std::endl;
	slog::info << "RealSense Serial number : " << devSerialNumber << slog::endl;
  cfg_.enable_device(devSerialNumber);
  
  if (3 * width != 4 * height) {
    slog::err << "The aspect ratio must be 4:3 when using RealSense camera" << slog::endl;
    return false;
  }
  cfg_.enable_stream(RS2_STREAM_COLOR, static_cast<int>(width), static_cast<int>(height),
    RS2_FORMAT_BGR8, 30);
  setInitStatus(pipe_.start(cfg_));
  setWidth(width);
  setHeight(height);
  if (!isInit()) {
    return false;
  }
  if (first_read_) {
    rs2::frameset frames;
    for (int i = 0; i < 30; i++) {
      // Wait for all configured streams to produce a frame
      try {
        frames = pipe_.wait_for_frames();
      } catch (...) {
        return false;
      }
    }
    first_read_ = false;
  }
  rscamera_count++;
  return true;
}
bool Input::RealSenseCamera::read(cv::Mat * frame)
{
  if (!isInit()) {
    return false;
  }
  rs2::frameset data = pipe_.wait_for_frames();  // Wait for next set of frames from the camera
  rs2::frame color_frame;
  try {
    color_frame = data.get_color_frame();
  } catch (...) {
    return false;
  }
  cv::Mat(cv::Size(static_cast<int>(getWidth()), static_cast<int>(getHeight())), CV_8UC3,
    const_cast<void *>(color_frame.get_data()), cv::Mat::AUTO_STEP)
  .copyTo(*frame);
  setHeader("realsense_camera_frame");
  return true;
}
