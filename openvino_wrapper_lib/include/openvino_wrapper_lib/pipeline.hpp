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
 * @brief a header file with declaration of Pipeline class
 * @file pipeline.h
 */
#ifndef OPENVINO_WRAPPER_LIB__PIPELINE_HPP_
#define OPENVINO_WRAPPER_LIB__PIPELINE_HPP_

#include <atomic>
#include <future>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <string>

#include "openvino_wrapper_lib/inferences/base_inference.hpp"
#include "openvino_wrapper_lib/inputs/standard_camera.hpp"
#include "openvino_wrapper_lib/outputs/base_output.hpp"
#include "openvino_wrapper_lib/pipeline_params.hpp"
#include "opencv2/opencv.hpp"

/**
 * @class Pipeline
 * @brief This class is a pipeline class that stores the topology of
 * the input device, output device and networks and make inference. One pipeline
 * should have only one input device.
 */
class Pipeline
{
public:
  explicit Pipeline(const std::string & name = "pipeline");
  /**
   * @brief Add input device to the pipeline.
   * @param[in] name name of the current input device.
   * @param[in] input_device the input device instance to be added.
   * @return whether the add operation is successful
   */
  bool add(const std::string & name, std::shared_ptr<Input::BaseInputDevice> input_device);
  /**
   * @brief Add inference network to the pipeline.
   * @param[in] parent name of the parent device or inference.
   * @param[in] name name of the current inference network.
   * @param[in] inference the inference instance to be added.
   * @return whether the add operation is successful
   */
  bool add(
    const std::string & parent, const std::string & name,
    std::shared_ptr<openvino_wrapper_lib::BaseInference> inference);
  /**
   * @brief Add output device to the pipeline.
   * @param[in] parent name of the parent inference.
   * @param[in] name name of the current output device.
   * @param[in] output the output instance to be added.
   * @return whether the add operation is successful
   */
  bool add(
    const std::string & parent, const std::string & name,
    std::shared_ptr<Outputs::BaseOutput> output);

  bool add(const std::string & name, std::shared_ptr<Outputs::BaseOutput> output);
  void addConnect(const std::string & parent, const std::string & name);
  // inline void addFilters(const std::vector<Params::ParamManager::FilterRawData>& filters)
  // {
  //   filters_.add(filters);
  // }
  bool add(const std::string & name, std::shared_ptr<openvino_wrapper_lib::BaseInference> inference);
  /**
   * @brief Add inference network-output device edge to the pipeline.
   * @param[in] parent name of the parent inference.
   * @param[in] name name of the current output device.
   * @return whether the add operation is successful
   */
  bool add(const std::string & parent, const std::string & name);
  /**
   * @brief Do the inference once.
   * Data flow from input device to inference network, then to output device.
   */
  void runOnce();

  void callback(const std::string & detection_name);
  /**
   * @brief Set the inference network to call the callback function as soon as
   * each inference is
   * finished.
   */
  void setCallback();
  void printPipeline();
  std::map<std::string, std::shared_ptr<Outputs::BaseOutput>> getOutputHandle()
  {
    return name_to_output_map_;
  }
  void setParams(PipelineParams pipeline_params)
  {
    params_ = std::make_shared<PipelineParams>(pipeline_params);
  }
  const std::shared_ptr<PipelineParams> getParameters()
  {
    return params_;
  }
  std::shared_ptr<Input::BaseInputDevice> getInputDevice()
  {
    return input_device_;
  }
  const std::multimap<std::string, std::string> getPipelineDetail()
  {
    return next_;
  }
  /**
  * @brief Get real time FPS (frames per second).
  */
  int getFPS() const
  {
    return fps_;
  }

  std::string findFilterConditions(const std::string & input, const std::string & output)
  {
    return params_->findFilterConditions(input, output);
  }

private:
  void initInferenceCounter();
  void increaseInferenceCounter();
  void decreaseInferenceCounter();
  bool isLegalConnect(const std::string parent, const std::string child);
  int getCatagoryOrder(const std::string name);
  void countFPS();
  void setFPS(int fps)
  {
    fps_ = fps;
  }

  const int kCatagoryOrder_Unknown = -1;
  const int kCatagoryOrder_Input = 1;
  const int kCatagoryOrder_Inference = 2;
  const int kCatagoryOrder_Output = 3;

  std::shared_ptr<PipelineParams> params_;

  std::shared_ptr<Input::BaseInputDevice> input_device_;
  std::string input_device_name_;
  std::multimap<std::string, std::string> next_;
  std::map<std::string, std::shared_ptr<openvino_wrapper_lib::BaseInference>> name_to_detection_map_;
  std::map<std::string, std::shared_ptr<Outputs::BaseOutput>> name_to_output_map_;
  int total_inference_ = 0;
  std::set<std::string> output_names_;
  int width_ = 0;
  int height_ = 0;
  cv::Mat frame_;
  // for multi threads
  std::atomic<int> counter_;
  std::mutex counter_mutex_;
  std::condition_variable cv_;
  int fps_ = 0;
  int frame_cnt_ = 0;
  std::chrono::time_point<std::chrono::high_resolution_clock> t_start_;
};

#endif  // OPENVINO_WRAPPER_LIB__PIPELINE_HPP_
