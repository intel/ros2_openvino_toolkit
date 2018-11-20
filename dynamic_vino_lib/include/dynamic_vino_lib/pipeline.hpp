/*
 * Copyright (c) 2018 Intel Corporation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**
 * @brief a header file with declaration of Pipeline class
 * @file pipeline.h
 */
#ifndef DYNAMIC_VINO_LIB__PIPELINE_HPP_
#define DYNAMIC_VINO_LIB__PIPELINE_HPP_

#include <atomic>
#include <future>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <string>

#include "dynamic_vino_lib/inferences/base_inference.hpp"
#include "dynamic_vino_lib/inputs/standard_camera.hpp"
#include "dynamic_vino_lib/outputs/base_output.hpp"
#include "dynamic_vino_lib/pipeline_params.hpp"
#include "opencv2/opencv.hpp"

/**
 * @class Pipeline
 * @brief This class is a pipeline class that stores the topology of
 * the input device, output device and networks and make inference. One pipeline
 * should have only one input device.
 */
class Pipeline {
 public:
  explicit Pipeline(const std::string& name = "pipeline");
  /**
   * @brief Add input device to the pipeline.
   * @param[in] name name of the current input device.
   * @param[in] input_device the input device instance to be added.
   * @return whether the add operation is successful
   */
  bool add(const std::string& name,
           std::shared_ptr<Input::BaseInputDevice> input_device);
  /**
   * @brief Add inference network to the pipeline.
   * @param[in] parent name of the parent device or inference.
   * @param[in] name name of the current inference network.
   * @param[in] inference the inference instance to be added.
   * @return whether the add operation is successful
   */
  bool add(const std::string& parent, const std::string& name,
           std::shared_ptr<dynamic_vino_lib::BaseInference> inference);
  /**
   * @brief Add output device to the pipeline.
   * @param[in] parent name of the parent inference.
   * @param[in] name name of the current output device.
   * @param[in] output the output instance to be added.
   * @return whether the add operation is successful
   */
  bool add(const std::string& parent, const std::string& name,
           std::shared_ptr<Outputs::BaseOutput> output);

  bool add(const std::string& name,
           std::shared_ptr<Outputs::BaseOutput> output);
  void addConnect(const std::string& parent, const std::string& name);
  bool add(const std::string& name,
           std::shared_ptr<dynamic_vino_lib::BaseInference> inference);
  /**
   * @brief Add inference network-output device edge to the pipeline.
   * @param[in] parent name of the parent inference.
   * @param[in] name name of the current output device.
   * @return whether the add operation is successful
   */
  bool add(const std::string& parent, const std::string& name);
  /**
   * @brief Do the inference once.
   * Data flow from input device to inference network, then to output device.
   */
  void runOnce();
  /**
   * @brief The callback function provided for all the inference network in the
   * pipeline.
   */
  void callback(const std::string& detection_name);
  /**
   * @brief Set the inference network to call the callback function as soon as
   * each inference is
   * finished.
   */
  void setCallback();
  void printPipeline();
  void outputHandler();
  void setParams(PipelineParams pipeline_params) {
    params_ = std::make_shared<PipelineParams>(pipeline_params);
  };
  const std::shared_ptr<PipelineParams> getParameters() { return params_; };

 private:
  void initInferenceCounter();
  void increaseInferenceCounter();
  void decreaseInferenceCounter();
  bool isLegalConnect(const std::string parent, const std::string child);
  int getCatagoryOrder(const std::string name);

  const int kCatagoryOrder_Unknown = -1;
  const int kCatagoryOrder_Input = 1;
  const int kCatagoryOrder_Inference = 2;
  const int kCatagoryOrder_Output = 3;

  std::shared_ptr<PipelineParams> params_;

  std::shared_ptr<Input::BaseInputDevice> input_device_;
  std::string input_device_name_;
  std::multimap<std::string, std::string> next_;
  std::map<std::string, std::shared_ptr<dynamic_vino_lib::BaseInference>>
      name_to_detection_map_;
  std::map<std::string, std::shared_ptr<Outputs::BaseOutput>>
      name_to_output_map_;
  int total_inference_ = 0;
  std::set<std::string> output_names_;
  int width_ = 0;
  int height_ = 0;
  cv::Mat frame_;
  // for multi threads
  std::atomic<int> counter_;
  std::mutex counter_mutex_;
  std::condition_variable cv_;
};

#endif  // DYNAMIC_VINO_LIB__PIPELINE_HPP_
