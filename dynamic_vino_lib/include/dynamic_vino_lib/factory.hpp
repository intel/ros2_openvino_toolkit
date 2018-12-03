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
 * @brief a header file with declaration of Factory class
 * @file factory.hpp
 */

#ifndef DYNAMIC_VINO_LIB__FACTORY_HPP_
#define DYNAMIC_VINO_LIB__FACTORY_HPP_


#include <inference_engine.hpp>

#include <memory>
#include <string>

#include "dynamic_vino_lib/inputs/base_input.hpp"
#include "extension/ext_list.hpp"
#include "dynamic_vino_lib/common.hpp"

/**
* @class Factory
* @brief This class is a factory class that produces the derived input device
* class corresponding to
* the input string
*/
class Factory {
 public:
  /**
  * @brief This function produces the derived input device class corresponding
  * to the input string
  * @param[in] input device name, can be RealSenseCamera, StandardCamera or
  * video directory
  * @param[in] input file path, file path for a video or image file
  * @return the instance of derived input device referenced by a smart pointer
  */
  static std::shared_ptr<Input::BaseInputDevice> makeInputDeviceByName(
      const std::string& input_device_name, const std::string& input_file_path="");
  /**
  * @brief This function produces the derived inference plugin corresponding to
  * the input string
  * @param[in] device_name The name of target device (CPU, GPU, FPGA, MYRIAD)
  * @param[in] custom_cpu_library_message Absolute path to CPU library with user
  * layers
  * @param[in] custom_cldnn_message  clDNN custom kernels path
  * @param[in] performance_message Enable per-layer performance report
  * @return the instance of derived inference plugin referenced by a smart
  * pointer
  */
  static std::unique_ptr<InferenceEngine::InferencePlugin> makePluginByName(
      const std::string& device_name,
      const std::string& custom_cpu_library_message,
      const std::string& custom_cldnn_message, bool performance_message);
};

#endif  // DYNAMIC_VINO_LIB__FACTORY_HPP_
