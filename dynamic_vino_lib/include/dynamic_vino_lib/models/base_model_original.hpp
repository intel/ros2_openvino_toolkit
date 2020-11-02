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
 * @brief A header file with declaration for BaseModel Class
 * @file base_model.h
 */

#ifndef DYNAMIC_VINO_LIB__MODELS__BASE_MODEL_HPP_
#define DYNAMIC_VINO_LIB__MODELS__BASE_MODEL_HPP_

#include <opencv2/opencv.hpp>

#include <vector>
#include <memory>
#include <string>
#include <map>
#include <iostream>
#include <fstream>

#include "inference_engine.hpp"
#include "dynamic_vino_lib/slog.hpp"
#include "dynamic_vino_lib/models/attributes/base_attribute.hpp"

namespace Engines
{
  class Engine;
}

namespace dynamic_vino_lib
{
  class ObjectDetectionResult;
}

namespace Models
{
  #if 0
  /**
 * @struct ModelAttr
 * @brief attributes for a given model
 */
  struct ModelAttr
  {
    int max_proposal_count = 0;
    int object_size = 0;
    int input_height = 0;
    int input_width = 0;
    std::string model_name;
    std::map<std::string, std::string> input_names;
    std::map<std::string, std::string> output_names;
    std::vector<std::string> labels;

    inline std::string getModelName() const
    {
      return model_name;
    }

    inline std::string getInputName(std::string &name)
    {
      std::map<std::string, std::string>::iterator it;
      it = input_names.find(name);
      if (it == input_names.end())
      {
        slog::warn << "No input named: " << name << slog::endl;
        return std::string("");
      }

      return it->second;
    }

    inline std::string getOutputName(std::string &name)
    {
      std::map<std::string, std::string>::iterator it;
      it = output_names.find(name);
      if (it == output_names.end())
      {
        slog::warn << "No output named: " << name << slog::endl;
        return std::string("");
      }

      return it->second;
    }

    inline int getMaxProposalCount() const
    {
      return max_proposal_count;
    }

    inline int getObjectSize() const
    {
      return object_size;
    }

    inline void addInputInfo(std::string &key, std::string &value)
    {
      input_names[key] = value;
    }

    inline void addOutputInfo(std::string &key, std::string &value)
    {
      output_names[key] = value;
    }

    inline void setInputHeight(const int height)
    {
      input_height = height;
    }

    inline void setInputWidth(const int width)
    {
      input_width = width;
    }

    inline void setModelName(const std::string& name)
    {
      model_name = name;
    }

    inline void loadLabelsFromFile(const std::string& file_path)
    {
      std::ifstream input_file(file_path);
      std::copy(std::istream_iterator<std::string>(input_file),
        std::istream_iterator<std::string>(),
        std::back_inserter(labels));
    }

    inline std::vector<std::string>& getLabels()
    {
      return labels;
    }

    inline void verify()
  {
    slog::info << "----Attributes for Model " << model_name << "----" << slog::endl;
    slog::info << "| model_name: " << model_name << slog::endl;
    slog::info << "| max_proposal_count: " << max_proposal_count << slog::endl;
    slog::info << "| object_size: " << object_size << slog::endl;
    slog::info << "| input_height: " << input_height << slog::endl;
    slog::info << "| input_width: " << input_width << slog::endl;
    slog::info << "| input_names: " << slog::endl;
    for (auto & item: input_names) {
      slog::info << "|    " << item.first << "-->" << item.second << slog::endl;
    }
    slog::info << "| output_names: " << slog::endl;
    for (auto & item: output_names) {
      slog::info << "|    " << item.first << "-->" << item.second << slog::endl;
    }

    if(max_proposal_count <= 0 || object_size <= 0 || input_height <= 0
      || input_width || input_names.empty() || output_names.empty()){
      slog::info << "--------" << slog::endl;
      slog::warn << "Not all attributes are set correctly! not 0 or empty is allowed in"
        << "the above list." << slog::endl;
    }
    slog::info << "--------------------------------" << slog::endl;
  }
  };
  #endif

  /**
 * @class BaseModel
 * @brief This class represents the network given by .xml and .bin file
 */
  class BaseModel : public ModelAttribute
  {
  public:
    using Ptr = std::shared_ptr<BaseModel>;
    /**
   * @brief Initialize the class with given .xml, .bin and .labels file. It will
   * also check whether the number of input and output are fit.
   * @param[in] model_loc The location of model' s .xml file
   * (model' s bin file should be the same as .xml file except for extension)
   * @param[in] input_num The number of input the network should have.
   * @param[in] output_num The number of output the network should have.
   * @param[in] batch_size The number of batch size (default: 1) the network should have.
   * @return Whether the input device is successfully turned on.
   */
    BaseModel(const std::string &model_loc, int batch_size = 1);
    //BaseModel(const std::string &model_loc, const size_t batch_size = 1);
    /**
   * @brief Get the maximum batch size of the model.
   * @return The maximum batch size of the model.
   */
    inline int getMaxBatchSize() const
    {
      return max_batch_size_;
    }
    inline void setMaxBatchSize(int max_batch_size)
    {
      max_batch_size_ = max_batch_size;
    }

    virtual bool enqueue(
        const std::shared_ptr<Engines::Engine> &engine,
        const cv::Mat &frame,
        const cv::Rect &input_frame_loc) { return true; }
    /**
   * @brief Initialize the model. During the process the class will check
   * the network input, output size, check layer property and
   * set layer property.
   */
    void modelInit();
    /**
   * @brief Get the name of the model.
   * @return The name of the model.
   */
    virtual const std::string getModelCategory() const = 0;
    inline ModelAttr getAttribute() { return attr_; }

    inline InferenceEngine::CNNNetReader::Ptr getNetReader() const
    {
      return net_reader_;
    }

  protected:
    /**
     * New infterface to check and update Layer Property
     * @brief Set the layer property (layer layout, layer precision, etc.).
     * @param[in] network_reader The reader of the network to be set.
     */
    virtual bool updateLayerProperty(InferenceEngine::CNNNetReader::Ptr network_reader) = 0;

    InferenceEngine::CNNNetReader::Ptr net_reader_;
    void setFrameSize(const int &w, const int &h)
    {
      frame_size_.width = w;
      frame_size_.height = h;
    }
    cv::Size getFrameSize()
    {
      return frame_size_;
    }

  private:
    int max_batch_size_;
    std::string model_loc_;
    cv::Size frame_size_;
  };

  class ObjectDetectionModel : public BaseModel
  {
  public:
    ObjectDetectionModel(const std::string &model_loc, int batch_size = 1);
    virtual bool fetchResults(
        const std::shared_ptr<Engines::Engine> &engine,
        std::vector<dynamic_vino_lib::ObjectDetectionResult> &result,
        const float &confidence_thresh = 0.3,
        const bool &enable_roi_constraint = false) = 0;
    virtual bool matToBlob(
        const cv::Mat &orig_image, const cv::Rect &, float scale_factor,
        int batch_index, const std::shared_ptr<Engines::Engine> &engine) = 0;
  };

} // namespace Models

#endif // DYNAMIC_VINO_LIB__MODELS__BASE_MODEL_HPP_
