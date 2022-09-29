// Copyright (c) 2018-2020 Intel Corporation
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
 * @brief A header file with declaration for ModelAttribute class.
 * @file base_attribute.hpp
 */

#ifndef DYNAMIC_VINO_LIB__MODELS__ATTRIBUTES_BASE_ATTRIBUTE_HPP_
#define DYNAMIC_VINO_LIB__MODELS__ATTRIBUTES_BASE_ATTRIBUTE_HPP_

#include <vector>
#include <map>
#include <string>
#include <fstream>

// #include "inference_engine.hpp"
#include "openvino/openvino.hpp"
#include "dynamic_vino_lib/slog.hpp"

namespace Models
{
/**
 * @class ModelAttribute
 * @brief This class represents the network given by .xml and .bin file
 */
class ModelAttribute
{
public:
  using Ptr = std::shared_ptr<ModelAttribute>;
  struct ModelAttr {
    int max_proposal_count = 0;
    int object_size = 0;
    int input_height = 0;
    int input_width = 0;
    std::string model_name;
    std::map<std::string, std::string> input_names;
    std::map<std::string, std::string> output_names;
    std::vector<std::string> labels;
  };
  
  ModelAttribute(const std::string model_name)
  {
    attr_.model_name = model_name;
  }

  inline bool isVerified()
  {
    return (attr_.max_proposal_count > 0 && attr_.object_size > 0 && attr_.input_height > 0
      && attr_.input_width > 0 && attr_.input_names.empty() && attr_.output_names.empty());
  }
  inline void printAttribute()
  {
    slog::info << "----Attributes for Model " << attr_.model_name << "----" << slog::endl;
    slog::info << "| model_name: " << attr_.model_name << slog::endl;
    slog::info << "| max_proposal_count: " << attr_.max_proposal_count << slog::endl;
    slog::info << "| object_size: " << attr_.object_size << slog::endl;
    slog::info << "| input_height: " << attr_.input_height << slog::endl;
    slog::info << "| input_width: " << attr_.input_width << slog::endl;
    slog::info << "| input_names: " << slog::endl;
    for (auto & item: attr_.input_names) {
      slog::info << "|    " << item.first << "-->" << item.second << slog::endl;
    }
    slog::info << "| output_names: " << slog::endl;
    for (auto & item: attr_.output_names) {
      slog::info << "|    " << item.first << "-->" << item.second << slog::endl;
    }

    if(attr_.max_proposal_count <= 0 || attr_.object_size <= 0 || attr_.input_height <= 0
      || attr_.input_width <= 0 || attr_.input_names.empty() || attr_.output_names.empty()){
      slog::info << "--------" << slog::endl;
      slog::warn << "Not all attributes are set correctly! not 0 or empty is allowed in"
        << " the above list." << slog::endl;
    }
    slog::info << "--------------------------------" << slog::endl;
  }

  virtual bool updateLayerProperty(
    const std::shared_ptr<ov::Model>&)
  { return false; }

  inline std::string getModelName() const
  {
    return attr_.model_name;
  }

  inline void setModelName(std::string name)
  {
    attr_.model_name = name;
  }

  inline std::string getInputName(std::string name = "input") const
  {
    // std::map<std::string, std::string>::iterator it;
    auto it = attr_.input_names.find(name);
    if(it == attr_.input_names.end()){
      slog::warn << "No input named: " << name << slog::endl;
      return std::string("");
    }
    
    return it->second;
  }

  inline std::string getOutputName(std::string name = "output") const
  {
    //std::map<std::string, std::string>::iterator it;
    auto it = attr_.output_names.find(name);
    if(it == attr_.output_names.end()){
      slog::warn << "No output named: " << name << slog::endl;
      return std::string("");
    }
    
    return it->second;
  }

  inline int getMaxProposalCount() const
  {
    return attr_.max_proposal_count;
  }

  inline int getObjectSize() const
  {
    return attr_.object_size;
  }

  inline void loadLabelsFromFile(const std::string file_path)
  {
    std::ifstream input_file(file_path);
    std::copy(std::istream_iterator<std::string>(input_file),
      std::istream_iterator<std::string>(),
      std::back_inserter(attr_.labels));
    }

  inline std::vector<std::string>& getLabels()
  {
    return attr_.labels;
  }

  inline void addInputInfo(std::string key, std::string value)
  {
    attr_.input_names[key] = value;
  }

  inline void addOutputInfo(std::string key, std::string value)
  {
    attr_.output_names[key] = value;
  }

  inline void setInputHeight(const int height)
  {
    attr_.input_height = height;
  }

  inline void setInputWidth(const int width)
  {
    attr_.input_width = width;
  }

  inline void setMaxProposalCount(const int max)
  {
    attr_.max_proposal_count = max;
  }

  inline void setObjectSize(const int size)
  {
    attr_.object_size = size;
  }

protected:
  ModelAttr attr_;

};

class SSDModelAttr : public ModelAttribute
{
public:
  explicit SSDModelAttr(const std::string model_name = "SSDNet-like");

  bool updateLayerProperty(
    const std::shared_ptr<ov::Model>&);

};



}  // namespace Models

#endif  // DYNAMIC_VINO_LIB__MODELS__ATTRIBUTES_BASE_ATTRIBUTE_HPP_
