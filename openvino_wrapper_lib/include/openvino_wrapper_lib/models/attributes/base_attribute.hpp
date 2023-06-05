// Copyright (c) 2018-2023 Intel Corporation
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

#ifndef OPENVINO_WRAPPER_LIB__MODELS__ATTRIBUTES_BASE_ATTRIBUTE_HPP_
#define OPENVINO_WRAPPER_LIB__MODELS__ATTRIBUTES_BASE_ATTRIBUTE_HPP_

#include <vector>
#include <map>
#include <string>
#include <fstream>
#include <iterator>

#include "openvino/openvino.hpp"
#include "openvino_wrapper_lib/slog.hpp"

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
  const char* DefaultInputName {"input0"};
  const char* DefaultOutputName = "output0";
  struct ModelAttr {
    // Input Tensor Size 
    int input_height = 0;
    int input_width = 0;

    //Input/Output Tensor Info
    int input_tensor_count = 1;  // The number of input tensors
    int output_tensor_count = 1; // The number of output tensors
    bool has_confidence_output = true; //Yolov5~7 have a float for confidence, while yolov8 hasn't.
    bool need_transpose = false; // If the output tensor needs transpose
    int max_proposal_count = 0; // The max number of objects in inference output tensor
    int object_size = 0; //The size of each object in inference output tensor
    std::map<std::string, std::string> input_names;
    std::map<std::string, std::string> output_names;

    std::string model_name;
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
    slog::info << "-------------------- Attributes for Model <Begin>----------------------" << slog::endl;
    slog::info << "| model_name: " << attr_.model_name << slog::endl;
    slog::info << "| max_proposal_count: " << attr_.max_proposal_count << slog::endl;
    slog::info << "| object_size: " << attr_.object_size << slog::endl;
    slog::info << "| input_height: " << attr_.input_height << slog::endl;
    slog::info << "| input_width: " << attr_.input_width << slog::endl;
    slog::info << "| input_tensor_count: " << attr_.input_tensor_count << slog::endl;
    slog::info << "| output_tensor_count: " << attr_.output_tensor_count << slog::endl;
    slog::info << "| need_transpose (max_proposal_count < object_size): " << std::boolalpha
               << attr_.need_transpose << slog::endl;
    slog::info << "| has_confidence_output: " << std::boolalpha << attr_.has_confidence_output <<
      slog::endl;

    slog::info << "| input_names: " << slog::endl;
    for (auto & item: attr_.input_names) {
      slog::info << "|    " << item.first << "-->" << item.second << slog::endl;
    }
    slog::info << "| output_names: " << slog::endl;
    for (auto & item: attr_.output_names) {
      slog::info << "|    " << item.first << "-->" << item.second << slog::endl;
    }

    slog::info << "| lables:" << slog::endl;
    for (size_t i = 0; i<attr_.labels.size(); i++){
      if (i % 8 == 0 ) slog::info << "|    ";
      slog::info  << "[" <<  i << ":" << attr_.labels[i] << "]";
      if (i % 8 == 7) slog::info << slog::endl;
    }
    slog::info << slog::endl;

    if(attr_.max_proposal_count <= 0 || attr_.object_size <= 0 || attr_.input_height <= 0
      || attr_.input_width <= 0 || attr_.input_names.empty() || attr_.output_names.empty()){
      slog::info << "--------" << slog::endl;
      slog::warn << "Not all attributes are set correctly! not 0 or empty is allowed in"
        << " the above list." << slog::endl;
    }
    if( attr_.input_tensor_count != static_cast<int>(attr_.input_names.size())){
      slog::info << "--------" << slog::endl;
      slog::warn << "The count of input_tensor(s) is not aligned with input names!"
        << slog::endl;
    }
    if( attr_.output_tensor_count != static_cast<int>(attr_.output_names.size())){
      slog::info << "--------" << slog::endl;
      slog::warn << "The count of output_tensor(s) is not aligned with output names!"
        << slog::endl;
    }
    slog::info << "-------------------- Attributes for Model <End>----------------------" << slog::endl;
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

  inline std::string getInputName(std::string name = "input0") const
  {
    auto it = attr_.input_names.find(name);
    if(it == attr_.input_names.end()){
      slog::warn << "No input named: " << name << slog::endl;
      return std::string("");
    }
    
    return it->second;
  }

  inline std::string getOutputName(std::string name = "output0") const
  {
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
    for(std::string name; std::getline(input_file, name);){
      attr_.labels.push_back(name);
    }
  }

  inline std::vector<std::string>& getLabels()
  {
    return attr_.labels;
  }

  inline void addInputInfo(std::string key, std::string value)
  {
    attr_.input_names[key] = value;
  }

  inline const std::string getInputInfo(std::string key)
  {
    return attr_.input_names[key];
  }

  inline void addOutputInfo(std::string key, std::string value)
  {
    attr_.output_names[key] = value;
  }

  inline void setInputHeight(const int height)
  {
    attr_.input_height = height;
  }

  inline int getInputHeight() const
  {
    return attr_.input_height;
  }

  inline void setInputWidth(const int width)
  {
    attr_.input_width = width;
  }

  inline int getInputWidth() const
  {
    return attr_.input_width;
  }

  inline void setMaxProposalCount(const int max)
  {
    attr_.max_proposal_count = max;
  }

  inline void setObjectSize(const int size)
  {
    attr_.object_size = size;
  }

  inline void setHasConfidenceOutput(const bool has)
  {
    attr_.has_confidence_output = has;
  }

  inline bool hasConfidenceOutput() const
  {
    return attr_.has_confidence_output;
  }

  inline void setCountOfInputs(const int count)
  {
    attr_.input_tensor_count = count;
  }

  inline int getCountOfInputs() const
  {
    return attr_.input_tensor_count;
  }

  inline void setCountOfOutputs(const int count)
  {
    attr_.output_tensor_count = count;
  }

  inline int getCountOfOutputs() const
  {
    return attr_.output_tensor_count;
  }

  inline void setTranspose(bool trans)
  {
    attr_.need_transpose = trans;
  }

  inline bool needTranspose() const
  {
    return attr_.need_transpose;
  }

  inline bool _renameMapKeyByValue(std::map<std::string, std::string>& map,
    const std::string& value, const std::string& new_key)
  {
    for (auto& item: map){
      auto n = item.second.find(value);
      if (std::string::npos != n) {
      //if(item.second.contains(value)){
        auto nh = map.extract(item.first);
        nh.key() = new_key;
        map.insert(std::move(nh));
        return true;
      }
    }

    return false;
  }

  inline bool retagOutputByValue(const std::string& value, const std::string& new_tag)
  {
    return _renameMapKeyByValue(attr_.output_names, value, new_tag);
  }

  inline bool retagInputByValue(const std::string& value, const std::string& new_tag)
  {
    return _renameMapKeyByValue(attr_.input_names, value, new_tag);
  }
protected:
  ModelAttr attr_;
  std::string input_tensor_name_;
  std::string output_tensor_name_;
  std::vector<ov::Output<ov::Node>> inputs_info_;
  std::vector<ov::Output<ov::Node>> outputs_info_;
};

class SSDModelAttr : public ModelAttribute
{
public:
  explicit SSDModelAttr(const std::string model_name = "SSDNet-like");

  bool updateLayerProperty(
    const std::shared_ptr<ov::Model>&);

};

}  // namespace Models

#endif  // OPENVINO_WRAPPER_LIB__MODELS__ATTRIBUTES_BASE_ATTRIBUTE_HPP_
