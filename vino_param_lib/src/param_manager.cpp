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

#include "vino_param_lib/param_manager.hpp"
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <iostream>
#include <map>
#include <string>
#include <vector>
#include <vino_param_lib/slog.hpp>

namespace Params {

void operator>>(const YAML::Node& node, ParamManager::PipelineParams& pipeline);
void operator>>(const YAML::Node& node,
                std::vector<ParamManager::InferenceParams>& list);
void operator>>(const YAML::Node& node, ParamManager::InferenceParams& infer);
void operator>>(const YAML::Node& node, std::vector<std::string>& list);
void operator>>(const YAML::Node& node,
                std::multimap<std::string, std::string>& connect);
void operator>>(const YAML::Node& node, std::string& str);
void operator>>(const YAML::Node& node, bool& val);
void operator>>(const YAML::Node& node, ParamManager::CommonParams& common);

#define YAML_PARSE(node, key, val)                                       \
  try {                                                                  \
    node[key] >> val;                                                    \
  } catch (YAML::Exception e) {                                          \
    slog::warn << e.msg << slog::endl;                                   \
  } catch (...) {                                                        \
    slog::warn << "Exception occurs when parsing string." << slog::endl; \
  }

void operator>>(const YAML::Node& node,
                std::vector<ParamManager::PipelineParams>& list) {
  slog::info << "Pipeline size: " << node.size() << slog::endl;
  for (unsigned i = 0; i < node.size(); i++) {
    ParamManager::PipelineParams temp;
    node[i] >> temp;
    list.push_back(temp);
  }
}

void operator>>(const YAML::Node& node, ParamManager::CommonParams& common) {
  YAML_PARSE(node, "camera_topic", common.camera_topic)
  YAML_PARSE(node, "custom_cpu_library", common.custom_cpu_library)
  YAML_PARSE(node, "custom_cldnn_library", common.custom_cldnn_library)
  YAML_PARSE(node, "enable_performance_count", common.enable_performance_count)
}

void operator>>(const YAML::Node& node,
                ParamManager::PipelineParams& pipeline) {
  YAML_PARSE(node, "name", pipeline.name)
  YAML_PARSE(node, "inputs", pipeline.inputs)
  YAML_PARSE(node, "infers", pipeline.infers)
  YAML_PARSE(node, "outputs", pipeline.outputs)
  YAML_PARSE(node, "connects", pipeline.connects)
  YAML_PARSE(node, "input_path", pipeline.input_meta)
  slog::info << "Pipeline Params:name=" << pipeline.name << slog::endl;
}

void operator>>(const YAML::Node& node,
                std::vector<ParamManager::InferenceParams>& list) {
  slog::info << "Inferences size: " << node.size() << slog::endl;
  for (unsigned i = 0; i < node.size(); i++) {
    ParamManager::InferenceParams temp_inf;
    node[i] >> temp_inf;
    list.push_back(temp_inf);
  }
}

void operator>>(const YAML::Node& node, ParamManager::InferenceParams& infer) {
  YAML_PARSE(node, "name", infer.name)
  YAML_PARSE(node, "model", infer.model)
  YAML_PARSE(node, "engine", infer.engine)
  YAML_PARSE(node, "label", infer.label)
  slog::info << "Inference Params:name=" << infer.name << slog::endl;
}

void operator>>(const YAML::Node& node, std::vector<std::string>& list) {
  for (unsigned i = 0; i < node.size(); i++) {
    std::string temp_i;
    node[i] >> temp_i;
    list.push_back(temp_i);
  }
}

void operator>>(const YAML::Node& node,
                std::multimap<std::string, std::string>& connect) {
  for (unsigned i = 0; i < node.size(); i++) {
    std::string left;
    node[i]["left"] >> left;
    std::vector<std::string> rights;
    node[i]["right"] >> rights;
    for (auto& r : rights) {
      connect.insert({left, r});
    }
  }
}

void operator>>(const YAML::Node& node, std::string& str) {
  str = node.as<std::string>();
}

void operator>>(const YAML::Node& node, bool& val) { val = node.as<bool>(); }

void ParamManager::print() const {
  slog::info << "--------parameters DUMP---------------------" << slog::endl;
  for (auto& pipeline : pipelines_) {
    slog::info << "Pipeline: " << pipeline.name << slog::endl;
    slog::info << "\tInputs: ";
    for (auto& i : pipeline.inputs) {
      slog::info << i.c_str() << ", ";
    }
    slog::info << slog::endl;

    slog::info << "\tOutputs: ";
    for (auto& i : pipeline.outputs) {
      slog::info << i.c_str() << ", ";
    }
    slog::info << slog::endl;

    slog::info << "\tInferences: " << slog::endl;
    for (auto& infer : pipeline.infers) {
      slog::info << "\t\tName: " << infer.name << slog::endl;
      slog::info << "\t\tModel: " << infer.model << slog::endl;
      slog::info << "\t\tEngine: " << infer.engine << slog::endl;
      slog::info << "\t\tLabel: " << infer.label << slog::endl;
    }

    slog::info << "\tConnections: " << slog::endl;
    for (auto& c : pipeline.connects) {
      slog::info << "\t\t" << c.first << "->" << c.second << slog::endl;
    }
  }

  // Pring Common Info
  slog::info << "Common:" << slog::endl;
  slog::info << "\tcamera_topic: " << common_.camera_topic << slog::endl;
  slog::info << "\tcustom_cpu_library: " << common_.custom_cpu_library
             << slog::endl;
  slog::info << "\tcustom_cldnn_library: " << common_.custom_cldnn_library
             << slog::endl;
  slog::info << "\tenable_performance_count: "
             << common_.enable_performance_count << slog::endl;
}

void ParamManager::parse(std::string path) {
  std::ifstream fin(path);
  if (fin.fail()) {
    slog::err << "Could not open config file:" << path << slog::endl;
    return;
  }
  YAML::Node doc = YAML::Load(fin);

  YAML_PARSE(doc, "Pipelines", pipelines_)
  YAML_PARSE(doc, "Common", common_)
}

std::vector<std::string> ParamManager::getPipelineNames() const {
  std::vector<std::string> names;
  for (auto& p : pipelines_) {
    names.push_back(p.name);
  }

  return names;
}

ParamManager::PipelineParams ParamManager::getPipeline(
    const std::string& name) const {
  for (auto& p : pipelines_) {
    if (p.name == name) {
      return p;
    }
  }
  throw std::logic_error("No parameters found for pipeline [" + name + "]");
}
}  // namespace Params
