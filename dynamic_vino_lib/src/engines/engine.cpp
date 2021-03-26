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
 * @brief a header file with definition of Engine class
 * @file engine.cpp
 */
#include "dynamic_vino_lib/engines/engine.hpp"
#include "dynamic_vino_lib/slog.hpp"

#if(defined(USE_OLD_E_PLUGIN_API))
Engines::Engine::Engine(
  InferenceEngine::InferencePlugin plg,
  const Models::BaseModel::Ptr base_model)
{
  request_ = (plg.LoadNetwork(base_model->getNetReader()->getNetwork(), {})).CreateInferRequestPtr();
}
#endif

Engines::Engine::Engine(
  InferenceEngine::InferRequest::Ptr & request)
{
  request_ = request;
}

Engines::Engine::Engine(
  const std::string & device)
  : device_(device)
 {
 }

InferenceEngine::CNNNetwork &
Engines::Engine::prepareNetwork(const std::string model, const int batch)
{
  network_ = ie_.ReadNetwork(model);
  executable_network_ = ie_.LoadNetwork(network_, device_);
  request_ = executable_network_.CreateInferRequestPtr();

  slog::info << "Engine is used:  " << device_ << slog::endl;
  // Set batch size to given max_batch_size_
  slog::info << "Batch size is set to  " << batch << slog::endl;
  network_.setBatchSize(batch);

  return network_;
}

InferenceEngine::CNNNetwork *
Engines::Engine::getNetwork()
{
  return &network_;
}