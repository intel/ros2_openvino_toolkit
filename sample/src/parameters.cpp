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
* \brief A sample for vino_param_manager library. This sample performs
* getting/setting
* parameters for vino related functions.
* \file sample/parameters.cpp
*/

#include <openvino_param_lib/param_manager.hpp>
#include <openvino_wrapper_lib/slog.hpp>
#include <fstream>
#include <functional>
#include <iostream>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>
#include "utility.hpp"

int main(int argc, char * argv[])
{
  try {
    // ------Parsing and validation of input args---------
    std::string config = getConfigPath(argc, argv);
    if(config.empty()){
      throw std::runtime_error("Config File is not correctly set.");
      return -1;
    }

    Params::ParamManager::getInstance().parse(config);
    Params::ParamManager::getInstance().print();

    slog::info << "print again, should same as above....." << slog::endl;
    Params::ParamManager::getInstance().print();
  } catch (const std::exception & error) {
    slog::err << error.what() << slog::endl;
    return -1;
  } catch (...) {
    slog::err << "Unknown/internal exception happened." << slog::endl;
    return -2;
  }

  return 0;
}
