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

#ifndef UTILITY_HPP_
#define UTILITY_HPP_

#include <gflags/gflags.h>

#include <string>
#include <vector>
#include <iostream>

#ifdef _WIN32
#include <os/windows/w_dirent.h>
#else
#include <dirent.h>
#endif

/// @brief message for help argument
static const char help_message[] = "Print a usage message.";

/// @brief message absolute path of parameter config file
static const char parameter_file_message[] = "Absolute path of parameter config file.";

/// \brief Define flag for showing help message <br>
DEFINE_bool(h, false, help_message);

/// @brief Absolute path to yaml-format configuration file <br>
/// It is a optional parameter
DEFINE_string(config, "", parameter_file_message);

/**
* \brief This function show a help message
*/
static void showUsageForParam()
{
  std::cout << std::endl;
  std::cout << "vino_param_sample [OPTION]" << std::endl;
  std::cout << "Options:" << std::endl;
  std::cout << std::endl;
  std::cout << "    -h                         " << help_message << std::endl;
  std::cout << "    -config \"<path>\"         " << parameter_file_message << std::endl;
}

#endif  // UTILITY_HPP_
