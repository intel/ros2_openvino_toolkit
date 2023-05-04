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

//
// @brief a header file with version information about Inference Engine.
// @file version_info.hpp
//

#ifndef OPENVINO_WRAPPER_LIB__UTILS__VERSION_INFO_HPP_
#define OPENVINO_WRAPPER_LIB__UTILS__VERSION_INFO_HPP_

#if(defined(USE_OLD_E_PLUGIN_API))
#include <ie_device.hpp>
#endif
#include <string>
#include <map>
#include <vector>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <utility>

#ifdef WIN32
#define UNUSED
#else
#define UNUSED __attribute__((unused))
#endif

/**
 * @brief Trims from both ends (in place)
 * @param s - string to trim
 * @return trimmed string
 */
inline std::string & trim(std::string & s)
{
  s.erase(s.begin(),
    std::find_if(s.begin(), s.end(), std::not1(std::ptr_fun<int, int>(std::isspace))));
  s.erase(
    std::find_if(s.rbegin(), s.rend(), std::not1(std::ptr_fun<int, int>(std::isspace))).base(),
    s.end());
  return s;
}

static std::ostream & operator<<(std::ostream & os, const ov::Version& version)
{
  os << "\n\tAPI version ............ ";
  os << OPENVINO_VERSION_MAJOR << "." << OPENVINO_VERSION_MINOR << "." << OPENVINO_VERSION_PATCH;
  os << "\n\t" <<
    "Build .................. " << version.buildNumber;
  os << "\n\t" <<
    "Description ............ " << version.description;

  return os;
}

#if(defined(USE_OLD_E_PLUGIN_API))
/**
 * @class PluginVersion
 * @brief A PluginVersion class stores plugin version and initialization status
 */
struct PluginVersion : public InferenceEngine::Version
{
  bool initialized = false;

  explicit PluginVersion(const InferenceEngine::Version * ver)
  {
    if (nullptr == ver) {
      return;
    }
    InferenceEngine::Version::operator=(*ver);
    initialized = true;
  }

  operator bool() const noexcept
  {
    return initialized;
  }
};

static UNUSED std::ostream & operator<<(std::ostream & os, const PluginVersion & version)
{
  os << "\tPlugin version ......... ";
  if (!version) {
    os << "UNKNOWN";
  } else {
    os << version.apiVersion.major << "." << version.apiVersion.minor;
  }

  os << "\n\tPlugin name ............ ";
  if (!version || version.description == nullptr) {
    os << "UNKNOWN";
  } else {
    os << version.description;
  }

  os << "\n\tPlugin build ........... ";
  if (!version || version.buildNumber == nullptr) {
    os << "UNKNOWN";
  } else {
    os << version.buildNumber;
  }

  return os;
}

inline void printPluginVersion(InferenceEngine::InferenceEnginePluginPtr ptr, std::ostream & stream)
{
  const PluginVersion * pluginVersion = nullptr;
  ptr->GetVersion((const InferenceEngine::Version * &)pluginVersion);
  stream << pluginVersion << std::endl;
}
#endif // (defined(USE_OLD_E_PLUGIN_API))

#endif  // OPENVINO_WRAPPER_LIB__UTILS__VERSION_INFO_HPP_
