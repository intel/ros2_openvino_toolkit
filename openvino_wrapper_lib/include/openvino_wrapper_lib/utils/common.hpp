// Copyright (C) 2018-2019 Intel Corporation
// SPDX-License-Identifier: Apache-2.0
//

/**
 * @brief a header file with common samples functionality
 * @file common.hpp
 */

#pragma once

#include <string>
#include <map>
#include <vector>
#include <list>
#include <limits>
#include <functional>
#include <fstream>
#include <iomanip>
#include <utility>
#include <algorithm>
#include <random>
#include <iostream>

#include <openvino/openvino.hpp>

#ifndef UNUSED
#ifdef _WIN32
#define UNUSED
#else
#define UNUSED __attribute__((unused))
#endif
#endif

template <typename T, std::size_t N>
constexpr std::size_t arraySize(const T (&)[N]) noexcept
{
  return N;
}

// Helpers to print IE version information.
// We don't directly define operator<< for InferenceEngine::Version
// and such, because that won't get picked up by argument-dependent lookup
// due to not being in the same namespace as the Version class itself.
// We need ADL to work in order to print these objects using slog.
// So instead, we define wrapper classes and operator<< for those classes.

class PrintableOvVersion
{
public:
  using ref_type = const ov::Version&;

  PrintableOvVersion(ref_type version) : version(version)
  {
  }

  friend std::ostream& operator<<(std::ostream& os, const PrintableOvVersion& p)
  {
    ref_type version = p.version;

    return os << "\t" << version.description << " version ......... " << version.buildNumber;
  }

private:
  ref_type version;
};

inline PrintableOvVersion printable(PrintableOvVersion::ref_type version)
{
  return { version };
}

class PrintableOvVersionMap
{
public:
  using ref_type = const std::map<std::string, ov::Version>&;

  PrintableOvVersionMap(ref_type versions) : versions(versions)
  {
  }

  friend std::ostream& operator<<(std::ostream& os, const PrintableOvVersionMap& p)
  {
    ref_type versions = p.versions;

    for (const auto& version : versions) {
      os << "\t" << version.first << std::endl << printable(version.second) << std::endl;
    }

    return os;
  }

private:
  ref_type versions;
};

inline PrintableOvVersionMap printable(PrintableOvVersionMap::ref_type versions)
{
  return { versions };
}

/**
 * @class Color
 * @brief A Color class stores channels of a given color
 */
class Color
{
private:
  unsigned char _r;
  unsigned char _g;
  unsigned char _b;

public:
  /**
   * A default constructor.
   * @param r - value for red channel
   * @param g - value for green channel
   * @param b - value for blue channel
   */
  Color(unsigned char r, unsigned char g, unsigned char b) : _r(r), _g(g), _b(b)
  {
  }

  inline unsigned char red() const
  {
    return _r;
  }

  inline unsigned char blue() const
  {
    return _b;
  }

  inline unsigned char green() const
  {
    return _g;
  }
};

// Known colors for training classes from the Cityscapes dataset
static UNUSED const Color CITYSCAPES_COLORS[] = {
  { 128, 64, 128 }, { 232, 35, 244 }, { 70, 70, 70 },   { 156, 102, 102 }, { 153, 153, 190 }, { 153, 153, 153 },
  { 30, 170, 250 }, { 0, 220, 220 },  { 35, 142, 107 }, { 152, 251, 152 }, { 180, 130, 70 },  { 60, 20, 220 },
  { 0, 0, 255 },    { 142, 0, 0 },    { 70, 0, 0 },     { 100, 60, 0 },    { 90, 0, 0 },      { 230, 0, 0 },
  { 32, 11, 119 },  { 0, 74, 111 },   { 81, 0, 81 }
};

static std::vector<std::pair<std::string, ov::ProfilingInfo>>
perfCountersSorted(std::map<std::string, ov::ProfilingInfo> perfMap)
{
  using perfItem = std::pair<std::string, ov::ProfilingInfo>;
  std::vector<perfItem> sorted;
  for (auto& kvp : perfMap)
    sorted.push_back(kvp);

  std::stable_sort(sorted.begin(), sorted.end(), [](const perfItem& l, const perfItem& r) {
    return l.second.real_time.count() < r.second.real_time.count();
  });

  return sorted;
}

static UNUSED void printPerformanceCounts(const std::map<std::string, ov::ProfilingInfo>& performanceMap,
                                          std::ostream& stream, const std::string& deviceName, bool bshowHeader = true)
{
  long long totalTime = 0;
  // Print performance counts
  if (bshowHeader) {
    stream << std::endl << "performance counts:" << std::endl << std::endl;
  }

  auto performanceMapSorted = perfCountersSorted(performanceMap);

  for (const auto& it : performanceMapSorted) {
    std::string toPrint(it.first);
    const int maxLayerName = 30;

    if (it.first.length() >= maxLayerName) {
      toPrint = it.first.substr(0, maxLayerName - 4);
      toPrint += "...";
    }

    stream << std::setw(maxLayerName) << std::left << toPrint;
    switch (it.second.status) {
      case ov::ProfilingInfo::Status::EXECUTED:
        stream << std::setw(15) << std::left << "EXECUTED";
        break;
      case ov::ProfilingInfo::Status::NOT_RUN:
        stream << std::setw(15) << std::left << "NOT_RUN";
        break;
      case ov::ProfilingInfo::Status::OPTIMIZED_OUT:
        stream << std::setw(15) << std::left << "OPTIMIZED_OUT";
        break;
    }
    stream << std::setw(30) << std::left << "layerType: " + std::string(it.second.node_type) + " ";
    stream << std::setw(20) << std::left << "realTime: " + std::to_string(it.second.real_time.count());
    stream << std::setw(20) << std::left << "cpu: " + std::to_string(it.second.cpu_time.count());
    stream << " execType: " << it.second.exec_type << std::endl;
    if (it.second.real_time.count() > 0) {
      totalTime += it.second.real_time.count();
    }
  }
  stream << std::setw(20) << std::left << "Total time: " + std::to_string(totalTime) << " microseconds" << std::endl;
  std::cout << std::endl;
  std::cout << "Full device name: " << deviceName << std::endl;
  std::cout << std::endl;
}

static UNUSED void printPerformanceCounts(ov::InferRequest request, std::ostream& stream, std::string deviceName,
                                          bool bshowHeader = true)
{
  auto performanceMap = request.get_profiling_info();
  //printPerformanceCounts(performanceMap, stream, deviceName, bshowHeader);
}

inline std::map<std::string, std::string> getMapFullDevicesNames(ov::Core& core, std::vector<std::string> devices)
{
  std::map<std::string, std::string> devicesMap;
  ov::Any p;
  for (std::string& deviceName : devices) {
    if (deviceName != "") {
      try {
        std::string fullDeviceName = core.get_property(deviceName, ov::device::full_name);
        devicesMap.insert(std::pair<std::string, std::string>(deviceName, fullDeviceName));
      } catch (ov::Exception&) {
      }
    }
  }
  return devicesMap;
}

inline std::string getFullDeviceName(std::map<std::string, std::string>& devicesMap, std::string device)
{
  std::map<std::string, std::string>::iterator it = devicesMap.find(device);
  if (it != devicesMap.end()) {
    return it->second;
  } else {
    return "";
  }
}

inline std::string getFullDeviceName(ov::Core& core, std::string device)
{
  ov::Any p;
  try {
    return core.get_property(device, ov::device::full_name);
  } catch (ov::Exception&) {
    return "";
  }
}

inline std::size_t getTensorWidth(const ov::Tensor& tensor)
{
  const auto& shape = tensor.get_shape();
  if (shape.size() >= 2) {
    return shape.back();
  } else {
    throw std::runtime_error("Tensor does not have width dimension");
  }
  return 0;
}

inline std::size_t getTensorHeight(const ov::Tensor& tensor)
{
  const auto& shape = tensor.get_shape();
  if (shape.size() >= 2) {
    return shape.at(shape.size() - 2);
  } else {
    throw std::runtime_error("Tensor does not have height dimension");
  }
  return 0;
}

inline std::size_t getTensorChannels(const ov::Tensor& tensor)
{
  const auto& shape = tensor.get_shape();
  if (shape.size() >= 3) {
    return shape.at(1);
  } else {
    throw std::runtime_error("Tensor does not have channels dimension");
  }
  return 0;
}

inline std::size_t getTensorBatch(const ov::Tensor& tensor)
{
  const auto& shape = tensor.get_shape();
  if (shape.size() >= 4) {
    return shape.at(0);
  } else {
    throw std::runtime_error("Tensor does not have batch dimension");
  }
  return 0;
}

inline void showAvailableDevices()
{
  ov::Core core;
  std::vector<std::string> devices = core.get_available_devices();

  std::cout << std::endl;
  std::cout << "Available target devices:";
  for (const auto& device : devices) {
    std::cout << "  " << device;
  }
  std::cout << std::endl;
}

inline std::string fileNameNoExt(const std::string& filepath)
{
  auto pos = filepath.rfind('.');
  if (pos == std::string::npos)
    return filepath;
  return filepath.substr(0, pos);
}

static inline ov::Layout getLayoutFromShape(const ov::Shape& shape)
{
  if (shape.size() == 2) {
    return "NC";
  } else if (shape.size() == 3) {
    return (shape[0] >= 1 && shape[0] <= 4) ? "CHW" : "HWC";
  } else if (shape.size() == 4) {
    return (shape[1] >= 1 && shape[1] <= 4) ? "NCHW" : "NHWC";
  } else {
    throw std::runtime_error("Unsupported " + std::to_string(shape.size()) + "D shape");
  }
}
