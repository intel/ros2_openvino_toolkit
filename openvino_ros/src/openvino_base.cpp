#include <iostream>
#include <fstream>
#include <iomanip>
#include <opencv2/opencv.hpp>
#include "rdk_interfaces/msg/object_in_box.hpp"
#include "rdk_interfaces/msg/object.hpp"
#include "openvino/openvino_base.hpp"
#include "inference_engine.hpp"
#include "extension/ext_list.hpp"

using namespace InferenceEngine;

namespace openvino
{
OpenVINOBase::OpenVINOBase(rclcpp::Node & node)
: node_(node),
  is_first_frame_(true),
  num_channels_(0),
  blob_width_(0),
  blob_height_(0),
  cv_width_(0),
  cv_height_(0)
{
}

void OpenVINOBase::init()
{
  std::string engine_name = node_.declare_parameter("engine").get<rclcpp::PARAMETER_STRING>();
  loadEngine(engine_name);

  std::string model_path = node_.declare_parameter("model").get<rclcpp::PARAMETER_STRING>();
  std::string weights_path = node_.declare_parameter("weights").get<rclcpp::PARAMETER_STRING>();
  readNetwork(model_path, weights_path);

  size_t last_index = model_path.find_last_of(".");
  std::string raw_name = model_path.substr(0, last_index);
  std::string label_path = raw_name + ".labels";
  readLabels(label_path);

  prepareInputBlobs();
  prepareOutputBlobs();

  loadNetwork();
  initPublisher();
  initSubscriber();
  registerInferCompletionCallback();
}

void OpenVINOBase::loadEngine(const std::string & engine_name)
{
  plugin_ = PluginDispatcher({PLUGIN_DIRS,""}).getPluginByDevice(engine_name);

  if (engine_name == "CPU")
  {
    plugin_.AddExtension(std::make_shared<InferenceEngine::Extensions::Cpu::CpuExtensions>());
  }
}

void OpenVINOBase::readNetwork(const std::string & model_path,
  const std::string & weights_path)
{
  CNNNetReader network_reader;
  network_reader.ReadNetwork(model_path);
  network_reader.ReadWeights(weights_path);
  //debug info should be removed later
  RCLCPP_DEBUG(node_.get_logger(), "model path: %s", model_path.c_str());
  RCLCPP_DEBUG(node_.get_logger(), "weights path: %s", weights_path.c_str());
  network_ = network_reader.getNetwork();
}

void OpenVINOBase::readLabels(const std::string & label_path)
{
  std::fstream label_file(label_path);
  std::copy(std::istream_iterator<std::string>(label_file),
  std::istream_iterator<std::string>(),
  std::back_inserter(labels_));
}

void OpenVINOBase::loadNetwork()
{
  exec_network_ = plugin_.LoadNetwork(network_, {});
}
}  // namespace openvino
