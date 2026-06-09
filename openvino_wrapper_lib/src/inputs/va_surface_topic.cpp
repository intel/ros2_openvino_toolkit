// va_surface_topic.cpp — VaSurfaceTopic implementation.

#include "openvino_wrapper_lib/inputs/va_surface_topic.hpp"
#include "openvino_wrapper_lib/slog.hpp"

// VaDisplayHolder gives the process-global VADisplay so we can fill
// VaSurfaceHolder::va_display for intra-process delivery.
#ifdef ICAMERA_USM_HAS_LIBVA
#include "va_display_holder.hpp"
#endif

namespace Input
{

VaSurfaceTopic::VaSurfaceTopic(rclcpp::Node::SharedPtr node,
                               const std::string& topic,
                               const std::string& req_topic)
  : node_(node), topic_(topic), req_topic_(req_topic)
{
}

bool VaSurfaceTopic::initialize()
{
  if (!node_) {
    slog::err << "VaSurfaceTopic: no parent node" << slog::endl;
    return false;
  }

  using VaAdapter = rclcpp::TypeAdapter<openvino_wrapper_lib::VaSurfaceHolder,
                                        icamera_usm::msg::VaSurfaceFrame>;

  auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();
  sub_ = node_->create_subscription<VaAdapter>(
      topic_, qos,
      [this](openvino_wrapper_lib::VaSurfaceHolder holder) { cb(std::move(holder)); });

  // InferRequirements publisher — RELIABLE + TRANSIENT_LOCAL so the camera
  // node receives the message even if it started first.
  req_pub_ = node_->create_publisher<icamera_usm::msg::InferRequirements>(
      req_topic_,
      rclcpp::QoS(1).reliable().transient_local());

  slog::info << "VaSurfaceTopic: subscribed to " << topic_
             << ", requirements on " << req_topic_ << slog::endl;
  setInitStatus(true);
  return true;
}

bool VaSurfaceTopic::initialize(size_t width, size_t height)
{
  setWidth(width);
  setHeight(height);
  return initialize();
}

void VaSurfaceTopic::cb(openvino_wrapper_lib::VaSurfaceHolder holder)
{
  // Fill in the process-global VADisplay so the inference engine can call
  // VAContext::create_tensor_nv12() without needing to pass the display
  // through the message.
#ifdef ICAMERA_USM_HAS_LIBVA
  if (holder.va_surface_id != 0xFFFFFFFFu) {
    holder.va_display = icamera_usm::VaDisplayHolder::get();
  }
#endif

  std::lock_guard<std::mutex> lk(mu_);
  latest_    = std::move(holder);
  has_frame_ = true;
}

bool VaSurfaceTopic::readVaSurface(openvino_wrapper_lib::VaSurfaceHolder* holder)
{
  std::lock_guard<std::mutex> lk(mu_);
  if (!has_frame_) return false;
  *holder    = latest_;
  has_frame_ = false;
  return true;
}

bool VaSurfaceTopic::read(cv::Mat* frame)
{
  openvino_wrapper_lib::VaSurfaceHolder holder;
  {
    std::lock_guard<std::mutex> lk(mu_);
    if (!has_frame_) return false;
    holder     = latest_;
    has_frame_ = false;
  }

  if (!holder.cpu_mat.empty()) {
    *frame = holder.cpu_mat;
    return true;
  }

  // No cpu_mat: this is an intra-process frame with only a VASurfaceID.
  // Callers that end up here should use readVaSurface() instead.
  slog::warn << "VaSurfaceTopic::read(cv::Mat*): no CPU image; use readVaSurface()"
             << slog::endl;
  return false;
}

void VaSurfaceTopic::publishRequirements(uint32_t width, uint32_t height,
                                         const std::string& color_format)
{
  if (!req_pub_) return;
  icamera_usm::msg::InferRequirements msg;
  msg.header.stamp  = node_->get_clock()->now();
  msg.width         = width;
  msg.height        = height;
  msg.color_format  = color_format;
  msg.ready         = true;
  req_pub_->publish(msg);
  slog::info << "VaSurfaceTopic: published InferRequirements "
             << width << "x" << height << " " << color_format << slog::endl;
}

}  // namespace Input
