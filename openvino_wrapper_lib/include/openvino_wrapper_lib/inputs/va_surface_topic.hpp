// va_surface_topic.hpp — Input device backed by VaSurfaceFrame topic.
//
// Design
// ======
// VaSurfaceTopic extends BaseInputDevice with a second read path:
//   read(cv::Mat*)         — legacy fallback (mmap + NV12→BGR conversion)
//   readVaSurface(Holder*) — zero-copy intra-process path
//
// The Pipeline (or VaPipeline) checks isVaSurface() to choose the path.
//
// Type negotiation
// ================
// VaSurfaceTopic subscribes via the VaSurfaceHolder TypeAdapter so that
// intra-process delivery skips serialisation entirely.  It also publishes
// an InferRequirements message once the model dimensions are known so the
// camera pipeline can reconfigure its VEBOX/SFC scaler.

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <mutex>
#include <string>

#include "openvino_wrapper_lib/inputs/base_input.hpp"
#include "openvino_wrapper_lib/inputs/va_surface_holder.hpp"
#include "icamera_usm/msg/va_surface_frame.hpp"
#include "icamera_usm/msg/infer_requirements.hpp"

namespace Input
{

class VaSurfaceTopic : public BaseInputDevice
{
public:
  /// @param node          ROS node to create the subscription on.
  /// @param topic         VaSurfaceFrame topic, e.g. /icamera/camera0/color/va/frame
  /// @param req_topic     InferRequirements topic to publish on.
  explicit VaSurfaceTopic(rclcpp::Node::SharedPtr node,
                          const std::string& topic,
                          const std::string& req_topic =
                              "/icamera/infer_requirements");

  // BaseInputDevice interface ---------------------------------------------------
  bool initialize() override;
  bool initialize(size_t width, size_t height) override;

  /// Legacy path: block until a frame is available; convert to BGR cv::Mat.
  /// Uses mmap on the DMA-BUF fd for inter-process delivery.
  /// Intra-process: calls cv::cvtColor from the held cpu_mat (set in cb()).
  bool read(cv::Mat* frame) override;

  // VA-surface extension --------------------------------------------------------
  /// Non-blocking.  Fills @p holder and returns true when a fresh VA-surface
  /// frame is available.  Returns false when the queue is empty.
  bool readVaSurface(openvino_wrapper_lib::VaSurfaceHolder* holder);

  /// True if this input can deliver VA surfaces (always true for this class).
  static constexpr bool isVaSurface() noexcept { return true; }

  /// Publish an InferRequirements message so the camera pipeline knows the
  /// model's expected input size and format.
  /// Call once after the model is compiled.
  void publishRequirements(uint32_t width, uint32_t height,
                           const std::string& color_format = "NV12");

private:
  using AdaptedSub = rclcpp::Subscription<
      rclcpp::TypeAdapter<openvino_wrapper_lib::VaSurfaceHolder,
                          icamera_usm::msg::VaSurfaceFrame>>;

  void cb(openvino_wrapper_lib::VaSurfaceHolder holder);

  rclcpp::Node::SharedPtr node_;
  std::string             topic_;
  std::string             req_topic_;

  AdaptedSub::SharedPtr   sub_;
  rclcpp::Publisher<icamera_usm::msg::InferRequirements>::SharedPtr req_pub_;

  mutable std::mutex                          mu_;
  openvino_wrapper_lib::VaSurfaceHolder       latest_;
  bool                                        has_frame_ = false;
};

}  // namespace Input
