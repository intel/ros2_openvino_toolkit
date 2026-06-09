// va_surface_holder.hpp — TypeAdapter bridging VaSurfaceFrame ↔ cv::Mat.
//
// ROS 2 type negotiation path
// ===========================
// When icamera_usm and ros2_openvino_toolkit run in the SAME composable
// container (intra-process), the publisher and subscriber share memory.
// The TypeAdapter converts VaSurfaceFrame → VaSurfaceHolder without any
// copy or serialisation.  VaSurfaceHolder carries the VASurfaceID so the
// inference engine can call VAContext::create_tensor_nv12() directly.
//
// When running inter-process (separate nodes), the TypeAdapter falls back
// to an mmap of the DMA-BUF fd to produce a cpu-side cv::Mat — the same
// path the old Python node used.
//
// Usage (inference-side node):
//
//   using VaTopicAdapt =
//       rclcpp::TypeAdapter<VaSurfaceHolder,
//                           icamera_usm::msg::VaSurfaceFrame>;
//   RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(
//       VaSurfaceHolder, icamera_usm::msg::VaSurfaceFrame);
//
//   node->create_subscription<VaTopicAdapt>(topic, qos, cb);
//
// The callback receives VaSurfaceHolder directly; no deserialisation when
// intra-process.

#pragma once

#include <rclcpp/type_adapter.hpp>
#include <opencv2/opencv.hpp>
#include <cstdint>
#include <string>

// Forward-declare to avoid pulling in va.h everywhere.
typedef unsigned int VASurfaceID;
typedef void*        VADisplay;

#include "icamera_usm/msg/va_surface_frame.hpp"

namespace openvino_wrapper_lib
{

/// Lightweight frame descriptor shared between the camera pipeline and the
/// inference pipeline over an intra-process publication.
struct VaSurfaceHolder
{
  // ── Intra-process (valid when va_surface_id != VA_INVALID_SURFACE) ──
  VASurfaceID  va_surface_id  = 0xFFFFFFFFu;  // VA_INVALID_SURFACE
  VADisplay    va_display      = nullptr;       // shared process-global display
  uint32_t     width           = 0;
  uint32_t     height          = 0;            // Y-plane height

  // ── Fallback: cpu-side mat (valid when va_surface_id == VA_INVALID_SURFACE) ─
  cv::Mat      cpu_mat;                        // BGR or NV12 mapped from mmap

  // ── Common metadata ──
  uint64_t     token           = 0;
  uint64_t     frame_seq       = 0;
  builtin_interfaces::msg::Time hw_stamp;
  std_msgs::msg::Header         header;

  /// True when the caller should use the VA surface path.
  bool hasVaSurface() const noexcept
  {
    return va_surface_id != 0xFFFFFFFFu && va_display != nullptr;
  }
};

}  // namespace openvino_wrapper_lib

// ── ROS 2 TypeAdapter specialisation ────────────────────────────────────────
// Intra-process: convert_to_custom copies metadata only (zero-copy surface).
// Inter-process: convert_to_ros_message serialises to VaSurfaceFrame as-is.
// The DMA-BUF fd is NOT carried across process boundaries by this adapter;
// inter-process callers must use the separate AF-UNIX socket transport and
// will receive a VaSurfaceHolder with va_surface_id = VA_INVALID_SURFACE and
// a valid cpu_mat (mmap'd in VaSurfaceTopic::cb()).

template <>
struct rclcpp::TypeAdapter<openvino_wrapper_lib::VaSurfaceHolder,
                           icamera_usm::msg::VaSurfaceFrame>
{
  using is_specialized    = std::true_type;
  using custom_type       = openvino_wrapper_lib::VaSurfaceHolder;
  using ros_message_type  = icamera_usm::msg::VaSurfaceFrame;

  /// ROS → custom: called when receiving a message (intra-process fast-path).
  /// When the message was published intra-process the VaSurfaceFrame pointer is
  /// the same allocation — no copy.  When deserialised from the wire the
  /// va_surface_id comes from the message field (set by VaSurfacePublisher).
  static void convert_to_custom(const ros_message_type& src, custom_type& dst)
  {
    dst.va_surface_id = src.va_surface_id;
    dst.va_display    = nullptr;     // caller fills in from VaDisplayHolder::get()
    dst.width         = src.width;
    dst.height        = src.height;
    dst.token         = src.token;
    dst.frame_seq     = src.frame_seq;
    dst.hw_stamp      = src.hw_stamp;
    dst.header        = src.header;
    // cpu_mat stays empty; VaSurfaceTopic::cb() fills it for inter-process use.
  }

  /// custom → ROS: called when the TypeAdapter publisher serialises.
  /// We only need to round-trip the metadata fields.
  static void convert_to_ros_message(const custom_type& src, ros_message_type& dst)
  {
    dst.va_surface_id = src.va_surface_id;
    dst.width         = src.width;
    dst.height        = src.height;
    dst.token         = src.token;
    dst.frame_seq     = src.frame_seq;
    dst.hw_stamp      = src.hw_stamp;
    dst.header        = src.header;
  }
};
