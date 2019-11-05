#ifndef OPENVINO__OBJECT_DETECTION_YOLOV2_HPP_
#define OPENVINO__OBJECT_DETECTION_YOLOV2_HPP_

#include "rdk_interfaces/msg/object.hpp"
#include "rdk_interfaces/msg/object_in_box.hpp"
#include "rdk_interfaces/msg/objects_in_boxes.hpp"
#include "inference_engine.hpp"
#include "openvino_base.hpp"

namespace openvino
{
class ObjectDetectionYOLOV2 : public OpenVINOBase
{
public:
  ObjectDetectionYOLOV2(rclcpp::Node & node);
  virtual ~ObjectDetectionYOLOV2() = default;
  void initSubscriber() override;
  void initPublisher() override;
  void prepareInputBlobs() override;
  void prepareOutputBlobs() override;
  template <typename T>
  void process(const T msg);
  void registerInferCompletionCallback() override;

private:
  std::string input_name_;
  std::string output_name_;
  int max_proposal_count_;
  int object_size_;
  rclcpp::Publisher<rdk_interfaces::msg::ObjectsInBoxes>::SharedPtr pub_;
  static double intersectionOverUnion(const cv::Rect & box_1, const cv::Rect & box_2);
  int getEntryIndex(int side, int lcoords, int lclasses, int location, int entry);
  rdk_interfaces::msg::ObjectsInBoxes objs_;

  static bool sortByProbility(rdk_interfaces::msg::ObjectInBox begin,
    rdk_interfaces::msg::ObjectInBox end)
  {
    return begin.object.probability < end.object.probability;
  }

  int imw_;
  int imh_;
  int srcw_;
  int srch_;
};
}  // namespace openvino

#endif  // OPENVINO__OBJECT_DETECTION_YOLOV2_HPP_
