// Copyright (c) 2023 Intel Corporation
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
 * @brief a realization file with declaration of HumanPoseEstimationModel class
 * @file human_pose_estimation_model.cpp
 */
#include <string>
#include "openvino_wrapper_lib/models/human_pose_estimation_model.hpp"
#include "openvino_wrapper_lib/slog.hpp"

// Validated Human Pose Estimation Model Network
Models::HumanPoseEstimationModel::HumanPoseEstimationModel(
  const std::string & label_loc, const std::string & model_loc, int max_batch_size)
: BaseModel(label_loc, model_loc, max_batch_size) {}

bool Models::HumanPoseEstimationModel::updateLayerProperty(
  std::shared_ptr<ov::Model>& model)
{
  slog::info << "Checking INPUTs for model " << getModelName() << slog::endl;
  auto input_info_map = model->inputs();
  if (input_info_map.size() != 1) {
    throw std::logic_error("HPE OpenPose model wrapper supports topologies with only 1 input");
  }

  auto output_info_map = model->outputs();
  if (output_info_map.size() != 2) {
    throw std::logic_error("HPE OpenPose supports topologies with only 2 outputs");
  }

  // ------- Prepare Input  ------------------
  ov::preprocess::PrePostProcessor ppp(model);
  input_tensor_name_ = model->input().get_any_name();
  const ov::Layout inputLayout{"NHWC"};
  ppp.input().tensor().set_element_type(ov::element::u8).set_layout({"NHWC"});
  ppp.input().model().set_layout(inputLayout);

  // ------- Prepare output  ------------------
  const ov::Layout outputLayout("NCHW");
  for (const auto& output : model->outputs()) {
    const auto& outTensorName = output.get_any_name();
      ppp.output(outTensorName).tensor().
        set_element_type(ov::element::f32)
        .set_layout(outputLayout);
      outputsNames.push_back(outTensorName);
  }
  model = ppp.build();

  const size_t batchId = ov::layout::batch_idx(outputLayout);
  const size_t channelsId = ov::layout::channels_idx(outputLayout);
  const size_t widthId = ov::layout::width_idx(outputLayout);
  const size_t heightId = ov::layout::height_idx(outputLayout); 
  ov::Shape heatmapsOutputShape = model->outputs().front().get_shape();
  ov::Shape pafsOutputShape = model->outputs().back().get_shape();
  if (heatmapsOutputShape[channelsId] > pafsOutputShape[channelsId]) {
      std::swap(heatmapsOutputShape, pafsOutputShape);
      std::swap(outputsNames[0], outputsNames[1]);
  }

#if 0
  //////change input size
    ov::Shape inputShape = model->input().get_shape();
    const ov::Layout& layout = ov::layout::get_layout(model->inputs().front());
    // const auto batchId = ov::layout::batch_idx(layout);
    // const auto heightId = ov::layout::height_idx(layout);
    // const auto widthId = ov::layout::width_idx(layout);

    //if (!targetSize) {
    auto targetSize = inputShape[heightId];
    //}
    int stride = 8;
    auto f_size = getFrameSize();
    aspectRatio = 1.77778;
    slog::err<<"testsetsetsetswith: "<< f_size.width<< ", ghit: "<<f_size.height<<slog::endl;
    //aspectRatio = f_size.width / static_cast<double>(f_size.height);
    int height = static_cast<int>((targetSize + stride - 1) / stride) * stride;
    int inputWidth = static_cast<int>(std::round(targetSize * aspectRatio));
    int width = static_cast<int>((inputWidth + stride - 1) / stride) * stride;
    inputShape[batchId] = 1;
    inputShape[heightId] = height;
    inputShape[widthId] = width;
    inputLayerSize = cv::Size(width, height);
    model->reshape(inputShape);
  //////change input size end
#endif

  addInputInfo("input", input_tensor_name_);
  addOutputInfo("pafsOutput", output_info_map[0].get_any_name());
  addOutputInfo("heatmaps", output_info_map[1].get_any_name());


  printAttribute();
  return true;
}

const std::string Models::HumanPoseEstimationModel::getModelCategory() const
{
  return "Human Pose Estimation";
}
