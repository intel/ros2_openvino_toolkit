// Copyright (c) 2026 Intel Corporation
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

#include <string>
#include <memory>
#include "openvino_wrapper_lib/models/yolo26_model.hpp"

void Models::addYoloPreprocessing(std::shared_ptr<ov::Model>& model, const std::string& input_name)
{
  ov::preprocess::PrePostProcessor ppp(model);
  ov::preprocess::InputInfo& input_info = ppp.input(input_name);
  input_info.tensor()
      .set_element_type(ov::element::u8)
      .set_layout("NHWC")
      .set_color_format(ov::preprocess::ColorFormat::BGR);
  input_info.preprocess()
      .convert_element_type(ov::element::f32)
      .convert_color(ov::preprocess::ColorFormat::RGB)
      .scale(255.f);
  input_info.model().set_layout("NCHW");
  for (size_t i = 0; i < model->outputs().size(); i++) {
    ppp.output(i).tensor().set_element_type(ov::element::f32);
  }
  model = ppp.build();
}

Models::Yolo26Model::Yolo26Model(const Params::ParamManager::InferenceRawData& config) : BaseModel(config)
{
  setKeepInputShapeRatio(true);
}

bool Models::Yolo26Model::updateLayerProperty(std::shared_ptr<ov::Model>& model)
{
  BaseModel::updateLayerProperty(model);
  addYoloPreprocessing(model, getInputName());

  const ov::Shape input_shape = model->input().get_shape();
  setInputHeight(input_shape[1]);
  setInputWidth(input_shape[2]);
  checkOutputShape(model->output().get_shape());
  return true;
}

bool Models::Yolo26Model::enqueue(const std::shared_ptr<Engines::Engine>& engine, const cv::Mat& frame,
                                  const cv::Rect& input_frame_loc)
{
  setFrameSize(frame.cols, frame.rows);
  return matToBlob(frame, input_frame_loc, 1, 0, engine);
}
