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
#include "openvino_wrapper_lib/models/object_segmentation_yolo26_model.hpp"
#include "openvino_wrapper_lib/models/yolo26_model.hpp"

Models::ObjectSegmentationYolo26Model::ObjectSegmentationYolo26Model(const std::string& label_loc,
                                                                     const std::string& model_loc, int max_batch_size)
  : ObjectSegmentationModel(label_loc, model_loc, max_batch_size)
{
  setKeepInputShapeRatio(true);
}

const std::string Models::ObjectSegmentationYolo26Model::getModelCategory() const
{
  return "Object Segmentation Yolo26";
}

bool Models::ObjectSegmentationYolo26Model::updateLayerProperty(std::shared_ptr<ov::Model>& model)
{
  BaseModel::updateLayerProperty(model);
  addYoloPreprocessing(model, getInputName());

  const ov::Shape input_shape = model->input().get_shape();
  setInputHeight(input_shape[1]);
  setInputWidth(input_shape[2]);

  const ov::Shape output_shape = model->output().get_shape();
  if (output_shape.size() != 3) {
    throw std::logic_error("YOLO26 semantic segmentation needs a class map output [1, H, W], but the output of " +
                           getModelName() + " is " + output_shape.to_string() + ".");
  }
  addOutputInfo("detection", getOutputName());
  return true;
}

bool Models::ObjectSegmentationYolo26Model::enqueue(const std::shared_ptr<Engines::Engine>& engine,
                                                    const cv::Mat& frame, const cv::Rect& input_frame_loc)
{
  setFrameSize(frame.cols, frame.rows);
  return BaseModel::matToBlob(frame, input_frame_loc, 1, 0, engine);
}
