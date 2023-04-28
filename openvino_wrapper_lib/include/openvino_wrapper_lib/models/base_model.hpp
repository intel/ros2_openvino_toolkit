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

/**
 * @brief A header file with declaration for BaseModel Class
 * @file base_model.h
 */

#ifndef OPENVINO_WRAPPER_LIB__MODELS__BASE_MODEL_HPP_
#define OPENVINO_WRAPPER_LIB__MODELS__BASE_MODEL_HPP_

#include <opencv2/opencv.hpp>

#include <vector>
#include <memory>
#include <string>
#include <map>
#include <iostream>
#include <fstream>

#include <openvino/openvino.hpp>
#include "openvino_wrapper_lib/slog.hpp"
#include "openvino_wrapper_lib/models/attributes/base_attribute.hpp"

namespace Engines
{
  class Engine;
}

namespace openvino_wrapper_lib
{
  class ObjectDetectionResult;
}

namespace Models
{
  /**
   * @class BaseModel
   * @brief This class represents the network given by .xml and .bin file
   */
  class BaseModel : public ModelAttribute
  {
  public:
    using Ptr = std::shared_ptr<BaseModel>;
  /**
   * @brief Initialize the class with given .xml, .bin and .labels file. It will
   * also check whether the number of input and output are fit.
   * @param[in] model_loc The location of model' s .xml file
   * (model' s bin file should be the same as .xml file except for extension)
   * @param[in] input_num The number of input the network should have.
   * @param[in] output_num The number of output the network should have.
   * @param[in] batch_size The number of batch size (default: 1) the network should have.
   * @return Whether the input device is successfully turned on.
   */
    BaseModel(const std::string& label_loc, const std::string& model_loc, int batch_size = 1);

  /**
   * @brief Get the maximum batch size of the model.
   * @return The maximum batch size of the model.
   */
    inline int getMaxBatchSize() const
    {
      return max_batch_size_;
    }
    inline void setMaxBatchSize(int max_batch_size)
    {
      max_batch_size_ = max_batch_size;
    }

    virtual bool enqueue(
        const std::shared_ptr<Engines::Engine> &engine,
        const cv::Mat &frame,
        const cv::Rect &input_frame_loc) { return true; }
  /**
   * @brief Initialize the model. During the process the class will check
   * the network input, output size, check layer property and
   * set layer property.
   */
    void modelInit();
  /**
   * @brief Get the name of the model.
   * @return The name of the model.
   */
    virtual const std::string getModelCategory() const = 0;
    inline ModelAttr getAttribute() { return attr_; }

    inline std::shared_ptr<ov::Model> getModel() const
    {
      return model_;
    }

  protected:
    /**
     * New infterface to check and update Layer Property
     * @brief Set the layer property (layer layout, layer precision, etc.).
     * @param[in] network_reader The reader of the network to be set.
     */
    virtual bool updateLayerProperty(std::shared_ptr<ov::Model>& network_reader) = 0;
    ov::Core engine;
    std::shared_ptr<ov::Model> model_; 
    void setFrameSize(const int &w, const int &h)
    {
      frame_size_.width = w;
      frame_size_.height = h;
    }
    cv::Size getFrameSize()
    {
      return frame_size_;
    }

  private:
    int max_batch_size_;
    std::string model_loc_;
    std::string label_loc_;
    cv::Size frame_size_;
  };

  class ObjectDetectionModel : public BaseModel
  {
  public:
    ObjectDetectionModel(const std::string& label_loc, const std::string& model_loc, int batch_size = 1);
    virtual bool fetchResults(
        const std::shared_ptr<Engines::Engine> &engine,
        std::vector<openvino_wrapper_lib::ObjectDetectionResult> &result,
        const float &confidence_thresh = 0.3,
        const bool &enable_roi_constraint = false) = 0;
    virtual bool matToBlob(
        const cv::Mat &orig_image, const cv::Rect &, float scale_factor,
        int batch_index, const std::shared_ptr<Engines::Engine> &engine) = 0;

    cv::Mat resizeImage(const cv::Mat &orig_image, const int resized_width, const int resized_height,
                        double fx = 0, double fy = 0, int interpolation = cv::INTER_LINEAR,
                        int border_dt = 0, int border_db = 0, int border_dl = 0, int border_dr = 0,
                        cv::Scalar border_color = cv::Scalar(0, 0, 0));

    void dataToBlob(cv::Mat& resize_img, float scale_factor, int batch_index,
                const std::shared_ptr<Engines::Engine> & engine, bool mat_steps = true);
  };

} // namespace Models

#endif // OPENVINO_WRAPPER_LIB__MODELS__BASE_MODEL_HPP_
