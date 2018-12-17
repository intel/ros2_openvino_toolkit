/*
 * Copyright (c) 2018 Intel Corporation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**
 * @brief a header file with declaration of Pipeline Manager class
 * @file pipeline_manager.cpp
 */

#include <memory>
#include <string>
#include <utility>

#include <vino_param_lib/param_manager.hpp>
#include "dynamic_vino_lib/factory.hpp"
#include "dynamic_vino_lib/inferences/age_gender_detection.hpp"
#include "dynamic_vino_lib/inferences/emotions_detection.hpp"
#include "dynamic_vino_lib/inferences/face_detection.hpp"
#include "dynamic_vino_lib/inferences/head_pose_detection.hpp"
#include "dynamic_vino_lib/inferences/object_segmentation.hpp"
#include "dynamic_vino_lib/inputs/image_input.hpp"
#include "dynamic_vino_lib/inputs/realsense_camera.hpp"
#include "dynamic_vino_lib/inputs/realsense_camera_topic.hpp"
#include "dynamic_vino_lib/inputs/standard_camera.hpp"
#include "dynamic_vino_lib/inputs/video_input.hpp"
#include "dynamic_vino_lib/models/age_gender_detection_model.hpp"
#include "dynamic_vino_lib/models/emotion_detection_model.hpp"
#include "dynamic_vino_lib/models/face_detection_model.hpp"
#include "dynamic_vino_lib/models/head_pose_detection_model.hpp"
#include "dynamic_vino_lib/models/object_segmentation_model.hpp"
#include "dynamic_vino_lib/outputs/image_window_output.hpp"
#include "dynamic_vino_lib/outputs/ros_topic_output.hpp"
#include "dynamic_vino_lib/outputs/rviz_output.hpp"
#include "dynamic_vino_lib/pipeline.hpp"
#include "dynamic_vino_lib/pipeline_manager.hpp"
#include "dynamic_vino_lib/pipeline_params.hpp"

std::shared_ptr<Pipeline> PipelineManager::createPipeline(
    const Params::ParamManager::PipelineParams& params) {
  if (params.name == "") {
    throw std::logic_error("The name of pipeline won't be empty!");
  }
  PipelineData data;

  std::shared_ptr<Pipeline> pipeline = std::make_shared<Pipeline>(params.name);
  pipeline->getParameters()->update(params);

  auto inputs = parseInputDevice(params);
  if (inputs.size() != 1) {
    slog::err << "currently one pipeline only supports ONE input."
              << slog::endl;
    return nullptr;
  }
  for (auto it = inputs.begin(); it != inputs.end(); ++it) {
    pipeline->add(it->first, it->second);
    auto node = it->second->getHandler();
    if (node != nullptr) {
      data.spin_nodes.emplace_back(node);
    }
  }

  auto outputs = parseOutput(params);
  for (auto it = outputs.begin(); it != outputs.end(); ++it) {
    pipeline->add(it->first, it->second);
  }

  auto infers = parseInference(params);
  for (auto it = infers.begin(); it != infers.end(); ++it) {
    pipeline->add(it->first, it->second);
  }

  slog::info << "Updating connections ..." << slog::endl;
  for (auto it = params.connects.begin(); it != params.connects.end(); ++it) {
    pipeline->add(it->first, it->second);
  }

  data.pipeline = pipeline;
  data.params = params;
  data.state = PipelineState_ThreadNotCreated;
  pipelines_.insert({params.name, data});

  pipeline->setCallback();
  slog::info << "One Pipeline Created!" << slog::endl;
  pipeline->printPipeline();
  return pipeline;
}

std::map<std::string, std::shared_ptr<Input::BaseInputDevice>>
PipelineManager::parseInputDevice(
    const Params::ParamManager::PipelineParams& params) {
  std::map<std::string, std::shared_ptr<Input::BaseInputDevice>> inputs;
  for (auto& name : params.inputs) {
    slog::info << "Parsing InputDvice: " << name << slog::endl;
    std::shared_ptr<Input::BaseInputDevice> device = nullptr;
    if (name == kInputType_RealSenseCamera) {
      device = std::make_shared<Input::RealSenseCamera>();
    } else if (name == kInputType_StandardCamera) {
      device = std::make_shared<Input::StandardCamera>();
    } else if (name == kInputType_CameraTopic) {
      device = std::make_shared<Input::RealSenseCameraTopic>();
    } else if (name == kInputType_Video) {
      if (params.input_meta != "") {
        device = std::make_shared<Input::Video>(params.input_meta);
      }
    } else if (name == kInputType_Image) {
      if (params.input_meta != "") {
        device = std::make_shared<Input::Image>(params.input_meta);
      }
    } else {
      slog::err << "Invalid input device name: " << name << slog::endl;
    }

    if (device != nullptr) {
      device->initialize();
      inputs.insert({name, device});
      slog::info << " ... Adding one Input device: " << name << slog::endl;
    }
  }

  return inputs;
}

std::map<std::string, std::shared_ptr<Outputs::BaseOutput>>
PipelineManager::parseOutput(
    const Params::ParamManager::PipelineParams& params) {
  std::map<std::string, std::shared_ptr<Outputs::BaseOutput>> outputs;
  for (auto& name : params.outputs) {
    slog::info << "Parsing Output: " << name << slog::endl;
    std::shared_ptr<Outputs::BaseOutput> object = nullptr;
    if (name == kOutputTpye_RosTopic) {
      object = std::make_shared<Outputs::RosTopicOutput>();
    } else if (name == kOutputTpye_ImageWindow) {
      object = std::make_shared<Outputs::ImageWindowOutput>("Results");
    } else if (name == kOutputTpye_RViz) {
      object = std::make_shared<Outputs::RvizOutput>();
    } else {
      slog::err << "Invalid output name: " << name << slog::endl;
    }
    if (object != nullptr) {
      outputs.insert({name, object});
      slog::info << " ... Adding one Output: " << name << slog::endl;
    }
  }

  return outputs;
}

std::map<std::string, std::shared_ptr<dynamic_vino_lib::BaseInference>>
PipelineManager::parseInference(
    const Params::ParamManager::PipelineParams& params) {
  /**< update plugins for devices >**/
  auto pcommon = Params::ParamManager::getInstance().getCommon();
  std::string FLAGS_l = pcommon.custom_cpu_library;
  std::string FLAGS_c = pcommon.custom_cldnn_library;
  bool FLAGS_pc = pcommon.enable_performance_count;

  std::map<std::string, std::shared_ptr<dynamic_vino_lib::BaseInference>>
      inferences;
  for (auto& infer : params.infers) {
    if (infer.name.empty() || infer.model.empty()) {
      continue;
    }
    slog::info << "Parsing Inference: " << infer.name << slog::endl;
    std::shared_ptr<dynamic_vino_lib::BaseInference> object = nullptr;
    if (plugins_for_devices_.find(infer.engine) == plugins_for_devices_.end()) {
      plugins_for_devices_[infer.engine] =
          *Factory::makePluginByName(infer.engine, FLAGS_l, FLAGS_c, FLAGS_pc);
    }

    if (infer.name == kInferTpye_FaceDetection) {
      object = createFaceDetection(infer);

    } else if (infer.name == kInferTpye_AgeGenderRecognition) {
      object = createAgeGenderRecognition(infer);

    } else if (infer.name == kInferTpye_EmotionRecognition) {
      object = createEmotionRecognition(infer);

    } else if (infer.name == kInferTpye_HeadPoseEstimation) {
      object = createHeadPoseEstimation(infer);

    } else if (infer.name == kInferTpye_ObjectDetection) {
      object = createObjectDetection(infer);

    } else if (infer.name == kInferTpye_ObjectSegmentation) {
      object = createObjectSegmentation(infer);

    } else {
      slog::err << "Invalid inference name: " << infer.name << slog::endl;
    }

    if (object != nullptr) {
      inferences.insert({infer.name, object});
      slog::info << " ... Adding one Inference: " << infer.name << slog::endl;
    }
  }

  return inferences;
}

std::shared_ptr<dynamic_vino_lib::BaseInference>
PipelineManager::createFaceDetection(
    const Params::ParamManager::InferenceParams& infer) {
  // TODO: add batch size in param_manager
  auto face_detection_model =
      std::make_shared<Models::FaceDetectionModel>(infer.model, 1, 1, 1);
  face_detection_model->modelInit();
  auto face_detection_engine = std::make_shared<Engines::Engine>(
      plugins_for_devices_[infer.engine], face_detection_model);
  auto face_inference_ptr = std::make_shared<dynamic_vino_lib::FaceDetection>(
      0.5);  // TODO: add output_threshold in param_manager
  face_inference_ptr->loadNetwork(face_detection_model);
  face_inference_ptr->loadEngine(face_detection_engine);

  return face_inference_ptr;
}

std::shared_ptr<dynamic_vino_lib::BaseInference>
PipelineManager::createAgeGenderRecognition(
    const Params::ParamManager::InferenceParams& param) {
  auto model =
      std::make_shared<Models::AgeGenderDetectionModel>(param.model, 1, 2, 16);
  model->modelInit();
  auto engine = std::make_shared<Engines::Engine>(
      plugins_for_devices_[param.engine], model);
  auto infer = std::make_shared<dynamic_vino_lib::AgeGenderDetection>();
  infer->loadNetwork(model);
  infer->loadEngine(engine);

  return infer;
}

std::shared_ptr<dynamic_vino_lib::BaseInference>
PipelineManager::createEmotionRecognition(
    const Params::ParamManager::InferenceParams& param) {
  auto model =
      std::make_shared<Models::EmotionDetectionModel>(param.model, 1, 1, 16);
  model->modelInit();
  auto engine = std::make_shared<Engines::Engine>(
      plugins_for_devices_[param.engine], model);
  auto infer = std::make_shared<dynamic_vino_lib::EmotionsDetection>();
  infer->loadNetwork(model);
  infer->loadEngine(engine);

  return infer;
}

std::shared_ptr<dynamic_vino_lib::BaseInference>
PipelineManager::createHeadPoseEstimation(
    const Params::ParamManager::InferenceParams& param) {
  auto model =
      std::make_shared<Models::HeadPoseDetectionModel>(param.model, 1, 3, 16);
  model->modelInit();
  auto engine = std::make_shared<Engines::Engine>(
      plugins_for_devices_[param.engine], model);
  auto infer = std::make_shared<dynamic_vino_lib::HeadPoseDetection>();
  infer->loadNetwork(model);
  infer->loadEngine(engine);

  return infer;
}

std::shared_ptr<dynamic_vino_lib::BaseInference>
PipelineManager::createObjectDetection(
    const Params::ParamManager::InferenceParams& infer) {
  // TODO: not implemented yet

  return createFaceDetection(infer);
}

std::shared_ptr<dynamic_vino_lib::BaseInference>
PipelineManager::createObjectSegmentation(
    const Params::ParamManager::InferenceParams& infer) {
  auto obejct_segmentation_model =
      std::make_shared<Models::ObjectSegmentationModel>(infer.model, 1, 1, 1);
  obejct_segmentation_model->modelInit();
  auto obejct_segmentation_engine = std::make_shared<Engines::Engine>(
      plugins_for_devices_[infer.engine], obejct_segmentation_model);
  auto segmentation_inference_ptr = 
    std::make_shared<dynamic_vino_lib::ObjectSegmentation>(0.5);
  segmentation_inference_ptr->loadNetwork(obejct_segmentation_model);
  segmentation_inference_ptr->loadEngine(obejct_segmentation_engine);

  return segmentation_inference_ptr;
}

void PipelineManager::threadPipeline(const char* name) {
  PipelineData& p = pipelines_[name];
  while (p.state == PipelineState_ThreadRunning && p.pipeline != nullptr) {
    for (auto& node : p.spin_nodes) {
      rclcpp::spin_some(node);
    }
    p.pipeline->runOnce();
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
}

void PipelineManager::runAll() {
  for (auto it = pipelines_.begin(); it != pipelines_.end(); ++it) {
    if (it->second.state != PipelineState_ThreadRunning) {
      it->second.state = PipelineState_ThreadRunning;
    }
    if (it->second.thread == nullptr) {
      it->second.thread =
          std::make_shared<std::thread>(&PipelineManager::threadPipeline, this,
                                        it->second.params.name.c_str());
    }
  }
}

void PipelineManager::stopAll() {
  for (auto it = pipelines_.begin(); it != pipelines_.end(); ++it) {
    if (it->second.state == PipelineState_ThreadRunning) {
      it->second.state = PipelineState_ThreadStopped;
    }
  }
}

void PipelineManager::joinAll() {
  for (auto it = pipelines_.begin(); it != pipelines_.end(); ++it) {
    if (it->second.thread != nullptr &&
        it->second.state == PipelineState_ThreadRunning) {
      it->second.thread->join();
    }
  }
}
