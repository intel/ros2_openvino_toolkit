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
 * @brief a header file with declaration of Pipeline Manager class
 * @file pipeline_manager.cpp
 */

#include <openvino_param_lib/param_manager.hpp>
#include <memory>
#include <string>
#include <utility>
#include <map>

#if 0
#include "openvino_wrapper_lib/inferences/landmarks_detection.hpp"
#include "openvino_wrapper_lib/inferences/face_reidentification.hpp"
#include "openvino_wrapper_lib/models/face_reidentification_model.hpp"
#include "openvino_wrapper_lib/models/landmarks_detection_model.hpp"
#endif

#include "openvino_wrapper_lib/models/vehicle_attribs_detection_model.hpp"
#include "openvino_wrapper_lib/models/license_plate_detection_model.hpp"
#include "openvino_wrapper_lib/models/person_reidentification_model.hpp"
#include "openvino_wrapper_lib/models/person_attribs_detection_model.hpp"
#include "openvino_wrapper_lib/inferences/vehicle_attribs_detection.hpp"
#include "openvino_wrapper_lib/inferences/license_plate_detection.hpp"
#include "openvino_wrapper_lib/inferences/person_reidentification.hpp"
#include "openvino_wrapper_lib/inferences/person_attribs_detection.hpp"
#include "openvino_wrapper_lib/inferences/face_detection.hpp"
#include "openvino_wrapper_lib/models/face_detection_model.hpp"
#include "openvino_wrapper_lib/inferences/age_gender_detection.hpp"
#include "openvino_wrapper_lib/models/age_gender_detection_model.hpp"
#include "openvino_wrapper_lib/inferences/emotions_detection.hpp"
#include "openvino_wrapper_lib/models/emotion_detection_model.hpp"
#include "openvino_wrapper_lib/inferences/head_pose_detection.hpp"
#include "openvino_wrapper_lib/models/head_pose_detection_model.hpp"
#include "openvino_wrapper_lib/models/object_detection_yolov5_model.hpp"
#include "openvino_wrapper_lib/models/object_detection_ssd_model.hpp"
#include "openvino_wrapper_lib/inferences/object_segmentation.hpp"
#include "openvino_wrapper_lib/models/object_segmentation_model.hpp"
#include "openvino_wrapper_lib/inferences/human_pose_estimation.hpp"
#include "openvino_wrapper_lib/models/human_pose_estimation_model.hpp"
#include "openvino_wrapper_lib/inputs/base_input.hpp"
#include "openvino_wrapper_lib/inputs/image_input.hpp"
#include "openvino_wrapper_lib/inputs/realsense_camera.hpp"
#include "openvino_wrapper_lib/inputs/realsense_camera_topic.hpp"
#include "openvino_wrapper_lib/inputs/standard_camera.hpp"
#include "openvino_wrapper_lib/inputs/ip_camera.hpp"
#include "openvino_wrapper_lib/inputs/video_input.hpp"
#include "openvino_wrapper_lib/outputs/image_window_output.hpp"
#include "openvino_wrapper_lib/outputs/ros_topic_output.hpp"
#include "openvino_wrapper_lib/outputs/rviz_output.hpp"
#include "openvino_wrapper_lib/outputs/ros_service_output.hpp"
#include "openvino_wrapper_lib/pipeline.hpp"
#include "openvino_wrapper_lib/pipeline_manager.hpp"
#include "openvino_wrapper_lib/pipeline_params.hpp"
#include "openvino_wrapper_lib/services/pipeline_processing_server.hpp"
#include "openvino_wrapper_lib/engines/engine_manager.hpp"
std::shared_ptr<Pipeline>
PipelineManager::createPipeline(const Params::ParamManager::PipelineRawData & params,
  rclcpp::Node::SharedPtr node)
{
  if (params.name == "") {
    throw std::logic_error("The name of pipeline won't be empty!");
  }

  std::shared_ptr<Pipeline> pipeline = std::make_shared<Pipeline>(params.name);
  pipeline->getParameters()->update(params);

  PipelineData data;
  data.parent_node = node;
  data.pipeline = pipeline;
  data.params = params;
  data.state = PipelineState_ThreadNotCreated;

  auto inputs = parseInputDevice(data);
  if (inputs.size() != 1) {
    slog::err << "currently one pipeline only supports ONE input." << slog::endl;
    return nullptr;
  }
  for (auto it = inputs.begin(); it != inputs.end(); ++it) {
    pipeline->add(it->first, it->second);
    auto node = it->second->getHandler();
    if (node != nullptr) {
      data.spin_nodes.emplace_back(node);
    }
  }

  auto outputs = parseOutput(data);
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

  pipelines_.insert({params.name, data});

  pipeline->setCallback();
  slog::info << "One Pipeline Created!" << slog::endl;
  pipeline->printPipeline();
  return pipeline;
}

std::map<std::string, std::shared_ptr<Input::BaseInputDevice>>
PipelineManager::parseInputDevice(const PipelineData & pdata)
{
  std::map<std::string, std::shared_ptr<Input::BaseInputDevice>> inputs;
  for (auto & name : pdata.params.inputs) {
    slog::info << "Parsing InputDvice: " << name << slog::endl;
    std::shared_ptr<Input::BaseInputDevice> device = nullptr;
    if (name == kInputType_RealSenseCamera) {
      device = std::make_shared<Input::RealSenseCamera>();
    } else if (name == kInputType_StandardCamera) {
      device = std::make_shared<Input::StandardCamera>();
    } else if (name == kInputType_IpCamera) {
      if (pdata.params.input_meta != "") {
        device = std::make_shared<Input::IpCamera>(pdata.params.input_meta);
      }
    } else if (name == kInputType_CameraTopic || name == kInputType_ImageTopic) {
      device = std::make_shared<Input::RealSenseCameraTopic>(pdata.parent_node);
    } else if (name == kInputType_Video) {
      if (pdata.params.input_meta != "") {
        device = std::make_shared<Input::Video>(pdata.params.input_meta);
      }
    } else if (name == kInputType_Image) {
      if (pdata.params.input_meta != "") {
        device = std::make_shared<Input::Image>(pdata.params.input_meta);
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
PipelineManager::parseOutput(const PipelineData & pdata)
{
  std::map<std::string, std::shared_ptr<Outputs::BaseOutput>> outputs;
  for (auto & name : pdata.params.outputs) {
    slog::info << "Parsing Output: " << name << slog::endl;
    std::shared_ptr<Outputs::BaseOutput> object = nullptr;
    if (name == kOutputTpye_RosTopic) {
      object = std::make_shared<Outputs::RosTopicOutput>(pdata.params.name, pdata.parent_node);
    } else if (name == kOutputTpye_ImageWindow) {
      object = std::make_shared<Outputs::ImageWindowOutput>(pdata.params.name);
    } else if (name == kOutputTpye_RViz) {
      object = std::make_shared<Outputs::RvizOutput>(pdata.params.name, pdata.parent_node);
    } else if (name == kOutputTpye_RosService) {
      object = std::make_shared<Outputs::RosServiceOutput>(pdata.params.name);
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

std::map<std::string, std::shared_ptr<openvino_wrapper_lib::BaseInference>>
PipelineManager::parseInference(const Params::ParamManager::PipelineRawData & params)
{
  std::map<std::string, std::shared_ptr<openvino_wrapper_lib::BaseInference>> inferences;
  for (auto & infer : params.infers) {
    if (infer.name.empty() || infer.model.empty()) {
      continue;
    }
    slog::info << "Parsing Inference: " << infer.name << slog::endl;
    std::shared_ptr<openvino_wrapper_lib::BaseInference> object = nullptr;

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
    } else if (infer.name == kInferTpye_ObjectSegmentationMaskrcnn) {
      object = createObjectSegmentationMaskrcnn(infer);
    } else if (infer.name == kInferTpye_PersonReidentification) {
      object = createPersonReidentification(infer);
    } else if (infer.name == kInferTpye_PersonAttribsDetection) {
      object = createPersonAttribsDetection(infer);
    } /*else if (infer.name == kInferTpye_LandmarksDetection) {
      object = createLandmarksDetection(infer);
    } else if (infer.name == kInferTpye_FaceReidentification) {
      object = createFaceReidentification(infer);
    } */ else if (infer.name == kInferTpye_VehicleAttribsDetection) {
      object = createVehicleAttribsDetection(infer);
    } else if (infer.name == kInferTpye_LicensePlateDetection) {
      object = createLicensePlateDetection(infer);
    } else if (infer.name == kInferTpye_HumanPoseEstimation) {
      object = createHumanPoseEstimation(infer);
    }else {
      slog::err << "Invalid inference name: " << infer.name << slog::endl;
    }

    if (object != nullptr) {
      inferences.insert({infer.name, object});
      slog::info << " ... Adding one Inference: " << infer.name << slog::endl;
    }
  }

  return inferences;
}


std::shared_ptr<openvino_wrapper_lib::BaseInference>
PipelineManager::createFaceDetection(
  const Params::ParamManager::InferenceRawData & infer)
{
  return createObjectDetection(infer);
}

std::shared_ptr<openvino_wrapper_lib::BaseInference>
PipelineManager::createAgeGenderRecognition(const Params::ParamManager::InferenceRawData & param)
{
  auto model = std::make_shared<Models::AgeGenderDetectionModel>(param.label, param.model, param.batch);
  model->modelInit();
  auto engine = engine_manager_.createEngine(param.engine, model);
  auto infer = std::make_shared<openvino_wrapper_lib::AgeGenderDetection>();
  infer->loadNetwork(model);
  infer->loadEngine(engine);

  return infer;
}

std::shared_ptr<openvino_wrapper_lib::BaseInference>
PipelineManager::createEmotionRecognition(const Params::ParamManager::InferenceRawData & param)
{
  auto model = std::make_shared<Models::EmotionDetectionModel>(param.label, param.model, param.batch);
  model->modelInit();
  auto engine = engine_manager_.createEngine(param.engine, model);
  auto infer = std::make_shared<openvino_wrapper_lib::EmotionsDetection>();
  infer->loadNetwork(model);
  infer->loadEngine(engine);

  return infer;
}

std::shared_ptr<openvino_wrapper_lib::BaseInference>
PipelineManager::createHeadPoseEstimation(const Params::ParamManager::InferenceRawData & param)
{
  auto model = std::make_shared<Models::HeadPoseDetectionModel>(param.label, param.model, param.batch);
  model->modelInit();
  auto engine = engine_manager_.createEngine(param.engine, model);
  auto infer = std::make_shared<openvino_wrapper_lib::HeadPoseDetection>();
  infer->loadNetwork(model);
  infer->loadEngine(engine);

  return infer;
}


std::shared_ptr<openvino_wrapper_lib::BaseInference>
PipelineManager::createObjectDetection(
  const Params::ParamManager::InferenceRawData & infer)
{
  std::shared_ptr<Models::ObjectDetectionModel> object_detection_model;
  std::shared_ptr<openvino_wrapper_lib::ObjectDetection> object_inference_ptr;
  slog::debug << "for test in createObjectDetection()" << slog::endl;
  if (infer.model_type == kInferTpye_ObjectDetectionTypeSSD) {
    object_detection_model =
      std::make_shared<Models::ObjectDetectionSSDModel>(infer.label, infer.model, infer.batch);
  }
  if (infer.model_type == kInferTpye_ObjectDetectionTypeYolov5) {
    object_detection_model =
      std::make_shared<Models::ObjectDetectionYolov5Model>(infer.label, infer.model, infer.batch);
  }

  slog::debug << "for test in createObjectDetection(), Created SSDModel" << slog::endl;
  object_inference_ptr = std::make_shared<openvino_wrapper_lib::ObjectDetection>(
    infer.enable_roi_constraint, infer.confidence_threshold);  // To-do theshold configuration
  slog::debug << "for test in createObjectDetection(), before modelInit()" << slog::endl;
  object_detection_model->modelInit();
  auto object_detection_engine = engine_manager_.createEngine(
    infer.engine, object_detection_model);
  slog::debug << "for test in createObjectDetection(), before loadNetwork" << slog::endl;
  object_inference_ptr->loadNetwork(object_detection_model);
  object_inference_ptr->loadEngine(object_detection_engine);
  slog::debug << "for test in createObjectDetection(), OK" << slog::endl;
  return object_inference_ptr;
}

std::shared_ptr<openvino_wrapper_lib::BaseInference>
PipelineManager::createObjectSegmentation(const Params::ParamManager::InferenceRawData & infer)
{
  auto model =
    std::make_shared<Models::ObjectSegmentationModel>(infer.label, infer.model, infer.batch);
  model->modelInit();
  slog::info << "Segmentation model initialized." << slog::endl;
  auto engine = engine_manager_.createEngine(infer.engine, model);
  slog::info << "Segmentation Engine initialized." << slog::endl;
  auto segmentation_inference_ptr = std::make_shared<openvino_wrapper_lib::ObjectSegmentation>(
    infer.confidence_threshold);
    slog::info << "Segmentation Inference instanced." << slog::endl;
  segmentation_inference_ptr->loadNetwork(model);
  segmentation_inference_ptr->loadEngine(engine);

  return segmentation_inference_ptr;
}

std::shared_ptr<openvino_wrapper_lib::BaseInference>
PipelineManager::createObjectSegmentationMaskrcnn(const Params::ParamManager::InferenceRawData & infer)
{
  auto model =
    std::make_shared<Models::ObjectSegmentationMaskrcnnModel>(infer.label, infer.model, infer.batch);
  model->modelInit();
  slog::info << "Segmentation model initialized." << slog::endl;
  auto engine = engine_manager_.createEngine(infer.engine, model);
  slog::info << "Segmentation Engine initialized." << slog::endl;
  auto segmentation_inference_ptr = std::make_shared<openvino_wrapper_lib::ObjectSegmentationMaskrcnn>(
    infer.confidence_threshold);
    slog::info << "Segmentation Inference instanced." << slog::endl;
  segmentation_inference_ptr->loadNetwork(model);
  segmentation_inference_ptr->loadEngine(engine);

  return segmentation_inference_ptr;
}

std::shared_ptr<openvino_wrapper_lib::BaseInference>
PipelineManager::createPersonReidentification(
  const Params::ParamManager::InferenceRawData & infer)
{
  std::shared_ptr<Models::PersonReidentificationModel> person_reidentification_model;
  std::shared_ptr<openvino_wrapper_lib::PersonReidentification> reidentification_inference_ptr;
  slog::debug << "for test in createPersonReidentification()"<<slog::endl;
  person_reidentification_model =
    std::make_shared<Models::PersonReidentificationModel>(infer.label, infer.model, infer.batch);
  person_reidentification_model->modelInit();
  slog::info << "Reidentification model initialized" << slog::endl;
  auto person_reidentification_engine = engine_manager_.createEngine(infer.engine, person_reidentification_model);
  reidentification_inference_ptr =
    std::make_shared<openvino_wrapper_lib::PersonReidentification>(infer.confidence_threshold);
  slog::debug<< "for test in createPersonReidentification(), before loadNetwork"<<slog::endl;
  reidentification_inference_ptr->loadNetwork(person_reidentification_model);
  reidentification_inference_ptr->loadEngine(person_reidentification_engine);
  slog::debug<< "for test in createPersonReidentification(), OK"<<slog::endl;

  return reidentification_inference_ptr;
}

std::shared_ptr<openvino_wrapper_lib::BaseInference>
PipelineManager::createVehicleAttribsDetection(
  const Params::ParamManager::InferenceRawData & infer)
{
  auto model =
    std::make_shared<Models::VehicleAttribsDetectionModel>(infer.label, infer.model, infer.batch);
  model->modelInit();
  auto engine = engine_manager_.createEngine(infer.engine, model);
  auto vehicle_attribs_ptr =
    std::make_shared<openvino_wrapper_lib::VehicleAttribsDetection>();
  vehicle_attribs_ptr->loadNetwork(model);
  vehicle_attribs_ptr->loadEngine(engine);

  return vehicle_attribs_ptr;
}

std::shared_ptr<openvino_wrapper_lib::BaseInference>
PipelineManager::createLicensePlateDetection(
  const Params::ParamManager::InferenceRawData & infer)
{
  auto model =
    std::make_shared<Models::LicensePlateDetectionModel>(infer.label, infer.model, infer.batch);
  model->modelInit();
  auto engine = engine_manager_.createEngine(infer.engine, model);
  auto license_plate_ptr =
    std::make_shared<openvino_wrapper_lib::LicensePlateDetection>();
  license_plate_ptr->loadNetwork(model);
  license_plate_ptr->loadEngine(engine);

  return license_plate_ptr;
}

std::shared_ptr<openvino_wrapper_lib::BaseInference>
PipelineManager::createPersonAttribsDetection(
  const Params::ParamManager::InferenceRawData & infer)
{
  auto model =
    std::make_shared<Models::PersonAttribsDetectionModel>(infer.label, infer.model, infer.batch);
  slog::debug << "for test in createPersonAttributesDetection()"<<slog::endl;
  model->modelInit();
  auto engine = engine_manager_.createEngine(infer.engine, model);
  auto attribs_inference_ptr =
    std::make_shared<openvino_wrapper_lib::PersonAttribsDetection>(infer.confidence_threshold);
  attribs_inference_ptr->loadNetwork(model);
  attribs_inference_ptr->loadEngine(engine);

  return attribs_inference_ptr;
}

std::shared_ptr<openvino_wrapper_lib::BaseInference>
PipelineManager::createHumanPoseEstimation(
  const Params::ParamManager::InferenceRawData & infer)
{
  auto model =
    std::make_shared<Models::HumanPoseEstimationModel>(infer.label, infer.model, infer.batch);
  model->modelInit();
  auto engine = engine_manager_.createEngine(infer.engine, model);
  auto human_pose_ptr =
    std::make_shared<openvino_wrapper_lib::HumanPoseEstimation>();
  human_pose_ptr->loadNetwork(model);
  human_pose_ptr->loadEngine(engine);

  return human_pose_ptr;
}


#if 0
std::shared_ptr<openvino_wrapper_lib::BaseInference>
PipelineManager::createPersonReidentification(
  const Params::ParamManager::InferenceRawData & infer)
{
  auto model =
    std::make_shared<Models::PersonReidentificationModel>(infer.model, infer.batch);
  model->modelInit();
  auto engine = engine_manager_.createEngine(infer.engine, model);
  auto reidentification_inference_ptr =
    std::make_shared<openvino_wrapper_lib::PersonReidentification>(infer.confidence_threshold);
  reidentification_inference_ptr->loadNetwork(model);
  reidentification_inference_ptr->loadEngine(engine);

  return reidentification_inference_ptr;
}

std::shared_ptr<openvino_wrapper_lib::BaseInference>
PipelineManager::createPersonAttribsDetection(
  const Params::ParamManager::InferenceRawData & infer)
{
  auto model =
    std::make_shared<Models::PersonAttribsDetectionModel>(infer.model, infer.batch);
  
  model->modelInit();
  auto engine = engine_manager_.createEngine(infer.engine, model);
  auto attribs_inference_ptr =
    std::make_shared<openvino_wrapper_lib::PersonAttribsDetection>(infer.confidence_threshold);
  attribs_inference_ptr->loadNetwork(model);
  attribs_inference_ptr->loadEngine(engine);

  return attribs_inference_ptr;
}

std::shared_ptr<openvino_wrapper_lib::BaseInference>
PipelineManager::createLandmarksDetection(
  const Params::ParamManager::InferenceRawData & infer)
{
  auto model =
    std::make_shared<Models::LandmarksDetectionModel>(infer.model, infer.batch);
  model->modelInit();
  auto engine = engine_manager_.createEngine(infer.engine, model);
  auto landmarks_inference_ptr =
    std::make_shared<openvino_wrapper_lib::LandmarksDetection>();
  landmarks_inference_ptr->loadNetwork(model);
  landmarks_inference_ptr->loadEngine(engine);

  return landmarks_inference_ptr;
}

std::shared_ptr<openvino_wrapper_lib::BaseInference>
PipelineManager::createFaceReidentification(
  const Params::ParamManager::InferenceRawData & infer)
{
  auto model =
    std::make_shared<Models::FaceReidentificationModel>(infer.model, infer.batch);
  model->modelInit();
  auto engine = engine_manager_.createEngine(infer.engine, model);
  auto face_reid_ptr =
    std::make_shared<openvino_wrapper_lib::FaceReidentification>(infer.confidence_threshold);
  face_reid_ptr->loadNetwork(model);
  face_reid_ptr->loadEngine(engine);

  return face_reid_ptr;
}

std::shared_ptr<openvino_wrapper_lib::BaseInference>
PipelineManager::createVehicleAttribsDetection(
  const Params::ParamManager::InferenceRawData & infer)
{
  auto model =
    std::make_shared<Models::VehicleAttribsDetectionModel>(infer.model, infer.batch);
  model->modelInit();
  auto engine = engine_manager_.createEngine(infer.engine, model);
  auto vehicle_attribs_ptr =
    std::make_shared<openvino_wrapper_lib::VehicleAttribsDetection>();
  vehicle_attribs_ptr->loadNetwork(model);
  vehicle_attribs_ptr->loadEngine(engine);

  return vehicle_attribs_ptr;
}

std::shared_ptr<openvino_wrapper_lib::BaseInference>
PipelineManager::createLicensePlateDetection(
  const Params::ParamManager::InferenceRawData & infer)
{
  auto model =
    std::make_shared<Models::LicensePlateDetectionModel>(infer.model, infer.batch);
  model->modelInit();
  auto engine = engine_manager_.createEngine(infer.engine, model);
  auto license_plate_ptr =
    std::make_shared<openvino_wrapper_lib::LicensePlateDetection>();
  license_plate_ptr->loadNetwork(model);
  license_plate_ptr->loadEngine(engine);

  return license_plate_ptr;
}
#endif

void PipelineManager::threadPipeline(const char * name)
{
  PipelineData & p = pipelines_[name];
  while (p.state != PipelineState_ThreadStopped && p.pipeline != nullptr) {
    if (p.state == PipelineState_ThreadRunning) {
      p.pipeline->runOnce();
    }
    std::this_thread::sleep_for(std::chrono::microseconds(1));
  }
}
void PipelineManager::threadSpinNodes(const char * name)
{
  PipelineData & p = pipelines_[name];
  while (p.state != PipelineState_ThreadStopped && p.pipeline != nullptr) {
    for (auto & node : p.spin_nodes) {
      rclcpp::spin_some(node);
    }
    std::this_thread::sleep_for(std::chrono::microseconds(1));
  }
}

void PipelineManager::runAll()
{
  for (auto it = pipelines_.begin(); it != pipelines_.end(); ++it) {
    if (it->second.state != PipelineState_ThreadRunning) {
      it->second.state = PipelineState_ThreadRunning;
    }
    if (service_.state != PipelineState_ThreadRunning) {
      service_.state = PipelineState_ThreadRunning;
    }
    if (it->second.thread == nullptr) {
      it->second.thread = std::make_shared<std::thread>(&PipelineManager::threadPipeline, this,
          it->second.params.name.c_str());
    }
    #if 0 //DEBUGING
    // Consider of saving CPU loads, the spin thread is moved out from pipeline manager,
    // which is supposed to be handled by the upper-level applications.
    // (see @file pipeline_with_params.cpp for the calling sample.)
    if (service_.thread == nullptr) {
      service_.thread = std::make_shared<std::thread>(&PipelineManager::runService, this);
    }
    if (it->second.spin_nodes.size() > 0 && it->second.thread_spin_nodes == nullptr) {
      it->second.thread_spin_nodes = std::make_shared<std::thread>(
        &PipelineManager::threadSpinNodes, this,
        it->second.params.name.c_str());
    }
    #endif
  }
}

void PipelineManager::runService()
{
  auto node = std::make_shared<vino_service::PipelineProcessingServer
      <openvino_msgs::srv::PipelineSrv>>("pipeline_service");
  while (service_.state != PipelineState_ThreadStopped && service_.thread != nullptr) {
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
}

void PipelineManager::stopAll()
{
  for (auto it = pipelines_.begin(); it != pipelines_.end(); ++it) {
    if (it->second.state == PipelineState_ThreadRunning) {
      it->second.state = PipelineState_ThreadStopped;
    }
  }
  if (service_.state == PipelineState_ThreadRunning) {
    service_.state = PipelineState_ThreadStopped;
  }
}

void PipelineManager::joinAll()
{
  if (service_.thread != nullptr && service_.state == PipelineState_ThreadRunning) {
    service_.thread->join();  // pipeline service
  }

  for (auto it = pipelines_.begin(); it != pipelines_.end(); ++it) {
    if (it->second.thread != nullptr && it->second.state == PipelineState_ThreadRunning) {
      it->second.thread->join();
    }
    if (it->second.thread_spin_nodes != nullptr &&
      it->second.state == PipelineState_ThreadRunning)
    {
      it->second.thread_spin_nodes->join();
    }
  }
}
