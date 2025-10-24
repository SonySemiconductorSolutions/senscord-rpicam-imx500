/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2024, Raspberry Pi Ltd
 *
 * imx500_object_detection.cpp - IMX500 object detection for various networks
 */

#include <libcamera/control_ids.h>
#include <libcamera/geometry.h>

#include <algorithm>
#include <array>
#include <mutex>
#include <string>
#include <vector>

#include "core/logging.hpp"
#include "core/rpicam_app.hpp"
#include "imx500_post_processing_stage.hpp"
#include "post_processing_stages/object_detect.hpp"
#include "post_processing_stages/post_processing_stage.hpp"

using Rectangle    = libcamera::Rectangle;
using Size         = libcamera::Size;
namespace controls = libcamera::controls;

#define NAME "imx500_no_process"

class NoProcess : public IMX500PostProcessingStage {
 public:
  NoProcess(RPiCamApp *app) : IMX500PostProcessingStage(app) {}

  char const *Name() const override;

  void Read(boost::property_tree::ptree const &params) override;

  void Configure() override;

  void Stop() override;

  bool Process(CompletedRequestPtr &completed_request) override;

 private:
  bool network_loaderd_      = false;
  bool inference_started_    = false;
  bool network_load_started_ = false;
};

char const *NoProcess::Name() const { return NAME; }

void NoProcess::Read(boost::property_tree::ptree const &params) {
  if (!network_loaderd_) {
    IMX500PostProcessingStage::Read(params);
    network_loaderd_ = true;
  }
}

void NoProcess::Configure() {
  IMX500PostProcessingStage::Configure();
  if (!network_load_started_) {
    IMX500PostProcessingStage::ShowFwProgressBar();
    network_load_started_ = true;
  }
}

void NoProcess::Stop() {
  inference_started_    = false;
  network_loaderd_      = false;
  network_load_started_ = false;
}

bool NoProcess::Process(CompletedRequestPtr &completed_request) {
  auto scaler_crop = completed_request->metadata.get(controls::ScalerCrop);
  if (!raw_stream_ || !scaler_crop) {
    LOG_ERROR(
        "Must have RAW stream and scaler crop available to get sensor "
        "dimensions!");
    return false;
  }
  if (!inference_started_) {
    auto output =
        completed_request->metadata.get(controls::rpi::CnnOutputTensor);
    auto info =
        completed_request->metadata.get(controls::rpi::CnnOutputTensorInfo);
    if (!output || !info) {
      return false;
    }
    inference_started_ = true;
  }

  return IMX500PostProcessingStage::Process(completed_request);
}

static PostProcessingStage *Create(RPiCamApp *app) {
  return new NoProcess(app);
}

static RegisterStage reg(NAME, &Create);
