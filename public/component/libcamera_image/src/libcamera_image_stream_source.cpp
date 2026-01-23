/*
 * SPDX-FileCopyrightText: 2023 Sony Semiconductor Solutions Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "libcamera_image_stream_source.h"

#include <fcntl.h>
#include <libcamera/control_ids.h>
#include <libcamera/controls.h>
#include <linux/videodev2.h>
#include <sys/ioctl.h>

#include <chrono>
#include <filesystem>
#include <iomanip>
#include <sstream>
#include <string>
#include <thread>

#include "rpicam_app_adapter.h"
#include "senscord/inference_stream/inference_stream_types.h"
#include "senscord/logger.h"
#include "senscord/status.h"
#include "v4l2_ctrl_manager.h"

static constexpr const char *kBlockName = "libcamera";

namespace fs = std::filesystem;

// Default frame rate (30fps)
static constexpr uint32_t kDefaultFrameRateNum   = 3000;
static constexpr uint32_t kDefaultFrameRateDenom = 100;

// Default AI Model Bundle ID for RPI
static constexpr const char *kDefaultAIModelBundleId = "999997";

#ifndef ARGUMENT_NULL_CHECK
#define ARGUMENT_NULL_CHECK(arg, cause)                                  \
  do {                                                                   \
    if (!arg) {                                                          \
      return SENSCORD_STATUS_FAIL(kBlockName, Status::kCause##cause,     \
                                  "Arg.L%d " #arg " is null", __LINE__); \
    }                                                                    \
  } while (0)
#endif

namespace senscord {
namespace libcamera_image {

// Helper function to safely copy string to fixed-size buffer with NUL
// termination
static inline void SafeStringCopy(char *dest, size_t dest_size,
                                  const std::string &src) {
  if (dest_size > 0) {
    snprintf(dest, dest_size, "%s", src.c_str());
  }
}

static inline void SafeStringCopy(char *dest, size_t dest_size,
                                  const char *src) {
  if (dest_size > 0) {
    snprintf(dest, dest_size, "%s", src);
  }
}

// kWaitOnGetFrames will be referenced from GetFrames()
// Expected maximum framerate is 120fps, which is 8ms
// the half of it is sufficient to avoid both of busy loop and other performance
// problem
const uint64_t LibcameraImageStreamSource::kWaitOnGetFrames = 4'000'000;

LibcameraImageStreamSource::LibcameraImageStreamSource()
    : properties_initialized_(false),
      imx500_device_id_(""),
      camera_exposure_mode_{KCameraExposureModeAuto},
      camera_anti_flicker_mode_{kCameraAntiFlickerModeOff},
      camera_manual_exposure_{10000, 0.0f},
      camera_auto_exposure_metering_{
          kCameraAutoExposureMeteringModeFullScreen,
          {0, 0, CAMERA_IMAGE_HEIGHT_DEFAULT, CAMERA_IMAGE_WIDTH_DEFAULT}},
      camera_image_flip_{false, false},
      camera_image_size_{CAMERA_IMAGE_WIDTH_DEFAULT,
                         CAMERA_IMAGE_HEIGHT_DEFAULT,
                         kCameraScalingPolicySensitivity},
      camera_frame_rate_{kDefaultFrameRateNum, kDefaultFrameRateDenom},
      image_crop_{0, 0, 0, 0},
      cached_image_crop_{0, 0, 0, 0},
      isp_frame_rate_{kDefaultFrameRateNum, kDefaultFrameRateDenom} {
  // RPI only supports AI model "999997"
  SafeStringCopy(ai_model_bundle_id_.ai_model_bundle_id,
                 sizeof(ai_model_bundle_id_.ai_model_bundle_id),
                 kDefaultAIModelBundleId);
}

LibcameraImageStreamSource::~LibcameraImageStreamSource() {
  // Do nothing.
}

senscord::Status LibcameraImageStreamSource::Open(
    senscord::Core *core, senscord::StreamSourceUtility *util) {
  SENSCORD_LOG_DEBUG_TAGGED("libcamera", "LibcameraImageStreamSource::Open()");
  core_ = core;
  util_ = util;

  // RPI only supports AI model "999997" - reset to default
  SafeStringCopy(ai_model_bundle_id_.ai_model_bundle_id,
                 sizeof(ai_model_bundle_id_.ai_model_bundle_id),
                 kDefaultAIModelBundleId);

  // register optional properties
  SENSCORD_REGISTER_PROPERTY(util_,
                             senscord::kImageSensorFunctionSupportedPropertyKey,
                             senscord::ImageSensorFunctionSupportedProperty);
  SENSCORD_REGISTER_PROPERTY(
      util_, senscord::libcamera_image::kLibcameraAccessPropertyKey,
      senscord::libcamera_image::AccessProperty);
  SENSCORD_REGISTER_PROPERTY(
      util_, senscord::libcamera_image::kLibcameraDeviceEnumerationPropertyKey,
      senscord::libcamera_image::DeviceEnumerationProperty);
  SENSCORD_REGISTER_PROPERTY(util_, senscord::kImageCropPropertyKey,
                             senscord::ImageCropProperty);
  SENSCORD_REGISTER_PROPERTY(
      util_, senscord::libcamera_image::kLibcameraImageRotationPropertyKey,
      senscord::libcamera_image::ImageRotationProperty);
  SENSCORD_REGISTER_PROPERTY(
      util_, senscord::libcamera_image::kLibcameraCameraImageFlipPropertyKey,
      senscord::libcamera_image::CameraImageFlipProperty);
  SENSCORD_REGISTER_PROPERTY(
      util_, senscord::libcamera_image::kLibcameraAIModelBundleIdPropertyKey,
      senscord::libcamera_image::AIModelBundleIdProperty);
  SENSCORD_REGISTER_PROPERTY(
      util_, senscord::libcamera_image::kLibcameraInputDataTypePropertyKey,
      senscord::libcamera_image::InputDataTypeProperty);
  SENSCORD_REGISTER_PROPERTY(
      util_, senscord::libcamera_image::kLibcameraInfoStringPropertyKey,
      senscord::libcamera_image::InfoStringProperty);
  SENSCORD_REGISTER_PROPERTY(
      util_, senscord::libcamera_image::kPostProcessAvailablePropertyKey,
      senscord::libcamera_image::PostProcessAvailableProperty);
  SENSCORD_REGISTER_PROPERTY(
      util_, senscord::libcamera_image::kLibcameraCameraExposureModePropertyKey,
      senscord::libcamera_image::CameraExposureModeProperty);
  SENSCORD_REGISTER_PROPERTY(
      util_, senscord::libcamera_image::kLibcameraCameraAutoExposurePropertyKey,
      senscord::libcamera_image::CameraAutoExposureProperty);
  SENSCORD_REGISTER_PROPERTY(
      util_,
      senscord::libcamera_image::kLibcameraCameraEvCompensationPropertyKey,
      senscord::libcamera_image::CameraEvCompensationProperty);
  SENSCORD_REGISTER_PROPERTY(
      util_,
      senscord::libcamera_image::kLibcameraCameraAntiFlickerModePropertyKey,
      senscord::libcamera_image::CameraAntiFlickerModeProperty);
  SENSCORD_REGISTER_PROPERTY(
      util_,
      senscord::libcamera_image::
          kLibcameraCameraAutoExposureMeteringPropertykey,
      senscord::libcamera_image::CameraAutoExposureMeteringProperty);
  SENSCORD_REGISTER_PROPERTY(
      util_,
      senscord::libcamera_image::kLibcameraCameraManualExposurePropertykey,
      senscord::libcamera_image::CameraManualExposureProperty);
  SENSCORD_REGISTER_PROPERTY(
      util_, senscord::libcamera_image::kLibcameraTemperaturePropertyKey,
      senscord::libcamera_image::CameraTemperatureProperty);
  SENSCORD_REGISTER_PROPERTY(
      util_, senscord::libcamera_image::kLibcameraCameraImageSizePropertykey,
      senscord::libcamera_image::CameraImageSizeProperty);
  SENSCORD_REGISTER_PROPERTY(
      util_, senscord::libcamera_image::kLibcameraCameraFrameRatePropertykey,
      senscord::libcamera_image::CameraFrameRateProperty);
  SENSCORD_REGISTER_PROPERTY(
      util_, senscord::libcamera_image::kLibcameraIspImagePropertyKey,
      senscord::libcamera_image::IspImageProperty);
  SENSCORD_REGISTER_PROPERTY(
      util_, senscord::libcamera_image::kLibcameraIspFrameRatePropertyKey,
      senscord::libcamera_image::IspFrameRateProperty);
  SENSCORD_REGISTER_PROPERTY(util_, senscord::kStreamStatePropertyKey,
                             senscord::StreamStateProperty);
  SENSCORD_REGISTER_PROPERTY(util_, senscord::kInferencePropertyKey,
                             senscord::InferenceProperty);
  SENSCORD_REGISTER_PROPERTY(util_, senscord::kAIModelIndexPropertyKey,
                             senscord::AIModelIndexProperty);

  std::string device       = "";
  uint64_t uint_value      = 0;
  std::string string_value = "";
  senscord::Status status  = senscord::Status::OK();

  // parse arguments
  {
    status = util_->GetStreamArgument("device", &device);
    if (!status.ok()) {
      device = "";
    }
  }

  // parse arguments: ImageProperty
  {
    image_property_.width        = 640;
    image_property_.height       = 480;
    image_property_.pixel_format = senscord::kPixelFormatBGR24;
    // Initialize stride_bytes to default value for BGR24 (3 bytes per pixel).
    // This will be recalculated by libcamera_adapter::Configure() based on
    // actual hardware alignment requirements.
    image_property_.stride_bytes = image_property_.width * 3;

    {
      status = util_->GetStreamArgument("pixel_format", &string_value);
      if (status.ok()) {
        image_property_.pixel_format = string_value;
      }
    }
    util_->UpdateChannelProperty(AITRIOS_SENSOR_CHANNEL_ID_INFERENCE_RAW_IMAGE,
                                 senscord::kImagePropertyKey, &image_property_);
    util_->SendEventPropertyUpdated(senscord::kImagePropertyKey);
  }

  // parse arguments: FrameRateProperty
  {
    senscord::FrameRateProperty framerate_property = {};
    framerate_property.num                         = kDefaultFrameRateNum;
    framerate_property.denom                       = kDefaultFrameRateDenom;

    Set(senscord::kFrameRatePropertyKey, &framerate_property);
  }

  {
    display_channel_ = AITRIOS_SENSOR_CHANNEL_ID_INFERENCE_INPUT_IMAGE;
    status           = util_->GetStreamArgument("display_channel", &uint_value);
    if (!status.ok()) {
      display_channel_ = static_cast<uint32_t>(uint_value);
    }
  }

  // dump aurguments
  // device_name
  SENSCORD_LOG_DEBUG_TAGGED("libcamera", "device: '%s'", device.c_str());
  // image_property_
  SENSCORD_LOG_DEBUG_TAGGED("libcamera", "image_property_: %s %ux%u(%u)",
                            image_property_.pixel_format.c_str(),
                            image_property_.width, image_property_.height,
                            image_property_.stride_bytes);
  // framerate_property_
  SENSCORD_LOG_DEBUG_TAGGED("libcamera", "framerate_property_: %u/%u",
                            framerate_property_.num, framerate_property_.denom);
  SENSCORD_LOG_INFO("display_channel: '%d'", display_channel_);
  status = adapter_.Open(device, util_, image_property_);
  if (!status.ok()) {
    util_->SendEventError(status);
  }

  // Set Input Tensor channel info
  senscord::ImageProperty inference_image_property;
  inference_image_property.width        = image_property_.width;
  inference_image_property.height       = image_property_.height;
  inference_image_property.pixel_format = "image_rgb24";

  util_->UpdateChannelProperty(AITRIOS_SENSOR_CHANNEL_ID_INFERENCE_INPUT_IMAGE,
                               senscord::kImagePropertyKey,
                               &inference_image_property);
  util_->UpdateChannelProperty(AITRIOS_SENSOR_CHANNEL_ID_INFERENCE_RAW_IMAGE,
                               senscord::kImagePropertyKey, &image_property_);

  // Initialize image_crop_ with default values (full image size)
  image_crop_.left   = 0;
  image_crop_.top    = 0;
  image_crop_.width  = camera_image_size_.width;
  image_crop_.height = camera_image_size_.height;
  // Initialize cache to invalid state to force first update
  cached_image_crop_.left   = 0;
  cached_image_crop_.top    = 0;
  cached_image_crop_.width  = 0;
  cached_image_crop_.height = 0;

  // get device_id
  if (imx500_device_id_.empty()) {
    if (!GetDeviceID()) {
      return SENSCORD_STATUS_FAIL("libcamera",
                                  senscord::Status::kCauseInvalidArgument,
                                  "Open() Failed to get device_id");
    }

    SENSCORD_LOG_INFO("Get device_id: %s", imx500_device_id_.c_str());
  }

  return status;
}

senscord::Status LibcameraImageStreamSource::Close() {
  SENSCORD_LOG_DEBUG_TAGGED("libcamera", "LibcameraImageStreamSource::Close()");
  senscord::Status status;

  std::unique_lock<std::mutex> _lck(device_id_mutex_);

  status = adapter_.Close();
  if (!status.ok()) {
    util_->SendEventError(status);
  }

  return status;
}

senscord::Status LibcameraImageStreamSource::Start() {
  SENSCORD_LOG_DEBUG_TAGGED("libcamera", "LibcameraImageStreamSource::Start()");
  senscord::Status status;

  std::unique_lock<std::mutex> _lck(device_id_mutex_);

  // Reset properties initialized flag
  properties_initialized_ = false;
  // Reset property cache to ensure properties are updated on first frame
  // after stream_start
  cached_image_crop_.left   = 0;
  cached_image_crop_.top    = 0;
  cached_image_crop_.width  = 0;
  cached_image_crop_.height = 0;

  // Apply initial frame rate property first, before image property.
  // The frame rate must be set before Configure() is called (which happens
  // inside Set(ImageProperty)), because Configure() depends on the frame rate
  // information to properly set up the libcamera pipeline.
  status = Set(senscord::kFrameRatePropertyKey, &framerate_property_);
  if (!status.ok()) {
    util_->SendEventError(status);
    return status;
  }

  // Apply initial frame rate property first, before image property.
  // Get current image property from adapter first to ensure we have the latest
  // values.
  adapter_.GetImageProperty(&image_property_,
                            AITRIOS_SENSOR_CHANNEL_ID_INFERENCE_RAW_IMAGE);
  status = Set(senscord::kImagePropertyKey, &image_property_);
  if (!status.ok()) {
    util_->SendEventError(status);
    return status;
  }

  status = adapter_.Start();
  if (!status.ok()) {
    util_->SendEventError(status);
  }

  return status;
}

senscord::Status LibcameraImageStreamSource::Stop() {
  SENSCORD_LOG_DEBUG_TAGGED("libcamera", "LibcameraImageStreamSource::Stop()");
  senscord::Status status;

  std::unique_lock<std::mutex> _lck(device_id_mutex_);

  status = adapter_.Stop();
  if (!status.ok()) {
    util_->SendEventError(status);
  }

  return status;
}

void LibcameraImageStreamSource::GetFrames(
    std::vector<senscord::FrameInfo> *frames) {
  // SENSCORD_LOG_DEBUG_TAGGED("libcamera",
  // "LibcameraImageStreamSource::GetFrames()");

  std::unique_lock<std::mutex> _lck(device_id_mutex_);

  adapter_.GetFrames(frames);
  if (frames->empty()) {
    // Sleep to avoid busy loop
    senscord::osal::OSSleep(kWaitOnGetFrames);
  } else {
    // Initialize channel properties on first frame after stream start.
    // These properties are constant during streaming and only need to be set
    // once.
    if (!properties_initialized_) {
      // AIModelBundleIdProperty - set for inference input image channel (0)
      util_->UpdateChannelProperty(
          senscord::kChannelIdImage(0),
          senscord::libcamera_image::kLibcameraAIModelBundleIdPropertyKey,
          &ai_model_bundle_id_);

      // AIModelIndexProperty - set for inference input image channel (0)
      senscord::AIModelIndexProperty ai_model_index{};
      ai_model_index.ai_model_index = 0;
      util_->UpdateChannelProperty(senscord::kChannelIdImage(0),
                                   senscord::kAIModelIndexPropertyKey,
                                   &ai_model_index);

      // InferenceProperty - set data type for inference output channel (0)
      senscord::InferenceProperty inference{};
      SafeStringCopy(inference.data_type, sizeof(inference.data_type),
                     kInferenceDataFormatTensor32Float);
      util_->UpdateChannelProperty(senscord::kChannelIdImage(0),
                                   senscord::kInferencePropertyKey, &inference);

      // SubFrameProperty - set for both inference channels (0: input, 1: raw)
      senscord::SubFrameProperty sub_frame{};
      sub_frame.current_num  = 1;
      sub_frame.division_num = 1;
      util_->UpdateChannelProperty(senscord::kChannelIdImage(0),
                                   senscord::kSubFramePropertyKey, &sub_frame);
      util_->UpdateChannelProperty(senscord::kChannelIdImage(1),
                                   senscord::kSubFramePropertyKey, &sub_frame);

      // TensorValidProperty - indicate tensors are valid when present
      senscord::TensorValidProperty tensor_valid{};
      tensor_valid.valid = true;
      util_->UpdateChannelProperty(senscord::kChannelIdImage(0),
                                   senscord::kTensorValidPropertyKey,
                                   &tensor_valid);
      util_->UpdateChannelProperty(senscord::kChannelIdImage(1),
                                   senscord::kTensorValidPropertyKey,
                                   &tensor_valid);
      properties_initialized_ = true;
    }

    // Update ImageCropProperty only if changed
    if (image_crop_.left != cached_image_crop_.left ||
        image_crop_.top != cached_image_crop_.top ||
        image_crop_.width != cached_image_crop_.width ||
        image_crop_.height != cached_image_crop_.height) {
      util_->UpdateChannelProperty(senscord::kChannelIdImage(0),
                                   senscord::kImageCropPropertyKey,
                                   &image_crop_);
      cached_image_crop_ = image_crop_;
    }
  }
}

senscord::Status LibcameraImageStreamSource::ReleaseFrame(
    const senscord::FrameInfo &frameinfo,
    const std::vector<uint32_t> *referenced_channel_ids) {
  adapter_.ReleaseFrame(frameinfo, referenced_channel_ids);

  return senscord::Status::OK();
}

senscord::Status LibcameraImageStreamSource::Get(
    const std::string &key, senscord::ChannelInfoProperty *property) {
  ARGUMENT_NULL_CHECK(property, InvalidArgument);
  property->channels.clear();
  SENSCORD_LOG_DEBUG_TAGGED(
      "libcamera", "LibcameraImageStreamSource::Get(ChannelInfoProperty)");

  // TODO: multi-plane support
  senscord::ChannelInfo info0 = {};
  info0.raw_data_type         = "inference_data";
  info0.description           = "Inference data";

  property->channels.insert(
      std::make_pair(AITRIOS_SENSOR_CHANNEL_ID_INFERENCE_OUTPUT, info0));
  senscord::ChannelInfo info1 = {};
  info1.raw_data_type         = senscord::kRawDataTypeImage;
  info1.description           = "Input image data for AI Model";

  property->channels.insert(
      std::make_pair(AITRIOS_SENSOR_CHANNEL_ID_INFERENCE_INPUT_IMAGE, info1));

  senscord::ChannelInfo info2 = {};
  info2.raw_data_type         = senscord::kRawDataTypeImage;
  info2.description           = "Camera image data";

  property->channels.insert(
      std::make_pair(AITRIOS_SENSOR_CHANNEL_ID_INFERENCE_RAW_IMAGE, info2));

  return senscord::Status::OK();
}

senscord::Status LibcameraImageStreamSource::Get(
    const std::string &key, senscord::FrameRateProperty *property) {
  ARGUMENT_NULL_CHECK(property, InvalidArgument);
  SENSCORD_LOG_DEBUG_TAGGED(
      "libcamera", "LibcameraImageStreamSource::Get(FrameRateProperty)");
  *property = framerate_property_;
  return senscord::Status::OK();
}

senscord::Status LibcameraImageStreamSource::Set(
    const std::string &key, const senscord::FrameRateProperty *property) {
  ARGUMENT_NULL_CHECK(property, InvalidArgument);
  SENSCORD_LOG_DEBUG_TAGGED(
      "libcamera", "LibcameraImageStreamSource::Set(FrameRateProperty)");
  framerate_property_ = *property;

  int64_t frame_time =
      1000000 / framerate_property_.num / framerate_property_.denom;
  std::vector<int64_t> frame_duration = {frame_time, frame_time};

  libcamera_image::AccessProperty access_property = {};
  access_property.type = libcamera_image::AccessProperty::Type::kControl;
  access_property.id   = "FrameDurationLimits";
  access_property.value.Set(frame_duration);

  senscord::Status status = Set(kLibcameraAccessPropertyKey, &access_property);
  if (!status.ok()) {
    return status;
  }

  // notify framerate_property_ is updated
  util_->UpdateChannelProperty(senscord::kChannelIdImage(0),
                               senscord::kFrameRatePropertyKey,
                               &framerate_property_);
  util_->SendEventPropertyUpdated(senscord::kFrameRatePropertyKey);

  return senscord::Status::OK();
}

senscord::Status LibcameraImageStreamSource::Get(
    const std::string &key, senscord::ImageProperty *property) {
  ARGUMENT_NULL_CHECK(property, InvalidArgument);
  SENSCORD_LOG_DEBUG_TAGGED("libcamera",
                            "LibcameraImageStreamSource::Get(ImageProperty)");
  adapter_.GetImageProperty(property, display_channel_);
  return senscord::Status::OK();
}

senscord::Status LibcameraImageStreamSource::Set(
    const std::string &key, const senscord::ImageProperty *property) {
  ARGUMENT_NULL_CHECK(property, InvalidArgument);
  SENSCORD_LOG_DEBUG_TAGGED("libcamera",
                            "LibcameraImageStreamSource::Set(ImageProperty)");

  image_property_         = *property;
  senscord::Status status = adapter_.Configure(image_property_);
  if (!status.ok()) {
    util_->SendEventError(status);
    return status;
  }

  // image property could be changed by Configure()
  util_->SendEventPropertyUpdated(senscord::kImagePropertyKey);
  util_->UpdateChannelProperty(senscord::kChannelIdImage(0),
                               senscord::kImagePropertyKey, &image_property_);

  return senscord::Status::OK();
}

senscord::Status LibcameraImageStreamSource::Get(
    const std::string &key,
    senscord::ImageSensorFunctionSupportedProperty *property) {
  ARGUMENT_NULL_CHECK(property, InvalidArgument);
  SENSCORD_LOG_DEBUG_TAGGED(
      "libcamera",
      "LibcameraImageStreamSource::Get(ImageSensorFunctionSupportedProperty)");
  return adapter_.GetProperty(property);
}

senscord::Status LibcameraImageStreamSource::Set(
    const std::string &key,
    const senscord::ImageSensorFunctionSupportedProperty *property) {
  ARGUMENT_NULL_CHECK(property, InvalidArgument);
  SENSCORD_LOG_DEBUG_TAGGED(
      "libcamera",
      "LibcameraImageStreamSource::Set(ImageSensorFunctionSupportedProperty)");
  return SENSCORD_STATUS_FAIL(
      "libcamera", senscord::Status::kCauseNotSupported,
      "LibcameraImageStreamSource::Set(ImageSensorFunctionSupportedProperty) "
      "is not supported");
}

senscord::Status LibcameraImageStreamSource::Get(
    const std::string &key, senscord::ImageCropProperty *property) {
  ARGUMENT_NULL_CHECK(property, InvalidArgument);
  SENSCORD_LOG_DEBUG_TAGGED(
      "libcamera", "LibcameraImageStreamSource::Get(ImageCropProperty)");
  *property = image_crop_;
  return senscord::Status::OK();
}

senscord::Status LibcameraImageStreamSource::Set(
    const std::string &key, const senscord::ImageCropProperty *property) {
  ARGUMENT_NULL_CHECK(property, InvalidArgument);
  SENSCORD_LOG_DEBUG_TAGGED(
      "libcamera", "LibcameraImageStreamSource::Set(ImageCropProperty)");
  senscord::Status status;

  image_crop_ = *property;
  status      = adapter_.SetImageCrop(image_crop_.left, image_crop_.top,
                                      image_crop_.width, image_crop_.height);
  if (!status.ok()) {
    util_->SendEventError(status);
    return status;
  }

  return senscord::Status::OK();
}

senscord::Status LibcameraImageStreamSource::Get(
    const std::string &key,
    senscord::libcamera_image::AccessProperty *property) {
  ARGUMENT_NULL_CHECK(property, InvalidArgument);
  SENSCORD_LOG_DEBUG_TAGGED(
      "libcamera",
      "LibcameraImageStreamSource::Get(libcamera_image::AccessProperty)");
  senscord::Status status = senscord::Status::OK();

  // switch (property->type) {
  //   case senscord::libcamera_image::AccessProperty::Type::kProperty:
  //     status = adapter_.GetProperty(property);
  //     break;
  //   case senscord::libcamera_image::AccessProperty::Type::kControl:
  //     status = adapter_.GetControl(property);
  //     break;
  //   default:
  //     break;
  // }

  if (status.ok()) {
    return status;
  }

  util_->SendEventError(status);

  return SENSCORD_STATUS_FAIL(
      "libcamera", senscord::Status::kCauseInvalidArgument,
      "Get(libcamera::AccessProperty) type(%d) is not supported",
      property->type);
}

senscord::Status LibcameraImageStreamSource::Set(
    const std::string &key,
    const senscord::libcamera_image::AccessProperty *property) {
  ARGUMENT_NULL_CHECK(property, InvalidArgument);
  SENSCORD_LOG_DEBUG_TAGGED(
      "libcamera",
      "LibcameraImageStreamSource::Set(libcamera_image::AccessProperty)");
  senscord::Status status;

  switch (property->type) {
    case senscord::libcamera_image::AccessProperty::Type::kProperty:
      return SENSCORD_STATUS_FAIL("libcamera",
                                  senscord::Status::kCauseNotSupported,
                                  "Cannot change libcamera::properties");

    case senscord::libcamera_image::AccessProperty::Type::kControl:
      status = adapter_.SetControl(property);
      break;

    default:
      return SENSCORD_STATUS_FAIL(
          "libcamera", senscord::Status::kCauseOutOfRange,
          "LibcameraImageStreamSource::Set(libcamera_image::"
          "AccessProperty) type(%d) is out of range",
          property->type);
  }

  if (status.ok()) {
    return status;
  }

  util_->SendEventError(status);

  return SENSCORD_STATUS_FAIL(
      "libcamera", senscord::Status::kCauseInvalidArgument,
      "Set(libcamera::AccessProperty) type(%d) is not supported",
      property->type);
}

senscord::Status LibcameraImageStreamSource::Set(
    const std::string &key,
    const senscord::libcamera_image::DeviceEnumerationProperty *property) {
  ARGUMENT_NULL_CHECK(property, InvalidArgument);
  SENSCORD_LOG_DEBUG_TAGGED("libcamera",
                            "LibcameraImageStreamSource::Set(libcamera_image::"
                            "DeviceEnumerationProperty)");
  return SENSCORD_STATUS_FAIL(
      "libcamera", senscord::Status::kCauseNotSupported,
      "LibcameraImageStreamSource::Set(libcamera_image::"
      "DeviceEnumerationProperty) is not supported");
}

senscord::Status LibcameraImageStreamSource::Get(
    const std::string &key,
    senscord::libcamera_image::DeviceEnumerationProperty *property) {
  ARGUMENT_NULL_CHECK(property, InvalidArgument);
  SENSCORD_LOG_DEBUG_TAGGED("libcamera",
                            "LibcameraImageStreamSource::Get(libcamera_image::"
                            "DeviceEnumerationProperty)");
  senscord::Status status = senscord::Status::OK();

  // status = adapter_.GetDevices(property);
  // if (!status.ok()) {
  //   util_->SendEventError(status);
  // }

  return status;
}

senscord::Status LibcameraImageStreamSource::Set(
    const std::string &key,
    const senscord::libcamera_image::ImageRotationProperty *property) {
  ARGUMENT_NULL_CHECK(property, InvalidArgument);
  SENSCORD_LOG_DEBUG_TAGGED("libcamera",
                            "LibcameraImageStreamSource::Set(libcamera_image::"
                            "ImageRotationProperty)");
  return adapter_.SetProperty(property);
}

senscord::Status LibcameraImageStreamSource::Get(
    const std::string &key,
    senscord::libcamera_image::ImageRotationProperty *property) {
  ARGUMENT_NULL_CHECK(property, InvalidArgument);
  SENSCORD_LOG_DEBUG_TAGGED("libcamera",
                            "LibcameraImageStreamSource::Get(libcamera_image::"
                            "ImageRotationProperty)");
  senscord::Status status;
  status = adapter_.GetProperty(property);
  if (!status.ok()) {
    util_->SendEventError(status);
  }

  return status;
}
senscord::Status LibcameraImageStreamSource::Set(
    const std::string &key,
    const senscord::libcamera_image::CameraImageFlipProperty *property) {
  ARGUMENT_NULL_CHECK(property, InvalidArgument);
  SENSCORD_LOG_DEBUG_TAGGED("libcamera",
                            "LibcameraImageStreamSource::Set(libcamera_image::"
                            "CameraImageFlipProperty)");
  senscord::Status status;

  // Ask adapter to apply the change first. If adapter rejects (e.g. BUSY),
  // do not update internal state so that GET returns the previous value.
  status =
      adapter_.SetImageFlip(property->flip_horizontal, property->flip_vertical);

  if (!status.ok()) {
    util_->SendEventError(status);
    return status;
  }

  // Adapter accepted the change; update internal cached value and return OK.
  camera_image_flip_ = *property;
  return senscord::Status::OK();
}

senscord::Status LibcameraImageStreamSource::Get(
    const std::string &key,
    senscord::libcamera_image::CameraImageFlipProperty *property) {
  ARGUMENT_NULL_CHECK(property, InvalidArgument);
  SENSCORD_LOG_DEBUG_TAGGED("libcamera",
                            "LibcameraImageStreamSource::Get(libcamera_image::"
                            "CameraImageFlipProperty)");
  *property = camera_image_flip_;
  return senscord::Status::OK();
}

senscord::Status LibcameraImageStreamSource::Set(
    const std::string &key,
    const senscord::libcamera_image::AIModelBundleIdProperty *property) {
  ARGUMENT_NULL_CHECK(property, InvalidArgument);
  SENSCORD_LOG_DEBUG_TAGGED("libcamera",
                            "LibcameraImageStreamSource::Set(libcamera_image::"
                            "AIModelBundleIdProperty)");
  senscord::Status status;
  status = adapter_.SetProperty(property);
  if (!status.ok()) {
    util_->SendEventError(status);
    return status;
  }

  SafeStringCopy(ai_model_bundle_id_.ai_model_bundle_id,
                 sizeof(ai_model_bundle_id_.ai_model_bundle_id),
                 property->ai_model_bundle_id);

  return senscord::Status::OK();
}

senscord::Status LibcameraImageStreamSource::Get(
    const std::string &key,
    senscord::libcamera_image::AIModelBundleIdProperty *property) {
  ARGUMENT_NULL_CHECK(property, InvalidArgument);
  SENSCORD_LOG_DEBUG_TAGGED("libcamera",
                            "LibcameraImageStreamSource::Get(libcamera_image::"
                            "AIModelBundleIdProperty)");

  SafeStringCopy(property->ai_model_bundle_id,
                 sizeof(property->ai_model_bundle_id),
                 ai_model_bundle_id_.ai_model_bundle_id);
  return senscord::Status::OK();
}
senscord::Status LibcameraImageStreamSource::Set(
    const std::string &key,
    const senscord::libcamera_image::InfoStringProperty *property) {
  ARGUMENT_NULL_CHECK(property, InvalidArgument);
  SENSCORD_LOG_DEBUG_TAGGED("libcamera",
                            "LibcameraImageStreamSource::Set(libcamera_image::"
                            "InfoStringProperty)");

  // InfoStringProperty is read-only in this implementation: setting it is
  // not supported at any time. Tests expect Set() to return
  // INVALID_OPERATION even when the stream is opened but not started.
  return SENSCORD_STATUS_FAIL("libcamera",
                              senscord::Status::kCauseInvalidOperation,
                              "InfoStringProperty is read-only");
}

senscord::Status LibcameraImageStreamSource::Get(
    const std::string &key,
    senscord::libcamera_image::InfoStringProperty *property) {
  ARGUMENT_NULL_CHECK(property, InvalidArgument);
  SENSCORD_LOG_DEBUG_TAGGED("libcamera",
                            "LibcameraImageStreamSource::Get(libcamera_image::"
                            "InfoStringProperty)");
  if (property->category == INFO_STRING_CATEGORY_SENSOR_NAME) {
    SafeStringCopy(property->info, sizeof(property->info), "IMX500");
  } else if (property->category == INFO_STRING_CATEGORY_SENSOR_ID) {
    SafeStringCopy(property->info, sizeof(property->info), imx500_device_id_);
    SENSCORD_LOG_INFO("GetProperty device_id: %s", property->info);
  } else if (property->category == INFO_STRING_CATEGORY_KEY_GENERATION) {
    SafeStringCopy(property->info, sizeof(property->info), "0001");
  } else if (property->category == INFO_STRING_CATEGORY_FIRMWARE_VERSION) {
    property->info[0] = '\0';
  } else if (property->category == INFO_STRING_CATEGORY_LOADER_VERSION) {
    property->info[0] = '\0';
  } else if (property->category == INFO_STRING_CATEGORY_AI_MODEL_VERSION) {
    senscord::Status status;
    std::string ai_model_version;
    status = adapter_.GetAIModelVersion(ai_model_version);
    if (!status.ok()) {
      util_->SendEventError(status);
      return status;
    }

    SafeStringCopy(property->info, sizeof(property->info), ai_model_version);
    SENSCORD_LOG_INFO("GetProperty ai_model_version: %s", property->info);
  } else {
    return SENSCORD_STATUS_FAIL(
        "libcamera", senscord::Status::kCauseNotSupported,
        "LibcameraImageStreamSource::Get(libcamera_image::"
        "InfoStringProperty) category(%d) is not supported",
        property->category);
  }
  return senscord::Status::OK();
}

senscord::Status LibcameraImageStreamSource::Set(
    const std::string &key,
    const senscord::libcamera_image::InputDataTypeProperty *property) {
  ARGUMENT_NULL_CHECK(property, InvalidArgument);
  SENSCORD_LOG_DEBUG_TAGGED("libcamera",
                            "LibcameraImageStreamSource::Set(libcamera_image::"
                            "InputDataTypeProperty)");
  /*ToDO
    implement channel mask */
  return senscord::Status::OK();
}

// InferenceProperty: stream-level GET returns default "inference_t32f",
// SET is not allowed (INVALID_OPERATION)
senscord::Status LibcameraImageStreamSource::Get(
    const std::string &key, senscord::InferenceProperty *property) {
  ARGUMENT_NULL_CHECK(property, InvalidArgument);
  SafeStringCopy(property->data_type, sizeof(property->data_type),
                 kInferenceDataFormatTensor32Float);
  return senscord::Status::OK();
}

senscord::Status LibcameraImageStreamSource::Set(
    const std::string &key, const senscord::InferenceProperty *property) {
  ARGUMENT_NULL_CHECK(property, InvalidArgument);
  return SENSCORD_STATUS_FAIL("libcamera",
                              senscord::Status::kCauseInvalidOperation,
                              "not settable property");
}

// AIModelIndexProperty: stream-level API not supported (tests expect
// NOT_SUPPORTED)
senscord::Status LibcameraImageStreamSource::Get(
    const std::string &key, senscord::AIModelIndexProperty *property) {
  ARGUMENT_NULL_CHECK(property, InvalidArgument);
  return SENSCORD_STATUS_FAIL("libcamera", senscord::Status::kCauseNotSupported,
                              "not supported property");
}

senscord::Status LibcameraImageStreamSource::Set(
    const std::string &key, const senscord::AIModelIndexProperty *property) {
  ARGUMENT_NULL_CHECK(property, InvalidArgument);
  return SENSCORD_STATUS_FAIL("libcamera", senscord::Status::kCauseNotSupported,
                              "not supported property");
}

senscord::Status LibcameraImageStreamSource::Get(
    const std::string &key,
    senscord::libcamera_image::InputDataTypeProperty *property) {
  ARGUMENT_NULL_CHECK(property, InvalidArgument);
  SENSCORD_LOG_DEBUG_TAGGED("libcamera",
                            "LibcameraImageStreamSource::Get(libcamera_image::"
                            "InputDataTypeProperty)");
  /*ToDO
    implement channel mask */
  return senscord::Status::OK();
}

senscord::Status LibcameraImageStreamSource::Set(
    const std::string &key,
    const senscord::libcamera_image::PostProcessAvailableProperty *property) {
  ARGUMENT_NULL_CHECK(property, InvalidArgument);
  SENSCORD_LOG_DEBUG_TAGGED("libcamera",
                            "LibcameraImageStreamSource::Set(libcamera_image::"
                            "PostProcessAvailableProperty)");
  /*ToDO
    implement channel mask */
  return senscord::Status::OK();
}

senscord::Status LibcameraImageStreamSource::Get(
    const std::string &key,
    senscord::libcamera_image::PostProcessAvailableProperty *property) {
  ARGUMENT_NULL_CHECK(property, InvalidArgument);
  SENSCORD_LOG_DEBUG_TAGGED("libcamera",
                            "LibcameraImageStreamSource::Get(libcamera_image::"
                            "PostProcessAvailableProperty)");
  /*ToDO
    implement channel mask */
  return senscord::Status::OK();
}

senscord::Status LibcameraImageStreamSource::Set(
    const std::string &key,
    const senscord::libcamera_image::CameraExposureModeProperty *property) {
  ARGUMENT_NULL_CHECK(property, InvalidArgument);
  SENSCORD_LOG_DEBUG_TAGGED("libcamera",
                            "LibcameraImageStreamSource::Set(libcamera_image::"
                            "CameraExposureModeProperty)");
  senscord::Status status;

  switch (property->mode) {
    case KCameraExposureModeAuto:
      status = adapter_.SetExposureMode(kExposureModeParamAuto);
      if (!status.ok()) {
        util_->SendEventError(status);
        return status;
      }

      break;
    case KCameraExposureModeGainFix:
      status = adapter_.SetExposureMode(kExposureModeParamGainFix);
      if (!status.ok()) {
        util_->SendEventError(status);
        return status;
      }

      break;
    case KCameraExposureModeTimeFix:
      status = adapter_.SetExposureMode(kExposureModeParamTimeFix);
      if (!status.ok()) {
        util_->SendEventError(status);
        return status;
      }

      break;
    case KCameraExposureModeManual:
      status = adapter_.SetExposureMode(kExposureModeParamManual);
      if (!status.ok()) {
        util_->SendEventError(status);
        return status;
      }

      break;
    case KCameraExposureModeHold:
      status = adapter_.SetExposureMode(kExposureModeParamHold);
      if (!status.ok()) {
        util_->SendEventError(status);
        return status;
      }

      break;
    default:
      return SENSCORD_STATUS_FAIL(
          "libcamera", senscord::Status::kCauseOutOfRange,
          "LibcameraImageStreamSource::Set(libcamera_image::"
          "CameraExposureModeProperty) mode(%d) is not supported",
          property->mode);
  }

  camera_exposure_mode_ = *property;

  return senscord::Status::OK();
}

senscord::Status LibcameraImageStreamSource::Get(
    const std::string &key,
    senscord::libcamera_image::CameraExposureModeProperty *property) {
  ARGUMENT_NULL_CHECK(property, InvalidArgument);
  SENSCORD_LOG_DEBUG_TAGGED("libcamera",
                            "LibcameraImageStreamSource::Get(libcamera_image::"
                            "CameraExposureModeProperty)");
  *property = camera_exposure_mode_;
  return senscord::Status::OK();
}

senscord::Status LibcameraImageStreamSource::Set(
    const std::string &key,
    const senscord::libcamera_image::CameraAutoExposureProperty *property) {
  ARGUMENT_NULL_CHECK(property, InvalidArgument);
  SENSCORD_LOG_DEBUG_TAGGED("libcamera",
                            "LibcameraImageStreamSource::Set(libcamera_image::"
                            "CameraAutoExposureProperty)");
  senscord::Status status;

  CameraAutoExposureProperty camera_auto_exposure = *property;

  // Validate max >= min
  if (camera_auto_exposure.max_exposure_time <
      camera_auto_exposure.min_exposure_time) {
    return SENSCORD_STATUS_FAIL("libcamera",
                                senscord::Status::kCauseInvalidArgument,
                                "max_exposure_time(%u) < min_exposure_time(%u)",
                                camera_auto_exposure.max_exposure_time,
                                camera_auto_exposure.min_exposure_time);
  }

  // Validate max_gain is finite
  if (std::isfinite(camera_auto_exposure.max_gain) == 0) {
    return SENSCORD_STATUS_FAIL(
        "libcamera", senscord::Status::kCauseInvalidArgument,
        "invalid value: max_gain %f", camera_auto_exposure.max_gain);
  }

  status = adapter_.SetAutoExposureParam(
      camera_auto_exposure.max_exposure_time,
      camera_auto_exposure.min_exposure_time, camera_auto_exposure.max_gain,
      camera_auto_exposure.convergence_speed);
  if (!status.ok()) {
    util_->SendEventError(status);
    return status;
  }

  return senscord::Status::OK();
}

senscord::Status LibcameraImageStreamSource::Get(
    const std::string &key,
    senscord::libcamera_image::CameraAutoExposureProperty *property) {
  ARGUMENT_NULL_CHECK(property, InvalidArgument);
  SENSCORD_LOG_DEBUG_TAGGED("libcamera",
                            "LibcameraImageStreamSource::Get(libcamera_image::"
                            "CameraAutoExposureProperty)");
  senscord::Status status;

  CameraAutoExposureProperty camera_auto_exposure;
  status = adapter_.GetAutoExposureParam(
      camera_auto_exposure.max_exposure_time,
      camera_auto_exposure.min_exposure_time, camera_auto_exposure.max_gain,
      camera_auto_exposure.convergence_speed);
  if (!status.ok()) {
    util_->SendEventError(status);
    return status;
  }

  *property = camera_auto_exposure;

  return senscord::Status::OK();
}

senscord::Status LibcameraImageStreamSource::Set(
    const std::string &key,
    const senscord::libcamera_image::CameraEvCompensationProperty *property) {
  ARGUMENT_NULL_CHECK(property, InvalidArgument);
  SENSCORD_LOG_DEBUG_TAGGED("libcamera",
                            "LibcameraImageStreamSource::Set(libcamera_image::"
                            "CameraEvCompensationProperty)");
  senscord::Status status;

  CameraEvCompensationProperty camera_ev_compensation = *property;

  status = adapter_.SetAeEvCompensation(camera_ev_compensation.ev_compensation);
  if (!status.ok()) {
    util_->SendEventError(status);
    return status;
  }

  return senscord::Status::OK();
}

senscord::Status LibcameraImageStreamSource::Get(
    const std::string &key,
    senscord::libcamera_image::CameraEvCompensationProperty *property) {
  ARGUMENT_NULL_CHECK(property, InvalidArgument);
  SENSCORD_LOG_DEBUG_TAGGED("libcamera",
                            "LibcameraImageStreamSource::Get(libcamera_image::"
                            "CameraEvCompensationProperty)");
  senscord::Status status;

  CameraEvCompensationProperty camera_ev_compensation;

  status = adapter_.GetAeEvCompensation(camera_ev_compensation.ev_compensation);
  if (!status.ok()) {
    util_->SendEventError(status);
    return status;
  }

  *property = camera_ev_compensation;
  return senscord::Status::OK();
}

senscord::Status LibcameraImageStreamSource::Set(
    const std::string &key,
    const senscord::libcamera_image::CameraAntiFlickerModeProperty *property) {
  ARGUMENT_NULL_CHECK(property, InvalidArgument);
  SENSCORD_LOG_DEBUG_TAGGED("libcamera",
                            "LibcameraImageStreamSource::Set(libcamera_image::"
                            "CameraAntiFlickerModeProperty)");
  senscord::Status status;

  switch (property->anti_flicker_mode) {
    case kCameraAntiFlickerModeOff:
      status = adapter_.SetAeAntiFlickerMode(kAeAntiFlickerModeOff);
      if (!status.ok()) {
        util_->SendEventError(status);
        return status;
      }

      break;
    case kCameraAntiFlickerModeAuto:
      status = adapter_.SetAeAntiFlickerMode(kAeAntiFlickerModeAuto);
      if (!status.ok()) {
        util_->SendEventError(status);
        return status;
      }

      break;
    case kCameraAntiFlickerModeForce50Hz:
      status = adapter_.SetAeAntiFlickerMode(kAeAntiFlickerModeForce50Hz);
      if (!status.ok()) {
        util_->SendEventError(status);
        return status;
      }

      break;
    case kCameraAntiFlickerModeForce60Hz:
      status = adapter_.SetAeAntiFlickerMode(kAeAntiFlickerModeForce60Hz);
      if (!status.ok()) {
        util_->SendEventError(status);
        return status;
      }

      break;
    default:
      return SENSCORD_STATUS_FAIL(
          "libcamera", senscord::Status::kCauseOutOfRange,
          "LibcameraImageStreamSource::Set(libcamera_image::"
          "CameraAntiFlickerModeProperty) mode(%d) is out of range",
          property->anti_flicker_mode);
  }

  camera_anti_flicker_mode_ = *property;

  return senscord::Status::OK();
}

senscord::Status LibcameraImageStreamSource::Get(
    const std::string &key,
    senscord::libcamera_image::CameraAntiFlickerModeProperty *property) {
  ARGUMENT_NULL_CHECK(property, InvalidArgument);
  SENSCORD_LOG_DEBUG_TAGGED("libcamera",
                            "LibcameraImageStreamSource::Get(libcamera_image::"
                            "CameraAntiFlickerModeProperty)");
  *property = camera_anti_flicker_mode_;
  return senscord::Status::OK();
}

senscord::Status LibcameraImageStreamSource::Set(
    const std::string &key,
    const senscord::libcamera_image::CameraManualExposureProperty *property) {
  ARGUMENT_NULL_CHECK(property, InvalidArgument);
  SENSCORD_LOG_DEBUG_TAGGED("libcamera",
                            "LibcameraImageStreamSource::Set(libcamera_image::"
                            "CameraManualExposureProperty)");
  senscord::Status status;

  // Validate gain per T4 behavior: reject NaN/Inf values
  if (std::isfinite(property->gain) == 0) {
    return SENSCORD_STATUS_FAIL("libcamera",
                                senscord::Status::kCauseInvalidArgument,
                                "invalid value: gain %f", property->gain);
  }

  status =
      adapter_.SetManualExposureParam(property->exposure_time, property->gain);
  if (!status.ok()) {
    util_->SendEventError(status);
    return status;
  }
  // Retrieve applied (possibly clipped/rounded) values from adapter and
  // reflect them in camera_manual_exposure_ so GET returns the actual device
  // state.
  uint32_t applied_exp = 0;
  float applied_gain   = 0.0f;
  senscord::Status s2 =
      adapter_.GetManualExposureParam(applied_exp, applied_gain);
  if (s2.ok()) {
    camera_manual_exposure_.exposure_time = applied_exp;
    camera_manual_exposure_.gain          = applied_gain;
  } else {
    SENSCORD_LOG_WARNING_TAGGED(
        "libcamera", "Failed to get applied manual exposure values: %s",
        s2.ToString().c_str());
  }

  return senscord::Status::OK();
}

senscord::Status LibcameraImageStreamSource::Get(
    const std::string &key,
    senscord::libcamera_image::CameraManualExposureProperty *property) {
  ARGUMENT_NULL_CHECK(property, InvalidArgument);
  SENSCORD_LOG_DEBUG_TAGGED("libcamera",
                            "LibcameraImageStreamSource::Get(libcamera_image::"
                            "CameraManualExposureProperty)");
  *property = camera_manual_exposure_;
  return senscord::Status::OK();
}

senscord::Status LibcameraImageStreamSource::Set(
    const std::string &key,
    const senscord::libcamera_image::CameraAutoExposureMeteringProperty
        *property) {
  ARGUMENT_NULL_CHECK(property, InvalidArgument);
  SENSCORD_LOG_DEBUG_TAGGED("libcamera",
                            "LibcameraImageStreamSource::Set(libcamera_image::"
                            "CameraAutoExposureMeteringProperty)");
  senscord::Status status;
  AeMeteringWindow AeWindow;

  switch (property->mode) {
    case kCameraAutoExposureMeteringModeFullScreen:
      status = adapter_.SetAeMetering(kAeMeteringFullScreen, AeWindow);
      if (!status.ok()) {
        util_->SendEventError(status);
        return status;
      }

      break;
    case kCameraAutoExposureMeteringModeUserWindow:
      AeWindow = {property->window.top, property->window.left,
                  property->window.bottom, property->window.right};
      status   = adapter_.SetAeMetering(kAeMeteringUserWindow, AeWindow);
      if (!status.ok()) {
        util_->SendEventError(status);
        return status;
      }

      break;
    default:
      return SENSCORD_STATUS_FAIL(
          "libcamera", senscord::Status::kCauseOutOfRange,
          "LibcameraImageStreamSource::Set(libcamera_image::"
          "CameraAutoExposureMeteringProperty) mode(%d) is out of range",
          property->mode);
  }

  camera_auto_exposure_metering_ = *property;

  return senscord::Status::OK();
}

senscord::Status LibcameraImageStreamSource::Get(
    const std::string &key,
    senscord::libcamera_image::CameraAutoExposureMeteringProperty *property) {
  ARGUMENT_NULL_CHECK(property, InvalidArgument);
  SENSCORD_LOG_DEBUG_TAGGED("libcamera",
                            "LibcameraImageStreamSource::Get(libcamera_image::"
                            "CameraAutoExposureMeteringProperty)");
  *property = camera_auto_exposure_metering_;
  return senscord::Status::OK();
}

senscord::Status LibcameraImageStreamSource::Set(
    const std::string &key,
    const senscord::libcamera_image::CameraTemperatureProperty *property) {
  ARGUMENT_NULL_CHECK(property, InvalidArgument);
  SENSCORD_LOG_DEBUG_TAGGED("libcamera",
                            "LibcameraImageStreamSource::Set(libcamera_image::"
                            "CameraTemperatureProperty)");
  return SENSCORD_STATUS_FAIL(
      "libcamera", senscord::Status::kCauseInvalidOperation,
      "LibcameraImageStreamSource::Set(libcamera_image::"
      "CameraTemperatureProperty) is read-only");
}

senscord::Status LibcameraImageStreamSource::Get(
    const std::string &key,
    senscord::libcamera_image::CameraTemperatureProperty *property) {
  ARGUMENT_NULL_CHECK(property, InvalidArgument);
  SENSCORD_LOG_DEBUG_TAGGED("libcamera",
                            "LibcameraImageStreamSource::Get(libcamera_image::"
                            "CameraTemperatureProperty)");
  return adapter_.GetProperty(property);
}

senscord::Status LibcameraImageStreamSource::Set(
    const std::string &key,
    const senscord::libcamera_image::CameraImageSizeProperty *property) {
  ARGUMENT_NULL_CHECK(property, InvalidArgument);
  SENSCORD_LOG_DEBUG_TAGGED("libcamera",
                            "LibcameraImageStreamSource::Set(libcamera_image::"
                            "CameraImageSizeProperty)");
  senscord::Status status;

  // Validate scaling_policy input
  if (property->scaling_policy < kCameraScalingPolicyAuto ||
      property->scaling_policy > kCameraScalingPolicyResolution) {
    return SENSCORD_STATUS_FAIL(
        "libcamera", senscord::Status::kCauseUnknown,
        "LibcameraImageStreamSource::Set(CameraImageSizeProperty) "
        "scaling_policy(%d) is not supported",
        static_cast<int>(property->scaling_policy));
  }

  status = adapter_.SetImageSize(property->width, property->height);
  if (!status.ok()) {
    util_->SendEventError(status);
    return status;
  }

  camera_image_size_ = *property;

  return senscord::Status::OK();
}

senscord::Status LibcameraImageStreamSource::Get(
    const std::string &key,
    senscord::libcamera_image::CameraImageSizeProperty *property) {
  ARGUMENT_NULL_CHECK(property, InvalidArgument);
  SENSCORD_LOG_DEBUG_TAGGED("libcamera",
                            "LibcameraImageStreamSource::Get(libcamera_image::"
                            "CameraImageSizeProperty)");
  *property = camera_image_size_;
  return senscord::Status::OK();
}

senscord::Status LibcameraImageStreamSource::Set(
    const std::string &key,
    const senscord::libcamera_image::CameraFrameRateProperty *property) {
  ARGUMENT_NULL_CHECK(property, InvalidArgument);
  SENSCORD_LOG_DEBUG_TAGGED("libcamera",
                            "LibcameraImageStreamSource::Set(libcamera_image::"
                            "CameraFrameRateProperty)");
  senscord::Status status;

  // Validate frame rate values
  if (property->num == 0) {
    return SENSCORD_STATUS_FAIL("libcamera",
                                senscord::Status::kCauseInvalidArgument,
                                "frame rate numerator must be greater than 0");
  }

  if (property->denom == 0) {
    return SENSCORD_STATUS_FAIL(
        "libcamera", senscord::Status::kCauseInvalidArgument,
        "frame rate denominator must be greater than 0");
  }

  status = adapter_.SetFrameRate(property->num, property->denom);
  if (!status.ok()) {
    util_->SendEventError(status);
    return status;
  }

  camera_frame_rate_ = *property;

  return senscord::Status::OK();
}

senscord::Status LibcameraImageStreamSource::Get(
    const std::string &key,
    senscord::libcamera_image::CameraFrameRateProperty *property) {
  ARGUMENT_NULL_CHECK(property, InvalidArgument);
  SENSCORD_LOG_DEBUG_TAGGED("libcamera",
                            "LibcameraImageStreamSource::Get(libcamera_image::"
                            "CameraFrameRateProperty)");
  *property = camera_frame_rate_;
  return senscord::Status::OK();
}

senscord::Status LibcameraImageStreamSource::Set(
    const std::string &key,
    const senscord::libcamera_image::IspImageProperty *property) {
  ARGUMENT_NULL_CHECK(property, InvalidArgument);
  SENSCORD_LOG_DEBUG_TAGGED("libcamera",
                            "LibcameraImageStreamSource::Set(libcamera_image::"
                            "IspImageProperty)");
  senscord::Status status;
  IspImageProperty isp_image = *property;
  status = adapter_.SetIspImage(isp_image.width, isp_image.height,
                                isp_image.pixel_format);
  if (!status.ok()) {
    util_->SendEventError(status);
    return status;
  }

  return senscord::Status::OK();
}

senscord::Status LibcameraImageStreamSource::Get(
    const std::string &key,
    senscord::libcamera_image::IspImageProperty *property) {
  ARGUMENT_NULL_CHECK(property, InvalidArgument);
  SENSCORD_LOG_DEBUG_TAGGED("libcamera",
                            "LibcameraImageStreamSource::Get(libcamera_image::"
                            "IspImageProperty)");
  senscord::Status status;
  IspImageProperty isp_image;
  status = adapter_.GetIspImage(isp_image.width, isp_image.height,
                                isp_image.stride_bytes, isp_image.pixel_format);
  if (!status.ok()) {
    util_->SendEventError(status);
    return status;
  }

  *property = isp_image;
  return senscord::Status::OK();
}

senscord::Status LibcameraImageStreamSource::Set(
    const std::string &key,
    const senscord::libcamera_image::IspFrameRateProperty *property) {
  ARGUMENT_NULL_CHECK(property, InvalidArgument);
  SENSCORD_LOG_DEBUG_TAGGED("libcamera",
                            "LibcameraImageStreamSource::Set(libcamera_image::"
                            "IspFrameRateProperty)");
  senscord::Status status;
  isp_frame_rate_ = *property;
  status = adapter_.SetIspFrameRate(isp_frame_rate_.num, isp_frame_rate_.denom);
  if (!status.ok()) {
    util_->SendEventError(status);
    return status;
  }

  return senscord::Status::OK();
}

senscord::Status LibcameraImageStreamSource::Get(
    const std::string &key,
    senscord::libcamera_image::IspFrameRateProperty *property) {
  ARGUMENT_NULL_CHECK(property, InvalidArgument);
  SENSCORD_LOG_DEBUG_TAGGED("libcamera",
                            "LibcameraImageStreamSource::Get(libcamera_image::"
                            "IspFrameRateProperty)");
  *property = isp_frame_rate_;
  return senscord::Status::OK();
}

senscord::Status LibcameraImageStreamSource::Set(
    const std::string &key, const senscord::StreamStateProperty *property) {
  ARGUMENT_NULL_CHECK(property, InvalidArgument);
  SENSCORD_LOG_DEBUG_TAGGED("libcamera",
                            "LibcameraImageStreamSource::Set("
                            "StreamStateProperty)");
  return SENSCORD_STATUS_FAIL("libcamera", senscord::Status::kCauseNotSupported,
                              "LibcameraImageStreamSource::Set("
                              "StreamStateProperty) is not supported");
}

senscord::Status LibcameraImageStreamSource::Get(
    const std::string &key, senscord::StreamStateProperty *property) {
  ARGUMENT_NULL_CHECK(property, InvalidArgument);
  SENSCORD_LOG_DEBUG_TAGGED("libcamera",
                            "LibcameraImageStreamSource::Get("
                            "StreamStateProperty)");
  bool is_running = adapter_.GetIsRunning();
  property->state = is_running ? StreamState::kStreamStateRunning
                               : StreamState::kStreamStateReady;
  return senscord::Status::OK();
}

bool LibcameraImageStreamSource::GetDeviceID(void) {
  std::string device_id_str = "";
  senscord::Status status;

  std::unique_lock<std::mutex> _lck(device_id_mutex_);

  /* Start */
  adapter_.GetImageProperty(&image_property_,
                            AITRIOS_SENSOR_CHANNEL_ID_INFERENCE_RAW_IMAGE);
  status = Set(senscord::kFrameRatePropertyKey, &framerate_property_);
  if (!status.ok()) {
    SENSCORD_LOG_WARNING("Failed to set frame rate before configure");
    return false;
  }
  status = adapter_.Configure(image_property_);
  if (!status.ok()) {
    SENSCORD_LOG_WARNING("Failed to configure camera");
    return false;
  }
  status = adapter_.Start();
  if (!status.ok()) {
    SENSCORD_LOG_WARNING("Failed to start camera");
    return false;
  }

  /* GetFrames */
  const int max_try_count = 20; /* kWaitOnGetFrames(4ms) * 20 = 80ms */
  int try_count           = 0;
  while (max_try_count > try_count) {
    std::vector<FrameInfo> frames;
    adapter_.GetFrames(&frames, true);
    if (frames.empty()) {
      std::this_thread::sleep_for(std::chrono::microseconds(kWaitOnGetFrames));
      try_count++;
      continue;
    } else {
      /* If dry_run is enabled, an empty frame is got, so releasing is
       * unnecessary. */
      break;
    }
  }

  if (!adapter_.GetDeviceID(device_id_str)) {
    SENSCORD_LOG_WARNING("Failed to get device id.");

    status = adapter_.Stop();
    if (!status.ok()) {
      SENSCORD_LOG_WARNING("Failed to stop camera");
    }

    return false;
  }

  /* Stop */
  status = adapter_.Stop();
  if (!status.ok()) {
    SENSCORD_LOG_WARNING("Failed to stop camera");
    return false;
  }

  imx500_device_id_ = device_id_str;

  return true;
}

}  // namespace libcamera_image
}  // namespace senscord
