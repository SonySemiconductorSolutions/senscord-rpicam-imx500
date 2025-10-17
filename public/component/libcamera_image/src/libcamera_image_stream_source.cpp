/*
 * SPDX-FileCopyrightText: 2023 Sony Semiconductor Solutions Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "libcamera_image_stream_source.h"

#include <libcamera/control_ids.h>
#include <libcamera/controls.h>

#include <string>
#include <filesystem>
#include <iomanip>
#include <sstream>
#include <chrono>
#include <thread>
#include <sys/ioctl.h>
#include <linux/videodev2.h>
#include <fcntl.h>

#include "rpicam_app_adapter.h"
#include "v4l2_ctrl_manager.h"
#include "senscord/logger.h"
#include "senscord/status.h"

namespace fs = std::filesystem;

namespace senscord {
namespace libcamera_image {

// kWaitOnGetFrames will be referenced from GetFrames()
// Expected maximum framerate is 120fps, which is 8ms
// the half of it is sufficient to avoid both of busy loop and other performance
// problem
const uint64_t LibcameraImageStreamSource::kWaitOnGetFrames = 4'000'000;

LibcameraImageStreamSource::LibcameraImageStreamSource()
    : imx500_device_id_("") {}

LibcameraImageStreamSource::~LibcameraImageStreamSource() {
  // Do nothing.
}

senscord::Status LibcameraImageStreamSource::Open(
    senscord::Core *core, senscord::StreamSourceUtility *util) {
  SENSCORD_LOG_DEBUG_TAGGED("libcamera", "LibcameraImageStreamSource::Open()");
  core_ = core;
  util_ = util;

  memset(ai_model_bundle_id_.ai_model_bundle_id, 0, sizeof(char) * kAIModelBundleIdLength);

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
      util_, senscord::libcamera_image::kLibcameraCameraEvCompensationPropertyKey,
      senscord::libcamera_image::CameraEvCompensationProperty);
  SENSCORD_REGISTER_PROPERTY(
      util_, senscord::libcamera_image::kLibcameraCameraAntiFlickerModePropertyKey,
      senscord::libcamera_image::CameraAntiFlickerModeProperty);
  SENSCORD_REGISTER_PROPERTY(
      util_, senscord::libcamera_image::kLibcameraCameraAutoExposureMeteringPropertykey,
      senscord::libcamera_image::CameraAutoExposureMeteringProperty);
  SENSCORD_REGISTER_PROPERTY(
      util_, senscord::libcamera_image::kLibcameraCameraManualExposurePropertykey,
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
      util_, senscord::libcamera_image::kLibcameraCameraImagePropertyKey,
      senscord::libcamera_image::CameraImageProperty);

  std::string device = "";
  uint64_t uint_value = 0;
  std::string string_value = "";
  senscord::Status status = senscord::Status::OK();

  // parse arguments
  {
    status = util_->GetStreamArgument("device", &device);
    if (!status.ok()) {
      device = "";
    }
  }

  // parse arguments: ImageProperty
  {
    image_property_.width = 640;
    image_property_.height = 480;
    image_property_.pixel_format = senscord::kPixelFormatBGR24;

    {
      status = util_->GetStreamArgument("width", &uint_value);
      if (status.ok()) {
        image_property_.width = static_cast<uint32_t>(uint_value);
      }
    }

    {
      status = util_->GetStreamArgument("height", &uint_value);
      if (status.ok()) {
        image_property_.height = static_cast<uint32_t>(uint_value);
      }
    }

    {
      status = util_->GetStreamArgument("pixel_format", &string_value);
      if (status.ok()) {
        image_property_.pixel_format = string_value;
      }
    }
    util_->UpdateChannelProperty(AITRIOS_SENSOR_CHANNEL_ID_INFERENCE_RAW_IMAGE,
                               senscord::kImagePropertyKey,
                               &image_property_);
    // Do not treat image_property_.stride_bytes in this time, because it will
    // be filled by libcamera_adapter::Configure()
    util_->SendEventPropertyUpdated(senscord::kImagePropertyKey);
  }

  // parse arguments: FrameRateProperty
  {
    senscord::FrameRateProperty framerate_property = {};
    framerate_property.num = 30;
    framerate_property.denom = 1;

    status = util_->GetStreamArgument("fps", &uint_value);
    if (status.ok()) {
      framerate_property.num = static_cast<uint32_t>(uint_value);
    }

    Set(senscord::kFrameRatePropertyKey, &framerate_property);
  }

  {
    display_channel_ = AITRIOS_SENSOR_CHANNEL_ID_INFERENCE_INPUT_IMAGE;
    status = util_->GetStreamArgument("display_channel", &uint_value);
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
  inference_image_property.width = image_property_.width;
  inference_image_property.height = image_property_.height;
  inference_image_property.pixel_format = "image_rgb24";

  util_->UpdateChannelProperty(AITRIOS_SENSOR_CHANNEL_ID_INFERENCE_INPUT_IMAGE,
                               senscord::kImagePropertyKey,
                               &inference_image_property);
  util_->UpdateChannelProperty(AITRIOS_SENSOR_CHANNEL_ID_INFERENCE_RAW_IMAGE,
                               senscord::kImagePropertyKey, &image_property_);

  // get device_id
  if (imx500_device_id_.empty()) {
    if (!GetDeviceID()) {
      return SENSCORD_STATUS_FAIL(
        "libcamera", senscord::Status::kCauseInvalidArgument,
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

  // Apply initial image property
  adapter_.GetImageProperty(&image_property_, AITRIOS_SENSOR_CHANNEL_ID_INFERENCE_RAW_IMAGE);
  status = Set(senscord::kImagePropertyKey, &image_property_);
  if (!status.ok()) {
    util_->SendEventError(status);
    return status;
  }

  // Apply initial framerate
  status = Set(senscord::kFrameRatePropertyKey, &framerate_property_);
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
  SENSCORD_LOG_DEBUG_TAGGED(
      "libcamera", "LibcameraImageStreamSource::Get(ChannelInfoProperty)");

  // TODO: multi-plane support
  senscord::ChannelInfo info0 = {};
  info0.raw_data_type = senscord::kRawDataTypeMeta;
  info0.description = "meta";

  property->channels.insert(std::make_pair(AITRIOS_SENSOR_CHANNEL_ID_INFERENCE_OUTPUT, info0));
  senscord::ChannelInfo info1 = {};
  info1.raw_data_type = senscord::kRawDataTypeImage;
  info1.description = "input_tensor";

  property->channels.insert(std::make_pair(AITRIOS_SENSOR_CHANNEL_ID_INFERENCE_INPUT_IMAGE, info1));

  senscord::ChannelInfo info2 = {};
  info2.raw_data_type = senscord::kRawDataTypeImage;
  info2.description = "full_image";

  property->channels.insert(std::make_pair(AITRIOS_SENSOR_CHANNEL_ID_INFERENCE_RAW_IMAGE, info2));

  return senscord::Status::OK();
}

senscord::Status LibcameraImageStreamSource::Get(
    const std::string &key, senscord::FrameRateProperty *property) {
  SENSCORD_LOG_DEBUG_TAGGED(
      "libcamera", "LibcameraImageStreamSource::Get(FrameRateProperty)");
  *property = framerate_property_;
  return senscord::Status::OK();
}

senscord::Status LibcameraImageStreamSource::Set(
    const std::string &key, const senscord::FrameRateProperty *property) {
  SENSCORD_LOG_DEBUG_TAGGED(
      "libcamera", "LibcameraImageStreamSource::Set(FrameRateProperty)");
  framerate_property_ = *property;

  int64_t frame_time =
      1000000 / framerate_property_.num / framerate_property_.denom;
  std::vector<int64_t> frame_duration = {frame_time, frame_time};

  libcamera_image::AccessProperty access_property = {};
  access_property.type = libcamera_image::AccessProperty::Type::kControl;
  access_property.id = "FrameDurationLimits";
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
  SENSCORD_LOG_DEBUG_TAGGED("libcamera",
                            "LibcameraImageStreamSource::Get(ImageProperty)");
  adapter_.GetImageProperty(property, display_channel_);
  return senscord::Status::OK();
}

senscord::Status LibcameraImageStreamSource::Set(
    const std::string &key, const senscord::ImageProperty *property) {
  SENSCORD_LOG_DEBUG_TAGGED("libcamera",
                            "LibcameraImageStreamSource::Set(ImageProperty)");

  image_property_ = *property;
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
  SENSCORD_LOG_DEBUG_TAGGED(
      "libcamera",
      "LibcameraImageStreamSource::Get(ImageSensorFunctionSupportedProperty)");
  return adapter_.GetProperty(property);
}

senscord::Status LibcameraImageStreamSource::Set(
    const std::string &key,
    const senscord::ImageSensorFunctionSupportedProperty *property) {
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
  SENSCORD_LOG_DEBUG_TAGGED(
      "libcamera", "LibcameraImageStreamSource::Get(ImageCropProperty)");
  *property = image_crop_;
  return senscord::Status::OK();
}

senscord::Status LibcameraImageStreamSource::Set(
    const std::string &key, const senscord::ImageCropProperty *property) {
  SENSCORD_LOG_DEBUG_TAGGED(
      "libcamera", "LibcameraImageStreamSource::Set(ImageCropProperty)");
  senscord::Status status;

  image_crop_ = *property;
  status = adapter_.SetImageCrop(
                            image_crop_.left,
                            image_crop_.top,
                            image_crop_.width,
                            image_crop_.height);
  if (!status.ok()) {
    util_->SendEventError(status);
    return status;
  }

  return senscord::Status::OK();
}

senscord::Status LibcameraImageStreamSource::Get(
    const std::string &key,
    senscord::libcamera_image::AccessProperty *property) {
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
      break;
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
  SENSCORD_LOG_DEBUG_TAGGED("libcamera",
                            "LibcameraImageStreamSource::Set(libcamera_image::"
                            "ImageRotationProperty)");
  return adapter_.SetProperty(property);
}

senscord::Status LibcameraImageStreamSource::Get(
    const std::string &key,
    senscord::libcamera_image::ImageRotationProperty *property) {
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
  SENSCORD_LOG_DEBUG_TAGGED("libcamera",
                            "LibcameraImageStreamSource::Set(libcamera_image::"
                            "CameraImageFlipProperty)");
  senscord::Status status;

  camera_image_flip_ = *property;
  status = adapter_.SetImageFlip(
                    camera_image_flip_.flip_horizontal,
                    camera_image_flip_.flip_vertical);

  if (!status.ok()) {
    util_->SendEventError(status);
    return status;
  }

  return senscord::Status::OK();
}

senscord::Status LibcameraImageStreamSource::Get(
    const std::string &key,
    senscord::libcamera_image::CameraImageFlipProperty *property) {
  SENSCORD_LOG_DEBUG_TAGGED("libcamera",
                            "LibcameraImageStreamSource::Get(libcamera_image::"
                            "CameraImageFlipProperty)");
  *property = camera_image_flip_;
  return senscord::Status::OK();
}

senscord::Status LibcameraImageStreamSource::Set(
    const std::string &key,
    const senscord::libcamera_image::AIModelBundleIdProperty *property) {
  SENSCORD_LOG_DEBUG_TAGGED("libcamera",
                            "LibcameraImageStreamSource::Set(libcamera_image::"
                            "AIModelBundleIdProperty)");
  senscord::Status status;
  status = adapter_.SetProperty(property);
  if (!status.ok()) {
     util_->SendEventError(status);
     return status;
  }

  memcpy(ai_model_bundle_id_.ai_model_bundle_id, property->ai_model_bundle_id,
         senscord::libcamera_image::kAIModelBundleIdLength);

  return senscord::Status::OK();
}

senscord::Status LibcameraImageStreamSource::Get(
    const std::string &key,
    senscord::libcamera_image::AIModelBundleIdProperty *property) {
  SENSCORD_LOG_DEBUG_TAGGED("libcamera",
                            "LibcameraImageStreamSource::Get(libcamera_image::"
                            "AIModelBundleIdProperty)");
  memcpy(property->ai_model_bundle_id, ai_model_bundle_id_.ai_model_bundle_id,
         senscord::libcamera_image::kAIModelBundleIdLength);
  return senscord::Status::OK();
}
senscord::Status LibcameraImageStreamSource::Set(
    const std::string &key,
    const senscord::libcamera_image::InfoStringProperty *property) {
  SENSCORD_LOG_DEBUG_TAGGED("libcamera",
                            "LibcameraImageStreamSource::Set(libcamera_image::"
                            "InfoStringProperty)");

  return senscord::Status::OK();
}

senscord::Status LibcameraImageStreamSource::Get(
    const std::string &key,
    senscord::libcamera_image::InfoStringProperty *property) {
  SENSCORD_LOG_DEBUG_TAGGED("libcamera",
                            "LibcameraImageStreamSource::Get(libcamera_image::"
                            "InfoStringProperty)");
  if (property->category == INFO_STRING_CATEGORY_SENSOR_NAME) {
    std::string imx500_str = "IMX500";
    strncpy(property->info, imx500_str.c_str(), imx500_str.size());
  } else if (property->category == INFO_STRING_CATEGORY_SENSOR_ID) {
    strncpy(property->info, imx500_device_id_.c_str(), imx500_device_id_.length());
    SENSCORD_LOG_INFO("GetProperty device_id: %s", property->info);
  } else if (property->category == INFO_STRING_CATEGORY_KEY_GENERATION) {
    strncpy(property->info, "0001", 4);
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

    strncpy(property->info, ai_model_version.c_str(), ai_model_version.length());
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
  SENSCORD_LOG_DEBUG_TAGGED("libcamera",
                            "LibcameraImageStreamSource::Set(libcamera_image::"
                            "InputDataTypeProperty)");
  /*ToDO
    implement channel mask */
  return senscord::Status::OK();
}

senscord::Status LibcameraImageStreamSource::Get(
    const std::string &key,
    senscord::libcamera_image::InputDataTypeProperty *property) {
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
          "libcamera", senscord::Status::kCauseNotSupported,
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
  SENSCORD_LOG_DEBUG_TAGGED("libcamera",
                            "LibcameraImageStreamSource::Get(libcamera_image::"
                            "CameraExposureModeProperty)");
  *property = camera_exposure_mode_;
  return senscord::Status::OK();
}

senscord::Status LibcameraImageStreamSource::Set(
    const std::string &key,
    const senscord::libcamera_image::CameraAutoExposureProperty *property) {
  SENSCORD_LOG_DEBUG_TAGGED("libcamera",
                            "LibcameraImageStreamSource::Set(libcamera_image::"
                            "CameraAutoExposureProperty)");
  senscord::Status status;

  camera_auto_exposure_ = *property;
  status = adapter_.SetAutoExposureParam(
                            camera_auto_exposure_.max_exposure_time,
                            camera_auto_exposure_.min_exposure_time,
                            camera_auto_exposure_.max_gain,
                            camera_auto_exposure_.convergence_speed);
  if (!status.ok()) {
    util_->SendEventError(status);
    return status;
  }

  return senscord::Status::OK();
}

senscord::Status LibcameraImageStreamSource::Get(
    const std::string &key,
    senscord::libcamera_image::CameraAutoExposureProperty *property) {
  SENSCORD_LOG_DEBUG_TAGGED("libcamera",
                            "LibcameraImageStreamSource::Get(libcamera_image::"
                            "CameraAutoExposureProperty)");
  *property = camera_auto_exposure_;
  return senscord::Status::OK();
}

senscord::Status LibcameraImageStreamSource::Set(
    const std::string &key,
    const senscord::libcamera_image::CameraEvCompensationProperty *property) {
  SENSCORD_LOG_DEBUG_TAGGED("libcamera",
                            "LibcameraImageStreamSource::Set(libcamera_image::"
                            "CameraEvCompensationProperty)");
  senscord::Status status;

  camera_ev_compensation_ = *property;

  status = adapter_.SetAeEvCompensation(camera_ev_compensation_.ev_compensation);
  if (!status.ok()) {
    util_->SendEventError(status);
    return status;
  }

  return senscord::Status::OK();
}

senscord::Status LibcameraImageStreamSource::Get(
    const std::string &key,
    senscord::libcamera_image::CameraEvCompensationProperty *property) {
  SENSCORD_LOG_DEBUG_TAGGED("libcamera",
                            "LibcameraImageStreamSource::Get(libcamera_image::"
                            "CameraEvCompensationProperty)");
  *property = camera_ev_compensation_;
  return senscord::Status::OK();
}

senscord::Status LibcameraImageStreamSource::Set(
    const std::string &key,
    const senscord::libcamera_image::CameraAntiFlickerModeProperty *property) {
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
          "libcamera", senscord::Status::kCauseNotSupported,
          "LibcameraImageStreamSource::Set(libcamera_image::"
          "CameraAntiFlickerModeProperty) mode(%d) is not supported",
          property->anti_flicker_mode);
  }

  camera_anti_flicker_mode_ = *property;

  return senscord::Status::OK();
}

senscord::Status LibcameraImageStreamSource::Get(
    const std::string &key,
    senscord::libcamera_image::CameraAntiFlickerModeProperty *property) {
  SENSCORD_LOG_DEBUG_TAGGED("libcamera",
                            "LibcameraImageStreamSource::Get(libcamera_image::"
                            "CameraAntiFlickerModeProperty)");
  *property = camera_anti_flicker_mode_;
  return senscord::Status::OK();
}

senscord::Status LibcameraImageStreamSource::Set(
    const std::string &key,
    const senscord::libcamera_image::CameraManualExposureProperty *property) {
  SENSCORD_LOG_DEBUG_TAGGED("libcamera",
                            "LibcameraImageStreamSource::Set(libcamera_image::"
                            "CameraManualExposureProperty)");
  senscord::Status status;

  camera_manual_exposure_ = *property;
  status = adapter_.SetManualExposureParam(
                    camera_manual_exposure_.exposure_time,
                    camera_manual_exposure_.gain);
  if (!status.ok()) {
    util_->SendEventError(status);
    return status;
  }

  return senscord::Status::OK();
}

senscord::Status LibcameraImageStreamSource::Get(
    const std::string &key,
    senscord::libcamera_image::CameraManualExposureProperty *property) {
  SENSCORD_LOG_DEBUG_TAGGED("libcamera",
                            "LibcameraImageStreamSource::Get(libcamera_image::"
                            "CameraManualExposureProperty)");
  *property = camera_manual_exposure_;
  return senscord::Status::OK();
}

senscord::Status LibcameraImageStreamSource::Set(
    const std::string &key,
    const senscord::libcamera_image::CameraAutoExposureMeteringProperty *property) {
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
      AeWindow = {
          property->window.top,
          property->window.left,
          property->window.bottom,
          property->window.right
      };
      status = adapter_.SetAeMetering(kAeMeteringUserWindow, AeWindow);
      if (!status.ok()) {
        util_->SendEventError(status);
        return status;
      }

      break;
    default:
      return SENSCORD_STATUS_FAIL(
          "libcamera", senscord::Status::kCauseNotSupported,
          "LibcameraImageStreamSource::Set(libcamera_image::"
          "CameraAutoExposureMeteringProperty) mode(%d) is not supported",
          property->mode);
  }

  camera_auto_exposure_metering_ = *property;

  return senscord::Status::OK();
}

senscord::Status LibcameraImageStreamSource::Get(
    const std::string &key,
    senscord::libcamera_image::CameraAutoExposureMeteringProperty *property) {
  SENSCORD_LOG_DEBUG_TAGGED("libcamera",
                            "LibcameraImageStreamSource::Get(libcamera_image::"
                            "CameraAutoExposureMeteringProperty)");
  *property = camera_auto_exposure_metering_;
  return senscord::Status::OK();
}

senscord::Status LibcameraImageStreamSource::Set(
    const std::string &key,
    const senscord::libcamera_image::CameraTemperatureProperty *property) {
  SENSCORD_LOG_DEBUG_TAGGED("libcamera",
                            "LibcameraImageStreamSource::Set(libcamera_image::"
                            "CameraTemperatureProperty)");
  return SENSCORD_STATUS_FAIL(
      "libcamera", senscord::Status::kCauseNotSupported,
      "LibcameraImageStreamSource::Set(libcamera_image::"
      "CameraTemperatureProperty) is not supported");
}

senscord::Status LibcameraImageStreamSource::Get(
    const std::string &key,
    senscord::libcamera_image::CameraTemperatureProperty *property) {
  SENSCORD_LOG_DEBUG_TAGGED("libcamera",
                            "LibcameraImageStreamSource::Get(libcamera_image::"
                            "CameraTemperatureProperty)");
  return adapter_.GetProperty(property);
}

senscord::Status LibcameraImageStreamSource::Set(
    const std::string &key,
    const senscord::libcamera_image::CameraImageSizeProperty *property) {
  SENSCORD_LOG_DEBUG_TAGGED("libcamera",
                            "LibcameraImageStreamSource::Set(libcamera_image::"
                            "CameraImageSizeProperty)");
  senscord::Status status;

  camera_image_size_ = *property;
  status = adapter_.SetImageSize(
                    camera_image_size_.width,
                    camera_image_size_.height);
  if (!status.ok()) {
    util_->SendEventError(status);
    return status;
  }

  return senscord::Status::OK();
}

senscord::Status LibcameraImageStreamSource::Get(
    const std::string &key,
    senscord::libcamera_image::CameraImageSizeProperty *property) {
  SENSCORD_LOG_DEBUG_TAGGED("libcamera",
                            "LibcameraImageStreamSource::Get(libcamera_image::"
                            "CameraImageSizeProperty)");
  *property = camera_image_size_;
  return senscord::Status::OK();
}

senscord::Status LibcameraImageStreamSource::Set(
    const std::string &key,
    const senscord::libcamera_image::CameraFrameRateProperty *property) {
  SENSCORD_LOG_DEBUG_TAGGED("libcamera",
                            "LibcameraImageStreamSource::Set(libcamera_image::"
                            "CameraFrameRateProperty)");
  senscord::Status status;

  camera_frame_rate_ = *property;
  status = adapter_.SetFrameRate(
                    camera_frame_rate_.num,
                    camera_frame_rate_.denom);
  if (!status.ok()) {
    util_->SendEventError(status);
    return status;
  }

  return senscord::Status::OK();
}

senscord::Status LibcameraImageStreamSource::Get(
    const std::string &key,
    senscord::libcamera_image::CameraFrameRateProperty *property) {
  SENSCORD_LOG_DEBUG_TAGGED("libcamera",
                            "LibcameraImageStreamSource::Get(libcamera_image::"
                            "CameraFrameRateProperty)");
  *property = camera_frame_rate_;
  return senscord::Status::OK();
}

senscord::Status LibcameraImageStreamSource::Set(
    const std::string &key,
    const senscord::libcamera_image::CameraImageProperty *property) {
  SENSCORD_LOG_DEBUG_TAGGED("libcamera",
                            "LibcameraImageStreamSource::Set(libcamera_image::"
                            "CameraImageProperty)");
  senscord::Status status;
  CameraImageProperty camera_image = *property;
  status = adapter_.SetCameraImage(
                    camera_image.width,
                    camera_image.height,
                    camera_image.pixel_format);
  if (!status.ok()) {
    util_->SendEventError(status);
    return status;
  }

  return senscord::Status::OK();
}

senscord::Status LibcameraImageStreamSource::Get(
    const std::string &key,
    senscord::libcamera_image::CameraImageProperty *property) {
  SENSCORD_LOG_DEBUG_TAGGED("libcamera",
                            "LibcameraImageStreamSource::Get(libcamera_image::"
                            "CameraImageProperty)");
  senscord::Status status;
  CameraImageProperty camera_image;
  status = adapter_.GetCameraImage(
                    camera_image.width,
                    camera_image.height,
                    camera_image.stride_bytes,
                    camera_image.pixel_format);
  if (!status.ok()) {
    util_->SendEventError(status);
    return status;
  }

  *property = camera_image;
  return senscord::Status::OK();
}


bool LibcameraImageStreamSource::GetDeviceID(void) {
  std::string device_id_str = "";
  senscord::Status status;

  std::unique_lock<std::mutex> _lck(device_id_mutex_);

  /* Start */
  adapter_.GetImageProperty(&image_property_, AITRIOS_SENSOR_CHANNEL_ID_INFERENCE_RAW_IMAGE);
  status = adapter_.Configure(image_property_);
  if (!status.ok()) {
    SENSCORD_LOG_WARNING("Failed to configure camera");
    return false;
  }
  Set(senscord::kFrameRatePropertyKey, &framerate_property_);
  status = adapter_.Start();
  if (!status.ok()) {
    SENSCORD_LOG_WARNING("Failed to start camera");
    return false;
  }

  /* GetFrames */
  const int max_try_count = 20; /* kWaitOnGetFrames(4ms) * 20 = 80ms */
  int try_count = 0;
  while (max_try_count > try_count) {
    std::vector<FrameInfo> frames;
    adapter_.GetFrames(&frames, true);
    if (frames.empty()) {
      std::this_thread::sleep_for(std::chrono::microseconds(kWaitOnGetFrames));
      try_count++;
      continue;
    } else {
      /* If dry_run is enabled, an empty frame is got, so releasing is unnecessary. */
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
