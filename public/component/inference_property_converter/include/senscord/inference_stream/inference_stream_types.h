/*
 * SPDX-FileCopyrightText: 2017-2024 Sony Semiconductor Solutions Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef SENSCORD_INFERENCE_STREAM_INFERENCE_STREAM_TYPES_H_
#define SENSCORD_INFERENCE_STREAM_INFERENCE_STREAM_TYPES_H_

#include <stdint.h>

#include <string>

#include "senscord/serialize.h"

namespace senscord {

const char kStreamTypeInference[] = "inference";

const char kRawDataTypeInference[] = "inference_data";

/** Inference data format for user defined. */
const char kInferenceDataFormatUserDefined[] = "inference_user_defined";

/** Inference data format for tensor of float. */
const char kInferenceDataFormatTensor32Float[] = "inference_t32f";

enum CameraScalingPolicy {
  kCameraScalingPolicyAuto,
  kCameraScalingPolicySensitivity,
  kCameraScalingPolicyResolution,
};

enum CameraExposureMode {
  KCameraExposureModeAuto,
  KCameraExposureModeGainFix,
  KCameraExposureModeTimeFix,
  KCameraExposureModeManual,
  KCameraExposureModeHold,
};

enum RotationAngle {
  kRotationAngle0Deg,
  kRotationAngle90Deg,
  kRotationAngle180Deg,
  kRotationAngle270Deg,
};

enum CameraAntiFlickerMode {
  kCameraAntiFlickerModeOff,
  kCameraAntiFlickerModeAuto,
  kCameraAntiFlickerModeForce50Hz,
  kCameraAntiFlickerModeForce60Hz,
};

enum InferenceWhiteBalanceMode {
  kInferenceWhiteBalanceModeAuto,
  kInferenceWhiteBalanceModeManualPreset,
  kInferenceWhiteBalanceModeManualGain,
  kInferenceWhiteBalanceModeHold,
};

}  // namespace senscord

SENSCORD_SERIALIZE_ADD_ENUM(senscord::CameraScalingPolicy)
SENSCORD_SERIALIZE_ADD_ENUM(senscord::CameraExposureMode)
SENSCORD_SERIALIZE_ADD_ENUM(senscord::RotationAngle)
SENSCORD_SERIALIZE_ADD_ENUM(senscord::CameraAntiFlickerMode)
SENSCORD_SERIALIZE_ADD_ENUM(senscord::InferenceWhiteBalanceMode)

namespace senscord {

/**
 * ImageRotationProperty
 */
const char kImageRotationPropertyKey[] = "image_rotation_property";

/**
 * @brief Property ImageRotation.
 */
struct ImageRotationProperty {
  RotationAngle rotation_angle;

  SENSCORD_SERIALIZE_DEFINE(rotation_angle)
};

/**
 * InferenceProperty
 */
const char kInferencePropertyKey[] = "inference_property";

/** Length of the inference data type string. */
const size_t kInferenceDataTypeLength = 64;

/**
 * @brief Property InferenceProperty.
 */
struct InferenceProperty {
  char data_type[kInferenceDataTypeLength];

  SENSCORD_SERIALIZE_DEFINE(data_type)
};

/**
 * TensorShapesProperty
 */
const char kTensorShapesPropertyKey[] = "tensor_shapes_property";

/** Length of the shapes array string. */
const size_t kShapesArrayLength = 192;

/**
 * @brief Property TensorShapesProperty.
 */
struct TensorShapesProperty {
  uint32_t tensor_count;
  uint32_t shapes_array[kShapesArrayLength];

  SENSCORD_SERIALIZE_DEFINE(tensor_count, shapes_array)
};

/*
 * AIModelBundleProperty
 */
const char kAIModelBundleIdPropertyKey[] = "ai_model_bundle_id_property";

/** Max buffer size of the AI model bundle id string including null terminate */
const uint32_t kAIModelBundleIdLength = 128;

struct AIModelBundleIdProperty {
  char ai_model_bundle_id[kAIModelBundleIdLength];

  SENSCORD_SERIALIZE_DEFINE(ai_model_bundle_id)
};

/**
 * kAIModelIndexPropertyKey.
 */
const char kAIModelIndexPropertyKey[] = "ai_model_index_property";

/**
 * @brief Property AIModelIndex.
 */
struct AIModelIndexProperty {
  uint32_t ai_model_index;

  SENSCORD_SERIALIZE_DEFINE(ai_model_index)
};

/**
 * kPostProcessAvailablePropertyKey.
 */
const char kPostProcessAvailablePropertyKey[] =
    "post_process_available_property";

/**
 * @brief Property PostProcessAvailable.
 */
struct PostProcessAvailableProperty {
  bool is_available;

  SENSCORD_SERIALIZE_DEFINE(is_available)
};

/**
 * PostProcessParameterProperty.
 */
const char kPostProcessParameterPropertyKey[] =
    "post_process_parameter_property";

/** Length of post process parameter. */
const size_t kPostProcessParamSize = 256;

/**
 * @brief Property for the post process parameter.
 */
struct PostProcessParameterProperty {
  uint8_t param[kPostProcessParamSize];

  SENSCORD_SERIALIZE_DEFINE(param)
};

/**
 * CameraFrameRateProperty
 */
const char kCameraFrameRatePropertyKey[] = "camera_frame_rate_property";

/**
 * @brief Property CameraFrameRate.
 */
struct CameraFrameRateProperty {
  uint32_t num;
  uint32_t denom;

  SENSCORD_SERIALIZE_DEFINE(num, denom)
};

/**
 * CameraAutoExposureProperty
 */
const char kCameraAutoExposurePropertyKey[] = "camera_auto_exposure_property";

/**
 * @brief Property CameraAutoExposure.
 */
struct CameraAutoExposureProperty {
  uint32_t max_exposure_time;
  uint32_t min_exposure_time;
  float max_gain;
  uint32_t convergence_speed;

  SENSCORD_SERIALIZE_DEFINE(max_exposure_time, min_exposure_time, max_gain,
                            convergence_speed)
};

/**
 * @brief Property CameraEvCompensationProperty.
 */
const char kCameraEvCompensationPropertyKey[] =
    "camera_ev_compensation_property";

/**
 * @brief Property CameraEvCompensation.
 */
struct CameraEvCompensationProperty {
  float ev_compensation;

  SENSCORD_SERIALIZE_DEFINE(ev_compensation)
};

const char kCameraAntiFlickerModePropertyKey[] =
    "camera_anti_flicker_mode_property";

struct CameraAntiFlickerModeProperty {
  CameraAntiFlickerMode anti_flicker_mode;

  SENSCORD_SERIALIZE_DEFINE(anti_flicker_mode)
};

/**
 * CameraImageSizeProperty
 */
const char kCameraImageSizePropertyKey[] = "camera_image_size_property";

/**
 * @brief Property CameraImageSize.
 */
struct CameraImageSizeProperty {
  uint32_t width;
  uint32_t height;
  CameraScalingPolicy scaling_policy;

  SENSCORD_SERIALIZE_DEFINE(width, height, scaling_policy)
};

/**
 * CameraImageFlipProperty
 */
const char kCameraImageFlipPropertyKey[] = "camera_image_flip_property";

/**
 * @brief Property CameraImageFlip.
 */
struct CameraImageFlipProperty {
  bool flip_horizontal;
  bool flip_vertical;

  SENSCORD_SERIALIZE_DEFINE(flip_horizontal, flip_vertical)
};

/**
 * kCameraDigitalZoomProperty
 */
const char kCameraDigitalZoomPropertyKey[] = "camera_digital_zoom_property";

/**
 * @brief Property CameraDigitalZoom.
 */
struct CameraDigitalZoomProperty {
  float magnification;

  SENSCORD_SERIALIZE_DEFINE(magnification)
};

/**
 * kCameraExposureModeProperty
 */
const char kCameraExposureModePropertyKey[] = "camera_exposure_mode_property";

/**
 * @brief Property CameraExposureMode.
 */
struct CameraExposureModeProperty {
  CameraExposureMode mode;

  SENSCORD_SERIALIZE_DEFINE(mode)
};

/**
 * kCameraManualExposureProperty
 */
const char kCameraManualExposurePropertyKey[] =
    "camera_manual_exposure_property";

/**
 * @brief Property CameraManualExposure.
 */
struct CameraManualExposureProperty {
  uint32_t exposure_time;
  float gain;

  SENSCORD_SERIALIZE_DEFINE(exposure_time, gain)
};

/**
 * kWhiteBalanceModeProeperty.
 */
const char kWhiteBalanceModePropertyKey[] = "white_balance_mode_property";

/**
 * @brief Property WhiteBalanceMode.
 */
struct WhiteBalanceModeProperty {
  InferenceWhiteBalanceMode mode;

  SENSCORD_SERIALIZE_DEFINE(mode)
};

/**
 * kAutoWhiteBalanceProperty.
 */
const char kAutoWhiteBalancePropertyKey[] = "auto_white_balance_property";

/**
 * @brief Property AutoWhiteBalance.
 */
struct AutoWhiteBalanceProperty {
  uint32_t convergence_speed;

  SENSCORD_SERIALIZE_DEFINE(convergence_speed)
};

/**
 * kManualWhiteBalancePresetProperty.
 */
const char kManualWhiteBalancePresetPropertyKey[] =
    "manual_white_balance_preset_property";

/**
 * @brief Property ManualWhiteBalancePreset.
 */
struct ManualWhiteBalancePresetProperty {
  uint32_t color_temperature;

  SENSCORD_SERIALIZE_DEFINE(color_temperature)
};

/**
 * kManualWhiteBalanceGainProperty.
 */
const char kManualWhiteBalanceGainPropertyKey[] =
    "manual_white_balance_gain_property";

/**
 * @brief Property ManualWhiteBalanceGain.
 */

struct WhiteBalanceGains {
  float red;
  float blue;

  SENSCORD_SERIALIZE_DEFINE(red, blue)
};

struct ManualWhiteBalanceGainProperty {
  struct WhiteBalanceGains gains;

  SENSCORD_SERIALIZE_DEFINE(gains)
};

/**
 * kInfoStringProperty.
 */
const char kInfoStringPropertyKey[] = "info_string_property";

/** Length of InfoString */
const size_t kInfoStringLength = 128;

/** Setting the category that member of InfoStringProperty */
/* for AITRIOS hardware info */
#define SENSCORD_INFO_STRING_SENSOR_NAME    0x00000000
#define SENSCORD_INFO_STRING_SENSOR_ID      0x00000001
#define SENSCORD_INFO_STRING_KEY_GENERATION 0x00000002
/* for AITRIOS sensor info */
#define SENSCORD_INFO_STRING_FIRMWARE_VERSION 0x00010000
#define SENSCORD_INFO_STRING_LOADER_VERSION   0x00010001
#define SENSCORD_INFO_STRING_AI_MODEL_VERSION 0x00010002
/* for vendor extension */
#define SENSCORD_INFO_STRING_VENDOR_BASE     0x80000000
#define SENSCORD_INFO_STRING_AIISP_DEVICE_ID 0x80000101

/**
 * @brief Property InfoString.
 */
struct InfoStringProperty {
  uint32_t category;
  char info[kInfoStringLength];

  SENSCORD_SERIALIZE_DEFINE(category, info)
};

/**
 * TemperatureEnableProperty.
 */
const char kTemperatureEnablePropertyKey[] = "temperature_enable_property";

/** Length of temperature enable list size */
const uint32_t kTemperatureListMax = 64;

/**
 * @brief Temperature enable.
 */
struct TemperatureEnable {
  uint32_t sensor_id;
  bool enable;

  SENSCORD_SERIALIZE_DEFINE(sensor_id, enable)
};

/**
 * @brief Property for temperature enable.
 */
struct TemperatureEnableProperty {
  uint32_t count;
  struct TemperatureEnable temperatures[kTemperatureListMax];

  SENSCORD_SERIALIZE_DEFINE(count, temperatures)
};

/**
 * InputDataTypeProperty.
 */
const char kInputDataTypePropertyKey[] = "input_data_type_property";

/** Length of channel list enable size */
const uint32_t kChannelListMax = 8;

/**
 * @brief Property for input data type.
 */
struct InputDataTypeProperty {
  uint32_t count;
  uint32_t channels[kChannelListMax];

  SENSCORD_SERIALIZE_DEFINE(count, channels)
};

}  // namespace senscord

#endif  // SENSCORD_INFERENCE_STREAM_INFERENCE_STREAM_TYPES_H_
