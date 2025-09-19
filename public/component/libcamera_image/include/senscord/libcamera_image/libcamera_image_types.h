/*
 * SPDX-FileCopyrightText: 2023 Sony Semiconductor Solutions Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef SENSCORD_LIBCAMERA_IMAGE_LIBCAMERA_IMAGE_TYPES_H_
#define SENSCORD_LIBCAMERA_IMAGE_LIBCAMERA_IMAGE_TYPES_H_

#include <string>

#include "AnyValue.h"
#include "senscord/serialize.h"

namespace senscord {
namespace libcamera_image {

/**
 * Property key.
 */
constexpr char kLibcameraAccessPropertyKey[] = "LibcameraAccessProperty";
constexpr char kLibcameraDeviceEnumerationPropertyKey[] =
    "LibcameraDeviceEnumerationProperty";
constexpr char kLibcameraImageRotationPropertyKey[] = "image_rotation_property";
constexpr char kLibcameraCameraImageFlipPropertyKey[] = "camera_image_flip_property";
constexpr char kLibcameraAIModelBundleIdPropertyKey[] = "ai_model_bundle_id_property";
constexpr char kLibcameraInputDataTypePropertyKey[] = "input_data_type_property";
constexpr char kLibcameraTensorShapesPropertyKey[] = "tensor_shapes_property";
constexpr char kLibcameraInfoStringPropertyKey[] = "info_string_property";
constexpr char kPostProcessAvailablePropertyKey[] =
    "post_process_available_property";
constexpr char kLibcameraCameraExposureModePropertyKey[] = "camera_exposure_mode_property";
constexpr char kLibcameraCameraAutoExposurePropertyKey[] = "camera_auto_exposure_property";
constexpr char kLibcameraCameraEvCompensationPropertyKey[] = "camera_ev_compensation_property";
constexpr char kLibcameraCameraAntiFlickerModePropertyKey[] =
    "camera_anti_flicker_mode_property";
constexpr char kLibcameraCameraAutoExposureMeteringPropertykey[] =
    "camera_auto_exposure_metering_property";
constexpr char kLibcameraCameraManualExposurePropertykey[] = "camera_manual_exposure_property";
constexpr char kLibcameraTemperaturePropertyKey[] = "temperature_property";
constexpr char kLibcameraCameraImageSizePropertykey[] = "camera_image_size_property";
constexpr char kLibcameraCameraFrameRatePropertykey[] = "camera_frame_rate_property";
constexpr char kLibcameraImageCropPropertyKey[] = "image_crop_property";

/**
 * @brief libcamera access property.
 */
struct AccessProperty {
  typedef enum { kProperty, kControl } Type;
  int type;

  std::string id;
  AnyValue value;

  SENSCORD_SERIALIZE_DEFINE(type, id, value)
};

/**
 * @brief device enumeration by libcamera
 */
struct DeviceEnumerationProperty {
  typedef struct {
    std::string id;
    std::string name;

    SENSCORD_SERIALIZE_DEFINE(id, name)
  } LibcameraDevice;

  std::vector<LibcameraDevice> devices;

  SENSCORD_SERIALIZE_DEFINE(devices)
};

enum SensorRotationAngle{
  SENSOR_ROTATION_ANGLE_0_DEG,
  SENSOR_ROTATION_ANGLE_90_DEG,
  SENSOR_ROTATION_ANGLE_180_DEG,
  SENSOR_ROTATION_ANGLE_270_DEG,
};

struct ImageRotationProperty {
  int rotation_angle;

  SENSCORD_SERIALIZE_DEFINE(rotation_angle)
};

struct CameraImageFlipProperty {
  bool flip_horizontal;
  bool flip_vertical;

  SENSCORD_SERIALIZE_DEFINE(flip_horizontal, flip_vertical)
};

/** Max buffer size of the AI model bundle id string including null terminate */
const uint32_t kAIModelBundleIdLength = 128;

struct AIModelBundleIdProperty {
  char ai_model_bundle_id[kAIModelBundleIdLength];

  SENSCORD_SERIALIZE_DEFINE(ai_model_bundle_id)
};

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
/** Length of InfoString */
const size_t kInfoStringLength = 128;
struct InfoStringProperty {
  uint32_t category;
  char info[kInfoStringLength];

  SENSCORD_SERIALIZE_DEFINE(category, info)
};

struct PostProcessAvailableProperty {
  bool is_available;

  SENSCORD_SERIALIZE_DEFINE(is_available)
};

/**
 * @brief Property CameraAutoExposureProperty.
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
 * @brief Property CameraExposureModeProperty.
 */
enum CameraExposureMode {
  KCameraExposureModeAuto,
  KCameraExposureModeGainFix,
  KCameraExposureModeTimeFix,
  KCameraExposureModeManual,
  KCameraExposureModeHold,
};

struct CameraExposureModeProperty {
  CameraExposureMode mode;

  SENSCORD_SERIALIZE_DEFINE(mode)
};

/**
 * @brief Property CameraEvCompensation.
 */
struct CameraEvCompensationProperty {
  float ev_compensation;

  SENSCORD_SERIALIZE_DEFINE(ev_compensation)
};

/**
 * @brief Property CameraAntiFlickerModeProperty.
 */
enum CameraAntiFlickerMode {
  kCameraAntiFlickerModeOff,
  kCameraAntiFlickerModeAuto,
  kCameraAntiFlickerModeForce50Hz,
  kCameraAntiFlickerModeForce60Hz,
};

struct CameraAntiFlickerModeProperty {
  CameraAntiFlickerMode anti_flicker_mode;

  SENSCORD_SERIALIZE_DEFINE(anti_flicker_mode)
};

/**
 * @brief Property CameraAutoExposureMeteringProperty.
 */
enum CameraAutoExposureMeteringMode {
  kCameraAutoExposureMeteringModeFullScreen,
  kCameraAutoExposureMeteringModeUserWindow,
};

struct CameraAutoExposureMeteringWindow {
  uint32_t top;
  uint32_t left;
  uint32_t bottom;
  uint32_t right;

  SENSCORD_SERIALIZE_DEFINE(top, left, bottom, right)
};

struct CameraAutoExposureMeteringProperty {
  CameraAutoExposureMeteringMode mode;
  CameraAutoExposureMeteringWindow window;

  SENSCORD_SERIALIZE_DEFINE(mode, window)
};

/**
 * @brief Property CameraManualExposureProperty.
 */
struct CameraManualExposureProperty {
  uint32_t exposure_time;
  float gain;

  SENSCORD_SERIALIZE_DEFINE(exposure_time, gain)
};

/**
 * @brief Property for the temperature information.
 */
struct TemperatureInfo {
  float temperature;         /**< Temperature data. */
  std::string description;   /**< Description of sensor. */

  SENSCORD_SERIALIZE_DEFINE(temperature, description)
};

/**
 * @brief Property for the temperature.
 */
struct CameraTemperatureProperty {
  /** Information for each temperature sensor. (Key = Sensor id) */
  std::map<uint32_t, TemperatureInfo> temperatures;

  SENSCORD_SERIALIZE_DEFINE(temperatures)
};

/**
 * @brief Property CameraImageSize.
 */
enum CameraScalingPolicy {
  kCameraScalingPolicyAuto,
  kCameraScalingPolicySensitivity,
  kCameraScalingPolicyResolution,
};

struct CameraImageSizeProperty {
  uint32_t width;
  uint32_t height;
  CameraScalingPolicy scaling_policy;

  SENSCORD_SERIALIZE_DEFINE(width, height, scaling_policy)
};

/**
 * @brief Property CameraFrameRate.
 */
struct CameraFrameRateProperty {
  uint32_t num;
  uint32_t denom;

  SENSCORD_SERIALIZE_DEFINE(num, denom)
};

}  // namespace libcamera_image
}  // namespace senscord

SENSCORD_SERIALIZE_ADD_ENUM(senscord::libcamera_image::CameraExposureMode)
SENSCORD_SERIALIZE_ADD_ENUM(senscord::libcamera_image::CameraAntiFlickerMode)
SENSCORD_SERIALIZE_ADD_ENUM(senscord::libcamera_image::CameraAutoExposureMeteringMode)
SENSCORD_SERIALIZE_ADD_ENUM(senscord::libcamera_image::CameraScalingPolicy)

#endif
