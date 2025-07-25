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

;;
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

}  // namespace libcamera_image
}  // namespace senscord

#endif
