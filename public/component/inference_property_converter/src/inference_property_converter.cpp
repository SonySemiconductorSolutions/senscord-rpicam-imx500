/*
 * SPDX-FileCopyrightText: 2024 Sony Semiconductor Solutions Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "inference_property_converter.h"

#include <senscord/osal.h>

#include <algorithm>
#include <cstring>

#define ARRAY_SIZE(x) (sizeof(x) / sizeof(x[0]))

#define AddType(c_type, cpp_type) \
  collector->Add<c_type, cpp_type>(k##cpp_type##Key, this);

namespace senscord {

/**
 * @brief Helper function that copies a char array to char array.
 * @param[out] dst Char array of destination.
 * @param[in] src Char array of copy source.
 */
template <size_t dst_size>
static void StrCpyToCharArray(char (&dst)[dst_size], const char *src) {
  if (dst_size) {
    const size_t src_size = strnlen(src, dst_size - 1);
    memcpy(dst, src, src_size);
    dst[src_size] = '\0';
  }
}

/**
 * @brief Initialize the converter library.
 * @param[in] collector  Converter collector.
 * @return Status object.
 */
Status InferencePropertyConverterLibrary::Init(ConverterCollector* collector) {
  AddType(senscord_image_rotation_property_t, ImageRotationProperty);
  AddType(senscord_ai_model_bundle_id_property_t, AIModelBundleIdProperty);
  AddType(senscord_ai_model_index_property_t, AIModelIndexProperty);
  AddType(senscord_inference_property_t, InferenceProperty);
  AddType(senscord_tensor_shapes_property_t, TensorShapesProperty);
  AddType(senscord_post_process_available_property_t,
          PostProcessAvailableProperty);
  AddType(senscord_post_process_parameter_property_t,
          PostProcessParameterProperty);
  AddType(senscord_camera_frame_rate_property_t, CameraFrameRateProperty);
  AddType(senscord_camera_auto_exposure_property_t, CameraAutoExposureProperty);
  AddType(senscord_camera_ev_compensation_property_t,
          CameraEvCompensationProperty);
  AddType(senscord_camera_anti_flicker_mode_property_t,
          CameraAntiFlickerModeProperty);
  AddType(senscord_camera_image_size_property_t, CameraImageSizeProperty);
  AddType(senscord_camera_image_flip_property_t, CameraImageFlipProperty);
  AddType(senscord_camera_digital_zoom_property_t, CameraDigitalZoomProperty);
  AddType(senscord_camera_exposure_mode_property_t, CameraExposureModeProperty);
  AddType(senscord_camera_manual_exposure_property_t,
          CameraManualExposureProperty);
  AddType(senscord_white_balance_mode_property_t, WhiteBalanceModeProperty);
  AddType(senscord_auto_white_balance_property_t, AutoWhiteBalanceProperty);
  AddType(senscord_manual_white_balance_preset_property_t,
          ManualWhiteBalancePresetProperty);
  AddType(senscord_manual_white_balance_gain_property_t,
          ManualWhiteBalanceGainProperty);
  AddType(senscord_info_string_property_t, InfoStringProperty);
  AddType(senscord_temperature_enable_property_t, TemperatureEnableProperty);
  AddType(senscord_input_data_type_property_t, InputDataTypeProperty);
  return Status::OK();
}

// ImageRotationProperty
Status InferencePropertyConverterLibrary::c_to_cxx(
    const senscord_image_rotation_property_t& src, ImageRotationProperty* dst) {
  dst->rotation_angle = static_cast<RotationAngle>(src.rotation_angle);
  return Status::OK();
}

Status InferencePropertyConverterLibrary::cxx_to_c(
    const ImageRotationProperty& src, senscord_image_rotation_property_t* dst) {
  dst->rotation_angle =
      static_cast<senscord_rotation_angle_t>(src.rotation_angle);
  return Status::OK();
}

// InferenceProperty
Status InferencePropertyConverterLibrary::c_to_cxx(
    const senscord_inference_property_t& src, InferenceProperty* dst) {
  StrCpyToCharArray(dst->data_type, src.data_type);
  return Status::OK();
}

Status InferencePropertyConverterLibrary::cxx_to_c(
    const InferenceProperty& src, senscord_inference_property_t* dst) {
  StrCpyToCharArray(dst->data_type, src.data_type);
  return Status::OK();
}

// TensorShapesProperty
Status InferencePropertyConverterLibrary::c_to_cxx(
    const senscord_tensor_shapes_property_t& src, TensorShapesProperty* dst) {
  dst->tensor_count = src.tensor_count;
  std::copy(src.shapes_array, src.shapes_array + ARRAY_SIZE(src.shapes_array),
            dst->shapes_array);
  return Status::OK();
}

Status InferencePropertyConverterLibrary::cxx_to_c(
    const TensorShapesProperty& src, senscord_tensor_shapes_property_t* dst) {
  dst->tensor_count = src.tensor_count;
  std::copy(src.shapes_array, src.shapes_array + ARRAY_SIZE(src.shapes_array),
            dst->shapes_array);
  return Status::OK();
}

// AIModelBundleIdProperty
Status InferencePropertyConverterLibrary::c_to_cxx(
    const senscord_ai_model_bundle_id_property_t& src,
    AIModelBundleIdProperty* dst) {
  StrCpyToCharArray(dst->ai_model_bundle_id, src.ai_model_bundle_id);
  return Status::OK();
}

Status InferencePropertyConverterLibrary::cxx_to_c(
    const AIModelBundleIdProperty& src,
    senscord_ai_model_bundle_id_property_t* dst) {
  StrCpyToCharArray(dst->ai_model_bundle_id, src.ai_model_bundle_id);
  return Status::OK();
}

// AIModelIndexProperty
Status InferencePropertyConverterLibrary::c_to_cxx(
    const senscord_ai_model_index_property_t& src, AIModelIndexProperty* dst) {
  dst->ai_model_index = src.ai_model_index;
  return Status::OK();
}

Status InferencePropertyConverterLibrary::cxx_to_c(
    const AIModelIndexProperty& src, senscord_ai_model_index_property_t* dst) {
  dst->ai_model_index = src.ai_model_index;
  return Status::OK();
}

// PostProcessAvailableProperty
Status InferencePropertyConverterLibrary::c_to_cxx(
    const senscord_post_process_available_property_t& src,
    PostProcessAvailableProperty* dst) {
  dst->is_available = src.is_aveilable;
  return Status::OK();
}

Status InferencePropertyConverterLibrary::cxx_to_c(
    const PostProcessAvailableProperty& src,
    senscord_post_process_available_property_t* dst) {
  dst->is_aveilable = src.is_available;
  return Status::OK();
}

// PostProcessParameterProperty
Status InferencePropertyConverterLibrary::c_to_cxx(
    const senscord_post_process_parameter_property_t& src,
    PostProcessParameterProperty* dst) {
  memcpy(dst->param, src.param, sizeof(src.param));
  return Status::OK();
}

Status InferencePropertyConverterLibrary::cxx_to_c(
    const PostProcessParameterProperty& src,
    senscord_post_process_parameter_property_t* dst) {
  memcpy(dst->param, src.param, sizeof(src.param));
  return Status::OK();
}

// CameraFrameRateProperty
Status InferencePropertyConverterLibrary::c_to_cxx(
    const senscord_camera_frame_rate_property_t& src,
    CameraFrameRateProperty* dst) {
  dst->num   = src.num;
  dst->denom = src.denom;
  return Status::OK();
}

Status InferencePropertyConverterLibrary::cxx_to_c(
    const CameraFrameRateProperty& src,
    senscord_camera_frame_rate_property_t* dst) {
  dst->num   = src.num;
  dst->denom = src.denom;
  return Status::OK();
}

// CameraAutoExposureProperty
Status InferencePropertyConverterLibrary::c_to_cxx(
    const senscord_camera_auto_exposure_property_t& src,
    CameraAutoExposureProperty* dst) {
  dst->convergence_speed = src.convergence_speed;
  dst->max_exposure_time = src.max_exposure_time;
  dst->max_gain          = src.max_gain;
  dst->min_exposure_time = src.min_exposure_time;
  return Status::OK();
}

Status InferencePropertyConverterLibrary::cxx_to_c(
    const CameraAutoExposureProperty& src,
    senscord_camera_auto_exposure_property_t* dst) {
  dst->convergence_speed = src.convergence_speed;
  dst->max_exposure_time = src.max_exposure_time;
  dst->max_gain          = src.max_gain;
  dst->min_exposure_time = src.min_exposure_time;
  return Status::OK();
}

// CameraEvCompensationProperty
Status InferencePropertyConverterLibrary::c_to_cxx(
    const senscord_camera_ev_compensation_property_t& src,
    CameraEvCompensationProperty* dst) {
  dst->ev_compensation = src.ev_compensation;
  return Status::OK();
}

Status InferencePropertyConverterLibrary::cxx_to_c(
    const CameraEvCompensationProperty& src,
    senscord_camera_ev_compensation_property_t* dst) {
  dst->ev_compensation = src.ev_compensation;
  return Status::OK();
}

// CameraAntiFlickerModeProperty
Status InferencePropertyConverterLibrary::c_to_cxx(
    const senscord_camera_anti_flicker_mode_property_t& src,
    CameraAntiFlickerModeProperty* dst) {
  dst->anti_flicker_mode =
      static_cast<CameraAntiFlickerMode>(src.anti_flicker_mode);
  return Status::OK();
}

Status InferencePropertyConverterLibrary::cxx_to_c(
    const CameraAntiFlickerModeProperty& src,
    senscord_camera_anti_flicker_mode_property_t* dst) {
  dst->anti_flicker_mode =
      static_cast<senscord_camera_anti_flicker_mode>(src.anti_flicker_mode);
  return Status::OK();
}

// CameraImageSizeProperty
Status InferencePropertyConverterLibrary::c_to_cxx(
    const senscord_camera_image_size_property_t& src,
    CameraImageSizeProperty* dst) {
  dst->height         = src.height;
  dst->scaling_policy = static_cast<CameraScalingPolicy>(src.scaling_policy);
  dst->width          = src.width;
  return Status::OK();
}

Status InferencePropertyConverterLibrary::cxx_to_c(
    const CameraImageSizeProperty& src,
    senscord_camera_image_size_property_t* dst) {
  dst->height = src.height;
  dst->scaling_policy =
      static_cast<senscord_camera_scaling_policy_t>(src.scaling_policy);
  dst->width = src.width;
  return Status::OK();
}

// CameraImageFlipProperty
Status InferencePropertyConverterLibrary::c_to_cxx(
    const senscord_camera_image_flip_property_t& src,
    CameraImageFlipProperty* dst) {
  dst->flip_horizontal = src.flip_horizontal;
  dst->flip_vertical   = src.flip_vertical;
  return Status::OK();
}

Status InferencePropertyConverterLibrary::cxx_to_c(
    const CameraImageFlipProperty& src,
    senscord_camera_image_flip_property_t* dst) {
  dst->flip_horizontal = src.flip_horizontal;
  dst->flip_vertical   = src.flip_vertical;
  return Status::OK();
}

// CameraDigitalZoomProperty
Status InferencePropertyConverterLibrary::c_to_cxx(
    const senscord_camera_digital_zoom_property_t& src,
    CameraDigitalZoomProperty* dst) {
  dst->magnification = src.magnification;
  return Status::OK();
}

Status InferencePropertyConverterLibrary::cxx_to_c(
    const CameraDigitalZoomProperty& src,
    senscord_camera_digital_zoom_property_t* dst) {
  dst->magnification = src.magnification;
  return Status::OK();
}

// CameraExposureModeProperty
Status InferencePropertyConverterLibrary::c_to_cxx(
    const senscord_camera_exposure_mode_property_t& src,
    CameraExposureModeProperty* dst) {
  dst->mode = static_cast<CameraExposureMode>(src.mode);
  return Status::OK();
}

Status InferencePropertyConverterLibrary::cxx_to_c(
    const CameraExposureModeProperty& src,
    senscord_camera_exposure_mode_property_t* dst) {
  dst->mode = static_cast<senscord_camera_exposure_mode_t>(src.mode);
  return Status::OK();
}

// CameraManualExposureProperty
Status InferencePropertyConverterLibrary::c_to_cxx(
    const senscord_camera_manual_exposure_property_t& src,
    CameraManualExposureProperty* dst) {
  dst->exposure_time = src.exposure_time;
  dst->gain          = src.gain;
  return Status::OK();
}

Status InferencePropertyConverterLibrary::cxx_to_c(
    const CameraManualExposureProperty& src,
    senscord_camera_manual_exposure_property_t* dst) {
  dst->exposure_time = src.exposure_time;
  dst->gain          = src.gain;
  return Status::OK();
}

// WhiteBalanceModeProperty
Status InferencePropertyConverterLibrary::c_to_cxx(
    const senscord_white_balance_mode_property_t& src,
    WhiteBalanceModeProperty* dst) {
  dst->mode = static_cast<InferenceWhiteBalanceMode>(src.mode);
  return Status::OK();
}

Status InferencePropertyConverterLibrary::cxx_to_c(
    const WhiteBalanceModeProperty& src,
    senscord_white_balance_mode_property_t* dst) {
  dst->mode = static_cast<senscord_inference_white_balance_mode_t>(src.mode);
  return Status::OK();
}

// AutoWhiteBalanceProperty
Status InferencePropertyConverterLibrary::c_to_cxx(
    const senscord_auto_white_balance_property_t& src,
    AutoWhiteBalanceProperty* dst) {
  dst->convergence_speed = src.convergence_speed;
  return Status::OK();
}

Status InferencePropertyConverterLibrary::cxx_to_c(
    const AutoWhiteBalanceProperty& src,
    senscord_auto_white_balance_property_t* dst) {
  dst->convergence_speed = src.convergence_speed;
  return Status::OK();
}

// ManualWhiteBalancePresetProperty
Status InferencePropertyConverterLibrary::c_to_cxx(
    const senscord_manual_white_balance_preset_property_t& src,
    ManualWhiteBalancePresetProperty* dst) {
  dst->color_temperature = src.color_temperature;
  return Status::OK();
}

Status InferencePropertyConverterLibrary::cxx_to_c(
    const ManualWhiteBalancePresetProperty& src,
    senscord_manual_white_balance_preset_property_t* dst) {
  dst->color_temperature = src.color_temperature;
  return Status::OK();
}

// InfoStringProperty
Status InferencePropertyConverterLibrary::c_to_cxx(
    const senscord_info_string_property_t& src, InfoStringProperty* dst) {
  dst->category = src.category;
  StrCpyToCharArray(dst->info, src.info);

  return Status::OK();
}

Status InferencePropertyConverterLibrary::cxx_to_c(
    const InfoStringProperty& src, senscord_info_string_property_t* dst) {
  dst->category = src.category;
  StrCpyToCharArray(dst->info, src.info);
  return Status::OK();
}

// ManualWhiteBalanceGainProperty
Status InferencePropertyConverterLibrary::c_to_cxx(
    const senscord_manual_white_balance_gain_property_t& src,
    ManualWhiteBalanceGainProperty* dst) {
  dst->gains.red  = src.gains.red;
  dst->gains.blue = src.gains.blue;
  return Status::OK();
}

Status InferencePropertyConverterLibrary::cxx_to_c(
    const ManualWhiteBalanceGainProperty& src,
    senscord_manual_white_balance_gain_property_t* dst) {
  dst->gains.red  = src.gains.red;
  dst->gains.blue = src.gains.blue;
  return Status::OK();
}

// TemperatureEnableProperty
Status InferencePropertyConverterLibrary::c_to_cxx(
    const senscord_temperature_enable_property_t& src,
    TemperatureEnableProperty* dst) {
  dst->count = std::min(src.count, kTemperatureListMax);
  for (uint32_t i = 0; i < dst->count; ++i) {
    dst->temperatures[i].sensor_id = src.temperatures[i].sensor_id;
    dst->temperatures[i].enable    = src.temperatures[i].enable;
  }
  return Status::OK();
}

Status InferencePropertyConverterLibrary::cxx_to_c(
    const TemperatureEnableProperty& src,
    senscord_temperature_enable_property_t* dst) {
  dst->count =
      std::min(src.count, static_cast<uint32_t>(SENSCORD_TEMPERATURE_LIST_MAX));
  for (uint32_t i = 0; i < dst->count; ++i) {
    dst->temperatures[i].sensor_id = src.temperatures[i].sensor_id;
    dst->temperatures[i].enable    = src.temperatures[i].enable;
  }
  return Status::OK();
}

// InputDataTypeProperty
Status InferencePropertyConverterLibrary::c_to_cxx(
    const senscord_input_data_type_property_t& src,
    InputDataTypeProperty* dst) {
  dst->count = std::min(src.count, kChannelListMax);
  for (uint32_t i = 0; i < dst->count; ++i) {
    dst->channels[i] = src.channels[i];
  }
  return Status::OK();
}

Status InferencePropertyConverterLibrary::cxx_to_c(
    const InputDataTypeProperty& src,
    senscord_input_data_type_property_t* dst) {
  dst->count =
      std::min(src.count, static_cast<uint32_t>(SENSCORD_CHANNEL_LIST_MAX));
  for (uint32_t i = 0; i < dst->count; ++i) {
    dst->channels[i] = src.channels[i];
  }
  return Status::OK();
}

}  // namespace senscord

// export register function.
SENSCORD_REGISTER_CONVERTER(senscord::InferencePropertyConverterLibrary)
