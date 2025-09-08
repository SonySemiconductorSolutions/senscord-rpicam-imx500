/*
 * SPDX-FileCopyrightText: 2024 Sony Semiconductor Solutions Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef SENSCORD_INFERENCE_PROPERTY_CONVERTER_INFERENCE_PROPERTY_CONVERTER_H_
#define SENSCORD_INFERENCE_PROPERTY_CONVERTER_INFERENCE_PROPERTY_CONVERTER_H_

#include "senscord/c_api/property_c_types.h"
#include "senscord/develop/converter.h"
#include "senscord/inference_stream/c_api/property_c_types.h"
#include "senscord/inference_stream/inference_stream_types.h"

// Macro for defining conversion function.
#define CONVERTER_FUNC(c, cxx)                               \
  virtual senscord::Status c_to_cxx(const c& src, cxx* dst); \
  virtual senscord::Status cxx_to_c(const cxx& src, c* dst)

namespace senscord {

/**
 * @brief Image property converter library.
 */
class InferencePropertyConverterLibrary
    : public ConverterLibrary,
      public StructConverterC<senscord_image_rotation_property_t,
                              ImageRotationProperty>,
      public StructConverterC<senscord_ai_model_bundle_id_property_t,
                              AIModelBundleIdProperty>,
      public StructConverterC<senscord_ai_model_index_property_t,
                              AIModelIndexProperty>,
      public StructConverterC<senscord_inference_property_t, InferenceProperty>,
      public StructConverterC<senscord_tensor_shapes_property_t,
                              TensorShapesProperty>,
      public StructConverterC<senscord_post_process_available_property_t,
                              PostProcessAvailableProperty>,
      public StructConverterC<senscord_post_process_parameter_property_t,
                              PostProcessParameterProperty>,
      public StructConverterC<senscord_camera_frame_rate_property_t,
                              CameraFrameRateProperty>,
      public StructConverterC<senscord_camera_auto_exposure_property_t,
                              CameraAutoExposureProperty>,
      public StructConverterC<senscord_camera_ev_compensation_property_t,
                              CameraEvCompensationProperty>,
      public StructConverterC<senscord_camera_anti_flicker_mode_property_t,
                              CameraAntiFlickerModeProperty>,
      public StructConverterC<senscord_camera_image_size_property_t,
                              CameraImageSizeProperty>,
      public StructConverterC<senscord_camera_image_flip_property_t,
                              CameraImageFlipProperty>,
      public StructConverterC<senscord_camera_digital_zoom_property_t,
                              CameraDigitalZoomProperty>,
      public StructConverterC<senscord_camera_exposure_mode_property_t,
                              CameraExposureModeProperty>,
      public StructConverterC<senscord_camera_manual_exposure_property_t,
                              CameraManualExposureProperty>,
      public StructConverterC<senscord_white_balance_mode_property_t,
                              WhiteBalanceModeProperty>,
      public StructConverterC<senscord_auto_white_balance_property_t,
                              AutoWhiteBalanceProperty>,
      public StructConverterC<senscord_manual_white_balance_preset_property_t,
                              ManualWhiteBalancePresetProperty>,
      public StructConverterC<senscord_manual_white_balance_gain_property_t,
                              ManualWhiteBalanceGainProperty>,
      public StructConverterC<senscord_info_string_property_t,
                              InfoStringProperty>,
      public StructConverterC<senscord_temperature_enable_property_t,
                              TemperatureEnableProperty>,
      public StructConverterC<senscord_input_data_type_property_t,
                              InputDataTypeProperty>,
      public StructConverterC<senscord_camera_auto_exposure_metering_property_t,
                              CameraAutoExposureMeteringProperty> {
 public:
  virtual ~InferencePropertyConverterLibrary() {}

  /**
   * @brief Initialize the converter library.
   * @param[in] collector  Converter collector.
   * @return Status object.
   */
  virtual Status Init(ConverterCollector* collector);

 private:
  CONVERTER_FUNC(senscord_image_rotation_property_t, ImageRotationProperty);
  CONVERTER_FUNC(senscord_ai_model_bundle_id_property_t,
                 AIModelBundleIdProperty);
  CONVERTER_FUNC(senscord_ai_model_index_property_t, AIModelIndexProperty);
  CONVERTER_FUNC(senscord_inference_property_t, InferenceProperty);
  CONVERTER_FUNC(senscord_tensor_shapes_property_t, TensorShapesProperty);
  CONVERTER_FUNC(senscord_post_process_available_property_t,
                 PostProcessAvailableProperty);
  CONVERTER_FUNC(senscord_post_process_parameter_property_t,
                 PostProcessParameterProperty);
  CONVERTER_FUNC(senscord_camera_frame_rate_property_t,
                 CameraFrameRateProperty);
  CONVERTER_FUNC(senscord_camera_auto_exposure_property_t,
                 CameraAutoExposureProperty);
  CONVERTER_FUNC(senscord_camera_ev_compensation_property_t,
                 CameraEvCompensationProperty);
  CONVERTER_FUNC(senscord_camera_anti_flicker_mode_property_t,
                 CameraAntiFlickerModeProperty);
  CONVERTER_FUNC(senscord_camera_image_size_property_t,
                 CameraImageSizeProperty);
  CONVERTER_FUNC(senscord_camera_image_flip_property_t,
                 CameraImageFlipProperty);
  CONVERTER_FUNC(senscord_camera_digital_zoom_property_t,
                 CameraDigitalZoomProperty);
  CONVERTER_FUNC(senscord_camera_exposure_mode_property_t,
                 CameraExposureModeProperty);
  CONVERTER_FUNC(senscord_camera_manual_exposure_property_t,
                 CameraManualExposureProperty);
  CONVERTER_FUNC(senscord_white_balance_mode_property_t,
                 WhiteBalanceModeProperty);
  CONVERTER_FUNC(senscord_auto_white_balance_property_t,
                 AutoWhiteBalanceProperty);
  CONVERTER_FUNC(senscord_manual_white_balance_preset_property_t,
                 ManualWhiteBalancePresetProperty);
  CONVERTER_FUNC(senscord_manual_white_balance_gain_property_t,
                 ManualWhiteBalanceGainProperty);
  CONVERTER_FUNC(senscord_info_string_property_t, InfoStringProperty);
  CONVERTER_FUNC(senscord_temperature_enable_property_t,
                 TemperatureEnableProperty);
  CONVERTER_FUNC(senscord_input_data_type_property_t, InputDataTypeProperty);
  CONVERTER_FUNC(senscord_camera_auto_exposure_metering_property_t, CameraAutoExposureMeteringProperty);
};

}  // namespace senscord
#endif  // SENSCORD_INFERENCE_PROPERTY_CONVERTER_INFERENCE_PROPERTY_CONVERTER_H_
