/*
 * SPDX-FileCopyrightText: 2023 Sony Semiconductor Solutions Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef LIB_COMPONENT_LIBCAMERA_IMAGE_SRC_LIBCAMERA_IMAGE_SOURCE_H_
#define LIB_COMPONENT_LIBCAMERA_IMAGE_SRC_LIBCAMERA_IMAGE_SOURCE_H_

#include <string>
#include <vector>

#include "rpicam_app_adapter.h"
#include "senscord/develop/stream_source.h"
#include "senscord/libcamera_image/libcamera_image_types.h"
#include "senscord/logger.h"
#include "senscord/osal.h"
#include "senscord/serialize.h"
#include "senscord/serialize_buffer.h"

#define INFO_STRING_CATEGORY_SENSOR_NAME (0x00000000)
#define INFO_STRING_CATEGORY_SENSOR_ID (0x00000001)
#define INFO_STRING_CATEGORY_KEY_GENERATION (0x00000002)
#define INFO_STRING_CATEGORY_FIRMWARE_VERSION (0x00010000)
#define INFO_STRING_CATEGORY_LOADER_VERSION (0x00010001)
#define INFO_STRING_CATEGORY_AI_MODEL_VERSION (0x00010002)

namespace senscord {
namespace libcamera_image {

/**
 * @brief The stream source of libcamera image (new style).
 */
class LibcameraImageStreamSource : public senscord::ImageStreamSource {
 public:
  /**
   * @brief Constructor
   */
  LibcameraImageStreamSource();

  /**
   * @brief Destructor
   */
  ~LibcameraImageStreamSource();

  /**
   * @brief Open the stream source.
   * @param[in] (core) The core instance.
   * @param[in] (util) The utility accessor to core.
   * @return The status of function.
   */
  virtual senscord::Status Open(senscord::Core* core,
                                senscord::StreamSourceUtility* util);

  /**
   * @brief Close the stream source.
   * @return The status of function.
   */
  virtual senscord::Status Close();

  /**
   * @brief Start the stream source.
   * @return The status of function.
   */
  virtual senscord::Status Start();

  /**
   * @brief Stop the stream source.
   * @return The status of function.
   */
  virtual senscord::Status Stop();

  /**
   * @brief Pull up the new frames.
   * @param[out] (frames) The information about new frames.
   */
  virtual void GetFrames(std::vector<senscord::FrameInfo>* frames);

  /**
   * @brief Release the used frame.
   * @param[in] (frameinfo) The information about used frame.
   * @param[in] (referenced_channel_ids) List of referenced channel IDs.
   *                                     (NULL is the same as empty)
   * @return The status of function.
   */
  virtual senscord::Status ReleaseFrame(
      const senscord::FrameInfo& frameinfo,
      const std::vector<uint32_t>* referenced_channel_ids);

  /// Mandatory properties.

  /**
   * @brief Get the stream source property.
   * @param[in] (key) The key of property.
   * @param[out] (property) The location of property.
   * @return The status of function.
   */
  virtual senscord::Status Get(const std::string& key,
                               senscord::ChannelInfoProperty* property);

  /**
   * @brief Get the stream source property.
   * @param[in] (key) The key of property.
   * @param[out] (property) The location of property.
   * @return The status of function.
   */
  virtual senscord::Status Get(const std::string& key,
                               senscord::FrameRateProperty* property);

  /**
   * @brief Set the new stream source property.
   * @param[in] (key) The key of property.
   * @param[in] (property) The location of property.
   * @return The status of function.
   */
  virtual senscord::Status Set(const std::string& key,
                               const senscord::FrameRateProperty* property);

  /**
   * @brief Get the stream source property.
   * @param[in] (key) The key of property.
   * @param[out] (property) The location of property.
   * @return The status of function.
   */
  virtual senscord::Status Get(const std::string& key,
                               senscord::ImageProperty* property);

  /**
   * @brief Set the new stream source property.
   * @param[in] (key) The key of property.
   * @param[in] (property) The location of property.
   * @return The status of function.
   */
  virtual senscord::Status Set(const std::string& key,
                               const senscord::ImageProperty* property);

  /**
   * @brief Get the stream source property.
   * @param[in] (key) The key of property.
   * @param[out] (property) The location of property.
   * @return The status of function.
   */
  senscord::Status Get(
      const std::string& key,
      senscord::ImageSensorFunctionSupportedProperty* property);

  /**
   * @brief Set the new stream source property.
   * @param[in] (key) The key of property.
   * @param[in] (property) The location of property.
   * @return The status of function.
   */
  senscord::Status Set(
      const std::string& key,
      const senscord::ImageSensorFunctionSupportedProperty* property);

  /**
   * @brief Get the stream source property.
   * @param[in] (key) The key of property.
   * @param[out] (property) The location of property.
   * @return The status of function.
   */
  senscord::Status Get(const std::string& key,
                       senscord::libcamera_image::AccessProperty* property);

  /**
   * @brief Set the new stream source property.
   * @param[in] (key) The key of property.
   * @param[in] (property) The location of property.
   * @return The status of function.
   */
  senscord::Status Set(
      const std::string& key,
      const senscord::libcamera_image::AccessProperty* property);

  /**
   * @brief Set the new stream source property.
   * @param[in] (key) The key of property.
   * @param[in] (property) The location of property.
   * @return The status of function.
   */
  senscord::Status Set(
      const std::string& key,
      const senscord::libcamera_image::DeviceEnumerationProperty* property);

  /**
   * @brief Get the stream source property.
   * @param[in] (key) The key of property.
   * @param[out] (property) The location of property.
   * @return The status of function.
   */
  senscord::Status Get(
      const std::string& key,
      senscord::libcamera_image::DeviceEnumerationProperty* property);


  /**
   * @brief Set the new stream source property.
   * @param[in] (key) The key of property.
   * @param[in] (property) The location of property.
   * @return The status of function.
   */
  senscord::Status Set(
      const std::string& key,
      const senscord::ImageCropProperty* property);

  /**
   * @brief Get the stream source property.
   * @param[in] (key) The key of property.
   * @param[out] (property) The location of property.
   * @return The status of function.
   */
  senscord::Status Get(
      const std::string& key,
      senscord::ImageCropProperty* property);

  /**
   * @brief Set the new stream source property.
   * @param[in] (key) The key of property.
   * @param[in] (property) The location of property.
   * @return The status of function.
   */
  senscord::Status Set(
      const std::string& key,
      const senscord::libcamera_image::ImageRotationProperty* property);

  /**
   * @brief Get the stream source property.
   * @param[in] (key) The key of property.
   * @param[out] (property) The location of property.
   * @return The status of function.
   */
  senscord::Status Get(
      const std::string& key,
      senscord::libcamera_image::ImageRotationProperty* property);
  /**
   * @brief Set the new stream source property.
   * @param[in] (key) The key of property.
   * @param[in] (property) The location of property.
   * @return The status of function.
   */
  senscord::Status Set(
      const std::string& key,
      const senscord::libcamera_image::CameraImageFlipProperty* property);

  /**
   * @brief Get the stream source property.
   * @param[in] (key) The key of property.
   * @param[out] (property) The location of property.
   * @return The status of function.
   */
  senscord::Status Get(
      const std::string& key,
      senscord::libcamera_image::CameraImageFlipProperty* property);


  /**
   * @brief Set the new stream source property.
   * @param[in] (key) The key of property.
   * @param[in] (property) The location of property.
   * @return The status of function.
   */
  senscord::Status Set(
      const std::string& key,
      const senscord::libcamera_image::AIModelBundleIdProperty* property);

  /**
   * @brief Get the stream source property.
   * @param[in] (key) The key of property.
   * @param[out] (property) The location of property.
   * @return The status of function.
   */
  senscord::Status Get(
      const std::string& key,
      senscord::libcamera_image::AIModelBundleIdProperty* property);

  /**
   * @brief Set the new stream source property.
   * @param[in] (key) The key of property.
   * @param[in] (property) The location of property.
   * @return The status of function.
   */
  senscord::Status Set(
      const std::string& key,
      const senscord::libcamera_image::InfoStringProperty* property);

  senscord::Status Get(
      const std::string& key,
      senscord::libcamera_image::InfoStringProperty* property);
  /**
   * @brief Set the new stream source property.
   * @param[in] (key) The key of property.
   * @param[in] (property) The location of property.
   * @return The status of function.
   */
  senscord::Status Set(
      const std::string& key,
      const senscord::libcamera_image::InputDataTypeProperty* property);

  /**
   * @brief Get the stream source property.
   * @param[in] (key) The key of property.
   * @param[out] (property) The location of property.
   * @return The status of function.
   */
  senscord::Status Get(
      const std::string& key,
      senscord::libcamera_image::InputDataTypeProperty* property);
  static const uint64_t kWaitOnGetFrames;

  senscord::Status Set(
      const std::string& key,
      const senscord::libcamera_image::PostProcessAvailableProperty* property);
  senscord::Status Get(
      const std::string& key,
      senscord::libcamera_image::PostProcessAvailableProperty* property);
  senscord::Status Set(
      const std::string &key,
      const senscord::libcamera_image::CameraExposureModeProperty *property);
  senscord::Status Get(
      const std::string &key,
      senscord::libcamera_image::CameraExposureModeProperty *property);
  senscord::Status Set(
      const std::string &key,
      const senscord::libcamera_image::CameraAutoExposureProperty *property);
  senscord::Status Get(
      const std::string &key,
      senscord::libcamera_image::CameraAutoExposureProperty *property);
  senscord::Status Set(
      const std::string &key,
      const senscord::libcamera_image::CameraEvCompensationProperty *property);
  senscord::Status Get(
      const std::string &key,
      senscord::libcamera_image::CameraEvCompensationProperty *property);
  senscord::Status Set(
      const std::string &key,
      const senscord::libcamera_image::CameraAntiFlickerModeProperty *property);
  senscord::Status Get(
      const std::string &key,
      senscord::libcamera_image::CameraAntiFlickerModeProperty *property);
  senscord::Status Set(
      const std::string &key,
      const senscord::libcamera_image::CameraAutoExposureMeteringProperty *property);
  senscord::Status Get(
      const std::string &key,
      senscord::libcamera_image::CameraAutoExposureMeteringProperty *property);
  senscord::Status Set(
      const std::string &key,
      const senscord::libcamera_image::CameraManualExposureProperty *property);
  senscord::Status Get(
      const std::string &key,
      senscord::libcamera_image::CameraManualExposureProperty *property);
  senscord::Status Set(
      const std::string &key,
      const senscord::libcamera_image::CameraTemperatureProperty *property);
  senscord::Status Get(
      const std::string &key,
      senscord::libcamera_image::CameraTemperatureProperty *property);

 private:
  bool GetDeviceID(void);

 private:
  senscord::Core* core_;
  senscord::StreamSourceUtility* util_;
  senscord::libcamera_image::LibcameraAdapter adapter_;

  // properties
  senscord::FrameRateProperty framerate_property_;
  senscord::ImageProperty image_property_;
  senscord::ImageCropProperty crop_property_;
  senscord::libcamera_image::AIModelBundleIdProperty ai_model_bundle_id_;
  uint32_t display_channel_;
  std::string imx500_device_id_;
  std::mutex device_id_mutex_;
  CameraExposureModeProperty camera_exposure_mode_;
  CameraAutoExposureProperty camera_auto_exposure_;
  CameraEvCompensationProperty camera_ev_compensation_;
  CameraAntiFlickerModeProperty camera_anti_flicker_mode_;
  CameraManualExposureProperty camera_manual_exposure_;
  CameraAutoExposureMeteringProperty camera_auto_exposure_metering_;
};

}  // namespace libcamera_image
}  // namespace senscord

#endif  // LIB_COMPONENT_LIBCAMERA_IMAGE_SRC_LIBCAMERA_IMAGE_SOURCE_H_
