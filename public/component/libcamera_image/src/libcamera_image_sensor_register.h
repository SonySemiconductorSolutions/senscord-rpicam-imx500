/*
 * SPDX-FileCopyrightText: 2023 Sony Semiconductor Solutions Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef LIB_COMPONENT_LIBCAMERA_IMAGE_SENSOR_REGISTER_H_
#define LIB_COMPONENT_LIBCAMERA_IMAGE_SENSOR_REGISTER_H_

#include <mutex>

#include "senscord/status.h"

#define I2C_DEVICE_NAME ("/dev/i2c-imx500")

namespace senscord {
namespace libcamera_image {

/* Temperature */
const uint16_t kRegTemperatureEnable = 0x0138U;   // Temperature Enable Register
const uint16_t kRegTemperatureValue = 0x013AU;    // Temperature Register

const uint8_t kRegTemperatureEnableMask = 0x01U;  // Temperature Enable Mask
const int8_t kRegTemperatureMin = -20;  // Minimum Temperature Value
const int8_t kRegTemperatureMax = 80;   // Maximum Temperature Value

/* Rotation */
const uint16_t kRegImageRotate = 0xD680U;  // Image Rotation Register
enum SensorRegRotationAngle {
  SENSOR_REG_ROTATION_0   = 0x00U,  // 0   degree
  SENSOR_REG_ROTATION_90  = 0x01U,  // 90  degrees
  SENSOR_REG_ROTATION_180 = 0x02U,  // 180 degrees
  SENSOR_REG_ROTATION_270 = 0x03U,  // 270 degrees
  SENSOR_REG_ROTATION_NONE
};

/**
 * @brief The image sensor register control class.
 */
class SensorRegister {
 public:
  SensorRegister();
  ~SensorRegister();

  /**
   * @brief Open to control the image sensor registers.
   */
  senscord::Status Open(void);

  /**
   * @brief Close to control the image sensor registers.
   */
  senscord::Status Close(void);

  /**
   * @brief Read register value from image sensor.
   */
  senscord::Status ReadRegister(const uint16_t reg, uint8_t* value, size_t len = 1);
  
  /**
   * @brief Write register value to image sensor.
   */
  senscord::Status WriteRegister(const uint16_t reg, const uint8_t* value, size_t len = 1);

  /**
   * @brief Check to enable access
   */
  bool IsEnableAccess(void) {
    std::lock_guard<std::mutex> lock(mutex_access_);
    return enable_access_;
  }

  /**
   * @brief Set to enable access
   * @note Access to the sensor register is enabled after the Start streaming.
   */
  void SetEnableAccess(bool enable) {
    std::lock_guard<std::mutex> lock(mutex_access_);
    enable_access_ = enable;
  }

 private:
  const uint8_t kDeviceAddress = 0x1A;  // I2C address of IMX500
  const uint8_t kRegAddrSize = 2;       // 2 bytes
  const uint8_t kBufLimit = 32;         // I2C buffer size limitation
  static std::mutex mutex_access_;
  static bool enable_access_;
  int handle_;

  /**
   * @brief Check if the device is not opened.
   * @return true if the device is not opened.
   */
  inline bool IsNotOpen(void) { return handle_ == -1; }
};

}  // namespace libcamera_image
}  // namespace senscord

#endif
