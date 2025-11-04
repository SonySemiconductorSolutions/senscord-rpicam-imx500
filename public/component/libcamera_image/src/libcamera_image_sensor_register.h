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
const uint16_t kRegTemperatureEnable = 0x0138U;  // Temperature Enable Register
const uint16_t kRegTemperatureValue  = 0x013AU;  // Temperature Register

const uint8_t kRegTemperatureEnableMask = 0x01U;  // Temperature Enable Mask
const int8_t kRegTemperatureMin         = -20;    // Minimum Temperature Value
const int8_t kRegTemperatureMax         = 80;     // Maximum Temperature Value

/* Rotation */
const uint16_t kRegImageRotate = 0xD680U;  // Image Rotation Register
enum SensorRegRotationAngle {
  SENSOR_REG_ROTATION_0   = 0x00U,  // 0   degree
  SENSOR_REG_ROTATION_90  = 0x01U,  // 90  degrees
  SENSOR_REG_ROTATION_180 = 0x02U,  // 180 degrees
  SENSOR_REG_ROTATION_270 = 0x03U,  // 270 degrees
  SENSOR_REG_ROTATION_NONE
};

/* AeMetering */
const uint16_t kRegAeMeteringMode  = 0xD228U;      // AE Metering Mode Register
const uint16_t kRegAeMeteringRatio = 0xD229U;      // AE Metering Ratio Register
const uint8_t kAeMeteringModeFullScreen  = 0x00U;  // Full Screen Metering Mode
const uint8_t kAeMeteringRatioFullScreen = 0x00U;  // Full Screen Metering Ratio
const uint8_t kAeMeteringModeUserWindow  = 0x01U;  // User Window Metering Mode
const uint8_t kAeMeteringRatioUserWindow = 0xFFU;  // User Window Metering Ratio

const uint16_t kRegEvrefType1 = 0xD260U;  // EV Reference Type1 Register
const uint16_t kRegAeEvrefFreeMode =
    0xD22AU;  // AE EV Reference Free Mode Register
const uint16_t kRegAeOpdWidthType1  = 0xD24CU;  // AE OPD Width Type1 Register
const uint16_t kRegAeOpdHeightType1 = 0xD24EU;  // AE OPD Height Type1 Register
const uint16_t kRegOpdAeArbVOffset =
    0xE65CU;  // OPD AE Arbitration V Offset Register
const uint16_t kRegOpdAeArbHOffset =
    0xE65EU;  // OPD AE Arbitration H Offset Register
const uint16_t kRegOpdAeArbVValid =
    0xE660U;  // OPD AE Arbitration V Valid Register
const uint16_t kRegOpdAeArbHValid =
    0xE662U;  // OPD AE Arbitration H Valid Register
const uint16_t kAeOpdWidthType1Max  = 450;  // Maximum AE OPD Width Type1 Value
const uint16_t kAeOpdHeightType1Max = 434;  // Maximum AE OPD Height Type1 Value

/* AE Parameter */
const uint16_t kRegAelineLimitFType1 =
    0xD2B2U;  // AE Line Limit F Type1 Register
const uint16_t kRegAelineMaxshtLimitType1 =
    0xD2B4U;  // AE Line Max Shutter Limit Type1 Register
const uint16_t kRegShtctrltime1Type1 =
    0xD26DU;  // Shutter Control Time1 Type1 Register
const uint16_t kRegShtctrlmag1 =
    0xD984U;  // Shutter Control Magnitude1 Register
const uint16_t kRegIvtPrepllckDiv =
    0x0305U;                              // IVT Pre PLL Clock Divider Register
const uint16_t kRegIvtpllMpy  = 0x0306U;  // IVT PLL Multiplier Register
const uint16_t kRegIvtSyckDiv = 0x0303U;  // IVT System Clock Divider Register
const uint16_t kRegIvtPxckDiv = 0x0301U;  // IVT Pixel Clock Divider Register
const uint16_t kRegLineLengthPck = 0x0342U;  // Line Length PCK Register
const uint16_t kRegShtminline    = 0xE600U;  // Shutter Minimum Line Register
const uint16_t kRegAgcgain1Type1 = 0xD26EU;  // AGC Gain1 Type1 Register
const uint16_t kRegAespeedMoni   = 0xD2D8U;  // AE Speed Monitor Register
const uint16_t kRegErrscllmit    = 0xD9B2U;  // Error Scale Limit Register

/* Ev Compensation */
const uint16_t kRegEvsel = 0xD227U;  // EV Selection Register
const uint16_t kRegEvselGainP13 =
    0xD9EEU;  // EV Selection Gain Plus 13 Register
const uint16_t kRegEvselGainM13 =
    0xD9F4U;                           // EV Selection Gain Minus 13 Register
const int8_t kImx500IspEvGainNum = 6;  // IMX500 ISP EV Gain Number

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
  senscord::Status ReadRegister(const uint16_t reg, uint8_t* value,
                                size_t len = 1);

  /**
   * @brief Write register value to image sensor.
   */
  senscord::Status WriteRegister(const uint16_t reg, const uint8_t* value,
                                 size_t len = 1);

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
  const uint8_t kRegAddrSize   = 2;     // 2 bytes
  const uint8_t kBufLimit      = 32;    // I2C buffer size limitation
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
