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
const uint16_t kRegOpdAeArbOffset =
    0xE65CU;  // OPD AE Arbitration Offset Register
const uint16_t kRegOpdAeArbValid =
    0xE660U;                                // OPD AE Arbitration Valid Register
const uint16_t kAeOpdWidthType1Max  = 450;  // Maximum AE OPD Width Type1 Value
const uint16_t kAeOpdHeightType1Max = 434;  // Maximum AE OPD Height Type1 Value

/* AE Parameter */
const uint16_t kRegIspManualMode = 0xD800U;  // ISP Manual Mode Register
const uint16_t kRegAeModeSn1     = 0xD23DU;  // AE Mode SN1 Register
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

const uint8_t kIspManualModeOff = 0x00U;    // ISP Automatic Mode
const uint8_t kAeAutoMode       = 0x00U;    // AE Auto Mode
const uint16_t kEvrefType1      = 0x1888U;  // Default EV Reference Type1 Value

/* AWB Parameter */
const uint16_t kRegAwbModeS1 = 0xD23CU;  // AWB Mode SN1 Register
const uint16_t kRegAwbUser0R = 0xDE20U;  // User Register 0 Red Gain
const uint16_t kRegAwbUser0B = 0xDE22U;  // User Register 0 Blue Gain
const uint16_t kRegAwbUser1R = 0xDE2CU;  // User Register 1 Red Gain
const uint16_t kRegAwbUser1B = 0xDE2EU;  // User Register 1 Blue Gain
const uint16_t kRegAwbUser2R = 0xDE38U;  // User Register 2 Red Gain
const uint16_t kRegAwbUser2B = 0xDE3AU;  // User Register 2 Blue Gain
const uint16_t kRegAwbUser3R = 0xDE44U;  // User Register 3 Red Gain
const uint16_t kRegAwbUser3B = 0xDE46U;  // User Register 3 Blue Gain
const uint16_t kRegAtwGainsInMoni =
    0xDB67U;  // ATW Gains Input Monitor Register
const uint16_t kRegAtwGainsInNrMoni =
    0xDB66U;  // ATW Gains Input NR Monitor Register
const uint16_t kRegOpdAwbValid  = 0xE664U;  // OPD AWB Valid Register
const uint16_t kRegOpdAwbOffset = 0xE670U;  // OPD AWB Offset Register
const uint16_t kRegOpdAwbMode   = 0xE674U;  // OPD AWB Mode Register

const uint8_t kAwbAutoMode = 0x00U;          // AWB Auto Mode
const uint16_t kUser0GainR = 0x1000U;        // Default R Gain for User Window 0
const uint16_t kUser0GainB = 0x1000U;        // Default B Gain for User Window 0
const uint16_t kUser1GainR = 0x0CFFU;        // Default R Gain for User Window 1
const uint16_t kUser1GainB = 0x1452U;        // Default B Gain for User Window 1
const uint16_t kUser2GainR = 0x0B6BU;        // Default R Gain for User Window 2
const uint16_t kUser2GainB = 0x187AU;        // Default B Gain for User Window 2
const uint16_t kUser3GainR = 0x0ACEU;        // Default R Gain for User Window 3
const uint16_t kUser3GainB = 0x1AD3U;        // Default B Gain for User Window 3
const uint32_t kAwbConvergenceSpeed = 0x6U;  // Default AWB convergence speed
const uint8_t kOpdAwbMode           = 0x00U;    // Default OPD AWB mode
const uint8_t kOpdAwbVValid         = 0xFCU;    // Default OPD AWB V valid size
const uint8_t kOpdAwbHValid         = 0xFCU;    // Default OPD AWB H valid size
const uint16_t kOpdAwbVOffset       = 0x0000U;  // Default OPD AWB V offset
const uint16_t kOpdAwbHOffset       = 0x0000U;  // Default OPD AWB H offset

#define AWB_CONVERGENCE_SPEED_NUM (255 - 11)
const uint32_t kAwbConvergenceSpeedValue[AWB_CONVERGENCE_SPEED_NUM] = {
    1,    7,    11,   15,   19,   24,   28,  32,  36,  40,  45,  49,  53,  57,
    61,   66,   70,   74,   78,   82,   87,  91,  95,  99,  103, 108, 112, 116,
    120,  124,  129,  133,  137,  141,  145, 150, 154, 158, 162, 166, 171, 175,
    179,  183,  187,  192,  196,  200,  204, 208, 213, 217, 221, 225, 229, 234,
    238,  242,  246,  250,  255,  259,  263, 267, 271, 276, 280, 284, 288, 292,
    297,  301,  305,  309,  313,  318,  322, 326, 330, 334, 339, 343, 347, 351,
    355,  360,  364,  368,  372,  376,  381, 385, 389, 393, 397, 402, 406, 410,
    414,  418,  423,  427,  431,  435,  439, 444, 448, 452, 456, 460, 465, 469,
    473,  477,  481,  486,  490,  494,  498, 502, 507, 511, 515, 519, 523, 528,
    532,  536,  540,  544,  549,  553,  557, 561, 565, 570, 574, 578, 582, 586,
    591,  595,  599,  603,  607,  612,  616, 620, 624, 628, 633, 637, 641, 645,
    649,  654,  658,  662,  666,  670,  675, 679, 683, 687, 691, 696, 700, 704,
    708,  712,  717,  721,  725,  729,  733, 738, 742, 746, 750, 754, 759, 763,
    767,  771,  775,  780,  784,  788,  792, 796, 801, 805, 809, 813, 817, 822,
    826,  830,  834,  838,  843,  847,  851, 855, 859, 864, 868, 872, 876, 880,
    885,  889,  893,  897,  901,  906,  910, 914, 918, 922, 927, 931, 935, 939,
    943,  948,  952,  956,  960,  964,  969, 973, 977, 981, 985, 990, 994, 998,
    1002, 1006, 1011, 1015, 1019, 1023,
    // Greater than 1024 do not be disclosed.
    /* 1027, 1032, 1036, 1040, 1044, 1048, 1053, 1057, 1061, 1065, 1069 */
};

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
   * @note Values are read in big-endian format (MSB first).
   */
  senscord::Status ReadRegister(const uint16_t reg, uint8_t* value,
                                size_t len = 1);

  /**
   * @brief Write register value to image sensor.
   * @note Values are written in big-endian format (MSB first).
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
  const uint8_t kMaxDataSize   = 4;     // Maximum data size (1, 2, or 4 bytes)
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
