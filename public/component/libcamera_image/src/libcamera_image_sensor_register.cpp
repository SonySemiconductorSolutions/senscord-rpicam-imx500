/*
 * SPDX-FileCopyrightText: 2023 Sony Semiconductor Solutions Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "libcamera_image_sensor_register.h"

#include <errno.h>
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include "senscord/logger.h"

namespace senscord {
namespace libcamera_image {

namespace {
// I2C retry configuration constants
constexpr int kI2cRetries      = 3;
constexpr int kI2cRetryDelayUs = 20000;  // 20ms
}  // namespace

std::mutex SensorRegister::mutex_access_;
bool SensorRegister::enable_access_ = false;

SensorRegister::SensorRegister() : handle_(-1) {}

SensorRegister::~SensorRegister() { Close(); }

senscord::Status SensorRegister::Open(void) {
  {
    std::lock_guard<std::mutex> lock(mutex_access_);
    if (handle_ != -1) {
      return senscord::Status::OK();  // Already opened
    }

    std::string dev_path = I2C_DEVICE_NAME;
    for (int attempt = 0; attempt < kI2cRetries; ++attempt) {
      handle_ = open(dev_path.c_str(), O_RDWR);
      if (handle_ >= 0) {
        if (ioctl(handle_, I2C_SLAVE_FORCE, kDeviceAddress) == 0) {
          return senscord::Status::OK();
        }
        close(handle_);
        handle_ = -1;
      }
      usleep(kI2cRetryDelayUs);
    }
    SENSCORD_LOG_ERROR_TAGGED("libcamera", "I2C device open/init error: %s",
                              strerror(errno));
    handle_ = -1;
    return SENSCORD_STATUS_FAIL("libcamera",
                                senscord::Status::kCauseInvalidOperation,
                                "Failed to open Sensor(I2C) device.");
  }
}

senscord::Status SensorRegister::Close(void) {
  std::lock_guard<std::mutex> lock(mutex_access_);
  if (handle_ != -1) {
    int ret = close(handle_);
    if (ret < 0) {
      SENSCORD_LOG_ERROR_TAGGED("libcamera", "I2C close error: %s",
                                strerror(errno));
      return SENSCORD_STATUS_FAIL("libcamera",
                                  senscord::Status::kCauseInvalidOperation,
                                  "Failed to close Sensor(I2C) device.");
    }
    handle_ = -1;
  }
  return senscord::Status::OK();
}

senscord::Status SensorRegister::ReadRegister(const uint16_t reg,
                                              uint8_t* value, size_t len) {
  std::lock_guard<std::mutex> lock(mutex_access_);
  if (IsNotOpen()) {
    return SENSCORD_STATUS_FAIL("libcamera",
                                senscord::Status::kCauseInvalidOperation,
                                "Sensor(I2C) Device not opened.");
  }
  if (len < 1) {
    return SENSCORD_STATUS_FAIL("libcamera",
                                senscord::Status::kCauseInvalidArgument,
                                "Length must be 1 or more.");
  }
  if (len > kBufLimit) {  // I2C buffer size limitation
    SENSCORD_LOG_ERROR_TAGGED("libcamera", "I2C read size exceeded: %zu", len);
    return SENSCORD_STATUS_FAIL("libcamera",
                                senscord::Status::kCauseInvalidOperation,
                                "I2C buffer size exceeded.");
  }

  uint8_t buf[kRegAddrSize];
  buf[0] = static_cast<uint8_t>((reg & 0xFF00U) >> 8U);
  buf[1] = static_cast<uint8_t>(reg & 0x00FFU);

  for (int attempt = 0; attempt < kI2cRetries; ++attempt) {
    ssize_t r_size = write(handle_, buf, kRegAddrSize);
    if ((r_size >= 0) && (static_cast<signed>(r_size) == kRegAddrSize)) {
      r_size = read(handle_, value, len);
      if ((r_size >= 0) && (static_cast<size_t>(r_size) == len)) {
        return senscord::Status::OK();
      }
    }
    SENSCORD_LOG_WARNING_TAGGED("libcamera", "I2C read attempt %d failed: %s",
                                attempt, strerror(errno));
    usleep(kI2cRetryDelayUs);
  }

  SENSCORD_LOG_ERROR_TAGGED("libcamera", "I2C read error: %s", strerror(errno));
  return SENSCORD_STATUS_FAIL("libcamera",
                              senscord::Status::kCauseInvalidOperation,
                              "Failed to read register value.");
}

senscord::Status SensorRegister::WriteRegister(const uint16_t reg,
                                               const uint8_t* value,
                                               size_t len) {
  std::lock_guard<std::mutex> lock(mutex_access_);
  if (IsNotOpen()) {
    return SENSCORD_STATUS_FAIL("libcamera",
                                senscord::Status::kCauseInvalidOperation,
                                "Sensor(I2C) Device not opened.");
  }
  if (len < 1) {
    return SENSCORD_STATUS_FAIL("libcamera",
                                senscord::Status::kCauseInvalidArgument,
                                "Length must be 1 or more.");
  }

  size_t reg_len = kRegAddrSize + len;
  if (reg_len > kBufLimit) {  // I2C buffer size limitation
    SENSCORD_LOG_ERROR_TAGGED("libcamera", "I2C write size exceeded: %zu",
                              reg_len);
    return SENSCORD_STATUS_FAIL("libcamera",
                                senscord::Status::kCauseInvalidOperation,
                                "I2C buffer size exceeded.");
  }

  uint8_t buf[reg_len];
  buf[0] = static_cast<uint8_t>((reg & 0xFF00U) >> 8U);
  buf[1] = static_cast<uint8_t>(reg & 0x00FFU);
  for (size_t i = 0; i < len; i++) {
    buf[kRegAddrSize + i] = value[i];
  }

  for (int attempt = 0; attempt < kI2cRetries; ++attempt) {
    ssize_t r_size = write(handle_, buf, reg_len);
    if ((r_size >= 0) && (static_cast<size_t>(r_size) == reg_len)) {
      return senscord::Status::OK();
    }
    SENSCORD_LOG_WARNING_TAGGED("libcamera", "I2C write attempt %d failed: %s",
                                attempt, strerror(errno));
    usleep(kI2cRetryDelayUs);
  }

  SENSCORD_LOG_ERROR_TAGGED("libcamera", "I2C write error: %s",
                            strerror(errno));
  return SENSCORD_STATUS_FAIL("libcamera",
                              senscord::Status::kCauseInvalidOperation,
                              "Failed to write register value.");
}

}  // namespace libcamera_image
}  // namespace senscord
