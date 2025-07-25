/*
 * SPDX-FileCopyrightText: 2023 Sony Semiconductor Solutions Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef SENSCORD_LIBCAMERA_IMAGE_ANYVALUE_H_
#define SENSCORD_LIBCAMERA_IMAGE_ANYVALUE_H_

#include "senscord/logger.h"
#include "senscord/serialize.h"

namespace senscord {
namespace libcamera_image {

class AnyValue {
 public:
  // int, float, string, ...
  template <typename T>
  senscord::Status Set(const T& value) {
    senscord::serialize::SerializedBuffer buffer;
    senscord::serialize::Encoder encoder(&buffer);
    senscord::Status status = encoder.Push(value);
    if (status.ok()) {
      buffer.swap(&buffer_);
    }
    return SENSCORD_STATUS_TRACE(status);
  }

  template <typename T>
  senscord::Status Get(T* value) const {
    if (buffer_.empty()) {
      return SENSCORD_STATUS_FAIL(senscord::kStatusBlockCore,
                                  senscord::Status::kCauseInvalidArgument,
                                  "value is empty");
    }
    senscord::serialize::Decoder decoder(&buffer_[0], buffer_.size());
    senscord::Status status = decoder.Pop(*value);
    return SENSCORD_STATUS_TRACE(status);
  }

  // array
  template <typename T>
  senscord::Status Set(const std::vector<T>& value) {
    senscord::serialize::SerializedBuffer buffer;
    senscord::serialize::Encoder encoder(&buffer);
    senscord::Status status = encoder.Push(value);
    if (status.ok()) {
      buffer.swap(&buffer_);
    }
    return SENSCORD_STATUS_TRACE(status);
  }

  template <typename T>
  senscord::Status Get(std::vector<T>* value) const {
    if (buffer_.empty()) {
      return SENSCORD_STATUS_FAIL(senscord::kStatusBlockCore,
                                  senscord::Status::kCauseInvalidArgument,
                                  "value is empty");
    }
    senscord::serialize::Decoder decoder(&buffer_[0], buffer_.size());
    senscord::Status status = decoder.Pop(*value);
    return SENSCORD_STATUS_TRACE(status);
  }

  std::vector<uint8_t> buffer_;
  SENSCORD_SERIALIZE_DEFINE(buffer_)
};

}  // namespace libcamera_image
}  // namespace senscord

#endif
