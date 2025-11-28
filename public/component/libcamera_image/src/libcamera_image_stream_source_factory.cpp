/*
 * SPDX-FileCopyrightText: 2023 Sony Semiconductor Solutions Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <utility>
#include <vector>

#include "libcamera_image_stream_source.h"
#include "senscord/develop/standard_component.h"
#include "senscord/inference_stream/inference_stream_types.h"
#include "senscord/logger.h"

namespace senscord {
namespace libcamera_image {

/**
 * @brief The factory of stream sources for libcamera image component.
 */
class LibcameraImageStreamSourceFactory : public senscord::StreamSourceFactory {
 public:
  /**
   * @brief Get the List of supported types.
   * @param[in] (args) Arguments written by senscord.xml.
   * @param[out] (list) List of supported types.
   */
  virtual void GetSupportedList(const senscord::ComponentArgument &args,
                                SourceTypeList *list) {
    list->push_back(std::make_pair(senscord::kStreamTypeInference, 0));
  }

  /**
   * @brief Create the stream source on the component.
   * @param[in] (type) Type of creating source.
   * @param[out] (source) Stream source.
   * @return Status object.
   */
  virtual senscord::Status CreateSource(const SourceType &type,
                                        senscord::StreamSource **source) {
    if (type.first == senscord::kStreamTypeInference) {
      *source = new LibcameraImageStreamSource();
    }
    return senscord::Status::OK();
  }
};

}  // namespace libcamera_image
}  // namespace senscord

// register
SENSCORD_REGISTER_COMPONENT(
    senscord::libcamera_image::LibcameraImageStreamSourceFactory)
