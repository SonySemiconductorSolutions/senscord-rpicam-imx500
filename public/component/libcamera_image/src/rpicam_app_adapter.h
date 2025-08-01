/*
 * SPDX-FileCopyrightText: 2023 Sony Semiconductor Solutions Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef LIB_COMPONENT_LIBCAMERA_IMAGE_SRC_LIBCAMERA_ADAPTER_SOURCE_H_
#define LIB_COMPONENT_LIBCAMERA_IMAGE_SRC_LIBCAMERA_ADAPTER_SOURCE_H_

#include <libcamera/controls.h>

#include <map>
#include <mutex>
#include <string>
#include <vector>

#include "core/options.hpp"
#include "core/rpicam_app.hpp"
#include "senscord/libcamera_image/libcamera_image_types.h"
#define AITRIOS_SENSOR_CHANNEL_ID_INFERENCE_OUTPUT (0x00000000)
#define AITRIOS_SENSOR_CHANNEL_ID_INFERENCE_INPUT_IMAGE (0x00000001)
#define AITRIOS_SENSOR_CHANNEL_ID_INFERENCE_RAW_IMAGE (0x80000000)
#define IMX500_FULL_RESOLUTION_WIDTH (4056)
#define IMX500_FULL_RESOLUTION_HEIGHT (3040)
namespace libcamera {
class CameraManager;
class Camera;
class ControlInfo;
class FrameBufferAllocator;
class FrameBuffer;
class Request;
class PixelFormat;
}  // namespace libcamera

// The structures match the definitions in the libcamera and rpicam-apps.
// Here is the comment from the libcamera:
//   The following structures are used to export the CNN input/output tensor
//   information through the rpi::CnnOutputTensorInfo and
//   rpi::CnnInputTensorInfo controls. Applications must cast the span to these
//   structures exactly.
//
//   - networkName is the name of the CNN used,
//   - numTensors is the number of output tensors returned,
//   - tensorDataNum gives the number of elements in each output tensor,
//   - numDimensions gives the dimensionality of each output tensor,
//   - size gives the size of each dimension in each output tensor.
static constexpr unsigned int NetworkNameLen = 64;
static constexpr unsigned int MaxNumTensors = 16;
static constexpr unsigned int MaxNumDimensions = 16;
static constexpr char AiBundleIdItonly[] = "999997";
static constexpr char AiBundleIdReserve[] = "999992";
static constexpr char AiBundleIdVgaRgbItonly[] = "999999";
static constexpr char AiBundleIdVgaItonly[] = "00000000000000000000000000000000";
static constexpr char CustomParamJsonFile[] = "custom.json";
static constexpr char CustomVgaItonlyParamJsonFile[] = "custom_vga_itonly.json";

struct OutputTensorInfo {
  uint32_t tensorDataNum;
  uint32_t numDimensions;
  uint16_t size[MaxNumDimensions];
};

struct CnnOutputTensorInfo {
  char networkName[NetworkNameLen];
  uint32_t numTensors;
  OutputTensorInfo info[MaxNumTensors];
};

// Define here because rpicam_app doesn't control input tensor
struct CnnInputTensorInfo {
  char networkName[NetworkNameLen];
  uint32_t width;
  uint32_t height;
  uint32_t numChannels;
};

namespace senscord {

class Status;
class ImageProperty;
class FrameInfo;
class ImageSensorFunctionSupportedProperty;
class StreamSourceUtility;
class ImageRotationProperty;
class ImageCropProperty;
class CameraImageFlipProperty;

namespace libcamera_image {

class LibcameraAdapter {
 public:
  LibcameraAdapter();
  ~LibcameraAdapter();

  senscord::Status Open(std::string device_name,
                        senscord::StreamSourceUtility *util,
                        senscord::ImageProperty &image_property);
  senscord::Status Close();
  senscord::Status Start();
  senscord::Status Stop();
  senscord::Status Configure(senscord::ImageProperty &image_property);
  senscord::Status GetImageProperty(senscord::ImageProperty *image_property,
                                    uint32_t id);

  void RequestComplete(libcamera::Request *request);
  void GetFrames(std::vector<senscord::FrameInfo> *frames, bool dry_run=false);
  senscord::Status ReleaseFrame(
      const senscord::FrameInfo &frameinfo,
      const std::vector<uint32_t> *referenced_channel_ids);

  senscord::Status GetProperty(
      senscord::ImageSensorFunctionSupportedProperty *property);
  senscord::Status GetControl(
      senscord::libcamera_image::AccessProperty *property);
  senscord::Status GetProperty(
      senscord::libcamera_image::AccessProperty *property);
  senscord::Status GetDevices(
      senscord::libcamera_image::DeviceEnumerationProperty *property);
  senscord::Status SetControl(
      const senscord::libcamera_image::AccessProperty *property);
  senscord::Status SetProperty(const senscord::ImageCropProperty *property);
  senscord::Status GetProperty(senscord::ImageCropProperty *property);
  senscord::Status SetProperty(
      const senscord::libcamera_image::ImageRotationProperty *property);
  senscord::Status GetProperty(
      senscord::libcamera_image::ImageRotationProperty *property);
  senscord::Status SetProperty(
      const senscord::libcamera_image::CameraImageFlipProperty *property);
  senscord::Status GetProperty(
      senscord::libcamera_image::CameraImageFlipProperty *property);
  senscord::Status SetProperty(
      const senscord::libcamera_image::AIModelBundleIdProperty *property);
  // for internal use
  senscord::Status SetLibcameraControl(
      const libcamera::ControlId &control_id,
      const libcamera::ControlInfo &control_info, const AnyValue &value);

  template <typename T, std::size_t N = std::numeric_limits<std::size_t>::max()>
  senscord::Status SetLibcameraControl(
      const libcamera::ControlId &control_id,
      const libcamera::ControlInfo &control_info, const AnyValue &value);

  template <typename T, std::size_t N = std::numeric_limits<std::size_t>::max()>
  senscord::Status SetLibcameraControl(
      const libcamera::ControlId &control_id,
      const libcamera::ControlInfo &control_info, const T &value);

  senscord::Status FindLibcameraControlId(
      const std::string &control_name, const libcamera::ControlId **control_id,
      libcamera::ControlInfo *control_info);

  void ConvertPixelFormat(libcamera::PixelFormat *format,
                          const std::string &pixel_format);
  void ConvertPixelFormat(std::string &pixel_format,
                          const libcamera::PixelFormat *format);

  void UpdateImageSensorFunctionSupportedProperty(void);

  senscord::Status ConvertValue(const libcamera::ControlType type,
                                const libcamera::ControlValue &target_value,
                                senscord::libcamera_image::AnyValue &value);
 private:
  bool CheckBundleIdItOnly(const std::string ai_model_bundle_id);
  std::string ReplaceCustomJsonPathString(
      const std::string &path,
      const std::string &target,
      const std::string &replacement);
  std::string HandleDefaultCustomJsonPathString(
      const std::string &post_process_file,
      const std::string &ai_model_bundle_id);
  std::string ReadPostProcessJsonString(const std::string &post_process_file);
  bool CheckRpkExist(const std::string &post_process_file);

 private:
  static std::unique_ptr<libcamera::CameraManager> camera_manager_;
  static std::mutex mutex_camera_manager_;
  static size_t camera_manager_ref_count_;

  senscord::StreamSourceUtility *util_;
  std::shared_ptr<libcamera::Camera> camera_;
  std::unique_ptr<libcamera::FrameBufferAllocator> allocator_;
  std::vector<std::unique_ptr<libcamera::Request>> requests_;
  std::vector<senscord::FrameInfo> frames_;
  senscord::ImageSensorFunctionSupportedProperty
      image_sensor_function_supported_property_;
  libcamera::ControlList controls_;
  std::map<std::string, AnyValue> current_control_values_;

  std::mutex mutex_frames_;
  std::mutex mutex_controls_;
  RPiCamApp *libcam_;
  Options *options_;
  senscord::ImageProperty *it_image_property_;
  senscord::ImageProperty *full_image_property_;
  void InitializeOptions(Options *&options);
  senscord::ImageCropProperty crop_property_;
  senscord::libcamera_image::ImageRotationProperty rotation_property_;
  senscord::libcamera_image::CameraImageFlipProperty flip_property_;
  senscord::libcamera_image::TensorShapesProperty tensor_shapes_property_;
  libcamera::Rectangle roi_;
  libcamera::Rectangle sensor_output_;
  int32_t norm_val_[4];
  uint32_t norm_shift_[4];
  int32_t div_val_[4];
  uint32_t div_shift_;
  std::string device_name_;
  std::string ai_model_bundle_id_;

  void UpdateTensorShapesProperty(CompletedRequestPtr payload);
};

}  // namespace libcamera_image
}  // namespace senscord

#endif
