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
#include "libcamera_image_sensor_register.h"
#include "senscord/libcamera_image/libcamera_image_types.h"
#include "v4l2_ctrl_manager.h"

#define AITRIOS_SENSOR_CHANNEL_ID_INFERENCE_OUTPUT      (0x00000000)
#define AITRIOS_SENSOR_CHANNEL_ID_INFERENCE_INPUT_IMAGE (0x00000001)
#define AITRIOS_SENSOR_CHANNEL_ID_INFERENCE_RAW_IMAGE   (0x80000000)
#define IMX500_FULL_RESOLUTION_WIDTH                    (4056)
#define IMX500_FULL_RESOLUTION_HEIGHT                   (3040)
#define MAX_NUM_DROP_FRAMES                             (7)
#define AE_FLICKER_PERIOD_50HZ                          ("20000us")
#define AE_FLICKER_PERIOD_60HZ                          ("16667us")
#define CAMERA_IMAGE_WIDTH_DEFAULT                      (2028) /* pixel */
#define CAMERA_IMAGE_HEIGHT_DEFAULT                     (1520) /* pixel */
#define CAMERA_FRAME_RATE_DEFAULT                       (30.f) /* fps */
#define CAMERA_FRAME_RATE_DENOM_DEFAULT                 (1.001f)
#define MODEL_ID_CONVERTER_VERSION_IT_ONLY              ("000000")
#define MODEL_ID_VERSION_NUMBER_IT_ONLY                 ("0000")

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
static constexpr unsigned int NetworkNameLen         = 64;
static constexpr unsigned int MaxNumTensors          = 16;
static constexpr unsigned int MaxNumDimensions       = 16;
static constexpr char AiBundleIdItonly[]             = "999997";
static constexpr char AiBundleIdVgaRgbItonly[]       = "999999";
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

enum ExposureModeParam {
  kExposureModeParamAuto,
  kExposureModeParamGainFix,
  kExposureModeParamTimeFix,
  kExposureModeParamManual,
  kExposureModeParamHold,
};

enum AeAntiFlickerMode {
  kAeAntiFlickerModeOff,
  kAeAntiFlickerModeAuto,
  kAeAntiFlickerModeForce50Hz,
  kAeAntiFlickerModeForce60Hz,
};

enum AeMeteringMode {
  kAeMeteringFullScreen,
  kAeMeteringUserWindow,
};

struct AeMeteringWindow {
  uint32_t top;
  uint32_t left;
  uint32_t bottom;
  uint32_t right;
};

struct AutoExposureParam {
  ExposureModeParam mode;
  uint32_t max_exposure_time;
  uint32_t min_exposure_time;
  float max_gain;
  uint32_t convergence_speed;
  float ev_compensation;
  AeAntiFlickerMode anti_flicker_mode;
  AeMeteringMode metering_mode;
  AeMeteringWindow window;
};

struct ManualExposureParam {
  bool keep;
  uint32_t exposure_time;
  float gain;
};

struct ImageFlip {
  bool h;
  bool v;
};

struct ImageCrop {
  uint32_t x;
  uint32_t y;
  uint32_t w;
  uint32_t h;
};

namespace senscord {

class Status;
class ImageProperty;
class FrameInfo;
class ImageSensorFunctionSupportedProperty;
class StreamSourceUtility;
class ImageRotationProperty;
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
  void GetFrames(std::vector<senscord::FrameInfo> *frames,
                 bool dry_run = false);
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
  senscord::Status GetProperty(
      senscord::libcamera_image::CameraTemperatureProperty *property);

  senscord::Status GetAIModelVersion(std::string &ai_model_version);
  senscord::Status SetExposureMode(ExposureModeParam mode);
  senscord::Status SetAutoExposureParam(uint32_t &max_exposure_time,
                                        uint32_t &min_exposure_time,
                                        float &max_gain,
                                        uint32_t &convergence_speed);
  senscord::Status SetAeEvCompensation(float &ev_compensation);
  senscord::Status SetAeAntiFlickerMode(AeAntiFlickerMode mode);
  senscord::Status SetAeMetering(AeMeteringMode mode, AeMeteringWindow &window);
  senscord::Status SetManualExposureParam(uint32_t exposure_time, float gain);
  senscord::Status SetImageSize(uint32_t width, uint32_t height);
  senscord::Status SetFrameRate(uint32_t num, uint32_t denom);
  senscord::Status SetImageFlip(bool flip_horizontal, bool flip_vertical);
  senscord::Status SetImageCrop(uint32_t left, uint32_t top, uint32_t width,
                                uint32_t height);
  senscord::Status SetIspImage(uint32_t width, uint32_t height,
                               char *pixel_format);
  senscord::Status GetIspImage(uint32_t &width, uint32_t &height,
                               uint32_t &stride_bytes, char *pixel_format);
  senscord::Status SetIspFrameRate(uint32_t num, uint32_t denom);

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
  bool GetDeviceID(std::string &device_id_str);

 private:
  bool CheckBundleIdItOnly(const std::string ai_model_bundle_id);
  std::string ReplaceCustomJsonPathString(const std::string &path,
                                          const std::string &replacement);
  std::string HandleCustomJsonPathString(const std::string &post_process_file,
                                         const std::string &ai_model_bundle_id);
  std::string ReadPostProcessJsonString(const std::string &post_process_file);
  std::string GetRpkPath(const std::string &json_str);
  bool CheckRpkExist(const std::string &post_process_file);
  bool ReadInputTensorSizeFromRpkFile(const uint8_t *data, size_t size);
  bool ReadInputTensorSize(const std::string &post_process_file);
  uint32_t ConvertCropHorizontalToSensor(uint32_t target);
  uint32_t ConvertCropVerticalToSensor(uint32_t target);
  bool IsNoCrop(uint32_t crop_left, uint32_t crop_top, uint32_t crop_width,
                uint32_t crop_height);
  bool IsValidCropRange(uint32_t crop_left, uint32_t crop_top,
                        uint32_t crop_width, uint32_t crop_height);
  bool UpdateImageCrop(void);

 private:
  static const uint8_t kImx500SensorId = 0;  // IMX500 sensor celsius
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
  senscord::ImageProperty base_image_property_;
  void InitializeOptions(Options *&options);
  senscord::libcamera_image::ImageRotationProperty rotation_property_;
  senscord::libcamera_image::CameraImageFlipProperty flip_property_;
  senscord::libcamera_image::TensorShapesProperty tensor_shapes_property_;
  libcamera::Rectangle sensor_output_;
  int32_t norm_val_[4];
  uint32_t norm_shift_[4];
  int32_t div_val_[4];
  uint32_t div_shift_;
  std::string device_name_;
  std::string ai_model_bundle_id_;
  int32_t count_drop_frames_;
  ExposureModeParam exposure_mode_;
  ManualExposureParam manual_exposure_;
  senscord::libcamera_image::SensorRegister reg_handle_;
  uint32_t camera_image_size_width_;
  uint32_t camera_image_size_height_;
  float camera_frame_rate_;
  ImageFlip image_flip_;
  ImageCrop image_crop_;
  bool no_image_crop_;
  bool is_running_;
  uint32_t isp_image_width_;
  uint32_t isp_image_height_;
  uint32_t camera_image_stride_bytes_;
  std::string isp_image_pixel_format_;
  float isp_frame_rate_;

  void UpdateTensorShapesProperty(CompletedRequestPtr payload);
  void UpdateImageRotationProperty(void);
};

}  // namespace libcamera_image
}  // namespace senscord

#endif
