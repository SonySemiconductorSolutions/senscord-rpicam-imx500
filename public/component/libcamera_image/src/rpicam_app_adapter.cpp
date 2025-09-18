/*
 * SPDX-FileCopyrightText: 2023 Sony Semiconductor Solutions Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "rpicam_app_adapter.h"

#include <jpeglib.h>
#include <libcamera/camera.h>
#include <libcamera/control_ids.h>
#include <libcamera/controls.h>
#include <libcamera/geometry.h>
#include <libcamera/libcamera.h>
#include <libcamera/property_ids.h>
#include <libcamera/stream.h>
#include <libcamera/transform.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <array>
#include <nlohmann/json.hpp>
#include <optional>

#include "post_processing_stages/object_detect.hpp"
#include "post_processing_stages/post_processing_stage.hpp"
#include "senscord/develop/stream_source_utility.h"
#include "senscord/osal.h"
#include "senscord/property_types.h"
#include "senscord/senscord.h"
#include "senscord/status.h"
#include "senscord/stream.h"

namespace {
template <typename T>
senscord::Status convertValue(const libcamera::ControlValue &target_value,
                              senscord::libcamera_image::AnyValue &value) {
  if (target_value.isArray()) {
    std::vector<T> vec;
    for (size_t i = 0; i < target_value.numElements(); i++) {
      uint8_t packed_buffer[sizeof(T)];
      for (size_t j = 0; j < sizeof(T); j++) {
        packed_buffer[j] = target_value.data()[i * sizeof(T) + j];
      }
      vec.push_back(*reinterpret_cast<T *>(&packed_buffer));
    }
    value.Set<std::vector<T>>(vec);
    return senscord::Status::OK();
  }

  std::optional<T> optional_value = target_value.get<T>();
  if (optional_value.has_value()) {
    value.Set<T>(optional_value.value());
    return senscord::Status::OK();
  }

  return SENSCORD_STATUS_FAIL(
      "libcamera", senscord::Status::kCauseInvalidArgument, "invalid type");
}
}  // namespace

namespace senscord {
namespace libcamera_image {

std::unique_ptr<libcamera::CameraManager> LibcameraAdapter::camera_manager_;
std::mutex LibcameraAdapter::mutex_camera_manager_;
size_t LibcameraAdapter::camera_manager_ref_count_ = 0;

LibcameraAdapter::LibcameraAdapter()
    : util_(nullptr),
      libcam_(nullptr),
      options_(nullptr),
      it_image_property_(nullptr),
      roi_{0, 0, IMX500_FULL_RESOLUTION_WIDTH, IMX500_FULL_RESOLUTION_HEIGHT},
      sensor_output_{0, 0, IMX500_FULL_RESOLUTION_WIDTH,
                     IMX500_FULL_RESOLUTION_HEIGHT},
      norm_val_{0, 0, 0, 0},
      norm_shift_{0, 0, 0, 0},
      div_val_{1, 1, 1, 1},
      div_shift_(0),
      full_image_property_(nullptr),
      rotation_property_({0}),
      flip_property_({0, 0}),
      device_name_(""),
      ai_model_bundle_id_(std::string(AiBundleIdItonly)),
      count_drop_frames_(0),
      camera_image_width_(CAMERA_IMAGE_WIDTH_DEFAULT),
      camera_image_height_(CAMERA_IMAGE_HEIGHT_DEFAULT),
      camera_frame_rate_(
          CAMERA_FRAME_RATE_DEFAULT / CAMERA_FRAME_RATE_DENOM_DEFAULT),
      image_flip_{false, false},
      image_crop_{
          0, 0, CAMERA_IMAGE_WIDTH_DEFAULT, CAMERA_IMAGE_HEIGHT_DEFAULT} {}
LibcameraAdapter::~LibcameraAdapter() {}

senscord::Status LibcameraAdapter::Open(
    std::string device_name, senscord::StreamSourceUtility *util,
    senscord::ImageProperty &image_property) {
  std::lock_guard<std::mutex> lock(LibcameraAdapter::mutex_camera_manager_);

  exposure_mode_ = kExposureModeParamAuto;
  manual_exposure_.keep = false;
  manual_exposure_.exposure_time = 0;
  manual_exposure_.gain = 0.0;
  util_ = util;
  device_name_ = device_name;
  libcam_ = new RPiCamApp();
  std::string post_process_file;
  uint64_t uint_value = 0;
  std::string string_value = "";
  senscord::Status status = senscord::Status::OK();
  options_ = libcam_->GetOptions();
  InitializeOptions(options_);
  it_image_property_ = new senscord::ImageProperty(image_property);
  full_image_property_ = new senscord::ImageProperty(image_property);
  options_->viewfinder_width = 640;
  options_->viewfinder_height = 480;
  options_->post_process_file =
      "/usr/share/rpi-camera-assets/imx500_mobilenet_ssd.json";
  {
    status = util_->GetStreamArgument("width", &uint_value);
    if (status.ok()) {
      options_->viewfinder_width = static_cast<uint32_t>(uint_value);
    }
  }

  {
    status = util_->GetStreamArgument("height", &uint_value);
    if (status.ok()) {
      options_->viewfinder_height = static_cast<uint32_t>(uint_value);
    }
  }
  {
    status = util_->GetStreamArgument("fps", &uint_value);
    if (status.ok()) {
      options_->framerate = static_cast<uint32_t>(uint_value);
    }
  }
  {
    status = util_->GetStreamArgument("post_process_file", &post_process_file);
    if (status.ok()) {
      options_->post_process_file = post_process_file;
    }

    post_process_file =
        HandleCustomJsonPathString(options_->post_process_file, ai_model_bundle_id_);
    if (post_process_file.length()) {
      options_->post_process_file = post_process_file;
    } else {
      return SENSCORD_STATUS_FAIL(
          "libcamera", senscord::Status::kCauseInvalidOperation,
          "The default JSON file does not exist.");
    }
  }
  {
    status = util_->GetStreamArgument("pixel_format", &string_value);
    if (status.ok()) {
      options_->viewfinder_mode_string = string_value;
    }
  }

  options_->verbose = 2;
  options_->denoise = "auto";
  options_->contrast = 1.0f;
  options_->saturation = 1.0f;

  // Parse the JSON file
  std::string j_str = ReadPostProcessJsonString(options_->post_process_file);
  if (j_str.empty()) {
    return SENSCORD_STATUS_FAIL(
        "libcamera", senscord::Status::kCauseInvalidOperation,
        "Failed to read the json parameter for PostProcess.");
  }

  nlohmann::json j = nlohmann::json::parse(j_str);
  if (j.is_discarded()) {
    return SENSCORD_STATUS_FAIL(
        "libcamera", senscord::Status::kCauseInvalidOperation, "Not json");
  }
  for (auto &[key, value] : j.items()) {
    if (!value.is_object()) continue;

    for (auto &[subkey, subvalue] : value.items()) {
      if (subvalue.contains("norm_val")) {
        auto norm_val = subvalue["norm_val"];
        if (norm_val.is_array()) {
          if (norm_val.size() > 4) {
            return SENSCORD_STATUS_FAIL(
                "libcamera", senscord::Status::kCauseInvalidOperation,
                "norm_val Not correct size");
          }
          for (size_t i = 0; i < norm_val.size(); ++i) {
            norm_val_[i] = norm_val[i].get<int32_t>();
          }
        }
      }
      if (subvalue.contains("norm_shift")) {
        auto norm_shift = subvalue["norm_shift"];
        if (norm_shift.is_array()) {
          if (norm_shift.size() > 4) {
            return SENSCORD_STATUS_FAIL(
                "libcamera", senscord::Status::kCauseInvalidOperation,
                "norm_shift Not correct size");
          }
          for (size_t i = 0; i < norm_shift.size(); ++i) {
            norm_shift_[i] = norm_shift[i].get<uint32_t>();
          }
        }
      }
      if (subvalue.contains("div_val")) {
        auto div_val = subvalue["div_val"];
        if (div_val.is_array()) {
          if (div_val.size() > 4) {
            return SENSCORD_STATUS_FAIL(
                "libcamera", senscord::Status::kCauseInvalidOperation,
                "div_val Not correct size");
          }
          for (size_t i = 0; i < div_val.size(); ++i) {
            div_val_[i] = div_val[i].get<int32_t>();
            if (div_val_[i] == 0) {
              div_val_[i] = 1;  // Avoid division by zero
            }
          }
        }
      }
      if (subvalue.contains("div_shift")) {
        auto div_shift = subvalue["div_shift"];
        if (div_shift.is_number()) {
          div_shift_ = div_shift.get<uint32_t>();
        }
      }
    }
  }

  UpdateImageSensorFunctionSupportedProperty();

  libcamera::logSetLevel("*", "DEBUG");
  libcam_->OpenCamera();
  const auto &cameras = libcam_->GetCameras();
  if (device_name.empty()) {
    camera_ = cameras.at(0);
  } else {
    auto result =
        std::find_if(cameras.begin(), cameras.end(),
                     [&device_name](std::shared_ptr<libcamera::Camera> camera) {
                       return camera->id() == device_name;
                     });

    if (result == cameras.end()) {
      // if all of connected device_name are not matched, then try to access
      // with device_name as index
      try {
        int device_index_ = std::stoi(device_name);
        camera_ = cameras.at(device_index_ - 1);
      } catch (const std::exception &e) {
        // return OK even selected camera is not found.
        return senscord::Status::OK();
      }
    } else {
      camera_ = *result;
    }
  }

  status = reg_handle_.Open();
  if (!status.ok()) {
    SENSCORD_LOG_ERROR_TAGGED("libcamera",
      "Failed to open image sensor register control.");
    return status;
  }

  SENSCORD_LOG_INFO_TAGGED("libcamera", "libcamera::Camera(%s) : \"%s\"",
                           device_name.c_str(), camera_->id().c_str());
  return senscord::Status::OK();
}

void LibcameraAdapter::InitializeOptions(Options *&options) {
  // Set default values to the options
  // Note: With wasm module, the default values set using boost program_options.
  //       However with AOT file, the boost program_optoins does not work.
  //       As workaround, setting the default values here.
  options_->help = false;
  options_->version = false;
  options_->list_cameras = false;
  options_->verbose = 0;
  // options_->timeout = 0;               // Need to set it to private member
  // std::string config_file;             // No default
  // std::string output;                  // No default
  // std::string post_process_file;       // No default
  // std::string post_process_libs;       // No default
  options_->width = 0;
  options_->height = 0;
  options_->nopreview = true;
  std::string preview = "0,0,0,0";
  options_->fullscreen = false;
  options_->preview_x = 0;
  options_->preview_y = 0;
  options_->preview_width = 0;
  options_->preview_height = 0;
  // options_->transform;                 // No default
  std::string roi = "0,0,0,0";
  options_->roi_x = 0.0f;
  options_->roi_y = 0.0f;
  options_->roi_width = 0.0f;
  options_->roi_height = 0.0f;
  // options_->shutter = 0;               // In private
  options_->gain = 0.0f;
  std::string metering = "centre";
  options_->metering_index = 0;
  std::string exposure = "normal";
  options_->exposure_index = 0;
  options_->ev = 0.0f;
  std::string awb = "auto";
  options_->awb_index = 0;
  std::string awbgains = "0,0";
  options_->awb_gain_r = 0.0f;
  options_->awb_gain_b = 0.0f;
  options_->flush = false;
  options_->wrap = 0;
  options_->brightness = 0.0f;
  options_->contrast = 1.0f;
  options_->saturation = 1.0f;
  options_->sharpness = 1.0f;
  // options_->framerate = -1.0f;         // Need to set it to private member
  std::string denoise = "auto";
  std::string info_text = "frame";
  options_->viewfinder_width = 0;
  options_->viewfinder_height = 0;
  std::string tuning_file = "-";
  options_->qt_preview = false;
  options_->lores_width = 0;
  options_->lores_height = 0;
  options_->lores_par = false;
  options_->camera = 0;
  // std::string mode_string;             // No default
  // Mode mode;                           // No default
  // std::string viewfinder_mode_string;  // No default
  // Mode viewfinder_mode;                // No default
  options_->buffer_count = 0;
  options_->viewfinder_buffer_count = 0;
  std::string afMode = "afMode";
  options_->afMode_index = 0;
  std::string afRange = "normal";
  options_->afRange_index = 0;
  std::string afSpeed = "normal";
  options_->afSpeed_index = 0;
  std::string afWindow = "0,0,0,0";
  options_->afWindow_x = 0.0f;
  options_->afWindow_y = 0.0f;
  options_->afWindow_width = 0.0f;
  options_->afWindow_height = 0.0f;
  options_->lens_position = 0.0f;
  options_->set_default_lens_position = false;
  options_->af_on_capture = false;
  // std::string metadata;                // No default
  std::string metadata_format = "json";
  std::string hdr = "off";
  // TimeVal<std::chrono::microseconds> flicker_period; // In private
  options_->no_raw = false;
}

senscord::Status LibcameraAdapter::Close() {
  std::lock_guard<std::mutex> lock(LibcameraAdapter::mutex_camera_manager_);

  if (camera_) {
    camera_->stop();
    camera_.reset();
    libcam_->StopCamera();
    libcam_->Teardown();
    libcam_->CloseCamera();
  }
  delete libcam_;
  reg_handle_.Close();
  return senscord::Status::OK();
}

senscord::Status LibcameraAdapter::Start() {
  if (!camera_) {
    return SENSCORD_STATUS_FAIL(
        "libcamera", senscord::Status::kCauseInvalidOperation, "Not opened");
  }

  libcamera::ControlList cl;
  cl.set(libcamera::controls::rpi::CnnEnableInputTensor, true);
  libcam_->SetControls(cl);

  libcam_->StartCamera();
  UpdateImageRotationProperty();  // for reset in StartCamera()
  libcam_->SetInferenceRoiAbs(roi_);

  count_drop_frames_ = 0;

  return senscord::Status::OK();
}

senscord::Status LibcameraAdapter::Stop() {
  libcam_->StopCamera();

  {
    std::lock_guard<std::mutex> lock(mutex_frames_);
    frames_.clear();
  }

  libcam_->Teardown();

  return senscord::Status::OK();
}

senscord::Status LibcameraAdapter::GetImageProperty(
    senscord::ImageProperty *image_property, uint32_t channel_id) {
  if (channel_id == AITRIOS_SENSOR_CHANNEL_ID_INFERENCE_INPUT_IMAGE)
    *image_property = *it_image_property_;
  else
    *image_property = *full_image_property_;

  return senscord::Status::OK();
}

senscord::Status LibcameraAdapter::Configure(
    senscord::ImageProperty &image_property) {
  SENSCORD_LOG_INFO_TAGGED("libcamera", "LibcameraAdapter::Configure()");

  /* If InputTensor/OutputTensor output enable after executing STREAM_OFF, reopen device. */
  senscord::Status status = senscord::Status::OK();

  status = Close();
  if (!status.ok()) {
    return SENSCORD_STATUS_FAIL(
      "libcamera", senscord::Status::kCauseInvalidOperation,
      "Failed to close camera during reconfiguration");
  }

  status = Open(device_name_, util_, image_property);
  if (!status.ok()) {
    return SENSCORD_STATUS_FAIL(
      "libcamera", senscord::Status::kCauseInvalidOperation,
      "Failed to open the camera during reconfiguration");
  }

  if (!camera_) {
    return SENSCORD_STATUS_FAIL("libcamera",
                                senscord::Status::kCauseInvalidOperation, "");
  }

  options_->no_raw = false;
  options_->viewfinder_width = camera_image_width_;
  options_->viewfinder_height = camera_image_height_;
  options_->framerate = camera_frame_rate_;

  if (image_flip_.h && image_flip_.v) {
    options_->transform = libcamera::Transform::HVFlip;
  } else if (image_flip_.h) {
    options_->transform = libcamera::Transform::HFlip;
  } else if (image_flip_.v) {
    options_->transform = libcamera::Transform::VFlip;
  } else {
    options_->transform = libcamera::Transform::Identity;
  }

  std::unique_ptr<libcamera::CameraConfiguration> config =
      camera_->generateConfiguration({libcamera::StreamRole::Raw});

  libcam_->ConfigureViewfinder();

  if ((options_->viewfinder_mode.width == 0) ||
      (options_->viewfinder_mode.height == 0)) {
    return SENSCORD_STATUS_FAIL("libcamera",
                                senscord::Status::kCauseNotSupported,
                                "Not supported camera param: width %d, height %d, frame_rate %f",
                                camera_image_width_, camera_image_height_, camera_frame_rate_);
  }

  /* Control IMX500 Device Driver(v4l2-subdevice). */
  if (!UpdateImageCrop()) {
    return SENSCORD_STATUS_FAIL("libcamera",
                                senscord::Status::kCauseAborted,
                                "Failed to update crop parameters.");
  }

  // todo: add frame_rate, and multi-plane
  config.get()->at(0).size = {image_property.width, image_property.height};

  // convert Senscord Pixelformat definiton to libcamera::PixelFormat and pass
  // it to libcamera
  ConvertPixelFormat(&config.get()->at(0).pixelFormat,
                     image_property.pixel_format);

  switch (config.get()->validate()) {
    case libcamera::CameraConfiguration::Status::Valid:
      SENSCORD_LOG_INFO_TAGGED("libcamera",
                               "CameraConfiguration::Status::Valid");
      break;
    case libcamera::CameraConfiguration::Status::Adjusted:
      SENSCORD_LOG_INFO_TAGGED("libcamera",
                               "CameraConfiguration::Status::Adjusted");
      SENSCORD_LOG_INFO_TAGGED("libcamera", "%s",
                               config.get()->at(0).toString().c_str());
      image_property.stride_bytes = config.get()->at(0).stride;
      break;

    case libcamera::CameraConfiguration::Status::Invalid:
    default:
      return SENSCORD_STATUS_FAIL("libcamera",
                                  senscord::Status::kCauseInvalidArgument, "");
  }

  sensor_output_.width = camera_image_width_;
  sensor_output_.height = camera_image_height_;

  return senscord::Status::OK();
}

void LibcameraAdapter::RequestComplete(libcamera::Request *request) {
  // SENSCORD_LOG_INFO_TAGGED("libcamera",
  // "LibcameraAdapter::requestComplete()");
  static int seq_num = 0;
  // Check request status, continue if request is completed.
  switch (request->status()) {
    case libcamera::Request::Status::RequestComplete:
      break;
    case libcamera::Request::Status::RequestPending:
    case libcamera::Request::Status::RequestCancelled:
      return;
  }

  // Copy the frame buffer from the request
  senscord::FrameInfo frame = {};
  frame.sequence_number = seq_num++;
  senscord::osal::OSGetTime(&frame.sent_time);
  uint8_t stream_count = 0;
  for (auto buffer_map : request->buffers()) {
    senscord::ChannelRawData rawdata = {};
    rawdata.channel_id = stream_count++;  // senscord::kChannelIdImage(9);
    rawdata.captured_timestamp = frame.sent_time;
    rawdata.data_type = senscord::kRawDataTypeImage;
    rawdata.data_offset = 0;

    rawdata.data_size = 0;
    for (auto plane : buffer_map.second->planes()) {
      rawdata.data_size += plane.length;
    }

    senscord::MemoryAllocator *allocator = nullptr;
    senscord::Status status =
        util_->GetAllocator(senscord::kAllocatorDefaultKey, &allocator);
    if (!status.ok()) {
      SENSCORD_LOG_ERROR_TAGGED("libcamera",
                                "GetAllocator(kAllocatorDefaultKey) failed");
      break;
    }

    status = allocator->Allocate(rawdata.data_size, &rawdata.data_memory);
    if (!status.ok()) {
      SENSCORD_LOG_ERROR_TAGGED("libcamera",
                                "Allocate(rawdata.data_size) failed");
      break;
    }

    const int fd = buffer_map.second->planes().at(0).fd.get();
    void *src_ptr =
        ::mmap(nullptr, rawdata.data_size, PROT_READ, MAP_PRIVATE, fd, 0);
    if (src_ptr == MAP_FAILED) {
      break;
    } else {
      std::memcpy(reinterpret_cast<void *>(rawdata.data_memory->GetAddress()),
                  src_ptr, rawdata.data_size);
      ::munmap(src_ptr, rawdata.data_size);
    }
    frame.channels.push_back(rawdata);
  }

  {
    std::lock_guard<std::mutex> lock_frames(mutex_frames_);
    frames_.push_back(frame);
  }

  request->reuse(libcamera::Request::ReuseFlag::ReuseBuffers);

  {
    std::lock_guard<std::mutex> lock_controls(mutex_controls_);
    if (!controls_.empty()) {
      request->controls().merge(controls_);
      controls_.clear();
    }
  }

  camera_->queueRequest(request);
}

#if JPEG_LIB_VERSION_MAJOR > 9 || \
    (JPEG_LIB_VERSION_MAJOR == 9 && JPEG_LIB_VERSION_MINOR >= 4)
typedef size_t jpeg_mem_len_t;
#else
typedef unsigned long jpeg_mem_len_t;
#endif
static void YUV420_to_JPEG_fast(const uint8_t *input, int width, int height,
                                int stride, const int quality,
                                const unsigned int restart,
                                uint8_t *&jpeg_buffer,
                                jpeg_mem_len_t &enc_len) {
  struct jpeg_compress_struct cinfo;
  struct jpeg_error_mgr jerr;

  cinfo.err = jpeg_std_error(&jerr);
  jpeg_create_compress(&cinfo);

  cinfo.image_width = width;
  cinfo.image_height = height;
  cinfo.input_components = 3;
  cinfo.in_color_space = JCS_YCbCr;
  cinfo.restart_interval = restart;

  jpeg_set_defaults(&cinfo);
  cinfo.raw_data_in = TRUE;
  jpeg_set_quality(&cinfo, quality, TRUE);
  jpeg_buffer = NULL;
  enc_len = 0;
  jpeg_mem_dest(&cinfo, &jpeg_buffer, &enc_len);
  jpeg_start_compress(&cinfo, TRUE);

  int stride2 = stride / 2;
  uint8_t *Y = (uint8_t *)input;
  uint8_t *U = (uint8_t *)Y + stride * height;
  uint8_t *V = (uint8_t *)U + stride2 * (height / 2);
  uint8_t *Y_max = U - stride;
  uint8_t *U_max = V - stride2;
  uint8_t *V_max = U_max + stride2 * (height / 2);

  JSAMPROW y_rows[16];
  JSAMPROW u_rows[8];
  JSAMPROW v_rows[8];

  for (uint8_t *Y_row = Y, *U_row = U, *V_row = V;
       cinfo.next_scanline < height;) {
    for (int i = 0; i < 16; i++, Y_row += stride)
      y_rows[i] = std::min(Y_row, Y_max);
    for (int i = 0; i < 8; i++, U_row += stride2, V_row += stride2)
      u_rows[i] = std::min(U_row, U_max), v_rows[i] = std::min(V_row, V_max);

    JSAMPARRAY rows[] = {y_rows, u_rows, v_rows};
    jpeg_write_raw_data(&cinfo, rows, 16);
  }

  jpeg_finish_compress(&cinfo);
  jpeg_destroy_compress(&cinfo);
}

static void pack_rgb888_planar_to_interleaved(uint8_t *planar_data,
                                              uint8_t *output, int width,
                                              int height, int32_t norm_val_[],
                                              uint32_t norm_shift_[],
                                              int32_t div_val_[],
                                              uint32_t div_shift_) {
  int total_pixels = width * height;
  uint8_t *r_plane = planar_data;
  uint8_t *g_plane = planar_data + total_pixels;
  uint8_t *b_plane = planar_data + 2 * total_pixels;

  for (int i = 0; i < total_pixels; i++) {
    uint8_t rgb[3] = {r_plane[i], g_plane[i], b_plane[i]};
    for (int j = 0; j < 3; j++) {
      int32_t sample = static_cast<int32_t>(rgb[j]);
      sample = (sample << norm_shift_[j]) - norm_val_[j];
      sample = ((sample << div_shift_) / div_val_[j]) & 0xFF;
      output[i * 3 + j] = static_cast<uint8_t>(sample);
    }
  }
}

static void RGB888_to_JPEG_fast(const uint8_t *input, int width, int height,
                                int stride, const int quality,
                                const unsigned int restart,
                                uint8_t *&jpeg_buffer,
                                jpeg_mem_len_t &enc_len) {
  struct jpeg_compress_struct cinfo;
  struct jpeg_error_mgr jerr;

  // Set up error handling
  cinfo.err = jpeg_std_error(&jerr);
  jpeg_create_compress(&cinfo);

  // Configure basic image properties
  cinfo.image_width = width;
  cinfo.image_height = height;
  cinfo.input_components = 3;      // 3 channels for RGB
  cinfo.in_color_space = JCS_RGB;  // Set color space to RGB
  cinfo.restart_interval = restart;

  jpeg_set_defaults(&cinfo);
  jpeg_set_quality(&cinfo, quality, TRUE);
  jpeg_buffer = NULL;
  enc_len = 0;

  // Set the destination to memory buffer
  jpeg_mem_dest(&cinfo, &jpeg_buffer, &enc_len);

  // Start compression
  jpeg_start_compress(&cinfo, TRUE);

  // Write each row of the RGB image data to the JPEG file
  JSAMPROW row_pointer[1];
  while (cinfo.next_scanline < cinfo.image_height) {
    row_pointer[0] = (JSAMPROW)&input[cinfo.next_scanline * stride];
    jpeg_write_scanlines(&cinfo, row_pointer, 1);
  }

  // Complete the compression process
  jpeg_finish_compress(&cinfo);
  jpeg_destroy_compress(&cinfo);
}

void LibcameraAdapter::GetFrames(std::vector<senscord::FrameInfo> *frames, bool dry_run) {
  static int seq_num = 0;
  senscord::FrameInfo frame = {};
  CompletedRequestPtr payload;
  frame.sequence_number = seq_num++;
  senscord::osal::OSGetTime(&frame.sent_time);

  while (1) {
    RPiCamApp::Msg msg = libcam_->Wait();
    if (msg.type == RPiCamApp::MsgType::Quit) {
      SENSCORD_LOG_ERROR("Quit message received");
      break;
    } else if (msg.type != RPiCamApp::MsgType::RequestComplete) {
      return;
    }

    if (dry_run) {
      SENSCORD_LOG_INFO("dry_run is enabled");
      frames_.push_back(frame);
      *frames = frames_;
      frames_.clear();
      return;
    }

    if (count_drop_frames_ > MAX_NUM_DROP_FRAMES) {
      payload = std::get<CompletedRequestPtr>(msg.payload);
      break;
    }

    count_drop_frames_++;
  }

  senscord::MemoryAllocator *allocator = nullptr;
  senscord::Status status =
      util_->GetAllocator(senscord::kAllocatorDefaultKey, &allocator);

  // Fill the frame info with the ControlList items and ancillary bits.
  uint64_t timestamp = 0;
  senscord::osal::OSGetTime(&timestamp);

  auto output = payload->metadata.get(controls::rpi::CnnOutputTensor);
  auto input = payload->metadata.get(controls::rpi::CnnInputTensor);
  auto *input_info = reinterpret_cast<const CnnInputTensorInfo *>(
      payload->metadata.get(controls::rpi::CnnInputTensorInfo)->data());

  // Output Tensor
  if (output) {
    std::vector<float> output_tensor(output->data(),
                                     output->data() + output->size());
    senscord::ChannelRawData rawdata1 = {};
    rawdata1.channel_id =
        AITRIOS_SENSOR_CHANNEL_ID_INFERENCE_OUTPUT;  // senscord::kChannelIdImage(9);
    rawdata1.captured_timestamp = timestamp;
    rawdata1.data_type = senscord::kRawDataTypeMeta;
    rawdata1.data_offset = 0;
    rawdata1.data_size = output_tensor.size() * sizeof(float);
    status = allocator->Allocate(rawdata1.data_size, &rawdata1.data_memory);
    if (!status.ok()) {
      SENSCORD_LOG_ERROR_TAGGED("libcamera",
                                "Allocate(rawdata1.data_size) failed");
    }
    std::memcpy(reinterpret_cast<void *>(rawdata1.data_memory->GetAddress()),
                output_tensor.data(), rawdata1.data_size);
    frame.channels.push_back(rawdata1);

    // Get the output tensor information and update the tensor shapes property
    UpdateTensorShapesProperty(payload);
  }

  // Input Tensor
  if (input && input_info) {
    senscord::ChannelRawData rawdata2 = {};
    rawdata2.channel_id =
        AITRIOS_SENSOR_CHANNEL_ID_INFERENCE_INPUT_IMAGE;  // senscord::kChannelIdImage(9);
    std::vector<uint8_t> input_tensor(input->data(),
                                      input->data() + input->size());
    jpeg_mem_len_t enc_len;
    uint8_t *enc_buffer = nullptr;
    uint8_t *temp = (uint8_t *)malloc(input_tensor.size());
    pack_rgb888_planar_to_interleaved(
        input_tensor.data(), temp, input_info->width, input_info->height,
        norm_val_, norm_shift_, div_val_, div_shift_);

    if (strncmp(it_image_property_->pixel_format.c_str(), "image_jp",
                strlen("image_jp")) == 0) {
      RGB888_to_JPEG_fast(temp, input_info->width, input_info->height,
                          input_info->width * 3, 93, 0, enc_buffer, enc_len);
      free(temp);
    } else {
      it_image_property_->pixel_format = "image_rgb24";
      enc_buffer = temp;
      enc_len = input_tensor.size();
    }

    rawdata2.captured_timestamp = timestamp;
    rawdata2.data_type = senscord::kRawDataTypeImage;
    rawdata2.data_offset = 0;
    rawdata2.data_size = enc_len;
    status = allocator->Allocate(rawdata2.data_size, &rawdata2.data_memory);
    if (!status.ok()) {
      SENSCORD_LOG_ERROR_TAGGED("libcamera",
                                "Allocate(input_tensor.size) failed");
    }
    std::memcpy(reinterpret_cast<void *>(rawdata2.data_memory->GetAddress()),
                enc_buffer, rawdata2.data_size);
    if (enc_buffer) {
      free(enc_buffer);
    }
    frame.channels.push_back(rawdata2);
    it_image_property_->width = input_info->width;
    it_image_property_->height = input_info->height;
    it_image_property_->stride_bytes = input_info->width * 3;
    util_->UpdateChannelProperty(
        AITRIOS_SENSOR_CHANNEL_ID_INFERENCE_INPUT_IMAGE,
        senscord::kImagePropertyKey, it_image_property_);
  }

  libcamera::Stream *stream = libcam_->ViewfinderStream();
  StreamInfo info = libcam_->GetStreamInfo(stream);
  libcamera::FrameBuffer *buffer = payload->buffers[stream];
  BufferReadSync r(libcam_, buffer);
  libcamera::Span span = r.Get()[0];
  // ViewFinder
  const int fd = buffer->planes()[0].fd.get();
  void *src_ptr = ::mmap(nullptr, span.size(), PROT_READ, MAP_SHARED, fd, 0);
  if (src_ptr == MAP_FAILED) {
    struct stat s;
    fstat(fd, &s);
    SENSCORD_LOG_ERROR_TAGGED(
        "libcamera", "mmap() failed: %s -> fd: %d, size: %d, offset: %d",
        strerror(errno), fd, span.size(), 0);
    SENSCORD_LOG_ERROR_TAGGED("libcamera", "fstat(%d): st_size:%d", fd,
                              s.st_size);
  }

  jpeg_mem_len_t enc_len;
  uint8_t *enc_buffer = nullptr;

  if (strncmp(full_image_property_->pixel_format.c_str(), "image_jp",
              strlen("image_jp")) == 0) {
    if (info.pixel_format == libcamera::formats::BGR888) {
      SENSCORD_LOG_DEBUG_TAGGED("libcamera", "RGB888_to_JPEG_fast");
      RGB888_to_JPEG_fast((uint8_t *)src_ptr, info.width, info.height,
                          info.stride, 93, 0, enc_buffer, enc_len);
    } else if (info.pixel_format == libcamera::formats::YUV420) {
      SENSCORD_LOG_DEBUG_TAGGED("libcamera", "YUV420_to_JPEG_fast");
      YUV420_to_JPEG_fast((uint8_t *)src_ptr, info.width, info.height,
                          info.stride, 93, 0, enc_buffer, enc_len);
    } else {
      SENSCORD_LOG_ERROR_TAGGED("libcamera", "Unsupported pixel format");
      return;
    }
  } else {
    enc_len = (jpeg_mem_len_t)span.size();
    enc_buffer = (uint8_t *)src_ptr;
  }
  // ViewFinder Channel
  senscord::ChannelRawData rawdata0 = {};
  rawdata0.channel_id = AITRIOS_SENSOR_CHANNEL_ID_INFERENCE_RAW_IMAGE;
  rawdata0.captured_timestamp = timestamp;
  rawdata0.data_type = senscord::kRawDataTypeImage;
  rawdata0.data_offset = 0;
  rawdata0.data_size = enc_len;

  status = allocator->Allocate(rawdata0.data_size, &rawdata0.data_memory);
  if (!status.ok()) {
    SENSCORD_LOG_ERROR_TAGGED("libcamera",
                              "Allocate(rawdata0.data_size) failed");
  } else {
    std::memcpy(reinterpret_cast<void *>(rawdata0.data_memory->GetAddress()),
                enc_buffer, rawdata0.data_size);
    if (enc_buffer && (src_ptr != enc_buffer)) {
      free(enc_buffer);
    }
    ::munmap(src_ptr, span.size());
  }

  frame.channels.push_back(rawdata0);
  full_image_property_->width = info.width;
  full_image_property_->height = info.height;
  full_image_property_->stride_bytes = info.stride;
  util_->UpdateChannelProperty(AITRIOS_SENSOR_CHANNEL_ID_INFERENCE_RAW_IMAGE,
                               senscord::kImagePropertyKey,
                               full_image_property_);

  std::lock_guard<std::mutex> lock_frames(mutex_frames_);
  frames_.push_back(frame);
  *frames = frames_;
  frames_.clear();
  return;
}

void LibcameraAdapter::UpdateTensorShapesProperty(CompletedRequestPtr payload) {
  auto output_info = payload->metadata.get(controls::rpi::CnnOutputTensorInfo);
  if (!output_info) {
    SENSCORD_LOG_ERROR_TAGGED("libcamera",
                              "No CNN output tensor info found in metadata");
    return;
  }

  const CnnOutputTensorInfo output_info_ptr =
      *reinterpret_cast<const CnnOutputTensorInfo *>(output_info->data());

  SENSCORD_LOG_DEBUG_TAGGED(
      "libcamera", "CnnOutputTensorInfo: networkName=%s, numTensors=%d",
      output_info_ptr.networkName, output_info_ptr.numTensors);

  if (output_info_ptr.numTensors > MaxNumTensors) {
    SENSCORD_LOG_ERROR_TAGGED("libcamera",
                              "numTensors(%d) exceeds MaxNumTensors(%d)",
                              output_info_ptr.numTensors, MaxNumTensors);
    return;
  }

  tensor_shapes_property_ = {};
  uint32_t shapes_array_index = 0;

  for (uint32_t i = 0; i < output_info_ptr.numTensors; ++i) {
    const OutputTensorInfo &tensor_info = output_info_ptr.info[i];
    uint32_t num_dimensions = tensor_info.numDimensions;

    SENSCORD_LOG_DEBUG_TAGGED(
        "libcamera", "OutputTensorInfo[%d]: tensorDataNum=%d, numDimensions=%d",
        i, tensor_info.tensorDataNum, tensor_info.numDimensions);

    if (num_dimensions > MaxNumDimensions) {
      SENSCORD_LOG_ERROR_TAGGED(
          "libcamera", "numDimensions(%d) exceeds MaxNumDimensions(%d)",
          num_dimensions, MaxNumDimensions);
      return;
    }

    tensor_shapes_property_.shapes_array[shapes_array_index++] = num_dimensions;
    for (uint32_t j = 0; j < num_dimensions; ++j) {
      SENSCORD_LOG_DEBUG_TAGGED("libcamera", "info[%d]: size=%d", j,
                                tensor_info.size[j]);
      tensor_shapes_property_.shapes_array[shapes_array_index++] =
          tensor_info.size[j];
    }
  }

  tensor_shapes_property_.tensor_count = output_info_ptr.numTensors;
  util_->UpdateChannelProperty(
      AITRIOS_SENSOR_CHANNEL_ID_INFERENCE_OUTPUT,
      senscord::libcamera_image::kLibcameraTensorShapesPropertyKey,
      &tensor_shapes_property_);
}

senscord::Status LibcameraAdapter::ReleaseFrame(
    const senscord::FrameInfo &frameinfo,
    const std::vector<uint32_t> *referenced_channel_ids) {
  for (auto channel : frameinfo.channels) {
    channel.data_memory->GetAllocator()->Free(channel.data_memory);
  }
  return senscord::Status::OK();
}

senscord::Status LibcameraAdapter::GetProperty(
    senscord::ImageSensorFunctionSupportedProperty *property) {
  *property = image_sensor_function_supported_property_;
  return senscord::Status::OK();
}

senscord::Status LibcameraAdapter::GetProperty(
    senscord::libcamera_image::ImageRotationProperty *property) {
  *property = rotation_property_;
  return senscord::Status::OK();
}

senscord::Status LibcameraAdapter::SetProperty(
    const senscord::libcamera_image::ImageRotationProperty *property) {
  switch (property->rotation_angle) {
    case SENSOR_ROTATION_ANGLE_0_DEG:
    case SENSOR_ROTATION_ANGLE_90_DEG:
    case SENSOR_ROTATION_ANGLE_180_DEG:
    case SENSOR_ROTATION_ANGLE_270_DEG:
      rotation_property_ = *property;
      break;
    default:
      return SENSCORD_STATUS_FAIL("libcamera", senscord::Status::kCauseInvalidArgument,
                                  "Invalid rotation value : %d", property->rotation_angle);
  }
  UpdateImageRotationProperty();
  return senscord::Status::OK();
}

senscord::Status LibcameraAdapter::SetProperty(
    const senscord::libcamera_image::AIModelBundleIdProperty *property) {
  std::string bundle_id = property->ai_model_bundle_id;
  std::string post_process_file =
      HandleCustomJsonPathString(options_->post_process_file, bundle_id);
  if (post_process_file.length()) {
    options_->post_process_file = post_process_file;
    ai_model_bundle_id_ = bundle_id;
  } else {
    return SENSCORD_STATUS_FAIL(
        "libcamera", senscord::Status::kCauseInvalidOperation,
        "The set ai_model_bundle_id(%s) is invalid.",
        bundle_id.c_str());
  }

  return senscord::Status::OK();
}

senscord::Status LibcameraAdapter::GetProperty(
    senscord::libcamera_image::CameraTemperatureProperty *property) {
  senscord::Status status;
  uint8_t reg_value = 0;
  int8_t temp_value = 0;

  status = reg_handle_.ReadRegister(kRegTemperatureEnable, &reg_value);
  if (!status.ok()) {
    return SENSCORD_STATUS_FAIL("libcamera", senscord::Status::kCauseHardwareError,
                                "Failed to read temperature enable register");
  }
  if ((reg_value & kRegTemperatureEnableMask) == 0) {
    return SENSCORD_STATUS_FAIL(
        "libcamera", senscord::Status::kCauseInvalidOperation,
        "Temperature measurement is disabled");
  }

  status = reg_handle_.ReadRegister(kRegTemperatureValue, &reg_value);
  if (!status.ok()) {
    return SENSCORD_STATUS_FAIL("libcamera", senscord::Status::kCauseHardwareError,
                                "Failed to read temperature value register");
  }
  temp_value = static_cast<int8_t>(reg_value);
  if (temp_value < kRegTemperatureMin) {
    SENSCORD_LOG_INFO("Temperature is under range[%d]", temp_value);
    temp_value = kRegTemperatureMin;
  } else if (temp_value > kRegTemperatureMax) {
    SENSCORD_LOG_INFO("Temperature is over range[%d]", temp_value);
    temp_value = kRegTemperatureMax;
  }
  // Only use Sensor-ID : 0 (IMX500 sensor celsius)
  property->temperatures[kImx500SensorId] = {static_cast<float>(temp_value), "IMX500 sensor celsius"};

  SENSCORD_LOG_INFO("Temperature: [%d/%f]", temp_value, property->temperatures[kImx500SensorId].temperature);

  return senscord::Status::OK();
}

senscord::Status LibcameraAdapter::GetControl(
    senscord::libcamera_image::AccessProperty *property) {
  senscord::Status status;

  // return if current_control_values_ has the value of control
  auto itr = current_control_values_.find(property->id);
  if (itr != current_control_values_.end()) {
    property->value = itr->second;
    return senscord::Status::OK();
  }

  // find default value of the control
  const libcamera::ControlValue *default_value = nullptr;
  for (auto ctrl : camera_->controls()) {
    if (ctrl.first->name() == property->id) {
      default_value = &ctrl.second.def();
      break;
    }
  }

  if (default_value == nullptr) {
    return SENSCORD_STATUS_FAIL(
        "libcamera", senscord::Status::kCauseNotSupported, "control not found");
  }

  status = ConvertValue(default_value->type(), *default_value, property->value);
  if (status.ok()) {
    current_control_values_[property->id] = property->value;
  }

  return status;
}

senscord::Status LibcameraAdapter::GetProperty(
    senscord::libcamera_image::AccessProperty *property) {
  // find "property_name" => libcamra::control_id
  const libcamera::ControlId *target_id = nullptr;
  {
    auto properties = libcamera::properties::properties;
    auto itr = std::find_if(
        properties.begin(), properties.end(),
        [property](auto p) { return property->id == p.second->name(); });
    if (itr == properties.end()) {
      return SENSCORD_STATUS_FAIL("libcamera",
                                  senscord::Status::kCauseInvalidArgument,
                                  "property id not found");
    }
    target_id = itr->second;
  }

  const libcamera::ControlValue &libcamera_property_value =
      camera_->properties().get(target_id->id());
  return ConvertValue(target_id->type(), libcamera_property_value,
                      property->value);
}

senscord::Status LibcameraAdapter::GetDevices(
    senscord::libcamera_image::DeviceEnumerationProperty *property) {
  std::lock_guard<std::mutex> lock(LibcameraAdapter::mutex_camera_manager_);

  for (auto camera : camera_manager_->cameras()) {
    senscord::libcamera_image::DeviceEnumerationProperty::LibcameraDevice
        device = {};
    device.id = camera->id();
    device.name =
        camera->properties().get(libcamera::properties::Model).value();
    property->devices.push_back(device);
  }
  return senscord::Status::OK();
}

senscord::Status LibcameraAdapter::GetAIModelVersion(std::string &ai_model_version) {
  std::string j_str = ReadPostProcessJsonString(options_->post_process_file);
  if (j_str.empty()) {
    return SENSCORD_STATUS_FAIL("libcamera",
                              senscord::Status::kCauseNotSupported,
                              "The json parameter for PostProcess is empty.");
  }

  std::string file_name;
  std::string rpk_path = GetRpkPath(j_str);
  size_t pos = rpk_path.find_last_of("/\\");

  if (pos != std::string::npos) {
    file_name = rpk_path.substr(pos + 1);
  } else {
    file_name = rpk_path;
  }

  size_t first = file_name.find("_");
  size_t second = file_name.find("_", first + 1);
  ai_model_version = file_name.substr(first + 1, second - first - 1);

  return senscord::Status::OK();
}

senscord::Status LibcameraAdapter::SetExposureMode(ExposureModeParam mode) {
  senscord::Status status;

  switch (mode) {
    case kExposureModeParamAuto:
      options_->exposure_index = 0;
      manual_exposure_.keep = false;
      break;
    case kExposureModeParamGainFix:
      return SENSCORD_STATUS_FAIL("libcamera",
                              senscord::Status::kCauseNotSupported,
                              "ExposureModeGainFix is not supported.");
    case kExposureModeParamTimeFix:
      return SENSCORD_STATUS_FAIL("libcamera",
                              senscord::Status::kCauseNotSupported,
                              "ExposureModeTimeFix is not supported.");
    case kExposureModeParamManual:
      options_->exposure_index = 4;
      break;
    case kExposureModeParamHold:
      /* Check if the previous value is retained. */
      if (manual_exposure_.keep) {
        options_->exposure_index = 4;

        /* Set the previous value. */
        status = SetManualExposureParam(
              manual_exposure_.exposure_time,
              manual_exposure_.gain);
        if (!status.ok()) {
          return status;
        }
      } else {
        return SENSCORD_STATUS_FAIL("libcamera",
                              senscord::Status::kCauseInvalidOperation,
                              "ManualExposure was not executed previously.");
      }

      break;
    default:
      return SENSCORD_STATUS_FAIL("libcamera",
                              senscord::Status::kCauseNotSupported,
                              "mode(%d) is not supported.", mode);
  }

  exposure_mode_ = mode;

  return senscord::Status::OK();
}

senscord::Status LibcameraAdapter::SetAutoExposureParam(
    uint32_t &max_exposure_time,
    uint32_t &min_exposure_time,
    float &max_gain,
    uint32_t &convergence_speed) {
  /* There is no interface for configuring it in libcamera. */
  return SENSCORD_STATUS_FAIL("libcamera",
                          senscord::Status::kCauseNotSupported,
                          "The AutoExposure parameter is not supported.");
}

senscord::Status LibcameraAdapter::SetAeEvCompensation(
    float &ev_compensation) {
  options_->ev = ev_compensation;
  return senscord::Status::OK();
}

senscord::Status LibcameraAdapter::SetAeAntiFlickerMode(
    AeAntiFlickerMode mode) {
  switch (mode) {
    case kAeAntiFlickerModeOff:
      options_->flicker_period.set("0us");
      break;
    case kAeAntiFlickerModeAuto:
      return SENSCORD_STATUS_FAIL("libcamera",
                              senscord::Status::kCauseNotSupported,
                              "mode(%d) is not supported.", mode);

    case kAeAntiFlickerModeForce50Hz:
      options_->flicker_period.set(AE_FLICKER_PERIOD_50HZ);
      break;
    case kAeAntiFlickerModeForce60Hz:
      options_->flicker_period.set(AE_FLICKER_PERIOD_60HZ);
      break;
    default:
      return SENSCORD_STATUS_FAIL("libcamera",
                              senscord::Status::kCauseNotSupported,
                              "mode(%d) is not supported.", mode);
  }

  return senscord::Status::OK();
}

senscord::Status LibcameraAdapter::SetAeMetering(
    AeMeteringMode mode,
    AeMeteringWindow &window) {
  /* There is no interface for configuring it in libcamera. */
    return SENSCORD_STATUS_FAIL("libcamera",
                          senscord::Status::kCauseNotSupported,
                          "The AeMetering parameter is not supported.");
}

senscord::Status LibcameraAdapter::SetManualExposureParam(
    uint32_t exposure_time,
    float gain) {
  if (options_->exposure_index == 4) {
    std::string shutter_str = std::to_string(exposure_time) + "us";
    options_->shutter.set(shutter_str);
    options_->gain = gain;

    manual_exposure_.keep = true;
    manual_exposure_.exposure_time = exposure_time;
    manual_exposure_.gain = gain;
  }

  return senscord::Status::OK();
}

senscord::Status LibcameraAdapter::SetImageSize(
    uint32_t width,
    uint32_t height) {
  camera_image_width_ = width;
  camera_image_height_ = height;
  return senscord::Status::OK();
}

senscord::Status LibcameraAdapter::SetFrameRate(
    uint32_t num,
    uint32_t denom) {
  if (denom != 0) {
    float rate = (float)num / (float)denom;
    camera_frame_rate_ = rate;
  } else {
    return SENSCORD_STATUS_FAIL("libcamera",
                              senscord::Status::kCauseInvalidArgument,
                              "denominator is zero.");
  }

  return senscord::Status::OK();
}

senscord::Status LibcameraAdapter::SetImageFlip(
    bool flip_horizontal,
    bool flip_vertical) {
  image_flip_.h = flip_horizontal;
  image_flip_.v = flip_vertical;
  return senscord::Status::OK();
}

senscord::Status LibcameraAdapter::SetImageCrop(
    uint32_t left,
    uint32_t top,
    uint32_t width,
    uint32_t height) {
  image_crop_.x = left;
  image_crop_.y = top;
  image_crop_.w = width;
  image_crop_.h = height;

  if (!UpdateImageCrop()) {
    return SENSCORD_STATUS_FAIL("libcamera",
                                senscord::Status::kCauseAborted,
                                "Failed to update crop parameters.");
  }

  return senscord::Status::OK();
}

senscord::Status LibcameraAdapter::SetControl(
    const senscord::libcamera_image::AccessProperty *property) {
  const libcamera::ControlId *control_id = nullptr;
  libcamera::ControlInfo control_info;

  senscord::Status status =
      FindLibcameraControlId(property->id, &control_id, &control_info);
  if (!status.ok()) {
    return status;
  }

  status = SetLibcameraControl(*control_id, control_info, property->value);
  if (status.ok()) {
    current_control_values_[property->id] = property->value;
  }

  return status;
}

template <typename T, std::size_t N>
senscord::Status LibcameraAdapter::SetLibcameraControl(
    const libcamera::ControlId &control_id,
    const libcamera::ControlInfo &control_info, const AnyValue &value) {
  T raw_value;
  senscord::Status status = value.Get<T>(&raw_value);
  if (status.ok()) {
    return SetLibcameraControl<T>(control_id, control_info, raw_value);
  }

  std::vector<T> vec;
  status = value.Get<std::vector<T>>(&vec);
  if (status.ok()) {
    T *arr = new T[vec.size()];
    for (int i = 0; i < vec.size(); i++) {
      arr[i] = vec[i];
    }
    libcamera::Span<const T, N> span(arr, vec.size());
    status = SetLibcameraControl<libcamera::Span<const T, N>>(
        control_id, control_info, span);
    delete[] arr;
  }

  return status;
}

senscord::Status LibcameraAdapter::SetLibcameraControl(
    const libcamera::ControlId &control_id,
    const libcamera::ControlInfo &control_info, const AnyValue &value) {
  senscord::Status status;

  switch (control_id.type()) {
    case libcamera::ControlType::ControlTypeBool:
      status = SetLibcameraControl<bool>(control_id, control_info, value);
      break;

    case libcamera::ControlType::ControlTypeByte:
      status = SetLibcameraControl<uint8_t>(control_id, control_info, value);
      break;

    case libcamera::ControlType::ControlTypeInteger32:
      status = SetLibcameraControl<int32_t>(control_id, control_info, value);
      break;

    case libcamera::ControlType::ControlTypeInteger64:
      status = SetLibcameraControl<int64_t>(control_id, control_info, value);
      break;

    case libcamera::ControlType::ControlTypeFloat:
      status = SetLibcameraControl<float>(control_id, control_info, value);
      break;

    case libcamera::ControlType::ControlTypeString:
      status =
          SetLibcameraControl<std::string>(control_id, control_info, value);
      break;

    case libcamera::ControlType::ControlTypeRectangle:
    case libcamera::ControlType::ControlTypeSize:
      return SENSCORD_STATUS_FAIL(
          "libcamera", senscord::Status::kCauseNotSupported,
          "not implemented ControlType: %s", control_id.name().c_str());

    case libcamera::ControlType::ControlTypeNone:
    default:
      return SENSCORD_STATUS_FAIL("libcamera",
                                  senscord::Status::kCauseInvalidArgument, "");
  }

  return status;
}

template <typename T, std::size_t N>
senscord::Status LibcameraAdapter::SetLibcameraControl(
    const libcamera::ControlId &control_id,
    const libcamera::ControlInfo &control_info, const T &value) {
  if constexpr (!libcamera::details::is_span<T>::value) {
    if ((control_id.type() != libcamera::ControlType::ControlTypeString) &&
        !(control_info.min().get<T>() <= value &&
          value <= control_info.max().get<T>())) {
      return SENSCORD_STATUS_FAIL(
          "libcamera", senscord::Status::kCauseOutOfRange, "%s: %s",
          control_id.name().c_str(), control_info.toString().c_str());
    }
  }

  {
    std::lock_guard<std::mutex> lock(mutex_controls_);
    controls_.set(*static_cast<const libcamera::Control<T> *>(&control_id),
                  value);
  }

  return senscord::Status::OK();
}

// explicit instantiation for SetLibcameraControl<T,N>()
template senscord::Status LibcameraAdapter::SetLibcameraControl<bool>(
    const libcamera::ControlId &control_id,
    const libcamera::ControlInfo &control_info, const bool &value);
template senscord::Status LibcameraAdapter::SetLibcameraControl<float>(
    const libcamera::ControlId &control_id,
    const libcamera::ControlInfo &control_info, const float &value);
template senscord::Status LibcameraAdapter::SetLibcameraControl<int32_t>(
    const libcamera::ControlId &control_id,
    const libcamera::ControlInfo &control_info, const int32_t &value);
template senscord::Status LibcameraAdapter::SetLibcameraControl<int64_t>(
    const libcamera::ControlId &control_id,
    const libcamera::ControlInfo &control_info, const int64_t &value);

template senscord::Status
LibcameraAdapter::SetLibcameraControl<libcamera::Span<const float, 2>>(
    const libcamera::ControlId &control_id,
    const libcamera::ControlInfo &control_info,
    const libcamera::Span<const float, 2> &value);
template senscord::Status
LibcameraAdapter::SetLibcameraControl<libcamera::Span<const float, 9>>(
    const libcamera::ControlId &control_id,
    const libcamera::ControlInfo &control_info,
    const libcamera::Span<const float, 9> &value);
template senscord::Status
LibcameraAdapter::SetLibcameraControl<libcamera::Span<const int32_t, 4>>(
    const libcamera::ControlId &control_id,
    const libcamera::ControlInfo &control_info,
    const libcamera::Span<const int32_t, 4> &value);
template senscord::Status
LibcameraAdapter::SetLibcameraControl<libcamera::Span<const int64_t, 2>>(
    const libcamera::ControlId &control_id,
    const libcamera::ControlInfo &control_info,
    const libcamera::Span<const int64_t, 2> &value);

template senscord::Status LibcameraAdapter::SetLibcameraControl<float, 2>(
    const libcamera::ControlId &control_id,
    const libcamera::ControlInfo &control_info,
    const senscord::libcamera_image::AnyValue &value);
template senscord::Status LibcameraAdapter::SetLibcameraControl<float, 9>(
    const libcamera::ControlId &control_id,
    const libcamera::ControlInfo &control_info,
    const senscord::libcamera_image::AnyValue &value);
template senscord::Status LibcameraAdapter::SetLibcameraControl<int32_t, 4>(
    const libcamera::ControlId &control_id,
    const libcamera::ControlInfo &control_info,
    const senscord::libcamera_image::AnyValue &value);
template senscord::Status LibcameraAdapter::SetLibcameraControl<int64_t, 2>(
    const libcamera::ControlId &control_id,
    const libcamera::ControlInfo &control_info,
    const senscord::libcamera_image::AnyValue &value);

senscord::Status LibcameraAdapter::FindLibcameraControlId(
    const std::string &control_name, const libcamera::ControlId **control_id,
    libcamera::ControlInfo *control_info) {
  if (!camera_) {
    return SENSCORD_STATUS_FAIL(
        "libcamera", senscord::Status::kCauseInvalidOperation, "Not opened");
  }

  for (auto ctrl_pair : camera_->controls()) {
    if (ctrl_pair.first->name() == control_name) {
      *control_id = ctrl_pair.first;
      *control_info = ctrl_pair.second;
      return senscord::Status::OK();
    }
  }

  return SENSCORD_STATUS_FAIL("libcamera",
                              senscord::Status::kCauseInvalidArgument,
                              "control id not found");
}

void LibcameraAdapter::ConvertPixelFormat(libcamera::PixelFormat *format,
                                          const std::string &pixel_format) {
  constexpr static struct {
    const libcamera::PixelFormat &libcamera_pixelformat;
    const char *senscord_pixelformat;
  } convert_table[] = {
      {libcamera::formats::R8, senscord::kPixelFormatGREY},
      {libcamera::formats::R10, senscord::kPixelFormatY10},
      {libcamera::formats::R12, senscord::kPixelFormatY12},
      // {libcamera::formats::RGB565, ""},
      // {libcamera::formats::RGB565_BE, ""},
      {libcamera::formats::BGR888, senscord::kPixelFormatRGB24},
      {libcamera::formats::BGR888, senscord::kPixelFormatBGR24},
      {libcamera::formats::XRGB8888, senscord::kPixelFormatXRGB32},
      {libcamera::formats::XBGR8888, senscord::kPixelFormatXRGB444},
      // {libcamera::formats::RGBX8888, ""},
      // {libcamera::formats::BGRX8888, ""},
      {libcamera::formats::ARGB8888, senscord::kPixelFormatARGB32},
      {libcamera::formats::ABGR8888, senscord::kPixelFormatABGR32},
      // {libcamera::formats::RGBA8888, ""},
      // {libcamera::formats::BGRA8888, ""},
      {libcamera::formats::YUYV, senscord::kPixelFormatYUYV},
      // {libcamera::formats::YVYU, ""},
      {libcamera::formats::UYVY, senscord::kPixelFormatUYVY},
      // {libcamera::formats::VYUY, ""},
      // {libcamera::formats::AVUY8888, ""},
      // {libcamera::formats::XVUY8888, ""},
      {libcamera::formats::NV12, senscord::kPixelFormatNV12},
      // {libcamera::formats::NV21, ""},
      {libcamera::formats::NV16, senscord::kPixelFormatNV16},
      // {libcamera::formats::NV61, ""},
      // {libcamera::formats::NV24, ""},
      // {libcamera::formats::NV42, ""},
      {libcamera::formats::YUV420, senscord::kPixelFormatYUV420},
      // {libcamera::formats::YVU420, ""},
      // {libcamera::formats::YUV422, ""},
      // {libcamera::formats::YVU422, ""},
      {libcamera::formats::YUV444, senscord::kPixelFormatYUV444},
      // {libcamera::formats::YVU444, ""},
      {libcamera::formats::MJPEG, senscord::kPixelFormatJPEG},
      {libcamera::formats::SRGGB8, senscord::kPixelFormatSRGGB8},
      {libcamera::formats::SGRBG8, senscord::kPixelFormatSGRBG8},
      {libcamera::formats::SGBRG8, senscord::kPixelFormatSGBRG8},
      {libcamera::formats::SBGGR8, senscord::kPixelFormatSBGGR8},
      {libcamera::formats::SRGGB10, senscord::kPixelFormatSRGGB10},
      {libcamera::formats::SGRBG10, senscord::kPixelFormatSGRBG10},
      {libcamera::formats::SGBRG10, senscord::kPixelFormatSGBRG10},
      {libcamera::formats::SBGGR10, senscord::kPixelFormatSBGGR10},
      {libcamera::formats::SRGGB12, senscord::kPixelFormatSRGGB12},
      {libcamera::formats::SGRBG12, senscord::kPixelFormatSGRBG12},
      {libcamera::formats::SGBRG12, senscord::kPixelFormatSGBRG12},
      {libcamera::formats::SBGGR12, senscord::kPixelFormatSBGGR12},
      // {libcamera::formats::SRGGB14, ""},
      // {libcamera::formats::SGRBG14, ""},
      // {libcamera::formats::SGBRG14, ""},
      // {libcamera::formats::SBGGR14, ""},
      // {libcamera::formats::SRGGB16, ""},
      // {libcamera::formats::SGRBG16, ""},
      // {libcamera::formats::SGBRG16, ""},
      // {libcamera::formats::SBGGR16, ""},
      // {libcamera::formats::R10_CSI2P, ""},
      // {libcamera::formats::SRGGB10_CSI2P, ""},
      // {libcamera::formats::SGRBG10_CSI2P, ""},
      // {libcamera::formats::SGBRG10_CSI2P, ""},
      // {libcamera::formats::SBGGR10_CSI2P, ""},
      // {libcamera::formats::SRGGB12_CSI2P, ""},
      // {libcamera::formats::SGRBG12_CSI2P, ""},
      // {libcamera::formats::SGBRG12_CSI2P, ""},
      // {libcamera::formats::SBGGR12_CSI2P, ""},
      // {libcamera::formats::SRGGB14_CSI2P, ""},
      // {libcamera::formats::SGRBG14_CSI2P, ""},
      // {libcamera::formats::SGBRG14_CSI2P, ""},
      // {libcamera::formats::SBGGR14_CSI2P, ""},
      // {libcamera::formats::SRGGB10_IPU3, ""},
      // {libcamera::formats::SGRBG10_IPU3, ""},
      // {libcamera::formats::SGBRG10_IPU3, ""},
      // {libcamera::formats::SBGGR10_IPU3, ""}
  };
  int table_size = sizeof(convert_table) / sizeof(convert_table[0]);

  for (int i = 0; i < table_size; i++) {
    if (pixel_format.compare(convert_table[i].senscord_pixelformat) == 0) {
      SENSCORD_LOG_INFO_TAGGED(
          "libcamera", "ConvertPixelFormat: '%s' => libcamera::formats::%s",
          pixel_format.c_str(),
          convert_table[i].libcamera_pixelformat.toString().c_str());
      *format = convert_table[i].libcamera_pixelformat;
      return;
    }
  }
  SENSCORD_LOG_INFO_TAGGED("libcamera",
                           "ConvertPixelFormat failed: '%s' => unknown",
                           pixel_format.c_str());
}

void LibcameraAdapter::ConvertPixelFormat(
    std::string &pixel_format, const libcamera::PixelFormat *format) {
  constexpr static struct {
    const libcamera::PixelFormat &libcamera_pixelformat;
    const char *senscord_pixelformat;
  } convert_table[] = {
      {libcamera::formats::R8, senscord::kPixelFormatGREY},
      {libcamera::formats::R10, senscord::kPixelFormatY10},
      {libcamera::formats::R12, senscord::kPixelFormatY12},
      // {libcamera::formats::RGB565, ""},
      // {libcamera::formats::RGB565_BE, ""},
      {libcamera::formats::BGR888, senscord::kPixelFormatRGB24},
      {libcamera::formats::BGR888, senscord::kPixelFormatBGR24},
      {libcamera::formats::XRGB8888, senscord::kPixelFormatXRGB32},
      {libcamera::formats::XBGR8888, senscord::kPixelFormatXRGB444},
      // {libcamera::formats::RGBX8888, ""},
      // {libcamera::formats::BGRX8888, ""},
      {libcamera::formats::ARGB8888, senscord::kPixelFormatARGB32},
      {libcamera::formats::ABGR8888, senscord::kPixelFormatABGR32},
      // {libcamera::formats::RGBA8888, ""},
      // {libcamera::formats::BGRA8888, ""},
      {libcamera::formats::YUYV, senscord::kPixelFormatYUYV},
      // {libcamera::formats::YVYU, ""},
      {libcamera::formats::UYVY, senscord::kPixelFormatUYVY},
      // {libcamera::formats::VYUY, ""},
      // {libcamera::formats::AVUY8888, ""},
      // {libcamera::formats::XVUY8888, ""},
      {libcamera::formats::NV12, senscord::kPixelFormatNV12},
      // {libcamera::formats::NV21, ""},
      {libcamera::formats::NV16, senscord::kPixelFormatNV16},
      // {libcamera::formats::NV61, ""},
      // {libcamera::formats::NV24, ""},
      // {libcamera::formats::NV42, ""},
      {libcamera::formats::YUV420, senscord::kPixelFormatYUV420},
      // {libcamera::formats::YVU420, ""},
      // {libcamera::formats::YUV422, ""},
      // {libcamera::formats::YVU422, ""},
      {libcamera::formats::YUV444, senscord::kPixelFormatYUV444},
      // {libcamera::formats::YVU444, ""},
      {libcamera::formats::MJPEG, senscord::kPixelFormatJPEG},
      {libcamera::formats::SRGGB8, senscord::kPixelFormatSRGGB8},
      {libcamera::formats::SGRBG8, senscord::kPixelFormatSGRBG8},
      {libcamera::formats::SGBRG8, senscord::kPixelFormatSGBRG8},
      {libcamera::formats::SBGGR8, senscord::kPixelFormatSBGGR8},
      {libcamera::formats::SRGGB10, senscord::kPixelFormatSRGGB10},
      {libcamera::formats::SGRBG10, senscord::kPixelFormatSGRBG10},
      {libcamera::formats::SGBRG10, senscord::kPixelFormatSGBRG10},
      {libcamera::formats::SBGGR10, senscord::kPixelFormatSBGGR10},
      {libcamera::formats::SRGGB12, senscord::kPixelFormatSRGGB12},
      {libcamera::formats::SGRBG12, senscord::kPixelFormatSGRBG12},
      {libcamera::formats::SGBRG12, senscord::kPixelFormatSGBRG12},
      {libcamera::formats::SBGGR12, senscord::kPixelFormatSBGGR12},
      // {libcamera::formats::SRGGB14, ""},
      // {libcamera::formats::SGRBG14, ""},
      // {libcamera::formats::SGBRG14, ""},
      // {libcamera::formats::SBGGR14, ""},
      // {libcamera::formats::SRGGB16, ""},
      // {libcamera::formats::SGRBG16, ""},
      // {libcamera::formats::SGBRG16, ""},
      // {libcamera::formats::SBGGR16, ""},
      // {libcamera::formats::R10_CSI2P, ""},
      // {libcamera::formats::SRGGB10_CSI2P, ""},
      // {libcamera::formats::SGRBG10_CSI2P, ""},
      // {libcamera::formats::SGBRG10_CSI2P, ""},
      // {libcamera::formats::SBGGR10_CSI2P, ""},
      // {libcamera::formats::SRGGB12_CSI2P, ""},
      // {libcamera::formats::SGRBG12_CSI2P, ""},
      // {libcamera::formats::SGBRG12_CSI2P, ""},
      // {libcamera::formats::SBGGR12_CSI2P, ""},
      // {libcamera::formats::SRGGB14_CSI2P, ""},
      // {libcamera::formats::SGRBG14_CSI2P, ""},
      // {libcamera::formats::SGBRG14_CSI2P, ""},
      // {libcamera::formats::SBGGR14_CSI2P, ""},
      // {libcamera::formats::SRGGB10_IPU3, ""},
      // {libcamera::formats::SGRBG10_IPU3, ""},
      // {libcamera::formats::SGBRG10_IPU3, ""},
      // {libcamera::formats::SBGGR10_IPU3, ""}
  };
  int table_size = sizeof(convert_table) / sizeof(convert_table[0]);

  for (int i = 0; i < table_size; i++) {
    if (*format == convert_table[i].libcamera_pixelformat) {
      SENSCORD_LOG_INFO_TAGGED(
          "libcamera", "ConvertPixelFormat: '%s' => senscord::formats::%s",
          format->toString().c_str(), convert_table[i].senscord_pixelformat);
      pixel_format = convert_table[i].senscord_pixelformat;
      return;
    }
  }

  SENSCORD_LOG_INFO_TAGGED("libcamera",
                           "ConvertPixelFormat failed: '%s' => unknown",
                           format->toString().c_str());
}

void LibcameraAdapter::UpdateImageSensorFunctionSupportedProperty(void) {
  senscord::ImageSensorFunctionSupportedProperty *prop =
      &image_sensor_function_supported_property_;
  const static struct {
    int control_id;
    bool *property_member;
  } table[] = {
      {libcamera::controls::AE_ENABLE, &prop->auto_exposure_supported},
      {libcamera::controls::AWB_ENABLE, &prop->auto_white_balance_supported},
      {libcamera::controls::BRIGHTNESS, &prop->brightness_supported},
      {-1, &prop->iso_sensitivity_supported},
      {libcamera::controls::EXPOSURE_TIME, &prop->exposure_time_supported},
      {libcamera::controls::AE_EXPOSURE_MODE,
       &prop->exposure_metering_supported},
      {-1, &prop->gamma_value_supported},
      {-1, &prop->gain_value_supported},
      {-1, &prop->hue_supported},
      {libcamera::controls::SATURATION, &prop->saturation_supported},
      {libcamera::controls::SHARPNESS, &prop->sharpness_supported},
      {libcamera::controls::AWB_MODE, &prop->white_balance_supported}};

  if (camera_) {
    for (size_t i = 0; i < (sizeof(table) / sizeof(table[0])); i++) {
      *table[i].property_member =
          (camera_->controls().find(table[i].control_id) !=
           camera_->controls().end());
    }
  }
}

senscord::Status LibcameraAdapter::ConvertValue(
    const libcamera::ControlType type,
    const libcamera::ControlValue &target_value,
    senscord::libcamera_image::AnyValue &value) {
  switch (type) {
    case libcamera::ControlType::ControlTypeBool: {
      std::optional<bool> optional_value = target_value.get<bool>();
      if (optional_value.has_value()) {
        value.Set<bool>(optional_value.value());
      }
      break;
    }
    case libcamera::ControlType::ControlTypeByte:
      return convertValue<uint8_t>(target_value, value);

    case libcamera::ControlType::ControlTypeInteger32:
      return convertValue<int32_t>(target_value, value);

    case libcamera::ControlType::ControlTypeInteger64:
      return convertValue<int64_t>(target_value, value);

    case libcamera::ControlType::ControlTypeFloat:
      return convertValue<float>(target_value, value);

    case libcamera::ControlType::ControlTypeString: {
      std::optional<std::string> optional_value =
          target_value.get<std::string>();
      if (optional_value.has_value()) {
        value.Set<std::string>(optional_value.value());
      }
      break;
    }

    break;
    case libcamera::ControlType::ControlTypeRectangle:
    case libcamera::ControlType::ControlTypeSize:

    default:
    case libcamera::ControlType::ControlTypeNone:
      return SENSCORD_STATUS_FAIL("libcamera",
                                  senscord::Status::kCauseInvalidArgument, "");
      break;
  }

  return senscord::Status::OK();
}

bool LibcameraAdapter::CheckBundleIdItOnly(const std::string ai_model_bundle_id) {
  if ((ai_model_bundle_id == std::string(AiBundleIdItonly)) ||
      (ai_model_bundle_id == std::string(AiBundleIdVgaRgbItonly))) {
    SENSCORD_LOG_INFO(
      "ai_model_bundle_id(%s) is InputTensorOnly pattern.", ai_model_bundle_id.c_str());
    return true;
  }

  return false;
}

std::string LibcameraAdapter::ReplaceCustomJsonPathString(
    const std::string &path,
    const std::string &replacement) {
  std::string result_path = path;
  std::string dir_path;
  size_t pos = result_path.find_last_of("/\\");

  if (pos != std::string::npos) {
    dir_path = result_path.substr(0, pos);
  } else {
    /* The file name was specified directly. */
    dir_path = ".";
  }

  result_path =  dir_path + "/" + replacement;

  std::ifstream ifs(result_path);
  if (!ifs) {
    return "";
  }

  return result_path;
}

std::string LibcameraAdapter::HandleCustomJsonPathString(
    const std::string &post_process_file, const std::string &ai_model_bundle_id) {
  std::string post_process_file_new = post_process_file;

  if (CheckBundleIdItOnly(ai_model_bundle_id)) {
    std::string replacement = CustomVgaItonlyParamJsonFile;
    post_process_file_new = ReplaceCustomJsonPathString(post_process_file, replacement);
  } else {
    std::string custom_json_file = "custom_" + ai_model_bundle_id + ".json";
    post_process_file_new = ReplaceCustomJsonPathString(post_process_file, custom_json_file);
    if (!CheckRpkExist(post_process_file_new)) {
      return "";
    }
  }

  SENSCORD_LOG_INFO("Set post_process_file: %s", post_process_file_new.c_str());

  return post_process_file_new;
}

std::string LibcameraAdapter::ReadPostProcessJsonString(const std::string &post_process_file) {
  FILE *file = fopen(post_process_file.c_str(), "rb");
  if (!file) {
    SENSCORD_LOG_ERROR_TAGGED("libcamera", "Failed to open %s.", post_process_file.c_str());

    /* Return empty strings. */
    return "";
  }

  fseek(file, 0, SEEK_END);
  long size = ftell(file);
  rewind(file);
  std::vector<char> buffer(size + 1, 0);
  fread(buffer.data(), 1, size, file);
  fclose(file);

  return std::string(buffer.data(), size);
}

std::string LibcameraAdapter::GetRpkPath(const std::string &json_str) {
  nlohmann::json j;

  try {
    j = nlohmann::json::parse(json_str);
  } catch (const nlohmann::json::parse_error &e) {
    SENSCORD_LOG_WARNING_TAGGED("libcamera", "Failed to parse json parameter.");
    return "";
  }

  if (!(j.contains("imx500_no_process")) || !(j["imx500_no_process"].is_object())) {
    SENSCORD_LOG_WARNING_TAGGED(
        "libcamera",
        "The key \"imx500_no_process\" does not exist.");
    return "";
  }

  nlohmann::json imx500_no_process_j = j["imx500_no_process"];
  if (!(imx500_no_process_j.contains("network_file")) ||
      !(imx500_no_process_j["network_file"].is_string())) {
    SENSCORD_LOG_WARNING_TAGGED(
        "libcamera",
        "The key \"network_file\" does not exist.");
    return "";
  }

  return imx500_no_process_j["network_file"];
}

bool LibcameraAdapter::CheckRpkExist(const std::string &post_process_file) {
  std::string j_str = ReadPostProcessJsonString(post_process_file);
  if (j_str.empty()) {
    SENSCORD_LOG_WARNING_TAGGED("libcamera", "The json parameter for PostProcess is empty.");
    return false;
  }

  std::string rpk_path = GetRpkPath(j_str);
  FILE *f = fopen(rpk_path.c_str(), "rb");
  if (!f) {
    SENSCORD_LOG_WARNING_TAGGED(
        "libcamera",
        "Failed to open %s, %s.",
        rpk_path.c_str(), strerror(errno));
    return false;
  }

  fclose(f);

  return true;
}

bool LibcameraAdapter::GetDeviceID(std::string &device_id_str) {
  V4L2CtrlManager v4l2_manager;

  if (!v4l2_manager.Open()) {
    return false;
  }

  const uint32_t device_id_ctrl_id = DEVICE_ID_CTRL_ID;
  const int32_t device_id_num = 4;

  uint32_t device_id[device_id_num] = {0};

  if (!v4l2_manager.GetExtControl(
                  device_id_ctrl_id,
                  (void*)device_id,
                  sizeof(device_id),
                  sizeof(uint32_t) * 8)) {
    v4l2_manager.Close();
    SENSCORD_LOG_WARNING("Failed to execute ioctl");
    return false;
  }

  v4l2_manager.Close();

  std::string device_id_str_tmp = "";

  /* convert string */
  for (int i = 0; i < device_id_num; i++) {
    std::ostringstream ss;
    ss << std::setw(8) << std::setfill('0') << std::uppercase << std::hex << device_id[i];
    std::string result = ss.str();

    device_id_str_tmp += result;
  }

  device_id_str = device_id_str_tmp;

  return true;
}

bool LibcameraAdapter::UpdateImageCrop(void) {
  V4L2CtrlManager v4l2_manager;

  if (!v4l2_manager.Open()) {
    return false;
  }

  /* Check if the crop starting point exists within the image. */
  if ((image_crop_.x >= camera_image_width_) || (image_crop_.y >= camera_image_height_)) {
    return false;
  }

  /* Correct to crop the valid area. */
  uint32_t width = image_crop_.w;
  uint32_t height = image_crop_.h;

  if ((image_crop_.x + width) > camera_image_width_) {
    width = camera_image_width_ - image_crop_.x;
  }

  if ((image_crop_.y + height) > camera_image_height_) {
    height = camera_image_height_ - image_crop_.y;
  }

  const uint32_t crop_param[4] = {image_crop_.x, image_crop_.y, width, height};
  const uint32_t inference_window_id = INFERENCE_WINDOW_ID;

  if (!v4l2_manager.SetExtControl(
                inference_window_id,
                (void *)crop_param,
                sizeof(crop_param),
                sizeof(uint32_t) * 8)) {
    v4l2_manager.Close();
    return false;
  }

  v4l2_manager.Close();

  return true;
}

void LibcameraAdapter::UpdateImageRotationProperty(void) {
  uint8_t reg_value = 0;

  switch (rotation_property_.rotation_angle) {
    case SENSOR_ROTATION_ANGLE_0_DEG:
      reg_value = SENSOR_REG_ROTATION_0;
      break;
    case SENSOR_ROTATION_ANGLE_90_DEG:
      reg_value = SENSOR_REG_ROTATION_90;
      break;
    case SENSOR_ROTATION_ANGLE_180_DEG:
      reg_value = SENSOR_REG_ROTATION_180;
      break;
    case SENSOR_ROTATION_ANGLE_270_DEG:
      reg_value = SENSOR_REG_ROTATION_270;
      break;
    default:
      SENSCORD_LOG_WARNING_TAGGED("libcamera", "Invalid rotation angle: %d",
                                  rotation_property_.rotation_angle);
      return;
  }

  senscord::Status status = reg_handle_.WriteRegister(kRegImageRotate, &reg_value);
  if (!status.ok()) {
    SENSCORD_LOG_ERROR_TAGGED(
        "libcamera", "Failed to write image rotate register");
  }
}

}  // namespace libcamera_image
}  // namespace senscord
