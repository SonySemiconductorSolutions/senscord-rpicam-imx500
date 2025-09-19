#include "v4l2_ctrl_manager.h"
#include <fcntl.h>
#include <filesystem>
#include <unistd.h>
#include <sys/ioctl.h>
#include <cstring>
#include <linux/videodev2.h>

namespace fs = std::filesystem;

std::mutex V4L2CtrlManager::control_mutex;

V4L2CtrlManager::V4L2CtrlManager() : fd_(-1) {}

V4L2CtrlManager::~V4L2CtrlManager() {
    close(fd_);
}

bool V4L2CtrlManager::Open(void) {
  std::lock_guard<std::mutex> lock_controls(control_mutex);
  if (fd_ != -1) {
    return true;
  }

  int fd = -1;

  for (int i = 0; i < 3; i++) {
    const fs::path sys_dir { "/sys/class/video4linux/v4l-subdev" + std::to_string(i) + "/device" };
    const fs::path module_dir { sys_dir.string() + "/driver/module" };
    const fs::path id_dir { sys_dir.string() + "/of_node" };

    if (fs::exists(module_dir) && fs::is_symlink(module_dir)) {
      fs::path ln = fs::read_symlink(module_dir);

      if (ln.string().find("imx500") != std::string::npos) {
        const std::string dev_node { "/dev/v4l-subdev" + std::to_string(i) };
        fd = open(dev_node.c_str(), O_RDONLY, 0);
        if (fd < 0) {
          continue;
        } else {
          break;
        }
      }
    }
  }

  if (fd == -1) {
    return false;
  }

  fd_ = fd;

  return true;
}

void V4L2CtrlManager::Close() {
  std::lock_guard<std::mutex> lock_controls(control_mutex);
  if (fd_ != -1) {
    close(fd_);
    fd_ = -1;
  }
}

bool V4L2CtrlManager::SetExtControl(
    uint32_t id,
    void *buffer,
    size_t size,
    uint32_t bit) {
  std::lock_guard<std::mutex> lock_controls(control_mutex);

  if (fd_ < 0) {
    return false;
  }

  v4l2_ext_control ctrl[1];

  ctrl[0].id = id;
  ctrl[0].size = size;

  switch (bit) {
    case 8:
      ctrl[0].p_u8 = (uint8_t *)buffer;
      break;
    case 16:
      ctrl[0].p_u16 = (uint16_t *)buffer;
      break;
    case 32:
      ctrl[0].p_u32 = (uint32_t *)buffer;
      break;
  }

  v4l2_ext_controls ctrls;
  ctrls.ctrl_class = V4L2_CTRL_CLASS_USER;
  ctrls.count = 1;
  ctrls.controls = ctrl;

  int ret = ioctl(fd_, VIDIOC_S_EXT_CTRLS, &ctrls);

  if (ret < 0) {
    return false;
  }

  return true;
}

bool V4L2CtrlManager::GetExtControl(
    uint32_t id,
    void *buffer,
    size_t size,
    uint32_t bit) {
  std::lock_guard<std::mutex> lock_controls(control_mutex);

  if (fd_ < 0) {
    return false;
  }

  v4l2_ext_control ctrl[1];

  ctrl[0].id = id;
  ctrl[0].size = size;

  switch (bit) {
    case 8:
      ctrl[0].p_u8 = (uint8_t *)buffer;
      break;
    case 16:
      ctrl[0].p_u16 = (uint16_t *)buffer;
      break;
    case 32:
      ctrl[0].p_u32 = (uint32_t *)buffer;
      break;
  }

  v4l2_ext_controls ctrls;
  ctrls.ctrl_class = V4L2_CTRL_CLASS_USER;
  ctrls.count = 1;
  ctrls.controls = ctrl;

  int ret = ioctl(fd_, VIDIOC_G_EXT_CTRLS, &ctrls);

  if (ret < 0) {
    return false;
  }

  return true;
}
