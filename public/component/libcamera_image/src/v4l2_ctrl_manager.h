/*
 * SPDX-FileCopyrightText: 2023 Sony Semiconductor Solutions Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef V4L2_CTRL_MANAGER_H_
#define V4L2_CTRL_MANAGER_H_

#include <mutex>
#include <string>
#include <vector>

#define INFERENCE_WINDOW_ID (0x00982900)
#define DEVICE_ID_CTRL_ID (0x00982902)

class V4L2CtrlManager {
public:
    V4L2CtrlManager();
    ~V4L2CtrlManager();

    bool Open();
    void Close();
    bool SetExtControl(
        uint32_t id,
        void *buffer,
        size_t size,
        uint32_t bit);
    bool GetExtControl(
        uint32_t id,
        void *buffer,
        size_t size,
        uint32_t bit);

private:
    int fd_;
    static std::mutex control_mutex;
};

#endif // V4L2_CTRL_MANAGER_H_
