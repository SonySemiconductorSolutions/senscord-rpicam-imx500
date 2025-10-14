/*
 * SPDX-FileCopyrightText: 2023-2025 Sony Semiconductor Solutions Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef SENSCORD_RPICAM_IMX500_TEST_MOCK_MALLOC_MOCK_H_
#define SENSCORD_RPICAM_IMX500_TEST_MOCK_MALLOC_MOCK_H_

#include <gmock/gmock.h>

extern "C" {
#include <stddef.h>
}

using ::testing::NiceMock;

class MallocMock {
 public:
  static NiceMock<MallocMock> *GetMock(void);
  static void ResetMock(void);
  static void FreeMock(void);
  static void UseMock(void);
  static void BypassMock(void);

  MOCK_METHOD(void*, malloc, (size_t size), (const));
  MOCK_METHOD(void, free, (void* ptr), (const));

  bool IsBypassed(void) const;
  void SetBypass(bool bypass);

  // Method to reset mock expectations for instance
  void Reset() {
    testing::Mock::VerifyAndClearExpectations(this);
  }

 protected:
  MallocMock()                              = default;
  MallocMock(const MallocMock &)            = delete;
  MallocMock &operator=(const MallocMock &) = delete;
  virtual ~MallocMock() {};

  bool bypass_mock_ = true;
};

#endif // SENSCORD_RPICAM_IMX500_TEST_MOCK_MALLOC_MOCK_H_
