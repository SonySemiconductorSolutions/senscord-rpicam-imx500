/*
 * SPDX-FileCopyrightText: 2023-2025 Sony Semiconductor Solutions Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef SENSCORD_RPICAM_IMX500_TEST_SAMPLE_MOCK_H_
#define SENSCORD_RPICAM_IMX500_TEST_SAMPLE_MOCK_H_

#include <gmock/gmock.h>

class SampleMock {
 public:
  static SampleMock& GetInstance();

  MOCK_METHOD(int, SampleSub, (), (const));

  // Method to reset mock expectations
  void Reset() {
    testing::Mock::VerifyAndClearExpectations(this);
  }

 private:
  SampleMock()                             = default;
  SampleMock(const SampleMock&)            = delete;
  SampleMock& operator=(const SampleMock&) = delete;
};

#endif /* SENSCORD_RPICAM_IMX500_TEST_SAMPLE_MOCK_H_ */
