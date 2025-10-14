/*
 * SPDX-FileCopyrightText: 2023-2025 Sony Semiconductor Solutions Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "sample_mock.h"

SampleMock& SampleMock::GetInstance(void) {
  static SampleMock instance;
  return instance;
}

extern "C" int SampleSub(void) { return SampleMock::GetInstance().SampleSub(); }
