/*
 * SPDX-FileCopyrightText: 2023-2025 Sony Semiconductor Solutions Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "malloc_mock.h"

namespace {
std::unique_ptr<NiceMock<MallocMock>> p_mock = nullptr;
}

NiceMock<MallocMock>* MallocMock::GetMock(void) { return p_mock.get(); }

void MallocMock::ResetMock(void) {
  if (p_mock != nullptr) {
    ::testing::Mock::VerifyAndClearExpectations(p_mock.get());
  }
  p_mock.reset(new NiceMock<MallocMock>);
  p_mock->bypass_mock_ = true;
}

void MallocMock::FreeMock(void) {
  if (p_mock != nullptr) {
    ::testing::Mock::VerifyAndClearExpectations(p_mock.get());
    p_mock.reset();
  }
}

void MallocMock::UseMock(void) {
  NiceMock<MallocMock>* p_mock = MallocMock::GetMock();
  if (p_mock != nullptr) {
    p_mock->SetBypass(false);
  }
}

void MallocMock::BypassMock(void) {
  NiceMock<MallocMock>* p_mock = MallocMock::GetMock();
  if (p_mock != nullptr) {
    p_mock->SetBypass(true);
  }
}

bool MallocMock::IsBypassed(void) const { return bypass_mock_; }
void MallocMock::SetBypass(bool bypass) { bypass_mock_ = bypass; }

extern "C" {
void* __real_malloc(size_t size);
void __real_free(void* ptr);

void* __wrap_malloc(size_t size) {
  NiceMock<MallocMock>* mock = MallocMock::GetMock();
  if (mock == nullptr || mock->IsBypassed()) {
    return __real_malloc(size);
  }
  return mock->malloc(size);
}

void __wrap_free(void* ptr) {
  NiceMock<MallocMock>* mock = MallocMock::GetMock();
  if (mock == nullptr || mock->IsBypassed()) {
    __real_free(ptr);
  } else {
    mock->free(ptr);
  }
}
}
