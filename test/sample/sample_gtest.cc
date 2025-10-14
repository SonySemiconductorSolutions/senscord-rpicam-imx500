/*
 * SPDX-FileCopyrightText: 2023-2025 Sony Semiconductor Solutions Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "sample_mock.h"
#include "../mock/malloc/malloc_mock.h"

extern "C" {
#include "sample_src.h"
}

namespace {

// Test environment class to handle mock cleanup
class MockTestEnvironment : public ::testing::Environment {
 public:
  void SetUp() override {
    // Initialize MallocMock if needed
    if (MallocMock::GetMock() == nullptr) {
      MallocMock::ResetMock();
    }
  }

  void TearDown() override {
    // Clean up all mocks at the end of all tests
    MallocMock::FreeMock();
  }
};

class SampleTest : public ::testing::Test {
 protected:
  void SetUp() override {
    // This method is called before each test.
    // Ensure MallocMock is initialized if needed
    if (MallocMock::GetMock() == nullptr) {
      MallocMock::ResetMock();
    }
    MallocMock::BypassMock(); // Default to bypass
  }

  void TearDown() override {
    // This method is called after each test.
    // Just ensure bypass mode is set, no explicit cleanup
    MallocMock::BypassMock(); // Ensure bypass after test
  }

  SampleMock& mock = SampleMock::GetInstance();
};

TEST_F(SampleTest, BasicAssertions) { EXPECT_EQ(1, 1); }

TEST_F(SampleTest, MockedFunction) {
  EXPECT_CALL(mock, SampleSub()).WillOnce(testing::Return(12));

  EXPECT_EQ(SampleFunc(), 24);
}

TEST_F(SampleTest, MallocSuccessTest) {
  const size_t count = 5;
  int fake_array[5] = {0, 1, 2, 3, 4};
  
  // Enable mock and set expectation
  MallocMock::UseMock();
  EXPECT_CALL(*MallocMock::GetMock(), malloc(count * sizeof(int)))
      .WillOnce(testing::Return((void*)fake_array));
  
  int* result = SampleMallocFunc(count);
  
  EXPECT_NE(result, nullptr);
  EXPECT_EQ(result, fake_array);
}

TEST_F(SampleTest, MallocFailureTest) {
  const size_t count = 5;
  
  // Enable mock and set expectation for failure
  MallocMock::UseMock();
  EXPECT_CALL(*MallocMock::GetMock(), malloc(count * sizeof(int)))
      .WillOnce(testing::Return(nullptr));
  
  int* result = SampleMallocFunc(count);
  
  EXPECT_EQ(result, nullptr);
}

TEST_F(SampleTest, FreeTest) {
  int fake_array[5] = {0, 1, 2, 3, 4};
  
  // Enable mock and set expectation for free
  MallocMock::UseMock();
  EXPECT_CALL(*MallocMock::GetMock(), free((void*)fake_array))
      .Times(1);
  
  SampleFreeFunc(fake_array);
}

TEST_F(SampleTest, FreeNullTest) {
  // free with nullptr should not call the mock
  MallocMock::UseMock();
  EXPECT_CALL(*MallocMock::GetMock(), free(testing::_))
      .Times(0);
  
  SampleFreeFunc(nullptr);
}

TEST_F(SampleTest, BypassModeTest) {
  const size_t count = 2;
  
  // In bypass mode, should use real malloc/free
  MallocMock::BypassMock();
  
  int* result = SampleMallocFunc(count);
  EXPECT_NE(result, nullptr);
  
  // Values should be set correctly by the real function
  if (result != nullptr) {
    EXPECT_EQ(result[0], 0);
    EXPECT_EQ(result[1], 1);
    SampleFreeFunc(result);
  }
}

}  // namespace

// Register the test environment
int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  ::testing::AddGlobalTestEnvironment(new MockTestEnvironment);
  return RUN_ALL_TESTS();
}