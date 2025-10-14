# Unit Test Documentation for senscord-rpicam-imx500

This document describes the unit testing framework and procedures for the senscord-rpicam-imx500 component.

## Overview

The unit testing framework is designed to provide comprehensive testing coverage for all components in the senscord-rpicam-imx500 project. It uses Google Test (GTest) and Google Mock (GMock) as the testing framework, along with CMake for build management and lcov for coverage reporting.

## GitHub Actions Workflow

### run_unit_test.yml

The GitHub Actions workflow (`run_unit_test.yml`) provides automated unit testing for pull requests and manual triggers.

#### Features:
- **Automatic PR Testing**: Runs tests automatically on pull requests
- **Smart Execution**: Skips tests if only documentation or patch files are changed
- **Manual Trigger**: Can be triggered manually with `workflow_dispatch`
- **Reusable Workflow**: Can be called from other workflows using `workflow_call`
- **Coverage Reports**: Generates and uploads HTML coverage reports
- **Artifact Upload**: Saves test logs and coverage reports as artifacts

#### Workflow Steps:
1. **Check Necessity**: Determines if tests need to run based on changed files
2. **Environment Setup**: Installs required dependencies using `env_setup.sh`
3. **Test Execution**: Runs unit tests using `run_unit_test.sh`
4. **Coverage Collection**: Generates coverage reports with lcov
5. **Artifact Upload**: Uploads logs and coverage reports for review

## Shell Scripts

### env_setup.sh

Environment setup script that prepares the system for unit testing.

#### Purpose:
- Install essential build tools (cmake, gcc, g++, etc.)
- Install testing frameworks (Google Test, Google Mock)
- Install coverage tools (lcov, gcovr)
- Configure system environment for testing

#### Usage:
```bash
sudo bash env_setup.sh
```

#### What it installs:
- **Build Tools**: build-essential, cmake, make, gcc, g++, pkg-config
- **Testing Libraries**: libgtest-dev, libgmock-dev, googletest
- **Coverage Tools**: lcov, gcovr
- **Threading Support**: libpthread-stubs0-dev
- **Development Tools**: git, curl, wget, unzip, zip

#### Environment Verification:
The script includes verification steps to ensure all tools are properly installed and accessible.

### exec_unit_test.sh

Individual component testing script for building, testing, and collecting coverage data.

#### Usage:
```bash
./exec_unit_test.sh [build|test] <directory_path>
./exec_unit_test.sh collect [directory_path]
```

#### Commands:
- **build**: Build the unit tests for the specified directory
- **test**: Run the unit tests for the specified directory  
- **collect**: Collect coverage data for the specified directory or all components and generate HTML report

#### Arguments:
- **directory_path**: Test directory path (e.g., sample, public/component/libcamera_image, etc.)
- **For collect command**: directory_path is optional (omit for unified coverage)

#### Examples:
```bash
./exec_unit_test.sh build sample               # Build sample tests
./exec_unit_test.sh test sample                # Run sample tests
./exec_unit_test.sh collect sample             # Collect coverage for sample
./exec_unit_test.sh collect                    # Collect unified coverage for all components
```

#### Features:
- Validates target directory and CMakeLists.txt existence
- Creates build directory automatically
- Uses CMake with Debug configuration
- Runs tests with CTest
- Generates coverage reports with lcov
- Filters out system headers and test files from coverage

### run_unit_test.sh

Master script for running all unit test components and generating unified coverage reports.

#### Usage:
```bash
./run_unit_test.sh
```

#### What it does:
1. **Build All Components**: Builds all configured unit test components
2. **Run All Tests**: Executes tests for all components
3. **Collect Unified Coverage**: Gathers unified coverage data from all components and generates HTML report

#### Component Configuration:
Components are defined in the `COMPONENTS` array within the script:
```bash
COMPONENTS=(
    "sample"
    # Add other components here as they are created
    # "public/component/libcamera_image"
    # "public/component/inference_property_converter"
)
```

#### Output:
- Individual component coverage reports in `<component>/coverage/`
- Unified coverage report in `test/coverage/html/`
- Combined coverage info file for CI integration

## Test Sample Files

The `test/sample/` directory contains example implementations demonstrating the testing framework usage.

### File Structure:

#### Source Files:
- **sample_src.c**: Implementation of functions to be tested
  - `SampleFunc()`: Basic function that depends on external module
  - `SampleMallocFunc()`: Function demonstrating memory allocation testing
  - `SampleFreeFunc()`: Function demonstrating memory deallocation testing

- **sample_src.h**: Header file declaring the test functions

- **sample_sub.h**: Header for external dependency (mocked in tests)

#### Mock Files:
- **sample_mock.cc**: Implementation of mock for `SampleSub` function
  - Uses singleton pattern for mock instance management
  - Provides `Reset()` method for clearing expectations

- **sample_mock.h**: Header declaring the mock interface
  - Extends GMock framework
  - Defines `MOCK_METHOD` for `SampleSub` function

#### Test Files:
- **sample_gtest.cc**: Google Test implementation
  - Demonstrates basic mocking with GMock
  - Shows memory allocation/deallocation testing with `MallocMock`
  - Includes test environment setup for mock management
  - Contains various test scenarios (success, failure, bypass modes)

#### Build Configuration:
- **CMakeLists.txt**: CMake configuration for the sample tests
  - Sets up C/C++ standards and compiler flags
  - Configures coverage flags (`--coverage -O0 -g`)
  - Links required libraries (GTest, GMock, malloc_mock)
  - Defines `UNIT_TEST` preprocessor macro

### Mock Libraries:

#### MallocMock (`test/mock/malloc/`):
- **malloc_mock.h/cc**: Mock implementation for malloc/free functions
- **CMakeLists.txt**: Build configuration with linker wrap options
- **Features**:
  - Bypass mode for using real malloc/free
  - Mock mode for controlled testing
  - Thread-safe singleton pattern
  - Linker wrapping (`-Wl,--wrap=malloc -Wl,--wrap=free`)

## Creating Tests for Components

### Adding a New Component Test

To add unit tests for a new component (e.g., `libcamera_image`):

#### 1. Create Test Directory Structure:
```
test/
├── public/
│   └── component/
│       └── libcamera_image/
│           ├── core/
│           │   ├── CMakeLists.txt
│           │   └── libcamera_image_test.cc
│           └── mock/
│               ├── CMakeLists.txt
│               ├── libcamera_image_mock.h
│               └── libcamera_image_mock.cc
```

#### 2. Configure CMakeLists.txt:

**Core Test CMakeLists.txt** (`test/public/component/libcamera_image/core/CMakeLists.txt`):
```cmake
cmake_minimum_required(VERSION 3.16)
project(LibcameraImageUnitTest)

# Set standards and flags
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} --coverage -O0 -g")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --coverage -O0 -g")

# Find packages
find_package(GTest REQUIRED)

# Add mock subdirectories
add_subdirectory(../mock ${CMAKE_CURRENT_BINARY_DIR}/mock)

# Source files from actual component
set(SRC_FILES
    ../../../../../public/component/libcamera_image/src/libcamera_image_stream_source.cpp
    # Add other source files as needed
)

# Create test executable
add_executable(libcamera_image_unit_test 
    ${SRC_FILES}
    libcamera_image_test.cc
)

# Include directories
target_include_directories(libcamera_image_unit_test PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ../../../../../public/component/libcamera_image/include
    ../mock
    # Add other include paths
)

# Define preprocessor macros
target_compile_definitions(libcamera_image_unit_test PRIVATE UNIT_TEST=1)

# Link libraries
target_link_libraries(libcamera_image_unit_test
    libcamera_image_mock
    GTest::gtest
    GTest::gmock
    Threads::Threads
    gcov
)

# Add test to CTest
add_test(NAME LibcameraImageUnitTest COMMAND libcamera_image_unit_test)
```

**Mock CMakeLists.txt** (`test/public/component/libcamera_image/mock/CMakeLists.txt`):
```cmake
# libcamera_image mock library
add_library(libcamera_image_mock STATIC
    libcamera_image_mock.cc
)

# Include directories
target_include_directories(libcamera_image_mock PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ../../../../../public/component/libcamera_image/include
)

# Find required packages
find_package(GTest REQUIRED)

# Link libraries
target_link_libraries(libcamera_image_mock PUBLIC
    GTest::gmock
)
```

#### 3. Update Component List:
Add the new component to the `COMPONENTS` array in `run_unit_test.sh`:
```bash
COMPONENTS=(
    "sample"
    "public/component/libcamera_image"  # Add your new component here with full path
    # "public/component/inference_property_converter"
)
```

#### 4. Create Mock Interface:
```cpp
// libcamera_image_mock.h
#ifndef SENSCORD_RPICAM_IMX500_TEST_LIBCAMERA_IMAGE_MOCK_H_
#define SENSCORD_RPICAM_IMX500_TEST_LIBCAMERA_IMAGE_MOCK_H_

#include <gmock/gmock.h>

class LibcameraImageMock {
 public:
  static LibcameraImageMock& GetInstance();
  
  // Mock methods for dependencies
  MOCK_METHOD(int, dependency_function, (int param), (const));
  
  void Reset() {
    testing::Mock::VerifyAndClearExpectations(this);
  }

 private:
  LibcameraImageMock() = default;
  LibcameraImageMock(const LibcameraImageMock&) = delete;
  LibcameraImageMock& operator=(const LibcameraImageMock&) = delete;
};

#endif
```

#### 5. Implement Test Cases:
```cpp
// libcamera_image_test.cc
#include <gtest/gtest.h>
#include "libcamera_image_mock.h"

extern "C" {
#include "senscord/libcamera_image/libcamera_image_types.h"
}

class LibcameraImageTest : public ::testing::Test {
 protected:
  void SetUp() override {
    // Setup before each test
  }
  
  void TearDown() override {
    // Cleanup after each test
    mock.Reset();
  }
  
  LibcameraImageMock& mock = LibcameraImageMock::GetInstance();
};

TEST_F(LibcameraImageTest, BasicFunctionality) {
  // Set up mock expectations
  EXPECT_CALL(mock, dependency_function(testing::_))
      .WillOnce(testing::Return(42));
  
  // Execute test
  int result = libcamera_image_function();
  
  // Verify results
  EXPECT_EQ(result, expected_value);
}
```

### Best Practices:

1. **Isolate Dependencies**: Mock all external dependencies
2. **Use Descriptive Test Names**: Make test purpose clear
3. **Test Edge Cases**: Include boundary conditions and error cases
4. **Maintain Mock State**: Reset mocks between tests
5. **Coverage Goals**: Aim for high code coverage while ensuring meaningful tests
6. **Component Separation**: Keep tests focused on specific component functionality

#### Running Tests:

#### Individual Component:
```bash
cd test/scripts
./exec_unit_test.sh build public/component/libcamera_image
./exec_unit_test.sh test public/component/libcamera_image
./exec_unit_test.sh collect public/component/libcamera_image
```

#### All Components:
```bash
cd test/scripts
./run_unit_test.sh
```

The unified coverage report will be available at `test/coverage/html/index.html`.
