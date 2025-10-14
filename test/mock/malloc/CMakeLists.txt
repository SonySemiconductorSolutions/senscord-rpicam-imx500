# SPDX-FileCopyrightText: 2023-2025 Sony Semiconductor Solutions Corporation
#
# SPDX-License-Identifier: Apache-2.0

# malloc mock library
add_library(malloc_mock STATIC
    malloc_mock.cc
)

# Include directories
target_include_directories(malloc_mock PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# Find required packages
find_package(PkgConfig REQUIRED)

# Try to find GTest through various methods
find_package(GTest QUIET)
if(NOT GTest_FOUND)
    # Try pkg-config
    pkg_check_modules(GTEST gtest>=1.10.0)
    pkg_check_modules(GMOCK gmock>=1.10.0)
    
    if(NOT GTEST_FOUND OR NOT GMOCK_FOUND)
        message(FATAL_ERROR "GTest/GMock is required for malloc_mock")
    endif()
    
    # Create imported targets for compatibility
    add_library(GTest::gtest INTERFACE IMPORTED)
    target_link_libraries(GTest::gtest INTERFACE ${GTEST_LIBRARIES})
    target_include_directories(GTest::gtest INTERFACE ${GTEST_INCLUDE_DIRS})
    
    add_library(GTest::gmock INTERFACE IMPORTED)
    target_link_libraries(GTest::gmock INTERFACE ${GMOCK_LIBRARIES})
    target_include_directories(GTest::gmock INTERFACE ${GMOCK_INCLUDE_DIRS})
endif()

# Link libraries
target_link_libraries(malloc_mock PUBLIC
    GTest::gmock
)

# Provide linker options as interface property for users of this library
target_link_options(malloc_mock INTERFACE
    -Wl,--wrap=malloc
    -Wl,--wrap=free
)