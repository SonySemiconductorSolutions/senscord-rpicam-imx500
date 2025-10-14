cmake_minimum_required(VERSION 3.16)
project(SampleUnitTest)

# Set C standard
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug)
endif()

# Enable coverage flags
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} --coverage -O0 -g")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --coverage -O0 -g")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} --coverage")

# Warning flags
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Werror -Wextra -Wshadow -Wundef -Wformat -Wformat-y2k -Wformat-security -Wformat-signedness -Wformat-overflow=2 -Wstack-protector")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Werror -Wextra -Wshadow -Wundef -Wformat -Wformat-y2k -Wformat-security -Wformat-signedness -Wformat-overflow=2 -Wstack-protector")

# Find required packages
find_package(PkgConfig REQUIRED)
find_package(Threads REQUIRED)

# Try to find GTest through various methods
find_package(GTest QUIET)
if(NOT GTest_FOUND)
    # Try pkg-config
    pkg_check_modules(GTEST gtest>=1.10.0)
    pkg_check_modules(GMOCK gmock>=1.10.0)
    
    if(NOT GTEST_FOUND OR NOT GMOCK_FOUND)
        # Install GTest if not found
        message(STATUS "GTest/GMock not found. Please install using apt-get install libgtest-dev libgmock-dev")
        message(FATAL_ERROR "GTest/GMock is required for unit tests")
    endif()
    
    # Create imported targets for compatibility
    add_library(GTest::gtest INTERFACE IMPORTED)
    target_link_libraries(GTest::gtest INTERFACE ${GTEST_LIBRARIES})
    target_include_directories(GTest::gtest INTERFACE ${GTEST_INCLUDE_DIRS})
    
    add_library(GTest::gmock INTERFACE IMPORTED)
    target_link_libraries(GTest::gmock INTERFACE ${GMOCK_LIBRARIES})
    target_include_directories(GTest::gmock INTERFACE ${GMOCK_INCLUDE_DIRS})
    
    add_library(GTest::gtest_main INTERFACE IMPORTED)
    target_link_libraries(GTest::gtest_main INTERFACE GTest::gtest)
    
    add_library(GTest::gmock_main INTERFACE IMPORTED)
    target_link_libraries(GTest::gmock_main INTERFACE GTest::gmock GTest::gtest)
endif()

# Enable testing
enable_testing()

# Add mock subdirectories
add_subdirectory(../mock/malloc ${CMAKE_CURRENT_BINARY_DIR}/mock/malloc)

# Source files for the code under test
set(SRC_FILES
    sample_src.c
)

# Mock files
set(MOCK_FILES
    sample_mock.cc
)

# Test files
set(TEST_FILES
    sample_gtest.cc
)

# Create executable for unit test
add_executable(sample_unit_test 
    ${SRC_FILES}
    ${MOCK_FILES}
    ${TEST_FILES}
)

# Include directories
target_include_directories(sample_unit_test PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# Define UNIT_TEST preprocessor macro
target_compile_definitions(sample_unit_test PRIVATE UNIT_TEST=1)

# Link libraries
target_link_libraries(sample_unit_test
    malloc_mock
    GTest::gtest
    GTest::gmock
    Threads::Threads
    gcov
)

# Note: linker wrap options are now provided by malloc_mock library

# Add test to CTest
add_test(NAME SampleUnitTest COMMAND sample_unit_test)

# Set test properties
set_tests_properties(SampleUnitTest PROPERTIES
    TIMEOUT 30
)
