#
# SPDX-FileCopyrightText: 2023 Sony Semiconductor Solutions Corporation
#
# SPDX-License-Identifier: Apache-2.0
#

cmake_minimum_required(VERSION 3.21)

project(senscord_libcamera)

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/output/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/output/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/output/bin)

enable_testing()

# Options
option(SENSCORD_COMPONENT_LIBCAMERA "Enables the build of libcamera_image" ON)

option(BUILD_TESTING "Enables the build of tests for libcamera_image")

message(STATUS "build options...")
message(STATUS "CMAKE_INSTALL_PREFIX : ${CMAKE_INSTALL_PREFIX}")
message(STATUS "SENSCORD_COMPONENT_LIBCAMERA : ${SENSCORD_COMPONENT_LIBCAMERA}")
message(STATUS "BUILD_TESTING : ${BUILD_TESTING}")

# Find senscord
find_package(senscord)
if(NOT senscord_FOUND)
  message(FATAL_ERROR "set CMAKE_INSTALL_PREFIX=<senscord install directory>")
endif()

# Find libcamera
find_package(PkgConfig REQUIRED)
pkg_check_modules(LC REQUIRED libcamera)

set(RPICAM_APPS_SOURCE_DIR ${CMAKE_SOURCE_DIR}/rpicam-apps)
set(RPICAM_APPS_BINARY_DIR ${CMAKE_BINARY_DIR}/rpicam-apps-build)
add_subdirectory(
  ${RPICAM_APPS_SOURCE_DIR}
  ${RPICAM_APPS_BINARY_DIR}
)

include_directories(
  ${CMAKE_SOURCE_DIR}/rpicam-apps
  ${LC_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
link_directories(
  ${LC_LIBRARY_DIRS}
)

add_subdirectory(public)
