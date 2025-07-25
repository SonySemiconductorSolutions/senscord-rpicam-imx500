#
# SPDX-FileCopyrightText: 2024 Sony Semiconductor Solutions Corporation
#
# SPDX-License-Identifier: Apache-2.0
#

cmake_minimum_required(VERSION 3.17.0)
project(senscord_inference_property_converter LANGUAGES CXX)

set(_target ${PROJECT_NAME})

add_library(${_target} SHARED
  src/inference_property_converter.cpp
)

target_link_libraries(${_target} senscord)

target_include_directories(${_target} PRIVATE
  ${PROJECT_SOURCE_DIR}
  ${PROJECT_SOURCE_DIR}/include
  ${CMAKE_INSTALL_PREFIX}/include
  ${PROJECT_CURRENT_SOURCE_DIR}include)


target_compile_options(${_target}
  PRIVATE
    -fPIC -Wall -Wextra -Wconversion -Wpedantic -Wshadow -Wno-unused-parameter 
)


set_target_properties(${_target} PROPERTIES
  ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib
  CXX_CPPCHECK "${WITH_CPPCHECK}"
)

install(TARGETS ${_target}
    LIBRARY DESTINATION lib/senscord/converter
)
