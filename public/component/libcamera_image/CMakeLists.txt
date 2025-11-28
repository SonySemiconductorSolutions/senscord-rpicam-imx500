#
# SPDX-FileCopyrightText: 2023 Sony Semiconductor Solutions Corporation
#
# SPDX-License-Identifier: Apache-2.0
#

### Project
# set project name.
set(PROJECT_NAME component_libcamera_image)
set(OUTPUT_INCLUDE_DIR libcamera_image)

project(${PROJECT_NAME} CXX)

### Project Version
# set project Major Version
set(${PROJECT_NAME}_VERSION_MAJOR 0)

# set project Minor Version
set(${PROJECT_NAME}_VERSION_MINOR 1)

# set project Patch Version
set(${PROJECT_NAME}_VERSION_PATCH 0)

# set project Version
set(${${PROJECT_NAME}_VERSION}
    ${${PROJECT_NAME}_VERSION_MAJOR}.${${PROJECT_NAME}_VERSION_MINOR})

# target name
set(TARGET_NAME ${PROJECT_NAME})

find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBCAMERA REQUIRED libcamera)

### Sources
# Public header files.
file(GLOB PUBLIC_HEADER_FILES
     ${PROJECT_SOURCE_DIR}/include/senscord/${OUTPUT_INCLUDE_DIR}/*.h)

# Create senscord.lib target.
add_library(${TARGET_NAME} SHARED
            src/libcamera_image_stream_source_factory.cpp
            src/libcamera_image_stream_source.cpp
            src/libcamera_image_sensor_register.cpp
            src/rpicam_app_adapter.cpp
            src/v4l2_ctrl_manager.cpp
            ${PUBLIC_HEADER_FILES})

# Include directory.
target_include_directories(${TARGET_NAME} PRIVATE
  ${PROJECT_SOURCE_DIR}
  ${PROJECT_SOURCE_DIR}/include
  ${PROJECT_SOURCE_DIR}/../inference_property_converter/include
  ${LIBCAMERA_INCLUDE_DIRS})

# Ensure rpicam_app is built first and link against it directly
add_dependencies(${TARGET_NAME} rpicam_app_mod)
target_link_libraries(${TARGET_NAME}
    senscord
    ${LIBCAMERA_LINK_LIBRARIES}
    rpicam_app_mod
)

target_compile_options(${TARGET_NAME} PRIVATE ${SSDK_COMPILE_OPTIONS} -std=c++17)

install(TARGETS ${TARGET_NAME} DESTINATION lib/senscord/component)
install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/config/${TARGET_NAME}.xml
        DESTINATION lib/senscord/component)
