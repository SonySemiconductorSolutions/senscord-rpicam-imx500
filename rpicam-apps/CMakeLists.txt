cmake_minimum_required(VERSION 3.13)

project(rpicam-apps VERSION 1.7.0 LANGUAGES C CXX)

# -----------------------------------------------------------------------------
# Option definitions (Reproducing Meson's -Denable_xxx)
# -----------------------------------------------------------------------------
option(ENABLE_LIBAV    "Enable libav encoder support" OFF)
option(ENABLE_DRM      "Enable DRM preview support" OFF)
option(ENABLE_EGL      "Enable EGL preview support" OFF)
option(ENABLE_QT       "Enable Qt preview support" OFF)
option(ENABLE_OPENCV   "Enable OpenCV postprocessing" OFF)
option(ENABLE_TFLITE   "Enable TFLite postprocessing" OFF)
option(ENABLE_HAILO    "Enable Hailo postprocessing" OFF)
option(ENABLE_IMX500   "Enable IMX500 postprocessing" ON)

if(JPEG_ENABLED)
  message(STATUS "Test mode enabled: JPEG support activated")
endif()
if(EXIF_ENABLED)
  message(STATUS "Test mode enabled: EXIF support activated")
endif()

option(BUILD_TESTS     "Enable building tests" OFF)
set(NEON_FLAGS "auto" CACHE STRING "ARM NEON flags (auto|arm64|armv8-neon)")
set_property(CACHE NEON_FLAGS PROPERTY STRINGS auto arm64 armv8-neon)

# -----------------------------------------------------------------------------
# Language standards
# -----------------------------------------------------------------------------
set(CMAKE_C_STANDARD        11)
set(CMAKE_CXX_STANDARD      17)
set(CMAKE_C_STANDARD_REQUIRED   ON)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# -----------------------------------------------------------------------------
# Warning and error settings
# -----------------------------------------------------------------------------
add_compile_options(
  -Wall
  -Wextra
  -Wpedantic
  -Werror
  -Wno-unused-parameter
  -faligned-new
)
add_compile_definitions(_FILE_OFFSET_BITS=64)
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
  add_compile_options(-Wno-psabi)
endif()

# -----------------------------------------------------------------------------
# Optimization flags for CPU/NEON (Reproducing Meson logic)
# -----------------------------------------------------------------------------
if(NEON_FLAGS STREQUAL "auto")
  if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
    add_compile_options(-ftree-vectorize)
  endif()
elseif(NEON_FLAGS STREQUAL "arm64")
  add_compile_options(-ftree-vectorize)
elseif(NEON_FLAGS STREQUAL "armv8-neon")
  add_compile_options(-mfpu=neon-fp-armv8 -ftree-vectorize)
else()
  add_compile_options(-mfpu=${NEON_FLAGS})
endif()

# -----------------------------------------------------------------------------
# Detecting dependent libraries
# -----------------------------------------------------------------------------
find_package(PkgConfig REQUIRED)
pkg_check_modules(LC REQUIRED libcamera)

# Find JPEG library (optional)
if(JPEG_ENABLED)
  find_package(JPEG REQUIRED)
endif()

# Find EXIF library (optional)
if(EXIF_ENABLED)
  pkg_check_modules(EXIF REQUIRED libexif)
endif()

# Detect Boost program_options (used in rpicam_app)
find_package(Boost REQUIRED COMPONENTS program_options)
include_directories(${Boost_INCLUDE_DIRS})
message(STATUS "Found Boost.ProgramOptions: ${Boost_PROGRAM_OPTIONS_LIBRARY}")
if(JPEG_ENABLED)
  message(STATUS "Found JPEG: ${JPEG_LIBRARIES}")
endif()
if(EXIF_ENABLED)
  message(STATUS "Found EXIF: ${EXIF_LIBRARIES}")
endif()

# -----------------------------------------------------------------------------
# Include / link settings
# -----------------------------------------------------------------------------
include_directories(
  ${LC_INCLUDE_DIRS}                        # libcamera headers
  ${PROJECT_SOURCE_DIR}                     # rpicam-apps root
  ${PROJECT_SOURCE_DIR}/core                # core headers
  ${PROJECT_SOURCE_DIR}/image               # image headers
  ${PROJECT_SOURCE_DIR}/output              # output headers
  ${PROJECT_SOURCE_DIR}/preview             # preview headers
  ${PROJECT_SOURCE_DIR}/post_processing_stages # post_processing_stages headers
)
if(EXIF_ENABLED)
  include_directories(${EXIF_INCLUDE_DIRS}) # libexif headers
endif()
link_directories(
  ${LC_LIBRARY_DIRS}
)

# -----------------------------------------------------------------------------
# Collecting source files
# -----------------------------------------------------------------------------
file(GLOB RPICAM_SRCS
  core/*.cpp
  image/*.cpp
  output/*.cpp
  preview/*.cpp
  post_processing_stages/*.cpp
  post_processing_stages/imx500/*.cpp
)

# -----------------------------------------------------------------------------
# rpicam_app library
# -----------------------------------------------------------------------------
add_library(rpicam_app_mod SHARED ${RPICAM_SRCS})
# Set build output path to match Meson's multi-architecture path
set_target_properties(rpicam_app_mod PROPERTIES
  PREFIX ""                                   # Remove "lib" prefix
  LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/output/lib
)
target_include_directories(rpicam_app_mod PUBLIC
  ${PROJECT_SOURCE_DIR}
  ${PROJECT_SOURCE_DIR}/post_processing_stages
)
target_link_libraries(rpicam_app_mod PRIVATE
  ${LC_LIBRARIES}
  dl
  ${Boost_PROGRAM_OPTIONS_LIBRARY}
)
if(JPEG_ENABLED)
  target_link_libraries(rpicam_app_mod PRIVATE ${JPEG_LIBRARIES})
endif()
if(EXIF_ENABLED)
  target_link_libraries(rpicam_app_mod PRIVATE ${EXIF_LIBRARIES})
endif()

# Multi-architecture installation
install(TARGETS rpicam_app_mod LIBRARY DESTINATION lib)

# -----------------------------------------------------------------------------
# Tool binaries
# -----------------------------------------------------------------------------
file(GLOB RPICAM_TOOLS tools/*.cpp)
set(ALL_TOOLS "")
foreach(src ${RPICAM_TOOLS})
  get_filename_component(tool ${src} NAME_WE)
  add_executable(${tool} ${src} ${RPICAM_SRCS})
  target_link_libraries(${tool} PRIVATE
    ${LC_LIBRARIES}
    dl
  )
  list(APPEND ALL_TOOLS ${tool})
endforeach()

# -----------------------------------------------------------------------------
# Test settings
# -----------------------------------------------------------------------------
if(BUILD_TESTS)
  enable_testing()
  add_executable(test_hal tests/test_hal.cpp ${RPICAM_SRCS})
  target_link_libraries(test_hal PRIVATE ${LC_LIBRARIES})
  
  if(JPEG_ENABLED)
    target_link_libraries(test_hal PRIVATE ${JPEG_LIBRARIES})
    target_compile_definitions(test_hal PRIVATE HAVE_JPEG)
  endif()

  if(EXIF_ENABLED)
    target_link_libraries(test_hal PRIVATE ${EXIF_LIBRARIES})
    target_compile_definitions(test_hal PRIVATE HAVE_EXIF)
  endif()
  
  add_test(NAME hal_test COMMAND test_hal)
endif()

# -----------------------------------------------------------------------------
# Installation settings
# -----------------------------------------------------------------------------
install(TARGETS ${ALL_TOOLS} RUNTIME DESTINATION bin)

# -----------------------------------------------------------------------------
# dist script
# -----------------------------------------------------------------------------
add_custom_target(dist
  COMMAND ${CMAKE_COMMAND} -E chdir ${PROJECT_SOURCE_DIR} utils/gen-dist.sh
  COMMENT "Run utils/gen-dist.sh to generate distribution archive"
)
