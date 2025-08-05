#
# SPDX-FileCopyrightText: 2023 Sony Semiconductor Solutions Corporation
#
# SPDX-License-Identifier: Apache-2.0
#

# Setting the folder name displayed in the IDE
set(FOLDER_NAME "component")

if(SENSCORD_COMPONENT_LIBCAMERA)
  add_subdirectory(libcamera_image)
  add_subdirectory(inference_property_converter)
endif()
