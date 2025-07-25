#!/bin/sh

#
# SPDX-FileCopyrightText: 2022-2023 Sony Semiconductor Solutions Corporation
#
# SPDX-License-Identifier: Apache-2.0
#

SENSCORD_INSTALL_PATH=$(cd $(dirname $0)/../.. && pwd)

echo SENSCORD_INSTALL_PATH=${SENSCORD_INSTALL_PATH}

export PATH=${SENSCORD_INSTALL_PATH}/bin:${PATH}

FILE_PATH=${SENSCORD_INSTALL_PATH}/share/senscord/config
FILE_PATH=${FILE_PATH}:${SENSCORD_INSTALL_PATH}/lib
FILE_PATH=${FILE_PATH}:${SENSCORD_INSTALL_PATH}/lib/senscord/component
FILE_PATH=${FILE_PATH}:${SENSCORD_INSTALL_PATH}/lib/senscord/allocator
FILE_PATH=${FILE_PATH}:${SENSCORD_INSTALL_PATH}/lib/senscord/connection
FILE_PATH=${FILE_PATH}:${SENSCORD_INSTALL_PATH}/lib/senscord/converter
FILE_PATH=${FILE_PATH}:${SENSCORD_INSTALL_PATH}/lib/senscord/recorder
FILE_PATH=${FILE_PATH}:${SENSCORD_INSTALL_PATH}/lib/senscord/extension
export SENSCORD_FILE_PATH=${FILE_PATH}
export LD_LIBRARY_PATH=${SENSCORD_INSTALL_PATH}/lib:${SENSCORD_INSTALL_PATH}/lib/senscord/utility:${SENSCORD_INSTALL_PATH}/lib/aarch64-linux-gnu:/usr/lib/aarch64-linux-gnu:${LD_LIBRARY_PATH}
export LD_PRELOAD=${SENSCORD_INSTALL_PATH}/lib/rpicam_app_mod.so
exec "$@"
